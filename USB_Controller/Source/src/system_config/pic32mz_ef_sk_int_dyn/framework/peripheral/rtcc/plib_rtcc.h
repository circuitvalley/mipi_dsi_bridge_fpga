/**************************************************************************
  Real-Time Clock and Calendar (RTCC) Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.
	
  File Name:
    plib_rtcc.h
  
  Summary:
    RTCC Peripheral Library interface header for RTCC common definitions.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the RTCC
    Peripheral Library for all families of Microchip microcontrollers. The 
    definitions in this file are common to the RTCC peripheral.
**************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************
Copyright 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _PLIB_RTCC_H
#define _PLIB_RTCC_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued at end of file)
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.  However,
    please see the end of the file for additional implementation header files
    that are also included
*/

#include "processor/rtcc_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants & Data Types
// *****************************************************************************
// *****************************************************************************
//DOM-IGNORE-BEGIN
// ****************************************************************************


//DOM-IGNORE-END
// *****************************************************************************
// *****************************************************************************
// Section: RTCC Peripheral Library Interface Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    void PLIB_RTCC_Enable ( RTCC_MODULE_ID index )

  Summary:
    Enables the specific RTCC module on the device.

  Description:
    This function enables the specific RTCC module on the device.

  Precondition:
    The RTCC module should be unlocked for writing using the function
    PLIB_RTCC_WriteEnable before this function is called.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_Enable(RTCC_ID_0);
    </code>

  Remarks:
    By calling this function, the RTCC pins are controlled by the RTCC
    module. The RTCC module will continue to function when the device is held 
    in reset.

    This function implements an operation of the EnableControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsEnableControl in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_Enable ( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_RTCC_Disable ( RTCC_MODULE_ID index )

  Summary:
    Disables the specific RTCC module on the device.

  Description:
    This function disables the specific RTCC module on the device.

  Precondition:
    The RTCC module should be unlocked for writing using the function
    PLIB_RTCC_WriteEnable before this function is called.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_Disable(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the EnableControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsEnableControl in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_Disable ( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_RTCC_WriteEnable ( RTCC_MODULE_ID index )

  Summary:
    Enables writing to the specific RTCC module's value registers.

  Description:
    This function enables writing to the specific RTCC module's value
    registers.

  Precondition:
    The SYSLOCK unlock sequence must be executed prior to calling this
    function by calling the PLIB_CORE_SysUnlock function.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_WriteEnable(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the WriteEnable feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsWriteEnable in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_WriteEnable ( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_RTCC_WriteDisable ( RTCC_MODULE_ID index )

  Summary:
    Disables writing to the specific RTCC module's value registers.

  Description:
    This function disables writing to the specific RTCC module's value
    registers.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_WriteDisable(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the WriteEnable feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsWriteEnable in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_WriteDisable ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    bool PLIB_RTCC_RTCSyncStatusGet ( RTCC_MODULE_ID index )

  Summary:
    The function returns the synchronization status bit.

  Description:
    The function returns the synchronization status bit, which is used to
    determine whether it is safe to read the date/time values, or if the
	values will change within 32 RTCC clocks.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true  - Date and time will change within 32 RTCC clocks
    - false - Date and time are safe to read, and will not change soon

  Example:
    <code>
    if (PLIB_RTCC_RTCSyncStatusGet(RTCC_ID_0))
	{
	...
	}
    </code>

  Remarks:
    This bit is read-only.
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsSynchronization in your application to automatically 
    determine whether this feature is available.
*/

bool PLIB_RTCC_RTCSyncStatusGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    bool PLIB_RTCC_HalfSecondStatusGet ( RTCC_MODULE_ID index )

  Summary:
    The function returns the half second status bit.

  Description:
    The function returns the half second status bit, which is used in the
    calibration procedure. When the seconds byte is zero, the calibration value
    must be updated when the half second bit becomes '1'.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true  - Second half period of a second
    - false - First half period of a second

  Example:
    <code>
    // Wait for the half second status bit to be '1'.
    while(PLIB_RTCC_HalfSecondStatusGet(RTCC_ID_0));
    </code>

  Remarks:
    This bit is read-only. It is cleared to '0' on a write to the seconds
    value.
    This function implements an operation of the HalfSecond feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsHalfSecond in your application to automatically 
    determine whether this feature is available.
*/

bool PLIB_RTCC_HalfSecondStatusGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_ClockOutputEnable ( RTCC_MODULE_ID index )

  Summary:
    Enables the specific RTCC module's output pin.

  Description:
    This function enables the specific RTCC module's output and generates a
    square wave using either the alarm or the 1 Hz clock output on the RTCC
    pin.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_ClockOutputEnable(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the OutputControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsOutputControl in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_ClockOutputEnable ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_ClockOutputDisable ( RTCC_MODULE_ID index )

  Summary:
    Disables the specific RTCC module's output pin.

  Description:
    This function disables the specific RTCC module's output pin.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_ClockOutputDisable(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the OutputControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsOutputControl in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_ClockOutputDisable ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_ClockSourceSelect ( RTCC_MODULE_ID index, RTCC_CLOCK_SOURCE_SELECT source )

  Summary:
    Selects the clock source for the RTCC module.

  Description:
    This function determines which clock source the RTCC module will use
    depending on the features of the device.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    source          - Which clock source will be used

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_ClockSourceSelect(RTCC_ID_0, RTCC_CLOCK_SOURCE_SELECT_NONE);
    </code>

  Remarks:
    This function implements an operation of the ClockSelect feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsClockSelect in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_ClockSourceSelect ( RTCC_MODULE_ID index, RTCC_CLOCK_SOURCE_SELECT source );

//******************************************************************************
/* Function:
    bool PLIB_RTCC_ClockRunningStatus ( RTCC_MODULE_ID index )

  Summary:
    Provides the status of the RTCC clock.

  Description:
    This function provides the status of the RTCC clock.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    The status of the RTCC clock.

  Example:
    <code>
    bool status;
    status = PLIB_RTCC_ClockRunningStatus(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the ClockRunning feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsClockRunning in your application to automatically 
    determine whether this feature is available.

*/

bool PLIB_RTCC_ClockRunningStatus ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    uint16_t PLIB_RTCC_DriftCalibrateGet ( RTCC_MODULE_ID index )

  Summary:
    Reads the specific RTCC module's drift calibration bits.

  Description:
    This function reads the specific RTCC module's drift calibration bits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    uint16_t        - The current drift calibration value

  Example:
    <code>
    uint16_t calibrationbits;
    calibrationbits = PLIB_RTCC_DriftCalibrateGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the Calibration feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsCalibration in your application to automatically 
    determine whether this feature is available.
*/

uint16_t PLIB_RTCC_DriftCalibrateGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_DriftCalibrateSet ( RTCC_MODULE_ID index, uint16_t calibrationbits )

  Summary:
    Sets the specific RTCC module's drift calibration bits.

  Description:
    This function sets the specific RTCC module's drift calibration bits. The 
    error between the system clock and the external clock has to be computed and 
    calibration input must be provided to this function.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

    calibrationbits            - Drift calibration bits

  Returns:
    None.

  Example:
    <code>
    uint16_t calibrationbits = 3; //Positive 3 adjustment derived from the formula
                                  // Error = (Ideal Freq(32758) - Measured)*60;
    PLIB_RTCC_DriftCalibrateSet(RTCC_ID_0, calibrationbits);
    </code>

  Remarks:
    This function implements an operation of the Calibration feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsCalibration in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_DriftCalibrateSet ( RTCC_MODULE_ID index, uint16_t calibrationbits);

//******************************************************************************
/* Function:
    void PLIB_RTCC_StopInIdleEnable( RTCC_MODULE_ID index)

  Summary:
    Disables access to the RTCC module by the Peripheral Bus Clock (PBCLK) when
    the CPU enters Idle mode.

  Description:
    This function disables access to the RTCC module by the PBCLK when the CPU is
    in Idle mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_StopInIdleEnable(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the StopInIdle feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsStopInIdle in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_StopInIdleEnable ( RTCC_MODULE_ID index);

//******************************************************************************
/* Function:
    void PLIB_RTCC_StopInIdleDisable( RTCC_MODULE_ID index)

  Summary:
    Continues normal RTCC operation when the device enters Idle mode.

  Description:
    This function continues normal RTCC operation when the device enters Idle mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_StopInIdleDisable(RTCC_ID_0);
    </code>

  Remarks:
    None.

    This function implements an operation of the StopInIdle feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsStopInIdle in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_StopInIdleDisable ( RTCC_MODULE_ID index);

//******************************************************************************
/* Function:
    void PLIB_RTCC_OutputSelect ( RTCC_MODULE_ID index, RTCC_OUTPUT_SELECT data )

  Summary:
    Selects which signal will be presented on the RTCC pin

  Description:
    This function selects which signal will be presented on the RTCC pin.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    data            - Enumerated value of which signal to present
	
  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_OutputSelect(RTCC_ID_0, RTCC_OUTPUT_SECONDS_CLOCK);
    </code>

  Remarks:
    The RTCC module's output pin should be enabled using the function
    PLIB_RTCC_OutputEnable.

    This function implements an operation of the OutputSelect feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsOutputSelect in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_OutputSelect ( RTCC_MODULE_ID index, RTCC_OUTPUT_SELECT data);

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmEnable ( RTCC_MODULE_ID index )

  Summary:
    Enables the specific RTCC module's alarm.

  Description:
    This function enables the specific RTCC module's alarm.
    RTCC Alarm bit shouldn't be modified when RTCC on bit is enabled and 
    PLIB_RTCC_AlarmSyncStatusGet (ALRMSYNC bit) returns true.
    Meaning the check RTCC ON (RTCCON<15>) bit and the ALRMSYNC bit should be 
	equal to '1'.    

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // check RTCC enable bit and PLIB_RTCC_AlarmSyncStatusGet return value and
    // then modify the Alarm bit.
    PLIB_RTCC_AlarmEnable(RTCC_ID_0);
    </code>

  Remarks:
    The alarm enable bit is cleared automatically after an alarm event
    whenever the alarm is not set up to repeat, and chime is disabled.

    This function implements an operation of the AlarmControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmControl in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_AlarmEnable ( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmDisable ( RTCC_MODULE_ID index )

  Summary:
    Disables the specific RTCC module's alarm.

  Description:
    This function disables the specific RTCC module's alarm.
    RTCC Alarm bit shouldn't be modified when RTCC on bit is enabled and 
    PLIB_RTCC_AlarmSyncStatusGet (ALRMSYNC bit) returns true.
    Meaning the check RTCC ON (RTCCON<15>) bit and the ALRMSYNC bit should be 
	equal to '1'.
	
  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // check RTCC enable bit and PLIB_RTCC_AlarmSyncStatusGet return value and
    // then modify the Alarm bit.
    PLIB_RTCC_AlarmDisable(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmControl in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_AlarmDisable ( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmChimeEnable ( RTCC_MODULE_ID index )

  Summary:
    Enables the specific RTCC module's chime.

  Description:
    This function enables the specific RTCC module's chime. The alarm repeat 
    count bits are allowed to rollover.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_AlarmChimeEnable(RTCC_ID_0);
    </code>

  Remarks:
     This function implements an operation of the AlarmChimeControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmChimeControl in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_AlarmChimeEnable ( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmChimeDisable ( RTCC_MODULE_ID index )

  Summary:
    Disables the specific RTCC module's chime.

  Description:
    This function disables the specific RTCC module's chime. The alarm repeat 
    count value bits stop once they reach zero.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_RTCC_AlarmChimeDisable(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmChimeControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmChimeControl in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_AlarmChimeDisable ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmPulseInitialSet ( RTCC_MODULE_ID index )

  Summary:
    Enables the determination of the initial alarm pulse.

  Description:
    This function enables the determination of initial alarm pulse.

  Precondition:
    The ALRMEN bit should be '0' indicating the PLIB_RTCC_AlarmDisable was called.
    This function must not be called when the RTCC is ON and the Alarm Sync is 1.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    bool data = 0;
    PLIB_RTCC_AlarmPulseInitialSet(RTCC_ID_0, data);
    </code>

  Remarks:
    This function implements an operation of the AlarmPulseInitial feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmPulseInitial in your application to automatically 
    determine whether this feature is available.

*/

void PLIB_RTCC_AlarmPulseInitialSet ( RTCC_MODULE_ID index, bool data );

//******************************************************************************
/* Function:
    bool PLIB_RTCC_AlarmPulseInitialGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the state of the initial alarm pulse.

  Description:
    This function returns the state of the initial alarm pulse.

  Precondition:
    The ALRMEN bit should be '1' indicating the PLIB_RTCC_AlarmEnable function was
    called.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    * 1 - Logical High
    * 0 - Logical Low

  Example:
    <code>
    bool PulseValue;
    PulseValue = PLIB_RTCC_AlarmPulseInitialGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmPulseInitial feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmPulseInitial in your application to automatically 
    determine whether this feature is available.

*/

bool PLIB_RTCC_AlarmPulseInitialGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmMaskModeSelect ( RTCC_MODULE_ID index, RTCC_ALARM_MASK_CONFIGURATION data )

  Summary:
    Sets the specific RTCC module's alarm mask Configuration bits.

  Description:
    This function sets the specific RTCC module's alarm mask Configuration bits.

  Remarks:
    The actual definition of this enumeration is device-specific.

    Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    data            - Alarm mask Configuration bits

  Returns:
    None.

  Example:
    <code>
    uint8_t data = 0;
    PLIB_RTCC_AlarmMaskModeSelect(RTCC_ID_0, RTCC_ALARM_EVERY_HALF_SECOND);
    </code>

  Remarks:
    This function implements an operation of the AlarmMaskControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmMaskControl in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmMaskModeSelect ( RTCC_MODULE_ID index, RTCC_ALARM_MASK_CONFIGURATION data );


//******************************************************************************
/* Function:
    bool PLIB_RTCC_AlarmSyncStatusGet ( RTCC_MODULE_ID index )

  Summary:
    The function returns the synchronization status bit.

  Description:
    The function returns the synchronization status bit, which is used to
    determine whether it is safe to read the date/time values, or if the
	values will change within 32 RTCC clocks.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true  - Alarm repeat count may change as a result of a half-second 
			  rollover during a read.
    - false - Alarm repeat count is safe to read, and will not change
			  in less than 32 RTC clocks.

  Example:
    <code>
    if (PLIB_RTCC_AlarmSyncStatusGet(RTCC_ID_0))
	{
	...
	}
    </code>

  Remarks:
    This bit is read-only.
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmSynchronization in your application to automatically 
    determine whether this feature is available.
*/

bool PLIB_RTCC_AlarmSyncStatusGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmValueRegisterPointer ( RTCC_MODULE_ID index, uint8_t data )

  Summary:
    Sets the specific RTCC module's Alarm register pointer.

  Description:
    This function sets the specific RTCC module's Alarm register pointer.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    data            - Alarm register pointer

  Returns:
    None.

  Example:
    <code>
    uint8_t data = 2;
    PLIB_RTCC_AlarmValueRegisterPointer(RTCC_ID_0, data);
    </code>

  Remarks:
    None.
*/

void PLIB_RTCC_AlarmValueRegisterPointer ( RTCC_MODULE_ID index, uint8_t data );


//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmRepeatCountSet ( RTCC_MODULE_ID index, uint8_t data )

  Summary:
    Sets the specific RTCC module's alarm repeat counter.

  Description:
    This function sets the specific RTCC module's alarm repeat counter.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    data            - Alarm repeat counter bits

  Returns:
    None.

  Example:
    <code>
    uint8_t data = 0xFF;
    PLIB_RTCC_AlarmRepeatCountSet(RTCC_ID_0, data);
    </code>

  Remarks:
    The counter decrements on any alarm event. The counter is prevented from
    rolling over unless chime is enabled.

    This function implements an operation of the AlarmRepeatControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmRepeatControl in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmRepeatCountSet ( RTCC_MODULE_ID index, uint8_t data );

//******************************************************************************
/* Function:
    uint8_t PLIB_RTCC_AlarmRepeatCountGet ( RTCC_MODULE_ID index )

  Summary:
    Reads the specific RTCC module's alarm repeat counter.

  Description:
    This function reads the specific RTCC module's alarm repeat counter.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    uint8_t         - The current value of the alarm repeat counter

  Example:
    <code>
    uint8_t currentCount;
    currentCount = PLIB_RTCC_AlarmRepeatCountGet(RTCC_ID_0);
    </code>

  Remarks:
    The counter decrements on any alarm event. The counter is prevented from
    rolling over unless chime is enabled.

    This function implements an operation of the AlarmRepeatControl feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmRepeatControl in your application to automatically 
    determine whether this feature is available.
*/

uint8_t PLIB_RTCC_AlarmRepeatCountGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCTimeGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the specific RTCC module's Time register.

  Description:
    The function returns the contents of the specific RTCC module's Time
    register. Please refer to the specific device data sheet for the exact 
    sequence of digits.

  Precondition:

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Time register contents.

  Example:
    <code>
    uint32_t time;
    time = PLIB_RTCC_RTCTimeGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCTime in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCTimeGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_RTCTimeSet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Time register.

  Description:
    The function writes to the specific RTCC module's Time register. Please
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Time register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    uint32_t data = 0x04153300;     // Time = 4 hours, 15 minutes, and 33 seconds
    PLIB_RTCC_RTCTimeSet(RTCC_ID_0, data);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the RTCTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCTime in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCTimeSet ( RTCC_MODULE_ID index, uint32_t data );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCHourGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the Hours bits in the specific RTCC module's Time 
    register.

  Description:
    The function returns the contents of the Hours bits in the specific RTCC 
    module's Time register. Please refer to the specific device data sheet for
    the exact sequence of digits.

  Precondition:

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    BCD value of the Hours bits in the Time register.

  Example:
    <code>
    uint32_t Hour;
    Hour = PLIB_RTCC_RTCHourGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCTime in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCHourGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCHourSet ( RTCC_MODULE_ID index, uint32_t hour )

  Summary:
    Writes the contents of the Hours bits in the specific RTCC module's Time
    register.

  Description:
    The function writes the contents of the Hours bits in the specific RTCC
    module's Time register. Please refer to the specific device data sheet for
    the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    hour            - BCD value to be written to the Hours bits in the Time register

  Returns:
    None.

  Example:
    <code>
    uint32_t Hour = 0x04;
    PLIB_RTCC_RTCHourSet(RTCC_ID_0, Hour);
    </code>

  Remarks:
    This function implements an operation of the RTCTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCTime in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCHourSet ( RTCC_MODULE_ID index, uint32_t hour );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCMinuteGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the Minutes bits in the specific RTCC module's Time 
    register.

  Description:
    The function returns the contents of the Minutes bits in the specific RTCC 
    module's Time register. Please refer to the specific device data sheet for 
    the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    BCD value of the Minutes bits in the Time register.

  Example:
    <code>
    uint32_t Minute;
    Minute = PLIB_RTCC_RTCMinuteGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCTime in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCMinuteGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCMinuteSet ( RTCC_MODULE_ID index, uint32_t minute )

  Summary:
    Writes the contents of Minutes bits in the specific RTCC module's Time
    register.

  Description:
    The function writes the contents of these bits in the specific RTCC module's
    Time register. Please refer to the specific device data sheet for the exact
    sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    minute          - BCD value to be written to the Minutes bits in the Time 
                      register

  Returns:
    None.

  Example:
    <code>
    uint32_t Minute = 0x15;
    PLIB_RTCC_RTCMinuteSet(RTCC_ID_0, Minute);
    </code>

  Remarks:
    This function implements an operation of the RTCTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCTime in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCMinuteSet ( RTCC_MODULE_ID index, uint32_t minute );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCSecondGet ( RTCC_MODULE_ID index )

  Summary:
    The function returns the contents of the Seconds bits in the specific RTCC
    device's Time register.

  Description:
    The function returns the contents of the Seconds bits in the specific RTCC 
    module's Time register. Please refer to the specific device data sheet for 
    the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    BCD value of the Seconds bits in the Time register.

  Example:
    <code>
    uint32_t Second;
    Second = PLIB_RTCC_RTCSecondGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCTime in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCSecondGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCSecondSet ( RTCC_MODULE_ID index, uint32_t second )

  Summary:
    Writes the contents of Seconds bits in the specific RTCC module's Time 
    register.

  Description:
    The function writes the contents of the Seconds bits in the specific RTCC 
    module's Time register. Please refer to the specific device data sheet for 
    the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    second          - BCD value to be written to the Seconds bits in the Time 
                      register

  Returns:
    None.

  Example:
    <code>
    uint32_t Second = 0x33;
    PLIB_RTCC_RTCSecondSet(RTCC_ID_0, Second);
    </code>

  Remarks:
    This function implements an operation of the RTCTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCTime in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCSecondSet ( RTCC_MODULE_ID index, uint32_t second );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCDateGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the specific RTCC module's Date register.

  Description:
    The function returns the contents of the specific RTCC module's Date
    register. Please refer to the specific device data sheet for the exact 
	sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Date register contents.

  Example:
    <code>
    uint32_t Date;
    Date = PLIB_RTCC_RTCDateGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCDateGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_RTCDateSet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Date register.

  Description:
    The function writes to the specific RTCC module's Date register. Please
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    uint32_t data = 0x06102705;     //Date = 27 Oct 2006 Friday
    PLIB_RTCC_RTCDateSet(RTCC_ID_0, data);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCDateSet ( RTCC_MODULE_ID index, uint32_t data );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCYearGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the Year bits in the specific RTCC module's Date 
    register.

  Description:
    The function returns the contents of the specific RTCC module's Date register. 
    Please refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Year bits in the Date register.

  Example:
    <code>
    uint32_t Year;
    Year = PLIB_RTCC_RTCYearGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCYearGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_RTCYearSet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Date register.

  Description:
    The function writes to the specific RTCC module's Date register. Please
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured
    year            - The BCD value of the year to set in the Date register

  Returns:
    None.

  Example:
    <code>
    uint32_t Year = 0x06;       //Year = 2006
    PLIB_RTCC_RTCYearSet(RTCC_ID_0, Year);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCYearSet ( RTCC_MODULE_ID index, uint32_t year );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCMonthGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the Months bits in the specific RTCC module's Date 
    register.

  Description:
    The function returns the contents of the Months bits in the specific RTCC 
    module's Date register. Please refer to the specific device data sheet for 
    the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Months bits in the Date register.

  Example:
    <code>
    uint32_t Month;
    Month = PLIB_RTCC_RTCMonthGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCMonthGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_RTCMonthSet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Date register.

  Description:
    The function writes to the specific RTCC module's Date register. Please
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured
    month           - The BCD value of the month to set in the Date register

  Returns:
    None.

  Example:
    <code>
    uint32_t Month = 0x10;      //Month = October
    PLIB_RTCC_RTCMonthSet(RTCC_ID_0, Month);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCMonthSet ( RTCC_MODULE_ID index, uint32_t month );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCDayGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the Days bits in the specific RTCC module's Date register.

  Description:
    The function returns the contents of the Days bits in the specific RTCC module's 
    Date register. Please refer to the specific device data sheet for the exact
    sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Days bits in the Date register.

  Example:
    <code>
    uint32_t Day;
    Day = PLIB_RTCC_RTCDayGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCDayGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_RTCDaySet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Date register.

  Description:
    The function writes to the specific RTCC module's Date register. Please
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index          - Identifier for the device instance to be configured
    day            - The BCD value of the day to set in the Date register

  Returns:
    None.

  Example:
    <code>
    uint32_t Day = 0x27;        //Day = 27th of the month
    PLIB_RTCC_RTCDaySet(RTCC_ID_0, Day);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCDaySet ( RTCC_MODULE_ID index, uint32_t day );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_RTCWeekDayGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the WeekDay bits in the specific RTCC module's Date 
    register.

  Description:
    The function returns the contents of the WeekDay bits in the specific RTCC 
    module's Date register. Please refer to the specific device data sheet for 
    the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    WeekDay field in the Date register.

  Example:
    <code>
    uint32_t WeekDay;
    WeekDay = PLIB_RTCC_RTCWeekDayGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_RTCWeekDayGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_RTCWeekDaySet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Date register.

  Description:
    The function writes to the specific RTCC module's Date register. Please
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured
    weekday         - The BCD value of the weekday to set in the Date register

  Returns:
    None.

  Example:
    <code>
    uint32_t WeekDay = 0x05;        //WeekDay = Friday
    PLIB_RTCC_RTCWeekDaySet(RTCC_ID_0, WeekDay);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the RTCDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsRTCDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_RTCWeekDaySet ( RTCC_MODULE_ID index, uint32_t weekday );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmTimeGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the specific RTCC module's Alarm Time register.

  Description:
    The function returns the contents of the specific RTCC module's Alarm Time
    register. Please refer to the specific device data sheet for the exact 
    sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Value register.

  Example:
    <code>
    uint32_t time;
    time = PLIB_RTCC_AlarmTimeGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmTime in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_AlarmTimeGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmTimeSet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Alarm Time register.

  Description:
    The function writes to the specific RTCC module's Alarm Time register. Please 
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Alarm Time register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    uint32_t data = 0x04153300;     // Time = 4 hours, 15 minutes, and 33 seconds
    PLIB_RTCC_AlarmTimeSet(RTCC_ID_0, data);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the AlarmTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmTime in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmTimeSet ( RTCC_MODULE_ID index, uint32_t data );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmHourGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of Hours bits in the specific RTCC module's Alarm Time 
    register.

  Description:
    The function returns the contents of the Hours bits in the specific RTCC 
    module's Alarm Time register. Please refer to the specific device data sheet 
    for the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    BCD value of the Hours bits in the Alarm Time register.

  Example:
    <code>
    uint32_t Hour;
    Hour = PLIB_RTCC_AlarmHourGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmTime in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_AlarmHourGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmHourSet ( RTCC_MODULE_ID index, uint32_t hour )

  Summary:
    The function returns the contents of Hours bits in the specific RTCC module's
    Alarm Time register.

  Description:
    Returns the contents of the Hours bits in the specific RTCC module's
    Alarm Time register. Please refer to the specific device data sheet for the 
    exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    hour            - BCD value to be written to the Hours bits in the Alarm Time 
                      register

  Returns:
    None

  Example:
    <code>
    uint32_t Hour = 0x04;
    PLIB_RTCC_AlarmHourSet(RTCC_ID_0, Hour);
    </code>

  Remarks:
    This function implements an operation of the AlarmTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmTime in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmHourSet ( RTCC_MODULE_ID index, uint32_t hour );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmMinuteGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of Minutes bits in the specific RTCC module's
    Alarm Time register.

  Description:
    The function returns the contents of the field in the specific RTCC module's
    Alarm Time register. Please refer to the specific device data sheet for the 
    exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    BCD value of the Minutes bits in the Alarm Time register.

  Example:
    <code>
    uint32_t Minute;
    Minute = PLIB_RTCC_AlarmMinuteGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmTime in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_AlarmMinuteGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmMinuteSet ( RTCC_MODULE_ID index, uint32_t minute )

  Summary:
    Returns the contents of the Minutes bits in the specific RTCC module's Alarm 
    Time register.

  Description:
    The function returns the contents of the Minutes bits in the specific RTCC 
    module's Alarm Time register. Please refer to the specific device data sheet 
    for the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    minute          - BCD value to be written to the Minutes bits in the 
                      Alarm Time register

  Returns:
    None

  Example:
    <code>
    uint32_t Minute = 0x15;
    PLIB_RTCC_AlarmMinuteSet(RTCC_ID_0, Minute);
    </code>

  Remarks:
    This function implements an operation of the AlarmTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmTime in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmMinuteSet ( RTCC_MODULE_ID index, uint32_t minute );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmSecondGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the Seconds bits in the specific RTCC module's Alarm 
    Time register.

  Description:
    The function returns the contents of the Seconds bits in the specific RTCC 
    module's Alarm Time register. Please refer to the specific device data sheet 
    for the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    BCD value of the Seconds bits in the Alarm Time register.

  Example:
    <code>
    uint32_t Second;
    Second = PLIB_RTCC_AlarmSecondGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmTime in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_AlarmSecondGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmSecondSet ( RTCC_MODULE_ID index, uint32_t second )

  Summary:
    Returns the contents of Seconds bits in the specific RTCC module's Alarm Time 
    register.

  Description:
    The function returns the contents of the field in the specific RTCC module's
    Alarm Time register. Please refer to the specific device data sheet for the exact
    sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    second          - BCD value to be written to the Seconds bits in the Alarm 
                      Time register

  Returns:
    None

  Example:
    <code>
    uint32_t Second = 0x33;
    PLIB_RTCC_AlarmSecondSet(RTCC_ID_0, Second);
    </code>

  Remarks:
    This function implements an operation of the AlarmTime feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmTime in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmSecondSet ( RTCC_MODULE_ID index, uint32_t second );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmDateGet ( AlarmC_MODULE_ID index )

  Summary:
    Returns the contents of the specific RTCC module's Alarm Date register.

  Description:
    The function returns the contents of the specific RTCC module's Alarm Date
    register. Please refer to the specific device data sheet for the exact 
    sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Value register.

  Example:
    <code>
    uint32_t Date;
    Date = PLIB_RTCC_AlarmDateGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_AlarmDateGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmDateSet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Alarm Date register.

  Description:
    The function writes to the specific RTCC module's Alarm Date register. Please
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Alarm Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured
    data            - The value to set the Alarm Date register to, in BCD format

  Returns:
    None.

  Example:
    <code>
    uint32_t data = 0x06102705;     //Date = 27 Oct 2006 Friday
    PLIB_RTCC_AlarmDateSet(RTCC_ID_0, data);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the AlarmDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmDateSet ( RTCC_MODULE_ID index, uint32_t data );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmMonthGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the Month bits in the specific RTCC module's Alarm 
    Date register.

  Description:
    The function returns the contents of the Months bits in the specific RTCC 
    module's Alarm Date register. Please refer to the specific device data sheet 
    for the exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Months bits in the Date register.

  Example:
    <code>
    uint32_t Month;
    Month = PLIB_RTCC_AlarmMonthGet(RTCC_ID_0);
    </code>

  Remarks:

    This function implements an operation of the AlarmDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_AlarmMonthGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmMonthSet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Alarm Date register.

  Description:
    The function writes to the specific RTCC module's Alarm Date register. Please 
    refer to the specific device data sheet for the exact sequence of digits.

  Precondition:
    Prior to writing to the Alarm Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured
    month           - The BCD value of the month to set in the Alarm Date register

  Returns:
    None.

  Example:
    <code>
    uint32_t Month = 0x10;      //Month = October
    PLIB_RTCC_AlarmMonthSet(RTCC_ID_0, Month);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the AlarmDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmMonthSet ( RTCC_MODULE_ID index, uint32_t month );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmDayGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the Day bits in the specific RTCC module's Alarm 
    Date register.

  Description:
    The function returns the contents of Day bits in the specific RTCC module's 
    Alarm Date register. Please refer to the specific device data sheet for the 
    exact sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    Days bits in the Alarm Date register.

  Example:
    <code>
    uint32_t Day;
    Day = PLIB_RTCC_AlarmDayGet(RTCC_ID_0);
    </code>

  Remarks:

    This function implements an operation of the AlarmDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_AlarmDayGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmDaySet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Alarm Date value register.

  Description:
    The function writes to the specific RTCC module's Alarm Date register.
    Please refer to the specific device data sheet for the exact sequence of
    digits.

  Precondition:
    Prior to writing to the Alarm Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured
    day             - The BCD value of the day to set in the Alarm Date register

  Returns:
    None.

  Example:
    <code>
    uint32_t Day = 0x27;        //Day = 27th of the month
    PLIB_RTCC_AlarmDaySet(RTCC_ID_0, Day);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the AlarmDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmDaySet ( RTCC_MODULE_ID index, uint32_t day );

//******************************************************************************
/* Function:
    uint32_t PLIB_RTCC_AlarmWeekDayGet ( RTCC_MODULE_ID index )

  Summary:
    Returns the contents of the WeekDay bits in the specific RTCC module's Alarm 
    Date register.

  Description:
    The function returns the contents of Weekday bits in the specific RTCC module's
    Alarm Date register. Please refer to the specific device data sheet for the exact
    sequence of digits.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    WeekDay bits in the Alarm Date register.

  Example:
    <code>
    uint32_t WeekDay;
    WeekDay = PLIB_RTCC_AlarmWeekDayGet(RTCC_ID_0);
    </code>

  Remarks:
    This function implements an operation of the AlarmDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmDate in your application to automatically 
    determine whether this feature is available.
*/

uint32_t PLIB_RTCC_AlarmWeekDayGet ( RTCC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_RTCC_AlarmWeekDaySet ( RTCC_MODULE_ID index, uint32_t data )

  Summary:
    Writes to the specific RTCC module's Alarm Date register.

  Description:
    The function writes to the specific RTCC module's Alarm Date register.
    Please refer to the specific device data sheet for the exact sequence of
    digits.

  Precondition:
    Prior to writing to the Alarm Date register, an RTCC write must be
    enabled using the exact sequences required by the device.

  Parameters:
    index           - Identifier for the device instance to be configured
    weekday         - The BCD value of the weekday to set in the field of the 
                      Alarm Date register

  Returns:
    None.

  Example:
    <code>
    uint32_t WeekDay = 0x05;        //WeekDay = Friday
    PLIB_RTCC_AlarmWeekDaySet(RTCC_ID_0, WeekDay);
    </code>

  Remarks:
    A write to this register is only allowed when access is allowed by
    using the PLIB_RTCC_WriteEnable function.

    This function implements an operation of the AlarmDate feature.  
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_RTCC_ExistsAlarmDate in your application to automatically 
    determine whether this feature is available.
*/

void PLIB_RTCC_AlarmWeekDaySet ( RTCC_MODULE_ID index, uint32_t weekday );


// *****************************************************************************
// *****************************************************************************
// Section: RTCC Peripheral Library Exists Functions
// *****************************************************************************
// *****************************************************************************
/* The following functions indicate the existence of the features on the device.
*/

//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsEnableControl( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the EnableControl feature exists on the RTCC module.

  Description:
    This function identifies whether the EnableControl feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_Enable
    - PLIB_RTCC_Disable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EnableControl feature is supported on the device
    - false  - The EnableControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsEnableControl( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsWriteEnable( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the WriteEnable feature exists on the RTCC module.

  Description:
    This function identifies whether the WriteEnable feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_WriteEnable
    - PLIB_RTCC_WriteDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The WriteEnable feature is supported on the device
    - false  - The WriteEnable feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsWriteEnable( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsStopInIdleControl( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the StopInIdle feature exists on the RTCC module.

  Description:
    This function identifies whether the StopInIdle feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_StopInIdleEnable
    - PLIB_RTCC_StopInIdleDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The StopInIdle feature is supported on the device
    - false  - The StopInIdle feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsStopInIdleControl( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsOutputSelect( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the OutputSelect feature exists on the RTCC module.

  Description:
    This function identifies whether the OutputSelect feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_SecondsClockOutputSelect
    - PLIB_RTCC_AlarmPulseOutputSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OutputSelect feature is supported on the device
    - false  - The OutputSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsOutputSelect( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsClockSelect( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the ClockSelect feature exists on the RTCC module.

  Description:
    This function identifies whether the ClockSelect feature is available on the
    RTCC module. When this interface returns true, this function is supported
    on the device:
    - PLIB_RTCC_SourceClockSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ClockSelect feature is supported on the device
    - false  - The ClockSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsClockSelect( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsClockRunning( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the ClockRunning feature exists on the RTCC module.

  Description:
    This function identifies whether the ClockRunning feature is available on the
    RTCC module. When this interface returns true, this function is supported
    on the device:
    - PLIB_RTCC_ClockRunningStatus

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ClockRunning feature is supported on the device
    - false  - The ClockRunning feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsClockRunning( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsCalibration( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the Calibration feature exists on the RTCC module.

  Description:
    This function identifies whether the Calibration feature is available on the
    RTCC module. When this interface returns true, this function is supported
    on the device:
    - PLIB_RTCC_DriftCalibrate

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Calibration feature is supported on the device
    - false  - The Calibration feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsCalibration( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsSynchronization( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the Synchronization feature exists on the RTCC module.

  Description:
    This function identifies whether the Synchronization feature is available on the
    RTCC module. When this interface returns true, this function is supported
    on the device:
    - PLIB_RTCC_RTCSyncStatusGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Synchronization feature is supported on the device
    - false  - The Synchronization feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsSynchronization( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsHalfSecond( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the HalfSecond feature exists on the RTCC module.

  Description:
    This function identifies whether the HalfSecond feature is available on the
    RTCC module. When this interface returns true, this function is supported
    on the device:
    - PLIB_RTCC_HalfSecondStatusBit

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HalfSecond feature is supported on the device
    - false  - The HalfSecond feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsHalfSecond( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsOutputControl( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the OutputControl feature exists on the RTCC module.

  Description:
    This function identifies whether the OutputControl feature is available on the
    RTCC module. When this interface returns true, this function is supported
    on the device:
    - PLIB_RTCC_ClockOutputEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OutputControl feature is supported on the device
    - false  - The OutputControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsOutputControl( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsAlarmControl( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the AlarmControl feature exists on the RTCC module.

  Description:
    This function identifies whether the AlarmControl feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_AlarmEnable
    - PLIB_RTCC_AlarmDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlarmControl feature is supported on the device
    - false  - The AlarmControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsAlarmControl( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsAlarmChimeControl( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the AlarmChimeControl feature exists on the RTCC module.

  Description:
    This function identifies whether the AlarmChimeControl feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_AlarmChimeEnable
    - PLIB_RTCC_AlarmChimeDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlarmChimeControl feature is supported on the device
    - false  - The AlarmChimeControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsAlarmChimeControl( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsAlarmPulseInitial( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the AlarmPulseInitial feature exists on the RTCC module.

  Description:
    This function identifies whether the AlarmPulseInitialValue feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_AlarmPulseInitialSet
    - PLIB_RTCC_AlarmPulseInitialGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlarmPulseInitial feature is supported on the device
    - false  - The AlarmPulseInitial feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsAlarmPulseInitial( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsAlarmSynchronization( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the AlarmSynchronization feature exists on the RTCC module.

  Description:
    This function identifies whether the AlarmSynchronization feature is available on the
    RTCC module. When this interface returns true, this function is supported 
    on the device:
    - PLIB_RTCC_AlarmSyncGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlarmSynchronization feature is supported on the device
    - false  - The AlarmSynchronization feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsAlarmSynchronization( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsAlarmMaskControl( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the AlarmMaskControl feature exists on the RTCC module.

  Description:
    This function identifies whether the AlarmMaskControl feature is available on the
    RTCC module. When this interface returns true, this function is supported 
    on the device:
    - PLIB_RTCC_AlarmMaskModeSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlarmMaskControl feature is supported on the device
    - false  - The AlarmMaskControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsAlarmMaskControl( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsAlarmRepeatControl( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the AlarmRepeatControl feature exists on the RTCC module.

  Description:
    This function identifies whether the AlarmRepeatControl feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_AlarmRepeatCountSet
    - PLIB_RTCC_AlarmRepeatCountRead

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlarmRepeatControl feature is supported on the device
    - false  - The AlarmRepeatControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsAlarmRepeatControl( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsRTCTime( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the RTCTimeValue feature exists on the RTCC module.

  Description:
    This function identifies whether the RTCTimeValue feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_RTCTimeGet
    - PLIB_RTCC_RTCTimeSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The RTCTime feature is supported on the device
    - false  - The RTCTime feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsRTCTime( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsRTCDate( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the RTCDateValue feature exists on the RTCC module.

  Description:
    This function identifies whether the RTCDateValue feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_RTCDateGet
    - PLIB_RTCC_RTCDateSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The RTCDate feature is supported on the device
    - false  - The RTCDate feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsRTCDate( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsAlarmTime( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the AlarmTime feature exists on the RTCC module.

  Description:
    This function identifies whether the AlarmTime feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_AlarmTimeGet
    - PLIB_RTCC_AlarmTimeSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlarmTime feature is supported on the device
    - false  - The AlarmTime feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsAlarmTime( RTCC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_RTCC_ExistsAlarmDate( RTCC_MODULE_ID index )

  Summary:
    Identifies whether the AlarmDate feature exists on the RTCC module.

  Description:
    This function identifies whether the AlarmDate feature is available on the
    RTCC module. When this interface returns true, these functions are supported 
    on the device:
    - PLIB_RTCC_AlarmDateGet
    - PLIB_RTCC_AlarmDateSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlarmDate feature is supported on the device
    - false  - The AlarmDate feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_RTCC_ExistsAlarmDate( RTCC_MODULE_ID index );


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif  // #ifndef _PLIB_RTCC_H
/*******************************************************************************
 End of File
*/