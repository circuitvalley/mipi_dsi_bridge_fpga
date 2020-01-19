/*******************************************************************************
  Power Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_power.h

  Summary:
    Defines the Power Peripheral Library interface.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the Power
    Peripheral Library for Microchip microcontrollers.  The definitions in 
    this file are for the Power Controller module.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END

#ifndef _PLIB_POWER_H
#define _PLIB_POWER_H

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

#include "peripheral/power/processor/power_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Peripheral Library Interface Routines
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    void PLIB_POWER_PeripheralModuleDisable ( POWER_MODULE_ID index, POWER_MODULE source )

  Summary:
    Disable the power supply for the module selected in the source.

  Description:
    This function completely shuts down the selected peripheral, effectively 
    powering down all circuits and removing all clock sources. This has the 
    additional affect of making any of the module’s control and buffer registers, 
    which are mapped in the SFR space, unavailable for operations.
	
  Precondition:
    The PMDLOCK bit of the Configuration register should be cleared using the
    function PLIB_DEVCON_DeviceRegistersUnlock (this function allows changes in the 
	PMD registers).
	Refer to the Device Control (DEVCON) Peripheral Library Help (or System Service) 
	and the specific device data sheet for more information.

  Parameters:
    index   - Identifier for the device instance to be configured
    source  - Select the module to be disabled from POWER_MODULE enum

  Returns:
    None.

  Example:
    <code>
    
    // System Unlock
    PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
    // Unlock PMD registers
    PLIB_DEVCON_DeviceRegistersUnlock(DEVCON_ID_0, DEVCON_PMD_REGISTERS);
    // Disable ADC module
    PLIB_POWER_PeripheralModuleDisable (POWER_ID_0, POWER_MODULE_ADC);
    
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsPeripheralModuleControl in your application to determine whether
	this feature is available.
    
    Disabling a peripheral module while it’s ON bit is set, may result in
    undefined behavior. The ON bit for the associated peripheral module
    must be cleared prior to disable a module via this API.
   
 */

void PLIB_POWER_PeripheralModuleDisable (POWER_MODULE_ID index, POWER_MODULE source);


//******************************************************************************
/* Function:
    void PLIB_POWER_PeripheralModuleEnable ( POWER_MODULE_ID index, POWER_MODULE source )

  Summary:
    Enable the power supply for the module selected in the source.

  Description:
    This function enables the power supply for the selected module.
	
  Precondition:
    The PMDLOCK bit of the Configuration register should be cleared using the
    function PLIB_DEVCON_DeviceRegistersUnlock (this function allows changes in the 
	PMD registers).
	Refer to the Device Control (DEVCON) Peripheral Library Help (or System Service) 
	and the specific device data sheet for more information.

  Parameters:
    index   - Identifier for the device instance to be configured
    source  - Select the module to be enabled from POWER_MODULE enum

  Returns:
    None.

  Example:
    <code>
    
    // System Unlock
    PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
    // Unlock PMD registers
    PLIB_DEVCON_DeviceRegistersUnlock(DEVCON_ID_0, DEVCON_PMD_REGISTERS);
    // Enable ADC module
    PLIB_POWER_PeripheralModuleEnable (POWER_ID_0, POWER_MODULE_ADC);
    
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsPeripheralModuleControl in your application to determine whether
	this feature is available.
   
 */

void PLIB_POWER_PeripheralModuleEnable (POWER_MODULE_ID index, POWER_MODULE source);

//******************************************************************************
/* Function:
    bool PLIB_POWER_PeripheralModuleIsEnabled ( POWER_MODULE_ID index, POWER_MODULE source )

  Summary:
    Checks to see whether or not the selected peripheral is enabled.

  Description:
    This function checks to see whether or not the selected peripheral is enabled.
	
  Precondition:
    None.

  Parameters:
    index   - Identifier for the device instance to be configured
    source  - Select the module to be enabled from the POWER_MODULE enum

  Returns:
    - true    - Power of the selected peripheral is enabled
    - false   - Power of the selected peripheral is disabled

  Example:
    <code>
    if (PLIB_POWER_PeripheralModuleIsEnabled ( POWER_MODULE_0, POWER_MODULE_ADC ))
	{
	
	}
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsPeripheralModuleControl in your application to determine whether
	this feature is available.
   
 */

bool PLIB_POWER_PeripheralModuleIsEnabled (POWER_MODULE_ID index, POWER_MODULE source);

//*******************************************************************************
/*  Function:
    void PLIB_POWER_VoltageRegulatorEnable (POWER_MODULE_ID index)

  Summary:
    Enable the voltage regulator during Sleep mode.

  Description:
    This function enables the voltage regulator during Sleep mode for the selected
    device.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_VoltageRegulatorEnable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsVoltageRegulatorControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_VoltageRegulatorEnable (POWER_MODULE_ID index);


//*******************************************************************************
/*  Function:
    bool PLIB_POWER_VoltageRegulatorIsEnabled (POWER_MODULE_ID index)

  Summary:
    Provides the status of the voltage regulator during Sleep mode.

  Description:
    This function provides the status of the voltage regulator during Sleep mode 
    for the selected device.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    - true    - The voltage regulator will be enabled in Sleep mode
    - false   - The voltage regulator will be disabled in Sleep mode

  Example:
    <code>
    if (PLIB_POWER_VoltageRegulatorIsEnabled(POWER_ID_0))
	{
	
	}
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsVoltageRegulatorControl in your application to determine whether
	this feature is available.
*/
bool PLIB_POWER_VoltageRegulatorIsEnabled (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_VoltageRegulatorDisable (POWER_MODULE_ID index)

  Summary:
    Disables the voltage regulator during Sleep mode.

  Description:
    This function disables the voltage regulator during Sleep mode for
    the selected device.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_VoltageRegulatorDisable(POWER_ID_0); 
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsVoltageRegulatorControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_VoltageRegulatorDisable (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    bool PLIB_POWER_DeviceWasInSleepMode (POWER_MODULE_ID index)

  Summary:
    Returns the Sleep mode status of the device.

  Description:
    This function returns the Sleep mode status of the device. It indicates whether 
    or not the device was in Sleep mode before waking up.
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	
  Returns:
    - true    - The device was in Sleep mode before waking up
    - false   - The device was not in Sleep mode

  Example:
    <code>
    if(PLIB_POWER_DeviceWasInSleepMode(POWER_ID_0))
    {
	
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsSleepStatus in your application to determine whether
	this feature is available.
*/

bool PLIB_POWER_DeviceWasInSleepMode (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_ClearSleepStatus (POWER_MODULE_ID index)

  Summary:
    Clear the Sleep mode status bit of the device.

  Description:
    This function clears the Sleep status bit of the device if it was previously 
    in Sleep mode. However, it does not mean that it wakes the device from sleep.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	
  Returns:
    None.

  Example:
    <code>
    if(PLIB_POWER_DeviceWasInSleepMode(POWER_ID_0))
    {
	    PLIB_POWER_ClearSleepStatus (POWER_ID_0);
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsSleepStatus in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_ClearSleepStatus (POWER_MODULE_ID index);

//******************************************************************************
/*  Function:
    bool PLIB_POWER_DeviceWasInIdleMode (POWER_MODULE_ID index)

  Summary:
    Returns the Idle mode status of the device.

  Description:
    This function returns the Idle mode status of the device. It indicates whether 
    or not the device was in Idle mode before waking up.
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	
  Returns:
    - true    - The device was in Idle mode before waking up
    - false   - The device was not in Idle mode


  Example:
    <code>
    if(PLIB_POWER_DeviceWasInIdleMode(POWER_ID_0))
    {
	
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsIdleStatus in your application to determine whether
	this feature is available.
*/

bool PLIB_POWER_DeviceWasInIdleMode (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_ClearIdleStatus (POWER_MODULE_ID index)

  Summary:
    Clears the Idle mode status of the device.

  Description:
    This function clears the Idle status bit of the device if it was previously
    in Idle mode. However, it does not mean that it wakes the device from Idle.
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	
  Returns:
    None.

  Example:
    <code>
    if(PLIB_POWER_DeviceWasInIdleMode(POWER_ID_0))
    {
	    PLIB_POWER_ClearIdleStatus (POWER_ID_0);
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsIdleStatus in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_ClearIdleStatus (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    bool PLIB_POWER_HighVoltageOnVDD1V8HasOccurred (POWER_MODULE_ID index)

  Summary:
    Returns the DDR2 High Voltage Detection status of the device.

  Description:
    This function returns the DDR2 High Voltage Detection status of the device. 
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	
  Returns:
    - true    - A high-voltage condition on the VDD1V8 voltage has occurred
    - false   - A high-voltage condition on the VDD1V8 voltage has not occurred


  Example:
    <code>
    if(PLIB_POWER_HighVoltageOnVDD1V8HasOccurred(POWER_ID_0))
    {
	
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsHighVoltageOnVDD1V8 in your application to determine whether
	this feature is available.
*/

bool PLIB_POWER_HighVoltageOnVDD1V8HasOccurred (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    bool PLIB_POWER_DeepSleepModeHasOccurred (POWER_MODULE_ID index)

  Summary:
    Returns the Deep Sleep mode status of the device.

  Description:
    This function returns whether or not the device was in Deep Sleep mode 
	before waking up.
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	
  Returns:
    - true    - The device was in Deep Sleep mode before waking up
    - false   - The device was not in Deep Sleep mode


  Example:
    <code>
    if(PLIB_POWER_DeepSleepModeHasOccurred(POWER_ID_0))
    {
	
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepModeOccurrence in your application to determine whether
	this feature is available.
*/

bool PLIB_POWER_DeepSleepModeHasOccurred (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepStatusClear (POWER_MODULE_ID index)

  Summary:
    Clears the Deep Sleep mode status bit of the device.

  Description:
    This function clears the Deep Sleep status bit of the device if it was previously
    in Deep Sleep mode. However, it does not mean that it wakes the device from Deep Sleep.
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	
  Returns:
    None.

  Example:
    <code>
    if(PLIB_POWER_DeepSleepModeHasOccurred(POWER_ID_0))
    {
	    PLIB_POWER_DeepSleepStatusClear (POWER_ID_0);
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepModeOccurrence in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepStatusClear (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_HLVDEnable (POWER_MODULE_ID index)

  Summary:
    Enables High/Low-Voltage Detection (HLVD) on VDD.

  Description:
    This function enables the High/Low-Voltage Detection (HLVD) module. 
	The HLVD module is used to detect high/low-voltage conditions on VDD.

  Precondition:
    Select Low-Voltage or High-Voltage Detection mode by calling 
	PLIB_POWER_HLVDModeSelect.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_HLVDEnable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsHLVDEnableControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_HLVDEnable (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_HLVDDisable (POWER_MODULE_ID index)

  Summary:
    Disables High/Low-Voltage Detection on VDD.

  Description:
    This function disables the HLVD module. 
	The HLVD module is used to detect high/low-voltage conditions on VDD.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_HLVDDisable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsHLVDEnableControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_HLVDDisable (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    bool PLIB_POWER_HLVDIsEnabled (POWER_MODULE_ID index)

  Summary:
    Returns the enable/disable status of High/Low-Voltage Detection on VDD.

  Description:
    This function returns whether or not High/Low-Voltage Detection on VDD is 
	enabled.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    - true  - HLVD on VDD is enabled
    - false - HLVD on VDD is not enabled	

  Example:
    <code>
    if(PLIB_POWER_HLVDIsEnabled(POWER_ID_0))
	{
		// HLVD is enabled.
	}
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsHLVDEnableControl in your application to determine whether
	this feature is available.
*/

bool PLIB_POWER_HLVDIsEnabled (POWER_MODULE_ID index);


//******************************************************************************
/* Function:
    void PLIB_POWER_HLVDStopInIdleEnable( POWER_MODULE_ID index )

  Summary:
    Discontinues HLVD operation when the device enters Idle mode.

  Description:
    This function discontinues HLVD operation when the device enters Idle mode.

  Precondition:
    None.
	
  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_HLVDStopInIdleEnable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_POWER_ExistsHLVDStopInIdleControl in your application to determine whether this
    feature is available.
 */

void PLIB_POWER_HLVDStopInIdleEnable( POWER_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_POWER_HLVDStopInIdleDisable( POWER_MODULE_ID index )

  Summary:
    Continues HLVD operation when the device enters Idle mode.

  Description:
    This function continues HLVD operation when the device enters Idle mode.

  Precondition:
    None.
	
  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_HLVDStopInIdleDisable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_POWER_ExistsHLVDStopInIdleControl in your application to determine whether this
    feature is available.
 */

void PLIB_POWER_HLVDStopInIdleDisable( POWER_MODULE_ID index );

//******************************************************************************
/* Function:
    bool PLIB_POWER_HLVDStopInIdleIsEnabled( POWER_MODULE_ID index )

  Summary:
    Returns the Stop in Idle mode status of the HLVD operation.

  Description:
    This function returns whether HLVD continues or discontinues operation 
	when the device enters Idle mode.

  Precondition:
    None.
	
  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true  - HLVD discontinues operation when the device enters Idle mode
    - false - HLVD continues operation when the device enters Idle mode

  Example:
    <code>
	bool sidl_status;
    sidl_status = PLIB_POWER_HLVDStopInIdleIsEnabled(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_POWER_ExistsHLVDStopInIdleControl in your application to determine whether this
    feature is available.
 */

bool PLIB_POWER_HLVDStopInIdleIsEnabled( POWER_MODULE_ID index );


//******************************************************************************
/*  Function:
    bool  PLIB_POWER_HLVDStatusGet(POWER_MODULE_ID index)

  Summary:
    Returns the HLVD event status.  
	
  Description:
    This function returns the HLVD event interrupt status.
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    - true  - HLVD event interrupt is active
    - false - HLVD event interrupt is not active

  Example:
    <code>
    if(PLIB_POWER_HLVDStatusGet(POWER_ID_0))
    {
	    //HLVD event has occurred.
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsHLVDStatus in your application to determine whether
	this feature is available.
*/

bool  PLIB_POWER_HLVDStatusGet(POWER_MODULE_ID index);


//******************************************************************************
/* Function:
    void PLIB_POWER_HLVDModeSelect (POWER_MODULE_ID index, HLVD_MODE mode)

  Summary:
    Selects the Voltage Detection mode.

  Description:
    This function selects either the High-Voltage or Low-Voltage Detection mode.
	
    In High-Voltage Detection (HVD) mode, an event occurs when the voltage equals 
    or exceeds the voltage limit.
	
    In Low-Voltage Detection (LVD) mode, an event occurs when the voltage equals 
    or falls below the voltage limit.

  Precondition:
    The HLVD module should be disabled by calling PLIB_POWER_HLVDDisable.

  Parameters:
    index   - Identifier for the device instance to be configured
    mode    - Voltage Detection mode, either HVD or LVD

  Returns:
    None.

  Example:
    <code>
	PLIB_POWER_HLVDModeSelect( POWER_ID_0, HLVD_MODE_HIGH_VOLTAGE_DETECTION );  
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsHLVDModeControl in your application to determine whether
	this feature is available.
   
 */

void PLIB_POWER_HLVDModeSelect (POWER_MODULE_ID index, HLVD_MODE mode);


//******************************************************************************
/*  Function:
    bool PLIB_POWER_HLVDBandGapVoltageIsStable (POWER_MODULE_ID index)

  Summary:
    Returns the status of Band Gap voltage.

  Description:
    This function returns whether the Band Gap Voltage is stable or not.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    - true  - Band Gap Voltage is stable
    - false - Band Gap Voltage is unstable	

  Example:
    <code>
    bool BGStatus;
	BGStatus = PLIB_POWER_HLVDBandGapVoltageIsStable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsHLVDBandGapVoltageStability in your application to determine whether
	this feature is available.
*/

bool PLIB_POWER_HLVDBandGapVoltageIsStable (POWER_MODULE_ID index);


//******************************************************************************
/* Function:
    void PLIB_POWER_HLVDLimitSelect (POWER_MODULE_ID index, HLVD_LIMIT limit)

  Summary:
    Selects the HLVD limit.

  Description:
    This function selects voltage limit for HLVD on VDD.
	
  Precondition:
    The HLVD module should be disabled by calling PLIB_POWER_HLVDDisable.

  Parameters:
    index   - Identifier for the device instance to be configured
    limit   - Voltage limit for HLVD on VDD

  Returns:
    None.

  Example:
    <code>
	PLIB_POWER_HLVDLimitSelect( POWER_ID_0, HLVD_LIMIT_ANALOG_INPUT_ON_HLVDIN );  
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsHLVDLimitSelection in your application to determine whether
	this feature is available.
   
 */

void PLIB_POWER_HLVDLimitSelect (POWER_MODULE_ID index, HLVD_LIMIT limit);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepModeEnable (POWER_MODULE_ID index)

  Summary:
    Enables Deep Sleep mode.

  Description:
    This function enables Deep Sleep mode. Deep Sleep mode is intended to provide 
    the lowest levels of power consumption available without requiring the use of 
    external switches to completely remove all power from the device.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepModeEnable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepMode in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepModeEnable (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepModeDisable (POWER_MODULE_ID index)

  Summary:
    Disables Deep Sleep mode.

  Description:
    This function disables Deep Sleep mode. Deep Sleep mode is intended to 
    provide the lowest levels of power consumption available without requiring 
    the use of external switches to completely remove all power from the device.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepModeDisable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepMode in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepModeDisable (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    bool PLIB_POWER_DeepSleepModeIsEnabled (POWER_MODULE_ID index)

  Summary:
    Returns the enable/disable status of Deep Sleep mode.

  Description:
    This function returns whether or not Deep Sleep mode is enabled.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    - true  - Deep Sleep mode is enabled
    - false - Deep Sleep mode is not enabled	

  Example:
    <code>
    if(PLIB_POWER_DeepSleepModeIsEnabled(POWER_ID_0))
	{
		//Enter Deep Sleep by executing Sleep mode.
	}
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepMode in your application to determine whether
	this feature is available.
*/

bool PLIB_POWER_DeepSleepModeIsEnabled (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepGPRsRetentionEnable (POWER_MODULE_ID index)

  Summary:
    Enables the General Purpose Registers retention.

  Description:
    This function enables the Deep Sleep General Purpose Registers to 
	retain their values when the device enters Deep Sleep/VBAT mode, 
	which means the power to DEEP_SLEEP_GPR_1 to DEEP_SLEEP_GPR_32 
	will be maintained. 
	Power to DEEP_SLEEP_GPR_0 will be retained by default.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepGPRsRetentionEnable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepGPRsRetentionControl in your application to 
	determine whether this feature is available.
*/

void PLIB_POWER_DeepSleepGPRsRetentionEnable (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepGPRsRetentionDisable (POWER_MODULE_ID index)

  Summary:
    Disables the General Purpose Registers retention.

  Description:
    This function disables the Deep Sleep General Purpose Registers to 
	retain their values when the device enters Deep Sleep/VBAT mode, 
	which means the power to DEEP_SLEEP_GPR_1 to DEEP_SLEEP_GPR_32 will be 
	dismissed. 
	Power to DEEP_SLEEP_GPR_0 will be retained by default.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepGPRsRetentionDisable(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepGPRsRetentionControl in your application to 
	determine whether this feature is available.
*/

void PLIB_POWER_DeepSleepGPRsRetentionDisable (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepModuleEnable(POWER_MODULE_ID index, DEEP_SLEEP_MODULE module)

  Summary:
    Enables the module in Deep Sleep mode.

  Description:
    This function enables the selected module in Deep Sleep mode.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	module   - Possible module from DEEP_SLEEP_MODULE 
	            
  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepModuleEnable(POWER_ID_0 , DEEP_SLEEP_MODULE_RTCC);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepModuleControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepModuleEnable(POWER_MODULE_ID index, DEEP_SLEEP_MODULE module);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepModuleDisable(POWER_MODULE_ID index, DEEP_SLEEP_MODULE module)

  Summary:
    Disables the module in Deep Sleep mode.

  Description:
    This function disables the selected module in Deep Sleep mode.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	module   - Possible module from DEEP_SLEEP_MODULE 
	            
  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepModuleDisable(POWER_ID_0 , DEEP_SLEEP_MODULE_RTCC);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepModuleControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepModuleDisable(POWER_MODULE_ID index, DEEP_SLEEP_MODULE module);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepWakeupEventEnable(POWER_MODULE_ID index, DEEP_SLEEP_WAKE_UP_EVENT event)

  Summary:
    Enables wake-up from the Deep Sleep mode by the selected event.

  Description:
    This function enables wake-up from the Deep Sleep mode by the selected event.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
    event    - Possible event from DEEP_SLEEP_WAKE_UP_EVENT
	
  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepWakeupEventEnable(POWER_ID_0 , DEEP_SLEEP_WAKE_UP_EVENT_RTCC);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepWakeupEventControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepWakeupEventEnable(POWER_MODULE_ID index, DEEP_SLEEP_WAKE_UP_EVENT event);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepWakeupEventDisable(POWER_MODULE_ID index, DEEP_SLEEP_WAKE_UP_EVENT event)

  Summary:
    Disables wake-up from Deep Sleep mode by the selected event.

  Description:
    This function disables wake-up from Deep Sleep mode by the selected event.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
    event    - Possible event from DEEP_SLEEP_WAKE_UP_EVENT
	
  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepWakeupEventDisable(POWER_ID_0 , DEEP_SLEEP_WAKE_UP_EVENT_RTCC);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepWakeupEventControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepWakeupEventDisable(POWER_MODULE_ID index, DEEP_SLEEP_WAKE_UP_EVENT event);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepPortPinsStateRetain (POWER_MODULE_ID index)

  Summary:
    Enables the I/O pins to retain their previous states upon wake-up from 
	Deep Sleep mode.

  Description:
    This function enables the I/O pins to retain their previous states upon 
	wake-up from Deep Sleep mode.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepPortPinsStateRetain(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepPortPinsStateControl in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepPortPinsStateRetain (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepPortPinsStateRelease (POWER_MODULE_ID index)

  Summary:
    Releases I/O pins upon wake-up from Deep Sleep mode.

  Description:
    This function releases the I/O pins upon wake-up from Deep Sleep mode.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    PLIB_POWER_DeepSleepPortPinsStateRelease(POWER_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepPortPinsStateControl in your application to 
	determine whether this feature is available.
*/

void PLIB_POWER_DeepSleepPortPinsStateRelease (POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    DEEP_SLEEP_EVENT  PLIB_POWER_DeepSleepEventStatusGet(POWER_MODULE_ID index)

  Summary:
    Returns the events that occurred during Deep Sleep mode.  
	
  Description:
    This function returns the events that occurred during Deep Sleep mode.
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured

  Returns:
    DEEP_SLEEP_EVENT - The Event that has occurred during Deep Sleep

  Example:
    <code>
    if((PLIB_POWER_DeepSleepEventStatusGet(POWER_ID_0)& 
	(DEEP_SLEEP_EVENT_RTCC_ALARM|DEEP_SLEEP_EVENT_BOR) != 0)
    {
	    //BOR or RTCC alarm event occurred in Deep Sleep Mode
	    //Therefore, take appropriate action
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepEventStatus in your application to determine whether
	this feature is available.
*/

DEEP_SLEEP_EVENT PLIB_POWER_DeepSleepEventStatusGet(POWER_MODULE_ID index);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepEventStatusClear(POWER_MODULE_ID index , DEEP_SLEEP_EVENT event)

  Summary:
    Clear any events that occurred during Deep Sleep mode.  
	
  Description:
    This function accept the events to clear that occurred during Deep Sleep mode.
	
  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
	event    - The event that occurred during the Deep Sleep

  Returns:
    None.

  Example:
    <code>
    if((PLIB_POWER_DeepSleepEventStatusGet(POWER_ID_0)& DEEP_SLEEP_EVENT_BOR) != 0)
    {
		PLIB_POWER_DeepSleepEventStatusClear(POWER_ID_0, DEEP_SLEEP_EVENT_BOR);
		//BOR event occurred in Deep Sleep Mode
		//Therefore, take appropriate action
		
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepEventStatus in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepEventStatusClear(POWER_MODULE_ID index , DEEP_SLEEP_EVENT event);


//******************************************************************************
/*  Function:
    void PLIB_POWER_DeepSleepGPRWrite(POWER_MODULE_ID index, DEEP_SLEEP_GPR gpr, uint32_t data)

  Summary:
    Writes 32-bit data into the Deep Sleep General Purpose Register.  
	
  Description:
    This function accepts a 32-bit value to write into the 
	Deep Sleep General Purpose Register. User can store content in these 
	registers (any application critical content) before entering 
	the Deep Sleep/VBAT modes and read them upon exit.
	
  Precondition:
    The content of the DEEP_SLEEP_GPR_0 register is retained even in the
	Deep Sleep and VBAT modes, but the DEEP_SLEEP_GPR_1 through DEEP_SLEEP_GPR_32 
	registers are disabled by default in Deep Sleep and VBAT modes. They can be enabled
    with PLIB_POWER_DeepSleepGPRsRetentionEnable.

  Parameters:
    index    - Identifier for the device instance to be configured
	gpr      - The available Deep Sleep General Purpose Register
	data     - 32-bit data to write into the Deep Sleep General Purpose Register	

  Returns:
    None.

  Example:
    <code>
	uint32_t data;
	//Enable power to DEEP_SLEEP_GPR_1 through DEEP_SLEEP_GPR_32 in Deep Sleep
	PLIB_POWER_DeepSleepGPRsRetentionEnable(POWER_ID_0);
	
	//Write 32-bit data into the DEEP_SLEEP_GPR_1
    PLIB_POWER_DeepSleepGPRWrite(POWER_ID_0, DEEP_SLEEP_GPR_1, 0x1234);
	
	//Enter the Deep Sleep mode and Exit
	//Now read the data from DEEP_SLEEP_GPR_1
	data = PLIB_POWER_DeepSleepGPRRead(POWER_ID_0, DEEP_SLEEP_GPR_1);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepGPROperation in your application to determine whether
	this feature is available.
*/

void PLIB_POWER_DeepSleepGPRWrite(POWER_MODULE_ID index, DEEP_SLEEP_GPR gpr, uint32_t data);


//******************************************************************************
/*  Function:
    uint32_t PLIB_POWER_DeepSleepGPRRead(POWER_MODULE_ID index, DEEP_SLEEP_GPR gpr)

  Summary:
    Reads 32-bit data from the Deep Sleep General Purpose Register.  
	
  Description:
    This function reads 32-bit value from the Deep Sleep General Purpose Register. 
	User can store content in these registers (any application critical content) 
	before entering the Deep Sleep/VBAT modes and read them upon exit.
	
  Precondition:
    The content of the DEEP_SLEEP_GPR_0 register is retained even in the
	Deep Sleep and VBAT modes, but the DEEP_SLEEP_GPR_1 through DEEP_SLEEP_GPR_32 
	registers are disabled by default in Deep Sleep and VBAT modes. They can be enabled
    with PLIB_POWER_DeepSleepGPRsRetentionEnable.

  Parameters:
    index    - Identifier for the device instance to be configured
	gpr      - The available Deep Sleep General Purpose Register	

  Returns:
	32-bit data from the selected Deep Sleep General Purpose Register.

  Example:
    <code>
	uint32_t data;
	//Enable power to DEEP_SLEEP_GPR_1 through DEEP_SLEEP_GPR_32 in Deep Sleep
	PLIB_POWER_DeepSleepGPRsRetentionEnable(POWER_ID_0);
	
	//Write 32-bit data into the DEEP_SLEEP_GPR_1
    PLIB_POWER_DeepSleepGPRWrite(POWER_ID_0, DEEP_SLEEP_GPR_1, 0x1234);
	
	//Enter the Deep Sleep mode and Exit
	//Now read the data from DEEP_SLEEP_GPR_1
	data = PLIB_POWER_DeepSleepGPRRead(POWER_ID_0, DEEP_SLEEP_GPR_1);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_POWER_ExistsDeepSleepGPROperation in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_POWER_DeepSleepGPRRead(POWER_MODULE_ID index, DEEP_SLEEP_GPR gpr);


// *****************************************************************************
// *****************************************************************************
// Section: Power Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************

/* The following functions indicate the existence of the features on the device. 
*/

//******************************************************************************
/* Function :  PLIB_POWER_ExistsPeripheralModuleControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the PeripheralModuleControl feature exists on the Power 
	module. 

  Description:
    This function identifies whether the PeripheralModuleControl feature is available 
	on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_PeripheralModuleDisable
    - PLIB_POWER_PeripheralModuleEnable
    - PLIB_POWER_PeripheralModuleIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PeripheralModuleControl feature is supported on the device
    - false  - The PeripheralModuleControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsPeripheralModuleControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsVoltageRegulatorControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the VoltageRegulatorControl feature exists on the Power 
	module. 

  Description:
    This function identifies whether the VoltageRegulatorControl feature is available 
	on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_VoltageRegulatorEnable
    - PLIB_POWER_VoltageRegulatorDisable
    - PLIB_POWER_VoltageRegulatorIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The VoltageRegulatorControl feature is supported on the device
    - false  - The VoltageRegulatorControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsVoltageRegulatorControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsSleepStatus( POWER_MODULE_ID index )

  Summary:
    Identifies whether the SleepStatus feature exists on the Power module. 

  Description:
    This function identifies whether the SleepStatus feature is available on 
	the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeviceWasInSleepMode
    - PLIB_POWER_ClearSleepStatus

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The SleepStatus feature is supported on the device
    - false  - The SleepStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsSleepStatus( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsIdleStatus( POWER_MODULE_ID index )

  Summary:
    Identifies whether the IdleStatus feature exists on the Power module. 

  Description:
    This function identifies whether the IdleStatus feature is available on the 
	Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeviceWasInIdleMode
    - PLIB_POWER_ClearIdleStatus

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The IdleStatus feature is supported on the device
    - false  - The IdleStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsIdleStatus( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsHighVoltageOnVDD1V8( POWER_MODULE_ID index )

  Summary:
    Identifies whether the HighVoltageOnVDD1V8 feature exists on the Power module. 

  Description:
    This function identifies whether the HighVoltageOnVDD1V8 feature is available 
	on the Power module.
    When this function returns true, this function is supported on the device: 
    - PLIB_POWER_HighVoltageOnVDD1V8HasOccurred

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HighVoltageOnVDD1V8 feature is supported on the device
    - false  - The HighVoltageOnVDD1V8 feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsHighVoltageOnVDD1V8( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsDeepSleepModeOccurrence( POWER_MODULE_ID index )

  Summary:
    Identifies whether the DeepSleepModeOccurrence feature exists on the Power module. 

  Description:
    This function identifies whether the DeepSleepModeOccurrence feature is available 
	on the Power module.
    When this function returns true, this function is supported on the device: 
    - PLIB_POWER_DeepSleepModeHasOccurred
	- PLIB_POWER_DeepSleepStatusClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeepSleepModeOccurrence feature is supported on the device
    - false  - The DeepSleepModeOccurrence feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsDeepSleepModeOccurrence( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsHLVDEnableControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the HLVDEnableControl feature exists on the Power module. 

  Description:
    This function identifies whether the HLVDEnableControl feature is available 
	on the Power module.
    When this function returns true, this function is supported on the device: 
    - PLIB_POWER_HLVDEnable
    - PLIB_POWER_HLVDDisable
	- PLIB_POWER_HLVDIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HLVDEnableControl feature is supported on the device
    - false  - The HLVDEnableControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsHLVDEnableControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsHLVDStopInIdleControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the HLVDStopInIdleControl feature exists on the Power module. 

  Description:
    This function identifies whether the HLVDStopInIdleControl feature is available 
	on the Power module.
    When this function returns true, this function is supported on the device: 
    - PLIB_POWER_HLVDStopInIdleEnable
    - PLIB_POWER_HLVDStopInIdleDisable
	- PLIB_POWER_HLVDStopInIdleIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HLVDStopInIdleControl feature is supported on the device
    - false  - The HLVDStopInIdleControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsHLVDStopInIdleControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsHLVDStatus( POWER_MODULE_ID index )

  Summary:
    Identifies whether the HLVDStatus feature exists on the Power module. 

  Description:
    This function identifies whether the HLVDStatus feature is available 
	on the Power module.
    When this function returns true, this function is supported on the device: 
    - PLIB_POWER_HLVDStatusGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HLVDStatus feature is supported on the device
    - false  - The HLVDStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsHLVDStatus( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsHLVDModeControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the HLVDModeControl feature exists on the Power module. 

  Description:
    This function identifies whether the HLVDModeControl feature is available 
	on the Power module.
    When this function returns true, this function is supported on the device: 
    - PLIB_POWER_HLVDModeSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HLVDModeControl feature is supported on the device
    - false  - The HLVDModeControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsHLVDModeControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsHLVDBandGapVoltageStability( POWER_MODULE_ID index )

  Summary:
    Identifies whether the HLVDBandGapVoltageStability feature exists on the Power module. 

  Description:
    This function identifies whether the HLVDBandGapVoltageStability feature is available 
	on the Power module.
    When this function returns true, this function is supported on the device: 
    - PLIB_POWER_HLVDBandGapVoltageIsStable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HLVDBandGapVoltageStability feature is supported on the device
    - false  - The HLVDBandGapVoltageStability feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsHLVDBandGapVoltageStability( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsHLVDLimitSelection( POWER_MODULE_ID index )

  Summary:
    Identifies whether the HLVDLimitSelection feature exists on the Power module. 

  Description:
    This function identifies whether the HLVDLimitSelection feature is available 
	on the Power module.
    When this function returns true, this function is supported on the device: 
    - PLIB_POWER_HLVDLimitSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HLVDLimitSelection feature is supported on the device
    - false  - The HLVDLimitSelection feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsHLVDLimitSelection( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsDeepSleepMode( POWER_MODULE_ID index )

  Summary:
    Identifies whether the DeepSleepModeControl feature exists on the Power module. 

  Description:
    This function identifies whether the DeepSleepModeControl feature is available 
	on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeepSleepModeEnable
    - PLIB_POWER_DeepSleepModeDisable
	- PLIB_POWER_DeepSleepModeIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeepSleepModeControl feature is supported on the device
    - false  - The DeepSleepModeControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsDeepSleepMode( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsDeepSleepGPRsRetentionControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the DeepSleepGPRsRetentionControl feature exists on the 
	Power module. 

  Description:
    This function identifies whether the DeepSleepGPRsRetentionControl feature is 
	available on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeepSleepGPRsRetentionEnable
    - PLIB_POWER_DeepSleepGPRsRetentionDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeepSleepGPRsRetentionControl feature is supported on the device
    - false  - The DeepSleepGPRsRetentionControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsDeepSleepGPRsRetentionControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsDeepSleepModuleControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the DeepSleepModuleControl feature exists on the 
	Power module. 

  Description:
    This function identifies whether the DeepSleepModuleControl feature is available 
	on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeepSleepModuleEnable
    - PLIB_POWER_DeepSleepModuleDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeepSleepModuleControl feature is supported on the device
    - false  - The DeepSleepModuleControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsDeepSleepModuleControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsDeepSleepWakeupEventControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the DeepSleepWakeupEventControl feature exists on the Power 
	module. 

  Description:
    This function identifies whether the DeepSleepWakeupEventControl feature is 
	available on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeepSleepWakeupEventEnable
    - PLIB_POWER_DeepSleepWakeupEventDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeepSleepWakeupEventControl feature is supported on the device
    - false  - The DeepSleepWakeupEventControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsDeepSleepWakeupEventControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsDeepSleepPortPinsStateControl( POWER_MODULE_ID index )

  Summary:
    Identifies whether the DeepSleepPortPinsStateControl feature exists on the 
	Power module.

  Description:
    This function identifies whether the DeepSleepPortPinsStateControl feature is available 
	on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeepSleepPortPinsStateRetain
    - PLIB_POWER_DeepSleepPortPinsStateRelease

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeepSleepPortPinsStateControl feature is supported on the device
    - false  - The DeepSleepPortPinsStateControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsDeepSleepPortPinsStateControl( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsDeepSleepEventStatus( POWER_MODULE_ID index )

  Summary:
    Identifies whether the DeepSleepEventStatus feature exists on the Power module. 

  Description:
    This function identifies whether the DeepSleepEventStatus feature is available 
	on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeepSleepEventStatusGet
    - PLIB_POWER_DeepSleepEventStatusClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeepSleepEventStatus feature is supported on the device
    - false  - The DeepSleepEventStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsDeepSleepEventStatus( POWER_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_POWER_ExistsDeepSleepGPROperation( POWER_MODULE_ID index )

  Summary:
    Identifies whether the DeepSleepGPROperation feature exists on the Power module. 

  Description:
    This function identifies whether the DeepSleepGPROperation feature is available 
	on the Power module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_POWER_DeepSleepGPRWrite
    - PLIB_POWER_DeepSleepGPRRead

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeepSleepGPROperation feature is supported on the device
    - false  - The DeepSleepGPROperation feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_POWER_ExistsDeepSleepGPROperation( POWER_MODULE_ID index );


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif //#ifndef _PLIB_POWER_H
/*******************************************************************************
 End of File
*/