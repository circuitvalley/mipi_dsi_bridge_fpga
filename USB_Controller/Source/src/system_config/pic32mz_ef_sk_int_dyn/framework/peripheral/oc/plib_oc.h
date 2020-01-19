/*******************************************************************************
  Output Compare Module Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_oc.h

  Summary:
    This file contains the interface definition for the Output Compare Peripheral 
	Library.

  Description:
    This library provides a low-level abstraction of the Output Compare module
    on Microchip microcontrollers with a convenient C language interface.
    It can be used to simplify low-level access to the module without the necessity
    of interacting directly with the module's registers, thus hiding differences
    between one microcontroller variant and another.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2013-2015 released Microchip Technology Inc. All rights reserved.

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

#ifndef _PLIB_OC_H_    // Guards against multiple inclusion
#define _PLIB_OC_H_

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.
*/
#include "processor/oc_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/*  Function:
   void PLIB_OC_Enable( OC_MODULE_ID index )

  Summary:
   Enables the Output Compare module.

  Description:
    This function enables the Output Compare module.

  Precondition:
    The module should be appropriately configured before being enabled.

  Parameters:
    index      - Identifies the desired Output Compare module

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1

    //Do all the other configurations before enabling.

    PLIB_OC_Enable(MY_OC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsEnableControl in your application to determine
    whether this feature is available.
*/

void PLIB_OC_Enable( OC_MODULE_ID index );

//*******************************************************************************
/*  Function:
   bool PLIB_OC_IsEnabled( OC_MODULE_ID index )

  Summary:
    Checks whether the Output Compare module is enabled or not.

  Description:
    The function returns the enable status of the Output Compare module.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired Output Compare module

  Returns:
    Boolean
	- true  - The Output Compare module is enabled
    - false - The Output Compare module is not enabled

  Example:
    <code>
	
    #define MY_OC_ID OC_ID_1
	
    if(PLIB_OC_IsEnabled(MY_OC_ID))
	{
	//Take respective actions
	}
	else
	{
	//Take respective actions
	}
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsEnableControl in your application to determine
    whether this feature is available.
*/

bool PLIB_OC_IsEnabled( OC_MODULE_ID index );

//*******************************************************************************
/*  Function:
   void PLIB_OC_Disable( OC_MODULE_ID index )

  Summary:
    Disable the Output Compare module.

  Description:
    This function disables the Output Compare module.

  Precondition:
    None.

  Parameters:
    index       - Identifies the Output Compare module

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1

    PLIB_OC_Disable(MY_OC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsEnableControl in your application to determine
    whether this feature is available.
*/

void PLIB_OC_Disable( OC_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_OC_StopInIdleEnable( OC_MODULE_ID index )

  Summary:
    Discontinues Output Compare module operation when the device enters Idle mode.

  Description:
    This function discontinues Output Compare module operation when the device 
	enters Idle mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired Output Compare module

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1

    PLIB_OC_StopInIdleEnable(MY_OC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsStopInIdle in your application to determine
    whether this feature is available.
*/

void PLIB_OC_StopInIdleEnable( OC_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_OC_StopInIdleDisable( OC_MODULE_ID index )

  Summary:
    Output Compare module continues operating when the device enters Idle mode.

  Description:
    The function continues Output Compare module operation when the device enters 
	Idle mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired Output Compare module

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1

    PLIB_OC_StopInIdleDisable(MY_OC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsStopInIdle in your application to determine
    whether this feature is available.
*/

void PLIB_OC_StopInIdleDisable( OC_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_OC_Buffer32BitSet( OC_MODULE_ID index, uint32_t val32Bit)

  Summary:
    Sets a 32-bit primary compare value for compare operations.

  Description:
    This function sets a 32-bit primary compare value for compare operations in
    all modes except PWM modes.

  Precondition:
    The PWM mode of operation should not be selected. The buffer size should be 
	set to 32-bits by the PLIB_OC_BufferSizeSelect function.

  Parameters:
    index           - Identifies the desired Output Compare module
    val32Bit        - Sets a 32-bit primary compare value

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1

    PLIB_OC_Buffer32BitSet(MY_OC_ID, 0x000000FF);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsBufferValue in your application to determine
    whether this feature is available.
*/

void PLIB_OC_Buffer32BitSet( OC_MODULE_ID index, uint32_t val32Bit);


   // *****************************************************************************
/* Function:
    void PLIB_OC_Buffer16BitSet( OC_MODULE_ID index, uint16_t val16Bit)

  Summary:
    Sets a 16-bit primary compare value for compare operations.

  Description:
    This function sets a 16-bit primary compare value for compare operations in all
    modes except PWM modes.

  Precondition:
    The PWM mode of operation should not be selected. The buffer size should be set
    to 16-bits by the PLIB_OC_BufferSizeSelect function.

  Parameters:
    index           - Identifies the desired Output Compare module
    val16Bit        - Sets a 16-bit primary compare value
	
  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1

    PLIB_OC_Buffer16BitSet(MY_OC_ID, 0x00FF);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsBufferValue in your application to determine
    whether this feature is available.
*/

void PLIB_OC_Buffer16BitSet( OC_MODULE_ID index, uint16_t val16Bit);


//*****************************************************************************
/* Function:
    void PLIB_OC_PulseWidth32BitSet( OC_MODULE_ID index,uint32_t pulseWidth)

  Summary:
    Sets a 32-bit pulse width for Output Compare module output.

  Description:
    This function sets a 32-bit pulse width for Output Compare module in dual 
	compare modes. A dual compare mode can be selected using PLIB_OC_ModeSelect 
	function. Secondary compare match event (pulse width) decides the trailing 
	(falling) edge of the Output Compare module output.

  Precondition:
    Dual compare operation should be selected. The buffer size should be set to
    32-bits by the PLIB_OC_BufferSizeSelect function.

  Parameters:
    index           - Identifies the desired Output Compare module
    pulseWidth      - Pulse width value

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1

    PLIB_OC_PulseWidth32BitSet(MY_OC_ID, 0x00000FFF);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsPulseWidth in your application to determine
    whether this feature is available.
*/

void PLIB_OC_PulseWidth32BitSet( OC_MODULE_ID index, uint32_t pulseWidth);


//*****************************************************************************
/* Function:
    void PLIB_OC_PulseWidth16BitSet( OC_MODULE_ID index,uint16_t pulseWidth)

  Summary:
    Sets a 16-bit pulse width for Output Compare module output.

  Description:
    This function sets a 16-bit pulse width for the Output Compare module in 
	dual compare modes. A dual compare mode can be selected using the 
	PLIB_OC_ModeSelect function. Secondary compare match event (pulse width) 
	decides the trailing (falling) edge of the Output Compare module output.

  Precondition:
    Dual compare operation should be selected. The buffer size should be set to
    16-bits by the PLIB_OC_BufferSizeSelect function.

  Parameters:
    index           - Identifies the desired Output Compare module
    pulseWidth      - Pulse width value

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1

    PLIB_OC_PulseWidth16BitSet(MY_OC_ID, 0x0FFF);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsPulseWidth in your application to determine
    whether this feature is available.
*/

void PLIB_OC_PulseWidth16BitSet( OC_MODULE_ID index, uint16_t pulseWidth);


// *****************************************************************************
/* Function:
    void PLIB_OC_TimerSelect( OC_MODULE_ID index, OC_16BIT_TIMERS tmr )

  Summary:
    Selects a clock source for the Output Compare module.

  Description:
    This function selects a clock source for the Output Compare module if the 16-bit
    time base is set.

  Precondition:
    The 16-bit time base needs to be set.

  Parameters:
    index      - Identifies the desired Output Compare module
    tmr        - Identifies the timer

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1
    // 16-bit Timer2 is selected
    PLIB_OC_TimerSelect(MY_OC_ID, OC_TIMER_16BIT_TMR2);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsTimerSelect in your application to determine
    whether this feature is available.
*/
void PLIB_OC_TimerSelect( OC_MODULE_ID index, OC_16BIT_TIMERS tmr );


// *****************************************************************************
/* Function:
    void PLIB_OC_AlternateClockEnable(OC_MODULE_ID index)

  Summary:
    Selects the alternate clock source.

  Description:
    This function selects the alternate clock source instead of Timer2/Timer3.

  Precondition:
    A system unlock PLIB_DEVCON_SystemUnlock must be 
    performed before this function can be executed.

  Parameters:
    index      - Identifies the desired Output Compare module

  Returns:
    None.

  Example:
  <code>
    // Call system service to unlock oscillator
    #define MY_OC_ID OC_ID_1
    PLIB_OC_AlternateClockEnable( MY_OC_ID );
  </code>

  Remarks:
    The feature is not supported on all devices.  Please refer to the
    specific device data sheet to determine availability.
    A system unlock must be performed before this function can be executed.
    This function applies to all input capture modules, regardless of the
    OC_MODULE_ID passed in the call.
*/

void PLIB_OC_AlternateClockEnable(OC_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_OC_AlternateClockDisable(OC_MODULE_ID index)

  Summary:
    Selects Timer 2/3, instead of the alternate clock source.

  Description:
    This function selects Timer2/Timer3, instead of the alternate clock source.

  Precondition:
    A system unlock PLIB_DEVCON_SystemUnlock must be 
    performed before this function can be executed.

  Parameters:
    index      - Identifies the desired Output Compare module

  Returns:
    None.

  Example:
  <code>
    // Call system service to unlock oscillator
    #define MY_OC_ID OC_ID_1
    PLIB_OC_AlternateClockDisable( MY_OC_ID );
  </code>

  Remarks:
    The feature is not supported on all devices. Please refer to the
    specific device data sheet to determine availability.
    A system unlock must be performed before this function can be executed.
    This function applies to all input capture modules, regardless of the
    OC_MODULE_ID passed in the call.
*/

void PLIB_OC_AlternateClockDisable(OC_MODULE_ID index);


// *****************************************************************************
/* Function:
    bool PLIB_OC_AlternateTimerSelect( OC_MODULE_ID index, OC_ALT_TIMERS tmr )

  Summary:
    Selects an alternate timer as a clock source for the Output Compare module.

  Description:
    This function selects an alternate timer as a clock source for the Output 
    Compare module.
    * OC_ID_1,OC_ID_2,OC_ID_3: Can use Timer4 or Timer5 as alternate timers
    * OC_ID_4,OC_ID_5,OC_ID_6,OC_ID_13,OC_ID_14,OC_ID_15,OC_ID_16: Can use 
                                       Timer2 or Timer3 as alternate timers
    * OC_ID_7,OC_ID_8,OC_ID_9: Can use Timer6 or Timer7 as alternate timers
    * OC_ID_10,OC_ID_11,OC_ID_12: Can use Timer8 or Timer9 as alternate timers

  Precondition:
    The 16-bit time base needs to be set. The PLIB_OC_AlternateClockEnable 
    function should be called for the Output Compare module to enable the 
    alternate clock selection.

  Parameters:
    index      - Identifies the desired Output Compare module
    tmr        - Identifies the alternate timer

  Returns:        
     - true  - Alternate timer selected successfully
     - false - Alternate timer selection failure, select an appropriate alternate 
               timer for the Output Compare module index

  Example:
    <code>
    #define MY_OC_ID   OC_ID_1
    bool result;

    //Enabling alternate timer selection
    PLIB_OC_AlternateClockEnable( MY_OC_ID );

    // 16-bit Timer4 is selected as the clock source for Output Compare module 1
    result = PLIB_OC_AlternateTimerSelect(MY_OC_ID, OC_ALT_TIMER_TMR4);

    if(false == result)
    {
        // Selected alternate timer does not available for the desired Output 
        // Compare module.
        // Select the appropriate alternate timer.
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsAlternateTimerSelect in your application to determine
    whether this feature is available.
*/
bool PLIB_OC_AlternateTimerSelect( OC_MODULE_ID index, OC_ALT_TIMERS tmr );


//*****************************************************************************
/* Function:
    void PLIB_OC_FaultInputSelect(OC_MODULE_ID index, OC_FAULTS flt)

  Summary:
    Enables/Disables the Fault input for the Output Compare PWM mode.

  Description:
    This function enables/disables the Fault input for the Output Compare PWM mode.

    If some other mode was selected using PLIB_OC_ModeSelect, the mode selected by 
    PLIB_OC_ModeSelect will be overwritten as PLIB_OC_FaultInputSelect selects PWM 
    mode with/without Fault protection.

    Fault input is valid if the fault pin is enabled in the hardware.
    If a logic '0' is detected on the OCFA/OCFB pin, the selected PWM output pin(s) 
    are placed in the tri-state. The user may elect to provide a pull-down or pull-up 
    resistor on the PWM pin to provide for a desired state if a Fault condition occurs. 
    The shutdown of the PWM output is immediate and is not tied to the device clock 
    source. Fault occurrence can be detected by calling the function PLIB_OC_FaultHasOccurred. 
    The Output Compare module will be disabled until the following conditions are met:
    • The external Fault condition has been removed
    • The PWM mode is re-enabled

  Precondition:
    Fault pin: OCFA for OC_ID_1 to OC_ID_4 , OCFB for OC_ID_5 in MX devices.  
               OCFA for OC_ID_1 to OC_ID_3 and OC_ID_7 to OC_ID_9 , 
               OCFB for OC_ID_4 to OC_ID_6 in MZ devices. 
               should be enabled in the hardware if enabling the fault input, 
               that is if selecting the OC_FAULT_PRESET.

  Parameters:
    index      - Identifies the desired Output Compare module
    flt        - Identifies the Output Compare module Fault input

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1
    // Fault pin is enabled in the hardware
    // This function selects PWM with fault protection mode for MY_OC_ID instance.
    PLIB_OC_FaultInputSelect(MY_OC_ID, OC_FAULT_PRESET);
    // Output Compare fault input is now enabled for Output Compare Module
    </code>

  Remarks:
    This function selects the PWM mode of the Output Compare module with Fault 
    protection or without Fault protection.
    
    These modes can be selected using PLIB_OC_ModeSelect also.
    
    If any other Output Compare mode is selected prior to this function, that mode 
    will be overwritten as this feature is available for PWM mode only.
    
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsFaultInput in your application to determine
    whether this feature is available.
*/

void PLIB_OC_FaultInputSelect(OC_MODULE_ID index, OC_FAULTS flt);


// *****************************************************************************
/* Function:
    void PLIB_OC_BufferSizeSelect( OC_MODULE_ID index, OC_BUFFER_SIZE size )

  Summary:
    Sets the buffer size and pulse width to 16-bits or 32-bits.

  Description:
    This function sets the size of the buffer and pulse width to 16-bits or 32-bits.
    The choice is made based on whether a 16-bit timer or a 32-bit timer is selected.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired Output Compare module
    size       - Identifies the size of compare value

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1
    // Buffer size and pulse width size are set to 32-bits
    PLIB_OC_BufferSizeSelect(MY_OC_ID, OC_BUFFER_SIZE_32BIT);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsBufferSize in your application to determine
    whether this feature is available.
*/

void PLIB_OC_BufferSizeSelect( OC_MODULE_ID index, OC_BUFFER_SIZE size );


// *****************************************************************************
/* Function:
    void PLIB_OC_ModeSelect( OC_MODULE_ID index, OC_COMPARE_MODES cmpMode )

  Summary:
    Selects the compare mode for the Output Compare module.

  Description:
    This function selects the compare mode for the Output Compare module.

  Precondition:
    The Output Compare module must be turned off before a new mode is selected. 
	The Output Compare module is turned off through the
	PLIB_OC_ModeSelect(MY_OC_ID,OC_COMPARE_TURN_OFF_MODE) function.
	Refer to the enumeration description for information on different modes and 
	preconditions.

  Parameters:
    index      - Identifies the desired Output Compare module
    cmpMode    - Identifies the compare mode for Output Compare module

  Returns:
    None.

  Example:
    <code>
    #define MY_OC_ID OC_ID_1
    // Dual compare continuous pulse mode is selected
    PLIB_OC_ModeSelect(MY_OC_ID, OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE );
    </code>

  Remarks:
    If PLIB_OC_FaultInputSelect is called after PLIB_OC_ModeSelect, the mode 
	selected by the PLIB_OC_ModeSelect function will be overwritten as the
	PLIB_OC_FaultInputSelect function selects the PWM mode with or without Fault 
	protection.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsCompareModeSelect in your application to determine
    whether this feature is available.
*/

void PLIB_OC_ModeSelect( OC_MODULE_ID index, OC_COMPARE_MODES cmpMode );


// *****************************************************************************
/* Function:
   bool PLIB_OC_FaultHasOccurred(OC_MODULE_ID index)

  Summary:
   Checks if a PWM fault has occurred.

  Description:
   This function returns 'true' if a PWM Fault has occurred and 'false' if no
   Fault condition exists.

  Precondition:
   This function should be used only in Edge or Center-Aligned PWM mode
   set by the PLIB_OC_ModeSelect() function.

  Parameters:
   index        - Identifies the desired Output Compare module

  Returns:
    - true  - PWM Fault has occurred
    - false - No PWM fault has occurred

  Example:
  <code>
    #define MY_OC_ID OC_ID_1
    if(!PLIB_OC_FaultHasOccurred(MY_OC_ID))
    {
        // Do some operation
    };
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_OC_ExistsFaultStatus in your application to determine
    whether this feature is available.

*/

bool PLIB_OC_FaultHasOccurred(OC_MODULE_ID index);


// *****************************************************************************
// *****************************************************************************
// Section: Output Compare Peripheral Library Exists API Functions
// *****************************************************************************
// *****************************************************************************
/* The functions below indicate the existence of the features on the device.
*/

//******************************************************************************
//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsEnableControl( OC_MODULE_ID index )

  Summary:
    Identifies whether the EnableControl feature exists on the Output Compare 
	module.

  Description:
    This function identifies whether the EnableControl feature is available on the 
	Output Compare module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OC_Enable
    - PLIB_OC_Disable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the EnableControl feature is supported on the device
    - false  - If the EnableControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsEnableControl( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsStopInIdle( OC_MODULE_ID index )

  Summary:
    Identifies whether the StopInIdle feature exists on the Output Compare 
	module.

  Description:
    This function identifies whether the StopInIdle feature is available on the 
	Output Compare module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OC_StopInIdleEnable
    - PLIB_OC_StopInIdleDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the StopInIdle feature is supported on the device
    - false  - If the StopInIdle feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsStopInIdle( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsFaultInput( OC_MODULE_ID index )

  Summary:
    Identifies whether the FaultInput feature exists on the Output Compare module.

  Description:
    This function identifies whether the FaultInput feature is available on the 
	Output Compare module.
    When this function returns true, this function is supported on the device:
    - PLIB_OC_FaultInputSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the FaultInput feature is supported on the device
    - false  - If the FaultInput feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsFaultInput( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsFaultStatus( OC_MODULE_ID index )

  Summary:
    Identifies whether the FaultStatus feature exists on the Output Compare module.

  Description:
    This function identifies whether the FaultStatus feature is available on the 
	Output Compare module.
    When this function returns true, this function is supported on the device:
    - PLIB_OC_FaultHasOccurred

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the FaultStatus feature is supported on the device
    - false  - If the FaultStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsFaultStatus( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsTimerSelect( OC_MODULE_ID index )

  Summary:
    Identifies whether the TimerSelect feature exists on the Output Compare 
	module.

  Description:
    This function identifies whether the TimerSelect feature is available on the 
	Output Compare module.
    When this function returns true, this function is supported on the device:
    - PLIB_OC_TimerSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the TimerSelect feature is supported on the device
    - false  - If the TimerSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsTimerSelect( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_OC_ExistsAlternateClock( OC_MODULE_ID index )

  Summary:
    Identifies whether the AlternateClock feature exists on the Output Compare 
	module.

  Description:
    This function identifies whether the AlternateClock feature is available on 
	the Output Compare module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OC_AlternateClockEnable
    - PLIB_OC_AlternateClockDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AlternateClock feature is supported on the device
    - false  - The AlternateClock feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsAlternateClock( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsAlternateTimerSelect( OC_MODULE_ID index )

  Summary:
    Identifies whether the AlternateTimerSelect feature exists on the Output 
	Compare module.

  Description:
    This function identifies whether the AlternateTimerSelect feature is available 
	on the Output Compare module.
    When this function returns true, this function is supported on the device:
    - PLIB_OC_AlternateTimerSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the AlternateTimerSelect feature is supported on the device
    - false  - If the AlternateTimerSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsAlternateTimerSelect( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsBufferValue( OC_MODULE_ID index )

  Summary:
    Identifies whether the BufferValue feature exists on the Output Compare 
	module.

  Description:
    This function identifies whether the BufferValue feature is available on the 
	Output Compare module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OC_Buffer32BitSet
    - PLIB_OC_Buffer16BitSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the BufferValue feature is supported on the device
    - false  - If the BufferValue feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsBufferValue( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsPulseWidth( OC_MODULE_ID index )

  Summary:
    Identifies whether the PulseWidth feature exists on the Output Compare module.

  Description:
    This function identifies whether the PulseWidth feature is available on the 
	Output Compare module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OC_PulseWidth32BitSet
    - PLIB_OC_PulseWidth16BitSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PulseWidth feature is supported on the device
    - false  - If the PulseWidth feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsPulseWidth( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsBufferSize( OC_MODULE_ID index )

  Summary:
    Identifies whether the BufferSize feature exists on the Output Compare module.

  Description:
    This function identifies whether the BufferSize feature is available on the 
	Output Compare module.
    When this function returns true, this function is supported on the device:
    - PLIB_OC_BufferSizeSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the BufferSize feature is supported on the device
    - false  - If the BufferSize feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsBufferSize( OC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OC_ExistsCompareModeSelect( OC_MODULE_ID index )

  Summary:
    Identifies whether the CompareModeSelect feature exists on the Output Compare 
	module.

  Description:
    This function identifies whether the CompareModeSelect feature is available 
	on the Output Compare module.
    When this function returns true, this function is supported on the device:
    - PLIB_OC_ModeSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the CompareModeSelect feature is supported on the device
    - false  - If the CompareModeSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OC_ExistsCompareModeSelect( OC_MODULE_ID index );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif
/*******************************************************************************
 End of File
*/
