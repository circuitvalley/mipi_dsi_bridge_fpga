/*******************************************************************************
  IC Module Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_ic.h

  Summary:
    This file contains the interface definition for the Input Capture (IC)
    peripheral library.

  Description:
    This library provides a low-level abstraction of the Input Capture
    (IC) module on Microchip microcontrollers with a convenient C language interface.
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

#ifndef _HELP_HARMONY_TEMPLATE_H_    // Guards against multiple inclusion
#define _HELP_HARMONY_TEMPLATE_H_

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

#include "peripheral/ic/processor/ic_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/* Function:
   void PLIB_IC_Enable( IC_MODULE_ID index )

  Summary:
   Enables the IC module.

  Description:
    This function enables the IC module.

  Precondition:
    The module should be appropriately configured before being enabled.

  Parameters:
    index      - Identifies the desired IC module

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1

    //Do all the other configurations before enabling.

    PLIB_IC_Enable(MY_IC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsEnable in your application to determine
    whether this feature is available.
*******************************************************************************/

void PLIB_IC_Enable(IC_MODULE_ID index);


//*******************************************************************************
/* Function:
   void PLIB_IC_Disable( IC_MODULE_ID index )

  Summary:
   Disables the IC module.

  Description:
    This function disables the IC module.

  Precondition:
    None.

  Parameters:
    index       - Identifies the IC module

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1

    PLIB_IC_Disable(MY_IC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsEnable in your application to determine
    whether this feature is available.
*******************************************************************************/

void PLIB_IC_Disable( IC_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_IC_StopInIdleEnable( IC_MODULE_ID index )

  Summary:
    Discontinues IC module operation when the device enters Idle mode.

  Description:
    This function discontinues IC module operation when the device enters Idle mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired IC module

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1

    PLIB_IC_StopInIdleEnable(MY_IC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsStopInIdle in your application to determine
    whether this feature is available.
*/

 void PLIB_IC_StopInIdleEnable( IC_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_IC_StopInIdleDisable( IC_MODULE_ID index )

  Summary:
    Continues module operation when the device enters Idle mode

  Description:
    The function continues operation of the IC module when the device enters Idle mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired IC module

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1

    PLIB_IC_StopInIdleDisable(MY_IC_ID);
    </code>


  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsStopInIdle in your application to determine
    whether this feature is available.
*/

 void PLIB_IC_StopInIdleDisable( IC_MODULE_ID index);


  // *****************************************************************************
/* Function:

    void PLIB_IC_FirstCaptureEdgeSelect( IC_MODULE_ID index, IC_EDGE_TYPES edgeType )

  Summary:
    Captures the timer count value at the first selected edge of input signal.

  Description:
    This function captures the timer count value at the first selected edge of the input signal.

  Precondition:
    This setting applies only when the Every Edge Capture mode is set. The capture mode is set by
    the PLIB_IC_ModeSelect function.

  Parameters:
    index      - Identifies the desired IC module
    edgeType   - Identifies the edge type (i.e., whether rising or falling edge)

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1

    PLIB_IC_FirstCaptureEdgeSelect(MY_IC_ID,IC_EDGE_FALLING);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsEdgeCapture in your application to determine
    whether this feature is available.
*/

 void PLIB_IC_FirstCaptureEdgeSelect( IC_MODULE_ID index, IC_EDGE_TYPES edgeType );


   // *****************************************************************************
/* Function:
    uint16_t PLIB_IC_Buffer16BitGet( IC_MODULE_ID index )

  Summary:
    Obtains the 16-bit input capture buffer value.

  Description:
    This function reads the 16-bit value of the IC buffer to obtain the captured timer value.
    This function is used when the buffer size is set to 16-bits.

  Precondition:
    The buffer size should be set to 16 bits (i.e., PLIB_IC_BufferSizeSet(MY_IC_ID, IC_BUFFER_SIZE_16BIT).

  Parameters:
    index      - Identifies the desired IC module

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1
    uint16_t bufferVal;
    // Read input capture buffer value and store it in 'bufferVal'
    bufferVal = PLIB_IC_Buffer16BitGet(MY_IC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsBufferValue in your application to determine
    whether this feature is available.
*/

 uint16_t PLIB_IC_Buffer16BitGet( IC_MODULE_ID index );


    // *****************************************************************************
/* Function:
    uint32_t PLIB_IC_Buffer32BitGet( IC_MODULE_ID index )

  Summary:
    Obtains the 32-bit input capture buffer value.

  Description:
    This function reads the 32-bit value of the IC buffer to obtain the captured timer value.
    This function is used when the buffer size is set to 32 bits.

  Precondition:
    The buffer size should be set to 32 bits (i.e., PLIB_IC_BufferSizeSet(MY_IC_ID, IC_BUFFER_SIZE_32BIT).

  Parameters:
    index      - Identifies the desired IC module

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1
    uint32_t bufferVal;
    // Read input capture buffer value and store it in 'bufferVal'
    bufferVal = PLIB_IC_Buffer32BitGet(MY_IC_ID);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsBufferValue in your application to determine
    whether this feature is available.
*/

 uint32_t PLIB_IC_Buffer32BitGet( IC_MODULE_ID index );

   // *****************************************************************************
/* Function:
    void PLIB_IC_BufferSizeSelect( IC_MODULE_ID index, IC_BUFFER_SIZE bufSize)

  Summary:
    Sets the input capture buffer size.

  Description:
    This function sets the input capture buffer size for IC module. The buffer size can be set to
    16 bits or 32 bits. The buffer size should be consistent with the timer selected. A 32-bit timer
    requires a 32-bit buffer and a 16-bit timer requires a 16-bit buffer.

  Precondition:
    The buffer size should be consistent with the timer selected.

  Parameters:
    index      - Identifies the desired IC module
    bufSize    - Sets the buffer size to 16 bits or 32 bits

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1
    // 32-bit timer resource is selected
    PLIB_IC_BufferSizeSelect(MY_IC_ID, IC_BUFFER_SIZE_32BIT);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsBufferSize in your application to determine
    whether this feature is available.
*/

 void PLIB_IC_BufferSizeSelect( IC_MODULE_ID index, IC_BUFFER_SIZE bufSize);


 // *****************************************************************************
/* Function:
    void PLIB_IC_TimerSelect( IC_MODULE_ID index, IC_TIMERS tmr )

  Summary:
    Selects the clock source for the IC module.

  Description:
    This function selects the 16-bit timer source for the IC module.

  Precondition:
    The 16-bit time base needs to be set.  

  Parameters:
    index      - Identifies the desired IC module
    tmr        - Identifies the 16-bit timer

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1
    // 16-bit Timer-2 is selected
    PLIB_IC_TimerSelect(MY_IC_ID, IC_TIMER_TMR2);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsTimerSelect in your application to determine
    whether this feature is available.
*/
 void PLIB_IC_TimerSelect( IC_MODULE_ID index, IC_TIMERS tmr );
 
 
// *****************************************************************************
/* Function:
    void PLIB_IC_AlternateClockEnable(IC_MODULE_ID index)

  Summary:
    Selects the alternate clock source.

  Description:
    Selects the alternate clock source instead of Timer2/3.

  Precondition:
    A system unlock PLIB_DEVCON_SystemUnlock must be 
    performed before this function can be executed.

  Parameters:
    index      - Identifies the desired IC module

  Returns:
    None.

  Example:
  <code>
    // Call system service to unlock oscillator
    #define MY_IC_ID IC_ID_1
    PLIB_IC_AlternateClockEnable( MY_IC_ID );
  </code>

  Remarks:
    The feature is not supported on all devices.  Please refer to the
    specific device data sheet to determine availability.
    A system unlock must be performed before this function can be executed.
    This function applies to all input capture modules, regardless of the
    IC_MODULE_ID passed in the call.
*/

void PLIB_IC_AlternateClockEnable(IC_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_IC_AlternateClockDisable(IC_MODULE_ID index)

  Summary:
    Selects Timer2 and Timer3, instead of the alternate clock source.

  Description:
    Selects Timer2 and Timer3, instead of the alternate clock source.

  Precondition:
    A system unlock PLIB_DEVCON_SystemUnlock must be 
    performed before this function can be executed.

  Parameters:
    index      - Identifies the desired IC module

  Returns:
    None.

  Example:
  <code>
    // Call system service to unlock oscillator
    #define MY_IC_ID IC_ID_1
    PLIB_IC_AlternateClockDisable( MY_IC_ID );
  </code>

  Remarks:
    The feature is not supported on all devices. Please refer to the
    specific device data sheet to determine availability.
    A system unlock must be performed before this function can be executed.
    This function applies to all input capture modules, regardless of the
    IC_MODULE_ID passed in the call.
*/

void PLIB_IC_AlternateClockDisable(IC_MODULE_ID index);
 
 
// *****************************************************************************
/* Function:
    bool PLIB_IC_AlternateTimerSelect( IC_MODULE_ID index, IC_ALT_TIMERS tmr )

  Summary:
    Selects an alternate timer as a clock source for the IC module.

  Description:
    This function selects an alternate timer as a clock source for the IC module.
    IC_ID_1,IC_ID_2,IC_ID_3 : Can use Timer4 or Timer5 as alternate timers.
    IC_ID_4,IC_ID_5,IC_ID_6,IC_ID_13,IC_ID_14,IC_ID_15,IC_ID_16 : Can use 
                                         Timer2 or Timer3 as alternate timers.
    IC_ID_7,IC_ID_8,IC_ID_9 : Can use Timer6 or Timer7 as alternate timers.
    IC_ID_10,IC_ID_11,IC_ID_12 : Can use Timer8 or Timer9 as alternate timers.

  Precondition:
    The 16-bit time base needs to be set.
    PLIB_IC_AlternateClockEnable API should be called for the IC module
    to enable the alternate clock selection.

  Parameters:
    index      - Identifies the desired IC module
    tmr        - Identifies the alternate timer

  Returns:
    Boolean
    - true  - Alternate timer selected successfully.
    - false - Alternate timer selection failure, select appropriate alternate timer 
              for the IC module index.

  Example:
    <code>
    #define MY_IC_ID   IC_ID_1
    bool result;
    
    //Enabling alternate timer selection
    // Call system service to unlock oscillator
    PLIB_IC_AlternateClockEnable( MY_IC_ID );

    // 16-bit Timer4 is selected as the clock source for IC module 1
    result = PLIB_IC_AlternateTimerSelect(MY_IC_ID, IC_ALT_TIMER_TMR4);

    if(false == result)
    {
        //Selected alternate timer does not available for the desired IC module.
        //Select appropriate alternate timer.
    }
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsAlternateTimerSelect in your application to determine
    whether this feature is available.
*/
bool PLIB_IC_AlternateTimerSelect( IC_MODULE_ID index, IC_ALT_TIMERS tmr );


  // *****************************************************************************
/* Function:
    void PLIB_IC_ModeSelect( IC_MODULE_ID index, IC_INPUT_CAPTURE_MODES modeSel )

  Summary:
    Selects the input capture mode for IC module.

  Description:
    This function selects the input capture mode for the IC module.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired IC module
    modeSel    - Identifies the timer type required

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1
    // Every Edge Mode is selected
    PLIB_IC_ModeSelect(MY_IC_ID, IC_INPUT_CAPTURE_EVERY_EDGE_MODE);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsCaptureMode in your application to determine
    whether this feature is available.
*/
 void PLIB_IC_ModeSelect( IC_MODULE_ID index, IC_INPUT_CAPTURE_MODES modeSel );


   // *****************************************************************************
/* Function:
    void PLIB_IC_EventsPerInterruptSelect(IC_MODULE_ID index,IC_EVENTS_PER_INTERRUPT event)

  Summary:
    Selects the number of capture events before an interrupt is issued.

  Description:
    The IC module can be configured to generate interrupts depending upon the
    occurrence of a certain number of capture events. The number of capture events
    before an interrupt is generated is set by this function.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired IC module
    event      - Identifies the interrupt control mode

  Returns:
    None.

  Example:
    <code>
    #define MY_IC_ID IC_ID_1
    // IC module is configured to generate interrupt on every capture event
    PLIB_IC_EventsPerInterruptSelect(MY_IC_ID, IC_INTERRUPT_ON_EVERY_CAPTURE_EVENT );
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsEventsPerInterruptSelect in your application to determine
    whether this feature is available.
*/
void PLIB_IC_EventsPerInterruptSelect( IC_MODULE_ID index, IC_EVENTS_PER_INTERRUPT event );

 // *****************************************************************************
/* Function:
   bool PLIB_IC_BufferOverflowHasOccurred(IC_MODULE_ID index)

  Summary:
   Checks whether an input capture buffer overflow has occurred.

  Description:
   This function returns 'true' if an input capture buffer has overflowed and
   'false' if the buffer has not overflowed.

  Precondition:
   This function only applies when not in Edge Detect mode.

  Parameters:
   index        - Identifies the desired IC module

  Returns:
   Boolean
   - true  - An overflow of the input capture buffer has occurred
   - false - An overflow of the input capture buffer has not occurred

  Example:
  <code>
    #define MY_IC_ID IC_ID_1
    if(!PLIB_IC_BufferOverflowHasOccurred(MY_IC_ID))
    {
        // Do some operation
    };
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsBufferOverflowStatus in your application to determine
    whether this feature is available.

*/
bool PLIB_IC_BufferOverflowHasOccurred(IC_MODULE_ID index);


 // *****************************************************************************
/* Function:
   bool PLIB_IC_BufferIsEmpty(IC_MODULE_ID index)

  Summary:
   Checks whether the input capture buffer is empty.

  Description:
   This function returns 'true' if the input capture buffer is empty and
   'false' if the buffer is not empty.

  Precondition:
   None.

  Parameters:
   index        - Identifies the desired IC module

  Returns:
   Boolean
   - true  - The input capture buffer is empty
   - false - The input capture buffer is not empty

  Example:
  <code>
    #define MY_IC_ID IC_ID_1
    if(!PLIB_IC_BufferIsEmpty(MY_IC_ID))
    {
        // Do some operation
    };
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_IC_ExistsBufferIsEmptyStatus in your application to determine
    whether this feature is available.

*/
bool PLIB_IC_BufferIsEmpty(IC_MODULE_ID index);


// *****************************************************************************
// *****************************************************************************
// Section: IC Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************
/* The following functions indicate the existence of the features on the device.
*/

//******************************************************************************
/* Function:
    PLIB_IC_ExistsEnable( IC_MODULE_ID index )

  Summary:
    Identifies whether the EnableControl feature exists on the IC module.

  Description:
    This function identifies whether the EnableControl feature is available on 
	the IC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_IC_Enable
    - PLIB_IC_Disable

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

bool PLIB_IC_ExistsEnable( IC_MODULE_ID index );

//******************************************************************************
/* Function:
    PLIB_IC_ExistsEnable( IC_MODULE_ID index )

  Summary:
    Identifies whether the EnableControl feature exists on the IC module

  Description:
    This function identifies whether the EnableControl feature is available on 
	the IC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_IC_Enable
    - PLIB_IC_Disable

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

bool PLIB_IC_ExistsEnable( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsStopInIdle( IC_MODULE_ID index )

  Summary:
    Identifies whether the StopInIdle feature exists on the IC module.

  Description:
    This function identifies whether the StopInIdle feature is available on the 
	IC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_IC_StopInIdleEnable
    - PLIB_IC_StopInIdleDisable

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

bool PLIB_IC_ExistsStopInIdle( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsEdgeCapture( IC_MODULE_ID index )

  Summary:
    Identifies whether the EdgeCapture feature exists on the IC module.

  Description:
    This function identifies whether the EdgeCapture feature is available on 
	the IC module.
    When this function returns true, this function is supported on the device:
    - PLIB_IC_FirstCaptureEdgeSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EdgeCapture feature is supported on the device
    - false  - The EdgeCapture feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_IC_ExistsEdgeCapture( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsEventsPerInterruptSelect( IC_MODULE_ID index )

  Summary:
    Identifies whether the EventsPerInterruptSelect feature exists on the IC module.

  Description:
    This function identifies whether the EventsPerInterruptSelect feature is available 
	on the IC module.
    When this function returns true, this function is supported on the device:
    - PLIB_IC_EventsPerInterruptSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EventsPerInterruptSelect feature is supported on the device
    - false  - The EventsPerInterruptSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_IC_ExistsEventsPerInterruptSelect( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsBufferValue( IC_MODULE_ID index )

  Summary:
    Identifies whether the BufferValue feature exists on the IC module.

  Description:
    This function identifies whether the BufferValue feature is available on the IC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_IC_Buffer32BitGet
    - PLIB_IC_Buffer16BitGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The BufferValue feature is supported on the device
    - false  - The BufferValue feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_IC_ExistsBufferValue( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsBufferIsEmptyStatus( IC_MODULE_ID index )

  Summary:
    Identifies whether the BufferIsEmptyStatus feature exists on the IC module

  Description:
    This function identifies whether the BufferIsEmptyStatus feature is available on the IC module.
    When this function returns true, this function is supported on the device:
    - PLIB_IC_BufferIsEmpty

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The BufferIsEmptyStatus feature is supported on the device
    - false  - The BufferIsEmptyStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_IC_ExistsBufferIsEmptyStatus( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsBufferOverflowStatus( IC_MODULE_ID index )

  Summary:
    Identifies whether the BufferOverflowStatus feature exists on the IC module.

  Description:
    This function identifies whether the BufferOverflowStatus feature is available 
	on the IC module.
    When this function returns true, this function is supported on the device:
    - PLIB_IC_BufferOverflowHasOccurred

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The BufferOverflowStatus feature is supported on the device
    - false  - The BufferOverflowStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_IC_ExistsBufferOverflowStatus( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsCaptureMode( IC_MODULE_ID index )

  Summary:
    Identifies whether the CaptureMode feature exists on the IC module.

  Description:
    This function identifies whether the CaptureMode feature is available on the 
	IC module.
    When this function returns true, this function is supported on the device:
    - PLIB_IC_ModeSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CaptureMode feature is supported on the device
    - false  - The CaptureMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_IC_ExistsCaptureMode( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsBufferSize( IC_MODULE_ID index )

  Summary:
    Identifies whether the BufferSize feature exists on the IC module.

  Description:
    This function identifies whether the BufferSize feature is available on the 
	IC module.
    When this function returns true, this function is supported on the device:
    - PLIB_IC_BufferSizeSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The BufferSize feature is supported on the device
    - false  - The BufferSize feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_IC_ExistsBufferSize( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsTimerSelect( IC_MODULE_ID index )

  Summary:
    Identifies whether the TimerSelect feature exists on the IC module.

  Description:
    This function identifies whether the TimerSelect feature is available on the 
	IC module.
    When this function returns true, this function is supported on the device:
    - PLIB_IC_TimerSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TimerSelect feature is supported on the device
    - false  - The TimerSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_IC_ExistsTimerSelect( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_IC_ExistsAlternateClock( IC_MODULE_ID index )

  Summary:
    Identifies whether the AlternateClock feature exists on the IC module.

  Description:
    This function identifies whether the AlternateClock feature is available on the IC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_IC_AlternateClockEnable
    - PLIB_IC_AlternateClockDisable

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

bool PLIB_IC_ExistsAlternateClock( IC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_IC_ExistsAlternateTimerSelect( IC_MODULE_ID index )

  Summary:
    Identifies whether the AlternateTimerSelect feature exists on the IC module.

  Description:
    This function identifies whether the AlternateTimerSelect feature is available 
	on the IC module.
    When this function returns true, this function is supported on the device:
    - PLIB_IC_AlternateTimerSelect

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

bool PLIB_IC_ExistsAlternateTimerSelect( IC_MODULE_ID index );


// ****************************************************************************
// ****************************************************************************
// Section: Included Files (continued)
// ****************************************************************************
// ****************************************************************************
/*  The following included file maps the interface definitions above to appropriate
    implementations defined in the implementation (imp) file(s).
*/

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif
/*******************************************************************************
 End of File
*/