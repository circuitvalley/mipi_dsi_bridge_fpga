/*******************************************************************************
  Deadman Timer (DMT) Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_dmt.h

  Summary:
    Deadman Timer (DMT) Peripheral Library interface header for Deadman Timer 
    common definitions.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the Deadman
    Timer Peripheral Library for all families of Microchip microcontrollers. 
    The definitions in this file are common to the Deadman Timer peripheral.
**************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
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

#ifndef _PLIB_DMT_H
#define _PLIB_DMT_H


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

#include "peripheral/dmt/processor/dmt_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: DMT Peripheral Library Interface Routines - General Configuration
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    void PLIB_DMT_Enable ( DMT_MODULE_ID index )

  Summary:
    Enables the DMT module.

  Description:
    This function enables the DMT module. If it is already enabled through the
    Configuration bits, it will keep it enabled. 

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    None.

  Example:
    <code>
    PLIB_DMT_Enable ( DMT_ID_0 );
    </code>

  Remarks:
    Calling this function is not necessary if it is enabled through its
    Configuration bits.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsEnableControl in your application to determine whether
    this feature is available.
*/

void PLIB_DMT_Enable ( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_DMT_Disable ( DMT_MODULE_ID index )

  Summary:
    Disables the DMT module.

  Description:
    This function disables the DMT module if it is enabled in software. 
  
  Precondition:
    The DMT module must have been enabled through software.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    None.

  Example:
    <code>
    PLIB_DMT_Disable ( DMT_ID_0 );
    </code>

  Remarks:
    This function will not disable the DMT module if it is enabled through its
    Configuration bits.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsEnableControl in your application to determine whether
    this feature is available.
*/

void PLIB_DMT_Disable ( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_DMT_ClearStep1 ( DMT_MODULE_ID index )

  Summary:
    Resets the DMT module.

  Description:
    This function performs the STEP1 Clearing of the DMT module. The DMT module 
	should be cleared in two steps, within the interval determined by the Count 
	Window before the DMT forces an NMI or device reset.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    None.

  Example:
    <code>
    PLIB_DMT_Enable ( DMT_ID_0 );

    //Application loop
    while(1)
    {
        PLIB_DMT_ClearStep1 ( DMT_ID_0 );
        //user code
		PLIB_DMT_ClearStep2 ( DMT_ID_0 );
		//user code
    }
    </code>

  Remarks:
    Resetting the device before the count reaches the window will cause
    a reset in Windowed mode.

    The example code does not include the settings that should be done through
    the Configuration bits.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsTimerClear in your application to determine whether
    this feature is available.
*/

void PLIB_DMT_ClearStep1 ( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_DMT_ClearStep2 ( DMT_MODULE_ID index )

  Summary:
    Resets the DMT module.

  Description:
    This function performs the STEP2 Clearing of the DMT module. The DMT module 
	should be cleared in two steps, within the interval determined by the Count 
	Window before the DMT forces an NMI or device reset.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    None.

  Example:
    <code>
    PLIB_DMT_Enable ( DMT_ID_0 );

    //Application loop
    while(1)
    {
        PLIB_DMT_ClearStep1 ( DMT_ID_0 );
        //user code
		PLIB_DMT_ClearStep2 ( DMT_ID_0 );
		//user code
    }
    </code>

  Remarks:
    Resetting the device before the count reaches the window will cause
    a reset in Windowed mode.

    The example code doesn't include the settings that should be done through
    the Configuration bits.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsTimerClear in your application to determine whether
    this feature is available.
*/

void PLIB_DMT_ClearStep2 ( DMT_MODULE_ID index );

// *****************************************************************************
// *****************************************************************************
// Section: DMT Peripheral Library Interface Routines - General Status
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    bool PLIB_DMT_BAD1Get ( DMT_MODULE_ID index )

  Summary:
    Returns BAD1 flag from the DMT Status Register.

  Description:
    This function returns the DMT BAD1 Flag. This flag is set when there
    is an incorrect write to STEP1, such as the wrong value, writing too
    early, or writing too late.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    The flag value, true or false.

  Example:
    <code>
    PLIB_DMT_Enable ( DMT_ID_0 );
    //user code
    PLIB_DMT_ClearStep1();
    if( PLIB_DMT_BAD1Get ( DMT_ID_0 ))
    {
       //user code
    }
    </code>
  Remarks:
    The flag returned will indicated if a STEP1 write was not successful.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsStatus in your application to determine whether
    this feature is available.

*/

bool PLIB_DMT_BAD1Get ( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_DMT_BAD2Get ( DMT_MODULE_ID index )

  Summary:
    Returns BAD2 flag from the DMT Status Register.

  Description:
    This function returns the DMT BAD2 Flag. This flag is set when there
    is an incorrect write to STEP2, such as the wrong value, writing too
    early, or writing too late.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    The flag value, true or false.

  Example:
    <code>
    PLIB_DMT_Enable ( DMT_ID_0 );
    //user code
    PLIB_DMT_ClearStep1();
    //user code
    PLIB_DMT_ClearStep2();
    if( PLIB_DMT_BAD2Get ( DMT_ID_0 ))
    {
       //user code
    }
    </code>
  Remarks:
    The flag returned will indicated if a STEP2 write was not successful.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsStatus in your application to determine whether
    this feature is available.

*/

bool PLIB_DMT_BAD2Get ( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_DMT_WindowIsOpen( DMT_MODULE_ID index )

  Summary:
    Returns Window is Open flag from the DMT Status Register.

  Description:
    This function returns the flag indicating the DMT Window is open. 
    
  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    The flag value, true or false.

  Example:
    <code>
    PLIB_DMT_Enable ( DMT_ID_0 );
    //user code
    PLIB_DMT_ClearStep1();
    //user code
    if( PLIB_DMT_WindowIsOpen( DMT_ID_0 ))
    {
       PLIB_DMT_ClearStep2();
    }
    
    </code>
  Remarks:
    The flag returned will indicated if the DMT window is open.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsStatus in your application to determine whether
    this feature is available.

*/

bool PLIB_DMT_WindowIsOpen ( DMT_MODULE_ID index );

//******************************************************************************
/* Function:
    bool PLIB_DMT_EventOccurred( DMT_MODULE_ID index )

  Summary:
    Returns Event flag from the DMT Status Register.

  Description:
    This function returns the flag indicating a DMT event has happened,
    such as a Window Open, or a Bad Flag is set. 
    
  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    The flag value, true or false.

  Example:
    <code>
    PLIB_DMT_Enable ( DMT_ID_0 );
    //user code
    PLIB_DMT_ClearStep1();
    //user code
    if( PLIB_DMT_EventOccurred( DMT_ID_0 ))
    {
       //user code
    }
    
    </code>
  Remarks:
    The flag returned will indicate if a DMT event has happened.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsStatus in your application to determine whether
    this feature is available.

*/

bool PLIB_DMT_EventOccurred ( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    uint32_t PLIB_DMT_CounterGet ( DMT_MODULE_ID index )

  Summary:
    Returns the DMT counter value.

  Description:
    This function returns the DMT counter value. The value is the number of
    instructions counted since the count was last cleared.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    The counter value.

  Example:
    <code>
    uint32_t value;

    PLIB_DMT_Enable ( DMT_ID_0 );
    value = PLIB_DMT_CounterGet ( DMT_ID_0 );
    </code>

  Remarks:
    The value returned will be right-aligned.

    Refer the data sheet of the specific device to get the division factor
    corresponding to the value.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsCounterValue in your application to determine whether
    this feature is available.

*/

uint32_t PLIB_DMT_CounterGet ( DMT_MODULE_ID index );



//******************************************************************************
/* Function:
    uint32_t PLIB_DMT_PostscalerValueGet ( DMT_MODULE_ID index )

  Summary:
    Returns the DMT postscaler value.

  Description:
    This function returns the DMT postscaler value. The value will correspond to 
    a total number of instructions for DMT timeout, a value determined by 
	configuration bits.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    The postscaler value.

  Example:
    <code>
    uint32_t value;

    PLIB_DMT_Enable ( DMT_ID_0 );
    value = PLIB_DMT_PostscalerValueGet ( DMT_ID_0 );
    </code>

  Remarks:
    The value returned will be right-aligned.

    Refer to the specific device data sheet to get the association of the 
	configuration bits corresponding to this postscaler value.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsPostscalerValue in your application to determine whether
    this feature is available.

*/

uint32_t PLIB_DMT_PostscalerValueGet ( DMT_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_DMT_PostscalerIntervalGet ( DMT_MODULE_ID index )

  Summary:
    Returns the DMT postscaler interval value.

  Description:
    This function returns the DMT postscaler interval. The value will correspond 
    to a total number of instructions for DMT window, a value determined by 
	configuration bits.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    The postscaler interval.

  Example:
    <code>
    uint32_t value;

    PLIB_DMT_Enable ( DMT_ID_0 );
    value = PLIB_DMT_PostscalerIntervalGet ( DMT_ID_0 );
    </code>

  Remarks:
    The value returned will be right-aligned.

    Refer the data sheet of the specific device to get the association of 
	the configuration bits corresponding to this postscaler value.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsPostscalerValue in your application to determine whether
    this feature is available.

*/

uint32_t PLIB_DMT_PostscalerIntervalGet ( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_DMT_IsEnabled ( DMT_MODULE_ID index )

  Summary:
    Returns the Deadman Timer on/off(enable/disable) status.

  Description:
    Returns the 'true', if the Deadman Timer is already ON. Otherwise returns
    'false'.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired DMT module

  Returns:
    true 	- If the Deadman Timer is on
    false 	- If the Deadman Timer is off

  Example:
    <code>
    if (PLIB_DMT_IsEnabled ( DMT_ID_0 ) )
    {
        //Do some action
    }
    </code>

  Remarks:
    This function returns 'true' if the device is enabled  either though the
    Configuration bits or in the software.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_DMT_ExistsEnableControl in your application to determine whether
    this feature is available.
*/

bool PLIB_DMT_IsEnabled( DMT_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: DMT Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************
/* The functions below indicate the existence of the features on the device.
*/

//******************************************************************************
/* Function:
    PLIB_DMT_ExistsEnableControl( DMT_MODULE_ID index )

  Summary:
    Identifies whether the EnableControl feature exists on the DMT module.

  Description:
    This function identifies whether the EnableControl feature is available on the
    DMT module. When this function returns true, these functions are supported 
    on the device:
    - PLIB_DMT_Enable
    - PLIB_DMT_Disable
    - PLIB_DMT_IsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the EnableControl feature:
    - true   - When EnableControl feature is supported on the device
    - false  - When EnableControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_DMT_ExistsEnableControl( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_DMT_ExistsStatus( DMT_MODULE_ID index )

  Summary:
    Identifies whether the Status feature exists on the DMT module.

  Description:
    This function identifies whether the Status feature is available on the
    DMT module. When this function returns true, these functions are supported 
    on the device:
    - PLIB_DMT_WindowIsOpen
    - PLIB_DMT_EventOccurred
    - PLIB_DMT_BAD1Get
    - PLIB_DMT_BAD2Get

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the Status feature:
    - true   - When Status feature is supported on the device
    - false  - When Status feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_DMT_ExistsStatus( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_DMT_ExistsStep1( DMT_MODULE_ID index )

  Summary:
    Identifies whether the STEP1 Clear feature exists on the DMT module.

  Description:
    This function identifies whether the Step 1 Clear feature is available on the 
    DMT module. When this function returns true, this function is supported on 
    the device:
    - PLIB_DMT_ClearStep1

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The STEP1 Clear feature is supported on the device
    - false  - The STEP1 Clear feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_DMT_ExistsStep1( DMT_MODULE_ID index );

//******************************************************************************
/* Function:
    PLIB_DMT_ExistsStep2( DMT_MODULE_ID index )

  Summary:
    Identifies whether the STEP2 Clear feature exists on the DMT module.

  Description:
    This function identifies whether the STEP2 Clear feature is available on the 
    DMT module. When this function returns true, this function is supported on 
    the device:
    - PLIB_DMT_ClearStep2

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The STEP2 Clear feature is supported on the device
    - false  - The STEP2 Clear feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_DMT_ExistsStep2( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_DMT_ExistsCounter( DMT_MODULE_ID index )

  Summary:
    Identifies whether the Counter feature exists on the DMT module.

  Description:
    This function identifies whether the Counter feature is available 
    on the DMT module. When this function returns true, this function is 
    supported on the device:
    - PLIB_DMT_CounterGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Counter feature is supported on the device
    - false  - The Counter feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_DMT_ExistsCounter( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_DMT_ExistsPostscalerValue( DMT_MODULE_ID index )

  Summary:
    Identifies whether the Postscaler Value feature exists on the DMT module.

  Description:
    This function identifies whether the PostscalerValue feature is available 
    on the DMT module. When this function returns true, this function is 
    supported on the device:
    - PLIB_DMT_PostscalerValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Postscaler Value feature is supported on the device
    - false  - The Postscaler Value feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_DMT_ExistsPostscalerValue( DMT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_DMT_ExistsInterval( DMT_MODULE_ID index )

  Summary:
    Identifies whether the Postscaler Interval feature exists on the DMT module.

  Description:
    This function identifies whether the Postscaler Interval feature is available 
    on the DMT module. When this function returns true, this function is 
    supported on the device:
    - PLIB_DMT_PostscalerIntervalGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Postscaler Interval feature is supported on the device
    - false  - The Postscaler Interval feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_DMT_ExistsPostscalerInterval( DMT_MODULE_ID index );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif  //_PLIB_DMT_H
/*******************************************************************************
 End of File
*/
