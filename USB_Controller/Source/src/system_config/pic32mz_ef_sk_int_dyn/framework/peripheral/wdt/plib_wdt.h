/*******************************************************************************
  Watchdog Timer (WDT) Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_wdt.h

  Summary:
    Watchdog Timer (WDT) Peripheral Library interface header for Watchdog Timer 
    common definitions.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the Watchdog
    Timer Peripheral Library for all families of Microchip microcontrollers. 
    The definitions in this file are common to the Watchdog Timer peripheral.
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

#ifndef _PLIB_WDT_H
#define _PLIB_WDT_H

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

#include "peripheral/wdt/processor/wdt_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: WDT Peripheral Library Interface Routines - General Configuration
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    void PLIB_WDT_Enable ( WDT_MODULE_ID index )

  Summary:
    Enables the WDT module.

  Description:
    This function enables the WDT module. If it is already enabled through the
    Configuration bits, it will keep it enabled.
    This operation is atomic.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired WDT module

  Returns:
    None.

  Example:
    <code>
    PLIB_WDT_Enable ( WDT_ID_0 );
    </code>

  Remarks:
    Calling this function is not necessary if it is enabled through its
    Configuration bits.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_WDT_ExistsEnableControl in your application to determine whether
    this feature is available.
*/

void PLIB_WDT_Enable ( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_WDT_Disable ( WDT_MODULE_ID index )

  Summary:
    Disables the WDT module.

  Description:
    This function disables the WDT module if it is enabled in software.
    This operation is atomic.

  Precondition:
    The WDT module must be enabled through software.

  Parameters:
    index      - Identifies the desired WDT module

  Returns:
    None.

  Example:
    <code>
    PLIB_WDT_Disable ( WDT_ID_0 );
    </code>

  Remarks:
    This function will not disable the WDT module if it is enabled through its
    Configuration bits.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_WDT_ExistsEnableControl in your application to determine whether
    this feature is available.
*/

void PLIB_WDT_Disable ( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_WDT_WindowEnable ( WDT_MODULE_ID index )

  Summary:
    Enables the WDT Window mode.

  Description:
    This function enables the WDT Windowed mode.
    This operation is atomic.

  Precondition:
    The window size must be set through the Configuration bits.

  Parameters:
    index      - Identifies the desired WDT module

  Returns:
    None.

  Example:
    <code>
    PLIB_WDT_WindowEnable ( WDT_ID_0 );
    PLIB_WDT_Enable ( WDT_ID_0 );
    </code>

  Remarks:
    The window size must be set through the Configuration bits.

    The example code doesn't include the settings that should be done through
    the Configuration bits.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_WDT_ExistsWindowEnable in your application to determine whether
    this feature is available.
*/

void PLIB_WDT_WindowEnable ( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_WDT_WindowDisable ( WDT_MODULE_ID index )

  Summary:
    Disables the WDT Windowed mode.

  Description:
    This function disables the WDT Windowed mode.
    This operation is atomic.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired WDT module

  Returns:
    None.

  Example:
    <code>
    PLIB_WDT_WindowDisable ( WDT_ID_0 );
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_WDT_ExistsWindowEnable in your application to determine whether
    this feature is available.
*/

void PLIB_WDT_WindowDisable ( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_WDT_TimerClear ( WDT_MODULE_ID index )

  Summary:
    Resets the WDT module.

  Description:
    This function resets the WDT module. The WDT module should be cleared
    periodically before the count crosses and forces the device to reset.
    This operation is atomic.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired WDT module

  Returns:
    None.

  Example:
    <code>
    PLIB_WDT_Enable ( WDT_ID_0 );

    //Application loop
    while(1)
    {
        PLIB_WDT_TimerClear ( WDT_ID_0 );
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
    PLIB_WDT_ExistsTimerClear in your application to determine whether
    this feature is available.
*/

void PLIB_WDT_TimerClear ( WDT_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: WDT Peripheral Library Interface Routines - General Status
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    char PLIB_WDT_PostscalerValueGet ( WDT_MODULE_ID index )

  Summary:
    Returns the WDT postscaler value.

  Description:
    This function returns the WDT postscaler value. The value will correspond to 
    a division factor.
    This operation is atomic.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired WDT module

  Returns:
    The postscaler value.

  Example:
    <code>
    uint8_t value;

    PLIB_WDT_Enable ( WDT_ID_0 );
    value = PLIB_WDT_PostscalerValueGet ( WDT_ID_0 );
    </code>

  Remarks:
    The value returned will be right-aligned.

    Refer to the specific device data sheet to get the division factor
    corresponding to the value.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_WDT_ExistsPostscalerValue in your application to determine whether
    this feature is available.

*/

char PLIB_WDT_PostscalerValueGet ( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    char PLIB_WDT_SleepModePostscalerValueGet ( WDT_MODULE_ID index )

  Summary:
    Returns the WDT Sleep Mode postscaler value.

  Description:
    This function returns the WDT postscaler value in Sleep Mode. The value will 
	correspond to a division factor.
    This operation is atomic.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired WDT module

  Returns:
    The Sleep Mode postscaler value.

  Example:
    <code>
    uint8_t value;
    value = PLIB_WDT_SleepModePostscalerValueGet ( WDT_ID_0 );
    </code>

  Remarks:
    The value returned will be right-aligned.

    Refer to the specific device data sheet to get the division factor
    corresponding to the value.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_WDT_ExistsSleepModePostscalerValue in your application to determine whether
    this feature is available.

*/

char PLIB_WDT_SleepModePostscalerValueGet ( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_WDT_IsEnabled ( WDT_MODULE_ID index )

  Summary:
    Returns the watchdog timer on/off(enable/disable) status.

  Description:
    Returns the 'true', if the watchdog timer is already ON. Otherwise returns
    'false'.
    This operation is atomic.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired WDT module

  Returns:
    - true 	- If the watchdog timer is on
    - false - If the watchdog timer is off

  Example:
    <code>
    if (PLIB_WDT_IsEnabled ( WDT_ID_0 ) )
    {
        //Do some action
    }
    </code>

  Remarks:
    This function returns 'true' if the device is enabled  either though the
    Configuration bits or in the software.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_WDT_ExistsEnableControl in your application to determine whether
    this feature is available.
*/

bool PLIB_WDT_IsEnabled ( WDT_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: WDT Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************
/* The functions below indicate the existence of the features on the device.
*/

//******************************************************************************
/* Function:
    PLIB_WDT_ExistsEnableControl( WDT_MODULE_ID index )

  Summary:
    Identifies whether the EnableControl feature exists on the WDT module.

  Description:
    This function identifies whether the EnableControl feature is available on the
    WDT module. When this function returns true, these functions are supported 
    on the device:
    - PLIB_WDT_Enable
    - PLIB_WDT_Disable
    - PLIB_WDT_IsEnabled
    This operation is atomic.

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

bool PLIB_WDT_ExistsEnableControl( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_WDT_ExistsWindowEnable( WDT_MODULE_ID index )

  Summary:
    Identifies whether the WindowEnable feature exists on the WDT module.

  Description:
    This function identifies whether the WindowEnable feature is available on the 
    WDT module. When this function returns true, these functions are supported 
    on the device:
    - PLIB_WDT_WindowEnable
    - PLIB_WDT_WindowDisable
    This operation is atomic.

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The WindowEnable feature is supported on the device
    - false  - The WindowEnable feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_WDT_ExistsWindowEnable( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_WDT_ExistsTimerClear( WDT_MODULE_ID index )

  Summary:
    Identifies whether the TimerClear feature exists on the WDT module.

  Description:
    This function identifies whether the TimerClear feature is available on the 
    WDT module. When this function returns true, this function is supported on 
    the device:
    - PLIB_WDT_TimerClear
    This operation is atomic.

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TimerClear feature is supported on the device
    - false  - The TimerClear feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_WDT_ExistsTimerClear( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_WDT_ExistsPostscalerValue( WDT_MODULE_ID index )

  Summary:
    Identifies whether the PostscalerValue feature exists on the WDT module.

  Description:
    This function identifies whether the PostscalerValue feature is available 
    on the WDT module. When this function returns true, this function is 
    supported on the device:
    - PLIB_WDT_PostscalerValueGet
    This operation is atomic.

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PostscalerValue feature is supported on the device
    - false  - The PostscalerValue feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_WDT_ExistsPostscalerValue( WDT_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_WDT_ExistsSleepModePostscalerValue( WDT_MODULE_ID index )

  Summary:
    Identifies whether the SleepModePostscalerValue feature exists on the WDT module.

  Description:
    This function identifies whether the SleepModePostscalerValue feature is available 
    on the WDT module. When this function returns true, this function is 
    supported on the device:
    - PLIB_WDT_SleepModePostscalerValueGet
    This operation is atomic.

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The SleepModePostscalerValue feature is supported on the device
    - false  - The SleepModePostscalerValue feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_WDT_ExistsSleepModePostscalerValue( WDT_MODULE_ID index );


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif  //_PLIB_WDT_H
/*******************************************************************************
 End of File
*/