/*******************************************************************************
  Cycle Time Register (CTR) Header File Definitions

  Company:
    Microchip Technology Inc.

  File Name:
    plib_ctr.h

  Summary:
    This file contains the interface definition for the Cycle Time Register (CTR) 
	Peripheral Library.

  Description:
	This library provides a low-level abstraction of the Cycle Time Register (CTR) 
	module on Microchip microcontrollers with a convenient C language 
	interface. It can be used to simplify low-level access to the module
	without the necessity of interacting directly with the module's registers,
	thus hiding differences between one microcontroller variant and another.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2015 released Microchip Technology Inc.  All rights reserved.

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
#ifndef _PLIB_CTR_H
#define _PLIB_CTR_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued at end of file)
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.  However,
    please see the end of the file for additional implementation header files
    that are also included
*/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

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
#include "peripheral/ctr/processor/ctr_processor.h"


// *****************************************************************************
// *****************************************************************************
// Section:  General Configuration
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    PLIB_CTR_Exists1394Mode( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTR1394Mode feature exists on the CTR module. 

  Description:
    This function identifies whether the CTR1394Mode feature is available on the 
	CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_1394ModeSecondSet
    - PLIB_CTR_1394ModeSecondGet
    - PLIB_CTR_1394ModeCountSet
    - PLIB_CTR_1394ModeCountGet
    - PLIB_CTR_1394ModeOffsetGet
    - PLIB_CTR_1394ModeOffsetSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTR1394Mode feature is supported on the device
    - false  - The CTR1394Mode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_Exists1394Mode( CTR_MODULE_ID index );
//******************************************************************************
/* Function:
    void PLIB_CTR_1394ModeSecondSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
	                                  uint32_t secVal )

  Summary:
    Sets the seconds value for CTR0 or CTR1 in 1394 mode.

  Description:
    This function enables the CTR module to set the seconds value for 1394 mode of 
	operation for CTR0 or CTR1 as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	secVal		- Seconds value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t secVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_1394ModeSecondSet(MY_CTR_ID,ctrSel, secVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_Exists1394Mode in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_1394ModeSecondSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
                                  uint32_t secVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_1394ModeSecondGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

  Summary:
    Reads the current second value in 1394 mode for CTR0 or CTR1.

  Description:
    This function reads the current second value of CTR0 or CTR1 in 1394 mode.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value. CTR0 or CTR1

  Returns:
    The current seconds value for 1394 mode.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t secVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	secVal = PLIB_CTR_1394ModeSecondGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_Exists1394Mode in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_1394ModeSecondGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );


//******************************************************************************
/* Function:
    void PLIB_CTR_1394ModeCountSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
	                                 uint32_t countVal )

  Summary:
    Sets the count value for CTR0 or CTR1 in 1394 mode.

  Description:
    This function enables the CTR module to set the count value for 1394 mode of 
	operation for CTR0 or CTR1 as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	countVal	- Count value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t countVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_1394ModeCountSet(MY_CTR_ID,ctrSel, countVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_Exists1394Mode in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_1394ModeCountSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  uint32_t countVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_1394ModeCountGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

  Summary:
    Reads the current count value in 1394 mode for CTR0 or CTR1.

  Description:
    This function reads the current count value of CTR0 or CTR1 in 1394 mode.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value. CTR0 or CTR1

  Returns:
    The current count value for 1394 mode.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t countVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	countVal = PLIB_CTR_1394ModeCountGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_Exists1394Mode in your application to determine whether
	this feature is available.

*/

uint32_t PLIB_CTR_1394ModeCountGet ( CTR_MODULE_ID index , CTR_SELECT ctrSel);

//******************************************************************************
/* Function:
    void PLIB_CTR_1394ModeOffsetSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
	                                  uint32_t offsetVal )

  Summary:
    Sets the offset value for CTR0 or CTR1 in 1394 mode.

  Description:
    This function enables the CTR module to set the offset value for 1394 mode 
	of operation for CTR0 or CTR1 as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	offsetVal	- Offset value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t offsetVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_1394ModeOffsetSet(MY_CTR_ID,ctrSel, offsetVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	ExistsCTR1394 in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_1394ModeOffsetSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
                                  uint32_t offsetVal );


//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_1394ModeOffsetGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

  Summary:
    Reads the current offset value in 1394 mode for CTR0 or CTR1.

  Description:
    This function reads the current offset value of CTR0 or CTR1 in 1394 mode.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current offset value for 1394 mode.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t offsetVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	offsetVal = PLIB_CTR_1394ModeOffsetGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_Exists1394Mode in your application to determine whether
	this feature is available.

*/

uint32_t PLIB_CTR_1394ModeOffsetGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsUSMode( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRMicroSecondsMode feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRMicroSecondsMode feature is available 
	on the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_USModeSecondGet
    - PLIB_CTR_USModeSecondSet
    - PLIB_CTR_USModeValueGet
    - PLIB_CTR_USModeValueSet
    - PLIB_CTR_USMode10nsGet
    - PLIB_CTR_USMode10nsSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRMicroSecondsMode feature is supported on the device
    - false  - The CTRMicroSecondsMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsUSMode( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_USModeSecondSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
	                                uint32_t secUSVal )

  Summary:
    Sets the second value for CTR0 or CTR1 in US mode.

  Description:
    This function enables the CTR module to set the second value for the Microseconds 
	(US) mode of operation for CTR0 or CTR1 as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	secUSVal	- Seconds value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t secUSVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_USModeSecondSet(MY_CTR_ID,ctrSel, secUSVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsUSMode in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_USModeSecondSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
                                uint32_t secUSVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_USModeSecondGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

  Summary:
    Reads the current second value in US mode for CTR0 or CTR1.

  Description:
    This function reads the current second value of CTR0 or CTR1 in Microseconds (US) 
	mode.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current seconds value for US mode.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t secUSVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	secUSVal = PLIB_CTR_USModeSecondGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsUSMode in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_USModeSecondGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );


//******************************************************************************
/* Function:
    void PLIB_CTR_USModeValueSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
	                               uint32_t usVal )

  Summary:
    Sets the microseconds value for CTR0 or CTR1 in US mode.

  Description:
    This function enables the CTR module to set the microseconds value for 
	the Microseconds (US) mode of operation for CTR0 or CTR1 as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	usVal		- Microseconds value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t usVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_USModeValueSet(MY_CTR_ID,ctrSel, usVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsUSMode in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_USModeValueSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
                               uint32_t usVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_USModeValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

   Summary:
    Reads the current microseconds value in US mode for CTR0 or CTR1.

  Description:
    This function reads the current microseconds value of CTR0 or CTR1 in 
	Microseconds (US) mode.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current microseconds value for US mode.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t usVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	usVal = PLIB_CTR_USModeValueGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsUSMode in your application to determine whether
	this feature is available.

*/

uint32_t PLIB_CTR_USModeValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    void PLIB_CTR_USMode10nsSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
	                              uint32_t us10nsVal )

  Summary:
    Sets the 10 ns value for CTR0 or CTR1 in US mode.

  Description:
    This function enables the CTR module to set the 10 ns value for Microseconds (US) 
	mode of operation for CTR0 or CTR1 as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	us10nsVal	- 10 ns value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t us10nsVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_USMode10nsSet(MY_CTR_ID,ctrSel, us10nsVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsUSMode in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_USMode10nsSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel,  
                              uint32_t us10nsVal );


//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_USMode10nsGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

   Summary:
    Reads the current 10 ns value in US mode for CTR0 or CTR1.

  Description:
    This function reads the current 10 ns value of CTR0 or CTR1 in Microseconds (US) 
	mode.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current 10 ns value for US mode.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t us10nsVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	us10nsVal = PLIB_CTR_USMode10nsGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsUSMode in your application to determine whether
	this feature is available.

*/

uint32_t PLIB_CTR_USMode10nsGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsCTRLinear( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRLinear feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRLinear feature is available on the 
	CTR module.
    When this function returns true, this function is supported on the device: 
    - PLIB_CTR_CTRLinearGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRLinear feature is supported on the device
    - false  - The CTRLinear feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsLinearCTR( CTR_MODULE_ID index );
//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_LinearCTRGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

   Summary:
    Reads the current linear value for CTR0 or CTR1.

  Description:
    This function reads the current linear value of CTR0 or CTR1.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current linear value for US mode.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t linearVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	linearVal = PLIB_CTR_LinearCTRGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLinearCTR in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_LinearCTRGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsEnableCTR( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTREnable feature exists on the CTR module. 

  Description:
    This function identifies whether the CTREnable feature is available on the 
	CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_EnableCTR
    - PLIB_CTR_DisableCTR
    - PLIB_CTR_ModuleStatus

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTREnable feature is supported on the device
    - false  - The CTREnable feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsEnableCTR( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_EnableCTR ( CTR_MODULE_ID index, CTR_SELECT ctrSel)

  Summary:
    Enables the selected CTR0 or CTR1.

  Description:
    This function enables the selected CTR out of the two available CTRs.

  Precondition:
    None.

  Parameters:
    index         	- Identifies the desired CTR module
	ctrSel     		- CTR select value (CTR0 or CTR1)

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
    
	PLIB_CTR_EnableCTR( MY_CTR_ID, ctrSel );
    </code>

  Remarks:
	
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsEnableCTR in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_EnableCTR ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    void PLIB_CTR_DisableCTR ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

  Summary:
    Disables the selected CTR0 or CTR1.

  Description:
    This function disables the selected CTR out of the two available CTRs.

  Precondition:
    None.

  Parameters:
    index         	- Identifies the desired CTR module
	ctrSel     		- CTR select value (CTR0 or CTR1)

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
    
	PLIB_CTR_DisableCTR( MY_CTR_ID, ctrSel );
    </code>


  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsEnableCTR in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_DisableCTR ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    CTR_ENABLE_CONTROL PLIB_CTR_ModuleStatus ( CTR_MODULE_ID index, 
	                                           CTR_SELECT ctrSel )

  Summary:
    Reads the current status of the selected CTR.
	
  Description:
    This function reads the current status (enabled or disabled) of the selected CTR.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired CTR module
	ctrSel     - CTR select value (CTR0 or CTR1)

  Returns:
    The current status of the selected CTR.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    CTRModuleStatus = PLIB_CTR_ModuleStatus ( MY_CTR_ID, ctrSel );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsEnableCTR in your application to determine whether
	this feature is available.
*/

CTR_ENABLE_CONTROL PLIB_CTR_ModuleStatus ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsCTRFormatSel( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRFormatSelect feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRFormatSelect feature is available on 
	the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_CTRModeSelect
    - PLIB_CTR_CTRModeStatus

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRFormatSelect feature is supported on the device
    - false  - The CTRFormatSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsCTRFormatSel( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_CTRModeSelect ( CTR_MODULE_ID index, CTR_SELECT ctrSel, 
	                              CTR_MODE_SELECT modeVal)

  Summary:
    Sets the mode value for CTR0 or CTR1.

  Description:
    This function enables the CTR module to set the mode value for CTR0 or CTR1 
	as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	modeVal		- Mode value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t modeVal = CTR_MODE_SELECT_1;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_CTRModeSelect(MY_CTR_ID,ctrSel, modeVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRFormatSel in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_CTRModeSelect ( CTR_MODULE_ID index, CTR_SELECT ctrSel, 
                              CTR_MODE_SELECT modeVal);

//******************************************************************************
/* Function:
    CTR_MODE_SELECT PLIB_CTR_CTRModeStatus ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

  Summary:
    Reads the current mode status of the selected CTR
	
  Description:
    This function Reads the current mode status (1394 or Microseconds) of the 
	selected CTR.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired CTR module
	ctrSel     - CTR select value (CTR0 or CTR1)

  Returns:
    The current mode status of the selected CTR.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    CTRModeStatus = PLIB_CTR_CTRModeStatus ( MY_CTR_ID, ctrSel );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRFormatSel in your application to determine whether
	this feature is available.
*/

CTR_MODE_SELECT PLIB_CTR_CTRModeStatus ( CTR_MODULE_ID index , CTR_SELECT ctrSel);

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsCTRAdjustUS( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRAdjustUS feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRAdjustUS feature is available on the 
	CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_CTRAdjustValueInitialize
    - PLIB_CTR_CTRAdjustValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRAdjustUS feature is supported on the device
    - false  - The CTRAdjustUS feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsCTRAdjustUS( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_CTRAdjustValueInitialize ( CTR_MODULE_ID index , CTR_SELECT ctrSel, 
	                                         uint32_t adjVal)

  Summary:
    Sets the adjust value for CTR0 or CTR1.

  Description:
    This function enables the CTR module to set the adjust value for CTR0 or CTR1 
	as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	adjVal		- Adjust value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
    PLIB_CTR_CTRAdjustValueInitialize(MY_CTR_ID,ctrSel, adjVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRAdjustUS in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_CTRAdjustValueInitialize ( CTR_MODULE_ID index , CTR_SELECT ctrSel, 
                                         uint32_t adjVal);

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_CTRAdjustValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

   Summary:
    Reads the current adjust value for CTR0 or CTR1.

  Description:
    This function reads the current adjust value of CTR0 or CTR1.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current adjust value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t adjVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	adjVal = PLIB_CTR_CTRAdjustValueGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRAdjustUS in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_CTRAdjustValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsCTRDriftUS( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRDriftUS feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRDriftUS feature is available on the 
	CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_CTRDriftValueSet
    - PLIB_CTR_CTRDriftValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRDriftUS feature is supported on the device
    - false  - The CTRDriftUS feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsCTRDriftUS( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_CTRDriftValueSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel, 
	                                 uint32_t driftVal )

  Summary:
    Sets the drift value for CTR0 or CTR1.

  Description:
    This function enables the CTR module to set the drift value for CTR0 or CTR1 
	as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	driftVal	- Drift value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t driftVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_CTRDriftValueSet(MY_CTR_ID,ctrSel, driftVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRDriftUS in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_CTRDriftValueSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel , 
                                 uint32_t driftVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_CTRDriftValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

   Summary:
    Reads the current drift value for CTR0 or CTR1.

  Description:
    This function reads the current drift value of CTR0 or CTR1.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current drift value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t driftVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	driftVal = PLIB_CTR_CTRDriftValueGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRDriftUS in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_CTRDriftValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsCTRDriftAccuUS( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRDriftAccuUS feature exists on the CTR module 

  Description:
    This function identifies whether the CTRDriftAccuUS feature is available on 
	the CTR module.
    When this function returns true, this function is supported on the device: 
    - PLIB_CTR_CTRAccuUSDriftValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRDriftAccuUS feature is supported on the device
    - false  - The CTRDriftAccuUS feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsCTRDriftAccuUS( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_CTRAccuUSDriftValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

   Summary:
    Reads the current accumulator microseconds drift value for CTR0 or CTR1.

  Description:
    This function reads the current accumulator microseconds drift value of 
	CTR0 or CTR1.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current accumulator microseconds drift value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t accuUSDriftVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	accuUSDriftVal = PLIB_CTR_CTRAccuUSDriftValueGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRDriftAccuUS in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_CTRAccuUSDriftValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsCTRAdjustLIN( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRAdjustLinear feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRAdjustLinear feature is available on 
	the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_CTRLinearAdjustInitialize
    - PLIB_CTR_CTRLinearAdjustGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRAdjustLinear feature is supported on the device
    - false  - The CTRAdjustLinear feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsCTRAdjustLIN( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_CTRLinearAdjustInitialize ( CTR_MODULE_ID index, CTR_SELECT ctrSel, 
	                                          uint32_t linAdjVal )

  Summary:
    Sets the linear adjust value for CTR0 or CTR1.

  Description:
    This function enables the CTR module to set the linear adjust value for CTR0 or CTR1 
	as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	linAdjVal	- Linear adjust value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t linAdjVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_CTRLinearAdjustInitialize(MY_CTR_ID,ctrSel, linAdjVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRAdjustLIN in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_CTRLinearAdjustInitialize ( CTR_MODULE_ID index, CTR_SELECT ctrSel, 
                                          uint32_t linAdjVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_CTRLinearAdjustGet ( CTR_MODULE_ID index , CTR_SELECT ctrSel)

   Summary:
    Reads the current linear adjust value for CTR0 or CTR1.

  Description:
    This function reads the current linear adjust value of CTR0 or CTR1.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current linear adjust value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t linAdjVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	linAdjVal = PLIB_CTR_CTRLinearAdjustGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRAdjustLIN in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_CTRLinearAdjustGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsCTRDriftLIN( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRDriftLinear feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRDriftLinear feature is available on 
	the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_CTRLinearDriftSet
    - PLIB_CTR_CTRLinearDriftGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRDriftLinear feature is supported on the device
    - false  - The CTRDriftLinear feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsCTRDriftLIN( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_CTRLinearDriftSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel, 
	                                  uint32_t linDriftVal )

  Summary:
    Sets the linear drift value for CTR0 or CTR1.

  Description:
    This function enables the CTR module to set the linear drift value for CTR0 or CTR1 
	as specified.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	linDriftVal	- Linear drift value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t linDriftVal = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_CTRLinearDriftSet(MY_CTR_ID,ctrSel, linDriftVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRDriftLIN in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_CTRLinearDriftSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel, 
                                  uint32_t linDriftVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_CTRLinearDriftGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

   Summary:
    Reads the current linear drift value for CTR0 or CTR1.

  Description:
    This function reads the current linear drift value of CTR0 or CTR1.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current linear drift value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t linDriftVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	linDriftVal = PLIB_CTR_CTRLinearDriftGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRDriftLIN in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_CTRLinearDriftGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsCTRDriftAccuLIN( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRDriftAccuLinear feature exists on the CTR module 

  Description:
    This function identifies whether the CTRDriftAccuLinear feature is available 
	on the CTR module.
    When this function returns true, this function is supported on the device: 
    - PLIB_CTR_CTRAccuLinDriftValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRDriftAccuLinear feature is supported on the device
    - false  - The CTRDriftAccuLinear feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsCTRDriftAccuLIN( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_CTRAccuLinDriftValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

   Summary:
    Reads the current accumulator linear drift value for CTR0 or CTR1.

  Description:
    This function reads the current accumulator linear drift value of CTR0 or CTR1.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current accumulator linear drift value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t accuLinDriftVal;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	accuLinDriftVal = PLIB_CTR_CTRAccuLinDriftValueGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsCTRDriftAccuLIN in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_CTRAccuLinDriftValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsLatchTriggerSelect( CTR_MODULE_ID index )

  Summary:
    Identifies whether the LatchTriggerSelect feature exists on the CTR module. 

  Description:
    This function identifies whether the LatchTriggerSelect feature is available 
	on the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_LatchTriggerSelect
    - PLIB_CTR_LatchTriggerGet
    - PLIB_CTR_LatchCTRSelect
    - PLIB_CTR_LatchCTRGet
    - PLIB_CTR_LatchDivSet
    - PLIB_CTR_LatchDivGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LatchTriggerSelect feature is supported on the device
    - false  - The LatchTriggerSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsLatchTriggerSelect( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_LatchTriggerSelect( CTR_MODULE_ID index,CTR_LATCH_UNIT_SELECT latNum, 
	                                  CTR_LATCH_TRIGGER_SELECT latTrigSrc )

  Summary:
    Selects the trigger source value for Latch Unit.

  Description:
    This function enables the CTR module to set the latch trigger source value 
	for the specified Latch Unit. Six latches are available.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	latNum     	- Latch Unit select value
	latTrigSrc	- Latch trigger source value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t latTrigSrc = 4;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
    PLIB_CTR_LatchTriggerSelect(MY_CTR_ID,latNum, latTrigSrc);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchTriggerSelect in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_LatchTriggerSelect( CTR_MODULE_ID index,CTR_LATCH_UNIT_SELECT latNum, 
                                  CTR_LATCH_TRIGGER_SELECT latTrigSrc );

//******************************************************************************
/* Function:
    CTR_LATCH_TRIGGER_SELECT PLIB_CTR_LatchTriggerGet ( CTR_MODULE_ID index,
	                                               CTR_LATCH_UNIT_SELECT latNum)

   Summary:
    Reads the current latch trigger source value.

  Description:
    This function reads the current latch trigger source. Six latches are available.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	latNum     	- Latch Unit select value

  Returns:
    The current trigger source value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t latTrigSrc;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
	latTrigSrc = PLIB_CTR_LatchTriggerGet(MY_CTR_ID, latNum);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchTriggerSelect in your application to determine whether
	this feature is available.
*/

CTR_LATCH_TRIGGER_SELECT PLIB_CTR_LatchTriggerGet ( CTR_MODULE_ID index,
                                                 CTR_LATCH_UNIT_SELECT latNum);

//******************************************************************************
/* Function:
    void PLIB_CTR_LatchCTRSelect ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum, 
	                               CTR_LATCH_CTR_SELECT latctrVal)

  Summary:
    Selects which CTR to latch on a trigger.

  Description:
    This function enables the CTR module to select which CTR to latch on a trigger.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	latNum     	- Latch Unit select value
	latctrVal	- CTR to latch on trigger

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	CTR_LATCH_CTR_SELECT latctrVal = 4;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
    PLIB_CTR_LatchCTRSelect(MY_CTR_ID,latNum, latctrVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchTriggerSelect in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_LatchCTRSelect ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum, 
                               CTR_LATCH_CTR_SELECT latctrVal);

//******************************************************************************
/* Function:
    CTR_LATCH_CTR_SELECT PLIB_CTR_LatchCTRGet ( CTR_MODULE_ID index,
	                                              CTR_LATCH_UNIT_SELECT latNum )

  Summary:
    Reads the current CTR selected for the latch on a trigger.

  Description:
    This function reads the current CTR selected for the latch on a trigger.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	latNum     	- Latch Unit select value

  Returns:
    The current CTR selected for the latch on a trigger.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t latctrVal;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
	latctrVal = PLIB_CTR_LatchCTRGet(MY_CTR_ID, latNum);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchTriggerSelect in your application to determine whether
	this feature is available.
*/

CTR_LATCH_CTR_SELECT PLIB_CTR_LatchCTRGet ( CTR_MODULE_ID index,
                                            CTR_LATCH_UNIT_SELECT latNum );

//******************************************************************************
/* Function:
    void PLIB_CTR_LatchDivSet ( CTR_MODULE_ID index,CTR_LATCH_UNIT_SELECT latNum, 
	                            uint32_t divVal )

  Summary:
    Selects the latch trigger predivider value.

  Description:
    This function enables the CTR module to set the latch trigger predivider value.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	latNum     	- Latch Unit select value
	divVal		- Latch trigger predivider value

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t divVal = 4;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
    PLIB_CTR_LatchDivSet(MY_CTR_ID,latNum, divVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchTriggerSelect in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_LatchDivSet ( CTR_MODULE_ID index,CTR_LATCH_UNIT_SELECT latNum, 
                            uint32_t divVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_LatchDivGet ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum)

  Summary:
    Reads the current latch trigger predivider value.

  Description:
    This function reads the current latch trigger predivider value.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	latNum     	- Latch Unit select value

  Returns:
    The current latch trigger predivider value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t divVal;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
	divVal = PLIB_CTR_LatchDivGet(MY_CTR_ID, latNum);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchTriggerSelect in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_LatchDivGet ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum);
//******************************************************************************
/* Function:
    PLIB_CTR_ExistsLatchValue( CTR_MODULE_ID index )

  Summary:
    Identifies whether the LatchValue feature exists on the CTR module. 

  Description:
    This function identifies whether the LatchValue feature is available on the 
	CTR module.
    When this function returns true, this function is supported on the device: 
    - PLIB_CTR_LatchGetValue

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LatchValue feature is supported on the device
    - false  - The LatchValue feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsLatchValue( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_LatchGetValue ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum )

  Summary:
    Reads the current CTR latching result.

  Description:
    This function reads the current CTR latching result.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	latNum     	- Latch Unit select value

  Returns:
    The current CTR latching result.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t latchVal;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
	latchVal = PLIB_CTR_LatchGetValue(MY_CTR_ID, latNum);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchValue in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_LatchGetValue ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsLatchStatus( CTR_MODULE_ID index )

  Summary:
    Identifies whether the LatchStatus feature exists on the CTR module. 

  Description:
    This function identifies whether the LatchStatus feature is available on the CTR module.
    When this function returns true, this function is supported on the device: 
    - PLIB_CTR_LatchGetStatus

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LatchStatus feature is supported on the device
    - false  - The LatchStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsLatchStatus( CTR_MODULE_ID index );
//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_LatchGetStatus ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum)

  Summary:
    Reads the current number of available latched time-stamp values entries in the 
	FIFO.

  Description:
    This function reads the current number of available latched time-stamp values 
	entries in the FIFO.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	latNum     	- Latch Unit select value

  Returns:
    The current trigger source value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t latchStatus;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
	latchStatus = PLIB_CTR_LatchGetStatus(MY_CTR_ID, latNum);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchStatus in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_LatchGetStatus ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum);

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsLatchTriggerCountValue( CTR_MODULE_ID index )

  Summary:
    Identifies whether the LatchTriggerCountValue feature exists on the CTR module. 

  Description:
    This function identifies whether the LatchTriggerCountValue feature is available 
	on the CTR module.
    When this function returns true, this function is supported on the device: 
    - PLIB_CTR_LatchTriggerCountGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LatchTriggerCountValue feature is supported on the device
    - false  - The LatchTriggerCountValue feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsLatchTriggerCountValue( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_LatchTriggerCountGet ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum)

  Summary:
    Reads the actual resolution of the 100 MHz trigger.

  Description:
    This 400 MHz counter captures the actual resolution of the 100 MHz trigger.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	latNum     	- Latch Unit select value

  Returns:
    The actual resolution of the 100 MHz trigger.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t resVal;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
	resVal = PLIB_CTR_LatchTriggerCountGet(MY_CTR_ID, latNum);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLatchTriggerCountValue in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_LatchTriggerCountGet ( CTR_MODULE_ID index ,CTR_LATCH_UNIT_SELECT latNum);

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsTrigger( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRTrigger feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRTrigger feature is available on the 
	CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_TriggerSelect
    - PLIB_CTR_TriggerGet
    - PLIB_CTR_CycleOffsetValueSet
    - PLIB_CTR_CycleOffsetValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRTrigger feature is supported on the device
    - false  - The CTRTrigger feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsTrigger( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_TriggerSelect ( CTR_MODULE_ID index, CTR_LATCH_CTR_SELECT ctrTrigVal )

  Summary:
    Selects which CTR is to trigger the IRQ/GPIO.

  Description:
    This function enables the CTR module to select which CTR is to trigger the 
	IRQ/GPIO.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrTrigVal	- Latch Trigger Source value to be set

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	CTR_LATCH_CTR_SELECT ctrTrigVal=4;
	
    PLIB_CTR_TriggerSelect(MY_CTR_ID,latNum, ctrTrigVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsTrigger in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_TriggerSelect ( CTR_MODULE_ID index, CTR_LATCH_CTR_SELECT ctrTrigVal );

//******************************************************************************
/* Function:
    CTR_LATCH_CTR_SELECT PLIB_CTR_TriggerGet ( CTR_MODULE_ID index )

  Summary:
    Reads the selected CTR that will trigger the IRQ/GPIO.

  Description:
    This function reads the selected CTR that will trigger the IRQ/GPIO.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module.
	latNum     	- Latch Unit select value.

  Returns:
    The selected CTR that will trigger the IRQ/GPIO.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t ctrTrigVal;
	
	ctrTrigVal = PLIB_CTR_TriggerGet(MY_CTR_ID);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsTrigger in your application to determine whether
	this feature is available.
*/

CTR_LATCH_CTR_SELECT PLIB_CTR_TriggerGet ( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_CycleOffsetValueSet ( CTR_MODULE_ID index, uint32_t cycleOffsetVal )

  Summary:
    Sets the value for when the cycle_offset is to trigger.

  Description:
    This function enables the CTR module to specify the value for when the cycle_offset 
	is to trigger.

  Precondition:
    None.

  Parameters:
    index      		- Identifies the desired CTR module
	cycleOffsetVal	- Cycle_offset trigger value

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t cycleOffsetVal = 4;
	
    PLIB_CTR_CycleOffsetValueSet(MY_CTR_ID, cycleOffsetVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsTrigger in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_CycleOffsetValueSet ( CTR_MODULE_ID index, uint32_t cycleOffsetVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_CycleOffsetValueGet ( CTR_MODULE_ID index )

  Summary:
    Reads the cycle_offset trigger value

  Description:
    This function reads the value for when the cycle_offset is to trigger.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module

  Returns:
    The current cycle_offset trigger value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t cycleOffsetVal;
	
	cycleOffsetVal = PLIB_CTR_CycleOffsetValueGet(MY_CTR_ID, latNum);	
    </code>
	
  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsTrigger in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_CycleOffsetValueGet ( CTR_MODULE_ID index );
//******************************************************************************
/* Function :  
	bool PLIB_CTR_ExistsNValue( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRN feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRN feature is available on the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_NValueSet
    - PLIB_CTR_NValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRN feature is supported on the device
    - false  - The CTRN feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsNValue( CTR_MODULE_ID index );
//******************************************************************************
/* Function:
    void PLIB_CTR_NValueSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t valueN )

  Summary:
    Sets the N value.

  Description:
    This function enables the CTR module to set the N value, which is used to generate 
	CTR ticks from AHB clock frequencies.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	valueN		- N value

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t valueN = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_NValueSet(MY_CTR_ID,ctrSel, valueN);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsNValue in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_NValueSet ( CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t valueN );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_NValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

  Summary:
    Reads the N value.

  Description:
    This function reads the N value, which is used to generate CTR ticks from AHB 
	clock frequencies.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current N value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t valueN;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	valueN = PLIB_CTR_NValueGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsNValue in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_NValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsMValue( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRM feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRM feature is available on the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_MValueSet
    - PLIB_CTR_MValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRM feature is supported on the device
    - false  - The CTRM feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsMValue( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_MValueSet ( CTR_MODULE_ID index , CTR_SELECT ctrSel, uint32_t valueM)

  Summary:
    Sets the M value.

  Description:
    This function enables the CTR module to set the M value, which is used to 
	generate  CTR ticks from AHB clock frequencies.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	valueN		- M value

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t valueM = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_MValueSet(MY_CTR_ID,ctrSel, valueM);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsMValue in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_MValueSet ( CTR_MODULE_ID index , CTR_SELECT ctrSel, uint32_t valueM);

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_MValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel )

  Summary:
    Reads the M value.

  Description:
    This function reads the M value, which is used to generate CTR ticks from 
	AHB clock frequencies.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current M value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t valueM;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	valueM = PLIB_CTR_MValueGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsMValue in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_MValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsLSBValue( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRLSB feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRLSB feature is available on the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_LSBValueSet
    - PLIB_CTR_LSBValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRLSB feature is supported on the device
    - false  - The CTRLSB feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsLSBValue( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_LSBValueSet ( CTR_MODULE_ID index , CTR_SELECT ctrSel, 
	                            uint32_t valueLSB)

  Summary:
    Sets the LSB value.

  Description:
    This function enables the CTR module to set the LSB value, which is used to 
	generate CTR ticks from AHB clock frequencies.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)
	valueLSB	- LSB value

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t valueLSB = 4;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
    PLIB_CTR_LSBValueSet(MY_CTR_ID,ctrSel, valueLSB);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLSBValue in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_LSBValueSet ( CTR_MODULE_ID index , CTR_SELECT ctrSel, 
                            uint32_t valueLSB);

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_LSBValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel)

  Summary:
    Reads the LSB value.

  Description:
    This function reads the LSB value, which is used to generate CTR ticks from 
	AHB clock frequencies.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	ctrSel     	- CTR select value (CTR0 or CTR1)

  Returns:
    The current LSB value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t valueLSB;
	
	CTR_SELECT ctrSel = CTR_SELECT_1;
	
	valueLSB = PLIB_CTR_LSBValueGet(MY_CTR_ID, ctrSel);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsLSBValue in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_LSBValueGet ( CTR_MODULE_ID index, CTR_SELECT ctrSel);

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsInterrupt( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRM3InterruptSelect feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRM3InterruptSelect feature is available 
	on the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_IntModeLatchSelect
    - PLIB_CTR_IntModeLatchGet
    - PLIB_CTR_IntLatchSelect
    - PLIB_CTR_IntLatchGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRInterruptSelect feature is supported on the device
    - false  - The CTRInterruptSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsInterrupt( CTR_MODULE_ID index );
//******************************************************************************
/* Function:
    void PLIB_CTR_IntModeLatchSelect ( CTR_MODULE_ID index, CTR_LATCH_INT_MODE intMode, 
	                                   CTR_LATCH_UNIT_SELECT latNum )

  Summary:
    Sets the interrupt mode of the specified latch for M3.

  Description:
    This function enables the CTR module to set the interrupt mode of the specified 
	latch for M3.

  Precondition:
    None.

  Parameters:
    index      	- Identifies the desired CTR module
	intMode		- Interrupt mode for the Latch Unit
	latNum		- Latch number

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	CTR_LATCH_INT_MODE intMode = CTR_LATCH_INT_MODE_1;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
    PLIB_CTR_IntModeLatchSelect(MY_CTR_ID, intMode, latNum);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsInterrupt in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_IntModeLatchSelect ( CTR_MODULE_ID index, CTR_LATCH_INT_MODE intMode, 
                                   CTR_LATCH_UNIT_SELECT latNum );

//******************************************************************************
/* Function:
    CTR_LATCH_INT_MODE PLIB_CTR_IntModeLatchGet ( CTR_MODULE_ID index, 
	                                              CTR_LATCH_UNIT_SELECT latNum )

  Summary:
    Reads the current interrupt mode of the specified latch for M3.

  Description:
    This function reads the current interrupt mode of the specified latch for M3.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module
	latNum		- Latch number

  Returns:
    The current interrupt mode of the specified latch for M3.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t intmodeVal;
	
	CTR_LATCH_UNIT_SELECT latNum = CTR_LATCH_UNIT_SELECT_1;
	
	intmodeVal = PLIB_CTR_IntModeLatchGet(MY_CTR_ID, latNum);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsInterrupt in your application to determine whether
	this feature is available.
*/

CTR_LATCH_INT_MODE PLIB_CTR_IntModeLatchGet ( CTR_MODULE_ID index, 
                                              CTR_LATCH_UNIT_SELECT latNum);

//******************************************************************************
/* Function:
    void PLIB_CTR_IntLatchSelect ( CTR_MODULE_ID index, CTR_ENABLE_LATCH_INT_GEN 
	                               enableLatchVal)

  Summary:
    Updates the register CTRINT_M3_INT_SEL, whose bits enable the corresponding 
	latch for interrupt generation.

  Description:
    This function enables the CTR module to update the register CTRINT_M3_INT_SEL, 
	whose bits enable the corresponding latch for interrupt generation.

  Precondition:
    None.

  Parameters:
    index      			- Identifies the desired CTR module
	enableLatchVal		- value of corresponding latch for interrupt generation

  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	CTR_ENABLE_LATCH_INT_GEN enableLatchVal = CTR_ENABLE_LATCH_INT_GEN_1;
	
    PLIB_CTR_IntLatchSelect(MY_CTR_ID, enableLatchVal);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsInterrupt in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_IntLatchSelect ( CTR_MODULE_ID index, CTR_ENABLE_LATCH_INT_GEN 
                               enableLatchVal);

//******************************************************************************
/* Function:
    CTR_ENABLE_LATCH_INT_GEN PLIB_CTR_IntLatchGet ( CTR_MODULE_ID index )

  Summary:
    Reads the current latch for interrupt generation.

  Description:
    This function reads the current latch for interrupt generation.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired CTR module

  Returns:
    The current latch for interrupt generation.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1

	uint32_t enableLatchVal;
	
	enableLatchVal = PLIB_CTR_IntLatchGet(MY_CTR_ID);	
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsInterrupt in your application to determine whether
	this feature is available.
*/

CTR_ENABLE_LATCH_INT_GEN PLIB_CTR_IntLatchGet ( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsA5InterruptSelect( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRA5InterruptSelect feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRA5InterruptSelect feature is available 
	on the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_A5IntModeLatchSelect
    - PLIB_CTR_A5IntModeLatchGet
    - PLIB_CTR_A5LatchIntSelect
    - PLIB_CTR_A5LatchIntGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRA5InterruptSelect feature is supported on the device
    - false  - The CTRA5InterruptSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsA5InterruptSelect( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsSpare( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRSpare feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRSpare feature is available on the 
	CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_SpareValueGet
    - PLIB_CTR_SpareValueSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRSpare feature is supported on the device
    - false  - The CTRSpare feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsSpare( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_SpareValueSet ( CTR_MODULE_ID index, uint32_t spareVal )

  Summary:
    Sets the spare value.

  Description:
    This function enables the CTR module to set the spare value.

  Precondition:
    None.

  Parameters:
    index   	- Identifies the desired CTR module
	spareVal 	- Spare value
	
  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t spareVal = 4;
	
    PLIB_CTR_SpareValueSet( MY_CTR_ID, spareVal );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsSpare in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_SpareValueSet ( CTR_MODULE_ID index, uint32_t spareVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_SpareValueGet ( CTR_MODULE_ID index )

  Summary:
    Reads the spare value.

  Description:
    This function enables the CTR module to read the spare value.

  Precondition:
    None.

  Parameters:
    index   - Identifies the desired CTR module
	
	
  Returns:
    The current spare value.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	PLIB_CTR_SpareValueGet ( MY_CTR_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsSpare in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_SpareValueGet ( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsTestBusSelect( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRTestBusSelect feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRTestBusSelect feature is available on 
	the CTR module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_CTR_TestBusSelect
    - PLIB_CTR_TestBusGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRTestBusSelect feature is supported on the device
    - false  - The CTRTestBusSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsTestBusSelect( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_CTR_TestBusSelect ( CTR_MODULE_ID index, CTR_TEST_BUS_SELECT testBusVal )

  Summary:
    Sets the test bus for PRL CTR.

  Description:
    This function enables the CTR module to set the test bus to be used.

  Precondition:
    None.

  Parameters:
    index   	- Identifies the desired CTR module	
	testBusVal 	- Test bus value
	
  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	CTR_TEST_BUS_SELECT testBusVal = CTR_TEST_BUS_SELECT_1;
	
    PLIB_CTR_TestBusSelect ( MY_CTR_ID, testBusVal );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsTestBusSelect in your application to determine whether
	this feature is available.
*/

void PLIB_CTR_TestBusSelect ( CTR_MODULE_ID index, CTR_TEST_BUS_SELECT testBusVal );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_TestBusGet ( CTR_MODULE_ID index )

  Summary:
    Reads the currently selected test bus.

  Description:
    This function enables the CTR module to read the currently selected test bus.

  Precondition:
    None.

  Parameters:
    index   - Identifies the desired CTR module
	
  Returns:
    The selected test bus.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t testbusVal;
	
    testbusVal = PLIB_CTR_TestBusGet ( MY_CTR_ID);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsTestBusSelect in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_TestBusGet ( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    PLIB_CTR_ExistsRevision( CTR_MODULE_ID index )

  Summary:
    Identifies whether the CTRRevision feature exists on the CTR module. 

  Description:
    This function identifies whether the CTRRevision feature is available on the 
	CTR module.
    When this function returns true, this function is supported on the device: 
    - PLIB_CTR_BlockRevisionGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CTRRevision feature is supported on the device
    - false  - The CTRRevision feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_CTR_ExistsRevision( CTR_MODULE_ID index );

//******************************************************************************
/* Function:
    uint32_t PLIB_CTR_BlockRevisionGet ( CTR_MODULE_ID index )

  Summary:
    Reads the CTR block revision.

  Description:
    This function enables the CTR module to read its block revision number.

  Precondition:
    None.

  Parameters:
    index   - Identifies the desired CTR module
	
  Returns:
    None.

  Example:
    <code>
    #define MY_CTR_ID CTR_ID_1
	
	uint32_t blkRevNum;
	
    blkRevNum = PLIB_CTR_BlockRevisionGet ( MY_CTR_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_CTR_ExistsRevision in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_CTR_BlockRevisionGet ( CTR_MODULE_ID index );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _PLIB_CTR_H

/*******************************************************************************
 End of File
*/
