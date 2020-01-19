/*******************************************************************************
  Peripheral Trigger Generator (PTG) Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_ptg.h

  Summary:
    This file contains the interface definition for the PTG Peripheral Library.

  Description:
	This library provides a low-level abstraction of the PTG module on Microchip 
	microcontrollers with a convenient C language interface. It can be used to 
	simplify low-level access to the module without the necessity of interacting 
	directly with the module's registers, thus hiding differences between one 
	microcontroller variant and another.
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
#ifndef _PLIB_PTG_H
#define _PLIB_PTG_H

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
#include "peripheral/ptg/processor/ptg_processor.h"


// *****************************************************************************
// *****************************************************************************
// Section:  General Configuration
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    void PLIB_PTG_ClockSourceSelect ( PTG_MODULE_ID index,  PTG_CLK_SRC_SEL clkSrcSel )

  Summary:
    Selects the input clock source for PTG module.

  Description:
    This function allows the PTG module to select the clock source for operation.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.
	clkSrcSel  - Clock source select value.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	PTG_CLK_SRC_SEL clkSrcSel = PTG_CLK_SRC_CLKIN_1;
	
    PLIB_PTG_ClockSourceSelect ( MY_PTG_ID, clkSrcSel );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistClockSource in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_ClockSourceSelect ( PTG_MODULE_ID index, PTG_CLK_SRC_SEL clkSrcSel );


//******************************************************************************
/* Function:
    void PLIB_PTG_PrescaleSelect ( PTG_MODULE_ID index, uint8_t preScaleSel )

  Summary:
    Selects the prescale value for the clock input.

  Description:
    This function allows the PTG module to select the prescale value for clock.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired PTG module.
	preScaleSel - Prescale select value.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint8_t PreScaleSel = 1;
	
    PLIB_PTG_PrescaleSelect ( MY_PTG_ID, PreScaleSel );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistsPrescale in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_PrescaleSelect ( PTG_MODULE_ID index, uint8_t preScaleSel );


//******************************************************************************
/* Function:
    uint8_t PLIB_PTG_PrescaleGet ( PTG_MODULE_ID index )

  Summary:
    Gets the clock prescaler setting that is currently set in the prescaler 
	register.

  Description:
    This function is used to read the current clock prescaler value from 
	the prescaler register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.

  Returns:
    The current value set in the prescaler register.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	uint8_t preScale;
    
	preScale = PLIB_PTG_PrescaleGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistsPrescale in your application to determine whether
	this feature is available.
*/

uint8_t PLIB_PTG_PrescaleGet ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_PTG_TriggerPulseWidthSet ( PTG_MODULE_ID index, uint8_t trigOuputSel )

  Summary:
    Sets the Pulse Width of the PTG trigger output.

  Description:
    If the PTGOGL bits of PTGCON register are set to '0', a trigger output pulse is 
	generated with the width set in the PTGPWD bits. This function is used to set 
	the PTGPWD bits with the desired value.

  Precondition:
    None.

  Parameters:
    index        - Identifies the desired PTG module.
	trigOuputSel - Trigger output pulse width value.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	uint8_t TrigOutputMode = 4;
	
    PLIB_PTG_TriggerPulseWidthSet(MY_PTG_ID, trigOutputMode);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistTriggerPulseWidth in your application to determine whether
	this feature is available.

*/

void PLIB_PTG_TriggerPulseWidthSet ( PTG_MODULE_ID index, uint8_t trigOuputSel );


//******************************************************************************
/* Function:
    uint8_t PLIB_PTG_TriggerPulseWidthGet ( PTG_MODULE_ID index )

  Summary:
    Reads the output trigger pulse width value.

  Description:
    
	This function reads the current output trigger pulse width value from 
	the PTGPWD bits of the PTGCON register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.

  Returns:
    The current output trigger pulse width. 

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint8_t trigOutputSel;
	
    trigOutputSel = PLIB_PTG_TriggerPulseWidthGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistTriggerPulseWidth in your application to determine whether
	this feature is available.
*/

uint8_t PLIB_PTG_TriggerPulseWidthGet ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_PTG_WDTCountValueSet ( PTG_MODULE_ID index, PTG_WDT_TIMEOUT_SEL wdtTimeOutSel)

  Summary:
    Enables the Watchdog Timer (WDT) and sets the WDT time-out value.

  Description:
    This function enables the WDT module (if previously disabled), and sets the WDT 
	counter to the desired value.

  Precondition:
    None.

  Parameters:
    index         - Identifies the desired PTG module.
	wdtTimeOutSel - WDT time-out value to be set.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	PTG_WDT_TIMEOUT_SEL wdtTimeOutSel = PTG_WDT_TIMEOUT_COUNT_CYC_32;
    
	PLIB_PTG_WDTCountValueSet ( MY_PTG_ID, wdtTimeOutSel );
    </code>

  Remarks:
	If the WDT count value passed to PLIB_PTG_WDTCountValueSet is PTG_WDT_DISABLE, 
	the WDT is disabled.
	
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistWDTCount in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_WDTCountValueSet ( PTG_MODULE_ID index, PTG_WDT_TIMEOUT_SEL wdtTimeOutSel);


//******************************************************************************
/* Function:
    void PLIB_PTG_DisableWDT ( PTG_MODULE_ID index )

  Summary:
    Disables the WDT.

  Description:
    This function disables the WDT in the PTG module.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_DisableWDT ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistWDTCount in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_DisableWDT ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PTG_WDT_TIMEOUT_SEL PLIB_PTG_WDTCountValueGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current WDT count value.
	
  Description:
    This function reads the current WDT count select value from the PTGWDT 
	bits of the PTGCON register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.

  Returns:
    The current WDT count select value.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	PTG_WDT_TIMEOUT_SEL wdtCountSel;
	
    wdtCountSel = PLIB_PTG_WDTCountValueGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistWDTCount in your application to determine whether
	this feature is available.
*/

PTG_WDT_TIMEOUT_SEL PLIB_PTG_WDTCountValueGet ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_PTG_Enable ( PTG_MODULE_ID index )

  Summary:
    Enables the PTG module.

  Description:
    This function enables the PTG module.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_Enable(MY_PTG_ID);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistEnableControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_Enable ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_Disable ( PTG_MODULE_ID index )

  Summary:
    Disables the PTG module.

  Description:
    This function disables the PTG module.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_Disable(MY_PTG_ID);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistEnableControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_Disable ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_StopInIdleEnable ( PTG_MODULE_ID index )

  Summary:
    Enables the PTG module to stop when the processor enters Idle mode.

  Description:
    This function enables the PTG module to stop when the processor enters Idle
    mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_StopInIdleEnable ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistStopInIdle in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_StopInIdleEnable ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_PTG_StopInIdleDisable ( PTG_MODULE_ID index )

  Summary:
    Disables the Stop-in-Idle feature.

  Description:
    This function disables the Stop-in-Idle feature, preventing the PTG module
    from stopping when the processor enters Idle mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_StopInIdleDisable ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistStopInIdle in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_StopInIdleDisable ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_PTG_OutputTriggerToggle ( PTG_MODULE_ID index )

  Summary:
    Sets the output trigger toggle mode to toggle mode (1).

  Description:
    This function sets the output trigger toggle mode to toggle mode (1).

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_OutputTriggerToggle ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistOutputTriggerMode in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_OutputTriggerToggle ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_OutputTriggerPulse ( PTG_MODULE_ID index )

  Summary:
    Sets the output trigger toggle mode to pulse mode (0).

  Description:
    This function sets the output trigger toggle mode to pulse mode (0).

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_OutputTriggerPulse ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistOutputTriggerMode in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_OutputTriggerPulse ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_SWTEdgeTrigger ( PTG_MODULE_ID index )

  Summary:
    Generates a low-to-high transition on the SWT bit of the PTGCON register.

  Description:
    This function generates a low-to-high transition on the SWT bit of the PTGCON 
	register.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_SWTEdgeTrigger ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistSWTControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_SWTEdgeTrigger ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_SWTLevelTrigger ( PTG_MODULE_ID index )

  Summary:
    Sets the SWT bit of the PTGCON register.

  Description:
    This function sets the SWT bit of the PTGCON register.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_SWTLevelTrigger ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistSWTControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_SWTLevelTrigger ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_SWTClear ( PTG_MODULE_ID index )

  Summary:
    Clears the SWT bit of the PTGCON register.

  Description:
    This function clears the SWT bit of the PTGCON register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_SWTClear ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistSWTControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_SWTClear ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    bool PLIB_PTG_SWTGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current status of the SWT bit in the PTGCON register.

  Description:
    This function reads the current status of the SWT bit in the PTGCON register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	bool SWT_Status = FALSE;
    
	SWT_Status = PLIB_PTG_SWTGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistSWTControl in your application to determine whether
	this feature is available.
*/

bool PLIB_PTG_SWTGet ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_PTG_SingleStepEnable ( PTG_MODULE_ID index )

  Summary:
    Enables the Single-stepping feature on the PTG module.

  Description:
    This function enables the Single-stepping feature on the PTG module for 
	debugging purposes.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_SingleStepEnable ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistSingleStepControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_SingleStepEnable ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_SingleStepDisable ( PTG_MODULE_ID index )

  Summary:
    Disables the Single-stepping feature on the PTG module.

  Description:
    This function disables the Single-stepping feature on the PTG module for 
	debugging purposes.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_SingleStepDisable ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistSingleStepControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_SingleStepDisable ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_VisiblityEnable ( PTG_MODULE_ID index )

  Summary:
    Sets the Visibility bit, IVIS, in the PTGCON register.

  Description:
    This function sets the Visibility bit, IVIS, in the TGCON register so that 
	when the timer/counter limit register is read, the current internal timer/counter 
	values are read.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_VisiblityEnable ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistVisibilityControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_VisiblityEnable ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_VisiblityDisable ( PTG_MODULE_ID index )

  Summary:
    This function clears the Visibility bit, IVIS, in the PTGCON register.

  Description:
    This function clears the Visibility bit, IVIS, in the PTGCON register so that 
	when timer/counter limit register is read, initial values are read and not the 
	current timer/counter values.    

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_VisiblityDisable ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistVisibilityControl in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_VisiblityDisable ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_ExecutionStart ( PTG_MODULE_ID index )

  Summary:
    Enables the PTG to start executing the commands stored in PTG Step register.

  Description:
    This function enables the PTG to start executing the commands stored in 
	PTG Step register.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_ExecutionStart ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistStartExecution in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_ExecutionStart ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_ExecutionHalt ( PTG_MODULE_ID index )

  Summary:
    Clears the STRT bit in the PTGCON register.
	
  Description:
    This function clears the STRT bit in the PTGCON register so that the PTG module 
	will stop executing the commands in the Step register.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

    PLIB_PTG_Halt ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistStartExecution in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_ExecutionHalt ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_PTG_WDTStatusGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current status of the WDT.
	
  Description:
    This function reads the current status of the WDT from the PTGWDTO bits in 
	the PTGCON register.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    The current status of the WDT.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	bool WDTStatus;
	
    WDTStatus = PLIB_PTG_WDTStatusGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistWDTStatus in your application to determine whether
	this feature is available.
*/

bool PLIB_PTG_WDTStatusGet ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_WDTStatusClear ( PTG_MODULE_ID index )

  Summary:
    Clears the status bit of the WDT.
	
  Description:
    This function clears the status bit of the WDT in the PTGWDTO bits in the 
	PTGCON register.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    None.
  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
		
    PLIB_PTG_WDTStatusClear ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistWDTStatus in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_WDTStatusClear ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    bool PLIB_PTG_IsBusy ( PTG_MODULE_ID index )

  Summary:
    Reads the PTG Busy Status from the PTGCON register.

  Description:
    This function reads the PTG Busy Status from the PTGBUSY bits of the PTGCON 
	register.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.

  Returns:
    PTG Busy Status.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	bool PTGBusyStatus = FALSE;
    
	PTGBusyStatus = PLIB_PTG_IsBusy ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistPTGBusyStatus in your application to determine whether
	this feature is available.
*/

bool PLIB_PTG_IsBusy ( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_PTG_InputTriggerModeSelect ( PTG_MODULE_ID index, PTG_INPUT_MODE InputTrigMode )

  Summary:
    Sets the input trigger mode to the desired value.

  Description:
    This function sets the input trigger mode to the desired value.

  Precondition:
    None.

  Parameters:
    index         - Identifies the desired PTG module.
	InputTrigMode - Input mode value to be set.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	PTG_INPUT_MODE InputTrigMode = PTG_INPUT_MODE_3;
	
    PLIB_PTG_InputTriggerModeSelect ( MY_PTG_ID, InputTrigMode );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistInputTriggerMode in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_InputTriggerModeSelect ( PTG_MODULE_ID index, PTG_INPUT_MODE InputTrigMode );

//******************************************************************************
/* Function:
    PTG_INPUT_MODE PLIB_PTG_InputTriggerModeGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current input trigger mode.

  Description:
    This function reads the current input trigger mode read from the PTGITM 
	bits of the PTGCON register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.

  Returns:
    The current input trigger mode.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	PTG_INPUT_MODE InputTrigMode;
	
    InputTrigMode = PLIB_PTG_InputTriggerModeGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistInputTriggerMode in your application to determine whether
	this feature is available.
*/

PTG_INPUT_MODE PLIB_PTG_InputTriggerModeGet ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_TriggerBroadcastMaskSet ( PTG_MODULE_ID index , uint32_t broadcastMask )

  Summary:
    Sets the trigger broadcast mask.

  Description:
    This function sets the trigger broadcast mask in the PTGBTE register.

  Precondition:
    None.

  Parameters:
    index         - Identifies the desired PTG module.
	broadcastMask - BroadcastMask to be set.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint32_t broadcastMask = 0x0000FE00;
	
    PLIB_PTG_TriggerBroadcastMaskSet ( MY_PTG_ID, broadcastMask );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistTriggerBroadcastMask in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_TriggerBroadcastMaskSet ( PTG_MODULE_ID index , uint32_t broadcastMask );

//******************************************************************************
/* Function:
    uint32_t PLIB_PTG_TriggerBroadcastMaskGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current trigger broadcast mask.

  Description:
    This function reads the current trigger broadcast mask in the PTGBTE 
	register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.
	
  Returns:
    The current trigger broadcast mask.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint32_t broadcastMask = 0x00;
	
    broadcastMask = PLIB_PTG_TriggerBroadcastMaskGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistTriggerBroadcastMask in your application to determine whether
	this feature is available.
*/

uint32_t PLIB_PTG_TriggerBroadcastMaskGet ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_HoldValueSet ( PTG_MODULE_ID index , uint16_t holdValue )

  Summary:
    Initializes the PTGHOLD register with the desired hold value.

  Description:
    This function initializes the PTGHOLD register with the desired hold value.

  Precondition:
    None.

  Parameters:
    index     - Identifies the desired PTG module.
	holdValue - Hold value to be set.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint16_t holdValue = 0xFFFF;
	
    PLIB_PTG_HoldValueSet ( MY_PTG_ID, holdValue );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistsHoldValue in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_HoldValueSet ( PTG_MODULE_ID index , uint16_t holdValue );

//******************************************************************************
/* Function:
    uint16_t PLIB_PTG_HoldValueGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current hold value.

  Description:
    This function reads the current hold value from the PTGHOLD register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.
	
  Returns:
    The current hold value.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint16_t holdValue = 0x00;
	
    holdValue = PLIB_PTG_HoldValueGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistsHoldValue in your application to determine whether
	this feature is available.
*/

uint16_t PLIB_PTG_HoldValueGet ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_TimerLimitSet ( PTG_MODULE_ID index, PTG_TIMER_SEL timerSel, uint16_t timerLimitValue )

  Summary:
    Initializes the Timer limit register.

  Description:
    This function initializes the Timer Limit register selected by timerSel.

  Precondition:
    None.

  Parameters:
    index           - Identifies the desired PTG module.
	timerSel        - Internal Timer selection.
	timerLimitValue - Limit value.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	uint16_t TimerLimit = 0xFF00;
	
    PLIB_PTG_TimerLimitSet ( MY_PTG_ID, TimerLimit );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistTimerLimit in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_TimerLimitSet ( PTG_MODULE_ID index, PTG_TIMER_SEL timerSel, uint16_t timerLimitValue );

//******************************************************************************
/* Function:
    uint16_t PLIB_PTG_TimerLimitGet ( PTG_MODULE_ID index, PTG_TIMER_SEL timerSel)

  Summary:
    Reads the selected timer limit register.

  Description:
    This function reads the timer limit register selected by the parameter timerSel.

  Precondition:
    None.

  Parameters:
    index    - Identifies the desired PTG module.
	timerSel - Internal timer selection.
	
  Returns:
    The limit register content of the timer selected by the parameter timerSel.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	uint16_t timerLimit = 0x00;
	PTG_TIMER_SEL timerSel = PTG_TIMER_0;
	
    timerLimit = PLIB_PTG_TimerLimitGet ( MY_PTG_ID, timerSel );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistTimerLimit in your application to determine whether
	this feature is available.
*/

uint16_t PLIB_PTG_TimerLimitGet ( PTG_MODULE_ID index, PTG_TIMER_SEL timerSel);

//******************************************************************************
/* Function:
    void PLIB_PTG_StepDelaySet ( PTG_MODULE_ID index, uint16_t stepDelayLimit )

  Summary:
    Initializes the step delay limit register.

  Description:
    This function initializes the step delay limit register.
	
  Precondition:
    None.

  Parameters:
    index          - Identifies the desired PTG module.
	stepDelayLimit - Limit value.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	uint16_t stepDelayLimit = 0xFF00;
	
    PLIB_PTG_StepDelaySet ( MY_PTG_ID, stepDelayLimit );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistStepDelay in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_StepDelaySet ( PTG_MODULE_ID index, uint16_t stepDelayLimit );

//******************************************************************************
/* Function:
    uint16_t PLIB_PTG_StepDelayGet ( PTG_MODULE_ID index )

  Summary:
    Reads the step delay limit register.

  Description:
    This function reads the step delay limit register.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired PTG module.
	
  Returns:
    Step delay limit register content.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	uint16_t stepDelayLimit = 0x00;
	
    stepDelayLimit = PLIB_PTG_StepDelayGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistStepDelay in your application to determine whether
	this feature is available.
*/

uint16_t PLIB_PTG_StepDelayGet ( PTG_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_PTG_CounterLimitSet ( PTG_MODULE_ID index, PTG_COUNTER_SEL counterSel, uint16_t counterLimit )

  Summary:
    Initializes the limit register register of the counter.

  Description:
    This function initializes the limit register register of the counter 
	selected using the parameter counterSel.

  Precondition:
    None.

  Parameters:
    index        - Identifies the desired PTG module.
	counterSel   - Internal counter selection.
	counterLimit - Limit value.

  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	uint16_t counterLimit = 0x0F00;
	PTG_COUNTER_SEL counterSel = PTG_COUNTER_0;
	
    PLIB_PTG_CounterLimitSet ( MY_PTG_ID, counterSel, counterLimit );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistCounterLimit in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_CounterLimitSet ( PTG_MODULE_ID index, PTG_COUNTER_SEL counterSel, uint16_t counterLimit );

//******************************************************************************
/* Function:
    uint16_t PLIB_PTG_CounterLimitGet ( PTG_MODULE_ID index, PTG_COUNTER_SEL counterSel )

  Summary:
    Reads the counter limit register.

  Description:
    This function reads the limit register content of the counter selected by 
	the parameter counterSel.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired PTG module.
	counterSel - Internal counter selection.
	
  Returns:
    The limit register content of the counter selected by the parameter counterSel.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	
	uint16_t counter0Limit = 0x00;
	PTG_COUNTER_SEL counterSel = PTG_COUNTER_0;
	
    counter0Limit = PLIB_PTG_CounterLimitGet ( MY_PTG_ID, counterSel );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistsCounterLimit in your application to determine whether
	this feature is available.
*/

uint16_t PLIB_PTG_CounterLimitGet ( PTG_MODULE_ID index, PTG_COUNTER_SEL counterSel );

//******************************************************************************
/* Function:
    void PLIB_PTG_AdjustValueSet ( PTG_MODULE_ID index , uint16_t adjustValue )

  Summary:
    Initializes the PTGADJ register with the desired adjust value.

  Description:
    This function initializes the PTGADJ register with the desired adjust value.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired PTG module.
	adjustValue - adjust value to be set.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint16_t adjustValue = 0xFFFF;
	
    PLIB_PTG_AdjustValueSet ( MY_PTG_ID, adjustValue );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistAdjustValue in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_AdjustValueSet ( PTG_MODULE_ID index , uint16_t adjustValue );

//******************************************************************************
/* Function:
    uint16_t PLIB_PTG_AdjustValueGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current adjust value from the PTGADJ register.

  Description:
    This function reads the current adjust value from the PTGADJ register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.
	
  Returns:
    The current adjust value from PTGADJ register.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint16_t adjustValue = 0x00;
	
    adjustValue = PLIB_PTG_AdjustValueGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistAdjustValue in your application to determine whether
	this feature is available.
*/

 uint16_t PLIB_PTG_AdjustValueGet ( PTG_MODULE_ID index );

 //******************************************************************************
/* Function:
    void PLIB_PTG_LiteralStrobeValueSet ( PTG_MODULE_ID index , unsigned int strobeValue )

  Summary:
    Initializes the literal strobe PTGL0 register with the desired value to be 
	strobed in the PTG data bus.

  Description:
    This function initializes the literal strobe PTGL0 register with the desired 
	value to be strobed in the PTG data bus.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired PTG module.
	strobeValue - Strobe value.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint16_t strobeValue = 0xFFFF;
	
    PLIB_PTG_LiteralStrobeValueSet ( MY_PTG_ID, strobeValue );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistLiteralStrobe in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_LiteralStrobeValueSet ( PTG_MODULE_ID index , uint16_t strobeValue );

//******************************************************************************
/* Function:
    uint16_t PLIB_PTG_LiteralStrobeValueGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current literal strobe value.

  Description:
    This function reads the current literal strobe value from the PTGL0 register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.
	
  Returns:
    The current strobe value.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint16_t strobeValue = 0x00;
	
    strobeValue = PLIB_PTG_LiteralStrobeValueGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistLiteralStrobe in your application to determine whether
	this feature is available.
*/

 uint16_t PLIB_PTG_LiteralStrobeValueGet ( PTG_MODULE_ID index );
 
 //******************************************************************************
/* Function:
    void PLIB_PTG_QueuePointerSet ( PTG_MODULE_ID index , uint8_t queueLoc )

  Summary:
    Initializes the queue pointer register PTGQPTR with the specified location.

  Description:
    This function initializes the queue pointer register PTGQPTR with the 
	specified location.

  Precondition:
    None.

  Parameters:
    index    - Identifies the desired PTG module.
	queueLoc - Location to be pointed.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	#define STEP_5 0x05
	
	uint8_t queueLoc = STEP_5;
	
    PLIB_PTG_QueuePointerSet ( MY_PTG_ID, queueLoc );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistQueuePointer in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_QueuePointerSet ( PTG_MODULE_ID index , uint8_t queueLoc );

//******************************************************************************
/* Function:
    uint8_t PLIB_PTG_QueuePointerGet ( PTG_MODULE_ID index )

  Summary:
    Reads the current location indicated by the PTGQPTR register.

  Description:
    This function reads the current location indicated by the PTGQPTR register.

  Precondition:
    None.

  Parameters:
    index - Identifies the desired PTG module.
	
  Returns:
    The current location indicated by the PTGQPTR register.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0

	uint8_t queueLoc = 0x00;
	
    queueLoc = PLIB_PTG_QueuePointerGet ( MY_PTG_ID );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistQueuePointer in your application to determine whether
	this feature is available.
*/

 uint8_t PLIB_PTG_QueuePointerGet ( PTG_MODULE_ID index );

 //******************************************************************************
/* Function:
    void PLIB_PTG_StepCommandSet ( PTG_MODULE_ID index, uint8_t stepLoc, uint8_t command )

  Summary:
    Stores the step command in the specified step location.

  Description:
    This function stores the step command in the specified step location.

  Precondition:
    None.

  Parameters:
    index   - Identifies the desired PTG module.
	stepLoc - location where the command to be stored.
	command - command to be stored in step location.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	#define STEP_0 (0x00)
	#define PTGHI (0x04)
	#define TRIG_INPUT_0 (0x00)
	
	uint8_t stepLoc = STEP_0;
	uint8_t command = (PTGHI<<4)|TRIG_INPUT_0;
	
    PLIB_PTG_StepCommandSet ( MY_PTG_ID, stepLoc, command );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistStepCommand in your application to determine whether
	this feature is available.
*/

void PLIB_PTG_StepCommandSet ( PTG_MODULE_ID index, uint8_t stepLoc, uint8_t command );


//******************************************************************************
/* Function:
    uint8_t PLIB_PTG_StepCommandGet ( PTG_MODULE_ID index, uint8_t stepLoc)

  Summary:
    Reads the command currently stored in the specified step location.

  Description:
    This function reads the command currently stored in the specified step location.

  Precondition:
    None.

  Parameters:
    index   - Identifies the desired PTG module.
	stepLoc - Step location to be read.
	
  Returns:
    None.

  Example:
    <code>
    #define MY_PTG_ID PTG_ID_0
	#define STEP_1 (0x01)
	
	uint8_t Command;
	
    Command = PLIB_PTG_StepCommandGet ( MY_PTG_ID, STEP_1 );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_PTG_ExistStepCommand in your application to determine whether
	this feature is available.
*/

uint8_t PLIB_PTG_StepCommandGet ( PTG_MODULE_ID index, uint8_t stepLoc);

// *****************************************************************************
// *****************************************************************************
// Section: PTG Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************
/* The following functions indicate the existence of the features on the device. 
*/

//******************************************************************************
/* Function:
    PLIB_PTG_ExistsClockSource( PTG_MODULE_ID index )

  Summary:
    Identifies whether the ClockSource feature exists on the PTG module. 

  Description:
    This function identifies whether the ClockSource feature is available on the 
	PTG module.
    When this function returns true, this function is supported on the device: 
    - PLIB_PTG_ClockSourceSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ClockSource feature is supported on the device
    - false  - The ClockSource feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsClockSource( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsPrescale( PTG_MODULE_ID index )

  Summary:
    Identifies whether the Prescale feature exists on the PTG module. 

  Description:
    This function identifies whether the Prescale feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_PrescaleSelect
    - PLIB_PTG_PrescaleGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Prescale feature is supported on the device
    - false  - The Prescale feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsPrescale( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsTriggerPulseWidth( PTG_MODULE_ID index )

  Summary:
    Identifies whether the TriggerPulseWidth feature exists on the PTG module. 

  Description:
    This function identifies whether the TriggerPulseWidth feature is available 
	on the PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_TriggerPulseWidthSet
    - PLIB_PTG_TriggerPulseWidthGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TriggerPulseWidth feature is supported on the device
    - false  - The TriggerPulseWidth feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsTriggerPulseWidth( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsWDTCount( PTG_MODULE_ID index )

  Summary:
    Identifies whether the WDTCount feature exists on the PTG module. 

  Description:
    This function identifies whether the WDTCount feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_WDTCountValueSet
    - PLIB_PTG_DisableWDT
    - PLIB_PTG_WDTCountValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The WDTCount feature is supported on the device
    - false  - The WDTCount feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsWDTCount( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsEnableControl( PTG_MODULE_ID index )

  Summary:
    Identifies whether the EnableControl feature exists on the PTG module. 

  Description:
    This function identifies whether the EnableControl feature is available on 
	the PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_Enable
    - PLIB_PTG_Disable

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

bool PLIB_PTG_ExistsEnableControl( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsStopInIdle( PTG_MODULE_ID index )

  Summary:
    Identifies whether the StopInIdle feature exists on the PTG module. 

  Description:
    This function identifies whether the StopInIdle feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_StopInIdleEnable
    - PLIB_PTG_StopInIdleDisable

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

bool PLIB_PTG_ExistsStopInIdle( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsOutputTriggerMode( PTG_MODULE_ID index )

  Summary:
    Identifies whether the OutputTriggerMode feature exists on the PTG module. 

  Description:
    This function identifies whether the OutputTriggerMode feature is available 
	on the PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_OutputTriggerToggle
    - PLIB_PTG_OutputTriggerPulse

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OutputTriggerMode feature is supported on the device
    - false  - The OutputTriggerMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsOutputTriggerMode( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsSWTControl( PTG_MODULE_ID index )

  Summary:
    Identifies whether the SWTControl feature exists on the PTG module. 

  Description:
    This function identifies whether the SWTControl feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_SWTEdgeTrigger
    - PLIB_PTG_SWTLevelTrigger
    - PLIB_PTG_SWTClear
    - PLIB_PTG_SWTGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The SWTControl feature is supported on the device
    - false  - The SWTControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsSWTControl( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsSingleStepControl( PTG_MODULE_ID index )

  Summary:
    Identifies whether the SingleStepControl feature exists on the PTG module. 

  Description:
    This function identifies whether the SingleStepControl feature is available 
	on the PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_SingleStepEnable
    - PLIB_PTG_SingleStepDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The SingleStepControl feature is supported on the device
    - false  - The SingleStepControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsSingleStepControl( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsVisibilityControl( PTG_MODULE_ID index )

  Summary:
    Identifies whether the VisibilityControl feature exists on the PTG module. 

  Description:
    This function identifies whether the VisibilityControl feature is available 
	on the PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_VisiblityEnable
    - PLIB_PTG_VisiblityDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The VisibilityControl feature is supported on the device
    - false  - The VisibilityControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsVisibilityControl( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsStartExecution( PTG_MODULE_ID index )

  Summary:
    Identifies whether the StartExecution feature exists on the PTG module. 

  Description:
    This function identifies whether the StartExecution feature is available on 
	the PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_ExecutionStart
    - PLIB_PTG_ExecutionHalt

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The StartExecution feature is supported on the device
    - false  - The StartExecution feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsStartExecution( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsWDTStatus( PTG_MODULE_ID index )

  Summary:
    Identifies whether the WDTStatus feature exists on the PTG module. 

  Description:
    This function identifies whether the WDTStatus feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_WDTStatusGet
    - PLIB_PTG_WDTStatusClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The WDTStatus feature is supported on the device
    - false  - The WDTStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsWDTStatus( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsPTGBusyStatus( PTG_MODULE_ID index )

  Summary:
    Identifies whether the PTGBusyStatus feature exists on the PTG module. 

  Description:
    This function identifies whether the PTGBusyStatus feature is available on the 
	PTG module.
    When this function returns true, this function is supported on the device: 
    - PLIB_PTG_PTGBusyStatusGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PTGBusyStatus feature is supported on the device
    - false  - The PTGBusyStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsPTGBusyStatus( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsInputTriggerMode( PTG_MODULE_ID index )

  Summary:
    Identifies whether the InputTriggerMode feature exists on the PTG module. 

  Description:
    This function identifies whether the InputTriggerMode feature is available on 
	the PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_InputTriggerModeSelect
    - PLIB_PTG_InputTriggerModeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The InputTriggerMode feature is supported on the device
    - false  - The InputTriggerMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsInputTriggerMode( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsTriggerBroadcastMask( PTG_MODULE_ID index )

  Summary:
    Identifies whether the TriggerBroadcastMask feature exists on the PTG module. 

  Description:
    This function identifies whether the TriggerBroadcastMask feature is available 
	on the PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_TriggerBroadcastMaskSet
    - PLIB_PTG_TriggerBroadcastMaskGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TriggerBroadcastMask feature is supported on the device
    - false  - The TriggerBroadcastMask feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsTriggerBroadcastMask( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsHoldValue( PTG_MODULE_ID index )

  Summary:
    Identifies whether the HoldValue feature exists on the PTG module. 

  Description:
    This function identifies whether the HoldValue feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_HoldRegisterInitialize
    - PLIB_PTG_HoldRegisterGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HoldValue feature is supported on the device
    - false  - The HoldValue feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsHoldValue( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsTimerLimit( PTG_MODULE_ID index )

  Summary:
    Identifies whether the TimerLimit feature exists on the PTG module. 

  Description:
    This function identifies whether the TimerLimit feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_TimerLimitRegisterInitialize
    - PLIB_PTG_TimerLimitRegisterGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TimerLimit feature is supported on the device
    - false  - The TimerLimit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsTimerLimit( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsStepDelay( PTG_MODULE_ID index )

  Summary:
    Identifies whether the StepDelay feature exists on the PTG module. 

  Description:
    This function identifies whether the StepDelay feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_StepDelayInitialize
    - PLIB_PTG_StepDelayGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The StepDelay feature is supported on the device
    - false  - The StepDelay feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsStepDelay( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsCounterLimit( PTG_MODULE_ID index )

  Summary:
    Identifies whether the CounterLimit feature exists on the PTG module. 

  Description:
    This function identifies whether the CounterLimit feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_CounterLimitRegisterInitialize
    - PLIB_PTG_CounterLimitRegisterGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CounterLimit feature is supported on the device
    - false  - The CounterLimit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsCounterLimit( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsAdjustValue( PTG_MODULE_ID index )

  Summary:
    Identifies whether the AdjustValue feature exists on the PTG module. 

  Description:
    This function identifies whether the AdjustValue feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_AdjustValueInitialize
    - PLIB_PTG_AdjustValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AdjustValue feature is supported on the device
    - false  - The AdjustValue feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsAdjustValue( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsLiteralStrobe( PTG_MODULE_ID index )

  Summary:
    Identifies whether the LiteralStrobe feature exists on the PTG module. 

  Description:
    This function identifies whether the LiteralStrobe feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_LiteralStrobeValueInitialize
    - PLIB_PTG_LiteralStrobeValueGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LiteralStrobe feature is supported on the device
    - false  - The LiteralStrobe feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsLiteralStrobe( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsQueuePointer( PTG_MODULE_ID index )

  Summary:
    Identifies whether the QueuePointer feature exists on the PTG module. 

  Description:
    This function identifies whether the QueuePointer feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_QueuePointerInitialize
    - PLIB_PTG_QueuePointerGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The QueuePointer feature is supported on the device
    - false  - The QueuePointer feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsQueuePointer( PTG_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_PTG_ExistsStepCommand( PTG_MODULE_ID index )

  Summary:
    Identifies whether the StepCommand feature exists on the PTG module. 

  Description:
    This function identifies whether the StepCommand feature is available on the 
	PTG module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_PTG_StepCommandSet
    - PLIB_PTG_StepCommandGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The StepCommand feature is supported on the device
    - false  - The StepCommand feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_PTG_ExistsStepCommand( PTG_MODULE_ID index );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _PLIB_PTG_H
/*******************************************************************************
 End of File
*/