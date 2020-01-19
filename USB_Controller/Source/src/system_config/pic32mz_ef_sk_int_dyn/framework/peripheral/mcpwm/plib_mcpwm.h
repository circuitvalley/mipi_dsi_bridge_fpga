/******************************************************************************
  MCPWM Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_mcpwm.h

  Summary:
    MCPWM PLIB interface header for definitions common to the MCPWM peripheral.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the MCPWM
    PLIB for all families of Microchip microcontrollers. The definitions in
    this file are common to the MCPWM peripheral.
******************************************************************************/

// DOM-IGNORE-BEGIN
/******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _PLIB_MCPWM_H
#define _PLIB_MCPWM_H

#include <stdint.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued at end of file)
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.  However,
    please see the bottom of the file for additional implementation header files
    that are also included
*/

#include "peripheral/mcpwm/processor/mcpwm_processor.h"


// *****************************************************************************
// *****************************************************************************
// Section: MCPWM Peripheral Library Interface Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_Enable( MCPWM_MODULE_ID index )

  Summary:
    MCPWM module is enabled (turned ON).

  Description:
    This function enables (turns ON) the MCPWM module.

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	PLIB_MCPWM_Enable(MY_MCPWM_INSTANCE);
    </code>

  Remarks: 
	This feature may not be available on all devices. Please refer to the  
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsModuleControl in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_Enable( MCPWM_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_Disable( MCPWM_MODULE_ID index )

  Summary:
    MCPWM module is disabled (turned OFF).

  Description:
    This function disables (turns OFF) the MCPWM module.

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
	 // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
    PLIB_MCPWM_Disable( MY_MCPWM_INSTANCE );
    </code>

  Remarks: 
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsModuleControl in your application to determine whether this feature
	is available.

*/

void PLIB_MCPWM_Disable( MCPWM_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_StopInIdleEnable( MCPWM_MODULE_ID index )

  Summary:
    Halt MCPWM TIMEBASE  operation when device enters Idle mode.

  Description:
    This function halts MCPWM TIMEBASE  operation when the device enters Idle
    mode.

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
    PLIB_MCPWM_StopInIdleEnable(MY_MCPWM_INSTANCE);
    </code>

  Remarks: 
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsModuleControl in your application to determine whether this feature
	is available.

*/

void PLIB_MCPWM_StopInIdleEnable( MCPWM_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_StopInIdleDisable( MCPWM_MODULE_ID index )

  Summary:
    Continue MCPWM TIMEBASE  operation when the device is in Idle mode.

  Description:
    This function enables the MCPWM TIMEBASE to continue operation when the device
    is in Idle mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
    PLIB_MCPWM_StopInIdleDisable(MY_MCPWM_INSTANCE);
    </code>

  Remarks: 
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsModuleControl in your application to determine whether this feature
	is available.

*/

void PLIB_MCPWM_StopInIdleDisable( MCPWM_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_PrimarySpecialEventTriggerSetup( MCPWM_MODULE_ID index , 
													uint16_t compare_value , 
													MCPWM_TRIGGER_DIVIDER 
													postscaler_value ,
													bool interrupt_status )

  Summary:
    Setup Primary Special Event Trigger

  Description:
    This function performs following setup functions for Primary Special Event
	1. Sets the compare match value
	2. Sets the Primary Special Event Trigger Output Postscaler

    	

  Precondition:
    The MCPWM Module must be disabled before calling this function.

  Parameters:
    index            -	Identifier for the device instance to be configured
    
	compare_value	 - 	16 bit Compare value (in MCPWM Primary Timer counts) which is 
	                    compared against Primary Master Time Base to generate a 
						match event.
						0<=compare_value<=65535
	
	postscaler_value -	Trigger Post scaler value

	

	Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// Compare match event occurs at MCPWM Primary Timer (PMTMR) = 10000
	// Primary Special Event Trigger is generated at every 4th compare match event.
	
    PLIB_MCPWM_PrimarySpecialEventTriggerSetup(MY_MCPWM_INSTANCE , 10000 , 4 );
    </code>

   Remarks: 
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsPrimarySpecialEventTrigger in your application to determine whether this feature
	is available.

*/


void PLIB_MCPWM_PrimarySpecialEventTriggerSetup( MCPWM_MODULE_ID index ,
                                                 uint16_t compare_value , 
												 MCPWM_TRIGGER_DIVIDER 
												 postscaler_value );

// *****************************************************************************
/* Function:
    bool PLIB_MCPWM_PrimarySpecialEventTriggerInterruptIsPending( MCPWM_MODULE_ID index );

  Summary:
    Returns the interrupt status for Primary Special Event Interrupt.

  Description:
    This function identifies if the Primary Special Event Interrupt is pending or not.

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured
    

  Returns:
    -true	- Primary Special Event Interrupt is pending.
	-false	- Primary Special Event Interrupt is not pending.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	bool interruptStatus;
   	interruptStatus = PLIB_MCPWM_PrimarySpecialEventTriggerInterruptIsPending(MY_MCPWM_INSTANCE);
    </code>

  Remarks: 
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsPrimarySpecialEventTrigger in your application to determine whether this feature
	is available.

*/

bool PLIB_MCPWM_PrimarySpecialEventTriggerInterruptIsPending( MCPWM_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_PrimarySpecialEventTriggerInterruptEnable( MCPWM_MODULE_ID index )

  Summary:
    Enables the Primary Special Event Trigger Interrupt.

  Description:
    This function enables Primary Special Event Interrupt at the MCPWM module level. 
	
	When the Primary Special Event Trigger Interrupt bit is enabled, an interrupt 
	is generated at compare match between Primary MCPWM Timer (PMTMR) and Primary 
	Special Event Compare Register (SEVTCMP)
	
	In order to fully enable the Primary Special Event Interrupt generation, the 
	corresponding interrupt must be enabled in the Interrupt Controller. Please 
	refer to Interrupt Peripheral Library for more information on PLIB APIs to 
	enable interrupts in the Interrupt Controller.  

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
    PLIB_MCPWM_PrimarySpecialEventTriggerInterruptEnable(MY_MCPWM_INSTANCE);
    </code>

   Remarks: 
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsPrimarySpecialEventTrigger in your application to determine whether this feature
	is available.

*/

void PLIB_MCPWM_PrimarySpecialEventTriggerInterruptEnable( MCPWM_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_PrimarySpecialEventTriggerInterruptDisable( MCPWM_MODULE_ID index )

  Summary:
    Disables the Primary Special Event Trigger Interrupt.

  Description:
    This function disables Primary Special Event Interrupt at the module level.

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
    PLIB_MCPWM_PrimarySpecialEventTriggerInterruptDisable(MY_MCPWM_INSTANCE);
    </code>

   Remarks: 
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsPrimarySpecialEventTrigger in your application to determine whether this feature
	is available.

*/

void PLIB_MCPWM_PrimarySpecialEventTriggerInterruptDisable( MCPWM_MODULE_ID index );

// *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ModuleIsReady( MCPWM_MODULE_ID index );

  Summary:
    Returns the status for MCPWM Module.

  Description:
    This function returns the status of MCPWM module

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured
    

  Returns:
    -true	- MCPWM module is ready and operation has begun.
	-false	- MCPWM module is not ready.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
   	bool mcpwm_status;
	mcpwm_status = PLIB_MCPWM_ModuleIsReady(MY_MCPWM_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsModuleControl in your application to determine whether this feature
	is available.
*/

bool PLIB_MCPWM_ModuleIsReady( MCPWM_MODULE_ID index );





// *****************************************************************************
/* Function:
    void PLIB_MCPWM_PrimaryTimerSetup( MCPWM_MODULE_ID index , MCPWM_CLOCK_DIVIDER clock_div ,
                                       uint16_t period_value )

  Summary:
    Setup Primary Master PWM Timer

  Description:
    This function performs following setup functions for Primary MCPWM Timer.
	1. Sets the clock divider value by which the SYS_CLOCK is divided to generate the clock 
	   source for Primary MCPWM timer
	2. Sets the period value for the secondary timer

    	

  Precondition:
    None.

  Parameters:
    index            -	Identifier for the device instance to be configured
    
	clock_div		 -	Clock Divider 
	
	period_value	 - 	16 bit Period value (in MCPWM timer counts) at which the Seondary MCPWM Timer resets.
	                    8<=period_value<=65535
	

	Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// Primary MCPWM Timer clock = SYS_CLOCK / 4
	// Primary Timer Period = 10000
	
	
    PLIB_MCPWM_PrimaryTimerSetup(MY_MCPWM_INSTANCE , DIV_4 ,10000 );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsPrimaryTimerSetup in your application to determine whether this feature
	is available.
*/


void PLIB_MCPWM_PrimaryTimerSetup( MCPWM_MODULE_ID index , MCPWM_CLOCK_DIVIDER clock_div ,
                                   uint16_t period_value );

// *****************************************************************************
/* Function:
    uint16_t PLIB_MCPWM_PrimaryTimerCountRead(  MCPWM_MODULE_ID index )

  Summary:
   Read the Primary PWM Timer count	

  Description:
    This function reads the count of local PWM Timer. 
 
 Precondition:
    The MCPWM Module must be disabled before calling this function.

  Parameters:
    index           - Identifier for the device instance to be configured.

	
  Returns:
    timer_count		- 16 bit Primary MCPWM Timer Count Value
	                  

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	
	uint16_t timer_count = PLIB_MCPWM_PrimaryTimerCountRead( MY_MCPWM_INSTANCE ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsPrimaryTimerSetup in your application to determine whether this feature
	is available.
*/

uint16_t PLIB_MCPWM_PrimaryTimerCountRead(  MCPWM_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_SecondarySpecialEventTriggerSetup( MCPWM_MODULE_ID index , 
	                                                   uint16_t compare_value , 
													   MCPWM_TRIGGER_DIVIDER postscaler_value ,
													   bool interrupt_status )

  Summary:
    Setup Secondary Special Event Trigger 

  Description:
    This function performs following setup functions for Secondary Special Event Trigger
	1. Sets the compare match value
	2. Sets the Secondary Special Event Trigger Output Postscaler

    	

  Precondition:
    The MCPWM Module must be disabled before calling this function.

  Parameters:
    index            -	Identifier for the device instance to be configured
    
	compare_value	 - 	16 bit Compare value (in Secondary MCPWM timer counts) which is compared 
	                    against Secondary Master Time Base to generate a match event.
						0<=compare_value<=65535
	
	postscaler_value -	Trigger Post scaler value

	

	Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// Compare match event occurs at Secondary MCPWM Timer (SMTMR) = 10000
	// Secondary Special Event Trigger is generated at every 4th compare match event.
	
    PLIB_MCPWM_SecondarySpecialEventTriggerSetup(MY_MCPWM_INSTANCE , 10000 , 4 );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific device data sheet
	to determine availability or use PLIB_MCPWM_ExistsSecondarySpecialEventTrigger in your application to determine
	whether this feature is available.
*/


void PLIB_MCPWM_SecondarySpecialEventTriggerSetup( MCPWM_MODULE_ID index , 
                                                   uint16_t compare_value , 
												   MCPWM_TRIGGER_DIVIDER 
												   postscaler_value );
// *****************************************************************************
/* Function:
    bool PLIB_MCPWM_SecondarySpecialEventTriggerInterruptIsPending( MCPWM_MODULE_ID index );

  Summary:
    Returns the interrupt status for Secondary Special Event Interrupt.

  Description:
    This function identifies if the Secondary Special Event Interrupt is pending or not.

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured
    

  Returns:
    -true	- Secondary Special Event Trigger Interrupt is pending.
	-false	- Secondary Special Event Trigger Interrupt is not pending.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
   	interruptStatus = PLIB_MCPWM_SecondarySpecialEventTriggerInterruptIsPending(MY_MCPWM_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsSecondarySpecialEventTrigger in your application to determine whether this feature
	is available.
*/

bool PLIB_MCPWM_SecondarySpecialEventTriggerInterruptIsPending( MCPWM_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_SecondarySpecialEventTriggerInterruptEnable( MCPWM_MODULE_ID index )

  Summary:
    Secondary SpecialEventTriggerInterrupt is Enabled.

  Description:
    This function enables Secondary Special Event Trigger Interrupt.

    When the Secondary Special Event Trigger Interrupt bit is enabled, an interrupt is generated at compare match 
	between Secondary MCPWM Timer (SMTMR) and Secondary Special Event Compare Register (SSEVTCMP)
	
	This function enables Primary Special Event Interrupt at the MCPWM module level. In order to fully enable the Primary Special Event Interrupt generation, the corresponding interrupt must be enabled in the Interrupt Controller. Please refer to Interrupt Peripheral Library for more information on PLIB APIs to enable interrupts in the Interrupt Controller.  
	

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
    PLIB_MCPWM_SecondarySpecialEventTriggerInterruptEnable(MY_MCPWM_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsSecondarySpecialEventTrigger in your application to determine whether this feature
	is available.
*/


void PLIB_MCPWM_SecondarySpecialEventTriggerInterruptEnable( MCPWM_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_SecondarySpecialEventTriggerInterruptDisable( MCPWM_MODULE_ID index )

  Summary:
    Disables Secondary Special Event Trigger Interrupt

  Description:
    This function disables Secondary Special Event Trigger Interrupt.

  Precondition:
    None.

  Parameters:
     index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
    PLIB_MCPWM_SecondarySpecialEventTriggerInterruptDisable(MY_MCPWM_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsSecondarySpecialEventTrigger in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_SecondarySpecialEventTriggerInterruptDisable( MCPWM_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_SecondaryTimerSetup( MCPWM_MODULE_ID index , MCPWM_CLOCK_DIVIDER clock_div , uint16_t period_value )

  Summary:
    Setup Secondary MCPWM Timer

  Description:
    This function performs following setup functions for Secondary MCPWM Timer.
	1. Sets the clock divider value by which the SYS_CLOCK is divided to generate the clock source for Secondary MCPWM timer
	2. Sets the period value for the secondary timer

    	

  Precondition:
    The MCPWM Module must be disabled before calling this function.


  Parameters:
    index            -	Identifier for the device instance to be configured
    
	clock_div		 -	Clock Divider 
	
	                    
	period_value	 - 	16 bit Period value (in MCPWM timer counts) at which the Seondary MCPWM Timer resets.
	                    8<=period_value<=65535
	

	Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// Secondary MCPWM Timer clock = SYS_CLOCK / 4
	// Secondary Timer Period = 10000
	
	
    PLIB_MCPWM_SecondaryTimerSetup(MY_MCPWM_INSTANCE , MCPWM_CLOCK_DIVIDE_BY_4 ,10000 );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsSecondaryTimerSetup in your application to determine whether this feature
	is available.
*/


void PLIB_MCPWM_SecondaryTimerSetup( MCPWM_MODULE_ID index , MCPWM_CLOCK_DIVIDER clock_div , uint16_t period_value );

// *****************************************************************************
/* Function:
    uint16_t PLIB_MCPWM_SecondaryTimerCountRead(  MCPWM_MODULE_ID index )

  Summary:
   Read the Secondary MCPWM Timer count	

  Description:
    This function reads the count of Secondary MCPWM Timer. 
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.

	
  Returns:
    timer_count		- 16 bit Secondary MCPWM Timer Count Value

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	
	uint16_t timer_count = PLIB_MCPWM_SecondaryTimerCountRead( MY_MCPWM_INSTANCE ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsSecondaryTimerSetup in your application to determine whether this feature
	is available.
*/

uint16_t PLIB_MCPWM_SecondaryTimerCountRead(  MCPWM_MODULE_ID index );
// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChopClockSetup( MCPWM_MODULE_ID index , uint16_t mcpwm_chop_clk_divider, MCPWM_CHOP_CLOCK_CONTROL mcpwm_chop_clk_control  )

  Summary:
    Setup for Chop Clock Generator

  Description:
    This function sets up Chop Clock Generator which is used to chop the MCPWM output.

  Precondition:
     The MCPWM Module must be disabled before calling this function.

  Parameters:
    index           		- Identifier for the device instance to be configured
	mcpwm_chop_clk_divider	- The divider by which the MCPWM clock is divided to generate the Chop Clock i.e.Chop Clock = 		SYS_CLOCK/(2^PTCON<PCLOCKDIV>)/CHOPCLOCK.
						      Valid chop clock divider range : 2 - 1023. Minimum values is 2. A value of 0 or 1 will produce no chop clock
							  
	mcpwm_chop_clk_control	- Enables / Disables the MCPWM Chop Clock 	:MCPWM_CHOP_CLOCK_ENABLED
																		 MCPWM_CHOP_CLOCK_DISABLED		
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// Chop Clk = MCPWM Clock / 400.
	// Chop Clk is enabled
    PLIB_MCPWM_ChopClockSetup(MY_MCPWM_INSTANCE , 400 , MCPWM_CHOP_CLOCK_ENABLED);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChopClockSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChopClockSetup( MCPWM_MODULE_ID index , uint16_t mcpwm_chop_clk_divider, MCPWM_CHOP_CLOCK_CONTROL mcpwm_chop_clk_control );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelSetup( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel_id , 
	MCPWM_TIME_BASE_SOURCE time_base_source , MCPWM_TIME_BASE_MODE time_base_mode
	MCPWM_ALIGNMENT_MODE mcpwm_alignment_mode, MCPWM_OUTPUT_MODE mcpwm_output_mode, MCPWM_PWMxH_OUTPUT_POLARITY mcpwm_pwmh_output_polarity , MCPWM_PWMxL_OUTPUT_POLARITY mcpwm_pwml_output_polarity , 
	MCPWM_DEADTIME_MODE mcpwm_deadtime_mode ,  MCPWM_DEADTIME_COMPENSATION_POLARITY mcpwm_deadtime_compensation_polarity )

  Summary:
    Setup MCPWM Channel

  Description:
    This function performs following setup functions for specified MCPWM Channel.
	1. Selects the source of MCPWM Time base for the specified MCPWM Channel.
	2. Selects if the specified MCPWM Channel operates in SYNCHRONIZED (synchronized with master time base source) mode
       or INDEPENDENT time base mode.
	3. Selects the MCPWM Alignment mode of the specified MCPWM Channel
	4. Selects MCPWM output mode of the specified MCPWM Channel
	5. Defines polarity of PWMxH of the specified MCPWM Channel
	6. Defines polarity of PWMxL of the specified MCPWM Channel
	7. Selects the deadtime mode of the specified MCPWM Channel.
	8. Defines polarity of dead time compensation input of the specified MCPWM Channel.

    	

  Precondition:
    None.

  Parameters:
    index 						-	Identifier for the device instance to be configured
	
	channel_id					-	Channel ID of the MCPWM Channel being setup.
	
	time_base_source			-	MCPWM_TIME_BASE_SOURCE_PRIMARY 
									MCPWM_TIME_BASE_SOURCE_SECONDARY 
						
	time_base_mode				-	MCPWM_TIME_BASE_SYNCHRONIZED
									MCPWM_TIME_BASE_INDEPENDENT 
							
	mcpwm_alignment_mode		-	MCPWM_EDGE_ALIGNED 
									MCPWM_SYMMETRIC_CENTER_ALIGNED 
									MCPWM_ASYMMETRIC_CENTER_ALIGNED 
									MCPWM_ASYMMETRIC_CENTER_ALIGNED_WITH_SIMULTANEOUS_UPDATE
							
	mcpwm_output_mode			-	MCPWM_OUTPUT_COMPLIMENTARY_MODE 
									MCPWM_OUTPUT_REDUNDANT_MODE 
									MCPWM_OUTPUT_PUSH_PULL_MODE 
									MCPWM_OUTPUT_LOW_LATCHED_TO_ZERO 
							
	mcpwm_pwmh_output_polarity	-	MCPWM_PWMxH_ACTIVEHIGH
									MCPWM_PWMxH_ACTIVELOW
							
	mcpwm_pwml_output_polarity	-	MCPWM_PWMxH_ACTIVEHIGH
									MCPWM_PWMxH_ACTIVELOW

							
	mcpwm_deadtime_mode			-	MCPWM_DEADTIME_POSITIVE (Applicable for all MCPWM output modes.)
									MCPWM_DEADTIME_NEGATIVE (Applicable only for Complimentary Output Mode.)
									MCPWM_DEADTIME_DISABLE	(No Deadtime is applied to the MCPWM outputs.)
									MCPWM_DEADTIME_COMPENSATION (Applicable for all MCPWM output modes.)
		
	
	mcpwm_deadtime_compensation_pol	-	MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH	
										If the DTCMPx pin = 0, PWMxL is shortened, and PWMxH is lengthened
										If the DTCMPx pin = 1, PWMxH is shortened, and PWMxL is lengthened
						
										MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW
										If the DTCMPx pin = 0, PWMxH is shortened, and PWMxL is lengthened
										If the DTCMPx pin = 1, PWMxL is shortened, and PWMxH is lengthened
	Returns:
    None.

  Example:
    <code>
	// Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// PWM2 uses Primary Timer as its time base source
	// PWM2 is in MASTER time base mode
	// PWM2 is in EDGE_ALIGNED  and COMPLIMENTARY output mode
	// PWM2H and PWM2L have ACTIVE_HIGH polarity
	// PWM2 has POSITIVE_DEADTIME enabled.
	// PWM2's deadtime_comp_pol is ACTIVE_HIGH.
	
	// MCPWM_CHANNEL_ID mcpwm_channel = 2;
    PLIB_MCPWM_MCPWMChannelSetup(MY_MCPWM_INSTANCE , mcpwm_channel, MCPWM_TIME_BASE_PRIMARY_TIMER, MCPWM_TIME_BASE_SYNCHRONIZED, MCPWM_ALIGNMENT_EDGE ,
	MCPWM_OUTPUT_COMPLIMENTARY_MODE, MCPWM_PWMxH_ACTIVEHIGH , MCPWM_PWMxL_ACTIVEHIGH , POSITIVE_DEADTIME_ENABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelSetup in your application to determine whether this feature
	is available.
*/
	void PLIB_MCPWM_ChannelSetup( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel_id ,
                                  MCPWM_TIME_BASE_SOURCE time_base_source, 
								  MCPWM_TIME_BASE_MODE time_base_mode,
	                              MCPWM_ALIGNMENT_MODE mcpwm_alignment_mode, 
								  MCPWM_OUTPUT_MODE mcpwm_output_mode, 
								  MCPWM_PWMxH_OUTPUT_POLARITY mcpwm_pwmh_output_polarity,
								  MCPWM_PWMxL_OUTPUT_POLARITY mcpwm_pwml_output_polarity, 
	                              MCPWM_DEADTIME_MODE mcpwm_deadtime_mode ,  
								  MCPWM_DEADTIME_COMPENSATION_POLARITY mcpwm_deadtime_compensation_polarity );
	
	// *****************************************************************************
/* Function:
    void PLIB_MCPWM_IOCONxUnlock(  MCPWM_MODULE_ID index )

 Summary:
    Executes the unlock sequence to allow IOCONx write access.

  Description:
    This function executes the unlock sequence to allow write access to IOCONx. After a single 
	IOCONx write, the register is locked again and the unlock sequence must be executed again 
	to unlock the IOCONx register
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	
	
  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	
	PLIB_MCPWM_IOCONxUnlock( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelGeneralFunctions in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_IOCONxUnlock(  MCPWM_MODULE_ID index );
	
	// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPrimaryDutyCycleSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel,
                                          uint16_t value)

  Summary:
    Sets the Primary Duty Cycle value of the MCPWM Channel specified

  Description:
    This function sets the primary duty cycle value of the MCPWM Channel ID passed using function parameters.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID for which the duty cycle needs to be set.
	value        		    - 16 bit Duty Cycle value 
							  0<= value<= 65535
	

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel for which the primary duty cycle needs to be set.
    // 10000 is the value of the Duty Cycle
   
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL,10000);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPrimaryDutyCycleSet in your application to determine whether this feature
	is available.
*/
	


void PLIB_MCPWM_ChannelPrimaryDutyCycleSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value);

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelSecondaryDutyCycleSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel,
                                          uint16_t value)

  Summary:
    Sets the Secondary Duty Cycle value of the MCPWM Channel specified

  Description:
    This function sets the secondary duty cycle value of the MCPWM Channel ID passed using function parameters.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID for which the duty cycle needs to be set.
	value        		    - 16 bit Duty Cycle value
                              0<=value<=65535

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel for which the secondary duty cycle needs to be set.
    // 10000 is the value of the Duty Cycle
   
    PLIB_MCPWM_ChannelSecondaryDutyCycleSet( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL,10000);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelSecondaryDutyCycleSet in your application to determine whether this feature
	is available.
*/



void PLIB_MCPWM_ChannelSecondaryDutyCycleSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value);

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPhaseSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel,
                                          uint16_t value)

  Summary:
    Sets the Phase value of the MCPWM Channel specified

  Description:
    This function sets the phase value of the MCPWM Channel ID passed using function parameters.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID for which the duty cycle needs to be set.
	value        		    - 16 bit phase value
	                          0<=value<=65535

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel for which the secondary duty cycle needs to be set.
    // 5000 is the phase value 
   
    PLIB_MCPWM_ChannelSecondaryDutyCycleSet( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL,5000);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPhaseSet in your application to determine whether this feature
	is available.
*/



void PLIB_MCPWM_ChannelPhaseSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value);
										  
// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxHDeadtimeSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel,
                                          uint16_t value)

  Summary:
    Sets the Deadtime value for the PWMxH MCPWM Channel specified

  Description:
    This function sets the deadtime value of the PWMxH MCPWM of the MCPWM Channel ID passed using function parameters.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID for which the PWMxH deadtime needs to be set.
	value        		    - 14 bit Deadtime value (in MCPWM timer counts)
                              0<=value<=16383	                          

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel for which the deadtime needs to be set.
    // 100 is the value of the Deadtime (in MCPWM timer counts)
   
    PLIB_MCPWM_ChannelPWMxHDeadtimeSet( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL,100);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelDeadtimeSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxHDeadtimeSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value);
										  

										  
// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxLDeadtimeSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel,
                                          uint16_t value)

  Summary:
    Sets the Deadtime value for the PWMxL MCPWM Channel specified

  Description:
    This function sets the deadtime value of the PWMxL MCPWM of the MCPWM Channel ID passed using function parameters.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID for which the PWMxL  deadtime needs to be set.
	value        		    - 14 bit Deadtime value (in MCPWM timer counts)
                              0<=value<=16383	

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel for which the deadtime needs to be set.
    // 100 is the value of the Deadtime (in MCPWM timer counts)
   
    PLIB_MCPWM_ChannelPWMxLDeadtimeSet( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL,100);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelDeadtimeSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxLDeadtimeSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value);										  
										  
// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelDeadtimeCompSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel,
                                          uint16_t value)

  Summary:
    Sets the Deadtime Compensation value for the PWMxH MCPWM Channel specified

  Description:
    This function sets the deadtime compensation value of the MCPWM Channel ID passed using function parameters.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID for which the PWMxH deadtime needs to be set.
	value        		    - Deadtime value (in MCPWM timer counts)

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel for which the deadtime compensation needs to be set.
    // 100 is the value of the Deadtime compensation (in MCPWM timer counts)
   
    PLIB_MCPWM_ChannelDeadtimeCompSet( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL,100);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelDeadtimeSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelDeadtimeCompSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value);

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelTriggerSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_TRIGGER_DIVIDER trigger_postscaler_value ,
	MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT primary_trigger_cycle_select , MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT secondary_trigger_cycle_select , 
	MCPWM_ADC_TRIGGER_SOURCE mcpwm_adc_trigger_source , MCPWM_TRIGGER_INTERRUPT_SOURCE mcpwm_trigger_interrupt_source, uint16_t primary_trigger_compare_value , 
	uint16_t secondary_trigger_compare_value)

  Summary:
	Setup for individual MCPWM Channel Trigger

  Description:
    This function sets up the trigger generated by the specified MCPWM Channel

  Precondition:
    None.

  Parameters:
    index                   		-	Identifier for the device instance to be configured
    
	channel							-	MCPWM Channel ID whose triggers are being configured.
	
	trigger_postscaler_value		- 	Defines the post scaler value for the MCPWM Channel triggers
	
	primary_trigger_cycle_select	-	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING
	                                    Generate Triggers upon primary trigger compare match event while MCPWM timer is
										incrementing (PTDIR = 0) as well as decrementing (PTDIR = 1).
										
										MCPWM_PRIMARY_TRIGGER_DURING_TIMER_DECREMENTING
										Generate Triggers upon primary trigger compare match event while MCPWM timer is 
										decrementing (PTDIR = 1).
										
										MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING
										Generate Triggers upon primary trigger compare match event while MCPWM timer is 
										incrementing (PTDIR = 0).
										
	secondary_trigger_cycle_select	-	MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING
	                                    Generate Triggers upon secondary trigger compare match event while MCPWM timer is 
										incrementing (PTDIR = 0) as well as decrementing (PTDIR = 1).
										
										MCPWM_SECONDARY_TRIGGER_DURING_TIMER_DECREMENTING
										Generate Triggers upon secondary trigger compare match event while MCPWM timer is 
										decrementing (PTDIR = 1).
										
										MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING
										Generate Triggers upon secondary trigger compare match event while MCPWM timer is 
										incrementing (PTDIR = 0).
										
	mcpwm_adc_trigger_source		-	MCPWM_ADC_TRIGGER_PRIMARY 
	                                    ADC trigger is generated at primary trigger compare match event.
										
										MCPWM_ADC_TRIGGER_PRIMARY_SECONDARY
										ADC trigger is generated at primary as well as secondary trigger compare match event.
										
	mcpwm_trigger_interrupt_source	-	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY  
	                                    For mcpwm_adc_trigger_source = PRIMARY,interrupt is generated 
										on primary trigger compare match event.
										For mcpwm_adc_trigger_source = PRIMARY_SECONDARY, interrupt is 
										generated on primary as well as secondary trigger compare match event. 
										
										MCPWM_TRIGGER_INTERRUPT_SOURCE_SECONDARY	
									    Interrupt is generated on secondary trigger compare match event.
	
	primary_trigger_compare_value	- 	16 bit Primary trigger compare value (in MCPWM Timer counts).
	                                    0<=primary_trigger_compare_value<=65535
	
	secondary_trigger_compare_value -   16 bit Secondary trigger compare value (in MCPWM Timer counts).
	                                    0<=secondary_trigger_compare_value<=65535
									

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose triggers are being setup.
	// Generate Primary / Secondary MCPWM trigger at every 4th primary / secondary compare match event
    // Generate Primary trigger upon primary trigger compare match event while MCPWM timer is incrementing.
	// Generate Secondary trigger upon secondary trigger compare match event while MCPWM timer is incrementing.
	// ADC trigger is generated for primary as well as secondary trigger compare match event.
	// Interrupt is generated on Primary as well as Secondary trigger compare match event.
	// Primary Trigger compare value = 100.
	// Secondary Trigger compare value = 500.
   
    PLIB_MCPWM_ChannelTriggerSetup( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL, DIV_BY_4, COUNT_UP, COUNT_UP , PRIMARY_SECONDARY , PRIMARY , 100, 500);
    </code>

   Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelTrigger in your application to determine whether this feature
	is available.
*/

	void PLIB_MCPWM_ChannelTriggerSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_TRIGGER_DIVIDER trigger_postscaler_value ,
	MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT primary_trigger_cycle_select , MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT secondary_trigger_cycle_select , 
	MCPWM_ADC_TRIGGER_SOURCE mcpwm_adc_trigger_source , MCPWM_TRIGGER_INTERRUPT_SOURCE mcpwm_trigger_interrupt_source, uint16_t primary_trigger_compare_value , 
	uint16_t secondary_trigger_compare_value);
// *****************************************************************************
/* Function:
    uint16_t PLIB_MCPWM_ChannelCurrentLimitCaptureRead(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Reads the CAPx register

  Description:
    This function reads the value of CAPx register. CAPx register stores the local MCPWM timer value when a leading edge is detected on current limit input.
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose  CAPx register is being read

  Returns:
    uint16_t value stored in CAPx register

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose CAPx value is being read.
	
    uint16_t capture_value  = PLIB_MCPWM_ChannelCurrentLimitCaptureRead( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelCurrentLimit in your application to determine whether this feature
	is available.
*/

uint16_t PLIB_MCPWM_ChannelCurrentLimitCaptureRead( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_FAULT_INPUT_LEB_CONTROL mcpwm_fault_input_leb_control ,
	MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL mcpwm_currentlimit_input_leb_control , uint16_t leb_delay)

  Summary:
	Setup for Leading Edge Blanking (LEB) for the specified MCPWM Channel

  Description:
    This function sets up the leading edge blanking for MCPWM Channel ID specified in the function parameters.

  Precondition:
    None.

  Parameters:
    index                   				-	Identifier for the device instance to be configured
    
	channel									-	MCPWM Channel ID for which LEB needs to be set.
	
										
								
	mcpwm_fault_input_leb_control			- 	MCPWM_FAULT_INPUT_LEB_ENABLE 
	                                            LEB is applied to Fault input
												MCPWM_FAULT_INPUT_LEB_DISABLE
												LEB is note applied to Fault input
	
	mcpwm_currentlimit_input_leb_control	- 	MCPWM_CURRENTLIMIT_INPUT_LEB_ENABLE
	                                            LEB is applied to Current Limit input
												MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE
												LEB is not applied to Current Limit input
	
	leb_delay								-	12 bit Leading Edge Blanking delay (in MCPWM timer counts)
												Range for LEB delay is 0 to 4095 MCPWM timer counts
							

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel for which LEB needs to be configured.
    // LEB is applied to fault and current limit input
	// LEB delay is set to 500 MCPWM timer counts
   
    PLIB_MCPWM_ChannelLEBSetup( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL, MCPWM_FAULT_INPUT_LEB_ENABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_ENABLE , 500);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

    void PLIB_MCPWM_ChannelLEBSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, 
	                                 MCPWM_FAULT_INPUT_LEB_CONTROL mcpwm_fault_input_leb_control ,
									 MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL mcpwm_currentlimit_input_leb_control ,
									 uint16_t leb_delay);


 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Re-triggers LEB on rising edge of PWMxH

  Description:
    This function enables re-triggering of LEB counter on rising edge of PWMxH
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose LEB trigger source is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose LEB trigger source is being enabled.
	
    PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );






 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Re-triggers LEB on falling edge of PWMxH

  Description:
    This function enables re-triggering of LEB counter on falling edge of PWMxH
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose LEB trigger source is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose LEB trigger source is being enabled.
	
    PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );


 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Re-triggers LEB on rising edge of PWMxL

  Description:
    This function enables re-triggering of LEB counter on rising edge of PWMxL
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose LEB trigger source is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose LEB trigger source is being enabled.
	
    PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );


 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Re-triggers LEB on falling edge of PWMxL

  Description:
    This function enables re-triggering of LEB counter on falling edge of PWMxL
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose LEB trigger source is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose LEB trigger source is being enabled.
	
    PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Disables triggering of LEB on rising edge of PWMxH

  Description:
    This function disables re-triggering of LEB counter on rising edge of PWMxH
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose LEB trigger source is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose LEB trigger source is being disabled.
	
    PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );






 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Disables triggering of LEB on falling edge of PWMxH

  Description:
    This function disables re-triggering of LEB counter on falling edge of PWMxH
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose LEB trigger source is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose LEB trigger source is being disabled.
	
    PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );


 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Disables triggering of LEB on rising edge of PWMxL

  Description:
    This function disables re-triggering of LEB counter on rising edge of PWMxL
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose LEB trigger source is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose LEB trigger source is being disabled.
	
    PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

   Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );


 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Disables triggering of LEB on falling edge of PWMxL

  Description:
    This function disables re-triggering of LEB counter on falling edge of PWMxL
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose LEB trigger source is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose LEB trigger source is being disabled.
	
    PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelLEBSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );


	// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelChopSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_CHOP_CLOCK_SOURCE chop_clock_source , 
											MCPWM_PWMxH_CHOP_CONTROL mcpwm_pwmh_chop_control , MCPWM_PWMxL_CHOP_CONTROL mcpwm_pwml_chop_control )

  Summary:
	Setup for Chopping function of  MCPWM Channel.

  Description:
    This function sets up the Chopping function of the specified MCPWM channel

  Precondition:
    None.

  Parameters:
    index                   -	Identifier for the device instance to be configured
    
	channel					-	MCPWM Channel ID whose chopping function is being setup.
	
	chop_clock_source		- 	Selects the one of the possible values clock source from
								MCPWM_CHOP_CLOCK_SOURCE for generating chop frequency.
	
	mcpwm_pwmh_chop_control - Enables/Disables chopping function on PWMxH
									- MCPWM_PWMxH_CHOP_DISABLED
									- MCPWM_PWMxH_CHOP_ENABLED
	mcpwm_pwml_chop_control - Enables/Disables chopping function on PWMxL
									- MCPWM_PWMxL_CHOP_DISABLED
									- MCPWM_PWMxL_CHOP_ENABLED
	
	
  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose chopping function is being setup.
    // Chop Clock Generator is the clock source for MCPWM Chop frequency
	// Chopping of PWMxH and PWMxL, at chop clock frequency, is enabled.
   
    PLIB_MCPWM_ChannelChopSetup( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL, 
								 MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR ,
                                 MCPWM_PWMxH_CHOP_ENABLED, MCPWM_PWMxL_CHOP_ENABLED);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChopClockSetup in your application to determine whether this feature
	is available.
*/

	void PLIB_MCPWM_ChannelChopSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, 
	                                  MCPWM_CHOP_CLOCK_SOURCE chop_clock_source ,
                        			  MCPWM_PWMxH_CHOP_CONTROL mcpwm_pwmh_chop_control , 
									  MCPWM_PWMxL_CHOP_CONTROL mcpwm_pwml_chop_control );

// *****************************************************************************
/* Function:
    uint16_t PLIB_MCPWM_ChannelLocalPWMTimerCountRead(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
   Read the Local PWM Timer count	

  Description:
    This function reads the count of local PWM Timer. 
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose local PWM timer count is being read.
	
  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose local PWM timer count is being read.
	uint16_t local_tmr = PLIB_MCPWM_ChannelLocalPWMTimerCountRead( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific device data sheet
	to determine availability or use PLIB_MCPWM_ChannelExistsChannelGeneralFunctions in your application to determine
    whether this feature is available.
*/

uint16_t PLIB_MCPWM_ChannelLocalPWMTimerCountRead(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel);


// *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelFaultInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Fault Interrupt is pending.

  Description:
    This function reads FLTIF bit to ascertain if the fault interrupt is pending
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose fault interrupt status is being checked.
	
  Returns:
    true 			- Fault Interrupt is pending.
	false			- Fault Interrupt is not pending.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	 MY_MCPWM_CHANNEL, is the MCPWM Channel whose fault interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelFaultInterruptIsPending( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific device data sheet
	to determine availability or use PLIB_MCPWM_ChannelExistsChannelFault in your application to determine
    whether this feature is available.
*/

 bool PLIB_MCPWM_ChannelFaultInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 
 // *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelFaultInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Fault Interrupt is enabled.

  Description:
    This function reads FLTIEN bit to ascertain if the fault interrupt is enabled
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose fault interrupt status is being checked.
	
  Returns:
    true 			- Fault Interrupt is enabled.
	false			- Fault Interrupt is not enabled.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	 MY_MCPWM_CHANNEL, is the MCPWM Channel whose fault interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelFaultInterruptIsEnabled( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific device data sheet
	to determine availability or use PLIB_MCPWM_ChannelExistsChannelFault in your application to determine
    whether this feature is available.
*/

 bool PLIB_MCPWM_ChannelFaultInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 
 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelFaultInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
                                          
										  

  Summary:
    Clears the Fault Interrupt Flag

  Description:
    This function clears the Fault Interrupt Flag Bit of the MCPWM Channel Specified.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose Fault Interrupt flag is to be cleared.


  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose fault interrupt flag is to be cleared.
    
   
    PLIB_MCPWM_ChannelFaultInterruptFlagClear( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelFault in your application to determine whether this feature
	is available.
*/

 void PLIB_MCPWM_ChannelFaultInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);
 // *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Current Limit Interrupt is pending.

  Description:
    This function reads CLIF bit to ascertain if the current limit interrupt is pending
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose current limit interrupt status is being checked.
	
  Returns:
    true 			- Current Limit Interrupt is pending.
	false			- Current Limit Interrupt is not pending.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose current limit interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific device data sheet
	to determine availability or use PLIB_MCPWM_ChannelExistsChannelFault in your application to determine
    whether this feature is available.
*/

 bool PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
  // *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Current Limit Interrupt is enabled.

  Description:
    This function reads CLIEN bit to ascertain if the current limit interrupt is enabled
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose current limit interrupt status is being checked.
	
  Returns:
    true 			- Current Limit Interrupt is enabled.
	false			- Current Limit Interrupt is not enabled.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose current limit interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific device data sheet
	to determine availability or use PLIB_MCPWM_ChannelExistsChannelFault in your application to determine
    whether this feature is available.
*/

 bool PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
                                          
										  

  Summary:
    Clears the Current Limit Interrupt Flag

  Description:
    This function clears the Current Limit Interrupt Flag Bit of the MCPWM Channel Specified.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose current limit interrupt flag is to be cleared.


  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose current limit interrupt flag is to be cleared.
    
   
    PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelFault in your application to determine whether this feature
	is available.
*/

 void PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);
 
// *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelTriggerInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Trigger Interrupt is pending.

  Description:
    This function reads TRGIF bit to ascertain if the trigger interrupt is pending
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose trigger interrupt status is being checked.
	
  Returns:
    true 			- Trigger Interrupt is pending.
	false			- Trigger Interrupt is not pending.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose trigger interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelTriggerInterruptIsPending( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific device data sheet
	to determine availability or use PLIB_MCPWM_ChannelExistsChannelFault in your application to determine
    whether this feature is available.
*/

 bool PLIB_MCPWM_ChannelTriggerInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
// *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelTriggerInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Trigger Interrupt is enabled.

  Description:
    This function reads TRGIEN bit to ascertain if the trigger interrupt is enabled
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose trigger interrupt status is being checked.
	
  Returns:
    true 			- Trigger Interrupt is enabled.
	false			- Trigger Interrupt is not enabled.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose trigger interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelTriggerInterruptIsEnabled( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific device data sheet
	to determine availability or use PLIB_MCPWM_ChannelExistsChannelFault in your application to determine
    whether this feature is available.
*/

 bool PLIB_MCPWM_ChannelTriggerInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelTriggerInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
                                          
										  

  Summary:
    Clears the Trigger Interrupt Flag

  Description:
    This function clears the Trigger Interrupt Flag Bit of the MCPWM Channel Specified.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose Trigger interrupt flag is to be cleared.


  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Trigger interrupt flag is to be cleared.
    
   
    PLIB_MCPWM_ChannelTriggerInterruptFlagClear( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelTrigger in your application to determine whether this feature
	is available.
*/

 void PLIB_MCPWM_ChannelTriggerInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);
 // *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelPeriodResetInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Period Reset Interrupt is pending.

  Description:
    This function reads PWMxLIF bit to ascertain if the Period Reset interrupt is pending
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose Period Reset interrupt status is being checked.
	
  Returns:
    true 			- Period Reset Interrupt is pending.
	false			- Period Reset Interrupt is not pending.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Period Reset interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelPeriodResetInterruptIsPending( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodResetInterrupt in your application to determine whether this feature
	is available.
*/

 bool PLIB_MCPWM_ChannelPeriodResetInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 
  // *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelPeriodResetInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Period Reset Interrupt is enabled.

  Description:
    This function reads PWMxLIF bit to ascertain if the Period Reset interrupt is enabled
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose Period Reset interrupt status is being checked.
	
  Returns:
    true 			- Period Reset Interrupt is enabled.
	false			- Period Reset Interrupt is not enabled.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Period Reset interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelPeriodResetInterruptIsEnabled( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodResetInterrupt in your application to determine whether this feature
	is available.
*/

 bool PLIB_MCPWM_ChannelPeriodResetInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 
 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPeriodResetInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
                                          
										  

  Summary:
    Clears the Period Reset Interrupt Flag bit (PWMxLIF)

  Description:
    This function clears the Period Reset Interrupt Flag Bit (PWMxLIF) of the MCPWM Channel Specified.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose Period Reset interrupt flag is to be cleared.


  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Period Reset interrupt flag is to be cleared.
    
   
    PLIB_MCPWM_ChannelPeriodResetInterruptFlagClear( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodResetInterrupt in your application to determine whether this feature
	is available.
*/

 void PLIB_MCPWM_ChannelPeriodResetInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);
  // *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Period Match  Interrupt is pending.

  Description:
    This function reads PWMxHIF bit to ascertain if the Period Match  interrupt is pending
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose Period Match  interrupt status is being checked.
	
  Returns:
    true 			- Period Match  Interrupt is pending.
	false			- Period Match  Interrupt is not pending.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Period Match  interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

   Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodResetInterrupt in your application to determine whether this feature
	is available.
*/

 bool PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
// *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if Period Match Interrupt is enabled.

  Description:
    This function reads PWMxHIF bit to ascertain if the Period Match interrupt is enabled
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose Period Match interrupt status is being checked.
	
  Returns:
    true 			- Period Match Interrupt is enabled.
	false			- Period Match Interrupt is not enabled.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Period Match interrupt status is being checked.
	bool interrupt_status;
	interrupt_status = PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

   Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodResetInterrupt in your application to determine whether this feature
	is available.
*/

 bool PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 
 
 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
                                          
										  

  Summary:
    Clears the Period Match  Interrupt Flag bit (PWMxHIF)

  Description:
    This function clears the Period Match  Interrupt Flag Bit (PWMxHIF) of the MCPWM Channel Specified.

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose Period Match  interrupt flag is to be cleared.


  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Period Match  interrupt flag is to be cleared.
    
   
    PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt in your application to determine whether this feature
	is available.
*/

 void PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);
 
 
 
 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelFaultInterruptEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Enables the fault interrupt for the specified MCPWM channel.

  Description:
    This function enables the fault interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose fault interrupt is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose fault interrupt is being enabled.
	
    PLIB_MCPWM_ChannelFaultInterruptEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelFault in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelFaultInterruptEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelFaultInterruptDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Disables the fault interrupt for the specified MCPWM channel.

  Description:
    This function disables the fault interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose fault interrupt is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose fault interrupt is being disabled.
	
    PLIB_MCPWM_ChannelFaultInterruptDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelFault in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelFaultInterruptDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Enables the current limit interrupt for the specified MCPWM channel.

  Description:
    This function enables the current limit interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose current limit interrupt is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose current limit interrupt is being enabled.
	
    PLIB_MCPWM_ChannelCurrentLimitInterruptEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelCurrentLimit in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelCurrentLimitInterruptEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelCurrentLimitInterruptDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Disables the current limit interrupt for the specified MCPWM channel.

  Description:
    This function disables the current limit interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose current limit interrupt is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose current limit interrupt is being disabled.
	
    PLIB_MCPWM_ChannelCurrentLimitInterruptDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelCurrentLimit in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelCurrentLimitInterruptDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelTriggerInterruptEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Enables the trigger interrupt for the specified MCPWM channel.

  Description:
    This function enables the trigger interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose trigger interrupt is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose trigger interrupt is being enabled.
	
    PLIB_MCPWM_ChannelTriggerInterruptEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelTrigger in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelTriggerInterruptEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelTriggerInterruptDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Disables the trigger interrupt for the specified MCPWM channel.

  Description:
    This function disables the trigger interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose trigger interrupt is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose trigger interrupt is being disabled.
	
    PLIB_MCPWM_ChannelTriggerInterruptDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelTrigger in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelTriggerInterruptDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPeriodResetInterruptEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Enables the Period Reset interrupt for the specified MCPWM channel.

  Description:
    This function enables the Period Reset interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose Period Reset interrupt is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose Period Reset interrupt is being enabled.
	
    PLIB_MCPWM_ChannelPeriodResetInterruptEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodResetInterrupt in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPeriodResetInterruptEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPeriodResetInterruptDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Disables the Period Reset interrupt for the specified MCPWM channel.

  Description:
    This function disables the Period Reset interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose Period Reset interrupt is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose Period Reset interrupt is being disabled.
	
    PLIB_MCPWM_ChannelPeriodResetInterruptDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodResetInterrupt in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPeriodResetInterruptDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPeriodMatchInterruptEnable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Enables the Period Match interrupt for the specified MCPWM channel.

  Description:
    This function enables the Period Match interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose Period Match interrupt is being enabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose Period Match interrupt is being enabled.
	
    PLIB_MCPWM_ChannelPeriodMatchInterruptEnable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPeriodMatchInterruptEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPeriodMatchInterruptDisable(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)

  Summary:
    Disables the Period Match interrupt for the specified MCPWM channel.

  Description:
    This function disables the Period Match interrupt for the specified MCPWM Channel
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose Period Match interrupt is being disabled.

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL is the MCPWM Channel whose Period Match interrupt is being disabled.
	
    PLIB_MCPWM_ChannelPeriodMatchInterruptDisable( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL ); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPeriodMatchInterruptDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelFaultIsAsserted(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if MCPWM Fault is asserted.

  Description:
    This function reads FLTSTAT bit to ascertain if the MCPWM Fault is asserted
	
	In "Cycle by Cycle" Fault Mode, FLTSTAT bit reflects the status of FLTx pin.
	In "Latched" Fault Mode, FLTSTAT bit reflects the "latched" status of FLTx pin. In other words, even if the fault pin is de-asserted, 
	FLTSTAT bit will remain asserted until the fault is cleared. Following steps must be executed in order to recover from the fault and clear the FLTSTAT bit.
	
	1. FLTx pin must be de-asserted.
	2. After FLTx pin is de-asserted, MCPWM Fault must be disabled and re-enabled in Cycle by Cycle or Latched Mode.
	
	After following above two steps, the MCPWM will recover from the fault at the next period boundary.
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose MCPWM fault status is being checked.
	
  Returns:
    true 			- MCPWM fault is asserted.
	false			- MCPWM fault is not asserted.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose MCPWM fault status is being checked.
	bool fault_status;
	fault_status = PLIB_MCPWM_ChannelFaultIsAsserted( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelFault in your application to determine whether this feature
	is available.
	
*/

 bool PLIB_MCPWM_ChannelFaultIsAsserted(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 
 // *****************************************************************************
/* Function:
    bool PLIB_MCPWM_ChannelCurrentLimitIsAsserted(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Checks if MCPWM Current Limit is asserted.

  Description:
    This function reads CLTSTAT bit to ascertain if the MCPWM Current Limit is asserted
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose MCPWM current limit status is being checked.
	
  Returns:
    true 			- MCPWM current limit is asserted.
	false			- MCPWM current limit is not asserted.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose MCPWM current limit status is being checked.
	bool current_limit_status;
	current limit_status = PLIB_MCPWM_ChannelCurrentLimitIsAsserted( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelCurrentLimit in your application to determine whether this feature
	is available.
	
*/

 bool PLIB_MCPWM_ChannelCurrentLimitIsAsserted(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

 // *****************************************************************************
/* Function:
    MCPWM_TIMER_DIRECTION PLIB_MCPWM_ChannelTimerDirectionGet(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )

 Summary:
    Gets the PWM Timer direction

  Description:
    This function reads PTDIR bit to ascertain if the MCPWM timer is incrementing/decrementing.
 
 Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured.
	channel			- MCPWM Channel ID whose timer direction is being checked.
	
  Returns:
    MCPWM_TIMER_DIRECTION			- MCPWM_TIMER_INCREMENTING.
									- MCPWM_TIMER_DECREMENTING

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose timer count direction is being checked.
	MCPWM_TIMER_DIRECTION count_down;
	count_down = PLIB_MCPWM_ChannelTimerDirectionGet( MY_MCPWM_INSTANCE , MY_MCPWM_CHANNEL); 
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelGeneralFunctions in your application to determine whether this feature
	is available.
	
*/

 bool PLIB_MCPWM_ChannelTimerDirectionGet(  MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );
 
 

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelCurrentLimitSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, 
	                                          MCPWM_CURRENTLIMIT_SOURCE current_limit_source , 
											  MCPWM_CURRENTLIMIT_INPUT_POLARITY current_limit_input_polarity, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE current_limit_override_pwmh_value ,
											  MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE  current_limit_override_pwml_value, MCPWM_CURRENTLIMIT_MODE current_limit_mode )

  Summary:
	Current Limit Setup of the specified MCPWM Channel

  Description:
    This function sets up the current limit function of the specified MCPWM Channel

  Precondition:
	None.

  Parameters:
    index                   		  -	Identifier for the device instance to be configured
    
	channel							  -	MCPWM Channel ID whose current limit function is being setup.
	
	current_limit_source			  - Defines the input selected as current limit input.
	
	current_limit_input_polarity	  -	MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH
                                       	Polarity of the current limit input is active high.
									
										MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW
										Polarity of the current limit input is active low.
								
	current_limit_override_pwmh_value -	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0
										MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_1

	current_limit_override_pwml_value -	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0
										MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_1
										
	current_limit_mode				  -	MCPWM_CURRENTLIMIT_DISABLE
	                                    Disable Current Limit mode
										MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE
										Enable Standard Current Limit mode
									    MCPWM_XPRES_CURRENT_LIMIT_MODE_ENABLE
										Enable XPRES Current Limit mode

	
  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelCurrentLimit in your application to determine whether this feature
	is available.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose current limit function is being setup.
    // FLT3 acts as a current limit input
	// Polarity of the current limit input is active high.
	// PWMxH = 0, PWMxL = 0 when current limit input is asserted.
	// Standard Current Limit mode is enabled
   
    PLIB_MCPWM_ChannelCurrentLimitSetup( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT3 ,  
										MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, 
										MCPWM_STANDARD_CURRENTLIMIT_ENABLE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelCurrentLimit in your application to determine whether this feature
	is available.
*/

	void PLIB_MCPWM_ChannelCurrentLimitSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel,
                                              MCPWM_CURRENTLIMIT_SOURCE current_limit_source , 
											  MCPWM_CURRENTLIMIT_INPUT_POLARITY current_limit_input_polarity, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE current_limit_override_pwmh_value ,
											  MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE  current_limit_override_pwml_value, MCPWM_CURRENTLIMIT_MODE current_limit_mode );



// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelFaultSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_FAULT_SOURCE mcpwm_fault_source , 
									   MCPWM_FAULT_INPUT_POLARITY mcpwm_fault_input_polarity, 
									   MCPWM_FAULT_OVERRIDE_PWMxH_VALUE  mcpwm_fault_override_pwmh_value, MCPWM_FAULT_OVERRIDE_PWMxL_VALUE  mcpwm_fault_override_pwml_value
									   MCPWM_FAULT_MODE  mcpwm_fault_mode )

  Summary:
	Fault Setup of the specified MCPWM Channel

  Description:
    This function sets up the fault function of the specified MCPWM Channel

  Precondition:
 	None.

  Parameters:
    index                   		-	Identifier for the device instance to be configured
    
	channel							-	MCPWM Channel ID whose fault function is being setup.
	
	mcpwm_fault_source				- 	Defines the input selected as fault input.
	
	mcpwm_fault_input_polarity		-	ACTIVE_HIGH_FLT_POLARITY
	                                    Polarity of the fault input is active high.
										ACTIVE_LOW_FLT_POLARITY
										Polarity of the fault input is active low.
								
	mcpwm_fault_override_pwmh_value -	MCPWM_FAULT_OVERRIDE_PWMxH_0
										MCPWM_FAULT_OVERRIDE_PWMxH_1
	
	mcpwm_fault_override_pwml_value -	MCPWM_FAULT_OVERRIDE_PWMxL_0
										MCPWM_FAULT_OVERRIDE_PWMxL_1
	

	 mcpwm_fault_mode				- 	MCPWM_FAULT_MODE_LATCHED			
										MCPWM_FAULT_MODE_CYCLE_BY_CYCLE	
										MCPWM_FAULT_MODE_DISABLED			

	
  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose fault function is being setup.
    // FLT3 acts as a fault input
	// Polarity of the fault input is active high.
	// PWMxH = 0, PWMxL = 0 when fault input is asserted.
	// Latched Fault Mode enabled.
   
    PLIB_MCPWM_ChannelFaultSetup( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL, MCPWM_FAULT_SOURCE_IS_FLT3,  ACTIVE_HIGH_FLT_POLARITY , MCPWM_FAULT_OVERRIDE_PWMxH_0,MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_LATCHED);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelFault in your application to determine whether this feature
	is available.
*/

	void PLIB_MCPWM_ChannelFaultSetup( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_FAULT_SOURCE mcpwm_fault_source , 
									   MCPWM_FAULT_INPUT_POLARITY mcpwm_fault_input_polarity, 
									   MCPWM_FAULT_OVERRIDE_PWMxH_VALUE  mcpwm_fault_override_pwmh_value, MCPWM_FAULT_OVERRIDE_PWMxL_VALUE  mcpwm_fault_override_pwml_value,
									   MCPWM_FAULT_MODE  mcpwm_fault_mode );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxHEnable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)

  Summary:
    Enables High Side output of the PWM Channel

  Description:
    This function enables the High Side output of the PWM Channel. 

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose PWMxH needs to be enabled.
	

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the PWM Channel whose PWMxH needs to be enabled.

   
    void PLIB_MCPWM_ChannelPWMxHEnable( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelGeneralFunctions in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxHEnable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);	

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxLEnable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)

  Summary:
    Enables Low Side output of the PWM Channel

  Description:
    This function enables the Low Side output of the PWM Channel. 

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose PWMxL needs to be enabled.
	

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the PWM Channel whose PWMxL needs to be enabled.

   
    PLIB_MCPWM_ChannelPWMxLEnable( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelGeneralFunctions in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxLEnable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);	

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxHDisable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)

  Summary:
    Disables High Side of the PWM Channel 

  Description:
    This function disables the High Side output of the PWM Channel. PWMxH pin acts as GPIO

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose PWMxH needs to be disabled.
	

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the PWM Channel whose PWMxH needs to be disabled.

   
    void PLIB_MCPWM_ChannelPWMxHDisable( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelGeneralFunctions in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxHDisable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);		

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxLDisable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)

  Summary:
    Disables low side of the PWM Channel 

  Description:
      This function disables the Low Side output of the PWM Channel. PWMxL pin acts as GPIO

  Precondition:
    None.

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose PWMxL needs to be disabled.
	

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the PWM Channel whose PWMxL needs to be disabled.

   
    PLIB_MCPWM_ChannelPWMxLDisable( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelGeneralFunctions in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxLDisable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel);	



// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxHOverrideEnable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel )

  Summary:
   Enables Override of PWMxH

  Description:
    This function enables override of PWMxH. The override status is defined in OVRDAT<1>

  Precondition:
   None.

  Parameters:
    index           - Identifier for the device instance to be configured
	channel			- MCPWM Channel ID whose PWMxH override is being enabled.
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose PWMxH override is being enabled.
	PLIB_MCPWM_ChannelPWMxHOverrideEnable(MY_MCPWM_INSTANCE,MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelOverrideSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxHOverrideEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxHOverrideDisable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel )

  Summary:
   Disables Override of PWMxH

  Description:
    This function disables override of PWMxH. 

  Precondition:
   None.

  Parameters:
    index           - Identifier for the device instance to be configured
	channel			- MCPWM Channel ID whose PWMxH override is being disabled.
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose PWMxH override is being disabled.
	PLIB_MCPWM_ChannelPWMxHOverrideDisable(MY_MCPWM_INSTANCE,MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelOverrideSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxHOverrideDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelPWMxLOverrideEnable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel )

  Summary:
   Enables Override of PWMxL

  Description:
    This function enables override of PWMxL. The override status is defined in OVRDAT<1>

  Precondition:
   None.

  Parameters:
    index           - Identifier for the device instance to be configured
	channel			- MCPWM Channel ID whose PWMxL override is being enabled.
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose PWMxL override is being enabled.
	PLIB_MCPWM_ChannelPWMxLOverrideEnable(MY_MCPWM_INSTANCE,MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelOverrideSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxLOverrideEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

/* Function:
    void PLIB_MCPWM_ChannelPWMxLOverrideDisable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel )

  Summary:
   Disables Override of PWMxL

  Description:
    This function disables override of PWMxL. 

  Precondition:
   None.

  Parameters:
    index           - Identifier for the device instance to be configured
	channel			- MCPWM Channel ID whose PWMxL override is being disabled.
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose PWMxL override is being disabled.
	PLIB_MCPWM_ChannelPWMxLOverrideDisable(MY_MCPWM_INSTANCE,MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelOverrideSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelPWMxLOverrideDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelOverrideOutputSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_OVERRIDE_PWMxH_VALUE override_pwmh_value,
	MCPWM_OVERRIDE_PWMxL_VALUE override_pwml_value)

  Summary:
    Specifies the MCPWM Override Output

  Description:
    This function specifies output value of the MCPWM during override. MCPWM can have any of the following outputs:
	
	
  Precondition:
    MCPWM Override must be enabled

  Parameters:
    index                   - Identifier for the device instance to be configured
    channel					- MCPWM Channel ID whose Override mode output needs to be defined.
	override_pwmh_value     - PWMxH Override Condition Output
							  MCPWM_OVERRIDE_PWMxH_0 (PWMxH = 0)
							  MCPWM_OVERRIDE_PWMxH_1 (PWMxH = 1)
	
	override_pwml_value     - PWMxL Override Condition Output
							  MCPWM_OVERRIDE_PWMxL_0 (PWMxL = 0)
							  MCPWM_OVERRIDE_PWMxL_1 (PWMxL = 1)

  Returns:
    None

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose override mode output is being defined.
    // During Override, PWMxH=0, PWMxL=0.
   
    void PLIB_MCPWM_ChannelOverrideOutputSet( MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelOverrideSetup in your application to determine whether this feature
	is available.
*/

  void PLIB_MCPWM_ChannelOverrideOutputSet( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, 
											MCPWM_OVERRIDE_PWMxH_VALUE override_pwmh_value,
											MCPWM_OVERRIDE_PWMxL_VALUE override_pwml_value);

 // *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelSwapHighLowEnable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel )

  Summary:
   Swap PWMxH and PWMxL Output

  Description:
    This function enables swapping of PWMxH and PWMxL output. 

  Precondition:
   None.

  Parameters:
    index           - Identifier for the device instance to be configured
	channel			- MCPWM Channel ID whose MCPWM outputs are to be swapped.
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose MCPWM outputs are to be swapped.
	PLIB_MCPWM_ChannelSwapHighLowEnable(MY_MCPWM_INSTANCE, MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelGeneralFunctions in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelSwapHighLowEnable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelSwapHighLowDisable( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel )

  Summary:
   Swap PWMxH and PWMxL Output

  Description:
    This function disables swapping of PWMxH and PWMxL output. 

  Precondition:
   None.

  Parameters:
    index           - Identifier for the device instance to be configured
	channel			- MCPWM Channel ID whose MCPWM outputs are NOT to be swapped.
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose MCPWM outputs are NOT to be swapped.
	PLIB_MCPWM_ChannelSwapHighLowDisable(MY_MCPWM_INSTANCE,MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelGeneralFunctions in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelSwapHighLowDisable( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );


// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel )

  Summary:
   Override outputs are asserted at the next MCPWM period boundary

  Description:
    This function ensures that Override outputs are asserted at the immediate next MCPWM period boundary after enabling override.

  Precondition:
   None.

  Parameters:
    index           - Identifier for the device instance to be configured
	channel			- MCPWM Channel ID whose Override outputs are synchronized with MCPWM Period Boundary.
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Override outputs are synchronized with MCPWM Period Boundary.
	PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary(MY_MCPWM_INSTANCE,MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelOverrideSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
/* Function:
    void PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary( MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel )

  Summary:
   Override outputs are asserted at the next CPU Clock boundary

  Description:
    This function ensures that Override outputs are asserted at the immediate next CPU Clock boundary after enabling override.

  Precondition:
   None.

  Parameters:
    index           - Identifier for the device instance to be configured
	channel			- MCPWM Channel ID whose Override outputs are synchronized with CPU Clock Boundary.
  Returns:
    None.

  Example:
    <code>
    // Where MY_MCPWM_INSTANCE, is the MCPWM instance selected for use by the
    // application developer.
	// MY_MCPWM_CHANNEL, is the MCPWM Channel whose Override outputs are synchronized with CPU Clock Boundary.
	PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary(MY_MCPWM_INSTANCE,MY_MCPWM_CHANNEL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the 
	specific device data sheet to determine availability or use 
	PLIB_MCPWM_ExistsChannelOverrideSetup in your application to determine whether this feature
	is available.
*/

void PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel );

// *****************************************************************************
// *****************************************************************************
// Section: MCPWM Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************

/* The following functions indicate the existence of the features on the device. */

//******************************************************************************

/* Function:
    PLIB_MCPWM_ExistsModuleControl( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ModuleControl feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ModuleControl feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_MCPWM_Enable
    - PLIB_MCPWM_Disable
	- PLIB_MCPWM_StopInIdleEnable
    - PLIB_MCPWM_StopInIdleDisable
	- PLIB_MCPWM_ModuleIsReady

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ModuleControl feature: 
    - true   - When ModuleControl feature is supported on the device
    - false  - When ModuleControl feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsModuleControl( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsPrimarySpecialEventTrigger( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the PrimarySpecialEventTrigger feature exists on the MCPWM module. 

  Description:
    This function identifies whether the PrimarySpecialEventTrigger feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - PrimarySpecialEventTriggerSetup
    - PrimarySpecialEventTriggerInterruptIsPending
	- PrimarySpecialEventTriggerInterruptEnable
    - PrimarySpecialEventTriggerInterruptDisable
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the PrimarySpecialEventTrigger feature: 
    - true   - When PrimarySpecialEventTrigger feature is supported on the device
    - false  - When PrimarySpecialEventTrigger feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsPrimarySpecialEventTrigger( MCPWM_MODULE_ID index );
/* Function:
    PLIB_MCPWM_ExistsSecondarySpecialEventTrigger( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the SecondarySpecialEventTrigger feature exists on the MCPWM module. 

  Description:
    This function identifies whether the SecondarySpecialEventTrigger feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - SecondarySpecialEventTriggerSetup
    - SecondarySpecialEventTriggerInterruptIsPending
	- SecondarySpecialEventTriggerInterruptEnable
    - SecondarySpecialEventTriggerInterruptDisable
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the SecondarySpecialEventTrigger feature: 
    - true   - When SecondarySpecialEventTrigger feature is supported on the device
    - false  - When SecondarySpecialEventTrigger feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsSecondarySpecialEventTrigger( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsPrimaryTimerSetup( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the PrimaryTimerSetup feature exists on the MCPWM module. 

  Description:
    This function identifies whether the PrimaryTimerSetup feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - PrimaryTimerSetup
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the PrimaryTimerSetup feature: 
    - true   - When PrimaryTimerSetup feature is supported on the device
    - false  - When PrimaryTimerSetup feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsPrimaryTimerSetup( MCPWM_MODULE_ID index );
/* Function:
    PLIB_MCPWM_ExistsSecondaryTimerSetup( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the SecondaryTimerSetup feature exists on the MCPWM module. 

  Description:
    This function identifies whether the SecondaryTimerSetup feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - SecondaryTimerSetup
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the SecondaryTimerSetup feature: 
    - true   - When SecondaryTimerSetup feature is supported on the device
    - false  - When SecondaryTimerSetup feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsSecondaryTimerSetup( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChopClockSetup( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChopClockSetup feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChopClockSetup feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChopClockSetup
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChopClockSetup feature: 
    - true   - When ChopClockSetup feature is supported on the device
    - false  - When ChopClockSetup feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChopClockSetup( MCPWM_MODULE_ID index );
/* Function:
    PLIB_MCPWM_ExistsChannelSetup( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelSetup feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelSetup feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelSetup
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelSetup feature: 
    - true   - When ChannelSetup feature is supported on the device
    - false  - When ChannelSetup feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelSetup( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelPrimaryDutyCycleSet( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelPrimaryDutyCycleSet feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelPrimaryDutyCycleSet feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelPrimaryDutyCycleSet
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelPrimaryDutyCycleSet feature: 
    - true   - When ChannelPrimaryDutyCycleSet feature is supported on the device
    - false  - When ChannelPrimaryDutyCycleSet feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelPrimaryDutyCycleSet( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelSecondaryDutyCycleSet( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelSecondaryDutyCycleSet feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelSecondaryDutyCycleSet feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelSecondaryDutyCycleSet
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelSecondaryDutyCycleSet feature: 
    - true   - When ChannelSecondaryDutyCycleSet feature is supported on the device
    - false  - When ChannelSecondaryDutyCycleSet feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelSecondaryDutyCycleSet( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelPhaseSet( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelPhaseSet feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelPhaseSet feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelPhaseSet
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelPhaseSet feature: 
    - true   - When ChannelPhaseSet feature is supported on the device
    - false  - When ChannelPhaseSet feature is not supported on the device

   Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelPhaseSet( MCPWM_MODULE_ID index );
/* Function:
    PLIB_MCPWM_ExistsChannelDeadtimeSetup( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelDeadtimeSetup feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelDeadtimeSetup feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelPWMxHDeadtimeSet
	- ChannelPWMxLDeadtimeSet
	- ChannelDeadtimeCompSet
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelDeadtimeSetup feature: 
    - true   - When ChannelDeadtimeSetup feature is supported on the device
    - false  - When ChannelDeadtimeSetup feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelDeadtimeSetup( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelTrigger( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelTrigger feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelTrigger feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelTriggerSetup
	- ChannelTriggerInterruptIsPending
	- ChannelTriggerInterruptFlagClear
	- ChannelTriggerInterruptEnable
	- ChannelTriggerInterruptDisable
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelTrigger feature: 
    - true   - When ChannelTrigger feature is supported on the device
    - false  - When ChannelTrigger feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelTrigger( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelCurrentLimit( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelCurrentLimit feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelCurrentLimit feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelCurrentLimitSetup
	- ChannelCurrentLimitInterruptIsPending
	- ChannelCurrentLimitInterruptFlagClear
	- ChannelCurrentLimitInterruptEnable
	- ChannelCurrentLimitInterruptDisable
	- ChannelCurrentLimitCaptureRead
	- ChannelCurrentLimitIsAsserted
	
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelCurrentLimit feature: 
    - true   - When ChannelCurrentLimit feature is supported on the device
    - false  - When ChannelCurrentLimit feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelCurrentLimit( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelFault( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelFault feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelFault feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelFaultSetup
	- ChannelFaultInterruptIsPending
	- ChannelFaultInterruptFlagClear
	- ChannelFaultInterruptEnable
	- ChannelFaultInterruptDisable
	
	
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelFault feature: 
    - true   - When ChannelFault feature is supported on the device
    - false  - When ChannelFault feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelFault( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelChopSetup( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelChopSetup feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelChopSetup feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelChopSetup
	
	
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelChopSetup feature: 
    - true   - When ChannelChopSetup feature is supported on the device
    - false  - When ChannelChopSetup feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelChopSetup( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelLEBSetup( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelLEBSetup feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelLEBSetup feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelLEBSetup
	
	
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelLEBSetup feature: 
    - true   - When ChannelLEBSetup feature is supported on the device
    - false  - When ChannelLEBSetup feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelLEBSetup( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelPeriodResetInterrupt( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelPeriodResetInterrupt feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelPeriodResetInterrupt feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelPeriodResetInterruptIsPending
	- ChannelPeriodResetInterruptFlagClear
	- ChannelPeriodResetInterruptEnable
	- ChannelPeriodResetInterruptDisable
	
	
	
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelPeriodResetInterrupt feature: 
    - true   - When ChannelPeriodResetInterrupt feature is supported on the device
    - false  - When ChannelPeriodResetInterrupt feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelPeriodResetInterrupt( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelPeriodMatchInterrupt feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelPeriodMatchInterrupt feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelPeriodMatchInterruptIsPending
	- ChannelPeriodMatchInterruptFlagClear
	- ChannelPeriodMatchInterruptEnable
	- ChannelPeriodMatchInterruptDisable
	
	
	
	

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelPeriodMatchInterrupt feature: 
    - true   - When ChannelPeriodMatchInterrupt feature is supported on the device
    - false  - When ChannelPeriodMatchInterrupt feature is not supported on the device

  Remarks:
	None.
 */

 
bool PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt( MCPWM_MODULE_ID index );

/* Function:
    PLIB_MCPWM_ExistsChannelOverrideSetup( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelOverrideSetup feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelOverrideSetup feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelPWMxHOverrideEnable
	- ChannelPWMxHOverrideDisable
	- ChannelPWMxLOverrideEnable
	- ChannelPWMxLOverrideDisable
	- ChannelOverrideOutputSet
	- ChannelSyncOverrideAtPeriodBoundary
	- ChannelSyncOverrideAtCPUClockBoundary

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelOverrideSetup feature: 
    - true   - When ChannelOverrideSetup feature is supported on the device
    - false  - When ChannelOverrideSetup feature is not supported on the device

  Remarks:
	None.
 */
 bool PLIB_MCPWM_ExistsChannelOverrideSetup( MCPWM_MODULE_ID index );

 /* Function:
    PLIB_MCPWM_ExistsChannelGeneralFunctions( MCPWM_MODULE_ID index )

  Summary:
    Identifies whether the ChannelGeneralFunctions feature exists on the MCPWM module. 

  Description:
    This function identifies whether the ChannelGeneralFunctions feature is available on 
    the MCPWM module.
    When this function returns true, these functions are supported on the device: 
    - ChannelLocalPWMTimerCountRead
	- IOCONxUnlock
	- ChannelTimerDirectionGet
	- ChannelPWMxHEnable
	- ChannelPWMxHDisable
	- ChannelPWMxLEnable
	- ChannelPWMxLDisable
	- ChannelSwapHighLowEnable
	- ChannelSwapHighLowDisable

	
  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    Existence of the ChannelGeneralFunctions feature: 
    - true   - When ChannelGeneralFunctions feature is supported on the device
    - false  - When ChannelGeneralFunctions feature is not supported on the device

  Remarks:
	None.
 */

bool PLIB_MCPWM_ExistsChannelGeneralFunctions( MCPWM_MODULE_ID index );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // #ifndef _PLIB_MCPWM_ChannelH
/*******************************************************************************
 End of File
*/