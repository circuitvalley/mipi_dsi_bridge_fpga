/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelFault_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelFault
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_ChannelFaultSetup
        PLIB_MCPWM_ChannelFaultInterruptIsPending
		PLIB_MCPWM_ChannelFaultInterruptIsEnabled
        PLIB_MCPWM_ChannelFaultInterruptFlagClear
        PLIB_MCPWM_ChannelFaultInterruptEnable
        PLIB_MCPWM_ChannelFaultInterruptDisable
        PLIB_MCPWM_ChannelFaultIsAsserted
        PLIB_MCPWM_ExistsChannelFault

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _MCPWM_CHANNELFAULT_DEFAULT_H
#define _MCPWM_CHANNELFAULT_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_ChannelFaultSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelFaultSetup 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelFaultSetup function.
*/

PLIB_TEMPLATE void MCPWM_ChannelFaultSetup_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , 
MCPWM_FAULT_SOURCE mcpwm_fault_source , MCPWM_FAULT_INPUT_POLARITY mcpwm_fault_input_polarity , 
MCPWM_FAULT_OVERRIDE_PWMxH_VALUE mcpwm_fault_override_pwmh_value, MCPWM_FAULT_OVERRIDE_PWMxL_VALUE mcpwm_fault_override_pwml_value,
MCPWM_FAULT_MODE  mcpwm_fault_mode )
{
    volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	
    module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
    channel_regs->IOCONxCLR = 0x00FF0000;
    
    unsigned int temp = mcpwm_fault_source << 19;
    temp = temp | (mcpwm_fault_input_polarity << 18);
    temp = temp | (mcpwm_fault_mode<< 16);
    
	module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
	channel_regs->IOCONxSET = temp;
	channel_regs->IOCONx.FLTDAT = ((mcpwm_fault_override_pwmh_value<<1)|mcpwm_fault_override_pwml_value);

}


//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptIsPending_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelFaultInterruptIsPending 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelFaultInterruptIsPending function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelFaultInterruptIsPending_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
     volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.FLTIF == 0b1)
		return true;
	else
		return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelFaultInterruptIsEnabled 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelFaultInterruptIsEnabled function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelFaultInterruptIsEnabled_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
     volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.FLTIEN == 0b1)
		return true;
	else
		return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptFlagClear_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelFaultInterruptFlagClear 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelFaultInterruptFlagClear function.
*/

PLIB_TEMPLATE void MCPWM_ChannelFaultInterruptFlagClear_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
   volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxCLR = PWMCONx_FLTIF_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelFaultInterruptEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelFaultInterruptEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelFaultInterruptEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxSET = PWMCONx_FLTIEN_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelFaultInterruptDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelFaultInterruptDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelFaultInterruptDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxCLR = PWMCONx_FLTIEN_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelFaultIsAsserted_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelFaultIsAsserted 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelFaultIsAsserted function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelFaultIsAsserted_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.FLTSTAT == 0b1)
		return true;
	else
		return false;
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelFault_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsChannelFault

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsChannelFault function.
*/

#define PLIB_MCPWM_ExistsChannelFault PLIB_MCPWM_ExistsChannelFault
PLIB_TEMPLATE bool MCPWM_ExistsChannelFault_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_CHANNELFAULT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

