/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelCurrentLimit_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelCurrentLimit
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_ChannelCurrentLimitSetup
        PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending
		PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled
        PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear
        PLIB_MCPWM_ChannelCurrentLimitInterruptEnable
        PLIB_MCPWM_ChannelCurrentLimitInterruptDisable
        PLIB_MCPWM_ChannelCurrentLimitCaptureRead
        PLIB_MCPWM_ChannelCurrentLimitIsAsserted
        PLIB_MCPWM_ExistsChannelCurrentLimit

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

#ifndef _MCPWM_CHANNELCURRENTLIMIT_DEFAULT_H
#define _MCPWM_CHANNELCURRENTLIMIT_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelCurrentLimitSetup 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelCurrentLimitSetup function.
*/

PLIB_TEMPLATE void MCPWM_ChannelCurrentLimitSetup_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel ,
 MCPWM_CURRENTLIMIT_SOURCE current_limit_source , MCPWM_CURRENTLIMIT_INPUT_POLARITY current_limit_input_polarity ,
 MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE current_limit_override_pwmh_value , MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE current_limit_override_pwml_value,
 MCPWM_CURRENTLIMIT_MODE current_limit_mode )
{
	volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
	volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	unsigned int temp_current_limit_mode = 0;
    channel_regs->PWMCONxCLR = PWMCONx_XPRES_MASK;
    if((current_limit_mode == 0) | (current_limit_mode == 2))
    {
    temp_current_limit_mode = 0;
    }
    else
    {
        temp_current_limit_mode = 1;
    }
    module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
    channel_regs->IOCONxCLR = 0xFF000000;
    
    unsigned int temp = current_limit_source << 26;
    temp = temp | (current_limit_input_polarity << 25);
    temp = temp | (temp_current_limit_mode << 24);
	module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
	channel_regs->IOCONxSET = temp;
	channel_regs->IOCONx.CLDAT = ((current_limit_override_pwmh_value << 1)|current_limit_override_pwml_value);
	if(current_limit_mode == 2)
    {
        channel_regs->PWMCONxSET = PWMCONx_ITB_MASK;
        channel_regs->PWMCONxSET = PWMCONx_XPRES_MASK;
    }
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptIsPending_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelCurrentLimitInterruptIsPending_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
   volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.CLIF == 0b1)
		return true;
	else
		return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelCurrentLimitInterruptIsEnabled_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
   volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.CLIEN == 0b1)
		return true;
	else
		return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptFlagClear_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear function.
*/

PLIB_TEMPLATE void MCPWM_ChannelCurrentLimitInterruptFlagClear_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxCLR = PWMCONx_CLIF_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelCurrentLimitInterruptEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelCurrentLimitInterruptEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
     volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxSET = PWMCONx_CLIEN_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelCurrentLimitInterruptDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelCurrentLimitInterruptDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxCLR = PWMCONx_CLIEN_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitCaptureRead_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelCurrentLimitCaptureRead 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelCurrentLimitCaptureRead function.
*/

PLIB_TEMPLATE uint16_t MCPWM_ChannelCurrentLimitCaptureRead_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	uint16_t capture_value = channel_regs->CAPx;
	return capture_value;
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitIsAsserted_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelCurrentLimitIsAsserted 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelCurrentLimitIsAsserted function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelCurrentLimitIsAsserted_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.CLTSTAT == 0b1)
		return true;
	else
		return false;
}



//******************************************************************************
/* Function :  MCPWM_ExistsChannelCurrentLimit_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsChannelCurrentLimit

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsChannelCurrentLimit function.
*/

#define PLIB_MCPWM_ExistsChannelCurrentLimit PLIB_MCPWM_ExistsChannelCurrentLimit
PLIB_TEMPLATE bool MCPWM_ExistsChannelCurrentLimit_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_CHANNELCURRENTLIMIT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

