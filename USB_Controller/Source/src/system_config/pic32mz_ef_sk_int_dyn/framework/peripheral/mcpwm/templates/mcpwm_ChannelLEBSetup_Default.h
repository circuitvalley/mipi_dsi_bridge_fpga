/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelLEBSetup_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelLEBSetup
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_ChannelLEBSetup
		PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable
		PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable
		PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable
		PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable
		PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable
		PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable
		PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable
		PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable
        PLIB_MCPWM_ExistsChannelLEBSetup

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

#ifndef _MCPWM_CHANNELLEBSETUP_DEFAULT_H
#define _MCPWM_CHANNELLEBSETUP_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_ChannelLEBSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBSetup 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBSetup function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBSetup_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , MCPWM_FAULT_INPUT_LEB_CONTROL mcpwm_fault_input_leb_control , MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL mcpwm_currentlimit_input_leb_control , uint16_t leb_delay )
{
   volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
			
	if(mcpwm_fault_input_leb_control == MCPWM_FAULT_INPUT_LEB_ENABLE)
	{
		channel_regs->LEBCONx.FLTLEBEN = 0b1;
	}
	else
	{
		channel_regs->LEBCONx.FLTLEBEN = 0b0;
	}
	
	if(mcpwm_currentlimit_input_leb_control == MCPWM_CURRENTLIMIT_INPUT_LEB_ENABLE)
	{
		channel_regs->LEBCONx.CLLEBEN = 0b1;
	}
	else
	{
		channel_regs->LEBCONx.CLLEBEN = 0b0;
	}
	
	channel_regs->LEBDLYx = leb_delay;
			
}


//******************************************************************************
/* Function :  MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)
{
	 volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	 channel_regs->LEBCONxSET = LEBCONx_PHR_MASK;
}
//******************************************************************************
/* Function :  MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)
{
	 volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	 channel_regs->LEBCONxSET = LEBCONx_PHF_MASK;
}




//******************************************************************************
/* Function :  MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)
{
	 volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	 channel_regs->LEBCONxSET = LEBCONx_PLR_MASK;
}

//******************************************************************************
/* Function :  MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)
{
	 volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	 channel_regs->LEBCONxSET = LEBCONx_PLF_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)
{
	 volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	 channel_regs->LEBCONxCLR = LEBCONx_PHR_MASK;
}
//******************************************************************************
/* Function :  MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)
{
	 volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	 channel_regs->LEBCONxCLR = LEBCONx_PHF_MASK;
}




//******************************************************************************
/* Function :  MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)
{
	 volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	 channel_regs->LEBCONxCLR = LEBCONx_PLR_MASK;
}

//******************************************************************************
/* Function :  MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel)
{
	 volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	 channel_regs->LEBCONxCLR = LEBCONx_PLF_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelLEBSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsChannelLEBSetup

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsChannelLEBSetup function.
*/

#define PLIB_MCPWM_ExistsChannelLEBSetup PLIB_MCPWM_ExistsChannelLEBSetup
PLIB_TEMPLATE bool MCPWM_ExistsChannelLEBSetup_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_CHANNELLEBSETUP_DEFAULT_H*/

/******************************************************************************
 End of File
*/

