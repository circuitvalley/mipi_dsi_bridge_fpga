/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelPeriodMatchInterrupt_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelPeriodMatchInterrupt
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending
		PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled
        PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear
        PLIB_MCPWM_ChannelPeriodMatchInterruptEnable
        PLIB_MCPWM_ChannelPeriodMatchInterruptDisable
        PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt

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

#ifndef _MCPWM_CHANNELPERIODMATCHINTERRUPT_DEFAULT_H
#define _MCPWM_CHANNELPERIODMATCHINTERRUPT_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptIsPending_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelPeriodMatchInterruptIsPending_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
     volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.PWMHIF == 0b1)
		return true;
	else
		return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelPeriodMatchInterruptIsEnabled_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
     volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.PWMHIEN == 0b1)
		return true;
	else
		return false;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptFlagClear_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPeriodMatchInterruptFlagClear_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxCLR = PWMCONx_PWMHIF_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPeriodMatchInterruptEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPeriodMatchInterruptEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxSET = PWMCONx_PWMHIEN_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPeriodMatchInterruptDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPeriodMatchInterruptDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
     volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->PWMCONxCLR = PWMCONx_PWMHIEN_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelPeriodMatchInterrupt_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt function.
*/

#define PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt
PLIB_TEMPLATE bool MCPWM_ExistsChannelPeriodMatchInterrupt_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_CHANNELPERIODMATCHINTERRUPT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

