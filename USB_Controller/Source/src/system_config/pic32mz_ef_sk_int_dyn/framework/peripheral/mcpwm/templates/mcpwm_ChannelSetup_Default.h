/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelSetup_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelSetup
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_ChannelSetup
        PLIB_MCPWM_ExistsChannelSetup

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

#ifndef _MCPWM_CHANNELSETUP_DEFAULT_H
#define _MCPWM_CHANNELSETUP_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_ChannelSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelSetup 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelSetup function.
*/

PLIB_TEMPLATE void MCPWM_ChannelSetup_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel_id ,
 MCPWM_TIME_BASE_SOURCE time_base_source , MCPWM_TIME_BASE_MODE time_base_mode,	MCPWM_ALIGNMENT_MODE mcpwm_alignment_mode ,
 MCPWM_OUTPUT_MODE mcpwm_output_mode , MCPWM_PWMxH_OUTPUT_POLARITY mcpwm_pwmh_polarity , MCPWM_PWMxL_OUTPUT_POLARITY mcpwm_pwml_polarity,
 MCPWM_DEADTIME_MODE mcpwm_deadtime_mode , MCPWM_DEADTIME_COMPENSATION_POLARITY mcpwm_deadtime_compensation_polarity )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel_id;
	
    channel_regs->IOCONx.POLH=mcpwm_pwmh_polarity;
	channel_regs->IOCONx.POLL=mcpwm_pwml_polarity;
	
    channel_regs->PWMCONx.MTBS = time_base_source;
	channel_regs->PWMCONx.ITB = time_base_mode;
	channel_regs->PWMCONx.ECAM = mcpwm_alignment_mode;
	channel_regs->IOCONx.PMOD = mcpwm_output_mode;
	channel_regs->PWMCONx.DTC = mcpwm_deadtime_mode;
	channel_regs->PWMCONx.DTCP = mcpwm_deadtime_compensation_polarity;
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsChannelSetup

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsChannelSetup function.
*/

#define PLIB_MCPWM_ExistsChannelSetup PLIB_MCPWM_ExistsChannelSetup
PLIB_TEMPLATE bool MCPWM_ExistsChannelSetup_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_CHANNELSETUP_DEFAULT_H*/

/******************************************************************************
 End of File
*/

