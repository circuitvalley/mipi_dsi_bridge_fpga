/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelDeadtimeSetup_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelDeadtimeSetup
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_ChannelPWMxHDeadtimeSet
        PLIB_MCPWM_ChannelPWMxLDeadtimeSet
        PLIB_MCPWM_ChannelDeadtimeCompSet
        PLIB_MCPWM_ExistsChannelDeadtimeSetup

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

#ifndef _MCPWM_CHANNELDEADTIMESETUP_DEFAULT_H
#define _MCPWM_CHANNELDEADTIMESETUP_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_ChannelPWMxHDeadtimeSet_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPWMxHDeadtimeSet 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPWMxHDeadtimeSet function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxHDeadtimeSet_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , uint16_t value )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->DTRx = value;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxLDeadtimeSet_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPWMxLDeadtimeSet 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPWMxLDeadtimeSet function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxLDeadtimeSet_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , uint16_t value )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->ALTDTRx = value;
}


//******************************************************************************
/* Function :  MCPWM_ChannelDeadtimeCompSet_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelDeadtimeCompSet 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelDeadtimeCompSet function.
*/

PLIB_TEMPLATE void MCPWM_ChannelDeadtimeCompSet_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , uint16_t value )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->DTCOMPx = value;
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelDeadtimeSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsChannelDeadtimeSetup

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsChannelDeadtimeSetup function.
*/

#define PLIB_MCPWM_ExistsChannelDeadtimeSetup PLIB_MCPWM_ExistsChannelDeadtimeSetup
PLIB_TEMPLATE bool MCPWM_ExistsChannelDeadtimeSetup_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_CHANNELDEADTIMESETUP_DEFAULT_H*/

/******************************************************************************
 End of File
*/

