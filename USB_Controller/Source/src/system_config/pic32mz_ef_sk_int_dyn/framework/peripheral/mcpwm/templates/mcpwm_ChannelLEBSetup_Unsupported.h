/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelLEBSetup_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelLEBSetup
    and its Variant : Unsupported
    For following APIs :
        PLIB_MCPWM_ChannelLEBSetup
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

#ifndef _MCPWM_CHANNELLEBSETUP_UNSUPPORTED_H
#define _MCPWM_CHANNELLEBSETUP_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_ChannelLEBSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelLEBSetup 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelLEBSetup function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLEBSetup_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , MCPWM_LEB_TRIGGER_SOURCES mcpwm_leb_trigger_sources , MCPWM_FAULT_INPUT_LEB_CONTROL mcpwm_fault_input_leb_control , MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL mcpwm_currentlimit_input_leb_control , uint16_t leb_delay )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelLEBSetup");
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelLEBSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsChannelLEBSetup

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsChannelLEBSetup function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsChannelLEBSetup_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_CHANNELLEBSETUP_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

