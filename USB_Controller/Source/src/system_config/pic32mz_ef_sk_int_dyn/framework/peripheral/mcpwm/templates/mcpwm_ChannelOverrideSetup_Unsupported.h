/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelOverrideSetup_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelOverrideSetup
    and its Variant : Unsupported
    For following APIs :
        PLIB_MCPWM_ChannelPWMxHOverrideEnable
        PLIB_MCPWM_ChannelPWMxHOverrideDisable
        PLIB_MCPWM_ChannelPWMxLOverrideEnable
        PLIB_MCPWM_ChannelPWMxLOverrideDisable
        PLIB_MCPWM_ChannelOverrideOutputSet
        PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary
        PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary
        PLIB_MCPWM_ExistsChannelOverrideSetup

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

#ifndef _MCPWM_CHANNELOVERRIDESETUP_UNSUPPORTED_H
#define _MCPWM_CHANNELOVERRIDESETUP_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_ChannelPWMxHOverrideEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPWMxHOverrideEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPWMxHOverrideEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxHOverrideEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPWMxHOverrideEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxHOverrideDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPWMxHOverrideDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPWMxHOverrideDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxHOverrideDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPWMxHOverrideDisable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxLOverrideEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPWMxLOverrideEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPWMxLOverrideEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxLOverrideEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPWMxLOverrideEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxLOverrideDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPWMxLOverrideDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPWMxLOverrideDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxLOverrideDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPWMxLOverrideDisable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelOverrideOutputSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelOverrideOutputSet 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelOverrideOutputSet function.
*/

PLIB_TEMPLATE void MCPWM_ChannelOverrideOutputSet_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , MCPWM_OVERRIDE_DATA override_value )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelOverrideOutputSet");
}


//******************************************************************************
/* Function :  MCPWM_ChannelSyncOverrideAtPeriodBoundary_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary function.
*/

PLIB_TEMPLATE void MCPWM_ChannelSyncOverrideAtPeriodBoundary_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary");
}


//******************************************************************************
/* Function :  MCPWM_ChannelSyncOverrideAtCPUClockBoundary_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary function.
*/

PLIB_TEMPLATE void MCPWM_ChannelSyncOverrideAtCPUClockBoundary_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary");
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelOverrideSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsChannelOverrideSetup

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsChannelOverrideSetup function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsChannelOverrideSetup_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_CHANNELOVERRIDESETUP_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

