/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelCurrentLimit_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelCurrentLimit
    and its Variant : Unsupported
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

#ifndef _MCPWM_CHANNELCURRENTLIMIT_UNSUPPORTED_H
#define _MCPWM_CHANNELCURRENTLIMIT_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelCurrentLimitSetup 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelCurrentLimitSetup function.
*/

PLIB_TEMPLATE void MCPWM_ChannelCurrentLimitSetup_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , MCPWM_CURRENTLIMIT_SOURCE current_limit_source , MCPWM_CURRENTLIMIT_INPUT_POLARITY current_limit_input_polarity , MCPWM_CURRENTLIMIT_OVERRIDE_DATA current_limit_override_data , MCPWM_CURRENTLIMIT_MODE current_limit_mode )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelCurrentLimitSetup");
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptIsPending_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelCurrentLimitInterruptIsPending_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending");

    return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptIsEnabled_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelCurrentLimitInterruptIsEnabled_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled");

    return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptFlagClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear function.
*/

PLIB_TEMPLATE void MCPWM_ChannelCurrentLimitInterruptFlagClear_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear");
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelCurrentLimitInterruptEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelCurrentLimitInterruptEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelCurrentLimitInterruptEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitInterruptDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelCurrentLimitInterruptDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelCurrentLimitInterruptDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelCurrentLimitInterruptDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelCurrentLimitInterruptDisable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitCaptureRead_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelCurrentLimitCaptureRead 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelCurrentLimitCaptureRead function.
*/

PLIB_TEMPLATE uint16_t MCPWM_ChannelCurrentLimitCaptureRead_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelCurrentLimitCaptureRead");

    return (uint16_t) 0;
}


//******************************************************************************
/* Function :  MCPWM_ChannelCurrentLimitIsAsserted_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelCurrentLimitIsAsserted 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelCurrentLimitIsAsserted function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelCurrentLimitIsAsserted_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelCurrentLimitIsAsserted");

    return false;
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelCurrentLimit_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsChannelCurrentLimit

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsChannelCurrentLimit function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsChannelCurrentLimit_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_CHANNELCURRENTLIMIT_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

