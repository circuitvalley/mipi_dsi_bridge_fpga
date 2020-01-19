/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelPeriodMatchInterrupt_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelPeriodMatchInterrupt
    and its Variant : Unsupported
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

#ifndef _MCPWM_CHANNELPERIODMATCHINTERRUPT_UNSUPPORTED_H
#define _MCPWM_CHANNELPERIODMATCHINTERRUPT_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptIsPending_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelPeriodMatchInterruptIsPending_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending");

    return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptIsEnabled_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelPeriodMatchInterruptIsEnabled_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled");

    return false;
}
//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptFlagClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPeriodMatchInterruptFlagClear_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear");
}


//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPeriodMatchInterruptEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPeriodMatchInterruptEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPeriodMatchInterruptEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelPeriodMatchInterruptDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPeriodMatchInterruptDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPeriodMatchInterruptDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPeriodMatchInterruptDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPeriodMatchInterruptDisable");
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelPeriodMatchInterrupt_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsChannelPeriodMatchInterrupt_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_CHANNELPERIODMATCHINTERRUPT_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

