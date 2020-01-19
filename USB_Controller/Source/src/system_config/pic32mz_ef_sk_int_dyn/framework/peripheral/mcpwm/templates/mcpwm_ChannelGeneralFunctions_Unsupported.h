/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelGeneralFunctions_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelGeneralFunctions
    and its Variant : Unsupported
    For following APIs :
        PLIB_MCPWM_ChannelLocalPWMTimerCountRead
        PLIB_MCPWM_IOCONxUnlock
        PLIB_MCPWM_ChannelTimerDirectionGet
        PLIB_MCPWM_ChannelPWMxHEnable
        PLIB_MCPWM_ChannelPWMxHDisable
        PLIB_MCPWM_ChannelPWMxLEnable
        PLIB_MCPWM_ChannelPWMxLDisable
        PLIB_MCPWM_ChannelSwapHighLowEnable
        PLIB_MCPWM_ChannelSwapHighLowDisable
        PLIB_MCPWM_ExistsChannelGeneralFunctions

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

#ifndef _MCPWM_CHANNELGENERALFUNCTIONS_UNSUPPORTED_H
#define _MCPWM_CHANNELGENERALFUNCTIONS_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_ChannelLocalPWMTimerCountRead_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelLocalPWMTimerCountRead 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelLocalPWMTimerCountRead function.
*/

PLIB_TEMPLATE void MCPWM_ChannelLocalPWMTimerCountRead_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelLocalPWMTimerCountRead");
}


//******************************************************************************
/* Function :  MCPWM_IOCONxUnlock_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_IOCONxUnlock 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_IOCONxUnlock function.
*/

PLIB_TEMPLATE void MCPWM_IOCONxUnlock_Unsupported( MCPWM_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_IOCONxUnlock");
}


//******************************************************************************
/* Function :  MCPWM_ChannelTimerDirectionGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelTimerDirectionGet 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelTimerDirectionGet function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelTimerDirectionGet_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelTimerDirectionGet");

    return false;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxHEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPWMxHEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPWMxHEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxHEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPWMxHEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxHDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPWMxHDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPWMxHDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxHDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPWMxHDisable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxLEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPWMxLEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPWMxLEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxLEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPWMxLEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxLDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelPWMxLDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelPWMxLDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxLDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelPWMxLDisable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelSwapHighLowEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelSwapHighLowEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelSwapHighLowEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelSwapHighLowEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelSwapHighLowEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelSwapHighLowDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelSwapHighLowDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelSwapHighLowDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelSwapHighLowDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelSwapHighLowDisable");
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelGeneralFunctions_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsChannelGeneralFunctions

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsChannelGeneralFunctions function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsChannelGeneralFunctions_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_CHANNELGENERALFUNCTIONS_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

