/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelFault_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelFault
    and its Variant : Unsupported
    For following APIs :
        PLIB_MCPWM_ChannelFaultSetup
        PLIB_MCPWM_ChannelFaultInterruptIsPending
		PLIB_MCPWM_ChannelFaultInterruptIsEnabled
        PLIB_MCPWM_ChannelFaultInterruptFlagClear
        PLIB_MCPWM_ChannelFaultInterruptEnable
        PLIB_MCPWM_ChannelFaultInterruptDisable
        PLIB_MCPWM_ChannelFaultIsAsserted
        PLIB_MCPWM_ExistsChannelFault

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

#ifndef _MCPWM_CHANNELFAULT_UNSUPPORTED_H
#define _MCPWM_CHANNELFAULT_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_ChannelFaultSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelFaultSetup 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelFaultSetup function.
*/

PLIB_TEMPLATE void MCPWM_ChannelFaultSetup_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , MCPWM_FAULT_SOURCE mcpwm_fault_source , MCPWM_FAULT_INPUT_POLARITY mcpwm_fault_input_polarity , MCPWM_FAULT_OVERRIDE_DATA mcpwm_fault_override_data , MCPWM_FAULT_MODE  mcpwm_fault_mode )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelFaultSetup");
}


//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptIsPending_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelFaultInterruptIsPending 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelFaultInterruptIsPending function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelFaultInterruptIsPending_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelFaultInterruptIsPending");

    return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptIsEnabled_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelFaultInterruptIsEnabled 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelFaultInterruptIsEnabled function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelFaultInterruptIsEnabled_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelFaultInterruptIsEnabled");

    return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptFlagClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelFaultInterruptFlagClear 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelFaultInterruptFlagClear function.
*/

PLIB_TEMPLATE void MCPWM_ChannelFaultInterruptFlagClear_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelFaultInterruptFlagClear");
}


//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelFaultInterruptEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelFaultInterruptEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelFaultInterruptEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelFaultInterruptEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelFaultInterruptDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelFaultInterruptDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelFaultInterruptDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelFaultInterruptDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelFaultInterruptDisable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelFaultIsAsserted_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelFaultIsAsserted 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelFaultIsAsserted function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelFaultIsAsserted_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelFaultIsAsserted");

    return false;
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelFault_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsChannelFault

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsChannelFault function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsChannelFault_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_CHANNELFAULT_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

