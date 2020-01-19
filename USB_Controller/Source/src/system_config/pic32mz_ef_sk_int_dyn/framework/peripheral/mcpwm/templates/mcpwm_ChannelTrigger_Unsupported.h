/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelTrigger_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelTrigger
    and its Variant : Unsupported
    For following APIs :
        PLIB_MCPWM_ChannelTriggerSetup
        PLIB_MCPWM_ChannelTriggerInterruptIsPending
		PLIB_MCPWM_ChannelTriggerInterruptIsEnabled
        PLIB_MCPWM_ChannelTriggerInterruptFlagClear
        PLIB_MCPWM_ChannelTriggerInterruptEnable
        PLIB_MCPWM_ChannelTriggerInterruptDisable
        PLIB_MCPWM_ExistsChannelTrigger

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

#ifndef _MCPWM_CHANNELTRIGGER_UNSUPPORTED_H
#define _MCPWM_CHANNELTRIGGER_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_ChannelTriggerSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelTriggerSetup 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelTriggerSetup function.
*/

PLIB_TEMPLATE void MCPWM_ChannelTriggerSetup_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel , MCPWM_TRIGGER_DIVIDER trigger_postscaler_value , MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT primary_trigger_cycle_select , MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT secondary_trigger_cycle_select , MCPWM_ADC_TRIGGER_SOURCE mcpwm_adc_trigger_source , MCPWM_TRIGGER_INTERRUPT_SOURCE mcpwm_trigger_interrupt_source , uint16_t primary_trigger_compare_value , uint16_t secondary_trigger_compare_value )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelTriggerSetup");
}


//******************************************************************************
/* Function :  MCPWM_ChannelTriggerInterruptIsPending_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelTriggerInterruptIsPending 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelTriggerInterruptIsPending function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelTriggerInterruptIsPending_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelTriggerInterruptIsPending");

    return false;
}
//******************************************************************************
/* Function :  MCPWM_ChannelTriggerInterruptIsEnabled_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelTriggerInterruptIsEnabled 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelTriggerInterruptIsEnabled function.
*/

PLIB_TEMPLATE bool MCPWM_ChannelTriggerInterruptIsEnabled_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelTriggerInterruptIsEnabled");

    return false;
}

//******************************************************************************
/* Function :  MCPWM_ChannelTriggerInterruptFlagClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelTriggerInterruptFlagClear 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelTriggerInterruptFlagClear function.
*/

PLIB_TEMPLATE void MCPWM_ChannelTriggerInterruptFlagClear_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelTriggerInterruptFlagClear");
}


//******************************************************************************
/* Function :  MCPWM_ChannelTriggerInterruptEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelTriggerInterruptEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelTriggerInterruptEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelTriggerInterruptEnable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelTriggerInterruptEnable");
}


//******************************************************************************
/* Function :  MCPWM_ChannelTriggerInterruptDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ChannelTriggerInterruptDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ChannelTriggerInterruptDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelTriggerInterruptDisable_Unsupported( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_ChannelTriggerInterruptDisable");
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelTrigger_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsChannelTrigger

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsChannelTrigger function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsChannelTrigger_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_CHANNELTRIGGER_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

