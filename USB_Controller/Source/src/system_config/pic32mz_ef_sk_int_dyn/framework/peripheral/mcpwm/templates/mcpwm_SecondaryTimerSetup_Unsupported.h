/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_SecondaryTimerSetup_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : SecondaryTimerSetup
    and its Variant : Unsupported
    For following APIs :
        PLIB_MCPWM_SecondaryTimerSetup
        PLIB_MCPWM_SecondaryTimerCountRead
        PLIB_MCPWM_ExistsSecondaryTimerSetup

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

#ifndef _MCPWM_SECONDARYTIMERSETUP_UNSUPPORTED_H
#define _MCPWM_SECONDARYTIMERSETUP_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_SecondaryTimerSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_SecondaryTimerSetup 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_SecondaryTimerSetup function.
*/

PLIB_TEMPLATE void MCPWM_SecondaryTimerSetup_Unsupported( MCPWM_MODULE_ID index , MCPWM_CLOCK_DIVIDER clock_div , uint16_t period_value )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_SecondaryTimerSetup");
}


//******************************************************************************
/* Function :  MCPWM_SecondaryTimerCountRead_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_SecondaryTimerCountRead 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_SecondaryTimerCountRead function.
*/

PLIB_TEMPLATE void MCPWM_SecondaryTimerCountRead_Unsupported( MCPWM_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_SecondaryTimerCountRead");
}


//******************************************************************************
/* Function :  MCPWM_ExistsSecondaryTimerSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsSecondaryTimerSetup

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsSecondaryTimerSetup function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsSecondaryTimerSetup_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_SECONDARYTIMERSETUP_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

