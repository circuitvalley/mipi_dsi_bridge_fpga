/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_ClockResetControl_Unsupported.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ClockResetControl
    and its Variant : Unsupported
    For following APIs :
        PLIB_USBHS_ExistsClockResetControl
        PLIB_USBHS_GlobalInterruptEnable
        PLIB_USBHS_GlobalInterruptDisable

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

#ifndef _USBHS_CLOCKRESETCONTROL_UNSUPPORTED_H
#define _USBHS_CLOCKRESETCONTROL_UNSUPPORTED_H

//******************************************************************************
/* Function :  USBHS_ExistsClockResetControl_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ExistsClockResetControl

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ExistsClockResetControl function.
*/

PLIB_TEMPLATE bool USBHS_ExistsClockResetControl_Unsupported( USBHS_MODULE_ID index )
{
    return false;
}


//******************************************************************************
/* Function :  USBHS_GlobalInterruptEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_GlobalInterruptEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_GlobalInterruptEnable function.
*/

PLIB_TEMPLATE void USBHS_GlobalInterruptEnable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_GlobalInterruptEnable");
}


//******************************************************************************
/* Function :  USBHS_GlobalInterruptDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_GlobalInterruptDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_GlobalInterruptDisable function.
*/

PLIB_TEMPLATE void USBHS_GlobalInterruptDisable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_GlobalInterruptDisable");
}


#endif /*_USBHS_CLOCKRESETCONTROL_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

