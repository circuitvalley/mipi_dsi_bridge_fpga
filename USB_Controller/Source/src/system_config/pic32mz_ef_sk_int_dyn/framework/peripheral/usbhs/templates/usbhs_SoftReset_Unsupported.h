/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_SoftReset_Unsupported.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : SoftReset
    and its Variant : Unsupported
    For following APIs :
        PLIB_USBHS_SoftResetEnable
        PLIB_USBHS_SoftResetDisable
        PLIB_USBHS_ExistsSoftReset

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _USBHS_SOFTRESET_UNSUPPORTED_H
#define _USBHS_SOFTRESET_UNSUPPORTED_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    None.

  MASKs: 
    None.

  POSs: 
    None.

  LENs: 
    None.

*/


//******************************************************************************
/* Function :  USBHS_SoftResetEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_SoftResetEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_SoftResetEnable function.
*/

PLIB_TEMPLATE int USBHS_SoftResetEnable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_SoftResetEnable");

    return 0;
}


//******************************************************************************
/* Function :  USBHS_SoftResetDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_SoftResetDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_SoftResetDisable function.
*/

PLIB_TEMPLATE int USBHS_SoftResetDisable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_SoftResetDisable");

    return 0;
}


//******************************************************************************
/* Function :  USBHS_ExistsSoftReset_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ExistsSoftReset

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ExistsSoftReset function.
*/

PLIB_TEMPLATE bool USBHS_ExistsSoftReset_Unsupported( USBHS_MODULE_ID index )
{
    return false;
}


#endif /*_USBHS_SOFTRESET_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

