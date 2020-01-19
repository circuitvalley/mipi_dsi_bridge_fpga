/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_EP0Status_Unsupported.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : EP0Status
    and its Variant : Unsupported
    For following APIs :
        PLIB_USBHS_EP0StatusGet
        PLIB_USBHS_EP0StatusClear
        PLIB_USBHS_EP0INHandshakeSend
        PLIB_USBHS_EP0INTokenSend
        PLIB_USBHS_EP0OUTHandshakeSend
        PLIB_USBHS_EP0INHandshakeClear
        PLIB_USBHS_ExistsEP0Status

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

#ifndef _USBHS_EP0STATUS_UNSUPPORTED_H
#define _USBHS_EP0STATUS_UNSUPPORTED_H

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
/* Function :  USBHS_EP0StatusGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_EP0StatusGet 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_EP0StatusGet function.
*/

PLIB_TEMPLATE uint8_t USBHS_EP0StatusGet_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_EP0StatusGet");

    return 0;
}


//******************************************************************************
/* Function :  USBHS_EP0StatusClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_EP0StatusClear 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_EP0StatusClear function.
*/

PLIB_TEMPLATE void USBHS_EP0StatusClear_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_EP0StatusClear");
}


//******************************************************************************
/* Function :  USBHS_EP0INHandshakeSend_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_EP0INHandshakeSend 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_EP0INHandshakeSend function.
*/

PLIB_TEMPLATE void USBHS_EP0INHandshakeSend_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_EP0INHandshakeSend");
}


//******************************************************************************
/* Function :  USBHS_EP0INTokenSend_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_EP0INTokenSend 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_EP0INTokenSend function.
*/

PLIB_TEMPLATE void USBHS_EP0INTokenSend_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_EP0INTokenSend");
}


//******************************************************************************
/* Function :  USBHS_EP0OUTHandshakeSend_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_EP0OUTHandshakeSend 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_EP0OUTHandshakeSend function.
*/

PLIB_TEMPLATE void USBHS_EP0OUTHandshakeSend_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_EP0OUTHandshakeSend");
}


//******************************************************************************
/* Function :  USBHS_EP0INHandshakeClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_EP0INHandshakeClear 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_EP0INHandshakeClear function.
*/

PLIB_TEMPLATE void USBHS_EP0INHandshakeClear_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_EP0INHandshakeClear");
}


//******************************************************************************
/* Function :  USBHS_ExistsEP0Status_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ExistsEP0Status

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ExistsEP0Status function.
*/

PLIB_TEMPLATE bool USBHS_ExistsEP0Status_Unsupported( USBHS_MODULE_ID index )
{
    return false;
}


#endif /*_USBHS_EP0STATUS_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

