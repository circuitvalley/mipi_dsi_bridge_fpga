/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_TxEPStatus_Unsupported.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : TxEPStatus
    and its Variant : Unsupported
    For following APIs :
        PLIB_USBHS_TxEPStatusGet
        PLIB_USBHS_TxEPStatusClear
        PLIB_USBHS_TxEPOUTTokenSend
        PLIB_USBHS_ExistsTxEPStatus

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

#ifndef _USBHS_TXEPSTATUS_UNSUPPORTED_H
#define _USBHS_TXEPSTATUS_UNSUPPORTED_H

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
/* Function :  USBHS_TxEPStatusGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_TxEPStatusGet 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_TxEPStatusGet function.
*/

PLIB_TEMPLATE uint8_t USBHS_TxEPStatusGet_Unsupported( USBHS_MODULE_ID index , uint8_t endpoint )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_TxEPStatusGet");

    return 0;
}


//******************************************************************************
/* Function :  USBHS_TxEPStatusClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_TxEPStatusClear 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_TxEPStatusClear function.
*/

PLIB_TEMPLATE void USBHS_TxEPStatusClear_Unsupported( USBHS_MODULE_ID index , uint8_t endpoint , USBHS_TXEP_ERROR error )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_TxEPStatusClear");
}


//******************************************************************************
/* Function :  USBHS_TxEPOUTTokenSend_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_TxEPOUTTokenSend 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_TxEPOUTTokenSend function.
*/

PLIB_TEMPLATE void USBHS_TxEPOUTTokenSend_Unsupported( USBHS_MODULE_ID index , uint8_t endpoint )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_TxEPOUTTokenSend");
}


//******************************************************************************
/* Function :  USBHS_ExistsTxEPStatus_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ExistsTxEPStatus

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ExistsTxEPStatus function.
*/

PLIB_TEMPLATE bool USBHS_ExistsTxEPStatus_Unsupported( USBHS_MODULE_ID index )
{
    return false;
}


#endif /*_USBHS_TXEPSTATUS_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

