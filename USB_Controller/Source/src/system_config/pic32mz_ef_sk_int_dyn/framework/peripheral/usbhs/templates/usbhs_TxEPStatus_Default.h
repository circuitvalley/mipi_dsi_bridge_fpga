/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_TxEPStatus_Default.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : TxEPStatus
    and its Variant : Default
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

#ifndef _USBHS_TXEPSTATUS_DEFAULT_H
#define _USBHS_TXEPSTATUS_DEFAULT_H

#include "usbhs_registers.h"

//******************************************************************************
/* Function :  USBHS_TxEPStatusGet_Default

  Summary:
    Implements Default variant of PLIB_USBHS_TxEPStatusGet 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_TxEPStatusGet function.
*/

PLIB_TEMPLATE uint8_t USBHS_TxEPStatusGet_Default
( 
    USBHS_MODULE_ID index , 
    uint8_t endpoint 
)
{
    /* Returns the entire endpoint status register */
    
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    return(usbhs->EPCSR[endpoint].TXCSRL_DEVICEbits.w);
}

//******************************************************************************
/* Function :  USBHS_TxEPStatusClear_Default

  Summary:
    Implements Default variant of PLIB_USBHS_TxEPStatusClear 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_TxEPStatusClear function.
*/

PLIB_TEMPLATE void USBHS_TxEPStatusClear_Default
( 
    USBHS_MODULE_ID index , 
    uint8_t endpoint , 
    USBHS_TXEP_ERROR error 
)
{
    /* Clears the specified set of errors */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[endpoint].TXCSRL_DEVICEbits.w &= (~(error));
}

//******************************************************************************
/* Function :  USBHS_ExistsTxEPStatus_Default

  Summary:
    Implements Default variant of PLIB_USBHS_ExistsTxEPStatus

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_ExistsTxEPStatus function.
*/

#define PLIB_USBHS_ExistsTxEPStatus PLIB_USBHS_ExistsTxEPStatus
PLIB_TEMPLATE bool USBHS_ExistsTxEPStatus_Default( USBHS_MODULE_ID index )
{
    return true;
}


#endif /*_USBHS_TXEPSTATUS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

