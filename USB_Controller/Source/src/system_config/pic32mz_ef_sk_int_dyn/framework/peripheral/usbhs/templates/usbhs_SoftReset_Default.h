/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_SoftReset_Default.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : SoftReset
    and its Variant : Default
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

#ifndef _USBHS_SOFTRESET_DEFAULT_H
#define _USBHS_SOFTRESET_DEFAULT_H

#include "usbhs_registers.h"

//******************************************************************************
/* Function :  USBHS_SoftResetEnable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_SoftResetEnable 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_SoftResetEnable function.
*/

PLIB_TEMPLATE void USBHS_SoftResetEnable_Default( USBHS_MODULE_ID index )
{
    /* This function enables the soft reset bits */

    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->SOFTRSTbits.w = USBHS_SOFT_RST_NRST_SET|USBHS_SOFT_RST_NRSTX_SET;
}

//******************************************************************************
/* Function :  USBHS_SoftResetDisable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_SoftResetDisable 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_SoftResetDisable function.
*/

PLIB_TEMPLATE void USBHS_SoftResetDisable_Default( USBHS_MODULE_ID index )
{   
    /* This function will clear the NRST and NRSTX bits. */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->SOFTRSTbits.w = 0;
}

//******************************************************************************
/* Function :  USBHS_SoftResetNRSTXEnable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_SoftResetNRSTXEnable 

  Description:
    This template implements the Default variant of the
    PLIB_USBHS_SoftResetNRSTXEnable function.
*/

PLIB_TEMPLATE void USBHS_SoftResetNRSTXEnable_Default( USBHS_MODULE_ID index )
{   
    /* This function sets the NRSTX bits. It is cleared automatically by
     * hardware. */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->SOFTRSTbits.NRSTX = 1;
}


//******************************************************************************
/* Function :  USBHS_ExistsSoftReset_Default

  Summary:
    Implements Default variant of PLIB_USBHS_ExistsSoftReset

  Description:
    This template implements the Default variant of the PLIB_USBHS_ExistsSoftReset function.
*/

#define PLIB_USBHS_ExistsSoftReset PLIB_USBHS_ExistsSoftReset
PLIB_TEMPLATE bool USBHS_ExistsSoftReset_Default( USBHS_MODULE_ID index )
{
    return true;
}


#endif /*_USBHS_SOFTRESET_DEFAULT_H*/

/******************************************************************************
 End of File
*/

