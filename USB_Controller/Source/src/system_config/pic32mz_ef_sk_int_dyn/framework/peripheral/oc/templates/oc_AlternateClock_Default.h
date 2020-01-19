/*******************************************************************************
  OC Peripheral Library Template Implementation

  File Name:
    oc_AlternateClock_Default.h

  Summary:
    OC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : AlternateClock
    and its Variant : Default
    For following APIs :
        PLIB_OC_AlternateClockEnable
        PLIB_OC_AlternateClockDisable
        PLIB_OC_ExistsAlternateClock

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

#ifndef _OC_ALTERNATECLOCK_DEFAULT_H
#define _OC_ALTERNATECLOCK_DEFAULT_H

//******************************************************************************
/* Devcon SFR is directly hacked
*/


//******************************************************************************
/* Function :  OC_AlternateClockEnable_Default

  Summary:
    Implements Default variant of PLIB_OC_AlternateClockEnable 

  Description:
    This template implements the Default variant of the PLIB_OC_AlternateClockEnable function.
*/

PLIB_TEMPLATE void OC_AlternateClockEnable_Default( OC_MODULE_ID index )
{
    CFGCONbits.OCACLK = 1;
}


//******************************************************************************
/* Function :  OC_AlternateClockDisable_Default

  Summary:
    Implements Default variant of PLIB_OC_AlternateClockDisable 

  Description:
    This template implements the Default variant of the PLIB_OC_AlternateClockDisable function.
*/

PLIB_TEMPLATE void OC_AlternateClockDisable_Default( OC_MODULE_ID index )
{
    CFGCONbits.OCACLK = 0;
}


//******************************************************************************
/* Function :  OC_ExistsAlternateClock_Default

  Summary:
    Implements Default variant of PLIB_OC_ExistsAlternateClock

  Description:
    This template implements the Default variant of the PLIB_OC_ExistsAlternateClock function.
*/

#define PLIB_OC_ExistsAlternateClock PLIB_OC_ExistsAlternateClock
PLIB_TEMPLATE bool OC_ExistsAlternateClock_Default( OC_MODULE_ID index )
{
    return true;
}


#endif /*_OC_ALTERNATECLOCK_DEFAULT_H*/

/******************************************************************************
 End of File
*/

