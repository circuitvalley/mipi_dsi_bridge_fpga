/*******************************************************************************
  DMT Peripheral Library Template Implementation

  File Name:
    dmt_Step2_Default.h

  Summary:
    DMT PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : Step2
    and its Variant : Default
    For following APIs :
        PLIB_DMT_ClearStep2
        PLIB_DMT_ExistsStep2

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

#ifndef _DMT_STEP2_DEFAULT_H
#define _DMT_STEP2_DEFAULT_H

//******************************************************************************
/* Function :  DMT_ClearStep2_Default

  Summary:
    Implements Default variant of PLIB_DMT_ClearStep2 

  Description:
    This template implements the Default variant of the PLIB_DMT_ClearStep2 function.
*/

PLIB_TEMPLATE void DMT_ClearStep2_Default( DMT_MODULE_ID index )
{
    DMTCLRbits.STEP2 = 0x08;
}


//******************************************************************************
/* Function :  DMT_ExistsStep2_Default

  Summary:
    Implements Default variant of PLIB_DMT_ExistsStep2

  Description:
    This template implements the Default variant of the PLIB_DMT_ExistsStep2 function.
*/

#define PLIB_DMT_ExistsStep2 PLIB_DMT_ExistsStep2
PLIB_TEMPLATE bool DMT_ExistsStep2_Default( DMT_MODULE_ID index )
{
    return true;
}


#endif /*_DMT_STEP2_DEFAULT_H*/

/******************************************************************************
 End of File
*/

