/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTRRevision_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTRRevision
    and its Variant : Default
    For following APIs :
        PLIB_CTR_ExistsRevision
        PLIB_CTR_BlockRevisionGet

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

#ifndef _CTR_CTRREVISION_DEFAULT_H
#define _CTR_CTRREVISION_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsRevision_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsRevision

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsRevision function.
*/

#define PLIB_CTR_ExistsRevision PLIB_CTR_ExistsRevision
PLIB_TEMPLATE bool CTR_ExistsRevision_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_BlockRevisionGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_BlockRevisionGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_BlockRevisionGet function.
*/

PLIB_TEMPLATE uint32_t CTR_BlockRevisionGet_Default( CTR_MODULE_ID index )
{
    return CTR_REVISION;
}


#endif /*_CTR_CTRREVISION_DEFAULT_H*/

/******************************************************************************
 End of File
*/

