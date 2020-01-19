/*******************************************************************************
  EBI Peripheral Library Template Implementation

  File Name:
    ebi_ControlEnable_Default.h

  Summary:
    EBI PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ControlEnable
    and its Variant : Default
    For following APIs :
        PLIB_EBI_ControlEnableSet
        PLIB_EBI_ControlEnableGet
        PLIB_EBI_ExistsControlEnable

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

#ifndef _EBI_CONTROLENABLE_DEFAULT_H
#define _EBI_CONTROLENABLE_DEFAULT_H

#include "ebi_Registers.h"

//******************************************************************************
/* Function :  EBI_ControlEnableSet_Default

  Summary:
    Implements Default variant of PLIB_EBI_ControlEnableSet 

  Description:
    This template implements the Default variant of the PLIB_EBI_ControlEnableSet function.
*/
PLIB_TEMPLATE void EBI_ControlEnableSet_Default( EBI_MODULE_ID index , bool EnableBit )
{
    volatile cfgebi_register_t *ebi = (volatile cfgebi_register_t *)&CFGEBIA;

    ebi->CFGEBIA.EBIPINEN = EnableBit;
}

//******************************************************************************
/* Function :  EBI_ControlEnableGet_Default

  Summary:
    Implements Default variant of PLIB_EBI_ControlEnableGet 

  Description:
    This template implements the Default variant of the PLIB_EBI_ControlEnableGet function.
*/
PLIB_TEMPLATE bool EBI_ControlEnableGet_Default( EBI_MODULE_ID index )
{
    volatile cfgebi_register_t *ebi = (volatile cfgebi_register_t *)&CFGEBIA;

    return (bool)ebi->CFGEBIA.EBIPINEN;
}

//******************************************************************************
/* Function :  EBI_ExistsControlEnable_Default

  Summary:
    Implements Default variant of PLIB_EBI_ExistsControlEnable

  Description:
    This template implements the Default variant of the PLIB_EBI_ExistsControlEnable function.
*/
#define PLIB_EBI_ExistsControlEnable PLIB_EBI_ExistsControlEnable
PLIB_TEMPLATE bool EBI_ExistsControlEnable_Default( EBI_MODULE_ID index )
{
    return true;
}

#endif /*_EBI_CONTROLENABLE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

