/*******************************************************************************
  EBI Peripheral Library Template Implementation

  File Name:
    ebi_ReadyMode_Default.h

  Summary:
    EBI PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ReadyMode
    and its Variant : Default
    For following APIs :
        PLIB_EBI_ReadyModeSet
        PLIB_EBI_ReadyModeGet
        PLIB_EBI_ExistsReadyMode

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

#ifndef _EBI_READYMODE_DEFAULT_H
#define _EBI_READYMODE_DEFAULT_H

#include "ebi_Registers.h"

//******************************************************************************
/* Function :  EBI_ReadyModeSet_Default

  Summary:
    Implements Default variant of PLIB_EBI_ReadyModeSet 

  Description:
    This template implements the Default variant of the PLIB_EBI_ReadyModeSet function.
*/
PLIB_TEMPLATE void EBI_ReadyModeSet_Default( EBI_MODULE_ID index , bool ReadyPin0 , bool ReadyPin1 , bool ReadyPin2 )
{
    volatile ebi_register_t *ebi = (volatile ebi_register_t *)index;

    ebi->EBISMTx[0].RDYMODE = ReadyPin0;
    ebi->EBISMTx[1].RDYMODE = ReadyPin1;
    ebi->EBISMTx[2].RDYMODE = ReadyPin2;
}

//******************************************************************************
/* Function :  EBI_ReadyModeGet_Default

  Summary:
    Implements Default variant of PLIB_EBI_ReadyModeGet 

  Description:
    This template implements the Default variant of the PLIB_EBI_ReadyModeGet function.
*/

PLIB_TEMPLATE bool EBI_ReadyModeGet_Default( EBI_MODULE_ID index , int ChipSelectNumber )
{
    volatile ebi_register_t *ebi = (volatile ebi_register_t *)index;

    return (bool)ebi->EBISMTx[ChipSelectNumber].RDYMODE;
}

//******************************************************************************
/* Function :  EBI_ExistsReadyMode_Default

  Summary:
    Implements Default variant of PLIB_EBI_ExistsReadyMode

  Description:
    This template implements the Default variant of the PLIB_EBI_ExistsReadyMode function.
*/
#define PLIB_EBI_ExistsReadyMode PLIB_EBI_ExistsReadyMode
PLIB_TEMPLATE bool EBI_ExistsReadyMode_Default( EBI_MODULE_ID index )
{
    return true;
}

#endif /*_EBI_READYMODE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

