/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_AdjustValue_Unsupported.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : AdjustValue
    and its Variant : Unsupported
    For following APIs :
        PLIB_PTG_AdjustValueSet
        PLIB_PTG_AdjustValueGet
        PLIB_PTG_ExistsAdjustValue

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

#ifndef _PTG_ADJUSTVALUE_UNSUPPORTED_H
#define _PTG_ADJUSTVALUE_UNSUPPORTED_H

//******************************************************************************
/* Function :  PTG_AdjustValueSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_AdjustValueSet 

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_AdjustValueSet function.
*/

PLIB_TEMPLATE void PTG_AdjustValueSet_Unsupported( PTG_MODULE_ID index , uint16_t adjustValue )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_PTG_AdjustValueSet");
}


//******************************************************************************
/* Function :  PTG_AdjustValueGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_AdjustValueGet 

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_AdjustValueGet function.
*/

PLIB_TEMPLATE uint16_t PTG_AdjustValueGet_Unsupported( PTG_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_PTG_AdjustValueGet");

    return (uint16_t) 0;
}


//******************************************************************************
/* Function :  PTG_ExistsAdjustValue_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_ExistsAdjustValue

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_ExistsAdjustValue function.
*/

PLIB_TEMPLATE bool PTG_ExistsAdjustValue_Unsupported( PTG_MODULE_ID index )
{
    return false;
}


#endif /*_PTG_ADJUSTVALUE_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

