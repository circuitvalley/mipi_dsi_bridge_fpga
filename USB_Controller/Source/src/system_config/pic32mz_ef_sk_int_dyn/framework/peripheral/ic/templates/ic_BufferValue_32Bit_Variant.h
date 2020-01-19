/*******************************************************************************
  IC Peripheral Library Template Implementation

  File Name:
    ic_BufferValue_32Bit_Variant.h

  Summary:
    IC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : BufferValue
    and its Variant : 32Bit_Variant
    For following APIs :
        PLIB_IC_Buffer32BitGet
        PLIB_IC_Buffer16BitGet
        PLIB_IC_ExistsBufferValue

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _IC_BUFFERVALUE_32BIT_VARIANT_H
#define _IC_BUFFERVALUE_32BIT_VARIANT_H

#include "ic_Registers.h"

//******************************************************************************
/* Function :  IC_Buffer32BitGet_32Bit_Variant

  Summary:
    Implements 32Bit_Variant variant of PLIB_IC_Buffer32BitGet 

  Description:
    This template implements the 32Bit_Variant variant of the PLIB_IC_Buffer32BitGet function.
*/

PLIB_TEMPLATE uint32_t IC_Buffer32BitGet_32Bit_Variant( IC_MODULE_ID index )
{
    volatile ic_register_t *regs = (ic_register_t *)index;

    return regs->ICxBUF;
}


//******************************************************************************
/* Function :  IC_Buffer16BitGet_32Bit_Variant

  Summary:
    Implements 32Bit_Variant variant of PLIB_IC_Buffer16BitGet 

  Description:
    This template implements the 32Bit_Variant variant of the PLIB_IC_Buffer16BitGet function.
*/

PLIB_TEMPLATE uint16_t IC_Buffer16BitGet_32Bit_Variant( IC_MODULE_ID index )
{
    volatile ic_register_t *regs = (ic_register_t *)index;

    return (uint16_t) regs->ICxBUF;
}


//******************************************************************************
/* Function :  IC_ExistsBufferValue_32Bit_Variant

  Summary:
    Implements 32Bit_Variant variant of PLIB_IC_ExistsBufferValue

  Description:
    This template implements the 32Bit_Variant variant of the PLIB_IC_ExistsBufferValue function.
*/

#define PLIB_IC_ExistsBufferValue PLIB_IC_ExistsBufferValue
PLIB_TEMPLATE bool IC_ExistsBufferValue_32Bit_Variant( IC_MODULE_ID index )
{
    return true;
}


#endif /*_IC_BUFFERVALUE_32BIT_VARIANT_H*/

/******************************************************************************
 End of File
*/

