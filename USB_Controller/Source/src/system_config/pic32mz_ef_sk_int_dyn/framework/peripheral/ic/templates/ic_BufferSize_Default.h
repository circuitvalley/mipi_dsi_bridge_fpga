/*******************************************************************************
  IC Peripheral Library Template Implementation

  File Name:
    ic_BufferSize_Default.h

  Summary:
    IC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : BufferSize
    and its Variant : Default
    For following APIs :
        PLIB_IC_BufferSizeSelect
        PLIB_IC_ExistsBufferSize

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

#ifndef _IC_BUFFERSIZE_DEFAULT_H
#define _IC_BUFFERSIZE_DEFAULT_H

#include "ic_Registers.h"

//******************************************************************************
/* Function :  IC_BufferSizeSelect_Default

  Summary:
    Implements Default variant of PLIB_IC_BufferSizeSelect 

  Description:
    This template implements the Default variant of the PLIB_IC_BufferSizeSelect function.
*/

PLIB_TEMPLATE void IC_BufferSizeSelect_Default( IC_MODULE_ID index , IC_BUFFER_SIZE bufSize )
{
    volatile ic_register_t *regs = (ic_register_t *)index;

    regs->ICxCON.C32 = bufSize;
}


//******************************************************************************
/* Function :  IC_ExistsBufferSize_Default

  Summary:
    Implements Default variant of PLIB_IC_ExistsBufferSize

  Description:
    This template implements the Default variant of the PLIB_IC_ExistsBufferSize function.
*/

#define PLIB_IC_ExistsBufferSize PLIB_IC_ExistsBufferSize
PLIB_TEMPLATE bool IC_ExistsBufferSize_Default( IC_MODULE_ID index )
{
    return true;
}


#endif /*_IC_BUFFERSIZE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

