/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_QueuePointer_Default.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : QueuePointer
    and its Variant : Default
    For following APIs :
        PLIB_PTG_QueuePointerSet
        PLIB_PTG_QueuePointerGet
        PLIB_PTG_ExistsQueuePointer

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

#ifndef _PTG_QUEUEPOINTER_DEFAULT_H
#define _PTG_QUEUEPOINTER_DEFAULT_H

//******************************************************************************
/* Function :  PTG_QueuePointerSet_Default

  Summary:
    Implements Default variant of PLIB_PTG_QueuePointerSet 

  Description:
    This template implements the Default variant of the PLIB_PTG_QueuePointerSet function.
*/

PLIB_TEMPLATE void PTG_QueuePointerSet_Default( PTG_MODULE_ID index , uint8_t queueLoc )
{
	PTGQPTR = queueLoc & 0x1F;
}


//******************************************************************************
/* Function :  PTG_QueuePointerGet_Default

  Summary:
    Implements Default variant of PLIB_PTG_QueuePointerGet 

  Description:
    This template implements the Default variant of the PLIB_PTG_QueuePointerGet function.
*/

PLIB_TEMPLATE uint8_t PTG_QueuePointerGet_Default( PTG_MODULE_ID index )
{
	return (uint8_t) PTGQPTR;
}


//******************************************************************************
/* Function :  PTG_ExistsQueuePointer_Default

  Summary:
    Implements Default variant of PLIB_PTG_ExistsQueuePointer

  Description:
    This template implements the Default variant of the PLIB_PTG_ExistsQueuePointer function.
*/

#define PLIB_PTG_ExistsQueuePointer PLIB_PTG_ExistsQueuePointer
PLIB_TEMPLATE bool PTG_ExistsQueuePointer_Default( PTG_MODULE_ID index )
{
    return true;
}


#endif /*_PTG_QUEUEPOINTER_DEFAULT_H*/

/******************************************************************************
 End of File
*/

