/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTRTrigger_Unsupported.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTRTrigger
    and its Variant : Unsupported
    For following APIs :
        PLIB_CTR_ExistsTrigger
        PLIB_CTR_TriggerSelect
        PLIB_CTR_TriggerGet
        PLIB_CTR_CycleOffsetValueSet
        PLIB_CTR_CycleOffsetValueGet

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

#ifndef _CTR_CTRTRIGGER_UNSUPPORTED_H
#define _CTR_CTRTRIGGER_UNSUPPORTED_H


//******************************************************************************
/* Function :  CTR_ExistsTrigger_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_ExistsTrigger

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_ExistsTrigger function.
*/

PLIB_TEMPLATE bool CTR_ExistsTrigger_Unsupported( CTR_MODULE_ID index )
{
    return false;
}


//******************************************************************************
/* Function :  CTR_TriggerSelect_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_TriggerSelect 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_TriggerSelect function.
*/

PLIB_TEMPLATE void CTR_TriggerSelect_Unsupported( CTR_MODULE_ID index , CTR_LATCH_CTR_SELECT ctrTrigVal )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_TriggerSelect");
}


//******************************************************************************
/* Function :  CTR_TriggerGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_TriggerGet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_TriggerGet function.
*/

PLIB_TEMPLATE CTR_LATCH_CTR_SELECT CTR_TriggerGet_Unsupported( CTR_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_TriggerGet");

    return (CTR_LATCH_CTR_SELECT) 0;
}


//******************************************************************************
/* Function :  CTR_CycleOffsetValueSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_CycleOffsetValueSet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_CycleOffsetValueSet function.
*/

PLIB_TEMPLATE void CTR_CycleOffsetValueSet_Unsupported( CTR_MODULE_ID index , uint32_t cycleOffsetVal )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_CycleOffsetValueSet");
}


//******************************************************************************
/* Function :  CTR_CycleOffsetValueGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_CycleOffsetValueGet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_CycleOffsetValueGet function.
*/

PLIB_TEMPLATE uint32_t CTR_CycleOffsetValueGet_Unsupported( CTR_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_CycleOffsetValueGet");

    return (uint32_t) 0;
}


#endif /*_CTR_CTRTRIGGER_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

