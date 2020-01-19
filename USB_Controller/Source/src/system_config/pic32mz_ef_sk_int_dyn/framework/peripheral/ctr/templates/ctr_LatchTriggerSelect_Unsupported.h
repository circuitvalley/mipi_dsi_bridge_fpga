/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_LatchTriggerSelect_Unsupported.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : LatchTriggerSelect
    and its Variant : Unsupported
    For following APIs :
        PLIB_CTR_ExistsLatchTriggerSelect
        PLIB_CTR_LatchTriggerSelect
        PLIB_CTR_LatchTriggerGet
        PLIB_CTR_LatchCTRSelect
        PLIB_CTR_LatchCTRGet
        PLIB_CTR_LatchDivSet
        PLIB_CTR_LatchDivGet

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

#ifndef _CTR_LATCHTRIGGERSELECT_UNSUPPORTED_H
#define _CTR_LATCHTRIGGERSELECT_UNSUPPORTED_H



//******************************************************************************
/* Function :  CTR_ExistsLatchTriggerSelect_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_ExistsLatchTriggerSelect

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_ExistsLatchTriggerSelect function.
*/

PLIB_TEMPLATE bool CTR_ExistsLatchTriggerSelect_Unsupported( CTR_MODULE_ID index )
{
    return false;
}


//******************************************************************************
/* Function :  CTR_LatchTriggerSelect_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_LatchTriggerSelect 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_LatchTriggerSelect function.
*/

PLIB_TEMPLATE void CTR_LatchTriggerSelect_Unsupported( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum , CTR_LATCH_TRIGGER_SELECT latTrigSrc )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_LatchTriggerSelect");
}


//******************************************************************************
/* Function :  CTR_LatchTriggerGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_LatchTriggerGet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_LatchTriggerGet function.
*/

PLIB_TEMPLATE CTR_LATCH_TRIGGER_SELECT CTR_LatchTriggerGet_Unsupported( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_LatchTriggerGet");

    return (CTR_LATCH_TRIGGER_SELECT) 0;
}


//******************************************************************************
/* Function :  CTR_LatchCTRSelect_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_LatchCTRSelect 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_LatchCTRSelect function.
*/

PLIB_TEMPLATE void CTR_LatchCTRSelect_Unsupported( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum , CTR_LATCH_CTR_SELECT latctrVal )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_LatchCTRSelect");
}


//******************************************************************************
/* Function :  CTR_LatchCTRGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_LatchCTRGet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_LatchCTRGet function.
*/

PLIB_TEMPLATE CTR_LATCH_CTR_SELECT CTR_LatchCTRGet_Unsupported( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_LatchCTRGet");

    return (CTR_LATCH_CTR_SELECT) 0;
}


//******************************************************************************
/* Function :  CTR_LatchDivSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_LatchDivSet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_LatchDivSet function.
*/

PLIB_TEMPLATE void CTR_LatchDivSet_Unsupported( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum , uint32_t divVal )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_LatchDivSet");
}


//******************************************************************************
/* Function :  CTR_LatchDivGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_LatchDivGet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_LatchDivGet function.
*/

PLIB_TEMPLATE uint32_t CTR_LatchDivGet_Unsupported( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_LatchDivGet");

    return (uint32_t) 0;
}


#endif /*_CTR_LATCHTRIGGERSELECT_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

