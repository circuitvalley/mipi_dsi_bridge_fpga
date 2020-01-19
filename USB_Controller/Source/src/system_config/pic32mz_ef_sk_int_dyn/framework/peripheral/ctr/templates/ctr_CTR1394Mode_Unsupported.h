/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTR1394Mode_Unsupported.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTR1394Mode
    and its Variant : Unsupported
    For following APIs :
        PLIB_CTR_Exists1394Mode
        PLIB_CTR_1394ModeSecondGet
        PLIB_CTR_1394ModeSecondSet
        PLIB_CTR_1394ModeCountGet
        PLIB_CTR_1394ModeCountSet
        PLIB_CTR_1394ModeOffsetGet
        PLIB_CTR_1394ModeOffsetSet

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

#ifndef _CTR_CTR1394MODE_UNSUPPORTED_H
#define _CTR_CTR1394MODE_UNSUPPORTED_H

//******************************************************************************
/* Function :  CTR_Exists1394Mode_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_Exists1394Mode

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_Exists1394Mode function.
*/

PLIB_TEMPLATE bool CTR_Exists1394Mode_Unsupported( CTR_MODULE_ID index )
{
    return false;
}


//******************************************************************************
/* Function :  CTR_1394ModeSecondGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_1394ModeSecondGet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_1394ModeSecondGet function.
*/

PLIB_TEMPLATE uint32_t CTR_1394ModeSecondGet_Unsupported( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_1394ModeSecondGet");

    return (uint32_t) 0;
}


//******************************************************************************
/* Function :  CTR_1394ModeSecondSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_1394ModeSecondSet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_1394ModeSecondSet function.
*/

PLIB_TEMPLATE void CTR_1394ModeSecondSet_Unsupported( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t secVal )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_1394ModeSecondSet");
}


//******************************************************************************
/* Function :  CTR_1394ModeCountGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_1394ModeCountGet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_1394ModeCountGet function.
*/

PLIB_TEMPLATE uint32_t CTR_1394ModeCountGet_Unsupported( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_1394ModeCountGet");

    return (uint32_t) 0;
}


//******************************************************************************
/* Function :  CTR_1394ModeCountSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_1394ModeCountSet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_1394ModeCountSet function.
*/

PLIB_TEMPLATE void CTR_1394ModeCountSet_Unsupported( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t countVal )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_1394ModeCountSet");
}


//******************************************************************************
/* Function :  CTR_1394ModeOffsetGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_1394ModeOffsetGet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_1394ModeOffsetGet function.
*/

PLIB_TEMPLATE uint32_t CTR_1394ModeOffsetGet_Unsupported( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_1394ModeOffsetGet");

    return (uint32_t) 0;
}


//******************************************************************************
/* Function :  CTR_1394ModeOffsetSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_CTR_1394ModeOffsetSet 

  Description:
    This template implements the Unsupported variant of the PLIB_CTR_1394ModeOffsetSet function.
*/

PLIB_TEMPLATE void CTR_1394ModeOffsetSet_Unsupported( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t offsetVal )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_CTR_1394ModeOffsetSet");
}


#endif /*_CTR_CTR1394MODE_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

