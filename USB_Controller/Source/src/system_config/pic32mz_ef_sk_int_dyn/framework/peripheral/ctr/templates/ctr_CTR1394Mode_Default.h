/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTR1394Mode_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTR1394Mode
    and its Variant : Default
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

#ifndef _CTR_CTR1394MODE_DEFAULT_H
#define _CTR_CTR1394MODE_DEFAULT_H

//******************************************************************************

//******************************************************************************
/* Function :  CTR_Exists1394Mode_Default

  Summary:
    Implements Default variant of PLIB_CTR_Exists1394Mode

  Description:
    This template implements the Default variant of the PLIB_CTR_Exists1394Mode function.
*/

#define PLIB_CTR_Exists1394Mode PLIB_CTR_Exists1394Mode
PLIB_TEMPLATE bool CTR_Exists1394Mode_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_1394ModeSecondGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_1394ModeSecondGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_1394ModeSecondGet function.
*/

PLIB_TEMPLATE uint32_t CTR_1394ModeSecondGet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	return (uint32_t) ((*reg & _CTR0_1394_US_SEC_MASK) >> _CTR0_1394_US_SEC_POSITION);
}


//******************************************************************************
/* Function :  CTR_1394ModeSecondSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_1394ModeSecondSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_1394ModeSecondSet function.
*/

PLIB_TEMPLATE void CTR_1394ModeSecondSet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t secVal )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	*reg = (*reg & ~_CTR0_1394_US_SEC_MASK) | ((secVal & (_CTR0_1394_US_SEC_MASK >> _CTR0_1394_US_SEC_POSITION)) << _CTR0_1394_US_SEC_POSITION);
}


//******************************************************************************
/* Function :  CTR_1394ModeCountGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_1394ModeCountGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_1394ModeCountGet function.
*/

PLIB_TEMPLATE uint32_t CTR_1394ModeCountGet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	return (uint32_t) ((*reg & _CTR0_1394_US_COUNT_MASK) >> _CTR0_1394_US_COUNT_POSITION);
}


//******************************************************************************
/* Function :  CTR_1394ModeCountSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_1394ModeCountSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_1394ModeCountSet function.
*/

PLIB_TEMPLATE void CTR_1394ModeCountSet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t countVal )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	*reg = (*reg & ~_CTR0_1394_US_COUNT_MASK) | ((countVal & (_CTR0_1394_US_COUNT_MASK >> _CTR0_1394_US_COUNT_POSITION)) << _CTR0_1394_US_COUNT_POSITION);
}


//******************************************************************************
/* Function :  CTR_1394ModeOffsetGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_1394ModeOffsetGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_1394ModeOffsetGet function.
*/

PLIB_TEMPLATE uint32_t CTR_1394ModeOffsetGet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	return (uint32_t) ((*reg & _CTR0_1394_US_OFFSET_MASK) >> _CTR0_1394_US_OFFSET_POSITION);
}


//******************************************************************************
/* Function :  CTR_1394ModeOffsetSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_1394ModeOffsetSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_1394ModeOffsetSet function.
*/

PLIB_TEMPLATE void CTR_1394ModeOffsetSet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t offsetVal )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	*reg = (*reg & ~_CTR0_1394_US_OFFSET_MASK) | ((offsetVal & (_CTR0_1394_US_OFFSET_MASK >> _CTR0_1394_US_OFFSET_POSITION)) << _CTR0_1394_US_OFFSET_POSITION);
}


#endif /*_CTR_CTR1394MODE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

