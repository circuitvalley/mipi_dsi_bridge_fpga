/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTRMicroSecondsMode_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTRMicroSecondsMode
    and its Variant : Default
    For following APIs :
        PLIB_CTR_ExistsUSMode
        PLIB_CTR_USModeSecondGet
        PLIB_CTR_USModeSecondSet
        PLIB_CTR_USModeValueGet
        PLIB_CTR_USModeValueSet
        PLIB_CTR_USMode10nsGet
        PLIB_CTR_USMode10nsSet

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

#ifndef _CTR_CTRMICROSECONDSMODE_DEFAULT_H
#define _CTR_CTRMICROSECONDSMODE_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsUSMode_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsUSMode

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsUSMode function.
*/

#define PLIB_CTR_ExistsUSMode PLIB_CTR_ExistsUSMode
PLIB_TEMPLATE bool CTR_ExistsUSMode_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_USModeSecondGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_USModeSecondGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_USModeSecondGet function.
*/

PLIB_TEMPLATE uint32_t CTR_USModeSecondGet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	return (uint32_t) ((*reg & _CTR1_1394_US_SEC1_MASK) >> _CTR1_1394_US_SEC1_POSITION);
}


//******************************************************************************
/* Function :  CTR_USModeSecondSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_USModeSecondSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_USModeSecondSet function.
*/

PLIB_TEMPLATE void CTR_USModeSecondSet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t secUSVal )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	*reg = (*reg & ~_CTR1_1394_US_SEC1_MASK) | ((secUSVal & (_CTR1_1394_US_SEC1_MASK >> _CTR1_1394_US_SEC1_POSITION)) << _CTR1_1394_US_SEC1_POSITION);
}


//******************************************************************************
/* Function :  CTR_USModeValueGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_USModeValueGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_USModeValueGet function.
*/

PLIB_TEMPLATE uint32_t CTR_USModeValueGet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	return (uint32_t) ((*reg & _CTR1_1394_US_US_MASK) >> _CTR1_1394_US_US_POSITION);
}


//******************************************************************************
/* Function :  CTR_USModeValueSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_USModeValueSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_USModeValueSet function.
*/

PLIB_TEMPLATE void CTR_USModeValueSet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t usVal )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	*reg = (*reg & ~_CTR1_1394_US_US_MASK) | ((usVal & (_CTR0_1394_US_SEC_MASK >> _CTR0_1394_US_SEC_POSITION)) << _CTR0_1394_US_SEC_POSITION);
}


//******************************************************************************
/* Function :  CTR_USMode10nsGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_USMode10nsGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_USMode10nsGet function.
*/

PLIB_TEMPLATE uint32_t CTR_USMode10nsGet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	return (uint32_t) ((*reg & _CTR1_1394_US_NS10_MASK) >> _CTR1_1394_US_NS10_POSITION);
}


//******************************************************************************
/* Function :  CTR_USMode10nsSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_USMode10nsSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_USMode10nsSet function.
*/

PLIB_TEMPLATE void CTR_USMode10nsSet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t us10nsVal )
{
	volatile uint32_t *reg = &CTR0_1394_US + (ctrSel * 10);
	*reg = (*reg & ~_CTR1_1394_US_NS10_MASK) | ((us10nsVal & (_CTR1_1394_US_NS10_MASK >> _CTR1_1394_US_NS10_POSITION)) << _CTR1_1394_US_NS10_POSITION);
}


#endif /*_CTR_CTRMICROSECONDSMODE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

