/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_LatchTriggerSelect_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : LatchTriggerSelect
    and its Variant : Default
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

#ifndef _CTR_LATCHTRIGGERSELECT_DEFAULT_H
#define _CTR_LATCHTRIGGERSELECT_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsLatchTriggerSelect_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsLatchTriggerSelect

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsLatchTriggerSelect function.
*/

#define PLIB_CTR_ExistsLatchTriggerSelect PLIB_CTR_ExistsLatchTriggerSelect
PLIB_TEMPLATE bool CTR_ExistsLatchTriggerSelect_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_LatchTriggerSelect_Default

  Summary:
    Implements Default variant of PLIB_CTR_LatchTriggerSelect 

  Description:
    This template implements the Default variant of the PLIB_CTR_LatchTriggerSelect function.
*/

PLIB_TEMPLATE void CTR_LatchTriggerSelect_Default( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum , CTR_LATCH_TRIGGER_SELECT latTrigSrc )
{
	volatile uint32_t *reg = &LATCH_UNIT1_TRIG_SEL + (latNum * 4);
	((__LATCH_UNIT1_TRIG_SELbits_t *)reg)->TRIG_SEL = latTrigSrc;
}


//******************************************************************************
/* Function :  CTR_LatchTriggerGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_LatchTriggerGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_LatchTriggerGet function.
*/

PLIB_TEMPLATE CTR_LATCH_TRIGGER_SELECT CTR_LatchTriggerGet_Default( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum )
{
	volatile uint32_t *reg = &LATCH_UNIT1_TRIG_SEL + (latNum * 4);	
    return (CTR_LATCH_TRIGGER_SELECT) (((__LATCH_UNIT1_TRIG_SELbits_t *)reg)->TRIG_SEL);
}


//******************************************************************************
/* Function :  CTR_LatchCTRSelect_Default

  Summary:
    Implements Default variant of PLIB_CTR_LatchCTRSelect 

  Description:
    This template implements the Default variant of the PLIB_CTR_LatchCTRSelect function.
*/

PLIB_TEMPLATE void CTR_LatchCTRSelect_Default( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum , CTR_LATCH_CTR_SELECT latctrVal )
{
	volatile uint32_t *reg = &LATCH_UNIT1_TRIG_SEL + (latNum * 4);	
	((__LATCH_UNIT1_TRIG_SELbits_t *)reg)->CTR_SEL = latctrVal;
}


//******************************************************************************
/* Function :  CTR_LatchCTRGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_LatchCTRGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_LatchCTRGet function.
*/

PLIB_TEMPLATE CTR_LATCH_CTR_SELECT CTR_LatchCTRGet_Default( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum )
{
	volatile uint32_t *reg = &LATCH_UNIT1_TRIG_SEL + (latNum * 4);	
    return (CTR_LATCH_CTR_SELECT) (((__LATCH_UNIT1_TRIG_SELbits_t *)reg)->CTR_SEL);
}


//******************************************************************************
/* Function :  CTR_LatchDivSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_LatchDivSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_LatchDivSet function.
*/

PLIB_TEMPLATE void CTR_LatchDivSet_Default( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum , uint32_t divVal )
{
	volatile uint32_t *reg = &LATCH_UNIT1_TRIG_SEL + (latNum * 4);	
	((__LATCH_UNIT1_TRIG_SELbits_t *)reg)->DIV = divVal;
}


//******************************************************************************
/* Function :  CTR_LatchDivGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_LatchDivGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_LatchDivGet function.
*/

PLIB_TEMPLATE uint32_t CTR_LatchDivGet_Default( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum )
{
	volatile uint32_t *reg = &LATCH_UNIT1_TRIG_SEL + (latNum * 4);	
    return (uint32_t) (((__LATCH_UNIT1_TRIG_SELbits_t *)reg)->DIV);
}


#endif /*_CTR_LATCHTRIGGERSELECT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

