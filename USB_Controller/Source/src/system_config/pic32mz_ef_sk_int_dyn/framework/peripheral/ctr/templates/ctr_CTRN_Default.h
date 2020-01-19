/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTRN_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTRN
    and its Variant : Default
    For following APIs :
        PLIB_CTR_ExistsNValue
        PLIB_CTR_NValueSet
        PLIB_CTR_NValueGet

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

#ifndef _CTR_CTRN_DEFAULT_H
#define _CTR_CTRN_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsNValue_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsNValue

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsNValue function.
*/

#define PLIB_CTR_ExistsNValue PLIB_CTR_ExistsNValue
PLIB_TEMPLATE bool CTR_ExistsNValue_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_NValueSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_NValueSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_NValueSet function.
*/

PLIB_TEMPLATE void CTR_NValueSet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t valueN )
{
	volatile uint32_t *reg = &CTR0_N + (ctrSel * 10);	
	*reg = valueN;
}


//******************************************************************************
/* Function :  CTR_NValueGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_NValueGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_NValueGet function.
*/

PLIB_TEMPLATE uint32_t CTR_NValueGet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_N + (ctrSel * 10);		
    return *reg;
}


#endif /*_CTR_CTRN_DEFAULT_H*/

/******************************************************************************
 End of File
*/

