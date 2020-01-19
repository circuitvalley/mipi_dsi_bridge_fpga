/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTRDriftLinear_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTRDriftLinear
    and its Variant : Default
    For following APIs :
        PLIB_CTR_ExistsCTRDriftLIN
        PLIB_CTR_CTRLinearDriftSet
        PLIB_CTR_CTRLinearDriftGet

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

#ifndef _CTR_CTRDRIFTLINEAR_DEFAULT_H
#define _CTR_CTRDRIFTLINEAR_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsCTRDriftLIN_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsCTRDriftLIN

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsCTRDriftLIN function.
*/

#define PLIB_CTR_ExistsCTRDriftLIN PLIB_CTR_ExistsCTRDriftLIN
PLIB_TEMPLATE bool CTR_ExistsCTRDriftLIN_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_CTRLinearDriftSet_Default

  Summary:
    Implements Default variant of PLIB_CTR_CTRLinearDriftSet 

  Description:
    This template implements the Default variant of the PLIB_CTR_CTRLinearDriftSet function.
*/

PLIB_TEMPLATE void CTR_CTRLinearDriftSet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , uint32_t linDriftVal )
{
	volatile uint32_t *reg = &CTR0_DRIFT_LIN + (ctrSel * 10);
	*reg = linDriftVal & _CTR0_DRIFT_LIN_DRIFT_LIN_MASK;
}


//******************************************************************************
/* Function :  CTR_CTRLinearDriftGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_CTRLinearDriftGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_CTRLinearDriftGet function.
*/

PLIB_TEMPLATE uint32_t CTR_CTRLinearDriftGet_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_DRIFT_LIN + (ctrSel * 10);
    return (uint32_t) *reg;
}


#endif /*_CTR_CTRDRIFTLINEAR_DEFAULT_H*/

/******************************************************************************
 End of File
*/

