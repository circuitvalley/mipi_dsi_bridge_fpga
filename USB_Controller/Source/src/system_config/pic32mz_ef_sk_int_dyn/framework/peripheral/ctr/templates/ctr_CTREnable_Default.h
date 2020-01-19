/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTREnable_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTREnable
    and its Variant : Default
    For following APIs :
        PLIB_CTR_ExistsEnableCTR
        PLIB_CTR_EnableCTR
        PLIB_CTR_DisableCTR
        PLIB_CTR_ModuleStatus

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

#ifndef _CTR_CTRENABLE_DEFAULT_H
#define _CTR_CTRENABLE_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsEnableCTR_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsEnableCTR

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsEnableCTR function.
*/

#define PLIB_CTR_ExistsEnableCTR PLIB_CTR_ExistsEnableCTR
PLIB_TEMPLATE bool CTR_ExistsEnableCTR_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_EnableCTR_Default

  Summary:
    Implements Default variant of PLIB_CTR_EnableCTR 

  Description:
    This template implements the Default variant of the PLIB_CTR_EnableCTR function.
*/

PLIB_TEMPLATE void CTR_EnableCTR_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	if(CTR0 == ctrSel)
	{
		CTR0_ENABLE_FORMAT_SELbits.ENABLE = 1;
	}
	else if(CTR1 == ctrSel)
	{
		CTR1_ENABLE_FORMAT_SELbits.ENABLE = 1;
	}
	else
	{
		;
	}
}


//******************************************************************************
/* Function :  CTR_DisableCTR_Default

  Summary:
    Implements Default variant of PLIB_CTR_DisableCTR 

  Description:
    This template implements the Default variant of the PLIB_CTR_DisableCTR function.
*/

PLIB_TEMPLATE void CTR_DisableCTR_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	if(CTR0 == ctrSel)
	{
		CTR0_ENABLE_FORMAT_SELbits.ENABLE = 0;
	}
	else if(CTR1 == ctrSel)
	{
		CTR1_ENABLE_FORMAT_SELbits.ENABLE = 0;
	}
	else
	{
		;
	}
}


//******************************************************************************
/* Function :  CTR_ModuleStatus_Default

  Summary:
    Implements Default variant of PLIB_CTR_ModuleStatus 

  Description:
    This template implements the Default variant of the PLIB_CTR_ModuleStatus function.
*/

PLIB_TEMPLATE CTR_ENABLE_CONTROL CTR_ModuleStatus_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
    return (CTR_ENABLE_CONTROL) ((CTR0 == ctrSel) ? CTR0_ENABLE_FORMAT_SELbits.ENABLE : ((CTR1 == ctrSel) ? CTR0_ENABLE_FORMAT_SELbits.ENABLE : 0));
}


#endif /*_CTR_CTRENABLE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

