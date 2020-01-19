/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTRFormatSelect_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTRFormatSelect
    and its Variant : Default
    For following APIs :
        PLIB_CTR_ExistsCTRFormatSel
        PLIB_CTR_CTRModeSelect
        PLIB_CTR_CTRModeStatus

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

#ifndef _CTR_CTRFORMATSELECT_DEFAULT_H
#define _CTR_CTRFORMATSELECT_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsCTRFormatSel_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsCTRFormatSel

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsCTRFormatSel function.
*/

#define PLIB_CTR_ExistsCTRFormatSel PLIB_CTR_ExistsCTRFormatSel
PLIB_TEMPLATE bool CTR_ExistsCTRFormatSel_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_CTRModeSelect_Default

  Summary:
    Implements Default variant of PLIB_CTR_CTRModeSelect 

  Description:
    This template implements the Default variant of the PLIB_CTR_CTRModeSelect function.
*/

PLIB_TEMPLATE void CTR_CTRModeSelect_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel , CTR_MODE_SELECT modeVal )
{
	volatile uint32_t *reg = &CTR0_ENABLE_FORMAT_SEL + (ctrSel * 10);
	((__CTR0_ENABLE_FORMAT_SELbits_t *)reg)->US_1394_MODE = modeVal;
}


//******************************************************************************
/* Function :  CTR_CTRModeStatus_Default

  Summary:
    Implements Default variant of PLIB_CTR_CTRModeStatus 

  Description:
    This template implements the Default variant of the PLIB_CTR_CTRModeStatus function.
*/

PLIB_TEMPLATE CTR_MODE_SELECT CTR_CTRModeStatus_Default( CTR_MODULE_ID index , CTR_SELECT ctrSel )
{
	volatile uint32_t *reg = &CTR0_ENABLE_FORMAT_SEL + (ctrSel * 10);
	return (CTR_MODE_SELECT) (((__CTR0_ENABLE_FORMAT_SELbits_t *)reg)->US_1394_MODE);
}


#endif /*_CTR_CTRFORMATSELECT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

