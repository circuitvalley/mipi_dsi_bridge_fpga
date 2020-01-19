/*******************************************************************************
  OC Peripheral Library Template Implementation

  File Name:
    oc_EnableControl_Default.h

  Summary:
    OC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : EnableControl
    and its Variant : Default
    For following APIs :
        PLIB_OC_Enable
        PLIB_OC_Disable
        PLIB_OC_IsEnabled
        PLIB_OC_ExistsEnableControl

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

#ifndef _OC_ENABLECONTROL_DEFAULT_H
#define _OC_ENABLECONTROL_DEFAULT_H

#include "oc_Registers.h"

//******************************************************************************
/* Function :  OC_Enable_Default

  Summary:
    Implements Default variant of PLIB_OC_Enable 

  Description:
    This template implements the Default variant of the PLIB_OC_Enable function.
    Operation is atomic.
*/

PLIB_TEMPLATE void OC_Enable_Default( OC_MODULE_ID index )
{
    volatile oc_register_t *regs = (volatile oc_register_t *)index;

    regs->OCxCONSET = OCxCON_ON_MASK;
}


//******************************************************************************
/* Function :  OC_Disable_Default

  Summary:
    Implements Default variant of PLIB_OC_Disable 

  Description:
    This template implements the Default variant of the PLIB_OC_Disable function.
    Operation is atomic.
*/

PLIB_TEMPLATE void OC_Disable_Default( OC_MODULE_ID index )
{
    volatile oc_register_t *regs = (volatile oc_register_t *)index;

    regs->OCxCONCLR = OCxCON_ON_MASK;
}


//******************************************************************************
/* Function :  OC_IsEnabled_Default

  Summary:
    Implements Default variant of PLIB_OC_IsEnabled 

  Description:
    This template implements the Default variant of the PLIB_OC_IsEnabled function.
    Operation is atomic.
*/

PLIB_TEMPLATE bool OC_IsEnabled_Default( OC_MODULE_ID index )
{
    volatile oc_register_t *regs = (volatile oc_register_t *)index;

    return regs->OCxCON.ON;
}


//******************************************************************************
/* Function :  OC_ExistsEnableControl_Default

  Summary:
    Implements Default variant of PLIB_OC_ExistsEnableControl

  Description:
    This template implements the Default variant of the PLIB_OC_ExistsEnableControl function.
*/

#define PLIB_OC_ExistsEnableControl PLIB_OC_ExistsEnableControl
PLIB_TEMPLATE bool OC_ExistsEnableControl_Default( OC_MODULE_ID index )
{
    return true;
}


#endif /*_OC_ENABLECONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

