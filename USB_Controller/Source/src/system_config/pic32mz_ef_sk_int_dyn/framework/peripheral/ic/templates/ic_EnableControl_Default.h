/*******************************************************************************
  IC Peripheral Library Template Implementation

  File Name:
    ic_EnableControl_Default.h

  Summary:
    IC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : EnableControl
    and its Variant : Default
    For following APIs :
        PLIB_IC_Enable
        PLIB_IC_Disable
        PLIB_IC_ExistsEnable

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _IC_ENABLECONTROL_DEFAULT_H
#define _IC_ENABLECONTROL_DEFAULT_H

#include "ic_Registers.h"

//******************************************************************************
/* Function :  IC_Enable_Default

  Summary:
    Implements Default variant of PLIB_IC_Enable 

  Description:
    This template implements the Default variant of the PLIB_IC_Enable function.
*/

PLIB_TEMPLATE void IC_Enable_Default( IC_MODULE_ID index )
{
    volatile ic_register_t *regs = (ic_register_t *)index;

    regs->ICxCONSET = _IC1CON_ON_MASK;
}


//******************************************************************************
/* Function :  IC_Disable_Default

  Summary:
    Implements Default variant of PLIB_IC_Disable 

  Description:
    This template implements the Default variant of the PLIB_IC_Disable function.
*/

PLIB_TEMPLATE void IC_Disable_Default( IC_MODULE_ID index )
{
    volatile ic_register_t *regs = (ic_register_t *)index;

    regs->ICxCONCLR = _IC1CON_ON_MASK;
}


//******************************************************************************
/* Function :  IC_ExistsEnable_Default

  Summary:
    Implements Default variant of PLIB_IC_ExistsEnable

  Description:
    This template implements the Default variant of the PLIB_IC_ExistsEnable function.
*/

#define PLIB_IC_ExistsEnable PLIB_IC_ExistsEnable
PLIB_TEMPLATE bool IC_ExistsEnable_Default( IC_MODULE_ID index )
{
    return true;
}


#endif /*_IC_ENABLECONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

