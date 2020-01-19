/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_SingleStepControl_Default.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : SingleStepControl
    and its Variant : Default
    For following APIs :
        PLIB_PTG_SingleStepEnable
        PLIB_PTG_SingleStepDisable
        PLIB_PTG_ExistsSingleStepControl

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

#ifndef _PTG_SINGLESTEPCONTROL_DEFAULT_H
#define _PTG_SINGLESTEPCONTROL_DEFAULT_H

//******************************************************************************
/* Function :  PTG_SingleStepEnable_Default

  Summary:
    Implements Default variant of PLIB_PTG_SingleStepEnable 

  Description:
    This template implements the Default variant of the PLIB_PTG_SingleStepEnable function.
*/

PLIB_TEMPLATE void PTG_SingleStepEnable_Default( PTG_MODULE_ID index )
{
	PTGCONbits.PTGSSEN = 1;
}


//******************************************************************************
/* Function :  PTG_SingleStepDisable_Default

  Summary:
    Implements Default variant of PLIB_PTG_SingleStepDisable 

  Description:
    This template implements the Default variant of the PLIB_PTG_SingleStepDisable function.
*/

PLIB_TEMPLATE void PTG_SingleStepDisable_Default( PTG_MODULE_ID index )
{
	PTGCONbits.PTGSSEN = 0;
}


//******************************************************************************
/* Function :  PTG_ExistsSingleStepControl_Default

  Summary:
    Implements Default variant of PLIB_PTG_ExistsSingleStepControl

  Description:
    This template implements the Default variant of the PLIB_PTG_ExistsSingleStepControl function.
*/

#define PLIB_PTG_ExistsSingleStepControl PLIB_PTG_ExistsSingleStepControl
PLIB_TEMPLATE bool PTG_ExistsSingleStepControl_Default( PTG_MODULE_ID index )
{
    return true;
}


#endif /*_PTG_SINGLESTEPCONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

