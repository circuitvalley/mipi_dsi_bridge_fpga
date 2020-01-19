/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_StepCommand_Unsupported.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : StepCommand
    and its Variant : Unsupported
    For following APIs :
        PLIB_PTG_StepCommandSet
        PLIB_PTG_StepCommandGet
        PLIB_PTG_ExistsStepCommand

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

#ifndef _PTG_STEPCOMMAND_UNSUPPORTED_H
#define _PTG_STEPCOMMAND_UNSUPPORTED_H

//******************************************************************************
/* Function :  PTG_StepCommandSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_StepCommandSet 

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_StepCommandSet function.
*/

PLIB_TEMPLATE void PTG_StepCommandSet_Unsupported( PTG_MODULE_ID index , uint8_t stepLoc , uint8_t command )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_PTG_StepCommandSet");
}


//******************************************************************************
/* Function :  PTG_StepCommandGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_StepCommandGet 

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_StepCommandGet function.
*/

PLIB_TEMPLATE uint8_t PTG_StepCommandGet_Unsupported( PTG_MODULE_ID index , uint8_t stepLoc )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_PTG_StepCommandGet");

    return (uint8_t) 0;
}


//******************************************************************************
/* Function :  PTG_ExistsStepCommand_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_ExistsStepCommand

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_ExistsStepCommand function.
*/

PLIB_TEMPLATE bool PTG_ExistsStepCommand_Unsupported( PTG_MODULE_ID index )
{
    return false;
}


#endif /*_PTG_STEPCOMMAND_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

