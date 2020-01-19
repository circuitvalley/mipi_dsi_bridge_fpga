/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_StepCommand_Default.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : StepCommand
    and its Variant : Default
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

#ifndef _PTG_STEPCOMMAND_DEFAULT_H
#define _PTG_STEPCOMMAND_DEFAULT_H

//******************************************************************************
/* Function :  PTG_StepCommandSet_Default

  Summary:
    Implements Default variant of PLIB_PTG_StepCommandSet 

  Description:
    This template implements the Default variant of the PLIB_PTG_StepCommandSet function.
*/

PLIB_TEMPLATE void PTG_StepCommandSet_Default( PTG_MODULE_ID index , uint8_t stepLoc , uint8_t command )
{
	uint8_t stepRegOffset = stepLoc / 4;
	uint8_t stepOffset = stepLoc % 4;
	volatile uint32_t *regAddr = &PTGQUE0 + (stepRegOffset * 4);
	
	*regAddr = (*regAddr & ~(_PTGQUE0_STEP0_MASK << (stepOffset * 8))) | (command << (stepOffset * 8));
}


//******************************************************************************
/* Function :  PTG_StepCommandGet_Default

  Summary:
    Implements Default variant of PLIB_PTG_StepCommandGet 

  Description:
    This template implements the Default variant of the PLIB_PTG_StepCommandGet function.
*/

PLIB_TEMPLATE uint8_t PTG_StepCommandGet_Default( PTG_MODULE_ID index , uint8_t stepLoc )
{
	uint8_t stepRegOffset = stepLoc / 4;
	uint8_t stepOffset = stepLoc % 4;
	volatile uint32_t *regAddr = &PTGQUE0 + (stepRegOffset * 4);
	
	return (uint8_t)(*regAddr >> (stepOffset * 8));
}


//******************************************************************************
/* Function :  PTG_ExistsStepCommand_Default

  Summary:
    Implements Default variant of PLIB_PTG_ExistsStepCommand

  Description:
    This template implements the Default variant of the PLIB_PTG_ExistsStepCommand function.
*/

#define PLIB_PTG_ExistsStepCommand PLIB_PTG_ExistsStepCommand
PLIB_TEMPLATE bool PTG_ExistsStepCommand_Default( PTG_MODULE_ID index )
{
    return true;
}


#endif /*_PTG_STEPCOMMAND_DEFAULT_H*/

/******************************************************************************
 End of File
*/

