/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_InputTriggerMode_Default.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : InputTriggerMode
    and its Variant : Default
    For following APIs :
        PLIB_PTG_InputTriggerModeSelect
        PLIB_PTG_InputTriggerModeGet
        PLIB_PTG_ExistsInputTriggerMode

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

#ifndef _PTG_INPUTTRIGGERMODE_DEFAULT_H
#define _PTG_INPUTTRIGGERMODE_DEFAULT_H

//******************************************************************************
/* Function :  PTG_InputTriggerModeSelect_Default

  Summary:
    Implements Default variant of PLIB_PTG_InputTriggerModeSelect 

  Description:
    This template implements the Default variant of the PLIB_PTG_InputTriggerModeSelect function.
*/

PLIB_TEMPLATE void PTG_InputTriggerModeSelect_Default( PTG_MODULE_ID index , PTG_INPUT_MODE InputTrigMode )
{
	PTGCONbits.PTGITM = InputTrigMode & 0x03;
}


//******************************************************************************
/* Function :  PTG_InputTriggerModeGet_Default

  Summary:
    Implements Default variant of PLIB_PTG_InputTriggerModeGet 

  Description:
    This template implements the Default variant of the PLIB_PTG_InputTriggerModeGet function.
*/

PLIB_TEMPLATE PTG_INPUT_MODE PTG_InputTriggerModeGet_Default( PTG_MODULE_ID index )
{
	return (PTG_INPUT_MODE) PTGCONbits.PTGITM; 	   
}


//******************************************************************************
/* Function :  PTG_ExistsInputTriggerMode_Default

  Summary:
    Implements Default variant of PLIB_PTG_ExistsInputTriggerMode

  Description:
    This template implements the Default variant of the PLIB_PTG_ExistsInputTriggerMode function.
*/

#define PLIB_PTG_ExistsInputTriggerMode PLIB_PTG_ExistsInputTriggerMode
PLIB_TEMPLATE bool PTG_ExistsInputTriggerMode_Default( PTG_MODULE_ID index )
{
    return true;
}


#endif /*_PTG_INPUTTRIGGERMODE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

