/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_SWTControl_Default.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : SWTControl
    and its Variant : Default
    For following APIs :
        PLIB_PTG_SWTEdgeTrigger
        PLIB_PTG_SWTLevelTrigger
        PLIB_PTG_SWTClear
        PLIB_PTG_SWTGet
        PLIB_PTG_ExistsSWTControl

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

#ifndef _PTG_SWTCONTROL_DEFAULT_H
#define _PTG_SWTCONTROL_DEFAULT_H

//******************************************************************************
/* Function :  PTG_SWTEdgeTrigger_Default

  Summary:
    Implements Default variant of PLIB_PTG_SWTEdgeTrigger 

  Description:
    This template implements the Default variant of the PLIB_PTG_SWTEdgeTrigger function.
*/

PLIB_TEMPLATE void PTG_SWTEdgeTrigger_Default( PTG_MODULE_ID index )
{
	PTGCONbits.PTGSWT = 0;
	PTGCONbits.PTGSWT = 1;
}


//******************************************************************************
/* Function :  PTG_SWTLevelTrigger_Default

  Summary:
    Implements Default variant of PLIB_PTG_SWTLevelTrigger 

  Description:
    This template implements the Default variant of the PLIB_PTG_SWTLevelTrigger function.
*/

PLIB_TEMPLATE void PTG_SWTLevelTrigger_Default( PTG_MODULE_ID index )
{
	PTGCONbits.PTGSWT = 1;
}


//******************************************************************************
/* Function :  PTG_SWTClear_Default

  Summary:
    Implements Default variant of PLIB_PTG_SWTClear 

  Description:
    This template implements the Default variant of the PLIB_PTG_SWTClear function.
*/

PLIB_TEMPLATE void PTG_SWTClear_Default( PTG_MODULE_ID index )
{
	PTGCONbits.PTGSWT = 0;
}


//******************************************************************************
/* Function :  PTG_SWTGet_Default

  Summary:
    Implements Default variant of PLIB_PTG_SWTGet 

  Description:
    This template implements the Default variant of the PLIB_PTG_SWTGet function.
*/

PLIB_TEMPLATE bool PTG_SWTGet_Default( PTG_MODULE_ID index )
{
	return (bool) PTGCONbits.PTGSWT;
}


//******************************************************************************
/* Function :  PTG_ExistsSWTControl_Default

  Summary:
    Implements Default variant of PLIB_PTG_ExistsSWTControl

  Description:
    This template implements the Default variant of the PLIB_PTG_ExistsSWTControl function.
*/

#define PLIB_PTG_ExistsSWTControl PLIB_PTG_ExistsSWTControl
PLIB_TEMPLATE bool PTG_ExistsSWTControl_Default( PTG_MODULE_ID index )
{
    return true;
}


#endif /*_PTG_SWTCONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

