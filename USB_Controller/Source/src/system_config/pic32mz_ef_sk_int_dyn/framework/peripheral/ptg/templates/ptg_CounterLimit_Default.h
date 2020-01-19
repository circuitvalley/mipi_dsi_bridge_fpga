/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_CounterLimit_Default.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CounterLimit
    and its Variant : Default
    For following APIs :
        PLIB_PTG_CounterLimitSet
        PLIB_PTG_CounterLimitGet
        PLIB_PTG_ExistsCounterLimit

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

#ifndef _PTG_COUNTERLIMIT_DEFAULT_H
#define _PTG_COUNTERLIMIT_DEFAULT_H

//******************************************************************************
/* Function :  PTG_CounterLimitSet_Default

  Summary:
    Implements Default variant of PLIB_PTG_CounterLimitSet 

  Description:
    This template implements the Default variant of the PLIB_PTG_CounterLimitSet function.
*/

PLIB_TEMPLATE void PTG_CounterLimitSet_Default( PTG_MODULE_ID index , PTG_COUNTER_SEL counterSel , uint16_t counterLimit )
{
	volatile uint32_t * regAddr;
	regAddr = &PTGC0LIM + (counterSel * 4);
	*regAddr = counterLimit & 0xFFFF;
}


//******************************************************************************
/* Function :  PTG_CounterLimitGet_Default

  Summary:
    Implements Default variant of PLIB_PTG_CounterLimitGet 

  Description:
    This template implements the Default variant of the PLIB_PTG_CounterLimitGet function.
*/

PLIB_TEMPLATE uint16_t PTG_CounterLimitGet_Default( PTG_MODULE_ID index , PTG_COUNTER_SEL counterSel )
{
	volatile uint32_t * regAddr;
	regAddr = &PTGC0LIM + (counterSel * 4);	
	return *regAddr;
}


//******************************************************************************
/* Function :  PTG_ExistsCounterLimit_Default

  Summary:
    Implements Default variant of PLIB_PTG_ExistsCounterLimit

  Description:
    This template implements the Default variant of the PLIB_PTG_ExistsCounterLimit function.
*/

#define PLIB_PTG_ExistsCounterLimit PLIB_PTG_ExistsCounterLimit
PLIB_TEMPLATE bool PTG_ExistsCounterLimit_Default( PTG_MODULE_ID index )
{
    return true;
}


#endif /*_PTG_COUNTERLIMIT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

