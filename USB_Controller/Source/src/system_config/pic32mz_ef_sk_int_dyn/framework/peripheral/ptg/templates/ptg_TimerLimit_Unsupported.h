/*******************************************************************************
  PTG Peripheral Library Template Implementation

  File Name:
    ptg_TimerLimit_Unsupported.h

  Summary:
    PTG PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : TimerLimit
    and its Variant : Unsupported
    For following APIs :
        PLIB_PTG_TimerLimitSet
        PLIB_PTG_TimerLimitGet
        PLIB_PTG_ExistsTimerLimit

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

#ifndef _PTG_TIMERLIMIT_UNSUPPORTED_H
#define _PTG_TIMERLIMIT_UNSUPPORTED_H

//******************************************************************************
/* Function :  PTG_TimerLimitSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_TimerLimitSet 

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_TimerLimitSet function.
*/

PLIB_TEMPLATE void PTG_TimerLimitSet_Unsupported( PTG_MODULE_ID index , PTG_TIMER_SEL timerSel , uint16_t timerLimitValue )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_PTG_TimerLimitSet");
}


//******************************************************************************
/* Function :  PTG_TimerLimitGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_TimerLimitGet 

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_TimerLimitGet function.
*/

PLIB_TEMPLATE uint16_t PTG_TimerLimitGet_Unsupported( PTG_MODULE_ID index , PTG_TIMER_SEL timerSel )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_PTG_TimerLimitGet");

    return (uint16_t) 0;
}


//******************************************************************************
/* Function :  PTG_ExistsTimerLimit_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PTG_ExistsTimerLimit

  Description:
    This template implements the Unsupported variant of the PLIB_PTG_ExistsTimerLimit function.
*/

PLIB_TEMPLATE bool PTG_ExistsTimerLimit_Unsupported( PTG_MODULE_ID index )
{
    return false;
}


#endif /*_PTG_TIMERLIMIT_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

