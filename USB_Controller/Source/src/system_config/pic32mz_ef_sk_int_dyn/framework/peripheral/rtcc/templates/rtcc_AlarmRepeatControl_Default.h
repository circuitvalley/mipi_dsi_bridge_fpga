/*******************************************************************************
  RTCC Peripheral Library Template Implementation

  File Name:
    rtcc_AlarmRepeatControl_Default.h

  Summary:
    RTCC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : AlarmRepeatControl
    and its Variant : Default
    For following APIs :
        PLIB_RTCC_AlarmRepeatCountSet
        PLIB_RTCC_AlarmRepeatCountGet
        PLIB_RTCC_ExistsAlarmRepeatControl

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

#ifndef _RTCC_ALARMREPEATCONTROL_DEFAULT_H
#define _RTCC_ALARMREPEATCONTROL_DEFAULT_H

#include "rtcc_Registers.h"

//******************************************************************************
/* Function :  RTCC_AlarmRepeatCountSet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_AlarmRepeatCountSet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_AlarmRepeatCountSet function.
*/

PLIB_TEMPLATE void RTCC_AlarmRepeatCountSet_Default( RTCC_MODULE_ID index , uint8_t data )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    rtc->RTCALRM.ARPT = data;
}


//******************************************************************************
/* Function :  RTCC_AlarmRepeatCountGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_AlarmRepeatCountGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_AlarmRepeatCountGet function.
*/

PLIB_TEMPLATE uint8_t RTCC_AlarmRepeatCountGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCALRM.ARPT;
}


//******************************************************************************
/* Function :  RTCC_ExistsAlarmRepeatControl_Default

  Summary:
    Implements Default variant of PLIB_RTCC_ExistsAlarmRepeatControl

  Description:
    This template implements the Default variant of the PLIB_RTCC_ExistsAlarmRepeatControl function.
*/

#define PLIB_RTCC_ExistsAlarmRepeatControl PLIB_RTCC_ExistsAlarmRepeatControl
PLIB_TEMPLATE bool RTCC_ExistsAlarmRepeatControl_Default( RTCC_MODULE_ID index )
{
    return true;
}


#endif /*_RTCC_ALARMREPEATCONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

