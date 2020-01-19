/*******************************************************************************
  RTCC Peripheral Library Template Implementation

  File Name:
    rtcc_RTCTime_Default.h

  Summary:
    RTCC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : RTCTime
    and its Variant : Default
    For following APIs :
        PLIB_RTCC_RTCTimeGet
        PLIB_RTCC_RTCTimeSet
        PLIB_RTCC_RTCHourGet
        PLIB_RTCC_RTCHourSet
        PLIB_RTCC_RTCMinuteGet
        PLIB_RTCC_RTCMinuteSet
        PLIB_RTCC_RTCSecondGet
        PLIB_RTCC_RTCSecondSet
        PLIB_RTCC_ExistsRTCTime

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

#ifndef _RTCC_RTCTIME_DEFAULT_H
#define _RTCC_RTCTIME_DEFAULT_H

#include "rtcc_Registers.h"

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _RTCC_RTCTIME_VREG(index)
    _RTCC_TIME_HR10_VREG(index)
    _RTCC_TIME_HR01_VREG(index)
    _RTCC_TIME_MIN10_VREG(index)
    _RTCC_TIME_MIN01_VREG(index)
    _RTCC_TIME_SEC10_VREG(index)
    _RTCC_TIME_SEC01_VREG(index)

  MASKs: 
    _RTCC_RTCTIME_MASK(index)
    _RTCC_TIME_HR10_MASK(index)
    _RTCC_TIME_HR01_MASK(index)
    _RTCC_TIME_MIN10_MASK(index)
    _RTCC_TIME_MIN01_MASK(index)
    _RTCC_TIME_SEC10_MASK(index)
    _RTCC_TIME_SEC01_MASK(index)

  POSs: 
    _RTCC_RTCTIME_POS(index)
    _RTCC_TIME_HR10_POS(index)
    _RTCC_TIME_HR01_POS(index)
    _RTCC_TIME_MIN10_POS(index)
    _RTCC_TIME_MIN01_POS(index)
    _RTCC_TIME_SEC10_POS(index)
    _RTCC_TIME_SEC01_POS(index)

  LENs: 
    _RTCC_RTCTIME_LEN(index)
    _RTCC_TIME_HR10_LEN(index)
    _RTCC_TIME_HR01_LEN(index)
    _RTCC_TIME_MIN10_LEN(index)
    _RTCC_TIME_MIN01_LEN(index)
    _RTCC_TIME_SEC10_LEN(index)
    _RTCC_TIME_SEC01_LEN(index)

*/


//******************************************************************************
/* Function :  RTCC_RTCTimeGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCTimeGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCTimeGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCTimeGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCTIME.w;
}


//******************************************************************************
/* Function :  RTCC_RTCTimeSet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCTimeSet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCTimeSet function.
*/

PLIB_TEMPLATE void RTCC_RTCTimeSet_Default( RTCC_MODULE_ID index , uint32_t data )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    rtc->RTCTIME.w = data;
}


//******************************************************************************
/* Function :  RTCC_RTCHourGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCHourGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCHourGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCHourGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCTIME.HR01 | (rtc->RTCTIME.HR10 << _RTCTIME_HR01_LENGTH);
}


//******************************************************************************
/* Function :  RTCC_RTCHourSet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCHourSet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCHourSet function.
*/

PLIB_TEMPLATE void RTCC_RTCHourSet_Default( RTCC_MODULE_ID index , uint32_t hour )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;
    uint32_t tmpVal = rtc->RTCTIME.w;

    tmpVal &= ~(_RTCTIME_HR01_MASK | _RTCTIME_HR10_MASK);
    tmpVal |= hour << _RTCTIME_HR01_POSITION;
    rtc->RTCTIME.w = tmpVal;
}


//******************************************************************************
/* Function :  RTCC_RTCMinuteGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCMinuteGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCMinuteGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCMinuteGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCTIME.MIN01 | (rtc->RTCTIME.MIN10 << _RTCTIME_MIN01_LENGTH);
}


//******************************************************************************
/* Function :  RTCC_RTCMinuteSet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCMinuteSet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCMinuteSet function.
*/

PLIB_TEMPLATE void RTCC_RTCMinuteSet_Default( RTCC_MODULE_ID index , uint32_t minute )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;
    uint32_t tmpVal = rtc->RTCTIME.w;

    tmpVal &= ~(_RTCTIME_MIN01_MASK | _RTCTIME_MIN10_MASK);
    tmpVal |= minute << _RTCTIME_MIN01_POSITION;
    rtc->RTCTIME.w = tmpVal;
}


//******************************************************************************
/* Function :  RTCC_RTCSecondGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCSecondGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCSecondGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCSecondGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCTIME.SEC01 | (rtc->RTCTIME.SEC10 << _RTCTIME_SEC01_LENGTH);
}


//******************************************************************************
/* Function :  RTCC_RTCSecondSet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCSecondSet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCSecondSet function.
*/

PLIB_TEMPLATE void RTCC_RTCSecondSet_Default( RTCC_MODULE_ID index , uint32_t second )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;
    uint32_t tmpVal = rtc->RTCTIME.w;

    tmpVal &= ~(_RTCTIME_SEC01_MASK | _RTCTIME_SEC10_MASK);
    tmpVal |= second << _RTCTIME_SEC01_POSITION;
    rtc->RTCTIME.w = tmpVal;
}


//******************************************************************************
/* Function :  RTCC_ExistsRTCTime_Default

  Summary:
    Implements Default variant of PLIB_RTCC_ExistsRTCTime

  Description:
    This template implements the Default variant of the PLIB_RTCC_ExistsRTCTime function.
*/

#define PLIB_RTCC_ExistsRTCTime PLIB_RTCC_ExistsRTCTime
PLIB_TEMPLATE bool RTCC_ExistsRTCTime_Default( RTCC_MODULE_ID index )
{
    return true;
}


#endif /*_RTCC_RTCTIME_DEFAULT_H*/

/******************************************************************************
 End of File
*/

