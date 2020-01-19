/*******************************************************************************
  RTCC Peripheral Library Template Implementation

  File Name:
    rtcc_RTCDate_Default.h

  Summary:
    RTCC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : RTCDate
    and its Variant : Default
    For following APIs :
        PLIB_RTCC_RTCDateGet
        PLIB_RTCC_RTCDateSet
        PLIB_RTCC_RTCYearGet
        PLIB_RTCC_RTCYearSet
        PLIB_RTCC_RTCMonthGet
        PLIB_RTCC_RTCMonthSet
        PLIB_RTCC_RTCDayGet
        PLIB_RTCC_RTCDaySet
        PLIB_RTCC_RTCWeekDayGet
        PLIB_RTCC_RTCWeekDaySet
        PLIB_RTCC_ExistsRTCDate

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

#ifndef _RTCC_RTCDATE_DEFAULT_H
#define _RTCC_RTCDATE_DEFAULT_H
#include "rtcc_Registers.h"

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _RTCC_RTCDATE_VREG(index)
    _RTCC_DATE_YEAR10_VREG(index)
    _RTCC_DATE_YEAR01_VREG(index)
    _RTCC_DATE_MONTH10_VREG(index)
    _RTCC_DATE_MONTH01_VREG(index)
    _RTCC_DATE_DAY10_VREG(index)
    _RTCC_DATE_DAY01_VREG(index)
    _RTCC_DATE_WDAY01_VREG(index)

  MASKs: 
    _RTCC_RTCDATE_MASK(index)
    _RTCC_DATE_YEAR10_MASK(index)
    _RTCC_DATE_YEAR01_MASK(index)
    _RTCC_DATE_MONTH10_MASK(index)
    _RTCC_DATE_MONTH01_MASK(index)
    _RTCC_DATE_DAY10_MASK(index)
    _RTCC_DATE_DAY01_MASK(index)
    _RTCC_DATE_WDAY01_MASK(index)

  POSs: 
    _RTCC_RTCDATE_POS(index)
    _RTCC_DATE_YEAR10_POS(index)
    _RTCC_DATE_YEAR01_POS(index)
    _RTCC_DATE_MONTH10_POS(index)
    _RTCC_DATE_MONTH01_POS(index)
    _RTCC_DATE_DAY10_POS(index)
    _RTCC_DATE_DAY01_POS(index)
    _RTCC_DATE_WDAY01_POS(index)

  LENs: 
    _RTCC_RTCDATE_LEN(index)
    _RTCC_DATE_YEAR10_LEN(index)
    _RTCC_DATE_YEAR01_LEN(index)
    _RTCC_DATE_MONTH10_LEN(index)
    _RTCC_DATE_MONTH01_LEN(index)
    _RTCC_DATE_DAY10_LEN(index)
    _RTCC_DATE_DAY01_LEN(index)
    _RTCC_DATE_WDAY01_LEN(index)

*/


//******************************************************************************
/* Function :  RTCC_RTCDateGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCDateGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCDateGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCDateGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCDATE.w;
}


//******************************************************************************
/* Function :  RTCC_RTCDateSet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCDateSet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCDateSet function.
*/

PLIB_TEMPLATE void RTCC_RTCDateSet_Default( RTCC_MODULE_ID index , uint32_t data )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    rtc->RTCDATE.w = data;
}


//******************************************************************************
/* Function :  RTCC_RTCYearGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCYearGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCYearGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCYearGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCDATE.YEAR01 | (rtc->RTCDATE.YEAR10 << _RTCDATE_YEAR01_LENGTH);
}


//******************************************************************************
/* Function :  RTCC_RTCYearSet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCYearSet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCYearSet function.
*/

PLIB_TEMPLATE void RTCC_RTCYearSet_Default( RTCC_MODULE_ID index , uint32_t year )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;
    uint32_t tmpVal = rtc->RTCDATE.w;

    tmpVal &= ~(_RTCDATE_YEAR01_MASK | _RTCDATE_YEAR10_MASK);
    tmpVal |= year << _RTCDATE_YEAR01_POSITION;
    rtc->RTCDATE.w = tmpVal;
}


//******************************************************************************
/* Function :  RTCC_RTCMonthGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCMonthGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCMonthGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCMonthGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCDATE.MONTH01 | (rtc->RTCDATE.MONTH10 << _RTCDATE_MONTH01_LENGTH);
}


//******************************************************************************
/* Function :  RTCC_RTCMonthSet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCMonthSet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCMonthSet function.
*/

PLIB_TEMPLATE void RTCC_RTCMonthSet_Default( RTCC_MODULE_ID index , uint32_t month )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;
    uint32_t tmpVal = rtc->RTCDATE.w;

    tmpVal &= ~(_RTCDATE_MONTH01_MASK | _RTCDATE_MONTH10_MASK);
    tmpVal |= month << _RTCDATE_MONTH01_POSITION;
    rtc->RTCDATE.w = tmpVal;
}


//******************************************************************************
/* Function :  RTCC_RTCDayGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCDayGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCDayGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCDayGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCDATE.DAY01 | (rtc->RTCDATE.DAY10 << _RTCDATE_DAY01_LENGTH);
}


//******************************************************************************
/* Function :  RTCC_RTCDaySet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCDaySet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCDaySet function.
*/

PLIB_TEMPLATE void RTCC_RTCDaySet_Default( RTCC_MODULE_ID index , uint32_t day )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;
    uint32_t tmpVal = rtc->RTCDATE.w;

    tmpVal &=~(_RTCDATE_DAY01_MASK | _RTCDATE_DAY10_MASK);
    tmpVal |= day << _RTCDATE_DAY01_POSITION;
    rtc->RTCDATE.w = tmpVal;
}


//******************************************************************************
/* Function :  RTCC_RTCWeekDayGet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCWeekDayGet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCWeekDayGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCWeekDayGet_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCDATE.WDAY01;
}


//******************************************************************************
/* Function :  RTCC_RTCWeekDaySet_Default

  Summary:
    Implements Default variant of PLIB_RTCC_RTCWeekDaySet 

  Description:
    This template implements the Default variant of the PLIB_RTCC_RTCWeekDaySet function.
*/

PLIB_TEMPLATE void RTCC_RTCWeekDaySet_Default( RTCC_MODULE_ID index , uint32_t weekday )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    rtc->RTCDATE.WDAY01 = weekday;
}


//******************************************************************************
/* Function :  RTCC_ExistsRTCDate_Default

  Summary:
    Implements Default variant of PLIB_RTCC_ExistsRTCDate

  Description:
    This template implements the Default variant of the PLIB_RTCC_ExistsRTCDate function.
*/

#define PLIB_RTCC_ExistsRTCDate PLIB_RTCC_ExistsRTCDate
PLIB_TEMPLATE bool RTCC_ExistsRTCDate_Default( RTCC_MODULE_ID index )
{
    return true;
}


#endif /*_RTCC_RTCDATE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

