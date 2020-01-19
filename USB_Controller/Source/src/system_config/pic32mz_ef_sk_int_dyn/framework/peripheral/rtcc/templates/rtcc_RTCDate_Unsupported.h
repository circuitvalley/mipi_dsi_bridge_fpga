/*******************************************************************************
  RTCC Peripheral Library Template Implementation

  File Name:
    rtcc_RTCDate_Unsupported.h

  Summary:
    RTCC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : RTCDate
    and its Variant : Unsupported
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

#ifndef _RTCC_RTCDATE_UNSUPPORTED_H
#define _RTCC_RTCDATE_UNSUPPORTED_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    None.

  MASKs: 
    None.

  POSs: 
    None.

  LENs: 
    None.

*/


//******************************************************************************
/* Function :  RTCC_RTCDateGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCDateGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCDateGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCDateGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCDateGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_RTCDateSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCDateSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCDateSet function.
*/

PLIB_TEMPLATE void RTCC_RTCDateSet_Unsupported( RTCC_MODULE_ID index , uint32_t data )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCDateSet");
}


//******************************************************************************
/* Function :  RTCC_RTCYearGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCYearGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCYearGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCYearGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCYearGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_RTCYearSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCYearSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCYearSet function.
*/

PLIB_TEMPLATE void RTCC_RTCYearSet_Unsupported( RTCC_MODULE_ID index , uint32_t year )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCYearSet");
}


//******************************************************************************
/* Function :  RTCC_RTCMonthGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCMonthGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCMonthGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCMonthGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCMonthGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_RTCMonthSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCMonthSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCMonthSet function.
*/

PLIB_TEMPLATE void RTCC_RTCMonthSet_Unsupported( RTCC_MODULE_ID index , uint32_t month )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCMonthSet");
}


//******************************************************************************
/* Function :  RTCC_RTCDayGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCDayGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCDayGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCDayGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCDayGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_RTCDaySet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCDaySet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCDaySet function.
*/

PLIB_TEMPLATE void RTCC_RTCDaySet_Unsupported( RTCC_MODULE_ID index , uint32_t day )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCDaySet");
}


//******************************************************************************
/* Function :  RTCC_RTCWeekDayGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCWeekDayGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCWeekDayGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_RTCWeekDayGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCWeekDayGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_RTCWeekDaySet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_RTCWeekDaySet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_RTCWeekDaySet function.
*/

PLIB_TEMPLATE void RTCC_RTCWeekDaySet_Unsupported( RTCC_MODULE_ID index , uint32_t weekday )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_RTCWeekDaySet");
}


//******************************************************************************
/* Function :  RTCC_ExistsRTCDate_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_ExistsRTCDate

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_ExistsRTCDate function.
*/

PLIB_TEMPLATE bool RTCC_ExistsRTCDate_Unsupported( RTCC_MODULE_ID index )
{
    return false;
}


#endif /*_RTCC_RTCDATE_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

