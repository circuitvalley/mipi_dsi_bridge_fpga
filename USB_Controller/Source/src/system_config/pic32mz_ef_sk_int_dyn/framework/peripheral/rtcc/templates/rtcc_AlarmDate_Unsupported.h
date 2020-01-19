/*******************************************************************************
  RTCC Peripheral Library Template Implementation

  File Name:
    rtcc_AlarmDate_Unsupported.h

  Summary:
    RTCC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : AlarmDate
    and its Variant : Unsupported
    For following APIs :
        PLIB_RTCC_AlarmDateGet
        PLIB_RTCC_AlarmDateSet
        PLIB_RTCC_AlarmMonthGet
        PLIB_RTCC_AlarmMonthSet
        PLIB_RTCC_AlarmDayGet
        PLIB_RTCC_AlarmDaySet
        PLIB_RTCC_AlarmWeekDayGet
        PLIB_RTCC_AlarmWeekDaySet
        PLIB_RTCC_ExistsAlarmDate

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

#ifndef _RTCC_ALARMDATE_UNSUPPORTED_H
#define _RTCC_ALARMDATE_UNSUPPORTED_H

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
/* Function :  RTCC_AlarmDateGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmDateGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmDateGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_AlarmDateGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmDateGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_AlarmDateSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmDateSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmDateSet function.
*/

PLIB_TEMPLATE void RTCC_AlarmDateSet_Unsupported( RTCC_MODULE_ID index , uint32_t data )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmDateSet");
}


//******************************************************************************
/* Function :  RTCC_AlarmMonthGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmMonthGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmMonthGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_AlarmMonthGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmMonthGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_AlarmMonthSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmMonthSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmMonthSet function.
*/

PLIB_TEMPLATE void RTCC_AlarmMonthSet_Unsupported( RTCC_MODULE_ID index , uint32_t month )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmMonthSet");
}


//******************************************************************************
/* Function :  RTCC_AlarmDayGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmDayGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmDayGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_AlarmDayGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmDayGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_AlarmDaySet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmDaySet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmDaySet function.
*/

PLIB_TEMPLATE void RTCC_AlarmDaySet_Unsupported( RTCC_MODULE_ID index , uint32_t day )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmDaySet");
}


//******************************************************************************
/* Function :  RTCC_AlarmWeekDayGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmWeekDayGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmWeekDayGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_AlarmWeekDayGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmWeekDayGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_AlarmWeekDaySet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmWeekDaySet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmWeekDaySet function.
*/

PLIB_TEMPLATE void RTCC_AlarmWeekDaySet_Unsupported( RTCC_MODULE_ID index , uint32_t weekday )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmWeekDaySet");
}


//******************************************************************************
/* Function :  RTCC_ExistsAlarmDate_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_ExistsAlarmDate

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_ExistsAlarmDate function.
*/

PLIB_TEMPLATE bool RTCC_ExistsAlarmDate_Unsupported( RTCC_MODULE_ID index )
{
    return false;
}


#endif /*_RTCC_ALARMDATE_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

