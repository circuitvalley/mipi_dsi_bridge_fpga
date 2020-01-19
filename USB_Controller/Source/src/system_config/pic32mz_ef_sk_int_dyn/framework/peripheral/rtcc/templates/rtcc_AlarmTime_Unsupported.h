/*******************************************************************************
  RTCC Peripheral Library Template Implementation

  File Name:
    rtcc_AlarmTime_Unsupported.h

  Summary:
    RTCC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : AlarmTime
    and its Variant : Unsupported
    For following APIs :
        PLIB_RTCC_AlarmTimeGet
        PLIB_RTCC_AlarmTimeSet
        PLIB_RTCC_AlarmHourGet
        PLIB_RTCC_AlarmHourSet
        PLIB_RTCC_AlarmMinuteGet
        PLIB_RTCC_AlarmMinuteSet
        PLIB_RTCC_AlarmSecondGet
        PLIB_RTCC_AlarmSecondSet
        PLIB_RTCC_ExistsAlarmTime

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

#ifndef _RTCC_ALARMTIME_UNSUPPORTED_H
#define _RTCC_ALARMTIME_UNSUPPORTED_H

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
/* Function :  RTCC_AlarmTimeGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmTimeGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmTimeGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_AlarmTimeGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmTimeGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_AlarmTimeSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmTimeSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmTimeSet function.
*/

PLIB_TEMPLATE void RTCC_AlarmTimeSet_Unsupported( RTCC_MODULE_ID index , uint32_t data )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmTimeSet");
}


//******************************************************************************
/* Function :  RTCC_AlarmHourGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmHourGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmHourGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_AlarmHourGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmHourGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_AlarmHourSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmHourSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmHourSet function.
*/

PLIB_TEMPLATE void RTCC_AlarmHourSet_Unsupported( RTCC_MODULE_ID index , uint32_t hour )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmHourSet");
}


//******************************************************************************
/* Function :  RTCC_AlarmMinuteGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmMinuteGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmMinuteGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_AlarmMinuteGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmMinuteGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_AlarmMinuteSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmMinuteSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmMinuteSet function.
*/

PLIB_TEMPLATE void RTCC_AlarmMinuteSet_Unsupported( RTCC_MODULE_ID index , uint32_t minute )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmMinuteSet");
}


//******************************************************************************
/* Function :  RTCC_AlarmSecondGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmSecondGet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmSecondGet function.
*/

PLIB_TEMPLATE uint32_t RTCC_AlarmSecondGet_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmSecondGet");

    return 0;
}


//******************************************************************************
/* Function :  RTCC_AlarmSecondSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmSecondSet 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmSecondSet function.
*/

PLIB_TEMPLATE void RTCC_AlarmSecondSet_Unsupported( RTCC_MODULE_ID index , uint32_t second )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmSecondSet");
}


//******************************************************************************
/* Function :  RTCC_ExistsAlarmTime_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_ExistsAlarmTime

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_ExistsAlarmTime function.
*/

PLIB_TEMPLATE bool RTCC_ExistsAlarmTime_Unsupported( RTCC_MODULE_ID index )
{
    return false;
}


#endif /*_RTCC_ALARMTIME_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

