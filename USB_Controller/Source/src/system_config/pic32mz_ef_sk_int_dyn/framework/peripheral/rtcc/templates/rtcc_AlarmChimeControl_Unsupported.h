/*******************************************************************************
  RTCC Peripheral Library Template Implementation

  File Name:
    rtcc_AlarmChimeControl_Unsupported.h

  Summary:
    RTCC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : AlarmChimeControl
    and its Variant : Unsupported
    For following APIs :
        PLIB_RTCC_AlarmChimeEnable
        PLIB_RTCC_AlarmChimeDisable
        PLIB_RTCC_ExistsAlarmChimeControl

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

#ifndef _RTCC_ALARMCHIMECONTROL_UNSUPPORTED_H
#define _RTCC_ALARMCHIMECONTROL_UNSUPPORTED_H

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
/* Function :  RTCC_AlarmChimeEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmChimeEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmChimeEnable function.
*/

PLIB_TEMPLATE void RTCC_AlarmChimeEnable_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmChimeEnable");
}


//******************************************************************************
/* Function :  RTCC_AlarmChimeDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_AlarmChimeDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_AlarmChimeDisable function.
*/

PLIB_TEMPLATE void RTCC_AlarmChimeDisable_Unsupported( RTCC_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_RTCC_AlarmChimeDisable");
}


//******************************************************************************
/* Function :  RTCC_ExistsAlarmChimeControl_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_RTCC_ExistsAlarmChimeControl

  Description:
    This template implements the Unsupported variant of the PLIB_RTCC_ExistsAlarmChimeControl function.
*/

PLIB_TEMPLATE bool RTCC_ExistsAlarmChimeControl_Unsupported( RTCC_MODULE_ID index )
{
    return false;
}


#endif /*_RTCC_ALARMCHIMECONTROL_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

