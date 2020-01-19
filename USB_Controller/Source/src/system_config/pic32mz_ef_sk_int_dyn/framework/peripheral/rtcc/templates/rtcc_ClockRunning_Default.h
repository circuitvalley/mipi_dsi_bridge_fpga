/*******************************************************************************
  RTCC Peripheral Library Template Implementation

  File Name:
    rtcc_ClockRunning_Default.h

  Summary:
    RTCC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ClockRunning
    and its Variant : Default
    For following APIs :
        PLIB_RTCC_ClockRunningStatus
        PLIB_RTCC_ExistsClockRunning

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

#ifndef _RTCC_CLOCKRUNNING_DEFAULT_H
#define _RTCC_CLOCKRUNNING_DEFAULT_H

#include "rtcc_Registers.h"

//******************************************************************************
/* Function :  RTCC_ClockRunningStatus_Default

  Summary:
    Implements Default variant of PLIB_RTCC_ClockRunningStatus 

  Description:
    This template implements the Default variant of the PLIB_RTCC_ClockRunningStatus function.
*/

PLIB_TEMPLATE bool RTCC_ClockRunningStatus_Default( RTCC_MODULE_ID index )
{
    volatile rtcc_register_t *rtc = (rtcc_register_t *)index;

    return rtc->RTCCON.RTCCLKON;
}


//******************************************************************************
/* Function :  RTCC_ExistsClockRunning_Default

  Summary:
    Implements Default variant of PLIB_RTCC_ExistsClockRunning

  Description:
    This template implements the Default variant of the PLIB_RTCC_ExistsClockRunning function.
*/

#define PLIB_RTCC_ExistsClockRunning PLIB_RTCC_ExistsClockRunning
PLIB_TEMPLATE bool RTCC_ExistsClockRunning_Default( RTCC_MODULE_ID index )
{
    return true;
}


#endif /*_RTCC_CLOCKRUNNING_DEFAULT_H*/

/******************************************************************************
 End of File
*/

