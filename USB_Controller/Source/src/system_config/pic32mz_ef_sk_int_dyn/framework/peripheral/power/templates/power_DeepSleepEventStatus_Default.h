/*******************************************************************************
  POWER Peripheral Library Template Implementation

  File Name:
    power_DeepSleepEventStatus_Default.h

  Summary:
    POWER PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : DeepSleepEventStatus
    and its Variant : Default
    For following APIs :
        PLIB_POWER_ExistsDeepSleepEventStatus
        PLIB_POWER_DeepSleepEventStatusGet
        PLIB_POWER_DeepSleepEventStatusClear

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

#ifndef _POWER_DEEPSLEEPEVENTSTATUS_DEFAULT_H
#define _POWER_DEEPSLEEPEVENTSTATUS_DEFAULT_H

//******************************************************************************
/* Function :  POWER_ExistsDeepSleepEventStatus_Default

  Summary:
    Implements Default variant of PLIB_POWER_ExistsDeepSleepEventStatus

  Description:
    This template implements the Default variant of the PLIB_POWER_ExistsDeepSleepEventStatus function.
*/

#define PLIB_POWER_ExistsDeepSleepEventStatus PLIB_POWER_ExistsDeepSleepEventStatus
PLIB_TEMPLATE bool POWER_ExistsDeepSleepEventStatus_Default( POWER_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  POWER_DeepSleepEventStatusGet_Default

  Summary:
    Implements Default variant of PLIB_POWER_DeepSleepEventStatusGet 

  Description:
    This template implements the Default variant of the PLIB_POWER_DeepSleepEventStatusGet function.
*/

PLIB_TEMPLATE DEEP_SLEEP_EVENT POWER_DeepSleepEventStatusGet_Default( POWER_MODULE_ID index )
{
    return (DEEP_SLEEP_EVENT) (DSWAKE | (DSCONbits.DSBOR << 1u));
}


//******************************************************************************
/* Function :  POWER_DeepSleepEventStatusClear_Default

  Summary:
    Implements Default variant of PLIB_POWER_DeepSleepEventStatusClear 

  Description:
    This template implements the Default variant of the PLIB_POWER_DeepSleepEventStatusClear function.
*/

PLIB_TEMPLATE void POWER_DeepSleepEventStatusClear_Default( POWER_MODULE_ID index , DEEP_SLEEP_EVENT event )
{
    uint32_t dswakemask;
    
    if (DEEP_SLEEP_EVENT_BOR == event)
    {
        /*BOR event status flag is present in DSCON SFR */
        
        /* Deep Sleep SFRs need two consecutive writes without SET/CLR/INV usage */
        DSCONbits.DSBOR = 0u;
        DSCONbits.DSBOR = 0u;
    }
    else
    {
        /* All other Deep Sleep flags are in DSWAKE SFR */
        dswakemask = (DSWAKE & (~event));
        
        /* Deep Sleep SFRs need two consecutive writes without SET/CLR/INV usage */
        DSWAKE = dswakemask;
        DSWAKE = dswakemask;
    }
}


#endif /*_POWER_DEEPSLEEPEVENTSTATUS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

