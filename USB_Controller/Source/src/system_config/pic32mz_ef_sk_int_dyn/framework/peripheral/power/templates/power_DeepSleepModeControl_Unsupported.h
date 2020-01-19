/*******************************************************************************
  POWER Peripheral Library Template Implementation

  File Name:
    power_DeepSleepModeControl_Unsupported.h

  Summary:
    POWER PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : DeepSleepModeControl
    and its Variant : Unsupported
    For following APIs :
        PLIB_POWER_ExistsDeepSleepMode
        PLIB_POWER_DeepSleepModeEnable
        PLIB_POWER_DeepSleepModeDisable
        PLIB_POWER_DeepSleepModeIsEnabled

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

#ifndef _POWER_DEEPSLEEPMODECONTROL_UNSUPPORTED_H
#define _POWER_DEEPSLEEPMODECONTROL_UNSUPPORTED_H


//******************************************************************************
/* Function :  POWER_ExistsDeepSleepMode_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_POWER_ExistsDeepSleepMode

  Description:
    This template implements the Unsupported variant of the PLIB_POWER_ExistsDeepSleepMode function.
*/

PLIB_TEMPLATE bool POWER_ExistsDeepSleepMode_Unsupported( POWER_MODULE_ID index )
{
    return false;
}


//******************************************************************************
/* Function :  POWER_DeepSleepModeEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_POWER_DeepSleepModeEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_POWER_DeepSleepModeEnable function.
*/

PLIB_TEMPLATE void POWER_DeepSleepModeEnable_Unsupported( POWER_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_POWER_DeepSleepModeEnable");
}


//******************************************************************************
/* Function :  POWER_DeepSleepModeDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_POWER_DeepSleepModeDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_POWER_DeepSleepModeDisable function.
*/

PLIB_TEMPLATE void POWER_DeepSleepModeDisable_Unsupported( POWER_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_POWER_DeepSleepModeDisable");
}


//******************************************************************************
/* Function :  POWER_DeepSleepModeIsEnabled_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_POWER_DeepSleepModeIsEnabled 

  Description:
    This template implements the Unsupported variant of the PLIB_POWER_DeepSleepModeIsEnabled function.
*/

PLIB_TEMPLATE bool POWER_DeepSleepModeIsEnabled_Unsupported( POWER_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_POWER_DeepSleepModeIsEnabled");

    return false;
}


#endif /*_POWER_DEEPSLEEPMODECONTROL_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

