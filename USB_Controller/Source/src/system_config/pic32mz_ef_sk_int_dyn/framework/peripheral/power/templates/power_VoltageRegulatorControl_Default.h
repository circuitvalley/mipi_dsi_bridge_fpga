/*******************************************************************************
  POWER Peripheral Library Template Implementation

  File Name:
    power_VoltageRegulatorControl_Default.h

  Summary:
    POWER PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : VoltageRegulatorControl
    and its Variant : Default
    For following APIs :
        PLIB_POWER_ExistsVoltageRegulatorControl
        PLIB_POWER_VoltageRegulatorEnable
        PLIB_POWER_VoltageRegulatorDisable
        PLIB_POWER_VoltageRegulatorIsEnabled

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _POWER_VOLTAGEREGULATORCONTROL_DEFAULT_H
#define _POWER_VOLTAGEREGULATORCONTROL_DEFAULT_H

//******************************************************************************
/* Function :  POWER_ExistsVoltageRegulatorControl_Default

  Summary:
    Implements Default variant of PLIB_POWER_ExistsVoltageRegulatorControl

  Description:
    This template implements the Default variant of the PLIB_POWER_ExistsVoltageRegulatorControl function.
*/

#define PLIB_POWER_ExistsVoltageRegulatorControl PLIB_POWER_ExistsVoltageRegulatorControl
PLIB_TEMPLATE bool POWER_ExistsVoltageRegulatorControl_Default( POWER_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  POWER_VoltageRegulatorEnable_Default

  Summary:
    Implements Default variant of PLIB_POWER_VoltageRegulatorEnable 

  Description:
    This template implements the Default variant of the PLIB_POWER_VoltageRegulatorEnable function.
*/

PLIB_TEMPLATE void POWER_VoltageRegulatorEnable_Default( POWER_MODULE_ID index )
{
    RCONSET = _RCON_VREGS_MASK;
}


//******************************************************************************
/* Function :  POWER_VoltageRegulatorDisable_Default

  Summary:
    Implements Default variant of PLIB_POWER_VoltageRegulatorDisable 

  Description:
    This template implements the Default variant of the PLIB_POWER_VoltageRegulatorDisable function.
*/

PLIB_TEMPLATE void POWER_VoltageRegulatorDisable_Default( POWER_MODULE_ID index )
{
    RCONCLR = _RCON_VREGS_MASK;
}


//******************************************************************************
/* Function :  POWER_VoltageRegulatorIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_POWER_VoltageRegulatorIsEnabled 

  Description:
    This template implements the Default variant of the PLIB_POWER_VoltageRegulatorIsEnabled function.
*/

PLIB_TEMPLATE bool POWER_VoltageRegulatorIsEnabled_Default( POWER_MODULE_ID index )
{
    return (bool) (RCON & _RCON_VREGS_MASK);
}


#endif /*_POWER_VOLTAGEREGULATORCONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

