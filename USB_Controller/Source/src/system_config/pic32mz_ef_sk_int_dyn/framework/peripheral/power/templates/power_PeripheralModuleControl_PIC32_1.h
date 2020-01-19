/*******************************************************************************
  POWER Peripheral Library Template Implementation

  File Name:
    power_PeripheralModuleControl_PIC32_1.h

  Summary:
    POWER PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PeripheralModuleControl
    and its Variant : PIC32_1
    For following APIs :
        PLIB_POWER_ExistsPeripheralModuleControl
        PLIB_POWER_PeripheralModuleDisable
        PLIB_POWER_PeripheralModuleEnable
        PLIB_POWER_PeripheralModuleIsEnabled

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

#ifndef _POWER_PERIPHERALMODULECONTROL_PIC32_1_H
#define _POWER_PERIPHERALMODULECONTROL_PIC32_1_H

//******************************************************************************
/* Function :  POWER_ExistsPeripheralModuleControl_PIC32_1

  Summary:
    Implements PIC32_1 variant of PLIB_POWER_ExistsPeripheralModuleControl

  Description:
    This template implements the PIC32_1 variant of the PLIB_POWER_ExistsPeripheralModuleControl function.
*/

#define PLIB_POWER_ExistsPeripheralModuleControl PLIB_POWER_ExistsPeripheralModuleControl
PLIB_TEMPLATE bool POWER_ExistsPeripheralModuleControl_PIC32_1( POWER_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  POWER_PeripheralModuleDisable_PIC32_1

  Summary:
    Implements PIC32_1 variant of PLIB_POWER_PeripheralModuleDisable 

  Description:
    This template implements the PIC32_1 variant of the PLIB_POWER_PeripheralModuleDisable function.
*/

PLIB_TEMPLATE void POWER_PeripheralModuleDisable_PIC32_1( POWER_MODULE_ID index , POWER_MODULE source )
{
    volatile uint32_t *pmdRegister = (uint32_t *)(&PMD1 + ((source/32u) * 4u));
    uint8_t pmdPosition = source % 32u;

    *pmdRegister |= (1u << pmdPosition);
}


//******************************************************************************
/* Function :  POWER_PeripheralModuleEnable_PIC32_1

  Summary:
    Implements PIC32_1 variant of PLIB_POWER_PeripheralModuleEnable 

  Description:
    This template implements the PIC32_1 variant of the PLIB_POWER_PeripheralModuleEnable function.
*/

PLIB_TEMPLATE void POWER_PeripheralModuleEnable_PIC32_1( POWER_MODULE_ID index , POWER_MODULE source )
{
    volatile uint32_t *pmdRegister = (uint32_t *)(&PMD1 + ((source/32u) * 4u));
    uint8_t pmdPosition = source % 32u;

    *pmdRegister &= ~(1u << pmdPosition);
}


//******************************************************************************
/* Function :  POWER_PeripheralModuleIsEnabled_PIC32_1

  Summary:
    Implements PIC32_1 variant of PLIB_POWER_PeripheralModuleIsEnabled 

  Description:
    This template implements the PIC32_1 variant of the PLIB_POWER_PeripheralModuleIsEnabled function.
*/

PLIB_TEMPLATE bool POWER_PeripheralModuleIsEnabled_PIC32_1( POWER_MODULE_ID index , POWER_MODULE source )
{
    volatile uint32_t *pmdRegister = (uint32_t *)(&PMD1 + ((source/32u) * 4u));
    uint8_t pmdPosition = source % 32u;

    return (((*pmdRegister) & (1u << pmdPosition)) ? false : true);
}


#endif /*_POWER_PERIPHERALMODULECONTROL_PIC32_1_H*/

/******************************************************************************
 End of File
*/

