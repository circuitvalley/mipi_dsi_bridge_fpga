/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_USBIDControl_Unsupported.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : USBIDControl
    and its Variant : Unsupported
    For following APIs :
        PLIB_USBHS_ExistsUSBIDControl
        PLIB_USBHS_USBIDOverrideEnable
        PLIB_USBHS_USBIDOverrideDisable
        PLIB_USBHS_USBIDOverrideValueSet
        PLIB_USBHS_PhyIDMonitoringEnable
        PLIB_USBHS_PhyIDMonitoringDisable

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

#ifndef _USBHS_USBIDCONTROL_UNSUPPORTED_H
#define _USBHS_USBIDCONTROL_UNSUPPORTED_H

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
/* Function :  USBHS_ExistsUSBIDControl_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ExistsUSBIDControl

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ExistsUSBIDControl function.
*/

PLIB_TEMPLATE bool USBHS_ExistsUSBIDControl_Unsupported( USBHS_MODULE_ID index )
{
    return false;
}


//******************************************************************************
/* Function :  USBHS_USBIDOverrideEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_USBIDOverrideEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_USBIDOverrideEnable function.
*/

PLIB_TEMPLATE void USBHS_USBIDOverrideEnable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_USBIDOverrideEnable");
}


//******************************************************************************
/* Function :  USBHS_USBIDOverrideDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_USBIDOverrideDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_USBIDOverrideDisable function.
*/

PLIB_TEMPLATE void USBHS_USBIDOverrideDisable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_USBIDOverrideDisable");
}


//******************************************************************************
/* Function :  USBHS_USBIDOverrideValueSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_USBIDOverrideValueSet 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_USBIDOverrideValueSet function.
*/

PLIB_TEMPLATE void USBHS_USBIDOverrideValueSet_Unsupported( USBHS_MODULE_ID index , USBHS_USBID_OVERRIDE_VALUE id )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_USBIDOverrideValueSet");
}


//******************************************************************************
/* Function :  USBHS_PhyIDMonitoringEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_PhyIDMonitoringEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_PhyIDMonitoringEnable function.
*/

PLIB_TEMPLATE void USBHS_PhyIDMonitoringEnable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_PhyIDMonitoringEnable");
}


//******************************************************************************
/* Function :  USBHS_PhyIDMonitoringDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_PhyIDMonitoringDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_PhyIDMonitoringDisable function.
*/

PLIB_TEMPLATE void USBHS_PhyIDMonitoringDisable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_PhyIDMonitoringDisable");
}


#endif /*_USBHS_USBIDCONTROL_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

