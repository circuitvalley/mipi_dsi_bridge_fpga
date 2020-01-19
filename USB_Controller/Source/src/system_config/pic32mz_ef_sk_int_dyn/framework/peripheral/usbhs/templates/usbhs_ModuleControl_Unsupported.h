/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_ModuleControl_Unsupported.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ModuleControl
    and its Variant : Unsupported
    For following APIs :
        PLIB_USBHS_ResumeEnable
        PLIB_USBHS_ResumeDisable
        PLIB_USBHS_SuspendEnable
        PLIB_USBHS_SuspendDisable
        PLIB_USBHS_ResetEnable
        PLIB_USBHS_ResetDisable
        PLIB_USBHS_VBUSLevelGet
        PLIB_USBHS_HostModeIsEnabled
        PLIB_USBHS_IsBDevice
        PLIB_USBHS_SessionEnable
        PLIB_USBHS_SessionDisable
        PLIB_USBHS_DeviceAddressSet
        PLIB_USBHS_DeviceAttach
        PLIB_USBHS_DeviceDetach
        PLIB_USBHS_ExistsModuleControl

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

#ifndef _USBHS_MODULECONTROL_UNSUPPORTED_H
#define _USBHS_MODULECONTROL_UNSUPPORTED_H

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
/* Function :  USBHS_ResumeEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ResumeEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ResumeEnable function.
*/

PLIB_TEMPLATE void USBHS_ResumeEnable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_ResumeEnable");
}


//******************************************************************************
/* Function :  USBHS_ResumeDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ResumeDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ResumeDisable function.
*/

PLIB_TEMPLATE void USBHS_ResumeDisable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_ResumeDisable");
}


//******************************************************************************
/* Function :  USBHS_SuspendEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_SuspendEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_SuspendEnable function.
*/

PLIB_TEMPLATE void USBHS_SuspendEnable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_SuspendEnable");
}


//******************************************************************************
/* Function :  USBHS_SuspendDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_SuspendDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_SuspendDisable function.
*/

PLIB_TEMPLATE void USBHS_SuspendDisable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_SuspendDisable");
}


//******************************************************************************
/* Function :  USBHS_ResetEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ResetEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ResetEnable function.
*/

PLIB_TEMPLATE void USBHS_ResetEnable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_ResetEnable");
}


//******************************************************************************
/* Function :  USBHS_ResetDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ResetDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ResetDisable function.
*/

PLIB_TEMPLATE void USBHS_ResetDisable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_ResetDisable");
}


//******************************************************************************
/* Function :  USBHS_VBUSLevelGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_VBUSLevelGet 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_VBUSLevelGet function.
*/

PLIB_TEMPLATE USBHS_VBUS_LEVEL USBHS_VBUSLevelGet_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_VBUSLevelGet");

    return (USBHS_VBUS_LEVEL) 0;
}


//******************************************************************************
/* Function :  USBHS_HostModeIsEnabled_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_HostModeIsEnabled 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_HostModeIsEnabled function.
*/

PLIB_TEMPLATE bool USBHS_HostModeIsEnabled_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_HostModeIsEnabled");

    return false;
}


//******************************************************************************
/* Function :  USBHS_IsBDevice_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_IsBDevice 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_IsBDevice function.
*/

PLIB_TEMPLATE bool USBHS_IsBDevice_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_IsBDevice");

    return false;
}


//******************************************************************************
/* Function :  USBHS_SessionEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_SessionEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_SessionEnable function.
*/

PLIB_TEMPLATE void USBHS_SessionEnable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_SessionEnable");
}


//******************************************************************************
/* Function :  USBHS_SessionDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_SessionDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_SessionDisable function.
*/

PLIB_TEMPLATE void USBHS_SessionDisable_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_SessionDisable");
}


//******************************************************************************
/* Function :  USBHS_DeviceAddressSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_DeviceAddressSet 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_DeviceAddressSet function.
*/

PLIB_TEMPLATE void USBHS_DeviceAddressSet_Unsupported( USBHS_MODULE_ID index , uint8_t address )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_DeviceAddressSet");
}


//******************************************************************************
/* Function :  USBHS_DeviceAttach_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_DeviceAttach 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_DeviceAttach function.
*/

PLIB_TEMPLATE void USBHS_DeviceAttach_Unsupported( USBHS_MODULE_ID index , uint32_t speed )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_DeviceAttach");
}


//******************************************************************************
/* Function :  USBHS_DeviceDetach_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_DeviceDetach 

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_DeviceDetach function.
*/

PLIB_TEMPLATE void USBHS_DeviceDetach_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_DeviceDetach");
}


//******************************************************************************
/* Function :  USBHS_ExistsModuleControl_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ExistsModuleControl

  Description:
    This template implements the Unsupported variant of the PLIB_USBHS_ExistsModuleControl function.
*/

PLIB_TEMPLATE bool USBHS_ExistsModuleControl_Unsupported( USBHS_MODULE_ID index )
{
    return false;
}


#endif /*_USBHS_MODULECONTROL_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

