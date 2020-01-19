/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_EndpointFIFO_Unsupported.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : EndpointFIFO
    and its Variant : Unsupported
    For following APIs :
        PLIB_USBHS_EndpointFIFOLoad
        PLIB_USBHS_EndpointFIFOUnload
        PLIB_USBHS_Endpoint0FIFOFlush
        PLIB_USBHS_Endpoint0SetupPacketLoad
        PLIB_USBHS_ExistsEndpointFIFO

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

#ifndef _USBHS_ENDPOINTFIFO_UNSUPPORTED_H
#define _USBHS_ENDPOINTFIFO_UNSUPPORTED_H

//******************************************************************************
/* Function :  USBHS_EndpointFIFOLoad_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_EndpointFIFOLoad 

  Description:
    This template implements the Unsupported variant of the
    PLIB_USBHS_EndpointFIFOLoad function.
*/

PLIB_TEMPLATE void USBHS_EndpointFIFOLoad_Unsupported
( 
    USBHS_MODULE_ID index, 
    uint8_t endpoint, 
    void * source, 
    size_t nBytes 
)
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_EndpointFIFOLoad");
}

//******************************************************************************
/* Function :  USBHS_EndpointFIFOUnload_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_EndpointFIFOUnload 

  Description:
    This template implements the Unsupported variant of the
    PLIB_USBHS_EndpointFIFOUnload function.
*/

PLIB_TEMPLATE int USBHS_EndpointFIFOUnload_Unsupported
( 
    USBHS_MODULE_ID index, 
    uint8_t endpoint, 
    void * dest 
)
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_EndpointFIFOUnload");

    return 0;
}

//******************************************************************************
/* Function :  USBHS_Endpoint0FIFOFlush_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_Endpoint0FIFOFlush 

  Description:
    This template implements the Unsupported variant of the
    PLIB_USBHS_Endpoint0FIFOFlush function.
*/

PLIB_TEMPLATE void USBHS_Endpoint0FIFOFlush_Unsupported( USBHS_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_Endpoint0FIFOFlush");
}

//******************************************************************************
/* Function :  USBHS_Endpoint0SetupPacketLoad_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_Endpoint0SetupPacketLoad 

  Description:
    This template implements the Unsupported variant of the
    PLIB_USBHS_Endpoint0SetupPacketLoad function.
*/

PLIB_TEMPLATE void USBHS_Endpoint0SetupPacketLoad_Unsupported
( 
    USBHS_MODULE_ID index, 
    void * setupPacket, 
    uint8_t deviceAddress, 
    uint8_t hubAddress, 
    uint8_t hubPortAddress, 
    USB_SPEED speed
)
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_USBHS_Endpoint0SetupPacketLoad");
}

//******************************************************************************
/* Function :  USBHS_ExistsEndpointFIFO_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_USBHS_ExistsEndpointFIFO

  Description:
    This template implements the Unsupported variant of the
    PLIB_USBHS_ExistsEndpointFIFO function.
*/

PLIB_TEMPLATE bool USBHS_ExistsEndpointFIFO_Unsupported( USBHS_MODULE_ID index )
{
    return false;
}


#endif /*_USBHS_ENDPOINTFIFO_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

