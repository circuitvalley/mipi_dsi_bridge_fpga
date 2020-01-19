/*******************************************************************************
  USB Host Audio v1.0 Client Driver mapping

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_audio_v1_mapping.h

  Summary:
    USB Host Audio v1.0 Client Driver mapping

  Description:
    This file contain mapppings required for the USB Host Audio v1.0 Client driver
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _USB_HOST_AUDIO_V1_MAPPING_H
#define _USB_HOST_AUDIO_V1_MAPPING_H

#include "system_config.h"
#include "usb/src/usb_host_audio_local.h"

#if defined (USB_HOST_AUDIO_V1_0_INSTANCES_NUMBER) && !defined (USB_HOST_AUDIO_V1_INSTANCES_NUMBER)
 #define USB_HOST_AUDIO_V1_INSTANCES_NUMBER USB_HOST_AUDIO_V1_0_INSTANCES_NUMBER
 #endif 

#if defined (USB_HOST_AUDIO_V1_0_STREAMING_INTERFACES_NUMBER) && !defined (USB_HOST_AUDIO_V1_STREAMING_INTERFACES_NUMBER)
 #define USB_HOST_AUDIO_V1_STREAMING_INTERFACES_NUMBER USB_HOST_AUDIO_V1_0_STREAMING_INTERFACES_NUMBER
 #endif 

#if defined (USB_HOST_AUDIO_V1_0_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER) && !defined (USB_HOST_AUDIO_V1_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER)
 #define USB_HOST_AUDIO_V1_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER USB_HOST_AUDIO_V1_0_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER + 1
 #endif 


#define USB_HOST_AUDIO_V1_0_INSTANCE  USB_HOST_AUDIO_V1_INSTANCE
#define _USB_HOST_AUDIO_V1_0_ControlRequestCallback _USB_HOST_AUDIO_V1_ControlRequestCallback

#define USB_HOST_AUDIO_V1_StreamWrite(handle, transferHandle, source, length)  _USB_HOST_AUDIO_V1_StreamWrite(handle, transferHandle, source, length, USB_HOST_AUDIO_V1_API_VERSION_FLAG_V1)
#define USB_HOST_AUDIO_V1_0_StreamWrite(handle, transferHandle, source, length)  _USB_HOST_AUDIO_V1_StreamWrite(handle, transferHandle, source, length, USB_HOST_AUDIO_V1_API_VERSION_FLAG_V1_0_DEPRECIATED)
#define USB_HOST_AUDIO_V1_StreamRead(handle, transferHandle, source, length)  _USB_HOST_AUDIO_V1_StreamRead(handle, transferHandle, source, length, USB_HOST_AUDIO_V1_API_VERSION_FLAG_V1)
#define USB_HOST_AUDIO_V1_0_StreamRead(handle, transferHandle, source, length)  _USB_HOST_AUDIO_V1_StreamRead(handle, transferHandle, source, length, USB_HOST_AUDIO_V1_API_VERSION_FLAG_V1_0_DEPRECIATED)
#endif
