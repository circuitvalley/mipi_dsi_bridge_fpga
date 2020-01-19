/*******************************************************************************
  USB HID Function Driver Local Header file.

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_hid_local.h

  Summary:
    USB HID function driver local header file.
  
  Description:
    This file contains the type, definitions and function prototypes that are
    local to USB Device HID function Driver.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _USB_DEVICE_HID_LOCAL_H_
#define _USB_DEVICE_HID_LOCAL_H_

// *****************************************************************************
// *****************************************************************************
// Section: Local data types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* HID flags.

  Summary:
    Flags for tracking internal status.

  Description:
    Flags for tracking internal status.

  Remarks:
    This structure is internal to the HID function driver.
*/

typedef union _USB_DEVICE_HID_FLAGS
{
    struct
    {
        uint8_t interfaceReady:1;
        uint8_t interruptEpTxReady:1;
        uint8_t interruptEpRxReady:1;
    };
    uint8_t allFlags;

} USB_DEVICE_HID_FLAGS;

// *****************************************************************************
/* HID Instance structure.

  Summary:
    Identifies the HID instance.

  Description:
    This type identifies the HID instance.

  Remarks:
    This structure is internal to the HID function driver.
*/

typedef struct 
{

    USB_DEVICE_HID_FLAGS flags;
    USB_DEVICE_HANDLE devLayerHandle;
    USB_DEVICE_HID_INIT * hidFuncInit;
    USB_DEVICE_HID_EVENT_HANDLER appCallBack;
    uintptr_t userData;
    bool ignoreControlEvents;
    USB_ENDPOINT endpointTx;
    uint16_t endpointTxSize;
    USB_ENDPOINT endpointRx;
    uint16_t endpointRxSize;
    size_t currentTxQueueSize;
    size_t currentRxQueueSize;
    uint8_t *hidDescriptor;

} USB_DEVICE_HID_INSTANCE;

// *****************************************************************************
/* HID Common data object

  Summary:
    Object used to keep track of data that is common to all instances of the
    HID function driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the HID function driver.

  Remarks:
    None.
*/
typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool isMutexHidIrpInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexHIDIRP);

} USB_DEVICE_HID_COMMON_DATA_OBJ;
// *****************************************************************************
// *****************************************************************************
// Section: Local function protoypes.
// *****************************************************************************
// *****************************************************************************

void _USB_DEVICE_HID_InitializeByDescriptorType
(
    SYS_MODULE_INDEX iHID,
    USB_DEVICE_HANDLE usbDeviceHandle,
    void* funcDriverInit,
    uint8_t intfNumber,
    uint8_t altSetting,
    uint8_t descriptorType,
    uint8_t * pDescriptor
);

void _USB_DEVICE_HID_ReportSendCallBack(USB_DEVICE_IRP * irpTx);

void _USB_DEVICE_HID_ReportReceiveCallBack(USB_DEVICE_IRP * irpRx);

void _USB_DEVICE_HID_ControlTransferHandler
(
    SYS_MODULE_INDEX iHID,
    USB_DEVICE_EVENT controlEvent,
    USB_SETUP_PACKET * setupPkt
);

void _USB_DEVICE_HID_DeInitialize(SYS_MODULE_INDEX iHID);

void _USB_DEVICE_HID_GlobalInitialize (void); 

#endif
