/**************************************************************************
 USB Device Layer Remote wakeup Timed function implementations.

  Company:
    Microchip Technology Inc.
    
  File Name:
    usb_device_remote_wakeup.c
    
  Summary:
    This file contains implementations of both private and public functions of
    the USB Device Layer remote wakeup Timed feature. Add this file to your
    project only if the application wants to use remote wakeup functions. User
    should also add timer system service to his project in order to use this file.
    
  Description:
    This file contains the USB Device Layer remote wakeup functions
    implementation.
**************************************************************************/

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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "system_config.h"
#include "system/common/sys_module.h"
#include "usb/usb_common.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"
#include "system/debug/sys_debug.h"
#include "usb/src/usb_device_local.h"
#include "system/tmr/sys_tmr.h"


// *****************************************************************************
// *****************************************************************************
// Section: USB Device Layer Remote wakeup Interface functions.
// *****************************************************************************
// *****************************************************************************
// ******************************************************************************
// ******************************************************************************


// *****************************************************************************
/* Function:
    void USB_DEVICE_RemoteWakeupStartTimed ( USB_DEVICE_HANDLE usbDeviceHandle )

  Summary:
    This function will start a self timed Remote Wakeup.

  Description:
    This function will start a self timed Remote Wakeup sequence. The function
    will cause the device to generate resume signaling for 10 milliseconds. The
    resume signaling will stop after 10 milliseconds. The application can use
    this function instead of the USB_DEVICE_RemoteWakeupStart() and
    USB_DEVICE_RemoteWakeupStop() functions which require the application to
    manually start, maintain duration and then stop the resume signaling.

  Precondition:
    Client handle should be valid. The host should have enabled the Remote
    Wakeup feature for this device.

  Parameters:
    usbDeviceHandle    - Client's driver handle (returned from USB_DEVICE_Open)

  Returns:
    None.

  Remarks:
    None.
*/

void USB_DEVICE_RemoteWakeupStartTimed ( USB_DEVICE_HANDLE usbDeviceHandle )
{
    USB_DEVICE_OBJ * usbDeviceThisInstance;

    /* Validate the device handle */
    usbDeviceThisInstance = _USB_DEVICE_ClientHandleValidate(usbDeviceHandle);
    if(NULL == usbDeviceThisInstance)
    {
        SYS_DEBUG(0, "USB Device Layer Client Handle is invalid");
        return;
    }
    
    /*Call USBCD remote wakeup start function. */
    DRV_USB_DEVICE_RemoteWakeupStart(usbDeviceThisInstance->usbCDHandle);
    
    /* Generate 10 Milli Seconds Delay */
    SYS_TMR_CallbackSingle(10,(uintptr_t )usbDeviceThisInstance,
                                        _USB_DEVICE_RemotewakeupTimerCallback );
}

// *****************************************************************************
/* Function:
    void _USB_DEVICE_RemotewakeupTimerCallback(uintptr_t context, uint32_t currTick)

  Summary:
    This function is always invoked by the timer system service when a specified
    delay is completed.

  Description:
    This is a callback function invoked by  Timer system service once a delay
    period is expired. This is a local function and should not be invoked by
    user.

  Parameters:
    context    - passes back user data
    currTick    - The current system tick when the notification is called.
  Returns:
    None.

  Remarks:
    None.
*/
void _USB_DEVICE_RemotewakeupTimerCallback(uintptr_t context, uint32_t currTick)
{
    /* Retrieve USB Device Layer Handle */
    USB_DEVICE_OBJ * usbDeviceThisInstance = (USB_DEVICE_OBJ*)context;
  
    /* Call USBCD remote wakeup stop function. */
    DRV_USB_DEVICE_RemoteWakeupStop(usbDeviceThisInstance->usbCDHandle);

}
/********************End of file********************************/
