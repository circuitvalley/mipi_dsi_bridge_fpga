/*******************************************************************************
  USB CDC class driver interface header

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_cdc_local.h

  Summary:
    USB CDC class driver interface header

  Description:
    USB CDC class driver interface header
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
// DOM-IGNORE-END

#ifndef _USB_DEVICE_CDC_LOCAL_H
#define _USB_DEVICE_CDC_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "system/common/sys_common.h"
#include "system/common/sys_module.h"
#include "usb/usb_common.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"
#include "osal/osal.h"


// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define USB_DEVICE_CDC_ENDPOINT_RX          USB_DATA_DIRECTION_HOST_TO_DEVICE 
#define USB_DEVICE_CDC_ENDPOINT_TX          USB_DATA_DIRECTION_DEVICE_TO_HOST

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* CDC endpoint instance.

  Summary:
    Identifies the CDC endpoint instance.

  Description:
    This type identifies the CDC endpoint instance.

  Remarks:
    This structure is internal to the CDC function driver.
*/
typedef struct
{
    /* End point address */
    uint8_t address;

    /* End point maximum payload */
    uint16_t maxPacketSize;

    bool    isConfigured;

}USB_DEVICE_CDC_ENDPOINT;


// *****************************************************************************
/* CDC interface instance.

  Summary:
    Identifies the CDC interface instance.

  Description:
    This type identifies the CDC interface instance. CDC can have up to two
    interfaces.

  Remarks:
    This structure is internal to the CDC function driver.
*/
typedef struct
{

    /* interface number */
    uint8_t interfaceNum;

    /* end points associated with this interface */
    USB_DEVICE_CDC_ENDPOINT endpoint[2];

}USB_DEVICE_CDC_INTERFACE;

// *****************************************************************************
/* CDC instance structure.

  Summary:
    Identifies the CDC instance.

  Description:
    This type identifies the CDC instance.

  Remarks:
    This structure is internal to the CDC function driver.
*/
typedef struct
{
    /*  */
    USB_DEVICE_HANDLE deviceHandle;

    /* data interface */
    USB_DEVICE_CDC_INTERFACE dataInterface;

    /* notification interface */
    USB_DEVICE_CDC_INTERFACE notificationInterface;

    /* Application callback */
    USB_DEVICE_CDC_EVENT_HANDLER appEventCallBack;

    /* Application user data */
    uintptr_t userData;

    /* Transmit Queue Size*/
    unsigned int queueSizeWrite;

    /* Receive Queue Size */
    unsigned int queueSizeRead;

    /* Serial State Notification Queue Size */
    unsigned int queueSizeSerialStateNotification;

    /* Current Queue Size*/
    volatile unsigned int currentQSizeWrite;
    volatile unsigned int currentQSizeRead;
    volatile unsigned int currentQSizeSerialStateNotification;

} USB_DEVICE_CDC_INSTANCE;

// *****************************************************************************
/* CDC Common data object

  Summary:
    Object used to keep track of data that is common to all instances of the
    CDC function driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the CDC function driver.

  Remarks:
    None.
*/
typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool isMutexCdcIrpInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexCDCIRP);

} USB_DEVICE_CDC_COMMON_DATA_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: CDC specific functions
// *****************************************************************************
// *****************************************************************************


//******************************************************************************
/* Function:
    static void _USB_DEVICE_CDC_ControlTransferHandler
    (
        SYS_MODULE_INDEX iCDC ,
        USB_DEVICE_CONTROL_TRANSFER_EVENT controlTransferEvent,
        void * controlTransferEventData
    );
 
  Summary:
    Handles end-point 0 requests.

  Description:
    This function handles ep0 requests.

  Remarks:
    Called by the device layer.
 */

void _USB_DEVICE_CDC_ControlTransferHandler
(
    SYS_MODULE_INDEX iCDC ,
    USB_DEVICE_EVENT controlTransferEvent,
    USB_SETUP_PACKET * setupPacket
);


//******************************************************************************
/* Function:
    void USB_DEVICE_CDC_Initialization ( SYS_MODULE_INDEX iCDC ,
                                     DRV_HANDLE deviceHandle ,
                                     void* funcDriverInitData ,
                                     uint8_t infNum ,
                                     uint8_t altSetting ,
                                     uint8_t descType ,
                                     uint8_t * pDesc )
  Summary:
    CDC device class init function.

  Description:
    This function handles cdc initialization.

  Remarks:
    Called by the device layer per instance.
 */

void _USB_DEVICE_CDC_Initialization
(
    SYS_MODULE_INDEX iCDC ,
    DRV_HANDLE deviceHandle ,
    void* funcDriverInitData ,
    uint8_t infNum ,
    uint8_t altSetting ,
    uint8_t descType ,
    uint8_t * pDesc 
);


// *****************************************************************************

/* Function:
    void	USB_DEVICE_CDC_Deinitialization (SYS_MODULE_INDEX iCDC)

  Summary:
    CDC function driver deinitialization.

  Description:
    This function deinitializes the specified instance of the CDC function driver.
    This function is called by the USB device layer.

  Precondition:
    None.

  Parameters:
    iCDC	- USB function driver index

  Returns:
    None.

  Example:
    <code>
    // Called by the device layer.
    </code>

  Remarks:
    This function is internal to the USB stack. This API should not be
    called explicitly.
 */

void _USB_DEVICE_CDC_Deinitialization ( SYS_MODULE_INDEX iCDC );

// ******************************************************************************
/* Function:
    void _USB_DEVICE_CDC_GlobalInitialize ( void )

  Summary:
    This function initializes resourses required common to all instances of CDC
    function driver.

 Description:
    This function initializes resourses common to all instances of CDC function
    driver. This function is called by the USB Device layer during Initalization.

 Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Example:
    <code>
    // Called by the device layer.
    </code>

  Remarks:
    This is local function and should not be called directly by the application.
*/
void _USB_DEVICE_CDC_GlobalInitialize (void);


//******************************************************************************
/* Function:
   void _USB_DEVICE_CDC_ACMSetUpPacketHandler
   (
        SYS_MODULE_INDEX iCDC ,
        USB_DEVICE_CDC_INSTANCE * thisCDCDevice,
        void * controlTransferEventData
    );
 
  Summary:
    Handles ACM sub class specific requests.

  Description:
    This function handles ACM sub class specific requests

  Remarks:
    Called by the CDC function driver.
 */
void _USB_DEVICE_CDC_ACMSetUpPacketHandler
(
    SYS_MODULE_INDEX iCDC ,
    USB_DEVICE_CDC_INSTANCE * thisCDCDevice,
    USB_SETUP_PACKET * controlTransferEventData
);



//******************************************************************************
/* Function:
    void _USB_DEVICE_CDC_WriteIRPCallback (USB_DEVICE_IRP * irp )

  Summary:
    TX data callback.

  Description:
    This function handles TX data events 

  Remarks:
    Called by the controller driver 
 */

void _USB_DEVICE_CDC_WriteIRPCallback (USB_DEVICE_IRP * irp );


//******************************************************************************
/* Function:
    void _USB_DEVICE_CDC_ReadIRPCallback (USB_DEVICE_IRP * irp )

  Summary:
    RX data callback.

  Description:
    This function handles RX data events

  Remarks:
    Called by the controller driver
 */

void _USB_DEVICE_CDC_ReadIRPCallback (USB_DEVICE_IRP * irp );

extern USB_DEVICE_FUNCTION_DRIVER cdcFuncDriver;

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif // _USB_DEVICE_CDC_LOCAL_H

/*******************************************************************************
 End of File
*/


