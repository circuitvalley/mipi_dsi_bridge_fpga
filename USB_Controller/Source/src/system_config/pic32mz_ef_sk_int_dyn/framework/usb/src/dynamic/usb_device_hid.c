/*******************************************************************************
  USB HID Function Driver

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_hid.c

  Summary:
    USB HID function driver
  
  Description:
    This file contains the implementation of the USB Device HID Function Driver.
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "usb/usb_device_hid.h"
#include "usb/usb_device.h"
#include "system/common/sys_common.h"
#include "usb/src/usb_device_hid_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************

/**************************************
 * Allocate a global pool of IRPs
 **************************************/
USB_DEVICE_IRP gUSBDeviceHIDIRP[USB_DEVICE_HID_QUEUE_DEPTH_COMBINED];

/* Create a variable for holding HID IRP mutex Handle and status */
USB_DEVICE_HID_COMMON_DATA_OBJ gUSBDeviceHidCommonDataObj;
// *****************************************************************************
/* HID Device function driver function structure

  Summary:
    Defines the function driver structure required by the device layer.

  Description:
    This data type defines the function driver structure required by the
    device layer

  Remarks:
    This structure is private to the USB stack.
*/
const USB_DEVICE_FUNCTION_DRIVER hidFuncDriver =
{
    
    /* HID init function */
    .initializeByDescriptor = &_USB_DEVICE_HID_InitializeByDescriptorType,

    /* HID de-init function */
    .deInitialize           = &_USB_DEVICE_HID_DeInitialize,
		
     /* EP0 activity callback */
    .controlTransferNotification = &_USB_DEVICE_HID_ControlTransferHandler,

    /* HID tasks function */
    .tasks                  = NULL,

     /* HID Global Initialize */
    .globalInitialize = _USB_DEVICE_HID_GlobalInitialize
};

/***************************************
 * Array of USB HID Instances
 ****************************************/
USB_DEVICE_HID_INSTANCE gUsbDeviceHidInstance[USB_DEVICE_HID_INSTANCES_NUMBER];
// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************


// ******************************************************************************
/* Function:
    void _USB_DEVICE_HID_GlobalInitialize ( void )

  Summary:
    This function initializes resourses required common to all instances of CDC
    function driver.

  Description:
    This function initializes resourses common to all instances of CDC function
    driver. This function is called by the USB Device layer during Initalization.

  Remarks:
    This is local function and should not be called directly by the application.
*/
void _USB_DEVICE_HID_GlobalInitialize (void)
{
    OSAL_RESULT osal_err;
    
    /* Create Mutex for CDC IRP objects if not created already */
    if (gUSBDeviceHidCommonDataObj.isMutexHidIrpInitialized == false)
    {
        /* This means that mutexes where not created. Create them. */
        osal_err = OSAL_MUTEX_Create(&gUSBDeviceHidCommonDataObj.mutexHIDIRP);

        if(osal_err != OSAL_RESULT_TRUE)
        {
            /*do not proceed lock was not created, let user know about error*/
            return;
        }

         /* Set this flag so that global mutexes get allocated only once */
         gUSBDeviceHidCommonDataObj.isMutexHidIrpInitialized = true;
    }
}
// ******************************************************************************
/* Function:
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

  Summary:
    This function will initialize the HID function driver by interface.

  Description:
    This function will initialize the HID function driver by interface.

  Remarks:
    This is a local function and should not be called directly by the application.
*/

void _USB_DEVICE_HID_InitializeByDescriptorType
(
    SYS_MODULE_INDEX iHID,
    USB_DEVICE_HANDLE usbDeviceHandle,
    void* funcDriverInit,
    uint8_t intfNumber,
    uint8_t altSetting,
    uint8_t descriptorType,
    uint8_t * pDescriptor
)
{
    /* This function is called by the device layer when it
     * initializes the HID function driver. The device layer
     * passes descriptors to this function and the function
     * driver uses these to initialize itself */

    USB_ENDPOINT_DESCRIPTOR * epDescriptor = ( USB_ENDPOINT_DESCRIPTOR *)pDescriptor;
    USB_DEVICE_HID_INSTANCE * hidInstance = &gUsbDeviceHidInstance[iHID];

    SYS_ASSERT(altSetting == 0, "USB Device HID: HID supports only one setting \
            and does not support alternate settings.\
            Check configuration descriptors ");

    switch(descriptorType )
    {
        case USB_DESCRIPTOR_ENDPOINT:
            /* Device layer has configured and opened an endpoint.
               We just have to save the endpoint and arm if necessary.*/

            if( epDescriptor->transferType == USB_TRANSFER_TYPE_INTERRUPT )
            {
                if(epDescriptor->dirn == USB_DATA_DIRECTION_DEVICE_TO_HOST)
                {
                    /* Save the Tx endpoint information. */
                    hidInstance->endpointTx = epDescriptor->bEndpointAddress;
                    hidInstance->endpointTxSize =  epDescriptor->wMaxPacketSize;

                    /* Open the endpoint. */
                    USB_DEVICE_EndpointEnable(usbDeviceHandle,0,
                            hidInstance->endpointTx, USB_TRANSFER_TYPE_INTERRUPT, epDescriptor->wMaxPacketSize);

                    /* Indicate that TX endpoint is ready. */
                    hidInstance->flags.interruptEpTxReady = 1;

                    /* Initialize the current TX queue size. */
                    hidInstance->currentTxQueueSize = 0;
                }
                else
                {
                    /* Direction is OUT */
                    hidInstance->endpointRx = epDescriptor->bEndpointAddress;
                    hidInstance->endpointRxSize =  epDescriptor->wMaxPacketSize;

                    /* Open the endpoint. */
                    USB_DEVICE_EndpointEnable(usbDeviceHandle,0,
                            hidInstance->endpointRx, USB_TRANSFER_TYPE_INTERRUPT, epDescriptor->wMaxPacketSize);

                    /* Indicate that the RX endpoint is ready. */
                    hidInstance->flags.interruptEpRxReady = 1;

                    /* Initialize the current RX queue size. */
                    hidInstance->currentRxQueueSize = 0;
                }
            }
            else
            {
                SYS_ASSERT( false, "USB DEVICE HID: HID does not support \
                        anything other than interrupt endpoints.\
                        Please check the descriptors.");
            }
            break;

        case USB_DESCRIPTOR_INTERFACE:

            /* Just mark interface as ready. */
            hidInstance->flags.interfaceReady = 1;
            hidInstance->devLayerHandle = usbDeviceHandle;
            hidInstance->hidFuncInit = funcDriverInit;
            hidInstance->hidDescriptor = pDescriptor +9 ;

            break;

        default:
            break;
    }
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_HID_ReportSendCallBack(USB_DEVICE_IRP * handle)

  Summary:
    Callback function that gets called after the report is sent
    to host.

  Description:
    This callback forwards the event to application callback function.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_DEVICE_HID_ReportSendCallBack(USB_DEVICE_IRP * irpTx)
{
    /* This function is called when a Report Send IRP has
     * terminated */
    USB_DEVICE_HID_INDEX iHID = irpTx->userData;
    USB_DEVICE_HID_EVENT_DATA_REPORT_SENT reportSentData;
    USB_DEVICE_HID_INSTANCE * thisHIDInstance = &gUsbDeviceHidInstance[iHID];

    /* Update the current transmit queue size */
    thisHIDInstance->currentTxQueueSize --;

    /* Check if a event handler callback is registered and
     * send the event*/
    if(thisHIDInstance->appCallBack != NULL)
    {
        reportSentData.length = irpTx->size;
        reportSentData.handle = ( USB_DEVICE_HID_TRANSFER_HANDLE )irpTx;
        
        /* Get transfer status */
        if ((irpTx->status == USB_DEVICE_IRP_STATUS_COMPLETED) 
            || (irpTx->status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
        {
            /* Transfer completed successfully */
            reportSentData.status = USB_DEVICE_HID_RESULT_OK; 
        }
        else if (irpTx->status == USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT)
        {
            /* Transfer cancelled due to Endpoint Halt */
            reportSentData.status = USB_DEVICE_HID_RESULT_ERROR_ENDPOINT_HALTED; 
        }
        else if (irpTx->status == USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST)
        {
            /* Transfer Cancelled by Host (Host sent a Clear feature )*/
            reportSentData.status = USB_DEVICE_HID_RESULT_ERROR_TERMINATED_BY_HOST; 
        }
        else
        {
            /* Transfer was not completed successfully */
            reportSentData.status = USB_DEVICE_HID_RESULT_ERROR; 
        }

        thisHIDInstance->appCallBack
        (
            iHID,
            USB_DEVICE_HID_EVENT_REPORT_SENT, 
            &reportSentData,
            thisHIDInstance->userData
        );
    }    
}

// ******************************************************************************
/* Function:
    USB_DEVICE_HID_RESULT USB_DEVICE_HID_ReportSend
    (   
        USB_DEVICE_HID_INDEX iHID,
        USB_DEVICE_HID_TRANSFER_HANDLE * transferHandle,
        uint8_t * buffer, 
        size_t size
    )

  Summary:
    This function submits the application given buffer to HID function driver
    to send the report from device to host.

  Description:
    This function submits the application given buffer to HID function driver
    to send the report from device to host.
    
  Remarks:
    Refer to usb_device_hid.h for usage information.
*/

USB_DEVICE_HID_RESULT USB_DEVICE_HID_ReportSend
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_TRANSFER_HANDLE * transferHandle,
    void * buffer, 
    size_t size
)
{
    size_t count;
    USB_DEVICE_IRP * irp = NULL;
    USB_DEVICE_HID_INSTANCE * thisHIDInstance;
    USB_ERROR hidSendError;
    OSAL_RESULT osalError;
	OSAL_CRITSECT_DATA_TYPE status;
    
    /* Set the transfer handle to invalid */
    *transferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;

    /* Check if we have a valid instance index */
    if( (iHID < 0) || ( iHID > USB_DEVICE_HID_INSTANCES_NUMBER) )
    {
        SYS_ASSERT(false, "HID instance is not valid");
        return USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID ;
    }

    /* Create a local reference */
    thisHIDInstance = &gUsbDeviceHidInstance[iHID];

    /* Check if the endpoint was configured */
    if(!thisHIDInstance->flags.interruptEpTxReady )
    {
        SYS_ASSERT(false, "HID Transmit endpoint not configured");
        return USB_DEVICE_HID_RESULT_ERROR_INSTANCE_NOT_CONFIGURED;
    }

    /* Check if the Transmit queue is full */
    if(thisHIDInstance->currentTxQueueSize >= 
            thisHIDInstance->hidFuncInit->queueSizeReportSend)
    {
        SYS_ASSERT(false,"Transmit Queue is full");
        return USB_DEVICE_HID_RESULT_ERROR_TRANSFER_QUEUE_FULL;
    }

    /*Obtain mutex to get access to a shared resource, check return value*/
    osalError = OSAL_MUTEX_Lock(&gUSBDeviceHidCommonDataObj.mutexHIDIRP, OSAL_WAIT_FOREVER);
    if(osalError != OSAL_RESULT_TRUE)
    {
      /*Do not proceed lock was not obtained, or error occurred, let user know about error*/
      return (USB_DEVICE_HID_RESULT_ERROR);
    }

    // Check which IRP is free
    for(count = 0; count < USB_DEVICE_HID_QUEUE_DEPTH_COMBINED; count++)
    {
         if(gUSBDeviceHIDIRP[count].status
                 <= USB_DEVICE_IRP_STATUS_COMPLETED_SHORT)
         {
             /* This means we have free IRP. Populate
              * the IRP */
        
             irp =&gUSBDeviceHIDIRP[count];
             irp->size = size;
             irp->data = buffer;
             irp->callback = &_USB_DEVICE_HID_ReportSendCallBack;
             irp->userData = iHID;
             (*transferHandle) = ( USB_DEVICE_HID_TRANSFER_HANDLE )irp;
			 status = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
             thisHIDInstance->currentTxQueueSize ++;
			 OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, status);

             /* Submit the IRP and return */
             hidSendError = USB_DEVICE_IRPSubmit( thisHIDInstance->devLayerHandle,
                                  thisHIDInstance->endpointTx,
                                  irp);

             /* If IRP Submit function returned any error, then invalidate the
               Transfer handle.  */
            if (hidSendError != USB_ERROR_NONE )
            {
				status = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
                thisHIDInstance->currentTxQueueSize --;
				OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, status);
                *transferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
            }

             /*Release mutex, done with shared resource*/
            osalError = OSAL_MUTEX_Unlock(&gUSBDeviceHidCommonDataObj.mutexHIDIRP);
            if(osalError != OSAL_RESULT_TRUE)
            {
                /*Do not proceed, unlock was not complete, or error occurred, let user know about error*/
                return (USB_DEVICE_HID_RESULT_ERROR);
            }
            return hidSendError;

         }
    }
    /*Release mutex, done with shared resource*/
    osalError = OSAL_MUTEX_Unlock(&gUSBDeviceHidCommonDataObj.mutexHIDIRP);
    if(osalError != OSAL_RESULT_TRUE)
    {
	/*Do not proceed, unlock was not complete, or error occurred, let user know about error*/
	return (USB_DEVICE_HID_RESULT_ERROR);
    }
    /* We could not find a free IRP */
    SYS_ASSERT(false,"Transmit Queue is full");
    return USB_DEVICE_HID_RESULT_ERROR_TRANSFER_QUEUE_FULL;
}

// ******************************************************************************
/* Function:
    USB_DEVICE_HID_RESULT USB_DEVICE_HID_TransferCancel
    (
        USB_DEVICE_HID_INDEX iHID,
        USB_DEVICE_HID_TRANSFER_HANDLE transferHandle
    )

  Summary:
    This function cancels the HID transfer which has been submitted before.

  Description:
    This function cancels the HID transfer which has been submitted before.
    
  Remarks:
    Refer to usb_device_hid.h for usage information.
*/

USB_DEVICE_HID_RESULT USB_DEVICE_HID_TransferCancel
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_TRANSFER_HANDLE transferHandle
)
{
    /* Start of local variables */
    size_t count = 0;
    USB_DEVICE_HID_RESULT returnValue = USB_DEVICE_HID_RESULT_ERROR;
    USB_DEVICE_HID_INSTANCE * hidInstance = NULL;
    USB_ERROR irpCancelResult = USB_ERROR_NONE;
    /* End of local variables */
    
    /* Check if we have a valid instance index */
    if( (iHID < 0) || ( iHID >= USB_DEVICE_HID_INSTANCES_NUMBER) )
    {
        SYS_ASSERT(false, "HID instance is not valid");
        returnValue = USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID;
    }
    else
    {
        hidInstance = &gUsbDeviceHidInstance[iHID];
        
        for(count = 0; count < USB_DEVICE_HID_QUEUE_DEPTH_COMBINED; count++)
        {
            if((transferHandle) ==
                    (USB_DEVICE_HID_TRANSFER_HANDLE)&gUSBDeviceHIDIRP[count])
            {
                /* Found the transfer to cancel */
                returnValue = USB_DEVICE_HID_RESULT_OK;

                irpCancelResult = USB_DEVICE_IRPCancel(hidInstance->devLayerHandle,
                        (USB_DEVICE_IRP *)&gUSBDeviceHIDIRP[count]);

                if (irpCancelResult != USB_ERROR_NONE )
                {
                    returnValue = USB_DEVICE_HID_RESULT_ERROR;
                }
                break;
            }
        }

        if(count == USB_DEVICE_HID_QUEUE_DEPTH_COMBINED)
        {
            /* HID function driver does not own this Transfer Handle.
             * The input parameter was invalid */
            returnValue = USB_DEVICE_HID_RESULT_ERROR_PARAMETER_INVALID;
        }
    }
    
    /*
     * USB_DEVICE_HID_RESULT_OK : On success
     * USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID: Invalid HID instance
     * USB_DEVICE_HID_RESULT_ERROR_PARAMETER_INVALID : Transfer Handle not found
     * USB_DEVICE_HID_RESULT_ERROR : On other errors
     */
    return returnValue;
    
} /* End of USB_DEVICE_HID_TransferCancel() */

// ******************************************************************************
/* Function:
    void _USB_DEVICE_HID_ReportReceiveCallBack(void * handle)

  Summary:
    Callback function that gets called after the report is received
    from host.

  Description:
    This callback forwards the event to application callback function.

  Remarks:
    This is a local function and should not be called directly by the application.
*/

void _USB_DEVICE_HID_ReportReceiveCallBack(USB_DEVICE_IRP * irpRx)
{
    SYS_MODULE_INDEX iHID = irpRx->userData;
    USB_DEVICE_HID_INSTANCE * thisHIDInstance = &gUsbDeviceHidInstance[iHID];
    USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED reportReceivedData;

    /* Update the receive queue size */
    thisHIDInstance->currentRxQueueSize --;

    /* Check if an application event handler callback is
     * avaialable and then send the event to the application. */
    if(thisHIDInstance->appCallBack)
    {
        reportReceivedData.length = irpRx->size;
        reportReceivedData.handle =
                (USB_DEVICE_HID_TRANSFER_HANDLE)irpRx;
                
        /* Get transfer status */
        if ((irpRx->status == USB_DEVICE_IRP_STATUS_COMPLETED) 
            || (irpRx->status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
        {
            /* Transfer completed successfully */
            reportReceivedData.status = USB_DEVICE_HID_RESULT_OK; 
        }
        else if (irpRx->status == USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT)
        {
            /* Transfer cancelled due to Endpoint Halt */
            reportReceivedData.status = USB_DEVICE_HID_RESULT_ERROR_ENDPOINT_HALTED; 
        }
        else if (irpRx->status == USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST)
        {
            /* Transfer Cancelled by Host (Host sent a Clear feature )*/
            reportReceivedData.status = USB_DEVICE_HID_RESULT_ERROR_TERMINATED_BY_HOST; 
        }
        else
        {
            /* Transfer was not completed successfully */
            reportReceivedData.status = USB_DEVICE_HID_RESULT_ERROR; 
        }

        thisHIDInstance->appCallBack
        (
            iHID,
            USB_DEVICE_HID_EVENT_REPORT_RECEIVED,
            &reportReceivedData,
            thisHIDInstance->userData 
        );
    }
}

// ******************************************************************************
/* Function:
    USB_DEVICE_HID_RESULT USB_DEVICE_HID_ReportReceive
    (
        USB_DEVICE_HID_INDEX iHID,
        USB_DEVICE_HID_TRANSFER_HANDLE * transferHandle,
        void * buffer, 
        size_t size
    )

  Summary:
    This function submits the buffer to HID function driver to receive a report
    from host.

  Description:
    This function submits the buffer to HID function driver to receive a report
    from host.

  Remarks:
    Refer to usb_device_hid.h for usage information.
*/

USB_DEVICE_HID_RESULT USB_DEVICE_HID_ReportReceive
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_TRANSFER_HANDLE * transferHandle,
    void * buffer, 
    size_t size
)
{

    size_t count = 0;
    USB_DEVICE_IRP * irp;
    USB_DEVICE_HID_INSTANCE * thisHIDInstance;
    USB_ERROR hidReceiveError;
    OSAL_RESULT osalError; 
	OSAL_CRITSECT_DATA_TYPE status;

    /* Set the transfer handle to invalid */
    *transferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;

    /* Check if we have a valid instance index */
    if( (iHID < 0) || ( iHID > USB_DEVICE_HID_INSTANCES_NUMBER) )
    {
        SYS_ASSERT(false, "HID instance is not valid");
        return USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID ;
    }

    /* Create a local reference */
    thisHIDInstance = &gUsbDeviceHidInstance[iHID];

    /* Check if the endpoint was configured */
    if(!thisHIDInstance->flags.interruptEpRxReady )
    {
        SYS_ASSERT(false, "HID Receive endpoint not configured");
        return USB_DEVICE_HID_RESULT_ERROR_INSTANCE_NOT_CONFIGURED;
    }

    /* Check if the Receive queue is full */
    if(thisHIDInstance->currentRxQueueSize >=
            thisHIDInstance->hidFuncInit->queueSizeReportReceive)
    {
        SYS_ASSERT(false,"Receive Queue is full");
        return USB_DEVICE_HID_RESULT_ERROR_TRANSFER_QUEUE_FULL;
    }

    /*Obtain mutex to get access to a shared resource, check return value*/
    osalError = OSAL_MUTEX_Lock(&gUSBDeviceHidCommonDataObj.mutexHIDIRP, OSAL_WAIT_FOREVER);
    if(osalError != OSAL_RESULT_TRUE)
    {
      /*Do not proceed lock was not obtained, or error occurred, let user know about error*/
      return (USB_DEVICE_HID_RESULT_ERROR);
    }

    /* Search for a free IRP*/
    for(count = 0; count < USB_DEVICE_HID_QUEUE_DEPTH_COMBINED; count++)
    {
         if( gUSBDeviceHIDIRP[count].status <=
                 USB_DEVICE_IRP_STATUS_COMPLETED_SHORT)
         {
             /* We have found a free IRP. Populate the IRP
              * and then submit it*/
             irp = &gUSBDeviceHIDIRP[count];
             irp->size = size;
             irp->callback = &_USB_DEVICE_HID_ReportReceiveCallBack;
             irp->data = buffer;
             irp->userData = iHID;
			 status = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
             thisHIDInstance->currentRxQueueSize ++;
			 OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, status);
             (* transferHandle) = ( USB_DEVICE_HID_TRANSFER_HANDLE )irp;
             hidReceiveError = USB_DEVICE_IRPSubmit( thisHIDInstance->devLayerHandle,
                                           thisHIDInstance->endpointRx,
                                           irp);

             /* If IRP Submit function returned any error, then invalidate the
               Transfer handle.  */
            if (hidReceiveError != USB_ERROR_NONE )
            {
				status = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
                thisHIDInstance->currentRxQueueSize --;
				OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, status);
                *transferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
            }

            /*Release mutex, done with shared resource*/
            osalError = OSAL_MUTEX_Unlock(&gUSBDeviceHidCommonDataObj.mutexHIDIRP);
            if(osalError != OSAL_RESULT_TRUE)
            {
                /*Do not proceed, unlock was not complete, or error occurred, let user know about error*/
                return (USB_DEVICE_HID_RESULT_ERROR);
            }
             return hidReceiveError;
         }
    }

    /*Release mutex, done with shared resource*/
    osalError = OSAL_MUTEX_Unlock(&gUSBDeviceHidCommonDataObj.mutexHIDIRP);
    if(osalError != OSAL_RESULT_TRUE)
    {
	/*Do not proceed, unlock was not complete, or error occurred, let user know about error*/
	return (USB_DEVICE_HID_RESULT_ERROR);
    }
    /* We could not find a free IRP */
    SYS_ASSERT(false,"Receive Queue is full");
    return USB_DEVICE_HID_RESULT_ERROR_TRANSFER_QUEUE_FULL;
}


/******************************************************************************
  Function:
    void _USB_DEVICE_HID_ControlTransferHandler
    (
        USB_DEVICE_CONTROL_TRANSFER_HANDLE controlHandle,
        SYS_MODULE_INDEX iHID,
        void * pEventData 
    )

  Summary:
    Handles all HID related control transfers.

  Description:
    Handles all HID related control transfers.

  Remarks:
    This is a local function and should not be called directly by the
    application.

*/

void _USB_DEVICE_HID_ControlTransferHandler
(
    SYS_MODULE_INDEX iHID,
    USB_DEVICE_EVENT controlEvent,
    USB_SETUP_PACKET * setupPkt
)
{
    size_t length;
    uint8_t reportID;
    static uint8_t altSetting = 0;
    USB_HID_PROTOCOL_CODE setProtocol;
    USB_DEVICE_HID_INSTANCE * hidThisInstance;
    USB_DEVICE_HID_EVENT_DATA_SET_IDLE setIdle;
    USB_DEVICE_HID_EVENT_DATA_GET_REPORT getReport;
    USB_DEVICE_HID_EVENT_DATA_SET_REPORT setReport;
    
    hidThisInstance = &gUsbDeviceHidInstance[iHID] ;

    if(controlEvent == USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST)
    {
        hidThisInstance->ignoreControlEvents = false;
        if(( setupPkt->Recipient == 0x01) &&
                ((setupPkt->RequestType == 0)))
        {
            switch(setupPkt->bRequest)
            {
                case USB_REQUEST_GET_DESCRIPTOR:

                    if (setupPkt->bDescriptorType == USB_HID_DESCRIPTOR_TYPES_HID)
                    {
                        /* The HID Get Descriptor request is handled by the
                         * function driver itself. This event is not sent to the
                         * application. The function driver responds with a
                         * control send. */
                        if (setupPkt->wLength >= hidThisInstance->hidFuncInit->hidReportDescriptorSize)
                        {
                            length = hidThisInstance->hidFuncInit->hidReportDescriptorSize;
                        }
                        else
                        {
                            length = setupPkt->wLength ;
                        }
                        
                        USB_DEVICE_ControlSend(hidThisInstance->devLayerHandle, hidThisInstance->hidDescriptor, length);

                    }
                    else if (setupPkt->bDescriptorType == USB_HID_DESCRIPTOR_TYPES_REPORT)
                    {
                        /* The HID Get Descriptor request is handled by the
                         * function driver itself. This event is not sent to the
                         * application. The function driver responds with a
                         * control send. */
                        if (setupPkt->wLength >= hidThisInstance->hidFuncInit->hidReportDescriptorSize)
                        {
                            length = hidThisInstance->hidFuncInit->hidReportDescriptorSize;
                        }
                        else
                        {
                            length = setupPkt->wLength ;
                        }
                        
                        USB_DEVICE_ControlSend(hidThisInstance->devLayerHandle, hidThisInstance->hidFuncInit->hidReportDescriptor, length);
                    }
                    
                    /* Ignore further control transfer events. */
                    hidThisInstance->ignoreControlEvents = true;
                    break;

                case USB_REQUEST_SET_INTERFACE:

                     altSetting = setupPkt->W_Value.byte.LB;
                     USB_DEVICE_ControlStatus( hidThisInstance->devLayerHandle, USB_DEVICE_CONTROL_STATUS_OK);
                     break; 

                case USB_REQUEST_GET_INTERFACE:

                     USB_DEVICE_ControlSend( hidThisInstance->devLayerHandle, &altSetting, 1);
                     break;

                default:
                    USB_DEVICE_ControlStatus( hidThisInstance->devLayerHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
                    break; 
            }                   
        }
        else if( (setupPkt->RequestType == 1) &&
                (setupPkt->Recipient == 1)&&
                ( hidThisInstance->appCallBack != NULL ) )
        {
            switch( setupPkt->bRequest )
            {
                case  USB_HID_REQUESTS_GET_REPORT:

                    /* Get Report event is sent to the application */
                    getReport.reportType = setupPkt->W_Value.byte.HB;
                    getReport.reportID = setupPkt->W_Value.byte.LB;
                    getReport.reportLength = setupPkt->wLength;

                    hidThisInstance->appCallBack(iHID, USB_DEVICE_HID_EVENT_GET_REPORT, &getReport, hidThisInstance-> userData);
                    break;

                case USB_HID_REQUESTS_GET_IDLE:

                    /* Get Idle event is sent to the host */
                    reportID = setupPkt->W_Value.byte.LB;
                    hidThisInstance->appCallBack(iHID, USB_DEVICE_HID_EVENT_GET_IDLE, &reportID, hidThisInstance->userData);
                    break;

                case USB_HID_REQUESTS_GET_PROTOCOL:

                    /* Get Protocol event is sent to the application. There
                     * is no event data in this case. */

                    hidThisInstance->appCallBack(iHID, USB_DEVICE_HID_EVENT_GET_PROTOCOL, NULL, hidThisInstance->userData);
                    break;

                case USB_HID_REQUESTS_SET_REPORT:

                    /* Set Report event is sent to the application */
                    setReport.reportType = setupPkt->W_Value.byte.HB;
                    setReport.reportID = setupPkt->W_Value.byte.LB;
                    setReport.reportLength = setupPkt->wLength;

                    hidThisInstance->appCallBack(iHID, USB_DEVICE_HID_EVENT_SET_REPORT, &setReport, hidThisInstance->userData );
                    break;

                case USB_HID_REQUESTS_SET_PROTOCOL:

                    /* Set Protocol event is sent to the application */
                    setProtocol = (USB_HID_PROTOCOL_CODE)(setupPkt->wValue);
                    hidThisInstance->appCallBack(iHID, USB_DEVICE_HID_EVENT_SET_PROTOCOL, &setProtocol, hidThisInstance->userData );
                    break;

                case USB_HID_REQUESTS_SET_IDLE:

                    /* Set Idle event is sent to the application */
                    setIdle.duration = setupPkt->W_Value.byte.HB;
                    setIdle.reportID = setupPkt->W_Value.byte.LB;

                    hidThisInstance->appCallBack(iHID, USB_DEVICE_HID_EVENT_SET_IDLE, &setIdle, hidThisInstance->userData );
                    break;

                default:
                    
                    /* Stall anything that we cannot handle */
                    USB_DEVICE_ControlStatus( hidThisInstance->devLayerHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
                    break;
            }
        }
        else
        {
            /* Stall anything that we cannot handle */
            USB_DEVICE_ControlStatus( hidThisInstance->devLayerHandle,  USB_DEVICE_CONTROL_STATUS_ERROR);
        }
    }
    else if ((hidThisInstance->ignoreControlEvents == false) &&
            ( hidThisInstance->appCallBack != NULL))
    {
        /* These are control transfer related events */
        hidThisInstance->appCallBack(iHID, controlEvent, NULL, hidThisInstance->userData);
    }
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_HID_DeInitialize(SYS_MODULE_INDEX iHID)

  Summary:
     Deinitializes an instance of the HID.

  Description:
     Deinitializes an instance of the HID.

  Returns:
    This is a local function and should not be called directly by the application.
*/

void _USB_DEVICE_HID_DeInitialize(SYS_MODULE_INDEX iHID)
{
    USB_DEVICE_HID_INSTANCE * hidInstance = &gUsbDeviceHidInstance[iHID];

    if (hidInstance->flags.interruptEpTxReady)
    {
        USB_DEVICE_IRPCancelAll( hidInstance->devLayerHandle,
                                hidInstance->endpointTx );
        USB_DEVICE_EndpointDisable(  hidInstance->devLayerHandle,
                                hidInstance->endpointTx);
    }
    if (hidInstance->flags.interruptEpRxReady)
    {
         USB_DEVICE_IRPCancelAll( hidInstance->devLayerHandle,
                                hidInstance->endpointRx );
        USB_DEVICE_EndpointDisable(  hidInstance->devLayerHandle,
                                hidInstance->endpointRx);
    }
    hidInstance->flags.allFlags = 0;   
}

// ******************************************************************************
/* Function:
    USB_DEVICE_HID_RESULT USB_DEVICE_HID_EventHandlerSet 
    (
        USB_DEVICE_HID_INDEX iHID ,
        USB_DEVICE_HID_EVENT_HANDLER eventHandler
        uintptr_t context
    )

  Summary:
    Allows application to register an event handler.

  Description:
    This function allows the application to register an event handler.

  Remarks:
    Refer to usb_device_hid.h for usage information.
*/

USB_DEVICE_HID_RESULT USB_DEVICE_HID_EventHandlerSet
(
    USB_DEVICE_HID_INDEX iHID ,
    USB_DEVICE_HID_EVENT_HANDLER eventHandler,
    uintptr_t userData
)
{
    USB_DEVICE_HID_INSTANCE * thisHIDInstance;

    /* Check if we have a valid instance */
    if( (iHID < 0) || ( iHID > USB_DEVICE_HID_INSTANCES_NUMBER) )
    {
        SYS_ASSERT(false, "Invalid HID Instance");
        return USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID ;
    }

    /* Check if we have valid event handler*/
    if(eventHandler == NULL)
    {
        SYS_ASSERT(false, "Event Handler is NULL");
        return USB_DEVICE_HID_RESULT_ERROR_PARAMETER_INVALID;
    }

    thisHIDInstance = &gUsbDeviceHidInstance[iHID];

    thisHIDInstance->appCallBack = eventHandler;
    thisHIDInstance->userData = userData;
    
    return USB_DEVICE_HID_RESULT_OK;    
}

/******************************************************************************/













