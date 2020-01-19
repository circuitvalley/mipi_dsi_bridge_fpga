/**************************************************************************
 USB Device Layer Endpoint Read Write functions Implementation

  Company:
    Microchip Technology Inc.
    
  File Name:
    usb_device_endpoint_transfer.c
    
  Summary:
    This file contains implementations of both private and public functions
    of the USB Device Layer Enadpoint Transfer. Add this file to your project
    only if the application wants to write data to an Endpoint directly. Example
    USB Vendor devices.
    
  Description:
    This file contains the USB Device Layer Endpoint Transfer Implementation.
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

/* USB Device Endpoint IRP array. */
USB_DEVICE_IRP gUSBDeviceEndpointIRP[USB_DEVICE_ENDPOINT_QUEUE_DEPTH_COMBINED];

/* Array for tracking Read/Write Queue size for each USB Device instance */
USB_DEVICE_Q_SIZE_ENDPOINT qSizeEndpoint[USB_DEVICE_INSTANCES_NUMBER];

// *****************************************************************************
// *****************************************************************************
// Section: USB Device Layer System Interface functions.
// *****************************************************************************
// *****************************************************************************
// ******************************************************************************
/* Function:
    USB_DEVICE_RESULT USB_DEVICE_EndpointWrite
    (
        USB_DEVICE_HANDLE usbDeviceHandle,
        USB_DEVICE_TRANSFER_HANDLE * transferHandle,
        USB_ENDPOINT_ADDRESS endpoint,
        const void * data,
        size_t size,
        USB_DEVICE_TRANSFER_FLAGS flags
    )

  Summary:
    This function requests a data write to a USB Device Endpoint.

  Description:
    Refer to usb_device.h for usage information.

  Returns:
    Refer to usb_device.h for usage information.
*/

USB_DEVICE_RESULT USB_DEVICE_EndpointWrite
(
    USB_DEVICE_HANDLE usbDeviceHandle,
    USB_DEVICE_TRANSFER_HANDLE * transferHandle,
    USB_ENDPOINT_ADDRESS endpoint,
    const void * data,
    size_t size,
    USB_DEVICE_TRANSFER_FLAGS flags
)
{
    int count = 0;
    USB_DEVICE_OBJ* devClientHandle;
    USB_ERROR irpSubmitError;
    SYS_MODULE_INDEX deviceInstanceNumber;
    USB_DEVICE_Q_SIZE_ENDPOINT* thisEndpointQueueSize;
    USB_DEVICE_IRP * irp ;
    OSAL_RESULT osalError;
    *transferHandle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
    OSAL_CRITSECT_DATA_TYPE IntState;


    /* Validate the handle */
    devClientHandle = _USB_DEVICE_ClientHandleValidate(usbDeviceHandle);
    if (devClientHandle == NULL)
    {
        SYS_DEBUG(0, "USB Device Layer: Invalid Client Handle");
        return(USB_DEVICE_RESULT_ERROR_DEVICE_HANDLE_INVALID);
    }

    /* Get Device Instance Number */
    deviceInstanceNumber = devClientHandle->usbDevLayerIndex;

    /* Get Handle to the Endpoint Queue Size structure */
    thisEndpointQueueSize  = &qSizeEndpoint[deviceInstanceNumber];

    /* Make sure that we are with in the queue size for this instance */
    if(thisEndpointQueueSize->qSizeCurrentEpWrite >= thisEndpointQueueSize->qSizeMaxEpWrite)
    {
        SYS_DEBUG(0, "Write Queue is full");
        return(USB_DEVICE_RESULT_ERROR_TRANSFER_QUEUE_FULL);
    }

    /*Obtain mutex to get access to a shared resource, check return value*/
    osalError = OSAL_MUTEX_Lock(&(devClientHandle->mutexEndpointIRP), OSAL_WAIT_FOREVER);
    if(osalError != OSAL_RESULT_TRUE)
    {
      /*Do not proceed lock was not obtained, or error occurred, let user know about error*/
      return (USB_DEVICE_RESULT_ERROR);
    }

    /* Check if the if there is free slot available in queue */
    for(count = 0; count < (USB_DEVICE_ENDPOINT_QUEUE_DEPTH_COMBINED) ; count ++ )
    {
        if(gUSBDeviceEndpointIRP[count].status <
                (USB_DEVICE_IRP_STATUS)USB_DEVICE_IRP_FLAG_DATA_PENDING)
        {
            /* This means the IRP is free. Configure the IRP
             * update the current queue size and then submit */
            irp = &gUSBDeviceEndpointIRP[count]; 
            irp->data = (void *)data;
            irp->size = size;
            irp->flags = flags;
            irp->callback = &_USB_DEVICE_EndpointWriteCallBack;
            irp->userData = (uintptr_t)devClientHandle;
            (* transferHandle) = ( USB_DEVICE_TRANSFER_HANDLE )irp;
            
            /* Prevent other tasks pre-empting this sequence of code */ 
            IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
            (thisEndpointQueueSize->qSizeCurrentEpWrite)++;
            OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
            
            irpSubmitError = USB_DEVICE_IRPSubmit( devClientHandle, endpoint, irp);
            
            /* If IRP Submit function returned any error, then invalidate the
               Transfer handle.  */
            if (irpSubmitError != USB_ERROR_NONE )
            {
                /* Prevent other tasks pre-empting this sequence of code */ 
                IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
                /* Update the read queue size */ 
                (thisEndpointQueueSize->qSizeCurrentEpWrite)--;
                OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
                *transferHandle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
            }
            
            /*Release mutex, done with shared resource*/
            osalError = OSAL_MUTEX_Unlock(&(devClientHandle->mutexEndpointIRP));
            if(osalError != OSAL_RESULT_TRUE)
            {
                /*Do not proceed lock was not obtained, or error occurred, let user know about error*/
                return (USB_DEVICE_RESULT_ERROR);
            }
            return(irpSubmitError);
        }
    }
    
    /*Release mutex, done with shared resource*/
    osalError = OSAL_MUTEX_Unlock(&(devClientHandle->mutexEndpointIRP));
    if(osalError != OSAL_RESULT_TRUE)
    {
        /*Do not proceed, unlock was not completed, or error occurred, let user know about error*/
        return (USB_DEVICE_RESULT_ERROR);
    }
    /* We could not find a spare IRP */
    SYS_DEBUG(0, "USB Device Endpoint Write: Transfer queue is full");
    return USB_DEVICE_RESULT_ERROR_TRANSFER_QUEUE_FULL;
}


/******************************************************************************
  Function:
    USB_DEVICE_RESULT USB_DEVICE_EndpointRead
    (
        USB_DEVICE_HANDLE usbDeviceHandle,
        USB_DEVICE_TRANSFER_HANDLE * transferHandle,
        USB_ENDPOINT_ADDRESS endpoint,
        void * buffer,
        size_t bufferSize
    );

  Summary:
    Reads data received from host on the requested endpoint.

  Description:
    Refer to usb_device.h for usage information.

  Returns:
    Refer to usb_device.h for usage information.

*/

USB_DEVICE_RESULT USB_DEVICE_EndpointRead
(
    USB_DEVICE_HANDLE usbDeviceHandle,
    USB_DEVICE_TRANSFER_HANDLE * transferHandle,
    USB_ENDPOINT_ADDRESS endpoint,
    void * buffer,
    size_t bufferSize
)
{
    int count = 0;
    USB_DEVICE_OBJ* devClientHandle;
    USB_ERROR irpSubmitError;
    SYS_MODULE_INDEX deviceInstanceNumber;
    USB_DEVICE_Q_SIZE_ENDPOINT* thisEndpointQueueSize;
    USB_DEVICE_IRP * irp = gUSBDeviceEndpointIRP;
    OSAL_RESULT osalError;
    *transferHandle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
    OSAL_CRITSECT_DATA_TYPE IntState;
    
    /* Validate the handle */
    devClientHandle = _USB_DEVICE_ClientHandleValidate(usbDeviceHandle);
    if (devClientHandle == NULL)
    {
        SYS_DEBUG(0, "USB Device Layer: Invalid Client Handle");
        return(USB_DEVICE_RESULT_ERROR_DEVICE_HANDLE_INVALID);
    }

    /* Get Device Instance Number */
    deviceInstanceNumber = devClientHandle->usbDevLayerIndex;

    /* Get Handle to the Endpoint Queue Size structure */
    thisEndpointQueueSize  = &qSizeEndpoint[deviceInstanceNumber];
    
    /* Make sure that we are with in the queue size for this instance */
    if(thisEndpointQueueSize->qSizeCurrentEpRead >= thisEndpointQueueSize->qSizeMaxEpRead)
    {
        SYS_ASSERT(false, "Read Queue is full");
        return(USB_DEVICE_RESULT_ERROR_TRANSFER_QUEUE_FULL);
    }
    
    /*Obtain mutex to get access to a shared resource, check return value*/
    osalError = OSAL_MUTEX_Lock(&(devClientHandle->mutexEndpointIRP), OSAL_WAIT_FOREVER);
    if(osalError != OSAL_RESULT_TRUE)
    {
      /*Do not proceed lock was not obtained, or error occurred, let user know about error*/
      return (USB_DEVICE_RESULT_ERROR);
    }

    /* Check if the if there is free slot available in queue */
    for(count = 0; count < (USB_DEVICE_ENDPOINT_QUEUE_DEPTH_COMBINED) ; count ++ )
    {
        if(gUSBDeviceEndpointIRP[count].status <
                (USB_DEVICE_IRP_STATUS)USB_DEVICE_IRP_FLAG_DATA_PENDING)
        {
            /* This means the IRP is free. Configure the IRP
             * update the current queue size and then submit */
            irp = &gUSBDeviceEndpointIRP[count]; 
            irp->data = buffer;
            irp->size = bufferSize;
            irp->callback = &_USB_DEVICE_EndpointReadCallBack;
            irp->userData = (uintptr_t)devClientHandle;
            (*transferHandle) = (USB_DEVICE_TRANSFER_HANDLE ) irp;
            
            /* Prevent other tasks pre-empting this sequence of code */ 
            IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
            (thisEndpointQueueSize->qSizeCurrentEpRead)++;
            OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
            
            irpSubmitError = USB_DEVICE_IRPSubmit(devClientHandle, endpoint, irp);
            
            /* If IRP Submit function returned any error, then invalidate the
               Transfer handle.  */
            if (irpSubmitError != USB_ERROR_NONE )
            {
                /* Prevent other tasks pre-empting this sequence of code */ 
                IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
                /* Update the read queue size */ 
                (thisEndpointQueueSize->qSizeCurrentEpRead)--;
                OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
                *transferHandle = USB_DEVICE_TRANSFER_HANDLE_INVALID;
            }
            
            /*Release mutex, done with shared resource*/
            osalError = OSAL_MUTEX_Unlock(&(devClientHandle->mutexEndpointIRP));
            if(osalError != OSAL_RESULT_TRUE)
            {
                /*Do not proceed lock was not obtained, or error occurred, let user know about error*/
                return (USB_DEVICE_RESULT_ERROR);
            }

            return(irpSubmitError);
        }
    }
    /*Release mutex, done with shared resource*/
    osalError = OSAL_MUTEX_Unlock(&(devClientHandle->mutexEndpointIRP));
    if(osalError != OSAL_RESULT_TRUE)
    {
        /*Do not proceed, unlock was not complete, or error occurred, let user know about error*/
        return (USB_DEVICE_RESULT_ERROR);
    }
    /* Could not find a free IRP. */
    SYS_DEBUG(0, "USB Device Endpoint Read: Transfer queue is full");
    return USB_DEVICE_RESULT_ERROR_TRANSFER_QUEUE_FULL;
}

/******************************************************************************
  Function:
    USB_DEVICE_RESULT USB_DEVICE_EndpointTransferCancel
    (
        USB_DEVICE_HANDLE usbDeviceHandle,
        USB_ENDPOINT_ADDRESS endpoint,
        USB_DEVICE_TRANSFER_HANDLE transferHandle
    );

  Summary:
    This function cancels a transfer scheduled on an endpoint.

  Description:
    Refer to usb_device.h for usage information.

  Returns:
    Refer to usb_device.h for usage information.

*/
USB_DEVICE_RESULT USB_DEVICE_EndpointTransferCancel
(
    USB_DEVICE_HANDLE usbDeviceHandle,
    USB_ENDPOINT_ADDRESS endpoint,
    USB_DEVICE_TRANSFER_HANDLE transferHandle
)
{
    USB_DEVICE_OBJ* devClientHandle;
    USB_DEVICE_IRP * irp ;
    USB_ERROR irpCancelError;

     /* Validate the handle */
    devClientHandle = _USB_DEVICE_ClientHandleValidate(usbDeviceHandle);
    if (devClientHandle == NULL)
    {
        SYS_DEBUG(0, "USB Device Layer: Invalid Client Handle");
        return(USB_DEVICE_RESULT_ERROR_DEVICE_HANDLE_INVALID);
    }

    if (transferHandle == USB_DEVICE_TRANSFER_HANDLE_INVALID)
    {
        SYS_DEBUG(0, "USB_Device_Endpoint_Transfer_Cancel: Invalid Transfer Handle");
        return USB_DEVICE_RESULT_ERROR;
    }
    irp =  (USB_DEVICE_IRP*)transferHandle;
    if (irp->status == USB_DEVICE_IRP_STATUS_PENDING)
    {
        irpCancelError = USB_DEVICE_IRPCancelAll(devClientHandle,endpoint );
        return irpCancelError;
    }

    SYS_DEBUG(0, "USB_Device_Endpoint_Transfer_Cancel: Transfer could not be cancelled");
    return USB_DEVICE_RESULT_ERROR;

}
// ******************************************************************************
/* Function:
    void _USB_DEVICE_VENDOR_EndpointWriteCallBack( void * irp )

  Summary:
    Endpoint write callback.

  Description:
    This function is called when a endpoint write is complete.

  Remarks:
    This is a local function and should not be called directly by the application.
*/

void _USB_DEVICE_EndpointWriteCallBack( USB_DEVICE_IRP * irp )
{
    /* An endpoint write has completed */
    USB_DEVICE_EVENT_DATA_ENDPOINT_WRITE_COMPLETE eventData;
    SYS_MODULE_INDEX deviceInstanceNumber;
    USB_DEVICE_Q_SIZE_ENDPOINT* thisEndpointQueueSize;
    USB_DEVICE_OBJ* devClientHandle;

    /* Get Handle to the Device Object */
    devClientHandle = (USB_DEVICE_OBJ*)irp->userData;

    /* Get Device instance Number */
    deviceInstanceNumber = devClientHandle->usbDevLayerIndex;

    /* Get Handle to the Endpoint Queue Size */
    thisEndpointQueueSize = &qSizeEndpoint[deviceInstanceNumber];

    /* Update Queue Size. We have freed one read queue element as we have
     completed a Transfer */
    thisEndpointQueueSize->qSizeCurrentEpWrite --;
       
    if( devClientHandle->callBackFunc != NULL )
    {
        /* Get data size received from Host */
        eventData.length = irp->size;

        /* Get Transfer Handle */
        eventData.transferHandle = ( USB_DEVICE_TRANSFER_HANDLE )irp;

        /* Get transfer status */
        if ((irp->status == USB_DEVICE_IRP_STATUS_COMPLETED) 
            || (irp->status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
        {
            /* Transfer completed successfully */
            eventData.status = USB_DEVICE_RESULT_OK; 
        }
        else if (irp->status == USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT)
        {
            /* Transfer canceled due to Endpoint Halt */
            eventData.status = USB_DEVICE_RESULT_ERROR_ENDPOINT_HALTED; 
        }
        else if (irp->status == USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST)
        {
            /* Transfer Canceled by Host (Host sent a Clear feature )*/
            eventData.status = USB_DEVICE_RESULT_ERROR_TERMINATED_BY_HOST; 
        }
        else
        {
            /* Transfer was not completed successfully */
            eventData.status = USB_DEVICE_RESULT_ERROR; 
        }
        
        /* Send an event to application letting it know that a endpoint write
         * has completed */
        devClientHandle->callBackFunc(
            USB_DEVICE_EVENT_ENDPOINT_WRITE_COMPLETE,
            &eventData,
            devClientHandle->context);
    }
}
// ******************************************************************************
/* Function:
    void _USB_DEVICE_VENDOR_EndpointReadCallBack( void * irp )

  Summary:
    Endpoint read callback.

  Description:
    This function is called when an endpoint read has completed.

  Remarks:
    This is a local function and should not be called directly by the application.
*/

void _USB_DEVICE_EndpointReadCallBack( USB_DEVICE_IRP * irp )
{
    USB_DEVICE_EVENT_DATA_ENDPOINT_READ_COMPLETE eventData;
    SYS_MODULE_INDEX deviceInstanceNumber;
    USB_DEVICE_Q_SIZE_ENDPOINT* thisEndpointQueueSize;
    USB_DEVICE_OBJ* devClientHandle;

    /* Get Handle to the Device Object */
    devClientHandle = (USB_DEVICE_OBJ*)irp->userData;

    /* Get Device instance Number */
    deviceInstanceNumber = devClientHandle->usbDevLayerIndex;

    /* Get Handle to the Endpoint Queue Size */
    thisEndpointQueueSize = &qSizeEndpoint[deviceInstanceNumber];

    /* Update Queue Size. We have freed one read queue element as we have
    completed a Transfer */
    thisEndpointQueueSize->qSizeCurrentEpRead --;

    if( devClientHandle->callBackFunc != NULL )
    {
        /* Get data size received from Host */
        eventData.length = irp->size;

        /* Get Transfer Handle */
        eventData.transferHandle = ( USB_DEVICE_TRANSFER_HANDLE )irp;
        
        /* Get transfer status */
        if ((irp->status == USB_DEVICE_IRP_STATUS_COMPLETED) 
            || (irp->status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
        {
            /* Transfer completed successfully */
            eventData.status = USB_DEVICE_RESULT_OK; 
        }
        else if (irp->status == USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT)
        {
            /* Transfer canceled due to Endpoint Halt */
            eventData.status = USB_DEVICE_RESULT_ERROR_ENDPOINT_HALTED; 
        }
        else if (irp->status == USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST)
        {
            /* Transfer Canceled by Host (Host sent a Clear feature )*/
            eventData.status = USB_DEVICE_RESULT_ERROR_TERMINATED_BY_HOST; 
        }
        else
        {
            /* Transfer was not completed successfully */
            eventData.status = USB_DEVICE_RESULT_ERROR; 
        }

        /* Send an event to application letting it know that a endpoint read
         * has completed */
        devClientHandle->callBackFunc(
                USB_DEVICE_EVENT_ENDPOINT_READ_COMPLETE ,
                &eventData,
                (uintptr_t)devClientHandle->context );
    }
}

void _USB_DEVICE_Initialize_Endpoint_Q_Size(SYS_MODULE_INDEX index, uint16_t qSizeRead, uint16_t qSizeWrite )
{
    USB_DEVICE_Q_SIZE_ENDPOINT* thisEndpointQueue = &qSizeEndpoint[index];
    thisEndpointQueue->qSizeMaxEpRead = qSizeRead;
    thisEndpointQueue->qSizeMaxEpWrite = qSizeWrite;
    thisEndpointQueue->qSizeCurrentEpRead = 0;
    thisEndpointQueue->qSizeCurrentEpWrite = 0; 
}

void _USB_DEVICE_EndpointQueueSizeReset(SYS_MODULE_INDEX index)
{
    int iEntry;

    /* This function is called when the device layer receives
     * a Set Configuration request */
    USB_DEVICE_Q_SIZE_ENDPOINT* thisEndpointQueue = &qSizeEndpoint[index];
    thisEndpointQueue->qSizeCurrentEpRead = 0;
    thisEndpointQueue->qSizeCurrentEpWrite = 0;

    for(iEntry = 0; iEntry < USB_DEVICE_ENDPOINT_QUEUE_DEPTH_COMBINED; iEntry ++)
    {
        /* Get back all the IRPs */
        gUSBDeviceEndpointIRP[iEntry].status = USB_DEVICE_IRP_STATUS_COMPLETED;
    }
}
/********************End of file********************************/
