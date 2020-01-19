/*******************************************************************************
  USB Host CDC Client Driver Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_cdc.c

  Summary:
    USB Host CDC Client Driver Implementation

  Description:
    This file contains the implementation of the CDC Client Driver API. It
    should be included in the application if the CDC Host Client Driver
    functionality is desired.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/

#include "usb/usb_host_cdc.h"
#include "usb/usb_host_client_driver.h"
#include "usb/usb_host.h"
#include "usb/usb_cdc.h"
#include "usb/src/usb_host_cdc_local.h"

/************************************************
 * CDC Host Client Driver instance objects. One
 * for each CDC device.
 ************************************************/
 USB_HOST_CDC_INSTANCE_OBJ gUSBHostCDCObj[USB_HOST_CDC_INSTANCES_NUMBER];

/***********************************************
 * USB Host CDC Attach Listener Objects
 ***********************************************/
 USB_HOST_CDC_ATTACH_LISTENER_OBJ gUSBHostCDCAttachListener[USB_HOST_CDC_ATTACH_LISTENERS_NUMBER];

/************************************************
 * CDC Interface to the host layer
 ************************************************/
USB_HOST_CLIENT_DRIVER gUSBHostCDCClientDriver =
{
    .initialize = _USB_HOST_CDC_Initialize,
    .deinitialize = _USB_HOST_CDC_Deinitialize,
    .reinitialize = _USB_HOST_CDC_Reinitialize,
    .interfaceAssign = _USB_HOST_CDC_InterfaceAssign,
    .interfaceRelease = _USB_HOST_CDC_InterfaceRelease,
    .interfaceEventHandler = _USB_HOST_CDC_InterfaceEventHandler,
    .interfaceTasks = _USB_HOST_CDC_InterfaceTasks,
    .deviceEventHandler = _USB_HOST_CDC_DeviceEventHandler,
    .deviceAssign = _USB_HOST_CDC_DeviceAssign,
    .deviceRelease = _USB_HOST_CDC_DeviceRelease,
    .deviceTasks = _USB_HOST_CDC_DeviceTasks
};

// *****************************************************************************
// *****************************************************************************
// CDC Host Client Driver Local function
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:  
    USB_HOST_CDC_RESULT _USB_HOST_CDC_HostResutlToCDCResultMap
    (
        USB_HOST_RESULT hostResult
    )

  Summary: 
    This function will map the USB Host result to CDC Result.

  Description:
    This function will map the USB Host result to CDC Result.

  Remarks:
    This is a local function and should not be called by directly by the
    application.
*/

USB_HOST_CDC_RESULT _USB_HOST_CDC_HostResutlToCDCResultMap
(
    USB_HOST_RESULT result
)
{
    USB_HOST_CDC_RESULT cdcResult;
  
    switch(result)
    {
        case USB_HOST_RESULT_SUCCESS:
            cdcResult = USB_HOST_CDC_RESULT_SUCCESS;
            break;
        case USB_HOST_RESULT_FAILURE:
            /* Note the fall through here. This is intentional */
        case USB_HOST_RESULT_PARAMETER_INVALID:
        case USB_HOST_RESULT_PIPE_HANDLE_INVALID:
            cdcResult = USB_HOST_CDC_RESULT_FAILURE;
            break;
        case USB_HOST_RESULT_REQUEST_BUSY:
            cdcResult = USB_HOST_CDC_RESULT_BUSY;
            break;
        case USB_HOST_RESULT_REQUEST_STALLED:
            cdcResult = USB_HOST_CDC_RESULT_REQUEST_STALLED;
            break;
        case USB_HOST_RESULT_TRANSFER_ABORTED:
            cdcResult = USB_HOST_CDC_RESULT_ABORTED;
            break;
        default:
            cdcResult = USB_HOST_CDC_RESULT_FAILURE;
            break;
    }
  
    return(cdcResult);
}

// *****************************************************************************
/* Function:
    int _USB_HOST_CDC_DeviceHandleToInstance
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceClientHandle
    );

  Summary:
    This function will return the index of the CDC object that own this device.

  Description:
    This function will return the index of the CDC object that own this device.
    If an instance is not found, the function will return -1.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_CDC_DeviceHandleToInstance
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceClientHandle
)
{
    int result = -1;
    int iterator;

    for(iterator = 0; iterator < USB_HOST_CDC_INSTANCES_NUMBER; iterator++)
    {
        if(gUSBHostCDCObj[iterator].deviceClientHandle == deviceClientHandle)
        {
            result = iterator;
            break;
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    int _USB_HOST_CDC_InterfaceHandleToInstance
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    );

  Summary:
    This function will return the index of the CDC object that own this
    interface.

  Description:
    This function will return the index of the CDC object that owns this
    interface.  If an instance is not found, the function will return -1.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_CDC_InterfaceHandleToInstance
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    int result = -1;
    int iterator;

    for(iterator = 0; iterator < USB_HOST_CDC_INSTANCES_NUMBER; iterator++)
    {
        if((gUSBHostCDCObj[iterator].communicationInterfaceHandle == interfaceHandle) ||
            (gUSBHostCDCObj[iterator].dataInterfaceHandle == interfaceHandle))

        {
            result = iterator;
            break;
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    int _USB_HOST_CDC_DeviceObjHandleToInstance
    (
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle
    );

  Summary:
    This function will return the index of the CDC object that is associated
    with this device.

  Description:
    This function will return the index of the CDC object that is associated
    with this device.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_CDC_DeviceObjHandleToInstance
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle
)
{
    int result = -1;
    int iterator;

    for(iterator = 0; iterator < USB_HOST_CDC_INSTANCES_NUMBER; iterator++)
    {
        if(gUSBHostCDCObj[iterator].deviceObjHandle == deviceObjHandle) 
        {
            result = iterator;
            break;
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    void _USB_HOST_CDC_Initialize(void * msdInitData)

  Summary:
    This function is called when the Host Layer is initializing.

  Description:
    This function is called when the Host Layer is initializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_Initialize(void * data)
{
    int iterator;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstanceObj;

    for(iterator = 0; iterator < USB_HOST_CDC_INSTANCES_NUMBER; iterator ++)
    {
        /* Set the pipes handles to invalid */
        cdcInstanceObj = &gUSBHostCDCObj[iterator];
        cdcInstanceObj->deviceClientHandle = USB_HOST_DEVICE_CLIENT_HANDLE_INVALID;
        cdcInstanceObj->bulkInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        cdcInstanceObj->bulkOutPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        cdcInstanceObj->interruptPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        cdcInstanceObj->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_CDC_Deinitialize(void)

  Summary:
    This function is called when the Host Layer is deinitializing.

  Description:
    This function is called when the Host Layer is deinitializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_Deinitialize(void)
{
    /* This function is not implemented in this release of the USB Host stack */
}

// *****************************************************************************
/* Function:
    void _USB_HOST_CDC_Reinitialize(void)

  Summary:
    This function is called when the Host Layer is reinitializing.

  Description:
    This function is called when the Host Layer is reinitializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_Reinitialize(void * msdInitData)
{
    /* This function is not implemented in this release of the driver */
}

// *****************************************************************************
/* Function:
    void _USB_HOST_CDC_DeviceAssign 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        USB_DEVICE_DESCRIPTOR * deviceDescriptor
    )
 
  Summary: 
    This function is called when the host layer finds device level class
    subclass protocol match.

  Description:
    This function is called when the host layer finds device level class
    subclass protocol match.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_DeviceAssign 
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_DEVICE_DESCRIPTOR * deviceDescriptor
)
{
    int iterator;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstanceObj = NULL;

    /* If this function is being called, this means that a device class subclass
     * protocol match was obtained.  Look for a free instance */

    for(iterator = 0; iterator < USB_HOST_CDC_INSTANCES_NUMBER; iterator ++)
    {
        /* Search for an available CDC instance object */
        if(!gUSBHostCDCObj[iterator].inUse)
        {
            /* Allocate the object */
            cdcInstanceObj = &gUSBHostCDCObj[iterator];
            cdcInstanceObj->inUse = true;
            cdcInstanceObj->deviceObjHandle = deviceObjHandle;
            cdcInstanceObj->deviceClientHandle = deviceHandle;
            break;
        }
    }

    if(cdcInstanceObj == NULL)
    {
        /* This means an instance could not be allocated. Return the device back
         * to the host */
        USB_HOST_DeviceRelease(deviceHandle);
    }
    else
    {
        /* An instance object was allocated. Check if the device has any
         * configurations */

        if(deviceDescriptor->bNumConfigurations > 0)
        {
            /* This means we have configurations. We can try setting the first 
             * configuration. Also open the control pipe to the device. */
            cdcInstanceObj->state = USB_HOST_CDC_STATE_SET_CONFIGURATION;
            cdcInstanceObj->controlPipeHandle = USB_HOST_DeviceControlPipeOpen(deviceObjHandle);
        }
        else
        {
            /* There are no configurations! Move to error state */
            cdcInstanceObj->state = USB_HOST_CDC_STATE_ERROR;
        }
    }
}
 
// *****************************************************************************
/* Function:
    void _USB_HOST_CDC_DeviceRelease 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle
    )
 
  Summary: 
    This function is called when the device is detached. 

  Description:
    This function is called when the device is detached. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_DeviceRelease
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle
)
{
    int index;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;

    /* Find the CDC instance object that owns this device */
    index = _USB_HOST_CDC_DeviceHandleToInstance(deviceHandle);

    if(index >= 0)
    {
        cdcInstance = &gUSBHostCDCObj[index];
        cdcInstance->inUse = false;

        if(cdcInstance->bulkInPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
        {
            /* Close the bulk in pipe and invalidate the pipe handle */
            USB_HOST_DevicePipeClose(cdcInstance->bulkInPipeHandle);
            cdcInstance->bulkInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        }
        
        if(cdcInstance->bulkOutPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
        {
            /* Close the bulk Out pipe and invalidate the pipe handle */
            USB_HOST_DevicePipeClose(cdcInstance->bulkOutPipeHandle);
            cdcInstance->bulkOutPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        }

        if(cdcInstance->interruptPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
        {
            /* Close the interruptPipeHandle pipe handle and invalidate the pipe
             * handle */
            USB_HOST_DevicePipeClose(cdcInstance->interruptPipeHandle);
            cdcInstance->interruptPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        }

        if(cdcInstance->eventHandler != NULL)
        {
            /* Let the client know that the device is detached */
            cdcInstance->eventHandler((USB_HOST_CDC_HANDLE)(cdcInstance), 
                    USB_HOST_CDC_EVENT_DEVICE_DETACHED,
                    NULL, cdcInstance->context);
        }

        /* Release the object */
        cdcInstance->eventHandler = NULL;
        cdcInstance->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
        cdcInstance->deviceClientHandle = USB_HOST_DEVICE_CLIENT_HANDLE_INVALID;
        
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_CDC_DeviceTasks 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle
    )
 
  Summary: 
    This function is called when the host layer want the device to update its
    state. 

  Description:
    This function is called when the host layer want the device to update its
    state. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_DeviceTasks
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle
)
{
    int index, iterator;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_RESULT result;
    USB_HOST_REQUEST_HANDLE requestHandle;

    /* Get the index of the device instance */
    index = _USB_HOST_CDC_DeviceHandleToInstance(deviceHandle);

    if(index >= 0)
    {
        /* Get a pointer to the instance */
        cdcInstance = &gUSBHostCDCObj[index];
        
        if(cdcInstance->inUse)
        {
            /* Instance is valid */
            switch(cdcInstance->state)
            {
                case USB_HOST_CDC_STATE_NOT_READY:

                    /* The instance is not ready. We dont do anything yet */
                    break;

                case USB_HOST_CDC_STATE_SET_CONFIGURATION:

                    /* The instance should set the configuration. We clear the
                     * controlTransferDone flag. This will be set in the device
                     * event handler when the configuration set event is received. */
                    
                    cdcInstance->hostRequestDone = false;
                    result = USB_HOST_DeviceConfigurationSet(cdcInstance->deviceClientHandle, &requestHandle, 
                            0, (uintptr_t)(cdcInstance));
                    
                    if(result == USB_HOST_RESULT_SUCCESS)
                    {
                        /* The result was successful. Change state to wating for
                         * configuration. */
                        cdcInstance->state = USB_HOST_CDC_STATE_WAIT_FOR_CONFIGURATION_SET;
                    }
                    break;

                case USB_HOST_CDC_STATE_WAIT_FOR_CONFIGURATION_SET:

                    /* Here we are waiting for the configuration to be set */
                    if(cdcInstance->hostRequestDone == true) 
                    {
                        if(cdcInstance->hostRequestResult == USB_HOST_RESULT_SUCCESS)
                        {
                            /* The configuration has been set. Now wait for the host
                             * layer to send the communication and the data
                             * interface to the client driver */

                            cdcInstance->state = USB_HOST_CDC_STATE_WAIT_FOR_INTERFACES;
                        }
                        else
                        {
                            /* If we could not set the configuration, then state
                             * instance state to error */
                            cdcInstance->state = USB_HOST_CDC_STATE_ERROR;
                        }
                    }
                    break;

                case USB_HOST_CDC_STATE_WAIT_FOR_INTERFACES:

                    /* Here we wait for both the interfaces to get ready */
                    if((cdcInstance->interruptPipeHandle != USB_HOST_PIPE_HANDLE_INVALID) &&
                            (cdcInstance->bulkInPipeHandle != USB_HOST_PIPE_HANDLE_INVALID) &&
                            (cdcInstance->bulkOutPipeHandle != USB_HOST_PIPE_HANDLE_INVALID))
                    {
                        /* Set the state to ready */
                        cdcInstance->state = USB_HOST_CDC_STATE_READY;
                        
                        /* We know that the client driver is now ready. We can 
                         * let all the listeners know that the device has been
                         * attached */
                        
                        for(iterator = 0; iterator < USB_HOST_CDC_ATTACH_LISTENERS_NUMBER; iterator ++)
                        {
                            if(gUSBHostCDCAttachListener[iterator].inUse)
                            {
                                /* Call the attach listener event handler 
                                 * function. */
                                gUSBHostCDCAttachListener[iterator].eventHandler((USB_HOST_CDC_OBJ)(cdcInstance), 
                                        gUSBHostCDCAttachListener[iterator].context);
                            }
                        }
                    }
                            
                    break;
                    
                case USB_HOST_CDC_STATE_READY:
                    
                    /* The CDC client driver is ready to be opened */
                    break;

                case USB_HOST_CDC_STATE_ERROR:

                    /* The instance is an error state. We dont do anything here */
                    break;

            }
        }
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_CDC_InterfaceAssign 
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        size_t nInterfaces,
        uint8_t * descriptor
    )

  Summary:
    This function is called when the Host Layer attaches this driver to an
    interface.

  Description:
    This function is called when the Host Layer attaches this driver to an
    interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_InterfaceAssign 
(
    USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    size_t nInterfaces,
    uint8_t * descriptor
)
{
    int cdcInstanceIndex, iterator;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance = NULL;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;
    USB_ENDPOINT_DESCRIPTOR * endpointDescriptor;
    USB_HOST_ENDPOINT_DESCRIPTOR_QUERY endpointDescriptorQuery;
    USB_HOST_INTERFACE_DESCRIPTOR_QUERY interfaceDescriptorQuery; 

    /* This function will be called when there is an interface level match.
     * There are two possible cases here. In case of a simple CDC device, the
     * driver would have matched at the device level in which case the device Client
     * handle will not be invalid. Or else this driver matched at the interface
     * level because this is a composite device. */

    /* If the number of interfaces passed to this function is 1, then we know
     * this is a single interface */

    if(nInterfaces == 1)
    {
        /* One one interface was passed. This means there was device level class
         * subclass protocol match. The device must aleady exist in the system */

        cdcInstanceIndex = _USB_HOST_CDC_DeviceObjHandleToInstance(deviceObjHandle);
        if(cdcInstanceIndex >= 0)
        {
            /* Found the instance index that owns this object */
            cdcInstance = &gUSBHostCDCObj[cdcInstanceIndex];

            /* Get the interface descriptor. We know this is an interface
             * descriptor because the number of interfaces is 1 */
            interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)descriptor;

            if((interfaceDescriptor->bInterfaceClass == USB_CDC_COMMUNICATIONS_INTERFACE_CLASS_CODE) &&
                    (interfaceDescriptor->bInterfaceSubClass == USB_CDC_SUBCLASS_ABSTRACT_CONTROL_MODEL) &&
                    (interfaceDescriptor->bInterfaceProtocol == USB_CDC_PROTOCOL_AT_V250))
            {
                /* This interface is the communications class interface. Get the
                 * endpoint number */
                cdcInstance->communicationInterfaceHandle = interfaces[0];
                cdcInstance->commInterfaceNumber = interfaceDescriptor->bInterfaceNumber;
                USB_HOST_DeviceEndpointQueryContextClear(&endpointDescriptorQuery);
                endpointDescriptorQuery.transferType = USB_TRANSFER_TYPE_INTERRUPT;
                endpointDescriptorQuery.direction = USB_DATA_DIRECTION_DEVICE_TO_HOST;
                endpointDescriptorQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE|USB_HOST_ENDPOINT_QUERY_BY_DIRECTION;
                endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointDescriptorQuery);

                /* Did we find the endpoint? */
                if(endpointDescriptor != NULL)
                {
                    /* Found the endpoint. Open the pipe. If the pipe was not
                     * opened, the device will never move to a ready state. */
                    cdcInstance->interruptPipeHandle = USB_HOST_DevicePipeOpen(cdcInstance->communicationInterfaceHandle, endpointDescriptor->bEndpointAddress);
                }
            }
            else if((interfaceDescriptor->bInterfaceClass == USB_CDC_DATA_INTERFACE_CLASS_CODE) &&
                    (interfaceDescriptor->bInterfaceSubClass == 0x0) &&
                    (interfaceDescriptor->bInterfaceProtocol == USB_CDC_PROTOCOL_NO_CLASS_SPECIFIC))
            {
                /* This is the data interface */
                cdcInstance->dataInterfaceHandle = interfaces[0];
                cdcInstance->dataInterfaceNumber = interfaceDescriptor->bInterfaceNumber;
                USB_HOST_DeviceEndpointQueryContextClear(&endpointDescriptorQuery);
                endpointDescriptorQuery.transferType = USB_TRANSFER_TYPE_BULK;
                endpointDescriptorQuery.direction = USB_DATA_DIRECTION_DEVICE_TO_HOST;
                endpointDescriptorQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE|USB_HOST_ENDPOINT_QUERY_BY_DIRECTION;
                endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointDescriptorQuery);

                /* Did we find the bulk in point */
                if(endpointDescriptor != NULL)
                {
                    /* Yes we did. Open this pipe */
                    cdcInstance->bulkInPipeHandle = USB_HOST_DevicePipeOpen(cdcInstance->dataInterfaceHandle, endpointDescriptor->bEndpointAddress);
                }

                /* Bulk in pipe is opened. Now open the bulk out pipe */
                USB_HOST_DeviceEndpointQueryContextClear(&endpointDescriptorQuery);
                endpointDescriptorQuery.transferType = USB_TRANSFER_TYPE_BULK;
                endpointDescriptorQuery.direction = USB_DATA_DIRECTION_HOST_TO_DEVICE;
                endpointDescriptorQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE|USB_HOST_ENDPOINT_QUERY_BY_DIRECTION;
                endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointDescriptorQuery);

                /* Did we find the pipe */
                if(endpointDescriptor != NULL)
                {
                    /* Yes we did. Open this pipe */
                    cdcInstance->bulkOutPipeHandle = USB_HOST_DevicePipeOpen(cdcInstance->dataInterfaceHandle, 
                            endpointDescriptor->bEndpointAddress);
                }
            }
            else
            {
                /* Dont know what this interface is. Return it back */
                USB_HOST_DeviceInterfaceRelease(interfaces[0]);
            }
        }
        else
        {
            /* This is an error case. The instance should exist. We return this
             * interface back to the host. */
            USB_HOST_DeviceInterfaceRelease(interfaces[0]);
        }
    }
    else if(nInterfaces > 1)
    {
        /* Then this means that this is an IAD. We first assign a CDC instance
         * to this device */

        for(iterator = 0; iterator < USB_HOST_CDC_INSTANCES_NUMBER; iterator ++)
        {
            if(!gUSBHostCDCObj[iterator].inUse)
            {
                cdcInstance = &gUSBHostCDCObj[iterator];
                cdcInstance->inUse = true;
                break;
            }
        }

        if(cdcInstance == NULL)
        {
            /* This means we could not find an instance. Release all the
             * interfaces in this group */

            for(iterator = 0; iterator < nInterfaces; iterator ++)
            {
                USB_HOST_DeviceInterfaceRelease(interfaces[iterator]);
            }
        }
        else
        {
            /* Save the device object handle and open the control pipe */
            cdcInstance->deviceObjHandle = deviceObjHandle;
            cdcInstance->controlPipeHandle = USB_HOST_DeviceControlPipeOpen(deviceObjHandle);

            /* An instance is assigned. The descriptor will be a pointer to the
             * IAD. Lets get the first interface descriptor in the IAD group and
             * see which interface this is. */ 

            USB_HOST_DeviceInterfaceQueryContextClear(&interfaceDescriptorQuery);
            interfaceDescriptorQuery.flags = USB_HOST_INTERFACE_QUERY_ANY;

            /* We know that we need two interfaces */
            for(iterator = 0; iterator < 2; iterator ++)
            {
                /* We need to search for two interface descriptors */
                interfaceDescriptor = USB_HOST_DeviceGeneralInterfaceDescriptorQuery
                        ((USB_INTERFACE_ASSOCIATION_DESCRIPTOR *)(descriptor), &interfaceDescriptorQuery);
                
                /* If we have a valid interface descriptor find out its type */
                if(interfaceDescriptor != NULL)
                {
                    if((interfaceDescriptor->bInterfaceClass == USB_CDC_COMMUNICATIONS_INTERFACE_CLASS_CODE) &&
                            (interfaceDescriptor->bInterfaceSubClass == USB_CDC_SUBCLASS_ABSTRACT_CONTROL_MODEL) &&
                            (interfaceDescriptor->bInterfaceProtocol == USB_CDC_PROTOCOL_AT_V250))
                    {
                        /* We found the communication class */
                        cdcInstance->commInterfaceNumber = interfaceDescriptor->bInterfaceNumber;
                        cdcInstance->communicationInterfaceHandle = interfaces[iterator];

                        /* Create the endpoint query */
                        USB_HOST_DeviceEndpointQueryContextClear(&endpointDescriptorQuery);
                        endpointDescriptorQuery.transferType = USB_TRANSFER_TYPE_INTERRUPT;
                        endpointDescriptorQuery.direction = USB_DATA_DIRECTION_DEVICE_TO_HOST;
                        endpointDescriptorQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE|USB_HOST_ENDPOINT_QUERY_BY_DIRECTION;
                        endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointDescriptorQuery);

                        if(endpointDescriptor != NULL)
                        {
                            /* Open the pipe */
                            cdcInstance->interruptPipeHandle = USB_HOST_DevicePipeOpen(cdcInstance->communicationInterfaceHandle, 
                                    endpointDescriptor->bEndpointAddress);
                        }
                        else
                        {
                            /* Make sure that the pipe handle stays invalid if
                             * we could not open the pipe */
                            cdcInstance->interruptPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
                        }
                    }
                    else if ((interfaceDescriptor->bInterfaceClass == USB_CDC_DATA_INTERFACE_CLASS_CODE) && 
                            (interfaceDescriptor->bInterfaceSubClass == 0x00) && 
                            (interfaceDescriptor->bInterfaceProtocol == USB_CDC_PROTOCOL_NO_CLASS_SPECIFIC))
                    {
                        /* We found the data class */

                        cdcInstance->dataInterfaceHandle = interfaces[iterator];
                        cdcInstance->dataInterfaceNumber = interfaceDescriptor->bInterfaceNumber;

                        /* Get the bulk in endpoint */ 
                        USB_HOST_DeviceEndpointQueryContextClear(&endpointDescriptorQuery);
                        endpointDescriptorQuery.transferType = USB_TRANSFER_TYPE_BULK;
                        endpointDescriptorQuery.direction = USB_DATA_DIRECTION_DEVICE_TO_HOST;
                        endpointDescriptorQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE|USB_HOST_ENDPOINT_QUERY_BY_DIRECTION;
                        endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointDescriptorQuery);

                        if(endpointDescriptor != NULL)
                        {
                            cdcInstance->bulkInPipeHandle = USB_HOST_DevicePipeOpen(cdcInstance->dataInterfaceHandle, 
                                    endpointDescriptor->bEndpointAddress);
                        }
                        else
                        {
                            /* Make the pipe handle invalid */
                            cdcInstance->bulkInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
                        }

                        /* Get the bulk in endpoint */ 
                        USB_HOST_DeviceEndpointQueryContextClear(&endpointDescriptorQuery);
                        endpointDescriptorQuery.transferType = USB_TRANSFER_TYPE_BULK;
                        endpointDescriptorQuery.direction = USB_DATA_DIRECTION_HOST_TO_DEVICE;
                        endpointDescriptorQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE|USB_HOST_ENDPOINT_QUERY_BY_DIRECTION;
                        endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointDescriptorQuery);

                        if(endpointDescriptor != NULL)
                        {
                            cdcInstance->bulkOutPipeHandle = USB_HOST_DevicePipeOpen(cdcInstance->dataInterfaceHandle, 
                                    endpointDescriptor->bEndpointAddress);
                        }
                        else
                        {
                            /* Make the pipe handle invalid */
                            cdcInstance->bulkOutPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
                        }

                    }
                }
                else
                {
                    /* There have to be at least two interface descriptors */
                }
            }

            /* Now check if we can move the host client driver instance to a
             * ready state */

            if((cdcInstance->interruptPipeHandle != USB_HOST_PIPE_HANDLE_INVALID) &&
                    (cdcInstance->bulkInPipeHandle != USB_HOST_PIPE_HANDLE_INVALID) &&
                    (cdcInstance->bulkOutPipeHandle != USB_HOST_PIPE_HANDLE_INVALID))
            {
                /* All the pipes are opened. The client driver is ready */
                cdcInstance->state = USB_HOST_CDC_STATE_READY;

                /* We know that the client driver is now ready. We can 
                 * let all the listeners know that the device has been
                 * attached.  */

                for(iterator = 0; iterator < USB_HOST_CDC_ATTACH_LISTENERS_NUMBER; iterator ++)
                {
                    if(gUSBHostCDCAttachListener[iterator].inUse)
                    {
                        /* Call the attach listener event handler 
                         * function. */
                        gUSBHostCDCAttachListener[iterator].eventHandler((USB_HOST_CDC_OBJ)(cdcInstance), 
                                gUSBHostCDCAttachListener[iterator].context);
                    }
                }
            }
            else
            {
                /* Something went wrong. */
                cdcInstance->state = USB_HOST_CDC_STATE_ERROR;
            }
        }
    }
}

// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_CDC_InterfaceEventHandler
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
        USB_HOST_DEVICE_INTERFACE_EVENT event,
        void * eventData,
        uintptr_t context
    )

  Summary:
    This function is called when the Host Layer generates interface level
    events. 

  Description:
    This function is called when the Host Layer generates interface level
    events. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_CDC_InterfaceEventHandler
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_DEVICE_INTERFACE_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    int cdcIndex;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA * dataTransferEvent;
    USB_HOST_CDC_EVENT cdcEvent;
    USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA cdcTransferCompleteData;

    /* Find out to which CDC Instance this interface belongs */
    cdcIndex = _USB_HOST_CDC_InterfaceHandleToInstance(interfaceHandle);
    cdcInstance = &gUSBHostCDCObj[cdcIndex];
    cdcEvent = (USB_HOST_CDC_EVENT)(context); 

    switch(event)
    {
        case USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE:

            /* This means a data transfer has completed */
            dataTransferEvent = (USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA *)(eventData);
            cdcTransferCompleteData.transferHandle = dataTransferEvent->transferHandle;
            cdcTransferCompleteData.result = _USB_HOST_CDC_HostResutlToCDCResultMap(dataTransferEvent->result);
            cdcTransferCompleteData.length = dataTransferEvent->length;

            if(cdcInstance->eventHandler != NULL)
            {
                cdcInstance->eventHandler((USB_HOST_CDC_HANDLE)(cdcInstance), cdcEvent, 
                        &cdcTransferCompleteData, cdcInstance->context);
            }

            break;

        default:
            break;
    }
    
    return(USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE);
}

// *****************************************************************************
/* Function:
    void USB_HOST_CDC_InterfaceTasks
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    )

  Summary:
    This function is called by the Host Layer to update the state of this
    driver.

  Description:
    This function is called by the Host Layer to update the state of this
    driver.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_InterfaceTasks
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
}

// *****************************************************************************
/* Function:
    void USB_HOST_CDC_InterfaceRelease
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    )

  Summary:
    This function is called when the Host Layer detaches this driver from an
    interface.

  Description:
    This function is called when the Host Layer detaches this driver from an
    interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_CDC_InterfaceRelease
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    int cdcIndex;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    
    /* Get the instance associated with this interface */
    cdcIndex = _USB_HOST_CDC_InterfaceHandleToInstance(interfaceHandle);
    
    if(cdcIndex >= 0)
    {
        /* Get the pointer to the instance object */
        cdcInstance = &gUSBHostCDCObj[cdcIndex];
        
        if(cdcInstance->inUse)
        {
            cdcInstance->inUse = false;

            if(cdcInstance->bulkInPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
            {
                /* Close the bulk in pipe and invalidate the pipe handle */
                USB_HOST_DevicePipeClose(cdcInstance->bulkInPipeHandle);
                cdcInstance->bulkInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
            }

            if(cdcInstance->bulkOutPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
            {
                /* Close the bulk Out pipe and invalidate the pipe handle */
                USB_HOST_DevicePipeClose(cdcInstance->bulkOutPipeHandle);
                cdcInstance->bulkOutPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
            }

            if(cdcInstance->interruptPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
            {
                /* Close the interruptPipeHandle pipe handle and invalidate the pipe
                 * handle */
                USB_HOST_DevicePipeClose(cdcInstance->interruptPipeHandle);
                cdcInstance->interruptPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
            }

            if(cdcInstance->eventHandler != NULL)
            {
                /* Let the client know that the device is detached */
                cdcInstance->eventHandler((USB_HOST_CDC_HANDLE)(cdcInstance), 
                        USB_HOST_CDC_EVENT_DEVICE_DETACHED,
                        NULL, cdcInstance->context);
            }

            /* Release the object */
            cdcInstance->eventHandler = NULL;
            cdcInstance->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
            cdcInstance->deviceClientHandle = USB_HOST_DEVICE_CLIENT_HANDLE_INVALID;
        }
    }
}

// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_CDC_DeviceEventHandler
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
        USB_HOST_DEVICE_EVENT event,
        void * eventData,
        uintptr_t context
    )

  Summary:
    This function is called when the Host Layer generates device level
    events. 

  Description:
    This function is called when the Host Layer generates device level
    events. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_DEVICE_EVENT_RESPONSE _USB_HOST_CDC_DeviceEventHandler
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    USB_HOST_DEVICE_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    /* The event context is the pointer to the CDC Instance Object */
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)(context);
    USB_HOST_DEVICE_EVENT_CONFIGURATION_SET_DATA * configSetEventData;

    switch(event)
    {
        case USB_HOST_DEVICE_EVENT_CONFIGURATION_SET:

            /* This means the configuration was set. Update the instance
             * variables to let the main state machine know. */
            configSetEventData = (USB_HOST_DEVICE_EVENT_CONFIGURATION_SET_DATA *)(eventData);
            cdcInstance->hostRequestResult =  configSetEventData->result;
            cdcInstance->hostRequestDone = true;
            break;

        case USB_HOST_DEVICE_EVENT_CONFIGURATION_DESCRIPTOR_GET_COMPLETE:
            break;

        default:
            break;
    }
    
    return(USB_HOST_DEVICE_EVENT_RESPONSE_NONE);
}

// *****************************************************************************
// *****************************************************************************
// Public Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// CDC Host Client Driver Public function
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_AttachEventHandlerSet
    (
        USB_HOST_CDC_ATTACH_EVENT_HANDLER eventHandler,
        uintptr_t context
    );
           
  Summary:
    This function will set an attach event handler.

  Description:
    This function will set an attach event handler. The attach event handler
    will be called when a CDC device has been attached. The context will be
    returned in the event handler. This function should be called before the bus
    has been enabled.

  Remarks:
    Refer to usb_host_cdc.h for usage information.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_AttachEventHandlerSet
(
    USB_HOST_CDC_ATTACH_EVENT_HANDLER eventHandler,
    uintptr_t context
)
{
    int iterator;

    USB_HOST_CDC_RESULT result = USB_HOST_CDC_RESULT_FAILURE;
    USB_HOST_CDC_ATTACH_LISTENER_OBJ * attachListener;
    
    if(eventHandler == NULL)
    {
        result = USB_HOST_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Search for free listener object */
        for(iterator = 0; iterator < USB_HOST_CDC_ATTACH_LISTENERS_NUMBER; iterator ++)
        {
            if(!gUSBHostCDCAttachListener[iterator].inUse)
            {
                /* Found a free object */
                attachListener = &gUSBHostCDCAttachListener[iterator];
                attachListener->inUse = true;
                attachListener->eventHandler = eventHandler;
                attachListener->context = context;
                result = USB_HOST_CDC_RESULT_SUCCESS;
                break;
            }
        }
    }

    return(result);
}

// ****************************************************************************
/* Function:
    USB_HOST_CDC_HANDLE USB_HOST_CDC_Open
    (
        USB_HOST_CDC_OBJ cdcDeviceObj
    );
           
  Summary:
    This function opens the specified CDC device.

  Description:
    This function will open the specified CDC device. Once opened, the CDC
    device can be accessed via the handle which this function returns. The
    cdcDeviceObj parameter is the value returned in the
    USB_HOST_CDC_ATTACH_EVENT_HANDLER event handling function.

  Remarks:
    Refer to usb_host_cdc.h for usage information.
*/

USB_HOST_CDC_HANDLE USB_HOST_CDC_Open
(
    USB_HOST_CDC_OBJ cdcDeviceObj
)
{
    USB_HOST_CDC_HANDLE result = USB_HOST_CDC_HANDLE_INVALID;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;

    /* The present implementation is a single client implementation only */

    if(cdcDeviceObj != 0)
    {
        cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)cdcDeviceObj;
        if((cdcInstance->inUse) && (cdcInstance->state == USB_HOST_CDC_STATE_READY))
        {
            result = (USB_HOST_CDC_HANDLE)(cdcDeviceObj);
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_EventHandlerSet
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_EVENT_HANDLER eventHandler,
        uintptr_t context
    );

  Summary:
    Registers an event handler with the CDC Host Client Driver.

  Description:
    This function registers a client specific CDC Host Client Driver event
    handler. The CDC Host Client Driver will call this function with relevant
    event and assocaite event data, in response to command requests and data
    transfers that have been scheduled by the client.

  Remarks:
    Refer to usb_host_cdc.h for usage information.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_EventHandlerSet
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_EVENT_HANDLER eventHandler,
    uintptr_t context
)
{
    USB_HOST_CDC_RESULT result = USB_HOST_CDC_RESULT_HANDLE_INVALID;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)(handle);

    if(cdcInstance != NULL)
    {
        cdcInstance->eventHandler = eventHandler;
        cdcInstance->context = context;
        result = USB_HOST_CDC_RESULT_SUCCESS;
    }

    return(result);
}

// ****************************************************************************
/* Function:
    void USB_HOST_CDC_Close
    (
        USB_HOST_CDC_HANDLE cdcDeviceHandle
    );
           
  Summary:
    This function closes the CDC device.

  Description:
    This function will close the open CDC device. This closes the association
    between the application entity that opened the device and device. The driver
    handle becomes invalid.

  Remarks:
    None.
*/

void USB_HOST_CDC_Close
(
    USB_HOST_CDC_HANDLE cdcDeviceHandle
)
{
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)(cdcDeviceHandle);

    if(cdcInstance != NULL)
    {
        /* In this release of the CDC driver, the close function does not do
         * much. If the client registered an event handler, then this is set to
         * NULL. */

        cdcInstance->eventHandler = NULL;
    }
}

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_Write
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
        void * data,
        size_t size
    );
           
  Summary:
    This function will write data to the attached device.

  Description:
    This function will write data to the attached CDC device. The function will
    write size amount of bytes. If the request was accepted, transferHandle will
    contain a valid transfer handle, else it will contain
    USB_HOST_CDC_TRANSFER_HANDLE_INVALID. The completion of the request will be
    indicated by the USB_HOST_CDC_EVENT_WRITE_COMPLETE event. The transfer
    handle will be returned in the event. 

  Remarks:
    Refer to usb_host_cdc.h for usage information.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_Write
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
    void * data,
    size_t size
)
{
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_CDC_TRANSFER_HANDLE * tempTransferHandle, localTransferHandle;
    USB_HOST_CDC_RESULT cdcResult = USB_HOST_CDC_RESULT_FAILURE;
    USB_HOST_RESULT hostResult;

    cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)handle;

    if(cdcInstance == NULL)
    {
        /* This handle is not valid */
        cdcResult = USB_HOST_CDC_RESULT_HANDLE_INVALID;
    }
    else
    {
        /* Check if the specified transfer handle holder is NULL, if so use a local
         * transfer handle holder */

        tempTransferHandle = (transferHandle == NULL) ? &localTransferHandle: transferHandle;

        if(!cdcInstance->inUse)
        {
            /* This object is not valid */
            cdcResult = USB_HOST_CDC_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            if(cdcInstance->state != USB_HOST_CDC_STATE_READY)
            {
                /* The instance is not ready for requests */
                cdcResult = USB_HOST_CDC_RESULT_BUSY;
            }
            else
            {
                if((size != 0) && (data == NULL))
                {
                    /* Input paramters are not valid */
                    cdcResult = USB_HOST_CDC_RESULT_INVALID_PARAMETER;
                }
                else
                {
                    /* The context for the transfer is the event that needs to
                     * be sent to the application. In this case the event to be
                     * sent to the application when the transfer completes is
                     * USB_HOST_CDC_EVENT_WRITE_COMPLETE */
                    
                    hostResult = USB_HOST_DeviceTransfer(cdcInstance->bulkOutPipeHandle, tempTransferHandle, data, size, (uintptr_t)(USB_HOST_CDC_EVENT_WRITE_COMPLETE));
                    cdcResult = _USB_HOST_CDC_HostResutlToCDCResultMap(hostResult);
                }
            }
        }
    }

    return(cdcResult);
}

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_Read
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
        void * data,
        size_t size
    );
           
  Summary:
    This function will read data from the attached device.

  Description:
    This function will read data from the attached CDC device. The function will
    try to read size amount of bytes but will stop reading when the device
    terminates the USB transfer (sends a short packet or a ZLP). If the request
    was accepted, transferHandle will contain a valid transfer handle, else it
    will contain USB_HOST_CDC_TRANSFER_HANDLE_INVALID. The completion of the
    request will be indicated by the USB_HOST_CDC_EVENT_READ_COMPLETE
    event. The transfer handle will be returned in the event. 

  Remarks:
    None.
*/
USB_HOST_CDC_RESULT USB_HOST_CDC_Read
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
    void * data,
    size_t size
)
{
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_CDC_TRANSFER_HANDLE * tempTransferHandle, localTransferHandle;
    USB_HOST_CDC_RESULT cdcResult = USB_HOST_CDC_RESULT_FAILURE;
    USB_HOST_RESULT hostResult;

    cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)handle;

    if(cdcInstance == NULL)
    {
        /* This handle is not valid */
        cdcResult = USB_HOST_CDC_RESULT_HANDLE_INVALID;
    }
    else
    {
        /* Check if the specified transfer handle holder is NULL, if so use a local
         * transfer handle holder */

        tempTransferHandle = (transferHandle == NULL) ? &localTransferHandle: transferHandle;

        if(!cdcInstance->inUse)
        {
            /* This object is not valid */
            cdcResult = USB_HOST_CDC_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            if(cdcInstance->state != USB_HOST_CDC_STATE_READY)
            {
                /* The instance is not ready for requests */
                cdcResult = USB_HOST_CDC_RESULT_BUSY;
            }
            else
            {
                if((size != 0) && (data == NULL))
                {
                    /* Input paramters are not valid */
                    cdcResult = USB_HOST_CDC_RESULT_INVALID_PARAMETER;
                }
                else
                {
                    /* The context for the transfer is the event that needs to
                     * be sent to the application. In this case the event to be
                     * sent to the application when the transfer completes is
                     * USB_HOST_CDC_EVENT_READ_COMPLETE */
                    
                    hostResult = USB_HOST_DeviceTransfer(cdcInstance->bulkInPipeHandle, tempTransferHandle, data, size, (uintptr_t)(USB_HOST_CDC_EVENT_READ_COMPLETE));
                    cdcResult = _USB_HOST_CDC_HostResutlToCDCResultMap(hostResult);
                }
            }
        }
    }

    return(cdcResult);
}

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_SerialStateNotificationGet
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
        USB_CDC_SERIAL_STATE * serialState
    );
           
  Summary:
    This function will request Serial State Notification from the attached
    device.

  Description:
    This function will request Serial State Notification from the attached
    device. If the request was accepted, transferHandle will contain a valid
    transfer handle, else it will contain USB_HOST_CDC_TRANSFER_HANDLE_INVALID.
    The completion of the request will be indicated by the
    USB_HOST_CDC_EVENT_SERIAL_STATE_NOTIFICATION_RECEIVED event. The transfer
    handle will be returned in the event. 

  Remarks:
    None.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_SerialStateNotificationGet
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
    USB_CDC_SERIAL_STATE * serialState
)
{
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_CDC_TRANSFER_HANDLE * tempTransferHandle, localTransferHandle;
    USB_HOST_CDC_RESULT cdcResult = USB_HOST_CDC_RESULT_FAILURE;
    USB_HOST_RESULT hostResult;

    cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)handle;

    if(cdcInstance == NULL)
    {
        /* This handle is not valid */
        cdcResult = USB_HOST_CDC_RESULT_HANDLE_INVALID;
    }
    else
    {
        /* Check if the specified transfer handle holder is NULL, if so use a local
         * transfer handle holder */

        tempTransferHandle = (transferHandle == NULL) ? &localTransferHandle: transferHandle;

        if(!cdcInstance->inUse)
        {
            /* This object is not valid */
            cdcResult = USB_HOST_CDC_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            if(cdcInstance->state != USB_HOST_CDC_STATE_READY)
            {
                /* The instance is not ready for requests */
                cdcResult = USB_HOST_CDC_RESULT_BUSY;
            }
            else
            {
                if(serialState == NULL)
                {
                    /* Input paramters are not valid */
                    cdcResult = USB_HOST_CDC_RESULT_INVALID_PARAMETER;
                }
                else
                {
                    /* The context for the transfer is the event that needs to
                     * be sent to the application. In this case the event to be
                     * sent to the application when the transfer completes is
                     * USB_HOST_CDC_EVENT_SERIAL_STATE_NOTIFICATION_RECEIVED */
                    
                    hostResult = USB_HOST_DeviceTransfer(cdcInstance->interruptPipeHandle, tempTransferHandle, serialState, sizeof(USB_CDC_SERIAL_STATE), 
                            (uintptr_t)(USB_HOST_CDC_EVENT_SERIAL_STATE_NOTIFICATION_RECEIVED));
                    cdcResult = _USB_HOST_CDC_HostResutlToCDCResultMap(hostResult);
                }
            }
        }
    }

    return(cdcResult);
}


