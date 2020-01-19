/*******************************************************************************
  USB Host Hub driver implementation .

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_hub.c

  Summary:
    This file contains implementations of both private and public functions
    of the USB Host HUB client driver.

  Description:
    This file contains the USB host Hub client driver implementation. This file
    should be included in the project if USB HUB devices are to be supported.
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
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host_client_driver.h"
#include "usb/usb_host.h"
#include "usb/usb_hub.h"
#include "usb/usb_host_hub_interface.h"
#include "usb/src/usb_host_hub_local.h"
#include "system/debug/sys_debug.h"
#include "system/tmr/sys_tmr.h"

/**************************************************
 * Global array of HUB Instance Objects. Each for
 * one HUB device attached.
 ***************************************************/

USB_HOST_HUB_INSTANCE_OBJ gUSBHostHubObj[USB_HOST_HUB_INSTANCES_NUMBER];

/*************************************************
 * Driver interface that is provide to the
 * host layer.
 *************************************************/

USB_HOST_CLIENT_DRIVER  gUSBHostHUBClientDriver =
{
    .initialize         = _USB_HOST_HUB_Initialize ,
    .deinitialize       = _USB_HOST_HUB_Deinitialize ,
    .reinitialize       = _USB_HOST_HUB_Reinitialize,
    .deviceAssign       = _USB_HOST_HUB_DeviceAssign,
    .deviceRelease      = _USB_HOST_HUB_DeviceRelease,
    .deviceTasks        = _USB_HOST_HUB_DeviceTasks,
    .deviceEventHandler = _USB_HOST_HUB_DeviceEventHandler,
    .interfaceAssign    = _USB_HOST_HUB_InterfaceAssign,
    .interfaceRelease   = _USB_HOST_HUB_InterfaceRelease,
    .interfaceEventHandler = _USB_HOST_HUB_InterfaceEventHandler,
    .interfaceTasks     = _USB_HOST_HUB_InterfaceTasks
};

// *****************************************************************************
/* USB Hub Port API Interface

  Summary:
    Group of function pointers to the USB Hub Port Functions.

  Description:
    This structure is a group of function pointers pointing to the USB Hub Port
    API routines. The USB Hub or USB root hub should export this group of
    functions so that the Host Layer can access the port functionality.

  Remarks:
    None.
*/

USB_HUB_INTERFACE  externalHubInterface =
{
    .hubPortReset             =   USB_HOST_HUB_PortReset,
    .hubPortResetIsComplete   =   USB_HOST_HUB_PortResetComplete,
    .hubPortSuspend           =   USB_HOST_HUB_PortSuspend,
    .hubPortResume            =   USB_HOST_HUB_PortResume,
    .hubPortSpeedGet          =   USB_HOST_HUB_PortSpeedGet
};

// *****************************************************************************
// *****************************************************************************
// HUB Host Client Driver Local function
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_Initialize(void * hubInitData)

  Summary:
    This function is called when the Host Layer is initializing.

  Description:
    This function is called when the Host Layer is initializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_Initialize(void * hubInitData )
{
    /* Make sure all the pipe handles are invalid */

    int iterator;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstanceInfo;

    for (iterator = 0; iterator < USB_HOST_HUB_INSTANCES_NUMBER; iterator ++)
    {
        hubInstanceInfo = &gUSBHostHubObj[iterator];
        hubInstanceInfo->controlPipeHandle = USB_HOST_CONTROL_PIPE_HANDLE_INVALID;
        hubInstanceInfo->interruptInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_Deinitialize(void)

  Summary:
    This function is called when the Host Layer is deinitializing.

  Description:
    This function is called when the Host Layer is deinitializing .

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_Deinitialize(void)
{
    /* This function is not implemented in this release  */
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_Reinitialize(void)

  Summary:
    This function is called when the Host Layer is reinitializing.

  Description:
    This function is called when the Host Layer is reinitializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_Reinitialize(void * hubInitData)
{
    /* This function is not implemented in this release  */
}

// *****************************************************************************
/* Function:
    int _USB_HOST_HUB_DeviceHandleToInstance
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceClientHandle
    );

  Summary:
    This function will return the index of the HUB object that own this device.

  Description:
    This function will return the index of the HUB object that own this device.
    If an instance is not found, the function will return -1.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_HUB_DeviceHandleToInstance
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceClientHandle
)
{
    int result = -1;
    int iterator;

    for(iterator = 0; iterator < USB_HOST_HUB_INSTANCES_NUMBER; iterator++)
    {
        /* Check if the instance matches */

        if(gUSBHostHubObj[iterator].hubDeviceClientHandle == deviceClientHandle)
        {
            result = iterator;
            break;
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_DeviceAssign 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,,
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        USB_DEVICE_DESCRIPTOR * deviceDescriptor ,
    )

  Summary:
    This function is called when the Host Layer attaches this driver to a
    device.

  Description:
    This function is called when the Host Layer attaches this driver to a
    device at a device level.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_DeviceAssign 
( 
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_DEVICE_DESCRIPTOR * deviceDescriptor 
)
{
    int iterator;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstanceObj = NULL;
	uint8_t portIndex;
    USB_HOST_HUB_PORT_INFO  *portInfo = NULL ;

    /* If this function is being called, this means that a device class subclass
     * protocol match was obtained.  Look for a free instance */

    for(iterator = 0; iterator < USB_HOST_HUB_INSTANCES_NUMBER; iterator ++)
    {
        /* Search for an available Hub instance object */
        if(!gUSBHostHubObj[iterator].inUse)
        {
            /* Allocate the object */
            hubInstanceObj = &gUSBHostHubObj[iterator];
			memset(hubInstanceObj, 0, sizeof( USB_HOST_HUB_INSTANCE_OBJ ));
            hubInstanceObj->inUse = true;
            hubInstanceObj->hubObjHandle = deviceObjHandle;
            hubInstanceObj->hubDeviceClientHandle = deviceHandle;
            break;
        }
    }

    /* Validate the instance */
    if(hubInstanceObj == NULL)
    {
        /* This means an instance could not be allocated. Return the device back
         * to the host */
        USB_HOST_DeviceRelease(deviceHandle);

        /* Assert on invalid HUB index */
        SYS_DEBUG_MESSAGE (SYS_ERROR_INFO, "\r\nUSB Hub Driver: Insufficient Hub Instances" );
        
    }
    else
    {
        /* We could find a free hub instance object. Continue with the assign
         * operation. Set all pipe handles to invalid. */

        hubInstanceObj->controlPipeHandle = USB_HOST_CONTROL_PIPE_HANDLE_INVALID;
        hubInstanceObj->interruptInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        hubInstanceObj->controlRequestDone = false;
        
        /* Set the initial task states for the hub device task and the hub task
         * */
        hubInstanceObj->hubInstanceState = USB_HOST_HUB_STATE_DEVICE_STATUS_GET ;
        hubInstanceObj->hubTaskState = USB_HOST_HUB_TASK_STATE_HUB_STATUS_GET;
       
        /* Intialize all the port objects. Set the device handle to invalid.
         * Initialize the port task state and set the timer handle to invalid. */
        for( portIndex = 0 ; portIndex < USB_HOST_HUB_PORTS_NUMBER ; portIndex++ )
        {
            portInfo = & (hubInstanceObj->portInfo[portIndex]);
            portInfo->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID ;
            portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_POWER_ENABLE ;
            portInfo->timerHandle = SYS_TMR_HANDLE_INVALID;
        }

        /* Try opening a control pipe */
        hubInstanceObj->controlPipeHandle = USB_HOST_DeviceControlPipeOpen( deviceObjHandle );

        if( USB_HOST_CONTROL_PIPE_HANDLE_INVALID == hubInstanceObj->controlPipeHandle )
        {
            /* The control pipe could not be opened. Give the device back to the
             * host layer and deallocate the hub object. */

            USB_HOST_DeviceRelease(deviceHandle);
            hubInstanceObj->inUse = false;
            SYS_DEBUG_MESSAGE (SYS_ERROR_INFO, "\r\nUSB Hub Driver: Could not open Control Pipe" );
        }
        else
        {
            /* Control pipe was opened successfully. Check if the hub has
             * atleast one configuration. */

            if(deviceDescriptor->bNumConfigurations > 0)
            {
                /* Yes it does. We can proceed */
                hubInstanceObj->hubInstanceState = USB_HOST_HUB_STATE_DEVICE_STATUS_GET;

            }
            else
            {
                /* This hub does not have any configurations. We cannot proceed.
                 * Release the device and deallocate the hub instance */
                USB_HOST_DeviceRelease(deviceHandle);
                hubInstanceObj->inUse = false;
            }
        }
    
        return;
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_InterfaceAssign 
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

void _USB_HOST_HUB_InterfaceAssign 
(
    USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    size_t nInterfaces,
    uint8_t * descriptor 
)
{
    int hubInstanceIndex;
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle;
    USB_HOST_ENDPOINT_DESCRIPTOR_QUERY endpointQuery;
    USB_ENDPOINT_DESCRIPTOR *interruptEndpointDescriptor;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstanceObj = NULL;

    /* Hub driver will support only one interface which is the first interface
     * in the interfaces table. interfaces in this case will only contain one
     * interface handle. */
    
    interfaceHandle = interfaces[0];

    /* Find the hub instance */
    hubInstanceIndex = _USB_HOST_HUB_DeviceHandleToInstance ( deviceObjHandle );

    if ( hubInstanceIndex < 0  )
    {
        /* Dont know what this interface is. Return it back */
        USB_HOST_DeviceInterfaceRelease(interfaceHandle);
    }
    else
    {
        /* Get the instance object  */
        hubInstanceObj = &gUSBHostHubObj[hubInstanceIndex];
        
        /* Get the interface descriptor pointer */
        interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)(descriptor);
        
        /* Clear the query values */ 
        USB_HOST_DeviceEndpointQueryContextClear(&endpointQuery);
       
        /* Find the interrupt IN endpoint. To do this, first setup the
         * endpoint descriptor query */

        endpointQuery.transferType = USB_TRANSFER_TYPE_INTERRUPT;
        endpointQuery.direction = USB_DATA_DIRECTION_DEVICE_TO_HOST;
        endpointQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE | USB_HOST_ENDPOINT_QUERY_BY_DIRECTION ;

        /* Now find the endpoint */
        interruptEndpointDescriptor =  USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointQuery);

        if (NULL == interruptEndpointDescriptor)
        {
            /* The interface could not be found. Let the host know that this
             * interface cannot be processed. */
            USB_HOST_DeviceInterfaceRelease(interfaceHandle);

            /* Assert on invalid interrupt endpoint descriptor  */
            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not find Interrupt IN endpoint" );
        }
        else
        {
            /* We have found the IN interrupt endpoint. Try opening a pipe on this
             * endpoint */
            hubInstanceObj->interruptInPipeHandle = USB_HOST_DevicePipeOpen ( interfaceHandle , 
                        interruptEndpointDescriptor->bEndpointAddress );

            if (  USB_HOST_PIPE_HANDLE_INVALID == hubInstanceObj->interruptInPipeHandle  )
            {
                /* The pipe could not be opened. Let the host know that this
                 * interface cannot be processed */

                USB_HOST_DeviceInterfaceRelease(interfaceHandle);
                
                /* Assert on invalid PipeHandle */
                SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not open Interrupt IN pipe");
            }
        }
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_DeviceRelease 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceHandle
    )
 
  Summary: 
    This function is called when the device is detached. 

  Description:
    This function is called when the device is detached. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_DeviceRelease
(
    USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceHandle
)
{
    int index;
    uint8_t  portNumber;
    uint8_t  portIndex;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_HUB_PORT_INFO    * portInfo;

    /* Map the device handle to a hub instance */
    index = _USB_HOST_HUB_DeviceHandleToInstance(hubDeviceHandle);

    /* Validate the index */
    if(index >= 0)
    {
        hubInstance = &gUSBHostHubObj[index];

        /* Denumerate any device that is attached to the hub port */
        for ( portNumber = 1 ; portNumber <= hubInstance->hubDescriptor.bNbrPorts ; portNumber++ )
        {
            portIndex = portNumber - 1;
            portInfo = & ( hubInstance->portInfo[portIndex] );
            if ( portInfo->deviceObjHandle != USB_HOST_DEVICE_OBJ_HANDLE_INVALID )
            {
                /* Denumerate the device. If the attached device is a hub, then
                 * the denumerate function will in turn denumerate the the
                 * devices connected to the hub. */
                USB_HOST_DeviceDenumerate( portInfo->deviceObjHandle );
            }
        }

        if(hubInstance->interruptInPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
        {
            /* Close the interruptPipeHandle */
            USB_HOST_DevicePipeClose(hubInstance->interruptInPipeHandle);
        }

        /* Make instance invalid */
        hubInstance->hubObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
        hubInstance->hubDeviceClientHandle = USB_HOST_DEVICE_CLIENT_HANDLE_INVALID;
        hubInstance->inUse = false;
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_DeviceTasks 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceHandle
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

void _USB_HOST_HUB_DeviceTasks
(
    USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceHandle
)
{
    int hubInstanceIndex ;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_RESULT             result;
    USB_HOST_TRANSFER_HANDLE    transferHandle;
    uint8_t                     portNumber;

    /* Get the index of the device instance */
    hubInstanceIndex = _USB_HOST_HUB_DeviceHandleToInstance(hubDeviceHandle);

    if(hubInstanceIndex >= 0)
    {
        /* Get a pointer to the instance */
        hubInstance = &gUSBHostHubObj[hubInstanceIndex];

        if(hubInstance->inUse)
        {
            /* Instance is valid */
            switch(hubInstance->hubInstanceState )
            {
                case USB_HOST_HUB_STATE_DEVICE_STATUS_GET:

                    /* We start by getting the hub device status. This will tell
                     * us if the hub is presently self or bus powered */

                    hubInstance->controlRequestDone = false;
                    result = _USB_HOST_HUB_HubDeviceStatusGet(hubInstanceIndex);
                    if(result == USB_HOST_RESULT_SUCCESS)
                    {
                        /* The result was successful. Change state to waiting for
                         * Device Status Get */

                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_WAIT_FOR_DEVICE_STATUS_GET;
                    }
                    else if ( result == USB_HOST_RESULT_REQUEST_BUSY)
                    {
                        /* Keep try if another control request is processing */
                        break;
                    }
                    else 
                    {
                        /* The control transfer could not be scheduled */
                        _USB_HOST_HUB_DeviceRelease( hubDeviceHandle );
                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not get Hub Device Status" );
                    }
                    break;

                case USB_HOST_HUB_STATE_WAIT_FOR_DEVICE_STATUS_GET:

                    /* Here we are waiting to get the Hub Device Status */
                    
                    if( hubInstance->controlRequestDone == true) 
                    {
                        if( hubInstance->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                        {
                            /* We got the Hub Device Status. This will tell us
                             * if the hub is presently self or bus powered .
                             * Next we set the configuration */

                            hubInstance->hubInstanceState = USB_HOST_HUB_STATE_CONFIGURATION_SET;
                            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Setting Hub Configuration" );
                        }
                        else
                        {
                            /* If we could not set the configuration, then state
                             * instance state to error */
                            _USB_HOST_HUB_DeviceRelease( hubDeviceHandle );
                            hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not set Device Configuration" );
                        }
                    }
                    break;


                case USB_HOST_HUB_STATE_CONFIGURATION_SET:

                    /* The instance should set the configuration. We clear the
                     * controlTransferDone flag. This will be set in the device
                     * event handler when the configuration set event is
                     * received. The USB_HOST_DeviceConfigurationSet function
                     * takes the index of the configuration to be set. We always
                     * try to set the first configuration. */

                    hubInstance->controlRequestDone = false;
                    result = USB_HOST_DeviceConfigurationSet(hubInstance->hubObjHandle ,
                            &( hubInstance->controlTransferHandle), 
                            0 , hubInstanceIndex );

                    if(result == USB_HOST_RESULT_SUCCESS)
                    {
                        /* The result was successful. Change state to wating for
                         * configuration. */

                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_WAIT_FOR_CONFIGURATION_SET;
                    }
                    else if ( result == USB_HOST_RESULT_REQUEST_BUSY)
                    {
                        /* Keep try if another control request is processing */
                        break;
                    }
                    else 
                    {
                        /* The Configuration Set request failed. */
                        _USB_HOST_HUB_DeviceRelease( hubDeviceHandle );
                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not set Device Configuration" );
                    }
                    break;

                case USB_HOST_HUB_STATE_WAIT_FOR_CONFIGURATION_SET:
                    /* Here we are waiting for the configuration to be set */
                    if( hubInstance->controlRequestDone == true) 
                    {
                        if( hubInstance->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                        {
                            /* The configuration has been set. Now wait for the host
                             * hub events */

                            hubInstance->hubInstanceState = USB_HOST_HUB_STATE_WAIT_FOR_INTERFACE_READY;
                            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Waiting for Interface Ready" );
                        }
                        else
                        {
                            /* If we could not set the configuration, then state
                             * instance state to error */
                            _USB_HOST_HUB_DeviceRelease( hubDeviceHandle );
                            hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not set Device Configuration" );
                        }
                    }
                    break;

                case USB_HOST_HUB_STATE_WAIT_FOR_INTERFACE_READY:

                    /* Here we wait for interfaces to get ready. The interrupt
                     * pipe handle becomes valid in the
                     * _USB_HOST_HUB_InterfaceAssign() function. */

                    if( hubInstance->interruptInPipeHandle != USB_HOST_PIPE_HANDLE_INVALID) 
                    {
                        SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Getting Hub Descriptor" );
                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_HUB_DESCRIPTOR_GET ;
                    }
                    break;

                case USB_HOST_HUB_STATE_HUB_DESCRIPTOR_GET:

                    /* The configuration has been set. Now get the Hub
                     * Descriptor. */

                    hubInstance->controlRequestDone = false;
                    result =  _USB_HOST_HUB_HubDescriptorGet( hubInstanceIndex );
                    if ( result == USB_HOST_RESULT_SUCCESS )
                    {
                        SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Waiting for Hub Descriptor Get" );
                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_WAIT_FOR_HUB_DESCRIPTOR_GET;
                    }
                    else if ( result !=  USB_HOST_RESULT_REQUEST_BUSY )
                    {
                        _USB_HOST_HUB_DeviceRelease( hubDeviceHandle );
                        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not get Hub descriptor" );
                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                    }
                    break;

                case USB_HOST_HUB_STATE_WAIT_FOR_HUB_DESCRIPTOR_GET:
                    if( hubInstance->controlRequestDone == true) 
                    {
                        if( hubInstance->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                        {
                            /* Hub descriptor having the information about 
                               No of ports , logical power switching , over
                               current protection mode , port indicator support
                               , power on 2 power goods values */
                            SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Hub is in a running state" );
                            hubInstance->hubInstanceState = USB_HOST_HUB_STATE_RUNNING;
                        
                            /* Ready for launch interrupt transfer  */
                            USB_HOST_DeviceTransfer( hubInstance->interruptInPipeHandle,
                                    &transferHandle,&( hubInstance->changeStatus ), 1 , hubInstanceIndex );
                        }
                        else
                        {
                            /* If we could not set the configuration, then state
                             * instance state to error */

                            hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                        }
                    }
                    break;

                case USB_HOST_HUB_STATE_RUNNING:

                    /* The interrupt transfer is launched only if all the
                     * previous change status bits have been processed and if
                     * there is no interrupt transfer in progress. */
                    if(( hubInstance->changeStatus == 0) && (hubInstance->interruptTransferDone == true))
                    {
                        hubInstance->interruptTransferDone = false ;

                        /* All the change status bits have been handled by the
                         * port and hub tasks. We can now re-launch the
                         * interrupt IRP. */

                        USB_HOST_DeviceTransfer( hubInstance->interruptInPipeHandle,
                                &hubInstance->interruptTransferHandle,
                                &( hubInstance->changeStatus ), 1 , hubInstanceIndex );
                    }
                    
                    /* Hub task handles the hub level change bits */
                    _USB_HOST_HUB_HubTasks( hubInstanceIndex );

                    /* Port Tasks handles the port level change bits */
                    for ( portNumber = 1 ; portNumber <= hubInstance->hubDescriptor.bNbrPorts ; portNumber++ )
                    {
                        _USB_HOST_HUB_PortTasks( hubInstanceIndex , portNumber );
                    }
                    break;

                case USB_HOST_HUB_STATE_ERROR:
                    break;

                default:
                    break;
            }
        }
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_HubTasks( uint32_t hubInstanceIndex )
 
  Summary: 
    This function maintains the hub level status.

  Description:
    This function maintains the hub level status. It is called periodically when
    the hub state is USB_HOST_HUB_STATE_RUNNING. It is called from the
    _USB_HOST_HUB_DeviceTasks() function. This function checks if the hub power
    status has changed or if an over current has occurred. It clear the hub
    change bits if required.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_HubTasks( uint32_t hubInstanceIndex )
{
    USB_HOST_RESULT             result;
    uint8_t                     portNumber;
    uint8_t                     portIndex;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_HUB_PORT_INFO    * portInfo;

    hubInstance = &gUSBHostHubObj[hubInstanceIndex]; 

    /* The changeStatus is a bitmap that is received from the hub through the
     * interrupt transfers. If bit 0 is set, it means that something has changed
     * in the hub at a hub level and this need to be checked. */

    if (( hubInstance->changeStatus & 0x01 ) == 1)
    {
        switch ( hubInstance->hubTaskState )
        {
            case USB_HOST_HUB_TASK_STATE_HUB_STATUS_GET:

                /* Start by getting the hub status */

                hubInstance->controlRequestDone = false;
                result =  _USB_HOST_HUB_StatusRequest(hubInstanceIndex, &( hubInstance->hubStatus), 0, USB_HOST_HUB_REQUEST );

                if(result == USB_HOST_RESULT_SUCCESS)
                {
                    /* Wait for the hub status get request to complete */
                    hubInstance->hubTaskState = USB_HOST_HUB_TASK_STATE_WAIT_FOR_HUB_STATUS_GET ;
                }
                else if( result != USB_HOST_RESULT_REQUEST_BUSY )
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not get hub status" );
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                }
                break;

            case USB_HOST_HUB_TASK_STATE_WAIT_FOR_HUB_STATUS_GET :

                /* Wait for till hub status get */
                if( hubInstance->controlRequestDone == true) 
                {
                    if( hubInstance->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                    {
                        /* Validate any power status changes */
                        if( hubInstance->hubStatus.localPowerStatusChange == 1 )
                        {
                            /* Update the hub power status flag with the status
                             * in wHubStatus and then clear the change bit. */

                            hubInstance->isBusPowered = hubInstance->hubStatus.localPowerSource;
                            hubInstance->hubTaskState = USB_HOST_HUB_TASK_STATE_LPS_CHANGE_CLEAR ;
                        }
                        else
                        {
                            /* There was no change in the local power status.
                             * Check any for current status */
                            hubInstance->hubTaskState = USB_HOST_HUB_TASK_STATE_CHECK_OC_STATUS ;
                        }
                    }
                    else
                    {
                        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not get hub status" );
                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                    }
                }

                break;

            case USB_HOST_HUB_TASK_STATE_LPS_CHANGE_CLEAR :      

                /* Send a control transfer to clear the C_HUB_LOCAL_POWER bit */
                hubInstance->controlRequestDone = false;
                result = _USB_HOST_HUB_FeatureRequest ( hubInstanceIndex, 0 , USB_HUB_CLASS_FEATURE_C_HUB_LOCAL_POWER, 
                        USB_HUB_CLASS_REQUEST_CLEAR_FEATURE, USB_HOST_HUB_REQUEST );

                if(result == USB_HOST_RESULT_SUCCESS)
                {
                    /* Wait for the control transfer to complete */
                    hubInstance->hubTaskState = USB_HOST_HUB_TASK_STATE_WAIT_FOR_LPS_CHANGE_CLEAR ;
                }
                else if( result != USB_HOST_RESULT_REQUEST_BUSY )
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not clear C_HUB_LOCAL_POWER" );
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                }
                break;

            case USB_HOST_HUB_TASK_STATE_WAIT_FOR_LPS_CHANGE_CLEAR:
                if( hubInstance->controlRequestDone == true) 
                {
                    if( hubInstance->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                    {
                        /* We have clear the C_HUB_LOCAL_POWER bit. Check if a
                         * Hub Level OC event has occurred. */
                        hubInstance->hubTaskState = USB_HOST_HUB_TASK_STATE_CHECK_OC_STATUS ;
                    }
                    else
                    {
                        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not clear C_HUB_LOCAL_POWER" );
                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                    }
                }
                break;

            case USB_HOST_HUB_TASK_STATE_CHECK_OC_STATUS:

                if( hubInstance->hubStatus.overCurrentChange == 1 )
                {
                    /* This means an hub level over current event has occurred.
                     * We should clear the C_HUB_OVER_CURRENT bit. */

                    hubInstance->hubTaskState = USB_HOST_HUB_TASK_STATE_OC_CHANGE_CLEAR_SEND ;
                    
                    /* Validate any over current status */
                    if ( hubInstance->hubStatus.overCurrent == 1 )
                    {
                        /* This means hub level overcurrent has occurred. The
                         * hub would have powered down all the ports. We should
                         * denumerate each attached device and then place the
                         * port in a overcurrent state. */

                        for ( portNumber = 1 ; portNumber <= hubInstance->hubDescriptor.bNbrPorts ; portNumber++ )
                        {
                            portIndex = portNumber - 1;
                            portInfo = & ( hubInstance->portInfo[portIndex]);
                            if ( portInfo->deviceObjHandle != USB_HOST_DEVICE_OBJ_HANDLE_INVALID )
                            {
                                /* Denumerate the device and change port task state to overcurrent */
                                USB_HOST_DeviceDenumerate(portInfo->deviceObjHandle );
                                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_OVERCURRENT;
                            }
                        }
                        hubInstance->isOverCurrent = true ;
                    }
                }
                else
                {
                    /* We have checked all possible hub change bits. There is
                     * nothing else to change. Clear the bit 0 in changeStatus
                     * and wait for new status. */

                    hubInstance->changeStatus = ( hubInstance->changeStatus & ~0x01 );
                    hubInstance->hubTaskState = USB_HOST_HUB_TASK_STATE_HUB_STATUS_GET ;
                }
                break;

            case USB_HOST_HUB_TASK_STATE_OC_CHANGE_CLEAR_SEND :

                hubInstance->controlRequestDone = false;

                /* Send a control transfer to clear the C_HUB_OVER_CURRENT bit.
                 * */
                result = _USB_HOST_HUB_FeatureRequest ( hubInstanceIndex, 0 , USB_HUB_CLASS_FEATURE_C_HUB_OVER_CURRENT, 
                        USB_HUB_CLASS_REQUEST_CLEAR_FEATURE, USB_HOST_HUB_REQUEST );

                if(result == USB_HOST_RESULT_SUCCESS)
                {
                    /* Wait for the C_HUB_OVER_CURRENT to clear */
                    hubInstance->hubTaskState =  USB_HOST_HUB_TASK_STATE_WAIT_FOR_OC_CHANGE_CLEAR;
                }
                else if( result != USB_HOST_RESULT_REQUEST_BUSY )
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not clear C_HUB_OVER_CURRENT" );
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                }

                break;

            case USB_HOST_HUB_TASK_STATE_WAIT_FOR_OC_CHANGE_CLEAR:

                if( hubInstance->controlRequestDone == true) 
                {
                    if( hubInstance->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                    {
                        /* We successfully cleared the C_HUB_OVER_CURRENT bit.
                         * Now check if the overCurrent status is false and if
                         * the hub was in overCurrent state. This means the over
                         * current was there but is now cleared. */

                        if ((hubInstance->isOverCurrent == true) && (hubInstance->hubStatus.overCurrent == false))
                        {
                            /* Power up all the ports. This will automatically
                             * start the enumeration process for any attached
                             * devices. The port powering process is taken up in
                             * the _USB_HOST_HUB_PortTasks() function */

                            hubInstance->isOverCurrent = false;
                            for ( portNumber = 1 ; portNumber <= hubInstance->hubDescriptor.bNbrPorts ; portNumber++ )
                            {
                                portIndex = portNumber - 1;
                                portInfo = &(hubInstance->portInfo[portIndex]);
                                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_POWER_ENABLE;
                            }
                        }

                        /* Clear the hub change status bit and get ready for the
                         * next change status. */
                        hubInstance->changeStatus = ( hubInstance->changeStatus & ~0x01 );
                        hubInstance->hubTaskState = USB_HOST_HUB_TASK_STATE_HUB_STATUS_GET ;
                    }
                    else
                    {
                        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not clear C_HUB_OVER_CURRENT" );
                        hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
                    }
                }

                break;
            default :
                break;
        }
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_PortTasks(uint32_t hubInstanceIndex, uint32_t portNumber)
 
  Summary: 
    This function maintains the port level status.

  Description:
    This function maintains the port level status. It is called periodically
    when the hub state is USB_HOST_HUB_STATE_RUNNING. It is called from the
    _USB_HOST_HUB_DeviceTasks() function. This function checks the state of all
    the change bits in wPortChange and responds to each change source.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_PortTasks(uint32_t hubInstanceIndex, uint32_t portNumber)
{
    uint8_t  mask;
    uint8_t portIndex;
    uint8_t tempPortIndex;
    uint8_t tempPortNumber;
    USB_HOST_RESULT result;
    USB_HOST_HUB_PORT_INFO * portInfo;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;

    /* Get a pointer to the instance */
    portIndex = portNumber - 1;
    hubInstance = &gUSBHostHubObj[hubInstanceIndex];
    portInfo = &( hubInstance->portInfo[portIndex] );

    switch ( portInfo->portTaskState )
    {
        case USB_HOST_HUB_PORT_TASK_STATE_POWER_ENABLE:

            /* First the power to the port needs to be enabled */
            portInfo->controlRequestDone = false;

            /* Send a control transfer to Set Feature (Port Power). */
            result = _USB_HOST_HUB_FeatureRequest( hubInstanceIndex , portNumber , USB_HUB_CLASS_FEATURE_PORT_POWER , 
                    USB_HUB_CLASS_REQUEST_SET_FEATURE , USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the port power to be enabled */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_POWER_ENABLE ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not enable port power" );
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_POWER_ENABLE:

            if( portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    if( hubInstance->hubDescriptor.bPwrOn2PwrGood > 0 )
                    {
                        /* If the port power switch requires some delay, we
                         * should wait for this delay. Move the port task state
                         * machine to a state that provides this delay */
                        portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_PWRON2PWRGOOD_DELAY;
                    }
                    else
                    {
                        /* This port does not require any delay. We can move to
                         * new state. */
                        portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_CHANGE_STATUS;
                    }
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB Hub Driver: Could not enable port power" );
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_PWRON2PWRGOOD_DELAY:

            /* Start a delay of defined by the bPwrOn2PwrGood field in the hub
             * descriptor. */

            portInfo->timerHandle = SYS_TMR_DelayMS(2 * hubInstance->hubDescriptor.bPwrOn2PwrGood );
            if( SYS_TMR_HANDLE_INVALID != portInfo->timerHandle )
            {
                /* The delay could be started. Wait for the delay to complete */
                portInfo->portTaskState =  USB_HOST_HUB_PORT_TASK_STATE_PWRON2PWRGOOD_DELAY_WAIT;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_PWRON2PWRGOOD_DELAY_WAIT:

            if(SYS_TMR_DelayStatusGet( portInfo->timerHandle))
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Port is powered up ");
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_CHANGE_STATUS;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_OVERCURRENT:

            /* The port is placed in this state when a hub level over current
             * event has been detected. The _USB_HOST_HUB_HubTasks() will place
             * all ports in this state when the over current condition is
             * active. */

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CHECK_CHANGE_STATUS:

            /* In this state we check if there is any change in the port status
             * This is known by checking the corresponding port bit in the
             * changeStatus bitmap. */

            mask = 1;
            mask = mask << portNumber;
            if ((hubInstance->changeStatus & mask ) )
            {
                /* There is a change. Find out what the change is. */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_PORT_STATUS_GET ;
            }
            else
            {
                /* There is no change. Check if any command has to be executed
                 * on the port. */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_COMMAND ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_PORT_STATUS_GET:

            portInfo->controlRequestDone = false;

            /* Send a request to read the port status. */
            result = _USB_HOST_HUB_StatusRequest( hubInstanceIndex, &(portInfo->portStatus),
                    portNumber ,USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait till we get the port status. */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_STATUS_GET ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not read port status for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_STATUS_GET:

            if( portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    if( portInfo->portStatus.connectStatusChange == 1 )
                    {
                        if( portInfo->portStatus.currentConnectStatus == 1 )
                        {
                            /* Device is connected. Call the
                             * USB_HOST_DeviceEnumerate() to enumerate the
                             * device. The function will return an device object
                             * handle which the port uses to identify the device
                             * to the host. */
                            
                            if(USB_HOST_DEVICE_OBJ_HANDLE_INVALID == portInfo->deviceObjHandle)
                            {
                                /* This means this is a fresh attach. There was
                                 * no device attached to this port. */
                                portInfo->deviceObjHandle = USB_HOST_DeviceEnumerate( hubInstance->hubObjHandle, portNumber) ;
                            }
                            else
                            {
                                /* Consecutive enumeration requests have come
                                 * without de-enumeration. De-enumerate and 
                                 * enumerate the device again. The hub may send
                                 * an device attach notification on the port
                                 * without a detach notification. The fact that
                                 * the port deviceObjHandle handle is still
                                 * valid indicates that the presently attached
                                 * device was not detached. The hub may do this
                                 * in a case where the device was detached and
                                 * attached rapidly. */

                                USB_HOST_DeviceDenumerate(portInfo->deviceObjHandle );
                                portInfo->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
                                
                                portInfo->deviceObjHandle = USB_HOST_DeviceEnumerate( hubInstance->hubObjHandle, portNumber) ;
                            }
                        }
                        else
                        {
                            /* Device is disconnected. Disconnect the device. */
                            if( USB_HOST_DEVICE_OBJ_HANDLE_INVALID != portInfo->deviceObjHandle )
                            {
                                USB_HOST_DeviceDenumerate(portInfo->deviceObjHandle );
                                portInfo->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
                            }

                        }

                        /* Clear the C_PORT_CONNECT bit */
                        portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_CONNECT ; 
                    }
                    else
                    {
                        /* If the connectStatusChange bit is not set, check if
                         * port enable disable has changed. */ 

                        portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_PORT_ENABLE_CHANGE;
                    }
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not read port status for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_CONNECT:

            portInfo->controlRequestDone = false;
                   
            /* Send a control transfer to clear the C_PORT_CONNECT bit */
            result =_USB_HOST_HUB_FeatureRequest( hubInstanceIndex, portNumber, 
                    USB_HUB_CLASS_FEATURE_C_PORT_CONNECTION, USB_HUB_CLASS_REQUEST_CLEAR_FEATURE, 
                    USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the Clear Feature C_PORT_CONNECT request to complete
                 * */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_CLEAR_FEATURE_PORT_CONNECT ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not clear feature C_PORT_CONNECT for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_CLEAR_FEATURE_PORT_CONNECT:

            if(  portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* We sent a control transfer to clear the port connect
                     * change status bit. Check if this has been cleared */

                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_POST_PORT_CONNECT_STATUS_GET ;
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not clear feature C_PORT_CONNECT for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_POST_PORT_CONNECT_STATUS_GET:

            /* After the clear feature port connect control trasnfer has
             * completed, we get the port status to make sure that the bit has
             * cleared. */

            portInfo->controlRequestDone = false;

            /* Send a request to read the port status. */
            result = _USB_HOST_HUB_StatusRequest( hubInstanceIndex, &(portInfo->portStatus),
                    portNumber ,USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait till we get the port status. */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_POST_PORT_CONNECT_STATUS_GET;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not read port status for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_POST_PORT_CONNECT_STATUS_GET:

            /* Here we are waiting for the post clear feature port connect port
             * status get control transfer to complete. If the change bit has
             * not cleared, then we should wait till that happens */
            
            if( portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* Check if the port connect change bit has cleared */
                    if(portInfo->portStatus.connectStatusChange == 0)
                    {
                        /* The connect change status bit has cleared. We can
                         * move to the next state */

                        portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_PORT_ENABLE_CHANGE;
                    }
                    else
                    {
                        /* The status has not changed yet. We should read the
                         * port status again. */
                        portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_POST_PORT_CONNECT_STATUS_GET;
                    }
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not read port status for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CHECK_PORT_ENABLE_CHANGE:

            if( portInfo->portStatus.portEnableDisableChange == 1 )
            {
                /* This change status will get set if the port has been
                 * disabled. We only capture the status here and then 
                 * clear the enable change status. */

                portInfo->isEnabled = portInfo->portStatus.portEnabledDisabled; 
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_ENABLE_CHANGE ;
            }
            else
            {
                /* If the port has not been disabled, check if there is a change
                 * in suspend status. */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_SUSPEND_CHANGE;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_ENABLE_CHANGE:

            portInfo->controlRequestDone = false;

            /* Send a control transfer to clear port feature C_PORT_ENABLE */
            result =_USB_HOST_HUB_FeatureRequest( hubInstanceIndex , portNumber , USB_HUB_CLASS_FEATURE_C_PORT_ENABLE , 
                    USB_HUB_CLASS_REQUEST_CLEAR_FEATURE , USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the control transfer to complete. */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_ENABLE_CHANGE ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not clear feature C_PORT_ENABLE for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_ENABLE_CHANGE:

            if(  portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* We have checked port connct and port enable. Now check
                     * port suspend */
                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_SUSPEND_CHANGE ;
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not clear feature C_PORT_ENABLE for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CHECK_SUSPEND_CHANGE:

            if( portInfo->portStatus.suspendChange == 1 )
            {
                /* This means the port has resumed, that is it is no more
                 * suspended. The suspend change status should be cleared. */

                portInfo->isResumed = true;
                portInfo->isSuspended = false; 
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_SUSPEND_CHANGE ;
            }
            else
            {
                /* There was no suspend change. Check if there was an over
                 * current change */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_OC_CHANGE ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_SUSPEND_CHANGE:

            portInfo->controlRequestDone = false;

            /* Send a control transfer to clear feature C_PORT_SUSPEND */ 
            result =_USB_HOST_HUB_FeatureRequest( hubInstanceIndex , portNumber, USB_HUB_CLASS_FEATURE_C_PORT_SUSPEND, 
                    USB_HUB_CLASS_REQUEST_CLEAR_FEATURE, USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the control transfer to complete */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_SUSPEND_CHANGE ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not clear feature C_PORT_SUSPEND for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_SUSPEND_CHANGE:

            if(  portInfo->controlRequestDone == true) 
            {
                if(portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* We have checked for port connect, enable, suspend change.
                     * We should now check for OC change. */
                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_OC_CHANGE ;
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not clear feature C_PORT_SUSPEND for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CHECK_OC_CHANGE:

            if ( portInfo->portStatus.overCurrentIndicatorChange == 1 )
            {
                /* This means an over current condition has changed on this
                 * port. The OC change status should be cleared. */

                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_OC_CHANGE ;
                if( portInfo->portStatus.overCurrent == 1  )
                {
                    portInfo->overCurrent = true ;

                    /* If OC change bit is set and OC status bit is set, then
                     * this is port is in an OC state. The set overCurrent field
                     * lets us keep track of this port. */

                    if ( USB_HOST_DEVICE_OBJ_HANDLE_INVALID !=  portInfo->deviceObjHandle )
                    {
                        /* Denumerate the attached device. Set the
                         * isOCPoweredOff field to know that this port is
                         * switched off because of over current. */
                        USB_HOST_DeviceDenumerate(portInfo->deviceObjHandle );
                        portInfo->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
                        portInfo->isOCPoweredOff = true ;
                    }
                }
                else 
                {
                    if (portInfo->overCurrent == false)
                    {
                        /* This means this is an affected port. This overcurrent
                         * change status has been set but the overcurrent status
                         * is not set. We should denumerate the device. */
                        if ( USB_HOST_DEVICE_OBJ_HANDLE_INVALID !=  portInfo->deviceObjHandle )
                        {
                            USB_HOST_DeviceDenumerate(portInfo->deviceObjHandle );
                            portInfo->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
                            portInfo->isOCPoweredOff = true ;
                        }
                    }
                }
            }
            else
            {
                /* If there was no over current situation, then check if reset
                 * has complete. */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_PORT_RESET_CHANGE ;
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_OC_CHANGE:

            /* Send a control tranfer to clear the OC change status */
            portInfo->controlRequestDone = false;
            result =_USB_HOST_HUB_FeatureRequest( hubInstanceIndex ,
                    portNumber , USB_HUB_CLASS_FEATURE_C_PORT_OVER_CURRENT , 
                    USB_HUB_CLASS_REQUEST_CLEAR_FEATURE , 
                    USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the control transfer to complete */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_OC_CHANGE ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\n_USB_HOST_HUB_PortTasks: Could not clear feature C_PORT_OVER_CURRENT for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_OC_CHANGE:

            if(  portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    if ((portInfo->overCurrent == true) && (portInfo->portStatus.overCurrent == 0))
                    {
                        /* The overCurrent flag is true, the overCurrent status
                         * is false and we have cleared the over current change. This
                         * means the over current condition on this port has
                         * cleared. We must power up any ports that have been
                         * powered off due to over current. */
                        for ( tempPortNumber = 1 ; tempPortNumber <= hubInstance->hubDescriptor.bNbrPorts ; tempPortNumber++ )
                        {
                            tempPortIndex = tempPortNumber - 1;
                            if ( hubInstance->portInfo[tempPortIndex].isOCPoweredOff == true )
                            {
                                /* Enable the power */
                                hubInstance->portInfo[tempPortIndex].portTaskState = USB_HOST_HUB_PORT_TASK_STATE_POWER_ENABLE;
                                hubInstance->portInfo[tempPortIndex].isOCPoweredOff = false ;
                            }
                        }
                    }

                    /* We next check the reset change bit */
                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_PORT_RESET_CHANGE;
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\nUSB Hub Driver: Could not clear feature C_PORT_OVER_CURRENT for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CHECK_PORT_RESET_CHANGE:

            if( portInfo->portStatus.resetChange == 1 )
            {
                /* This means a reset sequence has completed. We should read the
                 * port speed. */
                
                if (portInfo->portStatus.lowSpeedDeviceAttached == 1  )
                {
                    portInfo->portSpeed =   USB_SPEED_LOW;
                }
                else if (  portInfo->portStatus.highSpeedDeviceAttached == 1 )
                {
                    portInfo->portSpeed =  USB_SPEED_HIGH;
                }
                else
                {
                    portInfo->portSpeed = USB_SPEED_FULL;
                }

                /* Clear the reset change bit */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_RESET_CHANGE  ;
            }
            else
            {
                /* At this point we have checked all the possible reasons why
                 * the port bit in changeStatus would have been set. We can
                 * clear the bit and now check if there are any port commands
                 * to be processed. */

                mask = 1 ;
                mask = mask << portNumber;
                hubInstance->changeStatus = hubInstance->changeStatus & ~mask ;
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_COMMAND  ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_RESET_CHANGE:    

            /* Send a control transfer to clear the rest change bit */
            portInfo->controlRequestDone = false;
            result =_USB_HOST_HUB_FeatureRequest( hubInstanceIndex ,
                    portNumber , USB_HUB_CLASS_FEATURE_C_PORT_RESET , 
                    USB_HUB_CLASS_REQUEST_CLEAR_FEATURE , 
                    USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the control transfer to complete */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_RESET_CHANGE  ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\n_USB_HOST_HUB_PortTasks: Could not clear feature C_PORT_RESET for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_RESET_CHANGE:

            if(  portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* At this point we have checked all the possible reasons why
                     * the port bit in changeStatus would have been set. We can
                     * clear the bit and now check if there are any port commands
                     * to be processed. */

                    mask = 1 ;
                    mask = mask << portNumber;
                    hubInstance->changeStatus = hubInstance->changeStatus & ~mask ;
                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_POST_RESET_COMPLETE_STATUS_GET;
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\n_USB_HOST_HUB_PortTasks: Could not clear feature C_PORT_RESET for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_POST_RESET_COMPLETE_STATUS_GET:

            /* The clear feature port reset would have completed now. We get the
             * port status and make sure that this bit has been cleared. */
            portInfo->controlRequestDone = false;

            /* Send a request to read the port status. */
            result = _USB_HOST_HUB_StatusRequest( hubInstanceIndex, &(portInfo->portStatus),
                    portNumber ,USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait till we get the port status. */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_POST_RESET_COMPLETE_STATUS_GET;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\n_USB_HOST_HUB_PortTasks: Could not read port status for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_POST_RESET_COMPLETE_STATUS_GET:
             
            /* Here we are waiting for the post clear feature port reset port
             * status get control transfer to complete. If the change bit has
             * not cleared, then we should wait till that happens */
            
            if( portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* Check if the port reset change bit has cleared */
                    if(portInfo->portStatus.resetChange == 0)
                    {
                        /* The reset change status bit has cleared. We can
                         * move to the next state */

                        //portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CLEAR_TT_BUFFER;
                        portInfo->isResetComplete = true;
                        portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_COMMAND;
                    }
                    else
                    {
                        /* The status has not changed yet. We should read the
                         * port status again. */
                        portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_POST_RESET_COMPLETE_STATUS_GET;
                    }
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\n_USB_HOST_HUB_PortTasks: Could not read port status for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CLEAR_TT_BUFFER:

            /* In this state, the Hub Driver will send the clear TT command to
             * the hub. Single TT is assumed. */
            portInfo->controlRequestDone = false;
            result =_USB_HOST_HUB_PortTTBufferClear( hubInstanceIndex, 0, 0, USB_TRANSFER_TYPE_CONTROL, 1, portNumber );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the control transfer to complete */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_TT_BUFFER  ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\n_USB_HOST_HUB_PortTasks: Could not clear port TT buffer");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_TT_BUFFER:

            /* In this state, the Hub Driver will wait for the clear TT command
             * to complete */
            if(  portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    portInfo->isResetComplete = true;
                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_COMMAND  ;
                }
                else
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_DEBUG , "\r\n_USB_HOST_HUB_PortTasks: Could not clear port TT buffer");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_CHECK_COMMAND:

            /* In this state, we check if any port command has been requested.
             * The port task routine will reach this state only if all change
             * sources have been cleared. */

            if ((portInfo->portCommand & USB_HOST_PORT_COMMAND_RESET) == 1)
            {
                /* The port needs to be reset */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_PORT_RESET  ;
            }
            else if ((portInfo->portCommand & USB_HOST_PORT_COMMAND_SUSPEND) == 1)
            {
                /* The port needs to be suspended */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_PORT_SUSPEND  ;
            }
            else if ((portInfo->portCommand & USB_HOST_PORT_COMMAND_RESUME) == 1)
            {
                /* The port needs to be resumed */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_PORT_RESUME  ;
            }
            else 
            {
                /* This means no command has been requested. We can continue to
                 * check status */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_CHANGE_STATUS;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_PORT_RESET:

            /* This state will issue a port reset. Start by clearing the reset
             * bit in port command*/

            portInfo->portCommand = ( portInfo->portCommand & ~USB_HOST_PORT_COMMAND_RESET);
            portInfo->isResetComplete = false;
            portInfo->controlRequestDone = false;

            /* Send the port reset control tranfer */
            result =_USB_HOST_HUB_FeatureRequest( hubInstanceIndex ,
                    portNumber , USB_HUB_CLASS_FEATURE_PORT_RESET , 
                    USB_HUB_CLASS_REQUEST_SET_FEATURE , 
                    USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the control transfer to complete */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_RESET_COMMAND  ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_DEBUG, "\r\n_USB_HOST_HUB_PortTasks: Could not Set Feature PORT_RESET for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_RESET_COMMAND:

            if(  portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* The command completed successfully. Note that the reset
                     * sequence completes when the C_PORT_RESET condition is
                     * set. This is handled in the change status states */

                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_CHANGE_STATUS ;
                }
                else
                {
                    SYS_DEBUG_MESSAGE (SYS_ERROR_DEBUG, "\r\n_USB_HOST_HUB_PortTasks: Could not Set Feature PORT_RESET for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_PORT_SUSPEND:

            /* This state will suspend the port. */

            portInfo->portCommand  = ( portInfo->portCommand & ~USB_HOST_PORT_COMMAND_SUSPEND);
            portInfo->controlRequestDone = false;

            /* Send a control transfer to  suspend the port */
            result =_USB_HOST_HUB_FeatureRequest( hubInstanceIndex ,
                    portNumber , USB_HUB_CLASS_FEATURE_C_PORT_SUSPEND , 
                    USB_HUB_CLASS_REQUEST_SET_FEATURE , 
                    USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the contorl transfer to complete */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_SUSPEND_COMMAND  ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_DEBUG, "\r\n_USB_HOST_HUB_PortTasks: Could not Set Feature PORT_SUSPEND for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_SUSPEND_COMMAND:

            if(  portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* The command completed successfully. Continue to check the
                     * change status */
                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_CHANGE_STATUS ;
                }
                else
                {
                    SYS_DEBUG_MESSAGE (SYS_ERROR_DEBUG, "\r\n_USB_HOST_HUB_PortTasks: Could not Set Feature PORT_SUSPEND for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }
            break;

        case USB_HOST_HUB_PORT_TASK_STATE_PORT_RESUME:

            /* This state starts the port resume sequence. Start by clear the
             * resume bit in port command*/

            portInfo->portCommand  = ( portInfo->portCommand & ~USB_HOST_PORT_COMMAND_RESUME);

            /* Send a control tranfer to resume the port */
            result =_USB_HOST_HUB_FeatureRequest( hubInstanceIndex ,
                    portNumber , USB_HUB_CLASS_FEATURE_C_PORT_SUSPEND , 
                    USB_HUB_CLASS_REQUEST_CLEAR_FEATURE , 
                    USB_HOST_HUB_PORT_REQUEST );

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* Wait for the control transfer to complete */
                portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_RESUME_COMMAND  ;
            }
            else if( result != USB_HOST_RESULT_REQUEST_BUSY )
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_DEBUG, "\r\n_USB_HOST_HUB_PortTasks: Could not Set Feature PORT_RESUME for port");
                hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR ;
            }

            break;

        case USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_RESUME_COMMAND:

            if( portInfo->controlRequestDone == true) 
            {
                if( portInfo->controlRequestResult == USB_HOST_RESULT_SUCCESS)
                {
                    /* The resume command was send. We can now continue to check
                     * the change status */
                    portInfo->portTaskState = USB_HOST_HUB_PORT_TASK_STATE_CHECK_CHANGE_STATUS ;
                }
                else
                {
                    SYS_DEBUG_MESSAGE (SYS_ERROR_DEBUG, "\r\n_USB_HOST_HUB_PortTasks: Could not Set Feature PORT_RESUME for port");
                    hubInstance->hubInstanceState = USB_HOST_HUB_STATE_ERROR;
                }
            }

            break;

        default:
            break;
    }
}

// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HUB_DeviceEventHandler
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

USB_HOST_DEVICE_EVENT_RESPONSE _USB_HOST_HUB_DeviceEventHandler
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    USB_HOST_DEVICE_EVENT event,
    void * eventData,
    uintptr_t context
)
{

    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_DEVICE_EVENT_CONFIGURATION_SET_DATA * configSetEventData;
    uint32_t   hubIndex;

    /* Get the index of the device instance */
    hubIndex =   ( context & HUB_INDEX_MASK );
    /* Get a pointer to the instance */
    hubInstance = &gUSBHostHubObj[hubIndex];

    switch(event)
    {
        case USB_HOST_DEVICE_EVENT_CONFIGURATION_SET:

            configSetEventData = (USB_HOST_DEVICE_EVENT_CONFIGURATION_SET_DATA * )(eventData);
            /* This means the configuration was set. Update the instance
             * variables to let the main state machine know. */
            hubInstance->controlRequestResult = configSetEventData->result;
            hubInstance->controlRequestDone = true;
            break;

        case USB_HOST_DEVICE_EVENT_CONFIGURATION_DESCRIPTOR_GET_COMPLETE:
            break;

        default:
            break;
    }

    return(USB_HOST_DEVICE_EVENT_RESPONSE_NONE);
}

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_InterfaceRelease 
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interface 
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

void _USB_HOST_HUB_InterfaceRelease ( USB_HOST_DEVICE_INTERFACE_HANDLE interface )
{
    /* This function is not implemented in this release of the hub driver */
}

// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HUB_InterfaceEventHandler
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

USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HUB_InterfaceEventHandler
(
      USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
      USB_HOST_DEVICE_INTERFACE_EVENT event,
      void * eventData,
      uintptr_t context
)
{
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    uint32_t   hubIndex;

    hubIndex =   ( context & HUB_INDEX_MASK );
    /* Get a pointer to the instance */
    hubInstance = &gUSBHostHubObj[hubIndex];

    if ( event == USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE )
    {
        hubInstance->interruptTransferDone = true;
    }

    return USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE;
}
  
// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_InterfaceTasks 
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

void _USB_HOST_HUB_InterfaceTasks (USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle)
{
    /* This function is not implemented in this release of the hub driver */
}
  
// ****************************************************************************
/* Function:
    void USB_HOST_HUB_PortReset( uintptr_t deviceHandle, uint8_t portNumber )
    
  Summary:
    Resets the specified root hub port.
	
  Description:
    This function resets the hub port.The status of the reset signalling can be
    checked using the USB_HOST_HUB_PortResetComplete() function.
	
  Remarks:
    This function is typically called by the host layer.
*/

USB_ERROR USB_HOST_HUB_PortReset  ( uintptr_t hubDeviceHandle, uint8_t portNumber )
{
    int index;
    USB_ERROR result = USB_ERROR_NONE;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_HUB_PORT_INFO * portInfo;
    uint32_t portIndex = portNumber-1;

    /* Find the HUB instance object that owns this device */
    index = _USB_HOST_HUB_DeviceHandleToInstance(hubDeviceHandle);

    /* Validate the index */
    if ( index <= -1 )
    {
        /* Assert on invalid HUB index */
        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortReset: Invalid HUB instance number " );
        result =  USB_ERROR_HOST_DEVICE_INSTANCE_INVALID;
    }
    else
    {
        /* Get Hub instance information and the port index */
        hubInstance = &gUSBHostHubObj[index];

        /* Validate hub entered any error conditions */
        if (  hubInstance->hubInstanceState == USB_HOST_HUB_STATE_ERROR )
        {
            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortReset: Hub is in an error state" );
            result = USB_ERROR_HOST_DRIVER_NOT_READY;
        }
        else
        {
            /* Is port valid ?*/
            if( portNumber < 1 || portNumber >  hubInstance->hubDescriptor.bNbrPorts )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortReset: Port is not valid" );
                result = USB_ERROR_HOST_ARGUMENTS_INVALID;
            }
            else
            {
                portInfo = & ( hubInstance->portInfo[portIndex] );

                /* Validate any overCurrent  */
                if( portInfo->overCurrent == true )
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortReset: Port is in an over current state" );
                    result = USB_ERROR_HOST_DRIVER_NOT_READY;
                }
                else
                {
                    /* Set the Reset command flag in the port command  field.
                     * The command will be completed by the
                     * _USB_HOST_HUB_PortTasks() function. */
                    portInfo->portCommand |= USB_HOST_PORT_COMMAND_RESET ;
                    result = USB_ERROR_NONE;
                }
            }
        }
    }

    return result;
}

// ****************************************************************************
/* Function:
    bool USB_HOST_HUB_PortResetComplete 
    (
        uintptr_t deviceHandle,
        uint8_t portNumber
    );

  Summary:
    Returns true if the root hub has completed the port reset operation.

  Description:
    This function returns true if the port reset operation has completed. It
    should be called after the USB_HOST_HUB_PortReset() function to
    check if the reset operation has completed.

  Remarks:
    This function is typically called by the host layer.
*/

bool USB_HOST_HUB_PortResetComplete 
(
   uintptr_t hubDeviceHandle, 
   uint8_t portNumber 
)
{
    int index;
    bool result;
    uint8_t  portIndex = portNumber-1;
    USB_HOST_HUB_PORT_INFO * portInfo;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;

    /* Find the HUB instance object that owns this device */
    index = _USB_HOST_HUB_DeviceHandleToInstance(hubDeviceHandle);

    /* validate the index */
    if ( index <= -1 )
    {
        /* Assert on invalid HUB index */
        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortResetComplete: Invalid HUB instance number " );
        result = false;
    }
    else
    {
        hubInstance = &gUSBHostHubObj[index];
        
        /* Is port number is valid ? */
        if( portNumber < 1 || portNumber >  hubInstance->hubDescriptor.bNbrPorts )
        {
            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortResetComplete: Port Number is not valid " );
            result = false;
        }
        else
        {
            /* Return the reset status of the port */
            portInfo = & ( hubInstance->portInfo[portIndex] );
            result = portInfo->isResetComplete ;
        }
    }

    return result ;
}
 
// ****************************************************************************
/* Function:
    USB_ERROR USB_HOST_HUB_PortSuspend 
    ( 
        uintptr_t deviceHandle, 
        uint8_t portNumber
    )

  Summary:
    Request the hub driver to suspend the port.

  Description:
    This function requests the hub driver to suspend the port. The actual suspend
    operation is completed in the _USB_HOST_HUB_PortTasks() function.
    
  Remarks:
    This function is called by the host layer.
*/

USB_ERROR USB_HOST_HUB_PortSuspend 
(
    uintptr_t hubDeviceHandle, 
    uint8_t portNumber
)
{
    int index;
    USB_ERROR result = USB_ERROR_NONE;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_HUB_PORT_INFO * portInfo;
    uint32_t portIndex = portNumber-1;

    /* Find the HUB instance object that owns this device */
    index = _USB_HOST_HUB_DeviceHandleToInstance(hubDeviceHandle);

    /* Validate the index */
    if ( index <= -1 )
    {
        /* Assert on invalid HUB index */
        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortSuspend: Invalid HUB instance number " );
        result =  USB_ERROR_HOST_DEVICE_INSTANCE_INVALID;
    }
    else
    {
        /* Get Hub instance information and the port index */
        hubInstance = &gUSBHostHubObj[index];

        /* Validate hub entered any error conditions */
        if (  hubInstance->hubInstanceState == USB_HOST_HUB_STATE_ERROR )
        {
            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortSuspend: Hub is in an error state" );
            result = USB_ERROR_HOST_DRIVER_NOT_READY;
        }
        else
        {
            /* Is port valid ?*/
            if( portNumber < 1 || portNumber >  hubInstance->hubDescriptor.bNbrPorts )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortSuspend: Port is not valid" );
                result = USB_ERROR_HOST_ARGUMENTS_INVALID;
            }
            else
            {
                portInfo = & ( hubInstance->portInfo[portIndex] );

                /* Validate any overCurrent  */
                if( portInfo->overCurrent == true )
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortSuspend: Port is in an over current state" );
                    result = USB_ERROR_HOST_DRIVER_NOT_READY;
                }
                else
                {
                    /* Set the Suspen command flag in the port command  field.
                     * The command will be completed by the
                     * _USB_HOST_HUB_PortTasks() function. */
                    portInfo->portCommand |= USB_HOST_PORT_COMMAND_SUSPEND ;
                    result = USB_ERROR_NONE;
                }
            }
        }
    }

    return result;   
}

// ****************************************************************************
/* Function:
    USB_ERROR USB_HOST_HUB_PortResume 
    ( 
        uintptr_t deviceHandle, 
        uint8_t portNumber
    )

  Summary:
    Request the hub driver to resume the port.

  Description:
    This function requests the hub driver to resume the port. The actual resume
    operation is completed in the _USB_HOST_HUB_PortTasks() function.
    
  Remarks:
    This function is called by the host layer.
*/

USB_ERROR USB_HOST_HUB_PortResume 
( 
    uintptr_t hubDeviceHandle, 
    uint8_t portNumber
)
{
    int index;
    USB_ERROR result = USB_ERROR_NONE;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_HUB_PORT_INFO * portInfo;
    uint32_t portIndex = portNumber-1;

    /* Find the HUB instance object that owns this device */
    index = _USB_HOST_HUB_DeviceHandleToInstance(hubDeviceHandle);

    /* Validate the index */
    if ( index <= -1 )
    {
        /* Assert on invalid HUB index */
        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortResume: Invalid HUB instance number " );
        result =  USB_ERROR_HOST_DEVICE_INSTANCE_INVALID;
    }
    else
    {
        /* Get Hub instance information and the port index */
        hubInstance = &gUSBHostHubObj[index];

        /* Validate hub entered any error conditions */
        if (  hubInstance->hubInstanceState == USB_HOST_HUB_STATE_ERROR )
        {
            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortResume: Hub is in an error state" );
            result = USB_ERROR_HOST_DRIVER_NOT_READY;
        }
        else
        {
            /* Is port valid ?*/
            if( portNumber < 1 || portNumber >  hubInstance->hubDescriptor.bNbrPorts )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortResume: Port is not valid" );
                result = USB_ERROR_HOST_ARGUMENTS_INVALID;
            }
            else
            {
                portInfo = & ( hubInstance->portInfo[portIndex] );
            
                /* Validate any overCurrent  */
                if( portInfo->overCurrent == true )
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortResume: Port is in an over current state" );
                    result = USB_ERROR_HOST_DRIVER_NOT_READY;
                }
                else
                {
                    /* Set the Resume command flag in the port command  field.
                     * The command will be completed by the
                     * _USB_HOST_HUB_PortTasks() function. */
                    portInfo->portCommand |= USB_HOST_PORT_COMMAND_RESUME ;
                    result = USB_ERROR_NONE;
                }
            }
        }
    }

    return result;
}

// ****************************************************************************
/* Function:
    USB_SPEED  USB_HOST_HUB_PortSpeedGet
    (
        uintptr_t deviceHandle, 
        uint8_t port
    )

  Summary:
    Returns the speed at which the bus to which this hub is connected is
    operating.

  Description:
    This function returns the speed at which the bus to which this port is
    connected is operating.

 Remarks:
    None.
*/

USB_SPEED  USB_HOST_HUB_PortSpeedGet
(
    uintptr_t deviceHandle, 
    uint8_t portNumber

)
{
    int index;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_SPEED portSpeed;

    uint32_t   portIndex = portNumber-1;

    USB_HOST_HUB_PORT_INFO * portInfo;
    /* Find the HUB instance object that owns this device */
    index = _USB_HOST_HUB_DeviceHandleToInstance(deviceHandle);

    /* Validate the index */
    if ( index <= -1 )
    {
        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortSpeedGet: Invalid HUB instance number " );
        portSpeed = USB_SPEED_ERROR;
    }
    {
        /* Get Hub instance information */
        hubInstance = &gUSBHostHubObj[index];
        portInfo = & ( hubInstance->portInfo[portIndex] );

        /* Validate hub entered any error conditions */
        if (  hubInstance->hubInstanceState == USB_HOST_HUB_STATE_ERROR )
        {
            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortSpeedGet: Hub is in an error state" );
            portSpeed = USB_SPEED_ERROR;
        }
        else
        {
            /* Is port valid ?*/
            if( portNumber < 1 || portNumber >  hubInstance->hubDescriptor.bNbrPorts )
            {
                SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortSpeedGet: Invalid port number" );
                portSpeed = USB_SPEED_ERROR;
            }
            else
            {
                /* Validate any overCurrent situation  */
                if( portInfo->overCurrent == true )
                {
                    SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO , "\r\nUSB_HOST_HUB_PortSpeedGet: Port is in an overcurrent state" );
                    portSpeed = USB_SPEED_ERROR;
                }
                else
                {
                    /* Return the port speed */
                    portSpeed = portInfo->portSpeed; 
                }
            }
        }
    }

    return portSpeed;
}

// *****************************************************************************
/* Function:
    void  _USB_HOST_HUB_ControlTransferComplete
    (
      USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
      USB_HOST_REQUEST_HANDLE requestHandle,
      USB_HOST_RESULT result,
      uint32_t size,
      uintptr_t context
    )

  Summary:
    This function is called when a control transfer initiated by the hub driver
    completes.

  Description:
    This function is called when a control transfer initiated by the hub driver
    completes.
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void  _USB_HOST_HUB_ControlTransferComplete
(
   USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
   USB_HOST_REQUEST_HANDLE requestHandle,
   USB_HOST_RESULT result,
   size_t size,
   uintptr_t context
)
{
    uint32_t hubIndex;
    uint32_t portIndex ;
    uint32_t portNumber;
    USB_HOST_HUB_PORT_INFO * portInfo;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;

    /* The control transfer context is a combination of the port number (bits
     * 31-16) and the hub instance index (bits 15-0). If the port number is 0,
     * then this control transfer was targetted at the hub */

    portNumber =  ( context & PORT_INDEX_MASK ) >> 16 ;
    hubIndex =   ( context & HUB_INDEX_MASK );

    /* Get a pointer to the instance */
    hubInstance = &gUSBHostHubObj[hubIndex];

    portIndex = portNumber - 1;
    portInfo = &( hubInstance->portInfo[portIndex] );

    if ( portNumber == 0x0 )
    { 
        /* This was a hub request */
        hubInstance->controlRequestDone = true ;
        hubInstance->controlRequestResult = result;
    }
    else 
    {
        /* This was a port request */
        portInfo->controlRequestDone = true ;
        portInfo->controlRequestResult = result;
    }
}
 
// *****************************************************************************
/* Function:
    USB_HOST_RESULT _USB_HOST_HUB_HubDescriptorGet (int hubInstanceIndex )

  Summary:
    Launches control transfers to get the Hub Descriptor. 

  Description:
    Launches control transfers to get the Hub Descriptor. 
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_RESULT _USB_HOST_HUB_HubDescriptorGet (int hubInstanceIndex )
{
    void * data;
    uintptr_t context;
    USB_HOST_RESULT  result;
    USB_SETUP_PACKET * setupPacket;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_CONTROL_PIPE_HANDLE  pipeHandle;
    USB_HOST_TRANSFER_HANDLE * transferHandle;
    USB_HOST_DEVICE_CONTROL_REQUEST_COMPLETE_CALLBACK callback;

    hubInstance = &gUSBHostHubObj[hubInstanceIndex];
    setupPacket = &(hubInstance->setupPacket);
    callback = _USB_HOST_HUB_ControlTransferComplete;
    context = hubInstanceIndex;

    /* Fill setup packet */
    setupPacket->bRequest = USB_HUB_CLASS_REQUEST_GET_DESCRIPTOR;
    setupPacket->bmRequestType = USB_HUB_CLASS_STANDARD_HUB_REQUEST;
    setupPacket->wLength = 0x09 ; 
    setupPacket->wIndex = 0x00;
    setupPacket->wValue = ( USB_HUB_DESCRIPTOR_TYPE << 8 );

    data = & ( hubInstance->hubDescriptor );
    pipeHandle = hubInstance->controlPipeHandle;
    transferHandle = &(hubInstance->controlTransferHandle);

    result =  USB_HOST_DeviceControlTransfer (pipeHandle, transferHandle, setupPacket, data, callback, context );
    return result;
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT _USB_HOST_HUB_HubDeviceStatusGet (int hubInstanceIndex )

  Summary:
    Launches control transfers to get the Hub Device Status. 

  Description:
    Launches control transfers to get the Hub Device Status. 
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_RESULT _USB_HOST_HUB_HubDeviceStatusGet (int hubInstanceIndex )
{
    void * data;
    uintptr_t context;
    USB_HOST_RESULT  result;
    USB_SETUP_PACKET * setupPacket;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_CONTROL_PIPE_HANDLE  pipeHandle;
    USB_HOST_TRANSFER_HANDLE * transferHandle;
    USB_HOST_DEVICE_CONTROL_REQUEST_COMPLETE_CALLBACK callback;

    hubInstance = &gUSBHostHubObj[hubInstanceIndex];
    setupPacket = &(hubInstance->setupPacket);
    callback = _USB_HOST_HUB_ControlTransferComplete;
    context = hubInstanceIndex;

    /* Fill setup packet */
    setupPacket->bmRequestType = ( USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE );
    setupPacket->bRequest = USB_REQUEST_GET_STATUS;
    setupPacket->wValue = 0;
    setupPacket->wIndex = 0x00;
    setupPacket->wLength = 0x02 ; 

    data = & ( hubInstance->deviceStatus );
    pipeHandle = hubInstance->controlPipeHandle;
    transferHandle = &(hubInstance->controlTransferHandle);

    result =  USB_HOST_DeviceControlTransfer (pipeHandle, transferHandle, setupPacket, data, callback, context );
    return result;
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_HUB_StatusRequest 
    ( 
        USB_HOST_HUB_DRIVER_INFO *hubDriverInstnceInfo, 
        USB_HOST_HUB_STATUS  *hubStatus ,
        uint8_t portNumber , 
        uint8_t requestType 
    )
 
  Summary:
    Launches control transfers to read hub or port status.

  Description:
    Launches control transfers to read hub or port status.
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
    
*/

USB_HOST_RESULT _USB_HOST_HUB_StatusRequest 
( 
    int hubInstanceIndex,
    void * status ,
    uint8_t portNumber , 
    uint8_t requestType 
)
{
    uintptr_t context;
    uint32_t portIndex;
    USB_HOST_RESULT  result;
    USB_SETUP_PACKET * setupPacket = NULL;
    USB_HOST_REQUEST_HANDLE transferHandle;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_CONTROL_PIPE_HANDLE  pipeHandle;
    USB_HOST_DEVICE_CONTROL_REQUEST_COMPLETE_CALLBACK callback;

    hubInstance = &gUSBHostHubObj[hubInstanceIndex];
    callback = _USB_HOST_HUB_ControlTransferComplete;
    
    /* Create the setup packet */
    if ( requestType == USB_HOST_HUB_PORT_REQUEST )
    {
        portIndex = portNumber - 1;
        setupPacket = &(hubInstance->portInfo[portIndex].setupPacket);
        setupPacket->bmRequestType = USB_HUB_CLASS_STANDARD_PORT_REQUEST;
    }
    else if (requestType == USB_HOST_HUB_REQUEST )
    {
        setupPacket = &(hubInstance->setupPacket);
        setupPacket->bmRequestType = USB_HUB_CLASS_STANDARD_HUB_REQUEST;
        portNumber = 0x0;
    }
    
    setupPacket->bRequest = USB_HUB_CLASS_REQUEST_GET_STATUS;
    setupPacket->wLength = 0x04 ;
    setupPacket->wIndex = portNumber;
    setupPacket->wValue = 0x00;

    /* Place the control transfer request. The context is a combination of the
     * port number (bits 31-16) and the hub instance index (bits 15-0) */
    context = (portNumber << 16) + hubInstanceIndex ;
    pipeHandle = hubInstance->controlPipeHandle;
    
    /* Try to launch the control transfer. Note that we are ignoring the
     * transferHandle by making it a local variable. */
    result = USB_HOST_DeviceControlTransfer(pipeHandle, &transferHandle,setupPacket, status, callback, context);
    
    return result;
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT _USB_HOST_HUB_PortTTBufferClear 
    ( 
        int hubInstanceIndex,
        uint8_t deviceAddress,
        uint8_t endpoint,
        USB_TRANSFER_TYPE transferType, 
        uint8_t ttPortNumber, 
        uint8_t portNumber
    )

  Summary:
    Launches control transfer to clear the hub port TT buffer

  Description:
    Launches control transfer to clear the hub port TT buffer
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
    
*/

USB_HOST_RESULT _USB_HOST_HUB_PortTTBufferClear
( 
    int hubInstanceIndex,
    uint8_t deviceAddress,
    uint8_t endpoint,
    USB_TRANSFER_TYPE transferType, 
    uint8_t ttPortNumber, 
    uint8_t portNumber
)
{
    uintptr_t context;
    uint32_t portIndex;
    USB_HOST_RESULT  result;
    USB_SETUP_PACKET * setupPacket;
    USB_HOST_REQUEST_HANDLE transferHandle;

    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_CONTROL_PIPE_HANDLE  pipeHandle;
    USB_HOST_DEVICE_CONTROL_REQUEST_COMPLETE_CALLBACK callback;

    hubInstance = &gUSBHostHubObj[hubInstanceIndex];
    callback = _USB_HOST_HUB_ControlTransferComplete;

    /* Create the setup packet */
    portIndex = portNumber - 1;
    setupPacket = &(hubInstance->portInfo[portIndex].setupPacket);

    setupPacket->bmRequestType = 0x23;
    setupPacket->bRequest = USB_HUB_CLASS_REQUEST_CLEAR_TT_BUFFER;
    setupPacket->wLength = 0x00 ;
    setupPacket->wIndex = ttPortNumber;
    setupPacket->wValue = 0x00;

    /* Place the control transfer request. The context is a combination of the
     * port number (bits 31-16) and the hub instance index (bits 15-0) */
    context = (portNumber << 16) + hubInstanceIndex ;
    pipeHandle = hubInstance->controlPipeHandle;

    /* Try to launch the control transfer. Note that we are ignoring the
     * transferHandle by making it a local variable. */
    result = USB_HOST_DeviceControlTransfer(pipeHandle, &transferHandle,setupPacket, NULL, callback, context);

    return result;
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT  _USB_HOST_HUB_FeatureRequest 
    (
        USB_HOST_HUB_DRIVER_INFO *hubDriverInstanceInfo,
        uint32_t portNumber ,
        uint8_t Feature, 
        uint8_t hubCommand ,
        uint8_t requestType
    )
 
  Summary:
    Launches control transfers targetted at the hub or port. 

  Description:
    This function is called by the Hub Driver to launch set or clear feature
    control transfers targetted to the hub or port.
  
  Remarks:
    This is local function and should not be called directly by the application.    
*/

USB_HOST_RESULT  _USB_HOST_HUB_FeatureRequest 
(
    uint32_t hubInstanceIndex,
    uint32_t portNumber ,
    uint8_t feature, 
    uint8_t hubCommand ,
    uint8_t requestType
)
{
    uintptr_t context;
    void * data = NULL;
    USB_HOST_RESULT result;
    USB_SETUP_PACKET * setupPacket = NULL;
    USB_HOST_CONTROL_PIPE_HANDLE pipeHandle;
    USB_HOST_REQUEST_HANDLE transferHandle;
    USB_HOST_HUB_INSTANCE_OBJ * hubInstance;
    USB_HOST_DEVICE_CONTROL_REQUEST_COMPLETE_CALLBACK callback;

    hubInstance = &gUSBHostHubObj[hubInstanceIndex];

    /* Create the setup packet. While sending port specific requests, use the
     * setup packet holders which are a part of the port info object. */
    if ( requestType == USB_HOST_HUB_PORT_REQUEST )
    {
        setupPacket = &(hubInstance->portInfo[portNumber - 1].setupPacket);
        setupPacket->bmRequestType = 0x23;
    }
    else if (requestType == USB_HOST_HUB_REQUEST )
    {
        /* Port number is zero for Hub specific requests */
        setupPacket = &(hubInstance->setupPacket);
        setupPacket->bmRequestType = 0x20;
        portNumber = 0x0;
    }

    setupPacket->bRequest = hubCommand;
    setupPacket->wLength = 0x00;
    setupPacket->wValue = feature;
    setupPacket->wIndex = portNumber;

    /* Place the control transfer request. The context is a combination of the
     * port number (bits 31-16) and the hub instance index (bits 15-0) */
    callback = _USB_HOST_HUB_ControlTransferComplete;
    context = (portNumber << 16) + hubInstanceIndex ;
    pipeHandle = hubInstance->controlPipeHandle;
    result =  USB_HOST_DeviceControlTransfer ( pipeHandle , &transferHandle , setupPacket, data , callback , context );
    return result;
}

/******************** End of the file *************************************/
