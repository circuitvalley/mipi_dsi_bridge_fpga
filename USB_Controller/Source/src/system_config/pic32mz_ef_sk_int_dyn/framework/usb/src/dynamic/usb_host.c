/*******************************************************************************
  USB Host Layer Implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host.c

  Summary:
    This file contains implementations of both private and public functions
    of the USB Host Layer.

  Description:
    This file contains the USB host layer implementation. This file should be
    included in the project if USB Host functionality is desired.
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
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "usb/usb_common.h"
#include "usb/usb_chapter_9.h"
#include "system/common/sys_module.h"
#include "usb/usb_host.h"
#include "usb/src/usb_host_local.h"
#include "driver/tmr/drv_tmr.h"
#include "system/tmr/sys_tmr.h"
#include "system/debug/sys_debug.h"
#include "usb/src/usb_host_hub_mapping.h"
#include "osal/osal.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/*******************************************************
 * Host layer object. There is only such object because
 * multiple host controller will be controlled by only
 * one host layer.
 *******************************************************/
static USB_HOST_OBJ  gUSBHostObj;

/*************************************************************
 * Host bus objects. One object per bus on in other words
 * one object per host controller. The index of the bus object
 * is also the bus number. Hence bus object at index 1 in this
 * array will be bus 1.
 *************************************************************/
static USB_HOST_BUS_OBJ  gUSBHostBusList[USB_HOST_CONTROLLERS_NUMBER];

/************************************************************
 * Host device Objects. One object per attached device. This
 * array tracks the attached device. Additional device objects
 * are needed for root hubs.
 ************************************************************/
static USB_HOST_DEVICE_OBJ  gUSBHostDeviceList [ USB_HOST_CONTROLLERS_NUMBER + USB_HOST_DEVICES_NUMBER ];

/************************************************************
 * Array of Pipe Objects. These pipes will be used by all the
 * client drivers that needs to access attached devices. This 
 * array is a shared pool. Pipe object are assigned to client
 * drivers when client driver open pipes.
 ************************************************************/
static USB_HOST_PIPE_OBJ  gUSBHostPipeObj[ USB_HOST_PIPES_NUMBER ];

/************************************************************
 * Array of transfer object. Each object tracks one transfer.
 * This array is a shared pool. Transfer objects are assigned
 * to client driver transfer requests.
 ************************************************************/
static USB_HOST_TRANSFER_OBJ gUSBHostTransferObj[ USB_HOST_TRANSFERS_NUMBER ];

// *****************************************************************************
// *****************************************************************************
// Section: USB HOST Layer Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void * _USB_HOST_TimerCallback
    (
       uint32_t context,
       uint32_t currtick
    )

  Summary:
    Function is called when the SYS_TMR_CallbackSingle expires.

  Description:
    Function is called when the SYS_TMR_CallbackSingle expires.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_TimerCallback(uintptr_t context, uint32_t currtick)
{
    USB_HOST_BUS_OBJ * busObj = ((USB_HOST_BUS_OBJ *)(context));
    busObj->timerExpired = true;
}

// *****************************************************************************
/* Function:
    void * _USB_HOST_FindEndOfDescriptor(void * descriptor) 

  Summary:
    Function finds the end of descritor marker and returns the pointer to where
    the marker has started.

  Description:
    Function finds the end of descriptor marker and returns the pointer to where
    the marker has started.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void * _USB_HOST_FindEndOfDescriptor(void * descriptor) 
{
    uint8_t * search;
    void * result = NULL;
    int foundMarkers = 0;

    if(descriptor == NULL) 
    {
        result = NULL;
    }
    else
    {
        search = (uint8_t *)(descriptor);
        while(foundMarkers < 7)
        {
            if(*search == 0xFF)
            {
                /* Found a marker */
                foundMarkers ++;
                if(foundMarkers == 1)
                {
                    /* This is the first marker we found. Save the memory
                     * location */
                    result = search;
                }

            }
            else
            {
                /* Reset the result */
                foundMarkers = 0;
                result = NULL;
            }

            search ++;
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    void _USB_HOST_RootHubEventDisable(void) 

  Summary:
    Disables all root hub events.

  Description:
    This function will disable all root hub events.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_RootHubEventDisable(void)
{
    int iterator;
    USB_HOST_BUS_OBJ * busObj;

    for(iterator = 0; iterator < USB_HOST_CONTROLLERS_NUMBER; iterator ++)
    {
        busObj = &gUSBHostBusList[iterator];
        
        /* Disable the event. Save the event status */
        busObj->eventsStatusRestore = busObj->hcdInterface->hostEventsDisable(busObj->hcdHandle);
    } 
}

// *****************************************************************************
/* Function:
    void _USB_HOST_RootHubEventEnable(void) 

  Summary:
    Enables all root hub events.

  Description:
    This function will enables all root hub events.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_RootHubEventEnable(void)
{
    int iterator;
    USB_HOST_BUS_OBJ * busObj;

    for(iterator = 0; iterator < USB_HOST_CONTROLLERS_NUMBER; iterator ++)
    {
        busObj = &gUSBHostBusList[iterator];
        
        /* Restore the events to what their status was when they were disabled  */
        busObj->hcdInterface->hostEventsEnable(busObj->hcdHandle, busObj->eventsStatusRestore);
    } 
}

// *****************************************************************************
/* Function:
    bool _USB_HOST_NoInterfacesOwned
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
    );

  Summary:
    This function will return true if no interface have been owned and search
    has reach end of TPL.

  Description:
    This function will return true if no interface have been owned and search
    has reach end of TPL. It will return false if at least one interface is
    claimed or if all the interfaces are empty.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

bool _USB_HOST_NoInterfacesOwned
(
    USB_HOST_DEVICE_OBJ * deviceObj
)
{
    bool result = false;
    USB_HOST_INTERFACE_DESC_INFO * interfaceInfo;
    int iterator;

    /* Check every interface in this device */
    for(iterator = 0; iterator < USB_HOST_DEVICE_INTERFACES_NUMBER; iterator ++)
    {
        interfaceInfo = &(deviceObj->configDescriptorInfo.interfaceInfo[iterator]);
        if(interfaceInfo->interfaceDescriptor != NULL)
        {
            /* Interface has valid interface descriptor. Check if is owned */
            if((interfaceInfo->interfaceDriver == NULL) && (interfaceInfo->tplEntryMatched >= gUSBHostObj.nTPLEntries))
            {
                /* This means all the driver were tried and this interface was
                 * not owned */
                result = true;
            }
            else
            {
                /* Either the interface is claimed or all TPL entries have
                 * not been searched */
                result = false;
                break;
            }
        }
    }
    
    return(result);
}

// *****************************************************************************
/* Function:
    void _USB_HOST_UpdateInterfaceStatus
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
        int busIndex
    );

  Summary:
    This function will update status of the interfaces.

  Description:
    This function will update the status of the interfaces. If a interface is
    not assigned it is either assigned to the device level driver or it is
    assigned to an interface driver. The function will call the tasks routines
    of the interface driver. It checks if the all device is not owned at all
    then it will move the device to an error state.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_UpdateInterfaceStatus
(
    USB_HOST_DEVICE_OBJ * deviceObj,
    int busIndex
)
{
    USB_HOST_BUS_OBJ * busObj;
    USB_HOST_INTERFACE_DESC_INFO * interfaceInfo, * interfaceInfoIterator;
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandles[USB_HOST_DEVICE_INTERFACES_NUMBER];
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;
    USB_INTERFACE_ASSOCIATION_DESCRIPTOR * interfaceAssociation;
    int iterator, iadIterator;

    busObj = &(gUSBHostBusList[busIndex]);

    /* This redundant statement is added to avoid warning in a case where the
     * debug messages are disabled. */
    busObj = busObj;
    
    if((deviceObj->deviceState == USB_HOST_DEVICE_STATE_READY) && 
            (deviceObj->configDescriptorInfo.configurationNumber > 0) &&
            (deviceObj->configDescriptorInfo.configurationNumber != USB_HOST_CONFIGURATION_NUMBER_INVALID))
    {
        /* This means that the device is in a running state and the device is
         * configured */
        for(iterator = 0; iterator < deviceObj->nInterfaces ; iterator ++)
        {
            interfaceInfo = &deviceObj->configDescriptorInfo.interfaceInfo[iterator];

            if(interfaceInfo->interfaceDriver != NULL)
            {
                /* The interface is owned. Run the tasks routine of this driver */
                interfaceInfo->interfaceDriver->interfaceTasks(interfaceInfo->interfaceHandle);
            }
            else
            {
                /* Driver is not assigned. Check if this device has a device
                 * level driver and that this interface has not already been
                 * tried with that driver */

                if((deviceObj->deviceClientDriver != NULL) && (!interfaceInfo->wasTriedWithDeviceDriver))
                {
                    /* The device is owned and the interface has not been tried
                     * yet with the device driver. Call the interface assign
                     * function of the device client driver */

                    if(interfaceInfo->interfaceAssociationDescriptor != NULL)
                    {
                        /* IAD Case. Prepare the table of interfaces and assign
                         * all of them to the same driver */

                        interfaceInfoIterator = interfaceInfo;
                        iadIterator = 0;
                        while(interfaceInfoIterator != NULL)
                        {
                            /* Add the handle of this interface to the interface
                             * table. Assign the device level driver to all
                             * interfaces. */
                            interfaceHandles[iadIterator] = interfaceInfoIterator->interfaceHandle;
                            interfaceInfoIterator->interfaceDriver = deviceObj->deviceClientDriver;
                            interfaceInfoIterator->wasTriedWithDeviceDriver = true;
                            iadIterator ++;
                            interfaceInfoIterator = interfaceInfoIterator->nextInterface;
                        }

                        /* Now the interfaceHandles table has the handles of all
                         * the interfaces in this IAD. Call the device client
                         * driver interface assign function with this table. */

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Assigning IAD group to device driver", busIndex, deviceObj->deviceAddress);
                        deviceObj->deviceClientDriver->interfaceAssign(interfaceHandles,
                                deviceObj->deviceIdentifier, iadIterator, (uint8_t *)(interfaceInfo->interfaceAssociationDescriptor));
                    }
                    else if(interfaceInfo->interfaceDescriptor != NULL)
                    {
                        /* Non IAD case. Prepare the interface handle table. In
                         * case of a non IAD this will be one interface and
                         * hence one handle only. */

                        interfaceHandles[0] = interfaceInfo->interfaceHandle;

                        /* We assign the driver before calling the
                         * interfaceAssign function. This will allow the client
                         * to release the interface if it chooses to and the host
                         * will try re-matching the interface. */

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Assigning interface to device driver", busIndex, deviceObj->deviceAddress);
                        interfaceInfo->interfaceDriver = deviceObj->deviceClientDriver;
                        deviceObj->deviceClientDriver->interfaceAssign(interfaceHandles,
                                deviceObj->deviceIdentifier, 1, (uint8_t *)(interfaceInfo->interfaceDescriptor));

                        /* Set the flag indicating this was tried with device
                         * driver. */
                        interfaceInfo->wasTriedWithDeviceDriver = true;
                    }
                }
                else
                {
                    /* This means the interface driver is never assigned, or if
                     * there is a client driver it has released the interface,
                     * or there isn't a device level client driver. Check the
                     * TPL for a match. Has this searched reached the end of the
                     * TPL table */

                    if(interfaceInfo->tplEntryMatched < gUSBHostObj.nTPLEntries)
                    {
                        /* Not tried with all TPL entries */
                        if(interfaceInfo->interfaceAssociationDescriptor != NULL)
                        {
                            /* IAD case */

                            USB_HOST_CLIENT_DRIVER * matchedClientDriver;
                            int matchedTPLEntry;

                            /* Get the Interface association descriptor */
                            interfaceAssociation = interfaceInfo->interfaceAssociationDescriptor;

                            /* Search the TPL for a driver */
                            interfaceInfo->tplEntryMatched = _USB_HOST_FindClassSubClassProtocolDriver(interfaceAssociation->bFunctionClass,
                                    interfaceAssociation->bFunctionSubClass, interfaceAssociation->bFunctionProtocol, interfaceInfo->tplEntryMatched + 1);

                            /* Did we find a driver match */
                            if(interfaceInfo->tplEntryMatched < gUSBHostObj.nTPLEntries)
                            {
                                /* Yes we did */
                                interfaceInfoIterator = interfaceInfo;
                                iadIterator = 0;
                                matchedTPLEntry = interfaceInfo->tplEntryMatched;
                                matchedClientDriver = gUSBHostObj.tpl[interfaceInfo->tplEntryMatched].hostClientDriver;

                                while(interfaceInfoIterator != NULL)
                                {
                                    /* Add the handle of this interface to the interface
                                     * table. Assign the device level driver to all
                                     * interfaces. */
                                    interfaceHandles[iadIterator] = interfaceInfoIterator->interfaceHandle;
                                    interfaceInfoIterator->interfaceDriver = matchedClientDriver;
                                    interfaceInfoIterator->tplEntryMatched = matchedTPLEntry;
                                    iadIterator ++;
                                    interfaceInfoIterator = interfaceInfoIterator->nextInterface;
                                }

                                /* Now the interfaceHandles table has the
                                 * handles of all the interfaces in this IAD.
                                 * Call the client driver interface assign
                                 * function with this table. */
                                
                                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Assigning IAD to TPL entry %d", busIndex, deviceObj->deviceAddress, matchedTPLEntry);
                                matchedClientDriver->interfaceAssign(interfaceHandles,
                                        deviceObj->deviceIdentifier, iadIterator, (uint8_t *)(interfaceInfo->interfaceAssociationDescriptor));
                            }
                        }
                        else if(interfaceInfo->interfaceDescriptor != NULL)
                        {
                            /* Single interface case */
                            interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)(interfaceInfo->interfaceDescriptor);

                            /* Search for a driver */
                            interfaceInfo->tplEntryMatched = _USB_HOST_FindClassSubClassProtocolDriver(interfaceDescriptor->bInterfaceClass,
                                    interfaceDescriptor->bInterfaceSubClass, interfaceDescriptor->bInterfaceProtocol, interfaceInfo->tplEntryMatched + 1);

                            /* Did we find a driver match */
                            if(interfaceInfo->tplEntryMatched < gUSBHostObj.nTPLEntries)
                            {
                                /* This means we found a match. Assign the
                                 * driver. Create the interface table with one 
                                 * interface handle. */
                                
                                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Assigning Interface to TPL entry %d", busIndex, deviceObj->deviceAddress, interfaceInfo->tplEntryMatched);
                                interfaceHandles[0] = interfaceInfo->interfaceHandle;
                                interfaceInfo->interfaceDriver = gUSBHostObj.tpl[interfaceInfo->tplEntryMatched].hostClientDriver;
                                interfaceInfo->interfaceDriver->interfaceAssign(interfaceHandles, 
                                        deviceObj->deviceIdentifier, 1, (uint8_t *)(interfaceInfo->interfaceDescriptor));
                            }
                        }
                    }
                }
            }
        }

        /* We have to keep check if all the interfaces and the device are
         * owned. If we reach a point where none are owned and the search has
         * reached the end of TPL, then we move the device to an error state.
         * Moving the device to an error state will reduce the processing that
         * the host has to do for this device */

        if((deviceObj->deviceClientDriver == NULL) &&
                (deviceObj->tplEntryTried >= gUSBHostObj.nTPLEntries) &&
                (_USB_HOST_NoInterfacesOwned(deviceObj)))
        {
            /* This means that no driver will match this device. Move this
             * device to an error state */

            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d not matched to any TPL entry", busIndex, deviceObj->deviceAddress);
            deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
            deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
            if(gUSBHostObj.hostEventHandler != NULL)
            {
                /* Send an event to the application */
                gUSBHostObj.hostEventHandler(USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context);
            }
        }
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_ReleaseInterfaceDrivers
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
    );

  Summary:
    This function will release all the loaded interface drivers.

  Description:
    This function will release all the loaded interface drivers. The matching
    driver index for each interface will be updated to indicate that the
    matching should start at the top of the TPL table.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_ReleaseInterfaceDrivers
(
    USB_HOST_DEVICE_OBJ * deviceObj
)
{
    int iterator;
    USB_HOST_INTERFACE_DESC_INFO * interfaceInfo;

    for(iterator = 0; iterator < USB_HOST_DEVICE_INTERFACES_NUMBER; iterator ++)
    {
        interfaceInfo = &(deviceObj->configDescriptorInfo.interfaceInfo[iterator]);
        if(interfaceInfo->interfaceDriver != NULL)
        {
            interfaceInfo->interfaceDriver->interfaceRelease(interfaceInfo->interfaceHandle);
            interfaceInfo->interfaceDriver = NULL;
        }

        /* Clear up the other interface members */
        interfaceInfo->tplEntryMatched = -1;
        interfaceInfo->interfaceDescriptor = NULL;
        interfaceInfo->wasTriedWithDeviceDriver = false;
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_ConfigurationDescriptorParse
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
    );

  Summary:
    This function will parse the configuration descriptor contained in the
    configurationDescriptor of the configDescriptorInfo structure in deviceObj
    and will populate the interface tables. If the configuration descriptor
    contains IADs, it will then link the interfaces as defined by the IAD.

  Description:
    This function will parse the configuration descriptor contained in the
    configurationDescriptor of the configDescriptorInfo structure in deviceObj
    and will populate the interface tables. If the configuration descriptor
    contains IADs, it will then link the interfaces as defined by the IAD.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

bool _USB_HOST_ConfigurationDescriptorParse
(
    USB_HOST_DEVICE_OBJ * deviceObj
)
{
    USB_HOST_INTERFACE_DESCRIPTOR_QUERY interfaceQueryObj;
    USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;
    USB_INTERFACE_ASSOCIATION_DESCRIPTOR * interfaceAssociation;
    USB_HOST_INTERFACE_DESC_INFO * interfaceDescInfo, * previousInterface;
    USB_HOST_IAD_QUERY iadQueryObj;
    bool result = true;
    unsigned int iterator;
    uint8_t bFirstInterface;
    uint8_t bInterfaceCount;

    /* Get the device index and the pnp identifier. These are needed to form the
     * interface handle */
    unsigned int deviceIndex = USB_HOST_DEVICE_INDEX(deviceObj->deviceIdentifier);
    unsigned int pnpIdentifier = USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier);

    /* Get the configuration descriptor and the number of interfaces */
    configurationDescriptor = deviceObj->configDescriptorInfo.configurationDescriptor;
    int nInterfaces = configurationDescriptor->bNumInterfaces;

    /* Reset the interface query context. Set up the interface query to find 
     * interface by number and alternate setting 0 */
    USB_HOST_DeviceInterfaceQueryContextClear(&interfaceQueryObj);
    interfaceQueryObj.flags = USB_HOST_INTERFACE_QUERY_BY_NUMBER|USB_HOST_INTERFACE_QUERY_ALT_SETTING;
    interfaceQueryObj.bAlternateSetting = 0;

    for(iterator = 0; iterator < nInterfaces; iterator ++)
    {
        /* Search for interface descriptor */
        interfaceQueryObj.bInterfaceNumber = iterator;
        interfaceDescriptor = USB_HOST_DeviceInterfaceDescriptorQuery(configurationDescriptor, &interfaceQueryObj);
        if(interfaceDescriptor == NULL)
        {
            /* This should never happen. The host will check a configuration
             * descriptor for error before it allows the device to reach this
             * stage. Cannot say why the interface descriptor was not found */
            result = false;
            break;
        }

        /* Initialize the interface descriptor information object */
        interfaceDescInfo = &(deviceObj->configDescriptorInfo.interfaceInfo[iterator]);
        interfaceDescInfo->interfaceDescriptor = interfaceDescriptor;
        interfaceDescInfo->interfaceAssociationDescriptor = NULL;
        interfaceDescInfo->currentAlternateSetting = 0;
        interfaceDescInfo->interfaceHandle = _USB_HOST_DeviceInterfaceHandleGet(pnpIdentifier, iterator, deviceIndex);
        interfaceDescInfo->tplEntryMatched = -1;
    }

    /* Update the number of interfaces in the device object */
    deviceObj->nInterfaces = nInterfaces;

    /* Now we search the configuration descriptor for IADs. Clear the search
     * context before we start the search. */
    USB_HOST_DeviceIADQueryContextClear(&iadQueryObj);
    
    do
    {
        /* We will search for any IAD.  */
        iadQueryObj.flags = USB_HOST_IAD_QUERY_FLAG_ANY;

        /* Search for IAD */
        interfaceAssociation = USB_HOST_DeviceIADQuery(configurationDescriptor, &iadQueryObj);

        if(interfaceAssociation != NULL)
        {
            /* Get the starting interface and the number of contiguous
             * interfaces */
            bInterfaceCount = interfaceAssociation->bInterfaceCount;
            bFirstInterface = interfaceAssociation->bFirstInterface;
            previousInterface = NULL;

            for(iterator = bFirstInterface; iterator < (bFirstInterface + bInterfaceCount); iterator ++)
            {
                /* Get the pointer to the interface descriptor object for this
                 * interface. Set the interfaceAssociationDescriptor member to
                 * point to IAD. Set the next descriptor to point to the next
                 * interface descriptor in the group. */
                interfaceDescInfo = &(deviceObj->configDescriptorInfo.interfaceInfo[iterator]);
                interfaceDescInfo->interfaceAssociationDescriptor = interfaceAssociation;

                if(previousInterface == NULL)
                {
                    /* This is the first interface in this group. We set the
                     * previous interface to the current interface. */
                    previousInterface = interfaceDescInfo;
                    previousInterface->nextInterface = NULL;
                }
                else
                {
                    /* This is the not the first interface in the group. Set the
                     * next interface of previous one to this one. Then set
                     * previous to this interface. */
                    previousInterface->nextInterface = interfaceDescInfo;
                    interfaceDescInfo->nextInterface = NULL;
                    previousInterface = interfaceDescInfo;
                }
            }
        }
        else
        {
            /* No more IAD in this configuration descriptor */
        }
    } while(interfaceAssociation != NULL);

    /* At this point, if there was an IAD, say 2 IAD with interface 0 and 1 and
     * 2 and 3. The interfaceAssociationDescriptor of interfaceInfo[0] and
     * interfaceInfo[1] will point to the parent IAD.
     * interfaceInfo[0].nextInterface will point to interfaceInfo[1] and
     * interfaceInfo[1].nextInterface will be NULL because it is the last
     * interface in this group. Similarly interfaceAssociationDescriptor of
     * interfaceInfo[2] and interfaceInfo[3] will point to the parent IAD.
     * interfaceInfo[2].nextInterface will point to interfaceInfo[3] and
     * interfaceInfo[3].nextInterface will be NULL because it is the last
     * interface in this group. */
    return(result);
}

// *****************************************************************************
/* Function:
    int _USB_HOST_FindClassSubClassProtocolDriver
    (
        uint8_t bDeviceClass,
        uint8_t bDeviceSubClass,
        uint8_t bDeviceProtocol,
        int startPoint
    );

  Summary:
    This function will search for matching class subclass protocol driver in the
    TPL table.

  Description:
    This function will search for matching class subclass protocol driver in the
    TPL table. If a driver was not found, the function will return the last
    index of the TPL table + 1. The function will start searching from (and
    including) startPoint.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

int _USB_HOST_FindClassSubClassProtocolDriver
(
    uint8_t bDeviceClass,
    uint8_t bDeviceSubClass,
    uint8_t bDeviceProtocol,
    int startPoint
)
{
    USB_HOST_TPL_ENTRY * tpl;
    int iterator;
    USB_HOST_OBJ * hostObj = &gUSBHostObj;
    unsigned int matched = 0;
    unsigned int tplFlags;

    for(iterator = startPoint; iterator < hostObj->nTPLEntries; iterator ++)
    {
        tpl = &hostObj->tpl[iterator];

        /* Check if this entry is a class subclass protocol entry */
        if(tpl->tplFlags.driverType == TPL_FLAG_CLASS_SUBCLASS_PROTOCOL)
        {
            /* First we check if which field match */

            if(bDeviceClass == tpl->id.cl_sc_p.classCode)
            {
                /* Class matched */
                matched |= 0x2;
            }

            if(bDeviceSubClass == tpl->id.cl_sc_p.subClassCode)
            {
                /* Subclass matched */
                matched |= 0x4;
            }

            if(bDeviceProtocol == tpl->id.cl_sc_p.protocolCode)
            {
                /* Protocol matched */
                matched |= 0x8;
            }

            tplFlags = (tpl->tplFlags.ignoreClass << 1) | (tpl->tplFlags.ignoreSubClass << 2) | (tpl->tplFlags.ignoreProtocol << 3);
            matched = matched & (~(tplFlags & 0xE));

            /* Now check if the criteria matches */
            if((tplFlags & 0xE) == ((~matched) & 0xE))
            {
                /* We found a match */
                break;
            }
        }
    }

    return(iterator);
}

// *****************************************************************************
/* Function:
    void _USB_HOST_UpdateConfigurationState
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
        int busIndex
    );

  Summary:
    This function will update the configuration state of the device.

  Description:
    This function will check if the device configuration needs to be changed. If
    so then it gets the configuration, parses the configuration, sets up the
    interface tables and then sets the configuration.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_UpdateConfigurationState
(
    USB_HOST_DEVICE_OBJ * deviceObj,
    int busIndex
)
{
    USB_HOST_BUS_OBJ * busObj;
    USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor;
    USB_HOST_DEVICE_EVENT_CONFIGURATION_SET_DATA eventData;
    int iterator;
    uint8_t isSelfPowered;

    busObj = &(gUSBHostBusList[busIndex]);
    
    /* This redundant statement is added to avoid warning in a case where the
     * debug messages are disabled. */
    busObj = busObj;
    
    /* Only proceed if the device is in a ready state */
    if(deviceObj->deviceState == USB_HOST_DEVICE_STATE_READY)
    {
        switch(deviceObj->configurationState)
        {
            case USB_HOST_DEVICE_CONFIG_STATE_READY_FOR_CONFIG:

                /* We don't have to do anything here. The state indicates that
                 * the device is ready for configuration, but somebody has to
                 * set it */

                break;

            case USB_HOST_DEVICE_CONFIG_STATE_START:

                /* Start the process of setting the configuration. We first get
                 * the configuration header. The requestedConfigurationNumber
                 * member of deviceObj contains the index of the configuration to be
                 * set */

                _USB_HOST_FillSetupPacket(
                        &(deviceObj->setupPacket),
                        ( USB_SETUP_DIRN_DEVICE_TO_HOST |
                          USB_SETUP_TYPE_STANDARD |
                          USB_SETUP_RECIPIENT_DEVICE ),
                        USB_REQUEST_GET_DESCRIPTOR,
                        ( USB_DESCRIPTOR_CONFIGURATION << 8 )+ deviceObj->requestedConfigurationNumber , 0 , 9 ) ;

                /* Fill IRP */
                deviceObj->controlTransferObj.controlIRP.data = ( void * )deviceObj->buffer;
                deviceObj->controlTransferObj.controlIRP.setup = &(deviceObj->setupPacket);
                deviceObj->controlTransferObj.controlIRP.size = 9;
                deviceObj->controlTransferObj.controlIRP.callback = NULL;

                /* Set the next state */
                deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIG_DESCRIPTOR_HEADER_GET;

                /* Submit the IRP */
                if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, 
                            &(deviceObj->controlTransferObj.controlIRP)))
                {
                    /* We need to be able to send the IRP. We move the
                     * device to an error state. Close the pipe and send
                     * an event to the application. */
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Configuration Descriptor IRP failed. Device not supported.", 
                            busIndex, deviceObj->deviceAddress);
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                    if(gUSBHostObj.hostEventHandler != NULL)
                    {
                        /* Send an event to the application */
                        gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                    }
                }

                break;

            case USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIG_DESCRIPTOR_HEADER_GET:

                /* Here we are waiting for Get Short Configuration header to
                 * complete */
                if (deviceObj->controlTransferObj.controlIRP.status == USB_HOST_IRP_STATUS_COMPLETED)
                {
                    if (deviceObj->hubAddress == 0x00 )
                    {
                        /* Check if the device is a self powered or bus powered    */
                        isSelfPowered =  ( (((USB_CONFIGURATION_DESCRIPTOR *) deviceObj->buffer)->bmAttributes) & USB_ATTRIBUTE_SELF_POWERED );

                        if (isSelfPowered != USB_ATTRIBUTE_SELF_POWERED )
                        {
                            /* This means the device is bus powered. We should check
                             * if this configuration requires more current than what
                             * the root hub can provide. */

                            if ( ( 2 * (((USB_CONFIGURATION_DESCRIPTOR * ) deviceObj->buffer)->bMaxPower )) > busObj->rootHubInfo.power )
                            {
                                /* This means the device needs more power than what
                                 * the root hub can provide. We cannot set this
                                 * configuration. */

                                if(deviceObj->deviceClientDriver != NULL)
                                {
                                    /* This means this device has device level
                                     * driver and it is this driver that had
                                     * requested for the configuration chanage.
                                     * We let the driver know that the
                                     * configuration cannot be set and then move
                                     * the device to a ready but un-configured
                                     * state.  This will allow the driver level
                                     * driver to try setting another
                                     * configuration. */

                                    eventData.result = USB_HOST_RESULT_FAILURE;
                                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_READY;
                                    deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_READY_FOR_CONFIG;

                                    /* We should send an event to the client
                                     * driver result is failure  */
                                    deviceObj->deviceClientDriver->deviceEventHandler(deviceObj->deviceClientHandle,
                                            USB_HOST_DEVICE_EVENT_CONFIGURATION_SET, &eventData, deviceObj->controlTransferObj.context);
                                }
                                else
                                {
                                    /* The device does not have a devel lever
                                     * client driver. This means the host layer
                                     * owns the device. The host layer at this
                                     * time cannot try any other configuration.
                                     * The device must be moved to an
                                     * in-operational state because of an
                                     * over-current request. */

                                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                                }

                                /* We should send an event to the application */
                                if(gUSBHostObj.hostEventHandler != NULL)
                                {
                                    gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_REJECTED_INSUFFICIENT_POWER, NULL, gUSBHostObj.context );

                                }

                                break;
                            }

                        }
                    }

                    /* IRP was successful. Go to the next state */
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Obtained Configuration Descriptor header", busIndex, deviceObj->deviceAddress);  
                    deviceObj->configurationState =  USB_HOST_DEVICE_CONFIG_STATE_CONFIG_DESCRIPTOR_GET;
                }
                else
                {
                    /* The IRP did not complete successfully. */
                    if ( deviceObj->controlTransferObj.controlIRP.status < USB_HOST_IRP_STATUS_COMPLETED )
                    {
                        /* Close the pipe */
                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Error while obtaining Configuration Descriptor header",
                                busIndex, deviceObj->deviceAddress);  
                        
                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;

                        /* We should send an event to the application
                         * and then wait for device attach */
                        if(gUSBHostObj.hostEventHandler != NULL)
                        {
                            gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                        }
                    }
                }
                break;

            case USB_HOST_DEVICE_CONFIG_STATE_CONFIG_DESCRIPTOR_GET:

                /* Allocate memory and then get the entire configuration
                 * descriptor */
                if(deviceObj->configDescriptorInfo.configurationDescriptor != NULL)
                {
                    USB_HOST_FREE(deviceObj->configDescriptorInfo.configurationDescriptor);
                }

                /* Now allocate memory. While allocating the memory, we allocate
                 * 7 additional bytes to store the end of configuration
                 * descriptor memory configuration marker. This marker will
                 * allow the query functions to identify the end of the
                 * configuration descriptor */

                configurationDescriptor = (USB_CONFIGURATION_DESCRIPTOR *)(deviceObj->buffer);
                deviceObj->configDescriptorInfo.configurationDescriptor = USB_HOST_MALLOC(configurationDescriptor->wTotalLength + 7);

                if(deviceObj->configDescriptorInfo.configurationDescriptor == NULL)
                {
                    /* The memory allocation failed. We need memory to continue.
                     * We have to stop here */
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                    if(gUSBHostObj.hostEventHandler != NULL)
                    {
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Could not allocate memory for Configuration Descriptor", busIndex, deviceObj->deviceAddress);
                        gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                    }
                }
                else
                {
                    /* Place a request for the full configuration descriptor */
                    _USB_HOST_FillSetupPacket(
                            &(deviceObj->setupPacket), ( USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE ),
                            USB_REQUEST_GET_DESCRIPTOR, ( USB_DESCRIPTOR_CONFIGURATION << 8 ) + deviceObj->requestedConfigurationNumber,
                            0 ,configurationDescriptor->wTotalLength) ;

                    /* Create the IRP */
                    deviceObj->controlTransferObj.controlIRP.data = deviceObj->configDescriptorInfo.configurationDescriptor;
                    deviceObj->controlTransferObj.controlIRP.setup = &(deviceObj->setupPacket);
                    deviceObj->controlTransferObj.controlIRP.size = configurationDescriptor->wTotalLength;
                    deviceObj->controlTransferObj.controlIRP.callback = NULL;
                    deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIG_DESCRIPTOR_GET;

                    /* Submit the IRP */
                    if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, 
                                &(deviceObj->controlTransferObj.controlIRP)))
                    {
                        /* We need to be able to send the IRP. We move the
                         * device to an error state. Close the pipe and send
                         * an event to the application. */
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Configuration Descriptor IRP failed. Device not supported.", busIndex, deviceObj->deviceAddress);
                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                        if(gUSBHostObj.hostEventHandler != NULL)
                        {
                            /* Send an event to the application */
                            gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                        }
                    }
                }

                break;

            case USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIG_DESCRIPTOR_GET:

                /* Here we check if we have received the configuration
                 * descriptor */
                if (deviceObj->controlTransferObj.controlIRP.status == USB_HOST_IRP_STATUS_COMPLETED) 
                {
                    /* We have received the configuration descriptor. 
                     * We can set this configuration. */

                    deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_CONFIGURATION_SET;
                }
                else
                {
                    if ( deviceObj->controlTransferObj.controlIRP.status < USB_HOST_IRP_STATUS_COMPLETED )
                    {
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Error while obtaining Configuration Descriptor.", busIndex, deviceObj->deviceAddress);  
                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);

                        /* Move the device to error state */
                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                        
                        if(gUSBHostObj.hostEventHandler != NULL)
                        {
                            gUSBHostObj.hostEventHandler(USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                        }
                    }
                }

                break;

            case USB_HOST_DEVICE_CONFIG_STATE_CONFIGURATION_SET:
                 /* In this state, the host will set the configuration */
                _USB_HOST_FillSetupPacket(
                        &(deviceObj->setupPacket),
                        ( USB_SETUP_DIRN_HOST_TO_DEVICE |
                          USB_SETUP_TYPE_STANDARD |
                          USB_SETUP_RECIPIENT_DEVICE ),
                        USB_REQUEST_SET_CONFIGURATION,
                        deviceObj->configDescriptorInfo.configurationDescriptor->bConfigurationValue,
                        0 ,0 ) ;

                /* Fill IRP */
                deviceObj->controlTransferObj.controlIRP.data = NULL;
                deviceObj->controlTransferObj.controlIRP.setup = &(deviceObj->setupPacket);
                deviceObj->controlTransferObj.controlIRP.size = 0;
                deviceObj->controlTransferObj.controlIRP.callback = NULL;
                deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIGURATION_SET;

                /* Submit the IRP */
                if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, 
                            &(deviceObj->controlTransferObj.controlIRP)))
                {
                    /* We need to be able to send the IRP. We move the
                     * device to an error state. Close the pipe and send
                     * an event to the application. */
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Set Configuration IRP failed. Device not supported.", busIndex, deviceObj->deviceAddress);  
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                    if(gUSBHostObj.hostEventHandler != NULL)
                    {
                        /* Send an event to the application */
                        gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                    }
                }

                break;

            case USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIGURATION_SET:

                /* Here we check if the set configuration has completed */
                if (deviceObj->controlTransferObj.controlIRP.status == USB_HOST_IRP_STATUS_COMPLETED) 
                {
                    /* The configuration set was successful. Unload the existing
                     * interface drivers */

                    _USB_HOST_ReleaseInterfaceDrivers(deviceObj);

                    /* Insert the end of the configuration descriptor marker
                     * into the configuration */
                    configurationDescriptor = deviceObj->configDescriptorInfo.configurationDescriptor;
                    for(iterator = 0; iterator < 7; iterator ++)
                    {
                        /* The end of configuration descriptor is 7 bytes, each
                         * 0xFF */
                        ((uint8_t *)(configurationDescriptor))[configurationDescriptor->wTotalLength + iterator] = 0xFF;
                    }
                    
                    /* Parse the configuration descriptor and then update the
                     * interface tables. */
                    if(!_USB_HOST_ConfigurationDescriptorParse(deviceObj))
                    {
                        /* The parsing failed */
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Error in configuration desciptor", busIndex, deviceObj->deviceAddress);  
                        eventData.result = USB_HOST_RESULT_FAILURE;
                    }
                    else
                    {
                        /* The parsing worked. Update the active configuration
                         * to indicate that the configuration has been set */

                        deviceObj->configDescriptorInfo.configurationNumber = deviceObj->configDescriptorInfo.configurationDescriptor->bConfigurationValue;
                        eventData.result = USB_HOST_RESULT_SUCCESS;
                    }

                    /* If there is device level client driver, then we let it
                     * know that the configuration has been set. */

                    eventData.requestHandle = (USB_HOST_REQUEST_HANDLE)(&deviceObj->controlTransferObj);
                    if(deviceObj->deviceClientDriver != NULL)
                    {
                        deviceObj->deviceClientDriver->deviceEventHandler(deviceObj->deviceClientHandle,
                                USB_HOST_DEVICE_EVENT_CONFIGURATION_SET, &eventData, deviceObj->controlTransferObj.context);
                    }

                    /* The configuration set is complete. We are ready to set
                     * another configuration if requested. Return the control
                     * transfer object back */

                    deviceObj->controlTransferObj.inUse = false;
                    deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_READY_FOR_CONFIG;
                }
                else
                {
                    /* The set configuration request failed */
                    if ( deviceObj->controlTransferObj.controlIRP.status < USB_HOST_IRP_STATUS_COMPLETED )
                    {
                        eventData.result = USB_HOST_RESULT_FAILURE;
                        /* If there is device level client driver, then we let it
                         * know that the configuration has been set. */
                        
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Could not set configuration.", busIndex, deviceObj->deviceAddress);  

                        eventData.requestHandle = (USB_HOST_REQUEST_HANDLE)(&deviceObj->controlTransferObj);
                        if(deviceObj->deviceClientDriver != NULL)
                        {
                            deviceObj->deviceClientDriver->deviceEventHandler(deviceObj->deviceClientHandle,
                                    USB_HOST_DEVICE_EVENT_CONFIGURATION_SET, &eventData, deviceObj->controlTransferObj.context);
                        }
                        deviceObj->controlTransferObj.inUse = false;
                        deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_READY_FOR_CONFIG;

                    }
                }
                break;

            default:
                break;
        }
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_UpdateDeviceOwnership
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
        int busIndex
    );

  Summary:
    This function will find a device level owner client driver.

  Description:
    This function will find a device level client driver owner. If a VID PID
    level driver is not found then a device level class subclass protocol driver
    needs to be found. If device was released, then a new owner needs to be
    found. If the end of the TPL table is reached, then the stop searching and
    hand over ownership of the device to the host. If a driver is attached, the
    function will call the tasks routine of this driver.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_UpdateDeviceOwnership
(
    USB_HOST_DEVICE_OBJ * deviceObj,
    int busIndex
)
{
    int tplSearch;
    USB_HOST_TPL_ENTRY * tpl;
    USB_DEVICE_DESCRIPTOR * deviceDescriptor;
    USB_HOST_BUS_OBJ * busObj;

    busObj = &(gUSBHostBusList[busIndex]);

    /* This redundant statement is added to avoid warning in a case where the
     * debug messages are disabled. */
    busObj = busObj;
    
    /* Check if the device is in a ready state. */
    if(deviceObj->deviceState == USB_HOST_DEVICE_STATE_READY)
    {
        deviceDescriptor = &(deviceObj->deviceDescriptor);

        if(deviceObj->deviceClientDriver != NULL)
        {
            /* Run the tasks routine */
            deviceObj->deviceClientDriver->deviceTasks(deviceObj->deviceClientHandle);
        }

        /* Matching is needed only if the device is not claimed and we have not
         * already reached the end of the table*/

        if((deviceObj->deviceClientDriver == NULL) && (!(deviceObj->tplEntryTried >= gUSBHostObj.nTPLEntries)))
        {
            /* The initial value (on device connect) of tplEntryTried is -1. So then
             * the tplSearch will start matching from 0. If this is not the first
             * time matching, then matching will start from the next entry in the
             * TPL table */

            SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Bus %d Device %d Looking for Device Level Driver.", busIndex, deviceObj->deviceAddress);
            for (tplSearch = (deviceObj->tplEntryTried + 1); tplSearch < gUSBHostObj.nTPLEntries; tplSearch ++)
            {
                tpl = &gUSBHostObj.tpl[tplSearch];

                if(tpl->tplFlags.driverType == TPL_FLAG_VID_PID)
                {
                    /* This entry is a VID PID Entry */

                    if(tpl->tplFlags.ignoreVIDPID)
                    {
                        /* This means we should attach this driver as the entry says
                         * that ignore the VID PID and match */

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Bus %d Device %d matched entry %d in TPL table", 
                                busIndex, deviceObj->deviceAddress, tplSearch);

                        deviceObj->deviceClientDriver = (USB_HOST_CLIENT_DRIVER *)(tpl->hostClientDriver);
                        break;
                    }
                    else if(tpl->tplFlags.pidMasked)
                    {
                        /* This means we should apply the specified mask to the PID
                         * field and then compare. */

                        if((deviceDescriptor->idVendor == tpl->id.vid_pid.vid) && 
                                ((deviceDescriptor->idProduct & tpl->pidMask) == tpl->id.vid_pid.pid))
                        {
                            /* Criteria matched */
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Bus %d Device %d matched entry %d in TPL table", 
                                    busIndex, deviceObj->deviceAddress, tplSearch);

                            deviceObj->deviceClientDriver = (USB_HOST_CLIENT_DRIVER *)(tpl->hostClientDriver);
                            break;
                        }
                    }
                    else if((deviceDescriptor->idVendor == tpl->id.vid_pid.vid) && 
                            (deviceDescriptor->idProduct == tpl->id.vid_pid.pid))
                    {
                        /* Criteria matched */
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Bus %d Device %d matched entry %d in TPL table", 
                                busIndex, deviceObj->deviceAddress, tplSearch);
                        deviceObj->deviceClientDriver = (USB_HOST_CLIENT_DRIVER *)(tpl->hostClientDriver);
                        break;
                    }
                }
            }

            if(deviceObj->deviceClientDriver != NULL)
            {
                /* This means a driver was found. Call the driver assign function */
                SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Bus %d Assigning device level driver to device %d", busIndex, deviceObj->deviceAddress);
                deviceObj->deviceClientDriver->deviceAssign(deviceObj->deviceClientHandle, deviceObj->deviceIdentifier, &(deviceObj->deviceDescriptor));
            }

            /* Irrespective of the search result, we keep track of where the search 
             * stopped. If the search stopped at the end and no driver was assigned
             * then we know there was no VID PID match for this device. */
            deviceObj->tplEntryTried = tplSearch;
        }
        
        if((deviceObj->tplEntryTried >= gUSBHostObj.nTPLEntries ) && (deviceObj->deviceClientDriver == NULL))
        {
            /* This means VID PID matching failed and it has reached the end of
             * the TPL table. The device can be owned at a VID PID level or a
             * device class subclass protocol level. If the VID PID matching
             * reached the end of the table then we should check if device level
             * class subclass protocol can be matched. All this only if the
             * device specifies class subclass protocol at a device level and
             * the device level class subclass protocol matching has not reached
             * the end of the table */

            if(deviceDescriptor->bDeviceClass != 0x0)
            {
                /* This means the device level class, subclass and protocol can 
                 * be matched. Check if we have already tried this */
                if((deviceObj->deviceClScPTried < gUSBHostObj.nTPLEntries) && 
                        (deviceObj->deviceClientDriver == NULL))
                {
                    /* Search for match from the last match position. If the
                     * device was just connected */

                    SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Bus %d Device %d Looking for Device Level CL SC P driver", busIndex, deviceObj->deviceAddress);
                    deviceObj->deviceClScPTried = _USB_HOST_FindClassSubClassProtocolDriver(deviceDescriptor->bDeviceClass,
                            deviceDescriptor->bDeviceSubClass, deviceDescriptor->bDeviceProtocol, deviceObj->deviceClScPTried + 1);

                    if(deviceObj->deviceClScPTried < gUSBHostObj.nTPLEntries)
                    {
                        /* This means we found a match. Assign the corresponding driver */
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Bus %d Device %d. Assiging Device CL SC P Driver %d", 
                                busIndex, deviceObj->deviceAddress, deviceObj->deviceClScPTried);
                        deviceObj->deviceClientDriver = (USB_HOST_CLIENT_DRIVER *)(gUSBHostObj.tpl[deviceObj->deviceClScPTried].hostClientDriver);
                        deviceObj->deviceClientDriver->deviceAssign(deviceObj->deviceClientHandle, deviceObj->deviceIdentifier, &(deviceObj->deviceDescriptor));
                    }
                }
            }
        }

        if(deviceObj->deviceClientDriver == NULL)
        {
            /* The device is not owned. The host layer must try to set the
             * configuration */

            if(OSAL_MUTEX_Lock(&(gUSBHostObj.mutexControlTransferObj), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
            {
                if((deviceObj->configurationState == USB_HOST_DEVICE_CONFIG_STATE_READY_FOR_CONFIG)
                        && (deviceDescriptor->bNumConfigurations > 0)
                        && (deviceObj->configDescriptorInfo.configurationNumber == USB_HOST_CONFIGURATION_NUMBER_INVALID))
                {
                    /* The device is not configured, is ready to be configured
                     * and has at least one configuration. Check if the control
                     * transfer object is available to implement this command.
                     * */
                    if(!deviceObj->controlTransferObj.inUse)
                    {
                        /* This means we can set the configuration. We set to the
                         * first configuration */
                        deviceObj->controlTransferObj.inUse = true;
                        deviceObj->requestedConfigurationNumber = 0;
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Bus %d Device %d Setting first configuration", busIndex, deviceObj->deviceAddress); 
                        deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_START;
                    }
                }

                OSAL_MUTEX_Unlock(&(gUSBHostObj.mutexControlTransferObj));
            }
            else
            {
                SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\r\nUSB Host Layer: Mutex Lock failed", busIndex, deviceObj->deviceAddress); 
                /* OSAL error must be handled here. This needs to be implemented
                 * */
            }
        }
    }
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceControlTransfer 
    ( 
        USB_HOST_CONTROL_PIPE_HANDLE pipeHandle
        USB_HOST_TRANSFER_HANDLE * transferHandle
        USB_SETUP_PACKET * setupPacket,
        void * data,
        USB_HOST_DEVICE_CONTROL_REQUEST_COMPLETE_CALLBACK callback,
        uintptr_t context
    );

  Summary:
    Schedules a control transfer.

  Description:
    This function schedules a control transfer. pipeHandle contains a handle to
    a control pipe obtained through the USB_HOST_DeviceControlPipeOpen() function.
    setupPacket points to the setup command to be sent in the Setup Stage of the
    control transfer. The size and the direction of the data stage is indicated
    by the setup packet. In case of control transfers where there is no data
    stage, data is ignored and can be NULL. In all other cases, data should point
    to the data to data be transferred in the data stage of the control
    transfer. 
    
    If the transfer was scheduled successfully, transferHandle will contain a
    transfer handle that uniquely identifies this transfer. If the transfer
    could not be scheduled successfully, transferHandle will contain
    USB_HOST_TRANSFER_HANDLE_INVALID.

    When the control transfer completes, the host layer will call the specified
    callback function. The context parameter specified here will be returned in
    the callback.

  Remarks:
    Refer to usb_host_client_driver.h for usage details.
*/

USB_HOST_RESULT USB_HOST_DeviceControlTransfer
(
    USB_HOST_CONTROL_PIPE_HANDLE pipeHandle,
    USB_HOST_TRANSFER_HANDLE * transferHandle,
    USB_SETUP_PACKET * setupPacket,
    void * data,
    USB_HOST_DEVICE_CONTROL_REQUEST_COMPLETE_CALLBACK callback,
    uintptr_t context
)
{
    USB_HOST_DEVICE_OBJ  *deviceObj;
    uint8_t deviceIndex ;
    uint16_t pnpIdentifier;
    USB_HOST_RESULT result = USB_HOST_RESULT_FAILURE;

    if(transferHandle == NULL)
    {
        /* transferHandle cannot be NULL */
        result = USB_HOST_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Set transfer handle to invalid as the default value */
        *transferHandle = USB_HOST_TRANSFER_HANDLE_INVALID;

        if(pipeHandle == USB_HOST_CONTROL_PIPE_HANDLE_INVALID)
        {
            /* Pipe handle is not valid */
            result = USB_HOST_RESULT_PIPE_HANDLE_INVALID;
        }
        else if(setupPacket == NULL)
        {
            /* Required parameters are NULL */
            result = USB_HOST_RESULT_PARAMETER_INVALID;
        }
        else if((setupPacket->wLength != 0) && (data == NULL))
        {
            /* If this is not a zero data stage control transfer then data cannot
             * be NULL. */
            result = USB_HOST_RESULT_PARAMETER_INVALID;
        }
        else
        {

            /* The control pipe handle is the same as the device object handle. We get
             * the index of the device object that owns this pipe. */
            deviceIndex =  USB_HOST_DEVICE_INDEX( pipeHandle );

            /* PNP identifier is needed for the IRP user data */
            pnpIdentifier = USB_HOST_PNP_IDENTIFIER( pipeHandle );

            /* Get a pointer to the device object */
            deviceObj = &gUSBHostDeviceList[deviceIndex];

            /* Get a mutual exclusion lock as this is a global resource */
            if(OSAL_MUTEX_Lock(&(gUSBHostObj.mutexControlTransferObj), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
            {
                if(!deviceObj->controlTransferObj.inUse)
                {
                    /* This means that there no control request in progress. We can assign
                     * request now. The transfer handle is updated to point to the device
                     * control transfer object. */

                    deviceObj->controlTransferObj.inUse = true;
                    *transferHandle = (USB_HOST_TRANSFER_HANDLE)(&deviceObj->controlTransferObj);
                }
                else
                {
                    /* A control transfer is in progress. */
                    result = USB_HOST_RESULT_REQUEST_BUSY;
                }

                /* Unlock the mutual exclusion */
                OSAL_MUTEX_Unlock(&(gUSBHostObj.mutexControlTransferObj));
            }
            else
            {
                /* The mutual exclusion could not be obtained */
                result = USB_HOST_RESULT_REQUEST_BUSY;
            }

            if(*transferHandle != USB_HOST_TRANSFER_HANDLE_INVALID)
            {
                /* Set up the control transfer object */
                deviceObj->controlTransferObj.requestType = USB_HOST_CONTROL_REQUEST_TYPE_CLIENT_DRIVER_SPECIFIC;
                deviceObj->controlTransferObj.controlIRP.data = data;
                deviceObj->controlTransferObj.controlIRP.setup = setupPacket;
                deviceObj->controlTransferObj.controlIRP.size = setupPacket->wLength;
                deviceObj->controlTransferObj.controlIRP.callback = _USB_HOST_DeviceControlTransferCallback;
                deviceObj->controlTransferObj.controlIRP.userData = _USB_HOST_ControlTransferIRPUserData(pnpIdentifier, 0, deviceIndex);
                deviceObj->controlTransferObj.context = context;
                deviceObj->controlTransferObj.callback = callback;

                if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, &(deviceObj->controlTransferObj.controlIRP)))
                {
                    /* There was a problem while submitting the IRP. Update the result and
                     * the transfer handle. Return the control transfer object back to the
                     * device object */

                    result = USB_HOST_RESULT_FAILURE;
                    deviceObj->controlTransferObj.inUse = false;
                    *transferHandle = USB_HOST_TRANSFER_HANDLE_INVALID;
                }
                else
                {
                    result = USB_HOST_RESULT_SUCCESS;
                }
            }
        }
    }

    return result;
}

// *****************************************************************************
/* Function:
    bool _USB_HOST_DeviceConfigurationDescriptorErrorCheck
    (
        USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor
    );

  Summary:
    This function checks the configuration descriptor for errors.

  Description:
    This function checks the configuration descriptor for errors. The following
    errors are checked. The sizes reported by each descriptor headers are added
    up to check if this sum is equal to total configuration descriptor size
    reported in the configuration descriptor header, The number of endpoint
    descriptors in an interface match the endpoints specified in the interface
    descriptor and the number of interfaces mentioned in the configuration
    header match the number of descriptors found in the configuration
    descriptor.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

bool _USB_HOST_DeviceConfigurationDescriptorErrorCheck
(
    USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor
)
{
    uint8_t * search;
    bool result = false;
    uint8_t bNumInterfaces;
    uint8_t bNumEndpoints;
    int uniqueInterfaces = 0;
    int nEndpointsFound = 0;
    uint8_t * endpointSearch;
    int currentInterfaceNumber = -1;
    USB_DESCRIPTOR_HEADER * descriptorHeader;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;
    uint16_t configDescriptorSize, analyzedSize = 0;
    
    /* This function checks the configuration descriptor for errors */

    if(configurationDescriptor == NULL)
	{
        /* The input parameter is not valid. Note that the result is already 
         * false. */
	}
    else
    {
        configDescriptorSize = configurationDescriptor->wTotalLength;

        /* Check the size of each descriptor in the configuration descriptor and
         * make sure that the size adds up. */

        search = (uint8_t *)(configurationDescriptor);
        while(search < ((uint8_t *)(configurationDescriptor) + configDescriptorSize))
        {
            /* Start adding the size of each descriptor */
            descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
            analyzedSize += descriptorHeader->size;
            search += descriptorHeader->size;
        }

        /* Check if the analyzed matches the size reported in the configuration 
         * descriptor header */

        if(analyzedSize != configDescriptorSize)
        {
            /* The size does not match. result is already false, so nothing to
             * do here. */
        }
        else
        {
            /* Now we check if the number of reported interface descriptors 
             * exist. Interfaces start with 0 */   
            
            search = (uint8_t *)(configurationDescriptor);
            
            /* Start the search after the configuration descriptor*/
            search += (sizeof(USB_CONFIGURATION_DESCRIPTOR)); 
            bNumInterfaces = configurationDescriptor->bNumInterfaces;
            
            /* Keep searching till we have either reached the end of the 
             * configuration descriptor or till we have found all the 
             * interfaces. While searching we need to notes that interfaces can
             * have alternate settings. */
            
            while(search < ((uint8_t *)(configurationDescriptor) + configDescriptorSize))
            {
                descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
                if(descriptorHeader->descType == USB_DESCRIPTOR_INTERFACE)
                {
                    /* We found an interface descriptor. We need to make sure
                     * that we have not found this before. */
                    interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)(search);
                    if(currentInterfaceNumber != interfaceDescriptor->bInterfaceNumber)
                    {
                        /* We have found an unique interface number */
                        currentInterfaceNumber = interfaceDescriptor->bInterfaceNumber;
                        uniqueInterfaces ++;
                    }
                }

                search += descriptorHeader->size;
                if (uniqueInterfaces >= bNumInterfaces)
                {
                    /* This means we have found all the interfaces. Don't bother
                     * searching any more. */
                    break;
                }
            }

            if(uniqueInterfaces < bNumInterfaces)
            {
                /* This means the configuration descriptor does not contain all
                 * the interface descriptors. result is still false so nothing
                 * to do here. */
                
            }
            else
            {
                /* Now we make sure the number of descriptors in the endpoint 
                 * match what is reported in the endpoint. */
                search = (uint8_t *)(configurationDescriptor);
                
                /* Start the search after the configuration descriptor*/
                search += (sizeof(USB_CONFIGURATION_DESCRIPTOR)); 
                bNumInterfaces = configurationDescriptor->bNumInterfaces;
                
                /* The logic in the code below requires the default value of
                 * result to be true. */
                result = true;
                
                /* Locate an interface descriptor and then check the number of  
                 * endpoints it has. */ 
                while(search < ((uint8_t *)(configurationDescriptor) + configDescriptorSize))
                {
                    descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
                    if(descriptorHeader->descType == USB_DESCRIPTOR_INTERFACE)
                    {
                        /* Found an interface descriptor. Now analyze the endpoints
                         * it contains. */

                        nEndpointsFound = 0;
                        interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)(search);
                        bNumEndpoints = interfaceDescriptor->bNumEndPoints;
                        endpointSearch = search + descriptorHeader->size;
                        
                        /* Start another while loop to search for endpoints */
                        while(endpointSearch < ((uint8_t *)(configurationDescriptor) + configDescriptorSize)) 
                        {
                            descriptorHeader = (USB_DESCRIPTOR_HEADER *)(endpointSearch);

                            /* Unless this is the last interface in the configuration
                             * descriptor,all endpoint belonging to an interface must
                             * be arranged in the descriptor between two interface
                             * descriptors. */
                            
                            if(descriptorHeader->descType == USB_DESCRIPTOR_INTERFACE)
                            {
                                /* This means we have reached another interface
                                 * descriptor but not the end of the configuration
                                 * descriptor. Stop searching and see how many
                                 * endpoints we found. */
                                break;
                            }
                            
                            if(descriptorHeader->descType == USB_DESCRIPTOR_ENDPOINT)
                            {
                                /* We found an endpoint. Increment the count */
                                nEndpointsFound ++;
                            }
                            endpointSearch += descriptorHeader->size;

                            if(nEndpointsFound >= bNumEndpoints)
                            {
                                /* We found all the breakpoints that we were 
                                 * looking for. */
                                break;
                            }
                        }

                        /* Did we find out all the endpoints */
                        if(nEndpointsFound != bNumEndpoints)
                        {
                            /* No point in continuing */
                            result = false;
                            break;   
                        }
                    }

                    /* Here if the descriptor was not an endpoint or if it was 
                     * an endpoint, then endpoint search was successful. */

                    if(result == false)
                    {
                        /* Stop processing altogether because while processing
                         * and interface, there was an endpoint mismatch */
                        break;
                    }
                    else
                    {
                        /* Continue to process interface descriptors*/
                        search += ((USB_DESCRIPTOR_HEADER *)(search))->size;
                    }
                }
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    uint8_t _USB_HOST_GetNewAddress( USB_HOST_BUS_OBJ *busObj )

  Summary:
    Searches and allocates a new device address.

  Description:
    This function searches and allocates a new device address.

  Remarks:
    This is a local function and should not be called by the application
    directly.
*/

uint8_t _USB_HOST_GetNewAddress( USB_HOST_BUS_OBJ *busObj )
{
    uint8_t tempAddress;

    /* Find Free address */
    for ( tempAddress = 1; tempAddress <= USB_HOST_DEVICES_NUMBER ; tempAddress++ )
    {
        if ((busObj->addressBits[ tempAddress / 8] & (1 << ( tempAddress % 8 ))) == 0)
        {
            break;
        }
    }

    /*Mark for allocated address */
    busObj->addressBits[ tempAddress / 8] |= (1 << ( tempAddress % 8 ));
    return tempAddress;
}

// *****************************************************************************
/* Function:
    void _USB_HOST_FillSetupPacket
    (
        USB_SETUP_PACKET *setupPacket ,
        uint8_t requestType,
        uint8_t request ,
        uint16_t value,
        uint16_t index,
        uint16_t length
    )

  Summary:
    Helper function to create setup packet.

  Description:
    Helper function to create setup packet

  Remarks:
    This is a local function and should not be called by the application
    directly.
*/

void _USB_HOST_FillSetupPacket
(
    USB_SETUP_PACKET *setupPacket ,
    uint8_t requestType,
    uint8_t request ,
    uint16_t value,
    uint16_t index,
    uint16_t length
)
{
    setupPacket->bmRequestType = requestType;
    setupPacket->bRequest = request ;
    setupPacket->wValue = value ;
    setupPacket->wIndex = index ;
    setupPacket->wLength = length;
}

// *****************************************************************************
/* Function:
    void _USB_HOST_MakeDeviceReady
    ( 
        USB_HOST_DEVICE_OBJ * deviceObj, 
        int busIndex
    )

  Summary:
    Maintains the state of the device at a device level.

  Description:
    Maintains the state of the device at a device level. Moves the state of the
    device from attached to ready. It opens the control transfer pipe and checks
    configuration descriptors for errors.

  Remarks:
    This is a local function and should not be called by the application
    directly.
*/

void _USB_HOST_MakeDeviceReady
(
    USB_HOST_DEVICE_OBJ *deviceObj, 
    int busIndex
)
{
    USB_HOST_BUS_OBJ * busObj;
    USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor;
    bool interruptIsEnabled;
   
    busObj = &(gUSBHostBusList[busIndex]);
    
    if(!deviceObj->inUse)
    {
        /* Although this should not happen, we make sure that we dont run tasks
         * for device object that is not valid */
    }
    else
    {
        switch (deviceObj->deviceState)
        {
            case USB_HOST_DEVICE_STATE_WAITING_FOR_ENUMERATION:

                /* If another device is enumerating on the bus, then we don't do
                 * anything. Only one device can enumerate on the bus */

                if(!busObj->deviceIsEnumerating)
                {
                    /* Remember which device is enumerating */
                    busObj->enumeratingDeviceIdentifier = deviceObj->deviceIdentifier;

                    /* Grab the flag */
                    busObj->deviceIsEnumerating = true;

                    /* Reset the device */
                    deviceObj->hubInterface->hubPortReset( deviceObj->hubHandle, deviceObj->devicePort );

                    /* Change the device state */
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_RESET_COMPLETE;

                    /* Check if the device object has any previous allocated memory. If
                     * so then free it up. Allocation is done for the
                     * configuration descriptor. */
                    if(deviceObj->configDescriptorInfo.configurationDescriptor != NULL)
                    {
                        USB_HOST_FREE(deviceObj->configDescriptorInfo.configurationDescriptor);
                        deviceObj->configDescriptorInfo.configurationDescriptor = NULL;
                    }

                    /* The holdingConfigurationDescriptor memory should be free
                     * but we double check this here just to be safe. */
                    if(deviceObj->holdingConfigurationDescriptor != NULL)
                    {
                        USB_HOST_FREE(deviceObj->holdingConfigurationDescriptor);
                        deviceObj->holdingConfigurationDescriptor = NULL;
                    }

                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device Attach detected. Starting Enumeration.", busIndex);
                }
                break;

            case USB_HOST_DEVICE_STATE_WAITING_FOR_RESET_COMPLETE:

                /* Check if the reset has completed */
                if(deviceObj->hubInterface->hubPortResetIsComplete( deviceObj->hubHandle ,deviceObj->devicePort ))
                {
                    /* The reset has completed. We can also obtain the speed of the
                     * device. We give a reset recovery delay to the device */
                    deviceObj->speed = deviceObj->hubInterface->hubPortSpeedGet(deviceObj->hubHandle, deviceObj->devicePort);
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_START_RESET_SETTLING_DELAY;
                }

                break;

            case USB_HOST_DEVICE_STATE_START_RESET_SETTLING_DELAY:

                /* In this state we start the Post Reset Settling delay */
                busObj->timerExpired = false;
                busObj->busOperationsTimerHandle = SYS_TMR_CallbackSingle(100, (uintptr_t ) busObj, _USB_HOST_TimerCallback);
                if(SYS_TMR_HANDLE_INVALID != busObj->busOperationsTimerHandle)
                {
                    /* Wait for the post bus reset to complete */
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_RESET_SETTLING_DELAY_COMPLETE;
                }
                else
                {
                    /* Continue to stay in the state */
                }
                break;

            case USB_HOST_DEVICE_STATE_WAITING_FOR_RESET_SETTLING_DELAY_COMPLETE:

                /* In this state we are waiting for the reset settling delay to
                 * complete. */

                if(busObj->timerExpired)
                {
                    busObj->busOperationsTimerHandle = SYS_TMR_HANDLE_INVALID;
                    /* Settling delay has completed. Now we can open default address
                     * pipe and and get the configuration descriptor */

                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device Reset Complete.", busIndex);
                    deviceObj->controlPipeHandle = deviceObj->hcdInterface->hostPipeSetup( deviceObj->hcdHandle,
                            USB_HOST_DEFAULT_ADDRESS , 0 /* Endpoint */, 
                            deviceObj->hubAddress /* Address of the hub */, 
                            deviceObj->devicePort /* Address of the port */, 
                            USB_TRANSFER_TYPE_CONTROL, /* Type of pipe to open */
                            0 /* bInterval */, 8 /* Endpoint Size */, deviceObj->speed );

                    if(DRV_USB_HOST_PIPE_HANDLE_INVALID == deviceObj->controlPipeHandle)
                    {
                        /* We need a pipe else we cannot proceed */
                        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Could not open control pipe. Device not supported.", busIndex);
                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                        if(gUSBHostObj.hostEventHandler != NULL)
                        {
                            /* Send an event to the application */
                            gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                        }

                        /* Release the device is enumerating flag. Another
                         * device can start enumerating. */
                        busObj->deviceIsEnumerating = false;
                    }
                    else
                    {
                        /* Create a setup command to get the device descriptor */
                        _USB_HOST_FillSetupPacket(  &(deviceObj->setupPacket),
                                ( USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE ),
                                USB_REQUEST_GET_DESCRIPTOR, ( USB_DESCRIPTOR_DEVICE << 8 ), 0 , 8 ) ;

                        /* Fill control IRP. Note the size of the control transfer data
                         * stage. We ask for the first 8 bytes of the device descriptor. */

                        deviceObj->controlTransferObj.inUse = true;
                        deviceObj->controlTransferObj.controlIRP.data = (void *) &( deviceObj->deviceDescriptor );
                        deviceObj->controlTransferObj.controlIRP.setup = &(deviceObj->setupPacket ) ;
                        deviceObj->controlTransferObj.controlIRP.size = 8 ;
                        deviceObj->controlTransferObj.controlIRP.callback = NULL;

                        /* Change device state to next state */
                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_GET_DEVICE_DESCRIPTOR_SHORT;
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Requesting Device Descriptor.", busIndex);

                        /* Submit the IRP */
                        if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit(deviceObj->controlPipeHandle, 
                                    &(deviceObj->controlTransferObj.controlIRP)))
                        {
                            /* We need to be able to send the IRP. We move the
                             * device to an error state. Close the pipe and send
                             * an event to the application. */
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device Descriptor IRP failed. Device not supported.", busIndex);
                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                            deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                            if(gUSBHostObj.hostEventHandler != NULL)
                            {
                                /* Send an event to the application */
                                gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                            }

                            /* Release the device is enumerating flag. Another
                             * device can start enumerating. */
                            busObj->deviceIsEnumerating = false;
                        }
                    }
                }
                break;

            case USB_HOST_DEVICE_STATE_WAITING_FOR_GET_DEVICE_DESCRIPTOR_SHORT:

                /* Check if the Device Descriptor was obtained */
                if ( deviceObj->controlTransferObj.controlIRP.status == USB_HOST_IRP_STATUS_COMPLETED )
                {
                    /* The IRP completed. deviceObj->deviceDescriptor has the device
                     * descriptor. We can move to addressing state. */

                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_SET_ADDRESS;

                    /* Reset the enumeration failure count */
                    deviceObj->enumerationFailCount = 0;

                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device Descriptor obtained. Setting device address.", busIndex);

                }
                else if ( deviceObj->controlTransferObj.controlIRP.status < USB_HOST_IRP_STATUS_COMPLETED )
                {
                    /* The IRP failed. We will either reset the device or place
                     * it in an error state. In either case the control pipe
                     * should be closed. */

                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);

                    /* Release the device is enumerating flag. This will give
                     * another device the chance to start enumerating. The
                     * enumeration of this device will be re-attempted after
                     * the possible enumeration of the other device. */
                    busObj->deviceIsEnumerating = false;

                    /* Check if we should retry the enumeration sequence */
                    if (deviceObj->enumerationFailCount < USB_HOST_ENUMERATION_RETRY_COUNT)
                    {
                        /* Yes we should retry. Update the retry count */
                        deviceObj->enumerationFailCount ++ ;

                        /* Set device state for reset */
                        deviceObj->deviceState =  USB_HOST_DEVICE_STATE_WAITING_FOR_ENUMERATION;
                        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device Descriptor Request Failed. Trying again.", busIndex);
                    }
                    else
                    {
                        /* We tried three times but were not able to get a proper
                         * device response. Place the device in an error state. */

                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device Request Failed 3 times. Device not supported.", busIndex);
                        if(gUSBHostObj.hostEventHandler != NULL)
                        {
                            /* Send an event to the application */
                            gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                        }
                    }
                }

                break;

            case USB_HOST_DEVICE_STATE_SET_ADDRESS:

                deviceObj->deviceAddress = _USB_HOST_GetNewAddress( busObj );

                /* Create the setup request */
                _USB_HOST_FillSetupPacket(  &(deviceObj->setupPacket),
                        ( USB_SETUP_DIRN_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE ),
                        USB_REQUEST_SET_ADDRESS , deviceObj->deviceAddress , 0 , 0 ) ;

                /* Create the IRP packet */
                deviceObj->controlTransferObj.controlIRP.data = (void *) ( deviceObj->buffer );
                deviceObj->controlTransferObj.controlIRP.setup = &( deviceObj->setupPacket ) ;
                deviceObj->controlTransferObj.controlIRP.size = 0 ;
                deviceObj->controlTransferObj.controlIRP.callback = NULL;

                /* Set the next host layer state */
                deviceObj->deviceState = USB_HOST_DEVICE_STATE_WATING_FOR_SET_ADDRESS_COMPLETE;
                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Setting Device Address to %d.", busIndex, deviceObj->deviceAddress);

                /* Submit the IRP */
                if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, & (deviceObj->controlTransferObj.controlIRP)))
                {
                    /* We need to be able to send the IRP. We move the device to
                     * an error state. Close the pipe and send an event to the
                     * application. The assigned address will be released when
                     * the device in un-plugged. */

                    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Set Addres IRP failed. Device not supported.", busIndex);

                    /* Move the device to error state */
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                    if(gUSBHostObj.hostEventHandler != NULL)
                    {
                        /* Send an event to the application */
                        gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                    }

                    /* Release the device is enumerating flag. Another device
                     * can start enumerating. */
                    busObj->deviceIsEnumerating = false;
                }

                break;

            case USB_HOST_DEVICE_STATE_WATING_FOR_SET_ADDRESS_COMPLETE:

                /* In this state the host is waiting for the set address request
                 * to complete */
                if ( deviceObj->controlTransferObj.controlIRP.status == USB_HOST_IRP_STATUS_COMPLETED )
                {
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Set Address complete", busIndex);

                    /* This means the Set Address request completed
                     * successfully. Now we can open an addressed control
                     * transfer pipe. Close the current control pipe. This one
                     * is to device address 0 */
                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle); 

                    /* Reset the enumeration failure count */
                    deviceObj->enumerationFailCount = 0x00;

                    /* Open the new addressed pipe */
                    deviceObj->controlPipeHandle = deviceObj->hcdInterface->hostPipeSetup( deviceObj->hcdHandle,
                            deviceObj->deviceAddress, 0 /* Endpoint */, deviceObj->hubAddress, deviceObj->devicePort,
                            USB_TRANSFER_TYPE_CONTROL/* Pipe type */, 0, /* bInterval */
                            deviceObj->deviceDescriptor.bMaxPacketSize0, deviceObj->speed );

                    if( DRV_USB_HOST_PIPE_HANDLE_INVALID == deviceObj->controlPipeHandle )
                    {
                        /* The control pipe could not be opened. We cannot
                         * do anything. We cannot support the device.
                         * Release the device address */

                        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d. Could not open addressed control pipe. Device not supported", busIndex);
                        _USB_HOST_FreeAddress ( deviceObj->deviceIdentifier );
                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;

                        /* Send an event to the application */
                        if(gUSBHostObj.hostEventHandler == NULL)
                        {
                            gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                        }

                        /* Release the device is enumerating flag. Another
                         * device can start enumerating. */
                        busObj->deviceIsEnumerating = false;
                    }
                    else
                    {
                        /* The pipe was opened and we can continue with the
                         * rest of the enumeration */
                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_POST_SET_ADDRESS_DELAY;
                    }
                }
                else
                {
                    if ( deviceObj->controlTransferObj.controlIRP.status < USB_HOST_IRP_STATUS_COMPLETED )
                    {
                        /* The Set Address Request failed. We should either
                         * retry or place the device in error state. In any case
                         * the the control pipe should be closed. */

                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);

                        /* Release the device is enumerating flag. This will give
                         * another device the chance to start enumerating. The
                         * enumeration of this device will be re-attempted after
                         * the possible enumeration of the other device. */
                        busObj->deviceIsEnumerating = false;

                        /* Should we retry? */
                        if (deviceObj-> enumerationFailCount < USB_HOST_ENUMERATION_RETRY_COUNT )
                        {
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d. Set Address failed. Trying again.", busIndex);

                            /* Yes we should. Increment the failure count */
                            deviceObj->enumerationFailCount ++ ;

                            /* The device address must be released because this
                             * will be attempted again */
                            _USB_HOST_FreeAddress(deviceObj->deviceIdentifier);

                            /* Set device state for enumeration */
                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_ENUMERATION; 
                        }
                        else
                        {
                            /* We have tried enumeration multiple times and
                             * failed. */

                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d. Set Address failed 3 times. Device not supported.", busIndex);
                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                            if(gUSBHostObj.hostEventHandler != NULL)
                            {
                                /* Send the event to the application */
                                gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                            }
                        }
                    }
                }

                break;

            case USB_HOST_DEVICE_STATE_POST_SET_ADDRESS_DELAY:

                /* After the address has been set, we provide a delay of 50
                 * milliseconds */
                busObj->timerExpired = false;
                busObj->busOperationsTimerHandle = SYS_TMR_CallbackSingle( 50, (uintptr_t ) busObj, _USB_HOST_TimerCallback);;

                if(SYS_TMR_HANDLE_INVALID != busObj->busOperationsTimerHandle)
                {
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_POST_SET_ADDRESS_DELAY;
                }
                break;

            case USB_HOST_DEVICE_STATE_WAITING_POST_SET_ADDRESS_DELAY:

                /* Here we check if the post device set address delay has
                 * completed */
                if(busObj->timerExpired)
                {
                    busObj->busOperationsTimerHandle = SYS_TMR_HANDLE_INVALID ;
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Post Set Address Delay completed.", busIndex);
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Requesting Full Device Descriptor.", busIndex, deviceObj->deviceAddress);
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_GET_DEVICE_DESCRIPTOR_FULL;
                }
                break;

            case USB_HOST_DEVICE_STATE_GET_DEVICE_DESCRIPTOR_FULL:

                /* In the state the host layer requests for the full device
                 * descriptor. Create the setup packet. */
                _USB_HOST_FillSetupPacket( &(deviceObj->setupPacket), ( USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE ),
                        USB_REQUEST_GET_DESCRIPTOR, ( USB_DESCRIPTOR_DEVICE << 8 ), 0 , deviceObj->deviceDescriptor.bLength ) ;

                /* Fill IRP */
                deviceObj->controlTransferObj.controlIRP.data = (void *) &( deviceObj->deviceDescriptor );
                deviceObj->controlTransferObj.controlIRP.setup = &(deviceObj->setupPacket);
                deviceObj->controlTransferObj.controlIRP.size = deviceObj->deviceDescriptor.bLength;
                deviceObj->controlTransferObj.controlIRP.callback = NULL;

                deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_GET_DEVICE_DESCRIPTOR_FULL;

                /* Submit the IRP */
                if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, &(deviceObj->controlTransferObj.controlIRP)))
                {
                    /* We need to be able to send the IRP. We move the device to
                     * an error state. Close the pipe and send an event to the
                     * application. */

                    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Device Descriptor IRP failed. Device not supported.", busIndex, deviceObj->deviceAddress);
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                    if(gUSBHostObj.hostEventHandler != NULL)
                    {
                        /* Send an event to the application */
                        gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                    }

                    /* Release the device is enumerating flag. Another device
                     * can start enumerating. */
                    busObj->deviceIsEnumerating = false;
                }
                break;

            case USB_HOST_DEVICE_STATE_WAITING_FOR_GET_DEVICE_DESCRIPTOR_FULL:

                /* Here we are waiting for Get Full Device Descriptor to
                 * complete */
                if (deviceObj->controlTransferObj.controlIRP.status == USB_HOST_IRP_STATUS_COMPLETED)
                {
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Full Device Descriptor obtained.", busIndex, deviceObj->deviceAddress);

                    /* IRP was successful. Go to the next state */
                    deviceObj->deviceState =  USB_HOST_DEVICE_STATE_GET_CONFIGURATION_DESCRIPTOR_SHORT;

                    /* Reset the enumeration failure count */
                    deviceObj->enumerationFailCount = 0x00;

                    /* Update the number of configurations */
                    deviceObj->nConfiguration = deviceObj->deviceDescriptor.bNumConfigurations;

                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d contains %d configurations.", busIndex, deviceObj->deviceAddress, deviceObj->nConfiguration);

                    /* Reset the configuration check count to indicate that we are
                     * checking the first configuration */
                    deviceObj->configurationCheckCount = 0;
                }
                else
                {
                    /* The IRP did not complete successfully. */
                    if ( deviceObj->controlTransferObj.controlIRP.status < USB_HOST_IRP_STATUS_COMPLETED )
                    {
                        /* Close the pipe */
                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);

                        /* Release the device is enumerating flag. This will give
                         * another device the chance to start enumerating. The
                         * enumeration of this device will be re-attempted after
                         * the possible enumeration of the other device. */
                        busObj->deviceIsEnumerating = false;

                        /* Should we retry */
                        if (deviceObj->enumerationFailCount < USB_HOST_ENUMERATION_RETRY_COUNT )
                        {
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Device Descriptor Request Failed. Trying again", busIndex, deviceObj->deviceAddress);

                            /* Yes we should. Increment the enumeration
                             * count and reset the device */
                            deviceObj->enumerationFailCount ++ ;

                            /* The device address must be release because the
                             * enumeration process will be repeated */
                            _USB_HOST_FreeAddress(deviceObj->deviceIdentifier);

                            /* Set device state for enumeration */
                            deviceObj->deviceState =  USB_HOST_DEVICE_STATE_WAITING_FOR_ENUMERATION;
                        }
                        else
                        {
                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Device Request Failed 3 times. Device not supported.", busIndex, deviceObj->deviceAddress);

                            /* We should send an event to the application and
                             * then wait for device attach */
                            if(gUSBHostObj.hostEventHandler != NULL)
                            {
                                gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                            }
                        }
                    }
                }
                break;

            case USB_HOST_DEVICE_STATE_GET_CONFIGURATION_DESCRIPTOR_SHORT:

                /* In this state the host will get the configuration descriptor
                 * header. This is needed so that we know what is the
                 * configuration descriptor size  */

                _USB_HOST_FillSetupPacket( &(deviceObj->setupPacket), ( USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE ), USB_REQUEST_GET_DESCRIPTOR,
                        ( USB_DESCRIPTOR_CONFIGURATION << 8 )+ deviceObj->configurationCheckCount , 0 , 9 ) ;

                /* Fill IRP */
                deviceObj->controlTransferObj.controlIRP.data = ( void * )deviceObj->buffer;
                deviceObj->controlTransferObj.controlIRP.setup = &(deviceObj->setupPacket);
                deviceObj->controlTransferObj.controlIRP.size = 9;
                deviceObj->controlTransferObj.controlIRP.callback = NULL;

                deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_GET_CONFIGURATION_DESCRIPTOR_SHORT;

                /* Submit the IRP */
                if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, 
                            &(deviceObj->controlTransferObj.controlIRP)))
                {
                    /* We need to be able to send the IRP. We move the device to
                     * an error state. Close the pipe and send an event to the
                     * application. */
                    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Configuration Descriptor IRP failed. Device not supported.", busIndex, deviceObj->deviceAddress);

                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                    if(gUSBHostObj.hostEventHandler != NULL)
                    {
                        /* Send an event to the application */
                        gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                    }

                    /* Release the device is enumerating flag. Another
                     * device can start enumerating. */
                    busObj->deviceIsEnumerating = false;
                }

                break;

            case USB_HOST_DEVICE_STATE_WAITING_FOR_GET_CONFIGURATION_DESCRIPTOR_SHORT:
                /* Here we are waiting for Get Short Configuration Descriptor to
                 * complete */
                if (deviceObj->controlTransferObj.controlIRP.status == USB_HOST_IRP_STATUS_COMPLETED)
                {
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Short Configuration Descriptor Request passed.", busIndex, deviceObj->deviceAddress);
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Getting Full Configuration Descriptor.", busIndex, deviceObj->deviceAddress);

                    /* IRP was successful. Go to the next state */
                    deviceObj->deviceState =  USB_HOST_DEVICE_STATE_GET_CONFIGURATION_DESCRIPTOR_FULL;

                    /* Reset the enumeration failure count */
                    deviceObj->enumerationFailCount = 0x00;
                }
                else
                {
                    /* The IRP did not complete successfully. */
                    if ( deviceObj->controlTransferObj.controlIRP.status < USB_HOST_IRP_STATUS_COMPLETED )
                    {
                        /* Close the pipe */
                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);

                        /* Release the device is enumerating flag. This will give
                         * another device the chance to start enumerating. The
                         * enumeration of this device will be re-attempted after
                         * the possible enumeration of the other device. */
                        busObj->deviceIsEnumerating = false;

                        /* Should we retry */
                        if (deviceObj->enumerationFailCount < USB_HOST_ENUMERATION_RETRY_COUNT )
                        {
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Configuration Descriptor Request Failed. Trying again", busIndex, deviceObj->deviceAddress);

                            /* Yes we should. Increment the enumeration count
                             * and reset the device */
                            deviceObj->enumerationFailCount ++ ;

                            /* Set device state for enumeration */
                            deviceObj->deviceState =  USB_HOST_DEVICE_STATE_WAITING_FOR_ENUMERATION;

                            /* Release the device address */
                            _USB_HOST_FreeAddress(deviceObj->deviceIdentifier);
                        }
                        else
                        {
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Configuration Request Failed 3 times. Device not supported.", busIndex, deviceObj->deviceAddress);
                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;

                            /* We should send an event to the application and
                             * then wait for device attach */
                            if(gUSBHostObj.hostEventHandler != NULL)
                            {
                                gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                            }
                        }
                    }
                }
                break;

            case USB_HOST_DEVICE_STATE_GET_CONFIGURATION_DESCRIPTOR_FULL:

                /* Here we will try to allocate memory for the full
                 * configuration descriptor and then get the full configuration
                 * descriptor. */  

                configurationDescriptor = (USB_CONFIGURATION_DESCRIPTOR *)(deviceObj->buffer);
                deviceObj->holdingConfigurationDescriptor = USB_HOST_MALLOC(configurationDescriptor->wTotalLength);

                if(deviceObj->holdingConfigurationDescriptor == NULL)
                {
                    /* The memory allocation failed. We need memory to continue.
                     * We have to stop here */
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                    deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                    if(gUSBHostObj.hostEventHandler != NULL)
                    {
                        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d. Insufficient memory for Configuration Descriptor", busIndex);
                        gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                    }

                    /* Release the device is enumerating flag. Another
                     * device can start enumerating. */
                    busObj->deviceIsEnumerating = false;
                }
                else
                {
                    /* Place a request for the full configuration descriptor */
                    _USB_HOST_FillSetupPacket( &(deviceObj->setupPacket), ( USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_DEVICE ),
                            USB_REQUEST_GET_DESCRIPTOR, ( USB_DESCRIPTOR_CONFIGURATION << 8 ) + deviceObj->configurationCheckCount ,
                            0 ,configurationDescriptor->wTotalLength) ;

                    /* Create the IRP. Note that the configuration descriptor is
                     * read into the holding configuration descriptor. The
                     * holding configuration descriptor memory area is used as a
                     * temporary holding area only. The memory is freed once the
                     * configuration descriptor has been checked for errors */

                    deviceObj->controlTransferObj.controlIRP.data = deviceObj->holdingConfigurationDescriptor;
                    deviceObj->controlTransferObj.controlIRP.setup = &(deviceObj->setupPacket);
                    deviceObj->controlTransferObj.controlIRP.size = configurationDescriptor->wTotalLength;
                    deviceObj->controlTransferObj.controlIRP.callback = NULL;
                    deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_GET_CONFIGURATION_DESCRIPTOR_FULL;

                    /* Submit the IRP */
                    if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, 
                                &(deviceObj->controlTransferObj.controlIRP)))
                    {
                        /* We need to be able to send the IRP. We move the
                         * device to an error state. Close the pipe and send an
                         * event to the application. */
                        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Configuration Request IRP failed. Device not supported.", busIndex, deviceObj->deviceAddress);
                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                        USB_HOST_FREE(deviceObj->holdingConfigurationDescriptor);
                        deviceObj->holdingConfigurationDescriptor = NULL;

                        if(gUSBHostObj.hostEventHandler != NULL)
                        {
                            /* Send an event to the application */
                            gUSBHostObj.hostEventHandler( USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context );
                        }

                        /* Release the device is enumerating flag. Another
                         * device can start enumerating. */
                        busObj->deviceIsEnumerating = false;
                    }
                }

                break;

            case USB_HOST_DEVICE_STATE_WAITING_FOR_GET_CONFIGURATION_DESCRIPTOR_FULL:

                /* In this state we are waiting for the full configuration
                 * descriptor */
                if (deviceObj->controlTransferObj.controlIRP.status == USB_HOST_IRP_STATUS_COMPLETED)
                {
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d Get Full Configuration Descriptor Request passed.", busIndex, deviceObj->deviceAddress);

                    /* Reset the failure counter */
                    deviceObj->enumerationFailCount = 0;

                    /* The configuration descriptor will be in
                     * holdingConfigurationDescriptor member of device object.
                     * Check it for errors */

                    if(_USB_HOST_DeviceConfigurationDescriptorErrorCheck(deviceObj->holdingConfigurationDescriptor) &&
                            (deviceObj->holdingConfigurationDescriptor->bNumInterfaces <= USB_HOST_DEVICE_INTERFACES_NUMBER))
                    {
                        /* This means there are no errors in the configuration
                         * descriptor. Have we checked all configuration
                         * descriptors. Update the configurationCheckCount
                         * variable */

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Device %d No errors detected in Configuration Descriptor %d.", 
                                busIndex, deviceObj->deviceAddress, deviceObj->configurationCheckCount);

                        deviceObj->configurationCheckCount ++;

                        if(deviceObj->configurationCheckCount >= deviceObj->nConfiguration)
                        {
                            /* This means we have checked all the configurations
                             * and there are no errors. We are okay to move the
                             * device to the ready state */

                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_READY;

                            /* Release the control transfer object */
                            deviceObj->controlTransferObj.inUse = false;

                            /* Release the device is enumerating flag. Another
                             * device can start enumerating. */
                            busObj->deviceIsEnumerating = false;
                        }
                        else
                        {
                            /* Check the next configuration specified by
                             * configurationCheckCount */

                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_GET_CONFIGURATION_DESCRIPTOR_SHORT;
                        }

                        /* Free up the allocated memory */
                        USB_HOST_FREE(deviceObj->holdingConfigurationDescriptor);
                        deviceObj->holdingConfigurationDescriptor = NULL;
                    }
                    else
                    {
                        /* The configuration check failed. Either there was an
                         * error in the configuration or the configuration has
                         * too many interfaces. We cannot use this
                         * device */

                        deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);
                        USB_HOST_FREE(deviceObj->holdingConfigurationDescriptor);
                        deviceObj->holdingConfigurationDescriptor = NULL;
                        if(gUSBHostObj.hostEventHandler != NULL)
                        {
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Errors detected in Configuration Descriptor %d. Device not supported", busIndex, deviceObj->deviceAddress, deviceObj->configurationCheckCount);

                            gUSBHostObj.hostEventHandler(USB_HOST_EVENT_DEVICE_UNSUPPORTED, NULL, gUSBHostObj.context);
                        }

                        /* Release the device is enumerating flag. Another
                         * device can start enumerating. */
                        busObj->deviceIsEnumerating = false;
                    }
                }
                else
                {
                    if ( deviceObj->controlTransferObj.controlIRP.status < USB_HOST_IRP_STATUS_COMPLETED )
                    {
                        deviceObj->hcdInterface->hostPipeClose(deviceObj->controlPipeHandle);

                        /* Release the device is enumerating flag. This will give
                         * another device the chance to start enumerating. The
                         * enumeration of this device will be re-attempted after
                         * the possible enumeration of the other device. */
                        busObj->deviceIsEnumerating = false;
                        USB_HOST_FREE(deviceObj->holdingConfigurationDescriptor);
                        deviceObj->holdingConfigurationDescriptor = NULL;

                        if (deviceObj->enumerationFailCount < USB_HOST_ENUMERATION_RETRY_COUNT )
                        {
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Get Configuration Descriptor Request failed. Trying again", busIndex, deviceObj->deviceAddress);

                            /* Increment the failure count */
                            deviceObj->enumerationFailCount ++ ;

                            /* Set device state for enumeration */
                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_ENUMERATION;

                            /* Release allocated device address */
                            _USB_HOST_FreeAddress(deviceObj->deviceIdentifier);
                        }
                        else
                        {
                            /* Move the device to error state */
                            deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR;
                            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Bus %d Device %d Get Configuration Descriptor Request failed 3 times. Device not supported", busIndex, deviceObj->deviceAddress);
                            if(gUSBHostObj.hostEventHandler != NULL)
                            {
                                gUSBHostObj.hostEventHandler(USB_HOST_EVENT_DEVICE_UNSUPPORTED , NULL, gUSBHostObj.context );
                            }
                        }
                    }
                }
                break;
                
            case USB_HOST_DEVICE_STATE_ERROR:
                
                /* The device has entered an error state.  The control pipe
                 * should have been closed already.  We will only assign an
                 * invalid value to the pipe handle.  Note that we are doing
                 * this in an atomically, just to be safe.  The device then
                 * enters the Error holding state. In this state, the device
                 * must be detached. */
                
                interruptIsEnabled = SYS_INT_Disable();
                
                deviceObj->controlPipeHandle = DRV_USB_HOST_PIPE_HANDLE_INVALID;
                
                if(interruptIsEnabled)
                {
                    SYS_INT_Enable();
                }
                
                deviceObj->deviceState = USB_HOST_DEVICE_STATE_ERROR_HOLDING;
                break;
                
            case USB_HOST_DEVICE_STATE_ERROR_HOLDING:
                
                break;

            case USB_HOST_DEVICE_STATE_READY:

                /* This means the device state is ready for further processing.
                 * The host layer will now try to match the device to the
                 * client driver. The client driver can start interacting with
                 * the device */
                break;
            default:
                break;
        }
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_UpdateDeviceTask(int busIndex)

  Summary:
    This function maintains the state of each device on the bus.

  Description:
    This function maintains the state of each device on the bus.

  Remarks:
    This is a local function and should not be called directly by the 
    application.
*/

void _USB_HOST_UpdateDeviceTask(int busIndex)
{
    USB_HOST_BUS_OBJ * busObj;
    USB_HOST_DEVICE_OBJ * deviceObj = NULL ;

    busObj = &(gUSBHostBusList[busIndex]);

    /* The first device on the bus is the root hub. We don't maintain the state
     * of the root hub. So get the next device on the bus. */
    deviceObj = busObj->busDeviceList;
    deviceObj = deviceObj->nextDeviceObj;

    while(deviceObj != NULL)
    {
        /* Check if the device is addressed, get the device descriptor, check
         * all the configuration descriptors and move the device to the ready
         * state */
        _USB_HOST_MakeDeviceReady(deviceObj, busIndex);

        /* If the device is not owned then find a driver that can own it */
        _USB_HOST_UpdateDeviceOwnership (deviceObj, busIndex);
        
        /* If the configuration needs to be set, then set the configuration */
        _USB_HOST_UpdateConfigurationState(deviceObj, busIndex);
        
        /* If the device is configured, update the state of the interfaces */
        _USB_HOST_UpdateInterfaceStatus(deviceObj, busIndex);

        //_USB_HOST_UpdateClientDriverState ( deviceObj );

        deviceObj =   deviceObj->nextDeviceObj ;
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_FreeAddress ( USB_HOST_DEVICE_OBJ_HANDLE deviceIdentifier)

  Summary:
    Frees up the address bit assigned to this device hence making the address 
    available.

  Description:
    This function frees up the address bit assigned to this device hence making 
    the address available.

  Remarks:
    This is a local function and should not be called directly by the 
    application.
*/

void _USB_HOST_FreeAddress ( USB_HOST_DEVICE_OBJ_HANDLE deviceIdentifier  )
{
    uint8_t busNumber;
    int deviceIndex;
    uint8_t deviceAddress;
    USB_HOST_BUS_OBJ *busObj;
    USB_HOST_DEVICE_OBJ *deviceObj;

    /* Get the device array index and the bus number from the device object
     * handle */
    deviceIndex = USB_HOST_DEVICE_INDEX (deviceIdentifier);
    busNumber = USB_HOST_BUS_NUMBER (deviceIdentifier);
    
    busObj =  &( gUSBHostBusList[busNumber]);
    deviceObj = &( gUSBHostDeviceList [ deviceIndex ]);
    deviceAddress = deviceObj->deviceAddress;
    
    if(deviceAddress == 0)
    {
        /* Don't do anything */
    }
    else
    {
        /* The address is no longer being used. Clear up the bit assigned to this
         * device address. Address now becomes available. */
        busObj->addressBits[ deviceAddress/ 8 ] &= ~(1 << ( deviceAddress % 8));
    }
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT _USB_HOST_IRPResultToHostResult( USB_HOST_IRP * irp )
 
  Summary:
    This function maps the IRP completion result to a USB_HOST_RESULT type.

  Description:
    This function maps the IRP completion result to a USB_HOST_RESULT type.

  Remarks
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_RESULT _USB_HOST_IRPResultToHostResult(USB_HOST_IRP * irp)
{
    USB_HOST_RESULT result;
    switch(irp->status)
    {
        case USB_HOST_IRP_STATUS_ABORTED:
            /* IRP was terminated by the application */
            result = USB_HOST_RESULT_TRANSFER_ABORTED;
            break; 

        case USB_HOST_IRP_STATUS_ERROR_STALL:
            /* IRP was terminated because of a STALL */
            result = USB_HOST_RESULT_REQUEST_STALLED;
            break;

        case USB_HOST_IRP_STATUS_COMPLETED:
        case USB_HOST_IRP_STATUS_COMPLETED_SHORT:
            /* IRP has been completed */
            result = USB_HOST_RESULT_SUCCESS;
            break;

        case USB_HOST_IRP_STATUS_ERROR_UNKNOWN: 
        case USB_HOST_IRP_STATUS_ERROR_BUS: 
        case USB_HOST_IRP_STATUS_ERROR_DATA: 
        case USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT:
        default:
            result = USB_HOST_RESULT_FAILURE;
            break;
    }
    return (result);
}

// *****************************************************************************
/* Function:
    void _USB_HOST_DataTransferIRPCallback( USB_HOST_IRP * irp )
 
  Summary:
    This is the callback for IRPs submitted through the
    USB_HOST_DeviceTransfer() function.

  Description:
    This is the callback function for IRPs submitted through the
    USB_HOST_DeviceTransfer() function. The function will get the
    USB_HOST_TRANSFER_OBJ object associated with this IRP, find out the
    interface on which the transfer took place and then call the
    interfaceEventHandler function of the client driver that owns this
    interface.

  Remarks
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_DataTransferIRPCallback( USB_HOST_IRP * irp )
{
    USB_HOST_TRANSFER_OBJ * transferObj;
    USB_HOST_INTERFACE_DESC_INFO * interfaceInfo;
    USB_HOST_CLIENT_DRIVER * clientDriver;
    
    USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA eventData;

    /* The user data field of the IRP contains the address of the transfer
     * object the own this IRP. */
    transferObj = (USB_HOST_TRANSFER_OBJ *)(irp->userData);

    /* The transfer object contains the reference to the interface object */
    interfaceInfo = transferObj->interfaceInfoObj;

    /* Get a pointer to the client driver than owns this interface */
    clientDriver = interfaceInfo->interfaceDriver;

    /* Set up the event data object */
    eventData.length = irp->size;
    eventData.transferHandle = (USB_HOST_TRANSFER_HANDLE)(transferObj);

    /* We need to map IRP completion status to event data completion status */
    eventData.result = _USB_HOST_IRPResultToHostResult(irp);
    
    /* If the the USB_HOST_DeviceTransfer function will be called, it should 
     * know that the host layer is in an interrupt context */
    gUSBHostObj.isInInterruptContext = true;
    
    /* We need to make sure that the interface is owned because the device could
     * have been detached between the time that this transfer was submitted
     * and the callback arrived */
    
    if(clientDriver != NULL)
    {
        clientDriver->interfaceEventHandler(interfaceInfo->interfaceHandle, 
                USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE, &eventData, transferObj->context);
    }
    
    gUSBHostObj.isInInterruptContext = false;

    /* Deallocate the transfer object */
    transferObj->inUse = false;
}

// *****************************************************************************
/* Function:
    void _USB_HOST_DeviceControlTransferCallback( USB_HOST_IRP * irp )
 
  Summary:
    This is the callback for control IRPs submitted through the
    USB_HOST_DeviceControlTransfer() function.

  Description:
    This is the callback function for control IRPs submitted through the
    USB_HOST_DeviceControlTransfer() function. The function will get the
    USB_HOST_CONTROL_TRANSFER_OBJ object associated with this IRP, find out the
    type of control request call the callback function associated with the
    control transfer.

  Remarks
    This is a local function and should not be called directly by the
    application.
*/

void  _USB_HOST_DeviceControlTransferCallback( USB_HOST_IRP * irp )
{
    int deviceIndex;
    int interfaceIndex;
    USB_HOST_DEVICE_OBJ * deviceObj;
    USB_HOST_CONTROL_TRANSFER_OBJ * controlTransferObj;
    USB_HOST_INTERFACE_DESC_INFO * interfaceInfo;
    USB_HOST_RESULT result;
    USB_HOST_DEVICE_INTERFACE_EVENT_PIPE_HALT_CLEAR_COMPLETE_DATA pipeHaltEventData;
    USB_HOST_DEVICE_INTERFACE_EVENT_SET_INTERFACE_COMPLETE_DATA interfaceCompleteData;
    USB_HOST_DEVICE_EVENT_CONFIGURATION_DESCRIPTOR_GET_COMPLETE_DATA configurationGetCompleteData;

    /* The userData field of the IRP will be a bit map that contains the pnp
     * identifier, the control transfer object index and the index of the device
     * object that submitted this control transfer. */

    deviceIndex = USB_HOST_DEVICE_INDEX(irp->userData);
    deviceObj = &gUSBHostDeviceList[deviceIndex];

    controlTransferObj = &deviceObj->controlTransferObj;

    /* Map the IRP result to USB_HOST_RESULT */
    result = _USB_HOST_IRPResultToHostResult(irp); 

    switch ( controlTransferObj->requestType )
    {
        case USB_HOST_CONTROL_REQUEST_TYPE_CLIENT_DRIVER_SPECIFIC:

            /* This was a client driver specific control request. Call the
             * register callback */

            if(controlTransferObj->callback != NULL)
            {
                ((USB_HOST_DEVICE_CONTROL_REQUEST_COMPLETE_CALLBACK)(controlTransferObj->callback))(deviceObj->deviceIdentifier, 
                (USB_HOST_REQUEST_HANDLE)(controlTransferObj), result, irp->size, controlTransferObj->context );
            }
            break;

        case USB_HOST_CONTROL_REQUEST_TYPE_PIPE_HALT_CLEAR:

            /* This is standard control transfer request. Send the event to
             * the client driver that requested this. The IRP user data in this
             * case will also contain the interface index. */

            interfaceIndex = USB_HOST_INTERFACE_INDEX(irp->userData);
            interfaceInfo = &deviceObj->configDescriptorInfo.interfaceInfo[interfaceIndex];

            /* Prepare the event data */
            pipeHaltEventData.result = result;
            pipeHaltEventData.requestHandle = (USB_HOST_REQUEST_HANDLE)(controlTransferObj);

            /* Send the event to the interface event handler */
            if((interfaceInfo->interfaceDriver != NULL) && 
                    (interfaceInfo->interfaceDriver->interfaceEventHandler != NULL))
            {
                interfaceInfo->interfaceDriver->interfaceEventHandler(interfaceInfo->interfaceHandle,
                        USB_HOST_DEVICE_INTERFACE_EVENT_PIPE_HALT_CLEAR_COMPLETE, &pipeHaltEventData, controlTransferObj->context);
            }

            break;

        case USB_HOST_CONTROL_REQUEST_TYPE_INTERFACE_SET:

            /* This is standard control transfer request. Send the event to
             * the client driver that requested this. The IRP user data in this
             * case will also contain the interface index. */

            interfaceIndex = USB_HOST_INTERFACE_INDEX(irp->userData);
            interfaceInfo = &deviceObj->configDescriptorInfo.interfaceInfo[interfaceIndex];

            /* Prepare the event data */
            interfaceCompleteData.result = result;
            interfaceCompleteData.requestHandle = (USB_HOST_REQUEST_HANDLE)(controlTransferObj);

            if(result == USB_HOST_RESULT_SUCCESS)
            {
                /* This means the alternate setting request was successful. We
                 * should updated the current alternate setting on this
                 * interface */
                interfaceInfo->currentAlternateSetting = deviceObj->requestedAlternateSetting;
            }

            /* Send the event to the interface event handler */
            if((interfaceInfo->interfaceDriver != NULL) && 
                    (interfaceInfo->interfaceDriver->interfaceEventHandler != NULL))
            {
                interfaceInfo->interfaceDriver->interfaceEventHandler(interfaceInfo->interfaceHandle,
                        USB_HOST_DEVICE_INTERFACE_EVENT_SET_INTERFACE_COMPLETE, &interfaceCompleteData, controlTransferObj->context);
            }

            break;

        case USB_HOST_CONTROL_REQUEST_TYPE_STRING_DESCRIPTOR:

            /* This request originated from the application. The context and the
             * callback field of the control transfer object contain callback
             * and context specified by the application. */

            if(controlTransferObj->callback != NULL)
            {
                /* This means we have a callback that we can call */
                if(result == USB_HOST_RESULT_SUCCESS)
                {
                    ((USB_HOST_STRING_REQUEST_COMPLETE_CALLBACK)(controlTransferObj->callback))
                        ((USB_HOST_REQUEST_HANDLE)controlTransferObj, irp->size, controlTransferObj->context);

                }
                else
                {
                    /* The string descriptor request failed. Invoke the callback
                     * with string size as 0 */
                    ((USB_HOST_STRING_REQUEST_COMPLETE_CALLBACK)(controlTransferObj->callback))
                        ((USB_HOST_REQUEST_HANDLE)controlTransferObj, irp->size, controlTransferObj->context);
                }
            }

            break;

        case USB_HOST_CONTROL_REQUEST_TYPE_CONFIGURATION_DESCRIPTOR_GET:

            /* This request originated from the client driver. An event should
             * be sent to the device level event handler. Populate the event
             * data. */

            configurationGetCompleteData.requestHandle = (USB_HOST_REQUEST_HANDLE)(&deviceObj->controlTransferObj);
            configurationGetCompleteData.result = result; 
            
            if(deviceObj->deviceClientDriver != NULL)
            {
                deviceObj->deviceClientDriver->deviceEventHandler(deviceObj->deviceClientHandle,
                        USB_HOST_DEVICE_EVENT_CONFIGURATION_DESCRIPTOR_GET_COMPLETE, &configurationGetCompleteData, 
                        deviceObj->controlTransferObj.context);
            }


        default:
            break;
    }

    /* Release the control transfer object back */
    controlTransferObj->inUse = false;
}

// *****************************************************************************
// *****************************************************************************
// Section: USB HOST Layer System Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ USB_HOST_Initialize
    (
       const SYS_MODULE_INIT * const init
   )

  Summary:
    Initializes the USB Host layer instance specified by the index.

  Description:
    This routine initializes the USB Host Layer. This function must be called
    before any other Host layer function can be called. The initialization data
    is specified by the init parameter.  This function is typically called in
    the SYS_Initialize() function. The initialization completion may require the
    USB_HOST_Tasks() routine to execute.  The initialization function does not
    start the operation of the Host on the USB.  This must be done explicitly
    via the USB_HOST_BusEnable() function.

  Remarks:
    Refer to usb_host.h for usage information.
*/

SYS_MODULE_OBJ  USB_HOST_Initialize
(
    const SYS_MODULE_INIT * initData
)
{
    int hcCount;
    SYS_MODULE_OBJ result;
    uint32_t tplEntryCount;
    USB_HOST_BUS_OBJ *busObj;
    USB_HOST_TPL_ENTRY *tplEntry;
    USB_HOST_INIT *hostInit = NULL;
    USB_HOST_OBJ *hostObj = &(gUSBHostObj);

    /* Typecast the initialization data parameter and check if it is NULL. The
     * host layer cannot be initialized if the initialization data structure is
     * NULL. */

    hostInit = ( USB_HOST_INIT * ) initData ;
    result =SYS_MODULE_OBJ_INVALID;
    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host Layer: Entering USB_HOST_Initialize().");

    if(NULL  == hostInit )
    {
        SYS_DEBUG_MESSAGE( SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Initialization data is NULL is USB_HOST_Initialize().");
    }
    else
    {
        if(OSAL_RESULT_TRUE != OSAL_MUTEX_Create(&(gUSBHostObj.mutexControlTransferObj)))
        {
            /* Could not create the mutual exclusion */
            SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Could not create Control Transfer Mutex in USB_HOST_Initialize().");
        }
        else
        {
            if(OSAL_RESULT_TRUE != OSAL_MUTEX_Create(&(gUSBHostObj.mutexPipeObj)))
            {
                /* Could not create the mutual exclusion*/
                SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Could not create Pipe Object Mutex in USB_HOST_Initialize().");
            }
            else
            {
                if(OSAL_RESULT_TRUE != OSAL_MUTEX_Create(&(gUSBHostObj.mutexTransferObj)))
                {
                    /* Could not create the mutual exclusion */
                    SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Could not create Transfer Object Mutex in USB_HOST_Initialize().");
                }
                else
                {
                    /* Initialize the bus objects */
                    for ( hcCount = 0 ; hcCount < USB_HOST_CONTROLLERS_NUMBER ; hcCount++ )
                    {
                        busObj  = &(gUSBHostBusList[hcCount]);

                        /* Update the Host Controller Driver. This index will be used
                         * when the Host Layer opens the driver (at root hub enumeration
                         * time). */
                        busObj->hcdIndex = hostInit->hostControllerDrivers[hcCount].drvIndex;

                        /* By default bus state is disabled */
                        busObj->state = USB_HOST_BUS_STATE_DISABLED ;

                        /* Pointer to the HCD interface */
                        busObj->hcdInterface = ( DRV_USB_HOST_INTERFACE * ) ( hostInit->hostControllerDrivers [hcCount]).hcdInterface ;

                        /* Attached Device list will be NULL*/
                        busObj->busDeviceList = NULL ;

                        /* Initialize HCD handle to invalid */
                        busObj->hcdHandle   = DRV_HANDLE_INVALID ;

                        /* This flag is set when any device is enumerating on the bus. 
                         * Initialize as false. */
                        busObj->deviceIsEnumerating = false ;

                        /* Initialize handle of the system timer that this bus object will use */
                        busObj->busOperationsTimerHandle = SYS_TMR_HANDLE_INVALID;

                        /* Bus bandwidth constants */
                        busObj->totalBandwidth = 1000;
                        busObj->availableBandwidth = 1000;

                        /* Device Plug and Play identifier */
                        busObj->pnpIdentifier = 0x00;

                        /* Device address " 0 " for newly connected device 
                           Device address is " 1 " for reserved for RootHUB 
                           Mark there as reserved  */
                        busObj->addressBits[0] = ( uint8_t ) 0x03;
                    }

                    /* The host layer is now ready */
                    hostObj->status = SYS_STATUS_READY;

                    /* Get the pointer to the Host TPL and the
                     * number of entries in the TPL table. */

                    hostObj->tpl = hostInit->tplList ;
                    hostObj->nTPLEntries = hostInit->nTPLEntries;

                    /* Initialize all drivers in TPL List */
                    for ( tplEntryCount = 0 ; tplEntryCount < hostObj->nTPLEntries ; tplEntryCount++ )
                    {
                        tplEntry = &(hostObj->tpl[tplEntryCount]);
                        (( USB_HOST_CLIENT_DRIVER *)tplEntry->hostClientDriver)->initialize( tplEntry->hostClientDriverInitData );
                    }

                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host Layer: Exiting USB_HOST_Initialize() successfully.");
                    result = ((SYS_MODULE_OBJ)hostObj);
                }
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    SYS_STATUS USB_HOST_Status( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of USB_HOST_Status system interface function.

  Description:
    This is the dynamic implementation of USB_HOST_Status system interface
    function.

  Remarks:
    See usb_host.h for usage information.
*/

SYS_STATUS USB_HOST_Status (SYS_MODULE_OBJ usbHostObject)
{
    USB_HOST_OBJ * hostObj =  (USB_HOST_OBJ *)usbHostObject;
    SYS_STATUS result = SYS_STATUS_UNINITIALIZED;

    /* Check if we have a valid object */
    if(NULL ==  hostObj)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Invalid System Module Object in USB_HOST_Status().");
    }
    else
    {
        /* Return the system status of the Host Layer */
        result = hostObj->status;
    }

    return(result);
}

// *****************************************************************************
/* Function:
    void USB_HOST_Deinitialize ( SYS_MODULE_OBJ usbHostObject )

  Summary:
    Dynamic implementation of USB_HOST_Deinitialize system interface function.

  Description:
    This is the dynamic implementation of USB_HOST_Deinitialize system interface
    function.

  Remarks:
    See usb_host.h for usage information.
*/

void USB_HOST_Deinitialize ( SYS_MODULE_OBJ usbHostObject )
{
    /* Host object */
    USB_HOST_OBJ * hostObj =  (USB_HOST_OBJ *)usbHostObject;

    /* Check for NULL pointer */
    if(NULL == hostObj )
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "\r\nUSB Host Layer: Invalid System Module Object in USB_HOST_Deinitialize().");
    }
    else
    {
        /* Set the instance status to de-initialized */
        hostObj->status =  SYS_STATUS_UNINITIALIZED ;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: USB HOST Layer Client Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceGetFirst 
    (
        USB_HOST_BUS bus, 
        USB_HOST_DEVICE_INFO * deviceInfo
    );

  Summary:
    Returns information about the first attached device on the bus.

  Description:
    This function returns information about the first attached device on the
    specified bus. The USB_HOST_DeviceGetNext() function can be used to get the
    reference to the next attached device on the bus. The USB_HOST_DEVICE_INFO
    object is provided by the application.The device information will be
    populated into this object. If there are no devices attached on the bus, the
    function will set the deviceObjHandle parameter, in the USB_HOST_DEVICE_INFO
    object, to USB_HOST_DEVICE_OBJ_HANDLE INVALID. 
    
  Remarks:
    None.
*/

USB_HOST_RESULT USB_HOST_DeviceGetFirst 
(
    USB_HOST_BUS bus, 
    USB_HOST_DEVICE_INFO * deviceInfo
)
{
    USB_HOST_RESULT result = USB_HOST_RESULT_SUCCESS;

    USB_HOST_BUS_OBJ * busObj;
    USB_HOST_DEVICE_OBJ * deviceObj, * rootHub;

    if(bus >= USB_HOST_CONTROLLERS_NUMBER)
    {
        /* This is an invalid bus */
        result = USB_HOST_RESULT_BUS_UNKNOWN;
    }
    else if (NULL == deviceInfo)
    {
        /* Parameter is not valid */
        result = USB_HOST_RESULT_PARAMETER_INVALID;
    }
    else 
    {
        /* Get a pointer to the bus object */
        busObj = &gUSBHostBusList[bus];

        /* Set the initial value of the device Info object to invalid incase we
         * have to exit with a failure */
        deviceInfo->deviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;

        if(busObj->state < USB_HOST_BUS_STATE_ENABLED)
        {
            /* Bus is not enabled */
            result = USB_HOST_RESULT_BUS_NOT_ENABLED;
        }
        else 
        {
            /* Get the pointer to the first attached device. This will be the
             * root hub. The next device after the root hub is the device
             * attached to the bus.  */

            rootHub = busObj->busDeviceList;
            deviceObj = rootHub->nextDeviceObj;
            result = USB_HOST_RESULT_END_OF_DEVICE_LIST;

            while(deviceObj != NULL)
            {
                /* A device can be reported only if it is ready for interaction
                 * with the application. If we come across a device which is not
                 * ready, then we skip it */
                if(deviceObj->deviceState == USB_HOST_DEVICE_STATE_READY)
                {
                    /* We have a device connected on the bus. Populate the device
                     * info object with details about this device */
                    deviceInfo->deviceObjHandle = deviceObj->deviceIdentifier;
                    deviceInfo->deviceAddress = deviceObj->deviceAddress;
                    deviceInfo->bus = USB_HOST_BUS_NUMBER(deviceObj->deviceIdentifier); 
                    result = USB_HOST_RESULT_SUCCESS;
                    break;
                }
                else
                {
                    deviceObj = deviceObj->nextDeviceObj;
                }
            }
        }
    }
    
    return(result);
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceGetNext (USB_HOST_DEVICE_INFO * deviceInfo);

  Summary:
    Returns information about the next device on the bus.

  Description:
    This function returns information of the next device attached on the bus.
    The  USB_HOST_DeviceGetFirst() function should have been called at least once
    on the deviceInfo object.  Then calling this function repeatedly on the
    deviceInfo object will return information about the next attached device on
    the bus. When there are no more attached devices to report, the function
    returns USB_HOST_RESULT_END_OF_DEVICE_LIST.

    Calling the USB_HOST_DeviceGetFirst() function on the deviceInfo object
    after the USB_HOST_DeviceGetNext() function has been called will cause the
    host to reset the deviceInfo object to point to the first attached device.

  Remarks:
    None.
*/

USB_HOST_RESULT USB_HOST_DeviceGetNext (USB_HOST_DEVICE_INFO * deviceInfo)
{
    USB_HOST_RESULT result = USB_HOST_RESULT_SUCCESS;
    USB_HOST_DEVICE_OBJ * deviceObj = NULL;
    unsigned int index = 0;

    if(NULL == deviceInfo)
    {
        result = USB_HOST_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Get the device index */
        index = USB_HOST_DEVICE_INDEX(deviceInfo->deviceObjHandle);

        if(USB_HOST_DEVICES_NUMBER <= index)
        {
            /* Index is not valid. This should not happen unless the deviceInfo
             * has bee tampered with. */
            result = USB_HOST_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            /* Get a pointer to the device object */
            deviceObj = &gUSBHostDeviceList[index];

            /* Cross the PNP identifier against the PNP identifier of the device
             * that is in the device list. If they don't match then this means
             * that device was unplugged and another device has been connected
             * */

            if(USB_HOST_PNP_IDENTIFIER(deviceInfo->deviceObjHandle) != USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier))
            {
                result = USB_HOST_RESULT_DEVICE_UNKNOWN;
            }
            else 
            {
                /* Set default result */
                result = USB_HOST_RESULT_END_OF_DEVICE_LIST;
                deviceObj = deviceObj->nextDeviceObj;
                while(deviceObj != NULL)
                {

                    if(deviceObj->deviceState == USB_HOST_DEVICE_STATE_READY)
                    {
                        deviceInfo->deviceObjHandle = deviceObj->deviceIdentifier;
                        deviceInfo->bus = USB_HOST_BUS_NUMBER(deviceObj->deviceIdentifier);
                        deviceInfo->deviceAddress = deviceObj->deviceAddress;
                        result = USB_HOST_RESULT_SUCCESS;
                        break;
                    }
                    else
                    {
                        deviceObj = deviceObj->nextDeviceObj;
                    }
                }
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_EventHandlerSet
    (
        USB_HOST_EVENT_HANDLER * eventHandler,
        uintptr_t context
    )

  Summary:
    USB Host Layer Event Handler Callback Function set function.

  Description:
    This is the USB Host Layer Event Handler Callback Set function. An
    application can receive USB Host Layer events by using this function to
    register and event handler callback function. The application can
    additionally specify a specific context which will returned with the event
    handler callback function. The event handler must be set (this function must
    be called) before any of the USB buses are enabled.

  Remarks:
    See usb_host.h for usage information.
*/

USB_HOST_RESULT USB_HOST_EventHandlerSet
(
    USB_HOST_EVENT_HANDLER  eventHandler,
    uintptr_t context
)
{
    /* Assign the event handler. The event handler can be NULL in which case the
     * host layer will not generate events. The context is returned along with
     * the event */

    gUSBHostObj.hostEventHandler = eventHandler ;
    gUSBHostObj.context = context ;
    return (USB_HOST_RESULT_SUCCESS);
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceSpeedGet 
    (
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle, 
        USB_SPEED * speed
    )

  Summary:
    Returns the speed at which this device is operating.

  Description:
    This function returns the speed at which this device is operating.

  Precondition:
    The USB_HOST_Initialize() function should have been called. 

  Parameters:
    deviceObjHandle - handle to the device whose speed is required.

    speed - output parameter. Will contain the speed of the device if the
    function was successful.
    
  Returns:
    USB_HOST_RESULT_SUCCESS - The function was successful. speed will contain
    the speed of the device.
    USB_HOST_RESULT_DEVICE_UNKNOWN - The device does not exist in the system.
    speed will contain USB_SPEED_ERROR. 
    USB_HOST_RESULT_FAILURE - an unknown error occurred.

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

USB_HOST_RESULT USB_HOST_DeviceSpeedGet 
( 
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle, 
    USB_SPEED * speed
)
{
    USB_HOST_RESULT  result = USB_HOST_RESULT_SUCCESS;
    unsigned int index = 0;
    USB_HOST_DEVICE_OBJ * deviceObj = NULL;

    /* Check if the parameter is NULL */
    if(NULL == speed)
    {
       result = USB_HOST_RESULT_PARAMETER_INVALID;
    }
    else
    {
       /* Check if the device index is valid */
        index = USB_HOST_DEVICE_INDEX(deviceObjHandle);
        if( ( USB_HOST_DEVICES_NUMBER + USB_HOST_CONTROLLERS_NUMBER ) <  index) 
        {
            result = USB_HOST_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            /* Get the pointer to the device object */
            deviceObj = &gUSBHostDeviceList[index];

            /* Validate the plug and play identifier */
            if(USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier ) != USB_HOST_PNP_IDENTIFIER(deviceObjHandle))
            {
                /* The device handle is pointing to a device that does not exist
                 * in the system */
                result = USB_HOST_RESULT_DEVICE_UNKNOWN;
            }
            else
            {
                /* Everything checks out. Return the device speed */
                *speed = deviceObj->speed;
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_BusEnable(USB_HOST_BUS bus)

  Summary:
    Starts host operations.

  Description:
    The function starts the operation of the USB Host Bus. It enables the root
    hub associated with specified bus and starts the process of detecting
    attached devices and enumerating them. The USB_HOST_EventHandlerSet()
    function should have been called to register an application host layer event
    handler before the Host layer is enabled (before the USB_HOST_BusEnable()
    function is called).  This will ensure that the application does not miss
    any host events.

  Remarks:
    See usb_host.h for usage information.
*/

USB_HOST_RESULT USB_HOST_BusEnable(USB_HOST_BUS bus)
{
    USB_HOST_BUS_OBJ        *busObj;
    int                      hcCount;
    USB_HOST_RESULT          status = USB_HOST_RESULT_FALSE ;

    /* Note that this function only sets the state of the bus object
     * to indicate that the bus needs will be enabled. The actual enabling is
     * performed in the USB Host Layer Tasks Routine. */

    if ( bus == USB_HOST_BUS_ALL )
    {
        for ( hcCount = 0 ; hcCount < USB_HOST_CONTROLLERS_NUMBER ; hcCount++ )
        {
            busObj  = &(gUSBHostBusList[hcCount]);

            if(busObj->state < USB_HOST_BUS_STATE_ENABLING)
            {
                /* This means the bus is not enabled. Set the state to enable
                 * the bus. */
                busObj->state = USB_HOST_BUS_STATE_ENABLING ;
                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d About to Open Root Hub Driver.", hcCount);
            }
            status  =   USB_HOST_RESULT_SUCCESS ;
        }
    }
    /* Enable specific bus */
    else
    {
        /* Validate bus number */
        if( bus < 0 || bus >= USB_HOST_CONTROLLERS_NUMBER )
        {
            status = USB_HOST_RESULT_BUS_UNKNOWN;
        }
        else
        {
            busObj  = &(gUSBHostBusList[bus]);
            if(busObj->state < USB_HOST_BUS_STATE_ENABLING)
            {
                /* This means the bus is not enabled. Set the state to enable
                 * the bus. */
                busObj->state = USB_HOST_BUS_STATE_ENABLING ;
                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d About to Open Root Hub Driver.", bus);
            }
            status = USB_HOST_RESULT_SUCCESS;
        }
    }

    return ( status );
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_BusIsEnabled(USB_HOST_BUS bus)

  Summary:
    Checks if the bus is enabled.

  Description:
    The function returns the enable status of the bus. It can be called after
    the USB_HOST_BusEnable() function is called, to check if the bus has been
    enabled yet. If the bus parameter is set to USB_HOST_BUS_ALL, then the
    function will check the enable status of all the busses and will return true
    only if all the busses are enabled.

     Remarks:
    See usb_host.h for usage information.

  Remarks:
    None.
*/

USB_HOST_RESULT USB_HOST_BusIsEnabled(USB_HOST_BUS bus)
{
    USB_HOST_BUS_OBJ        *busObj;
    int                      hcCount;
    USB_HOST_RESULT          status = USB_HOST_RESULT_TRUE ;

    if ( bus == USB_HOST_BUS_ALL )
    {
        for ( hcCount = 0 ; hcCount < USB_HOST_CONTROLLERS_NUMBER ; hcCount++ )
        {
            busObj  = &(gUSBHostBusList[hcCount]);
            if(busObj->state <= USB_HOST_BUS_STATE_WAIT_FOR_ENABLE_COMPLETE)
            {
                /* This means at least one bus is being enabled. And so all
                 * buses are not enabled yet. */
                status = USB_HOST_RESULT_FALSE;
                break;
            }
            else
            {
                /* The default value of status is USB_HOST_RESULT_TRUE */
            }
        }
    }
    else
    {
        /* Validate bus number */
        if( bus < 0 || bus >= USB_HOST_CONTROLLERS_NUMBER )
        {
            status = USB_HOST_RESULT_BUS_UNKNOWN;
        }
        else
        {
            busObj  = &(gUSBHostBusList[bus]);
            if(busObj->state < USB_HOST_BUS_STATE_ENABLED)
            {
                /* The bus is in the process of being enabled or is not enabled
                 * at all. */
                status = USB_HOST_RESULT_FALSE;
            }
            else
            {
                /* The default value of status is USB_HOST_RESULT_TRUE */
            }
        }
    }

    return ( status );
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_BusSuspend (USB_HOST_BUS bus);

  Summary:
    Suspends the bus.

  Description:
    The function suspends the bus. All devices on the bus will be suspended. If
    bus  is specified as USB_HOST_BUS_ALL, all the buses managed by this host
    will be suspended.

  Precondition:
    The USB_HOST_BusEnable() function should have been called to enable the bus.

  Remarks:
    See usb_host.h for usage information.
*/

USB_HOST_RESULT USB_HOST_BusSuspend (USB_HOST_BUS bus)
{
    USB_HOST_BUS_OBJ        *busObj;
    int                     hcCount;
    USB_HOST_RESULT         status = USB_HOST_RESULT_SUCCESS ;

    if ( bus == USB_HOST_BUS_ALL )
    {
        /* Suspend all USB busses in the system */
        for ( hcCount = 0 ; hcCount < USB_HOST_CONTROLLERS_NUMBER ; hcCount++ )
        {
            busObj  = &(gUSBHostBusList[hcCount]);

            if( busObj->state < USB_HOST_BUS_STATE_ENABLED )
            {
                /* This means the bus is not enabled yet */
                status = USB_HOST_RESULT_BUS_NOT_ENABLED;
                break;
            }
            else if(busObj->state > USB_HOST_BUS_STATE_ENABLED)
            {
                /* This means the bus is already being suspended or is in a
                 * suspended state. */
            }
            else 
            {
                /* This means the bus is in enabled state. Set the state to
                 * suspending */
                busObj->state = USB_HOST_BUS_STATE_SUSPENDING;
            }
        }
    }
    else
    {
        /* Suspend a specific bus. Validate bus number */
        if( bus < 0 || bus >= USB_HOST_CONTROLLERS_NUMBER )
        {
            status = USB_HOST_RESULT_BUS_UNKNOWN;
        }
        else
        {   /* Set a state for enabling will be in task routine */
            busObj  = &(gUSBHostBusList[bus]);

            if( busObj->state < USB_HOST_BUS_STATE_ENABLED )
            {
                /* This means the bus is not enabled yet */
                status = USB_HOST_RESULT_BUS_NOT_ENABLED;
            }
            else if(busObj->state > USB_HOST_BUS_STATE_ENABLED)
            {
                /* This means the bus is already being suspended or is in a
                 * suspended state. */
            }
            else 
            {
                /* This means the bus is in enabled state. Set the state to
                 * suspending */
                busObj->state = USB_HOST_BUS_STATE_SUSPENDING;
            }
        }
    }

    return ( status );
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_BusIsSuspended (USB_HOST_BUS bus)

  Summary:
    Returns the suspend status of the bus.

  Description:
    This function returns suspend status of the specified USB bus. This function
    can be used to check the completion of the Resume operation started by using
    the USB_HOST_BusResume() function. If the Resume signaling has completed,
    the USB_HOST_BusIsSuspended() function would return USB_HOST_RESULT_FALSE
    indicating that the bus is not suspended.  Calling the
    USB_HOST_BusIsSuspended() with bus specified as USB_HOST_BUS_ALL returns the
    suspend status of the all USB segments that are managed by the host layer.
    The function would return USB_HOST_RESULT_TRUE only if all the bus are in a
    suspended state.

Remarks:
    See usb_host.h for usage information.
*/

USB_HOST_RESULT USB_HOST_BusIsSuspended (USB_HOST_BUS bus)
{
    USB_HOST_BUS_OBJ    *busObj;
    int                 hcCount;
    USB_HOST_RESULT     status = USB_HOST_RESULT_SUCCESS ;

    if ( bus == USB_HOST_BUS_ALL )
    {
        /* Check if all USB busses are suspended */
        for ( hcCount = 0 ; hcCount < USB_HOST_CONTROLLERS_NUMBER ; hcCount++ )
        {
            busObj  = &(gUSBHostBusList[hcCount]);
            if( busObj->state < USB_HOST_BUS_STATE_ENABLED)
            {
                /* Bus is not enabled yet */
                status = USB_HOST_RESULT_BUS_NOT_ENABLED ;
                break;
            }
            else if(busObj->state <= USB_HOST_BUS_STATE_SUSPENDING)
            {
                /* Bus is not suspended */
                status = USB_HOST_RESULT_FALSE;
                break;
            }
            else
            {
                /* No action required */
            }
        }
    }
    else
    {
        /* Check if this bus is suspended. Validate the bus number */
        if ( bus < 0 || bus >= USB_HOST_CONTROLLERS_NUMBER )
        {
            status = USB_HOST_RESULT_BUS_UNKNOWN ;
        }
        else
        {
            busObj  = &(gUSBHostBusList[bus]);
            if( busObj->state < USB_HOST_BUS_STATE_ENABLED)
            {
                /* Bus is not enabled yet */
                status = USB_HOST_RESULT_BUS_NOT_ENABLED ;
            }
            else if(busObj->state < USB_HOST_BUS_STATE_SUSPENDED)
            {
                /* Bus is not suspended */
                status = USB_HOST_RESULT_FALSE;
            }
            else
            {
                /* No action required */
            }
        }
    }

    return ( status );
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_BusResume (USB_HOST_BUS bus);

  Summary:
    Resumes the bus.

  Description:
    The function resumes the bus. All devices on the bus will be receive resume
    signaling. If bus is specified as USB_HOST_BUS_ALL, all the buses managed by
    this host will be resumed.

  Remarks:
    See usb_host.h for usage information.
*/

USB_HOST_RESULT USB_HOST_BusResume (USB_HOST_BUS bus)
{
    /* This function is not implemented in this release of the USB Host Layer */

    USB_HOST_RESULT status = USB_HOST_RESULT_FAILURE ;

    return status;
}

// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_OBJ_HANDLE USB_HOST_DeviceEnumerate
    (
        USB_HOST_DEVICE_OBJ_HANDLE parentHubObjHandle, 
        uint8_t port 
    );

  Summary:
    This function will request the host layer to enumerate an attached device.

  Description:
    This function will request the host layer to enumerate an attached device.
    It is called by the hub driver or the root hub when a device is attached.The
    function will return a device object handle to the caller. The caller must
    specify this handle when the device is detached.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
*/

USB_HOST_DEVICE_OBJ_HANDLE USB_HOST_DeviceEnumerate
(
    USB_HOST_DEVICE_OBJ_HANDLE parentDeviceIdentifier, 
    uint8_t port
)
{
    bool pnpIsUnique;
    uint32_t busNumber;
    uint32_t deviceCount = USB_HOST_CONTROLLERS_NUMBER;
    USB_HOST_BUS_OBJ * busObj;
    uint8_t parentDeviceNumber;
    USB_HOST_DEVICE_OBJ * newDeviceObj = NULL,* parentDeviceObj;
    USB_HOST_DEVICE_OBJ  * deviceObj;
    bool interruptWasEnabled;
    USB_HOST_DEVICE_OBJ_HANDLE result = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
    USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor = NULL;

    /* Get the bus number and the device object index of the parent device. This
     * is needed to get the bus object and the parent device object */

    busNumber =  USB_HOST_BUS_NUMBER(parentDeviceIdentifier ); 
    parentDeviceNumber = USB_HOST_DEVICE_INDEX(parentDeviceIdentifier );
    parentDeviceObj =  & gUSBHostDeviceList [ parentDeviceNumber ];
    busObj  = &(gUSBHostBusList[busNumber]);

    /* We disable all interrupts here. When a hub is attached, this function 
     * will be called from the IRP callback of the hub driver status IRP. This
     * callback runs in an interrupt context. We want the process of assigning
     * a device object to be atomic */

    interruptWasEnabled = SYS_INT_Disable();

    /* Now search for a free device object. The search must start from the
     * USB_HOST_CONTROLLER_NUMBERS because the first set of device objects in
     * the device object array are reserved for root hub devices. */

    for (deviceCount = USB_HOST_CONTROLLERS_NUMBER; 
            deviceCount < (USB_HOST_DEVICES_NUMBER + USB_HOST_CONTROLLERS_NUMBER); deviceCount ++ )
    {
        if (gUSBHostDeviceList[deviceCount].inUse == false )
        {
            /* This means we found a new object. Grab it and stop the search */
            newDeviceObj = &gUSBHostDeviceList[deviceCount];

            /* Before we reset this object to zero, we must make a backup of the
             * configuration descriptor pointer, so that we can free this memory
             * before we start the enumeration process. The memory cannot be
             * freed here because this function is called in an interrupt
             * function. */

            configurationDescriptor = newDeviceObj->configDescriptorInfo.configurationDescriptor;

            /* Completely clear up this object */
            memset (newDeviceObj, 0, sizeof(USB_HOST_DEVICE_OBJ));

            /* Grab this object */
            newDeviceObj->inUse  = true;
            break;
        }
    }

    /* We can now enable the interrupts */
    if(interruptWasEnabled)
    {
        SYS_INT_Enable();
    }

    if(newDeviceObj != NULL)
    {
        /* This means we found a device object. Initialize the new device
         * object. The HCD interface will be always the HCD interface of the
         * parent */

        newDeviceObj->hcdInterface = parentDeviceObj->hcdInterface;
        newDeviceObj->devicePort = port; 
        if (parentDeviceObj->deviceAddress == USB_HOST_ROOT_HUB_ADDRESS )
        {
            /* If the parent device is the root hub, the hub interface for
             * the device should be root hub interface */
            newDeviceObj->hubInterface = &(parentDeviceObj->hcdInterface->rootHubInterface.rootHubPortInterface);
            newDeviceObj->hubAddress = 0;
            newDeviceObj->hubHandle = busObj->hcdHandle;

        }
        else
        {
            /* If the parent device is a hub, then the hub interface for the
             * device should be the external hub interface */

            newDeviceObj->hubInterface = USB_HOST_HUB_INTERFACE;
            newDeviceObj->hubAddress = parentDeviceObj->deviceAddress;
            newDeviceObj->hubHandle = parentDeviceIdentifier;
        }

        /* These members of the device object need to be initialized to specific
         * values */
        newDeviceObj->parentDeviceIdentifier = parentDeviceIdentifier;
        newDeviceObj->hcdHandle = busObj->hcdHandle;
        newDeviceObj->deviceAddress = USB_HOST_DEFAULT_ADDRESS ;
        newDeviceObj->deviceState = USB_HOST_DEVICE_STATE_WAITING_FOR_ENUMERATION;
        newDeviceObj->tplEntryTried = -1;
        newDeviceObj->deviceClScPTried = -1;
        newDeviceObj->configDescriptorInfo.configurationNumber = USB_HOST_CONFIGURATION_NUMBER_INVALID;

        /* Note that this memory address that is being assigned here will be
         * freed up before the enumeration process starts. */
        newDeviceObj->configDescriptorInfo.configurationDescriptor = configurationDescriptor;

        /* Allocate a Plug N Play identifier. This identifier will be unique
         * to the attached device. It counts upwards and only repeats after
         * 0xFFFE. */

        do
        {
            /* Increment the pnpIdentifier. */
            busObj->pnpIdentifier ++;

            if(busObj->pnpIdentifier == 0xFFFE)
            {
                /* Rollover to start from 1 */
                busObj->pnpIdentifier = 1;
            }

            /* Check if this identifier is used by any device on this bus
             * */

            deviceObj = busObj->busDeviceList;
            pnpIsUnique = true;
            while(deviceObj != NULL)
            {
                if(USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier) == busObj->pnpIdentifier)
                {
                    /* The proposed PNP is already in use. Stop scanning */
                    pnpIsUnique = false;
                    break;
                }
                else
                {
                    /* Get the next device */
                    deviceObj = deviceObj->nextDeviceObj;
                }
            }
        } while(pnpIsUnique == false);

        /* Add this object to the bus device list */
        deviceObj = busObj->busDeviceList;

        /* Get to the end of the list */
        while( deviceObj->nextDeviceObj != NULL )
        {
            deviceObj = deviceObj->nextDeviceObj;
        }

        /* Add the object */
        deviceObj->nextDeviceObj = newDeviceObj;

        /* Create the device object handle */
        newDeviceObj->deviceIdentifier = _USB_HOST_DeviceObjHandleGet(busObj->pnpIdentifier, busNumber, deviceCount);
        newDeviceObj->deviceClientHandle = newDeviceObj->deviceIdentifier;

        /* Update result */
        result = newDeviceObj->deviceIdentifier;
    }
    else
    {
        /* We could not find a spare device object */
        SYS_DEBUG_MESSAGE( SYS_ERROR_INFO , "USB_HOST_DeviceEnumerate : Max Devices connected  \r\n");
    }

    return(result);
}

// *****************************************************************************
/* Function:
    void USB_HOST_DeviceDenumerate
    ( 
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle
    );

  Summary:
    De-enumerates an attached device.

  Description:
    This function de-enumerates an attached device. This function is called by the
    USB_HOST_DeviceDenumerate() function which in turn is called by the root hub
    or the external hub when a device is detached. The deviceObjHandle is the
    handle of the device that was detached. This is the same handle that was
    returned by the USB_HOST_DeviceEnumerate() function. This function will
    release the device and interface level drivers. It will then remove the
    object from the bus list and will deallocate the device object.

    If the object to be removed is a hub, the hub driver will call the
    USB_HOST_DeviceDenumerate() function for all its ports. This function will
    not search for child devices when a parent is removed.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void USB_HOST_DeviceDenumerate( USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle )
{
    USB_HOST_DEVICE_OBJ  * prevDeviceObj, * deviceObj, * deleteDeviceObj;
    USB_HOST_BUS_OBJ * busObj;
    USB_HOST_CONFIGURATION_INFO * configurationInfo;
    int index, busIndex;
    bool interruptIsEnabled;
    
    /* Check if the device object handle is valid. */
    if(deviceObjHandle != USB_HOST_DEVICE_OBJ_HANDLE_INVALID)
    {
        /* Get the device index from the device object handle */
        index = USB_HOST_DEVICE_INDEX(deviceObjHandle);
        deviceObj = &gUSBHostDeviceList[index];

        /* It is possible that this function can be called from two paths. The
         * main is the device detach path. The other path is the device
         * malfunction (possibly due to over current). The device malfunction
         * path will be detected in the Host Task routine context. But the
         * device detach path occurs in an interrupt context. We cannot allow
         * multiple entries on the same device. This may occur especially in the
         * case of a root hub, where the root hub driver will disable the power
         * to the device when it detects an overcurrent situation, but disabling
         * the power which may also cause a device detach interrupt, in which
         * case this function will re-enter with the same device object handle.
         * Hence we make this a critical section */

        interruptIsEnabled = SYS_INT_Disable();
        
        if(deviceObj->inUse)
        {
            busIndex = USB_HOST_BUS_NUMBER(deviceObjHandle);
            busObj = &gUSBHostBusList[busIndex];
            deleteDeviceObj = deviceObj;

            /* If there is device level client driver, then release the client driver */
            if(deviceObj->deviceClientDriver != NULL )
            {
                deviceObj->deviceClientDriver->deviceRelease( deviceObj->deviceClientHandle);
            }

            /* Get the configuration information */
            configurationInfo = &(deviceObj->configDescriptorInfo);

            if(configurationInfo->configurationDescriptor != NULL)
            {
                /* For each interface in the configuration, if the interface is assigned to
                 * client driver, then release these client drivers. */

                _USB_HOST_ReleaseInterfaceDrivers(deviceObj); 
            }

            /* Close the control pipe */
            deviceObj->hcdInterface->hostPipeClose ( deviceObj->controlPipeHandle );
            deviceObj->controlPipeHandle  = DRV_USB_HOST_PIPE_HANDLE_INVALID;

            /* Release address */
            _USB_HOST_FreeAddress (deviceObj->deviceIdentifier);

            /* Any dynamic memory that is allocated to the device is released
             * when the device object is used again. The denumerate function
             * which could be called from an interrupt context is really a bad
             * place to free up memory. */

            /* If this device was enumerating then release the enumeration flag */
            if((busObj->deviceIsEnumerating) && (busObj->enumeratingDeviceIdentifier == deviceObjHandle))
            {
                /* Clear the enumerating flag, so that we can let other device
                 * enumerate */
                busObj->deviceIsEnumerating = false;
            }

            /* The device needs to be removed from the bus list. The first device in the
             * bus list is the root hub. So prevDeviceObj here is the root hub device.
             * */ 
            prevDeviceObj = busObj->busDeviceList;

            /* The device connect to the root hub is the device attached to the bus. */
            deviceObj = prevDeviceObj->nextDeviceObj;

            while(deviceObj != NULL)
            {
                /* Check if this is the object to delete */
                if ( deleteDeviceObj == deviceObj  )
                {
                    /* Remove this device from the linked list */
                    prevDeviceObj->nextDeviceObj = deviceObj->nextDeviceObj ;

                    /* Clear the nextDeviceObj of the deleted device object */
                    deleteDeviceObj->nextDeviceObj = NULL;
                    break;
                }
                else
                {
                    /* Else continue search till we have reached end of the list */
                    prevDeviceObj = deviceObj;
                    deviceObj = deviceObj->nextDeviceObj;
                }
            }

            /* Deallocate the device object */
            deviceObj->inUse = false;
        }
        else
        {
            /* This means this object is not valid any more. We should not have
             * to do anything */
        }
        
        if(interruptIsEnabled)
        {
            SYS_INT_Enable();
        }
    }
    else
    {
        /* The device object handle is not valid. We don't have to do anything */
    }
}

// *****************************************************************************
/* Function:
    void USB_HOST_Tasks (SYS_MODULE_OBJ object );

  Summary:
    Maintains the USB Host Layer state machine.

  Description:
    This routine maintains the USB Host layer state machine. It must be called
    frequently to ensure proper operation of the USB. This function should be
    called from the SYS_Tasks function.

 Remarks:
    See usb_host.h for usage information.
*/

void USB_HOST_Tasks ( SYS_MODULE_OBJ usbHostObject )
{
    /* Host object */
    USB_HOST_OBJ            *hostObj =  (USB_HOST_OBJ *)usbHostObject;
    USB_HOST_BUS_OBJ        *busObj;
    USB_HOST_DEVICE_OBJ     *rootHubDevice;
    uint32_t                rootHubUHD;
    int                     hcCount;
 
    /* Check if the host layer is ready. We do not run the tasks routine
     * otherwise */
    if ( hostObj->status != SYS_STATUS_READY)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host Layer: Not ready in USB_HOST_Tasks().");
    }
    else
    {
        /* Maintain the state of each bus in the system */
        for ( hcCount = 0 ; hcCount < USB_HOST_CONTROLLERS_NUMBER ; hcCount++ )
        {
            busObj  = &(gUSBHostBusList[hcCount]);

            switch ( busObj->state)
            {
                case USB_HOST_BUS_STATE_DISABLED:
                    /* No action required */
                    break;

                case USB_HOST_BUS_STATE_ENABLING:

                    /* The bus is being enabled. Try opening the HCD */
                    busObj->hcdHandle = busObj->hcdInterface->open(busObj->hcdIndex, DRV_IO_INTENT_EXCLUSIVE | 
                            DRV_IO_INTENT_NONBLOCKING | DRV_IO_INTENT_READWRITE );

                    /* Validate the Open function status */
                    if (DRV_HANDLE_INVALID == busObj->hcdHandle )
                    {
                        /* The driver may not open the first time. This is okay. We
                         * should try opening it again. The state of bus is not
                         * changed. */
                    }
                    else
                    {
                        /* Update the bus root hub information with the
                         * details of the controller. Get the bus speed, number of
                         * ports, the maximum current that the HCD can supply,
                         * pointer to the root hub port functions. */

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Root Hub Driver Opened.",hcCount);
                        busObj->rootHubInfo.speed =  busObj->hcdInterface->rootHubInterface.rootHubSpeedGet(busObj->hcdHandle);
                        busObj->rootHubInfo.ports = busObj->hcdInterface->rootHubInterface.rootHubPortNumbersGet(busObj->hcdHandle);
                        busObj->rootHubInfo.power =  busObj->hcdInterface->rootHubInterface.rootHubMaxCurrentGet(busObj->hcdHandle);
                        busObj->rootHubInfo.rootHubPortInterface = busObj->hcdInterface->rootHubInterface.rootHubPortInterface;

                        /* We need to add the root hub as a device in the device
                         * list. In the gUSBHostDeviceList, the first
                         * USB_HOST_CONTROLLER_NUMBERS of objects is reserved for
                         * root hubs. Device Object 0 for root hub 0 and so on. We
                         * simply grab the device object reserved for the bus root
                         * hub that we processing and then initialize it. */

                        rootHubDevice = &(gUSBHostDeviceList[hcCount]);
                        rootHubDevice->inUse = true;

                        /* The plug and play identifier is a unique ID that gets
                         * incremented when a new device is attached to the bus. We
                         * consider the root hub enumeration as a new device */

                        busObj->pnpIdentifier = busObj->pnpIdentifier + 1 ;

                        /* The UHD for the root hub can now be formed. It is
                         * combination of the the PNP identifier, the
                         * gUSBHostDeviceList index and the bus number */

                        rootHubUHD =  _USB_HOST_DeviceObjHandleGet(busObj->pnpIdentifier, hcCount, hcCount);  

                        /* Root hub has  device identifier and parent device
                         * identifier same. The hub interface is the HCD root hub
                         * interface. Each of these device object is a linked list
                         * node. We set the next object to NULL. */
                        
                        rootHubDevice->deviceIdentifier = rootHubUHD ;
                        rootHubDevice->parentDeviceIdentifier = rootHubUHD;
                        rootHubDevice->hcdHandle = busObj->hcdHandle ;
                        rootHubDevice->deviceAddress = USB_HOST_ROOT_HUB_ADDRESS ;
                        rootHubDevice->deviceState = USB_HOST_DEVICE_STATE_READY ;
                        rootHubDevice->hcdInterface = busObj->hcdInterface;
                        rootHubDevice->hubInterface = (USB_HUB_INTERFACE *) & ( busObj->hcdInterface->rootHubInterface);
                        rootHubDevice->nextDeviceObj = NULL;

                        /* The first device in the bus is the root hub */
                        busObj->busDeviceList = rootHubDevice ;

                        /* Initialize the root hub. The device identifier passed to
                         * the initialize function is returned as the parent ID when
                         * the root hub request for device attach. We then enable
                         * the root hub operation. The root hub operation enable
                         * function does not enable the root hub immediately. So we
                         * will have to continue checking the progress of the enable
                         * operation in another state. */
                        busObj->hcdInterface->rootHubInterface.rootHubInitialize( busObj->hcdHandle , rootHubDevice->deviceIdentifier );
                        busObj->hcdInterface->rootHubInterface.rootHubOperationEnable( busObj->hcdHandle , true );
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Enabling Root Hub operation.",hcCount);
                        busObj->state = USB_HOST_BUS_STATE_WAIT_FOR_ENABLE_COMPLETE ;
                    }

                    break;

                case USB_HOST_BUS_STATE_WAIT_FOR_ENABLE_COMPLETE:

                    /* Check if the root hub operation enable function has
                     * completed. If yes, then the bus enable routine has completed
                     * and we can update the bus state */

                    if(busObj->hcdInterface->rootHubInterface.rootHubOperationIsEnabled(busObj->hcdHandle))
                    {
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Root Hub Operation Enabled.",hcCount);
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host Layer: Bus %d Updating Attached Device states.",hcCount);
                        busObj->state = USB_HOST_BUS_STATE_ENABLED;
                    }
                    break;

                case USB_HOST_BUS_STATE_ENABLED:

                    /* Now that the bus is enabled, we can update the state of every
                     * device that is attached on this bus. */

                    _USB_HOST_UpdateDeviceTask(hcCount);
                    break;

                case USB_HOST_BUS_STATE_DISABLING:

                    break;

                case USB_HOST_BUS_STATE_SUSPENDING:

                    break;

                case USB_HOST_BUS_STATE_SUSPENDED:

                    break;

                default :
                    break;
            }
        }
    }
}

// *****************************************************************************
/* Function:
    USB_ENDPOINT_DESCRIPTOR * USB_HOST_DeviceEndpointDescriptorQuery
    (
        USB_INTERFACE_DESCRIPTOR * interface
        USB_HOST_DESCRIPTOR_QUERY * query 
    );

  Summary:
    Queries the configuration for the specified endpoint type and returns a
    pointer to the endpoint descriptor if found.

  Description:
  This function queries the specified configuration for the specified endpoint
    type and returns a pointer to the endpoint descriptor if found. The return
    pointer will point to the standard endpoint descriptor and class specific
    endpoint descriptors for that endpoint. The search criteria can specified by
    using the flags. 
    
    In a case where there are multiple endpoints in the interface, the function
    can be called repetitively to continue the search till the end of the
    interface descriptor is reached or till the search fails.  The query object
    maintains the last point where the search was successful and continues the
    search from that point onwards. Resetting the query object (through the
    USB_HOST_DeviceEndpointQueryClear) function will reset the search object and
    cause the search to start from the top of the interface descriptor..

  Remarks:
    This function is available in the PIC32 implementation of the USB Host.
*/

USB_ENDPOINT_DESCRIPTOR * USB_HOST_DeviceEndpointDescriptorQuery
(
    USB_INTERFACE_DESCRIPTOR * interface,
    USB_HOST_ENDPOINT_DESCRIPTOR_QUERY * query 
)
{
    USB_ENDPOINT_DESCRIPTOR * result = NULL;
    USB_ENDPOINT_DESCRIPTOR * matchDescriptor = NULL;
    uint8_t * search;
    int bNumEndPoints, iterator;
    USB_DESCRIPTOR_HEADER * descriptorHeader;
    USB_HOST_ENDPOINT_QUERY_FLAG matchedCriteria = 0;
    uint8_t * lastLocation;

    /* Validate input parameters */

    if((interface == NULL) || (query == NULL))
    {
        result = NULL;
    }
    else
    {
        /* Get the number of endpoints in this interface. If the context is
         * already equal pointing to the last endpoint in this interface then
         * we don't have anything to search. */

        bNumEndPoints = interface->bNumEndPoints;
        if(bNumEndPoints == query->context)
        {
            result = NULL;
        }
        else
        {
            /* This means parameters are valid. Irrespective of the context we
             * always start the search from the start of the interface descriptor. 
             * The lastLocation will contain the end of the configuration
             * descriptor address. */

            search = (uint8_t *)(interface);
            lastLocation = (uint8_t *)(_USB_HOST_FindEndOfDescriptor(search));
            search += sizeof(USB_INTERFACE_DESCRIPTOR);
            for(iterator = 0; iterator < bNumEndPoints; iterator++) 
            {
                if(search < lastLocation)
                {
                    /* Clear the matched criteria for  fresh search */
                    matchedCriteria = 0;

                    descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
                    while(descriptorHeader->descType != USB_DESCRIPTOR_ENDPOINT)
                    {
                        /* This means we have found not found an endpoint descriptor
                         * yet. Continue searching. */

                        search += descriptorHeader->size;
                        descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
                    }

                    /* This means we have found an endpoint descriptor */
                    matchDescriptor = (USB_ENDPOINT_DESCRIPTOR *)(search);

                    /* Check if the query object already covered this in the
                     * previous search. We will know about this through the context
                     * */

                    if((query->context > 0) && (iterator <= query->context))
                    {
                        /* This means this endpoint was searched previously. Update
                         * search to point to the next descriptor. We should
                         * continue the search. */

                        search += sizeof(USB_ENDPOINT_DESCRIPTOR);
                        continue;

                    }

                    /* Now check for matching criteria */
                    if(query->flags & USB_HOST_ENDPOINT_QUERY_BY_ENDPOINT_ADDRESS)
                    {
                        /* Check if the endpoint address matches the address */
                        if(matchDescriptor->bEndpointAddress == query->endpointAddress)
                        {
                            /* This is a match. Update the context to remember on
                             * which iteration we matched. End the search. */
                            query->context = iterator + 1;
                            result = matchDescriptor;
                            break;
                        }
                        else
                        {
                            search += descriptorHeader->size;
                        }
                    }
                    else
                    {
                        if(query->flags & USB_HOST_ENDPOINT_QUERY_BY_DIRECTION)
                        {
                            /* Check if the endpoint direction matches */
                            if(query->direction == matchDescriptor->dirn)
                            {
                                /* Update matchedCriteria. This will help us check
                                 * if all the query specified criteria have been
                                 * satisfied. */
                                matchedCriteria |= USB_HOST_ENDPOINT_QUERY_BY_DIRECTION;
                            }
                        }

                        if(query->flags & USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE)
                        {
                            /* Check if the endpoint direction matches */
                            if(query->transferType == matchDescriptor->transferType)
                            {
                                /* Update matchedCriteria. This will help us check
                                 * if all the query specified criteria have been
                                 * satisfied. */
                                matchedCriteria |= USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE;
                            }
                        }          

                        if(matchedCriteria == query->flags)
                        {
                            /* This means all the criteria matched. We can end the
                             * search. */

                            query->context = iterator + 1;
                            result = matchDescriptor;
                            break;
                        }
                        else
                        {
                            /* This is not the endpoint descriptor. Advance the 
                             * search. */
                            search += descriptorHeader->size;
                        }
                    }
                }
                else
                {
                    /* This means we have reached the end of the descriptor. The
                     * search should stop. */
                }
            }
        } 
    }
    
    return(result);
}

// *****************************************************************************
/* Function:
    USB_INTERFACE_DESCRIPTOR * USB_HOST_DeviceInterfaceDescriptorQuery
    (
        USB_CONFIGURATION_DESCRIPTOR * configuration
        USB_HOST_DESCRIPTOR_QUERY * query,
    );

  Summary:
    Queries the active configuration for the specified interface.

  Description:
    This function queries the active configuration for the specified interface
    and returns a pointer to the interface descriptor if found. The return
    pointer will point to the standard interface descriptor and class specific
    interface descriptors for that interface. The search criteria can specified
    by using the flags. 
    
    In a case where the interface has more than one alternate settings, the
    function can be called repetitively to continue the search till the end of the
    configuration descriptor is reached or till the search fails. The query flag
    in such should be set to ignore the alternate setting field.  The query
    object maintains the last point where the search was successful and
    continues the search from that point onwards. Resetting the query object
    (through the USB_HOST_QueryClear) function will reset the search object and
    cause the search to start from the top of the configuration descriptor.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
*/


USB_INTERFACE_DESCRIPTOR * USB_HOST_DeviceInterfaceDescriptorQuery
(
    USB_CONFIGURATION_DESCRIPTOR * configuration,
    USB_HOST_INTERFACE_DESCRIPTOR_QUERY * query
)
{
    USB_INTERFACE_DESCRIPTOR * result = NULL;
    USB_INTERFACE_DESCRIPTOR * matchedInterface;
    USB_DESCRIPTOR_HEADER * descriptorHeader;
    uint16_t wTotalLength; 
    uint8_t * search;

    USB_HOST_INTERFACE_QUERY_FLAG matchedCriteria = 0;

    if((NULL == configuration) || (NULL == query))
    {
        /* Input parameter are NULL. Nothing to do here as the result is already
         * set to NULL. */
    }
    else
    {
        /* Input parameters are valid. Get the number of interfaces in this
         * configuration. This number does not include number of alternate
         * settings. The search must start after the configuration descriptor
         * header. */

        wTotalLength = configuration->wTotalLength;
        search = (uint8_t *)(configuration) + sizeof(USB_CONFIGURATION_DESCRIPTOR);

        /* Keep searching till we have reached the end of the configuration
         * descriptor or we have found a match */
        while(search < ((uint8_t *)(configuration) + wTotalLength))
        {
            /* Search for an interface descriptor */
            descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
            
            /* The matchedCriteria bitmap must be cleared before every search */
            matchedCriteria = 0;
            
            while(descriptorHeader->descType != USB_DESCRIPTOR_INTERFACE)
            {
                search += descriptorHeader->size;
                descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
                
                /* Its possible we may reach the end of the configuration 
                 * descriptor here. So lets check to make sure we have a
                 * bounded check. */
                
                if(search >= ((uint8_t *)(configuration) + wTotalLength))
                {
                    /* We are at the end of the configuration descriptor. */
                    search = NULL;
                    break;
                }
            }
            
            if(search == NULL)
            {
                /* This means the above search loop reached the end of the 
                 * configuration descriptor. */
                result = NULL;
                break;
            }

            /* This means we have found an interface descriptor. If the
             * query context (which is the location of the last search) is
             * greater than the current search location, this means this
             * interface was already covered in the previous search */

            if(search <= (uint8_t *)(query->context))
            {
                /* This result was already obtained in the previous search. We
                 * must continue the search. */

                search += descriptorHeader->size;
                continue;
            }

            /* We can now start match the interface to the query */
            matchedInterface = (USB_INTERFACE_DESCRIPTOR *)(search);

            if(query->flags & USB_HOST_INTERFACE_QUERY_BY_NUMBER)
            {
                /* Check if the interface number matches the specified interface
                 * number */
                if(matchedInterface->bInterfaceNumber == query->bInterfaceNumber)
                {
                    matchedCriteria |= USB_HOST_INTERFACE_QUERY_BY_NUMBER;
                }
            }

            if(query->flags & USB_HOST_INTERFACE_QUERY_ALT_SETTING)
            {
                /* Check if the alternate number matches the specified alternate
                 * setting */
                if(matchedInterface->bAlternateSetting == query->bAlternateSetting)
                {
                    matchedCriteria |= USB_HOST_INTERFACE_QUERY_ALT_SETTING;
                }
            }

            if(query->flags & USB_HOST_INTERFACE_QUERY_BY_CLASS)
            {
                /* Check if the class of the interface matches the specified
                 * class. */
                 if(query->bInterfaceClass == matchedInterface->bInterfaceClass)
                 {
                     matchedCriteria |= USB_HOST_INTERFACE_QUERY_BY_CLASS;
                 }
            }

            if(query->flags & USB_HOST_INTERFACE_QUERY_BY_SUBCLASS)
            {
                /* Check if the interface subclass matches the specified
                 * interface subclass */
                if(matchedInterface->bInterfaceSubClass == query->bInterfaceSubClass)
                {
                    matchedCriteria |= USB_HOST_INTERFACE_QUERY_BY_SUBCLASS;
                }
            }

            if(query->flags & USB_HOST_INTERFACE_QUERY_BY_PROTOCOL)
            {
                /* Check if the interface protocol matches the specified
                 * interface protocol. */
                if(matchedInterface->bInterfaceProtocol == query->bInterfaceProtocol)
                {
                    matchedCriteria |= USB_HOST_INTERFACE_QUERY_BY_PROTOCOL;
                }
            }

            if(matchedCriteria == query->flags)
            {
                /* This means we have matched the specified criteria. Save the
                 * location of the match and then stop searching. */
                
                query->context = (uintptr_t)(search);
                result = matchedInterface;
                break;
            }
            else
            {
                /* Did not match the query criteria. */
                search += descriptorHeader->size;
                continue;
            }
        }
    }
    
    return(result);
}

// *****************************************************************************
/* Function:
    void USB_HOST_DeviceEndpointQueryClear
    (
        USB_HOST_INTERFACE_DESCRIPTOR_QUERY * query
    );

  Summary:
    Clear the query object.

  Description:
    This function clears the query object context. Using the query after it has
    been clear will cause the USB_HOST_DeviceEndpointDescriptorQuery() and
    function to reset the search location to the start of the configuration
    descriptor.

  Remarks
    Refer to usb_host_client_driver.h for usage details.
*/

void USB_HOST_DeviceEndpointQueryContextClear
(
    USB_HOST_ENDPOINT_DESCRIPTOR_QUERY * query
)
{
    if(NULL != query)
    {
        /* Reset the context to 0 */
        query->context = 0;
    }
}

// *****************************************************************************
/* Function:
    void USB_HOST_DeviceInterfaceQueryContextClear
    (
        USB_HOST_INTERFACE_DESCRIPTOR_QUERY * query
    );

  Summary:
    Clear the query object.

  Description:
    This function clears the query object context. Using the query after it has
    been clear will cause the USB_HOST_DeviceInterfaceDescriptorQuery() and
    function to reset the search location to the start of the configuration
    descriptor.

  Remarks:
    Refer to usb_host_client_driver.h for usage details.
*/    

void USB_HOST_DeviceInterfaceQueryContextClear
(
    USB_HOST_INTERFACE_DESCRIPTOR_QUERY * query
)
{
    if(NULL != query)
    {
        /* Reset the context to 0 */
        query->context = 0;
    }
} 

// *****************************************************************************
/* Function:
    USB_HOST_CONTROL_PIPE_HANDLE USB_HOST_DeviceControlPipeOpen
    (
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle
    );

  Summary:
    Open a control transfer pipe to the device.

  Description:
    This function opens a control transfer pipe to the device. The return control
    pipe handle can be used in the USB_HOST_DeviceControlTransfer() function to
    schedule control transfers to the device.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
*/

USB_HOST_CONTROL_PIPE_HANDLE USB_HOST_DeviceControlPipeOpen
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle
)
{
    uint8_t deviceIndex ;
    uint16_t pnpIdentifier;
    USB_HOST_DEVICE_OBJ  * deviceObj;
    USB_HOST_CONTROL_PIPE_HANDLE  controlPipeHandle = USB_HOST_CONTROL_PIPE_HANDLE_INVALID;

    /* Get the device object index from the device object handle */
    deviceIndex =  USB_HOST_DEVICE_INDEX( deviceObjHandle );

    /* Get a pointer to the device object */
    deviceObj = &gUSBHostDeviceList[deviceIndex];

    if(deviceObj->inUse)
    {
        /* This means the device object is valid. Now check the plugNplay
         * identifier to make sure that device object handle is not stale */

        pnpIdentifier = USB_HOST_PNP_IDENTIFIER(deviceObjHandle);

        if(pnpIdentifier == USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier))
        {
            /* The plugNplay identifier matches the one that device object
             * contains. The control pipe handle is the same as the device
             * object handle */

            controlPipeHandle = (USB_HOST_CONTROL_PIPE_HANDLE)(deviceObj->deviceIdentifier);
        }
    }

    return controlPipeHandle;
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceInterfaceRelease
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    );

  Summary:
    Releases an interface on the specified device.

  Description:
    This function allows the application to release a claimed interface on the
    specified device and within the selected configuration on that device.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
 */

USB_HOST_RESULT USB_HOST_DeviceInterfaceRelease
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    int deviceIndex;
    int interfaceIndex;
    unsigned int pnpIdentifier;
    USB_HOST_RESULT result;
    USB_HOST_DEVICE_OBJ * deviceObj;
    USB_HOST_INTERFACE_DESC_INFO * interfaceInfo;
   
    
    deviceIndex = USB_HOST_DEVICE_INDEX(interfaceHandle);
    interfaceIndex = USB_HOST_INTERFACE_INDEX(interfaceHandle);
    pnpIdentifier = USB_HOST_PNP_IDENTIFIER(interfaceHandle);
    
    /* Get the device object */
    deviceObj = &gUSBHostDeviceList[deviceIndex];
    
    if(!deviceObj->inUse)
    {
        /* This device object is not in use. May have been disconnected */
        result = USB_HOST_RESULT_DEVICE_UNKNOWN;
    }
    else 
    {
        if(pnpIdentifier != USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier))
        {
            /* This device is not the same that client driver thinks it is */
            result = USB_HOST_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            interfaceInfo = &deviceObj->configDescriptorInfo.interfaceInfo[interfaceIndex];
            if(interfaceInfo->interfaceDescriptor == NULL)
            {
                /* This means that the interface is not valid */
                result = USB_HOST_RESULT_INTERFACE_UNKNOWN;
            }
            else
            {
                /* Release the interface */
                interfaceInfo->interfaceDriver = NULL;
                result = USB_HOST_RESULT_SUCCESS;
            }
        }
    }    
        
    return(result);
}

// *****************************************************************************
/* Function:
    void USB_HOST_DeviceTransfer
    ( 
        USB_HOST_PIPE_HANDLE pipeHandle
        USB_HOST_TRANSFER_HANDLE * transferHandle,
        void * data,
        size_t size,
        uintptr_t context
    );

  Summary:
    Schedules a bulk, interrupt or isochronous transfer.

  Description:
    This function schedules a bulk, interrupt or isochronous. pipeHandle
    contains a handle to a bulk, interrupt or isochronous pipe obtained through
    the USB_HOST_DevicePipeOpen() function .
    
    If the transfer was scheduled successfully, transferHandle will contain a
    transfer handle that uniquely identifies this transfer. If the transfer
    could not be scheduled successfully, transferHandle will contain
    USB_HOST_TRANSFER_HANDLE_INVALID.  

    Multiple transfers can be queued. These transfers will be processed in the
    order they were scheduled. The host layer will called the interfaceEvent
    event handler with USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA
    when the transfer completes.

  Remarks
    Refer to usb_host_client_driver.h for usage information.
*/

USB_HOST_RESULT USB_HOST_DeviceTransfer
(
    USB_HOST_PIPE_HANDLE pipeHandle,
    USB_HOST_TRANSFER_HANDLE * transferHandle,
    void * data,
    size_t size,
    uintptr_t context
)
{
    uint8_t deviceIndex;
    uint8_t interfaceIndex;
    USB_HOST_DEVICE_OBJ *deviceObj;
    USB_HOST_TRANSFER_OBJ * transferObj = NULL;
    USB_HOST_PIPE_OBJ *pipeObj;
    DRV_USB_HOST_PIPE_HANDLE drvPipeHandle;
    USB_HOST_INTERFACE_DESC_INFO *interfaceInfo;
    USB_HOST_RESULT result = USB_HOST_RESULT_SUCCESS ;
    OSAL_RESULT osalResult;
    USB_HOST_OBJ * hostObj;
    int search;

    if(NULL == transferHandle)
    {
        /* transferHandle cannot be NULL */
        result = USB_HOST_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Initialize transfer handle in anticipation of an error */
        *transferHandle = USB_HOST_TRANSFER_HANDLE_INVALID;

        if(USB_HOST_PIPE_HANDLE_INVALID == pipeHandle)
        {
            /* Pipe handle is not valid */
            result = USB_HOST_RESULT_PIPE_HANDLE_INVALID;
        }
        else if ((size != 0) && (NULL == data))
        {
            /* If size is greater than 0, then data cannot be NULL */
            result = USB_HOST_RESULT_PARAMETER_INVALID;
        }
        else
        {
            /* Pointer to the host layer object */
            hostObj = &gUSBHostObj;

            /* pipeHandle is pointer to the pipe object */
            pipeObj = (USB_HOST_PIPE_OBJ *)(pipeHandle);

            /* The pipeObj contains the handle to the interface that this pipe
             * belongs to. The interface handle contains the index of the owner
             * device. We need this owner device index so that we can get the
             * pointer to the HCD functions. */

            deviceIndex =  USB_HOST_DEVICE_INDEX( pipeObj->interfaceHandle );
            interfaceIndex =  USB_HOST_INTERFACE_INDEX ( pipeObj->interfaceHandle );
            deviceObj = &gUSBHostDeviceList[deviceIndex];
            interfaceInfo = &(deviceObj->configDescriptorInfo.interfaceInfo[interfaceIndex]);

            /* Get the driver pipe handle */
            drvPipeHandle = pipeObj->pipeHandle ;

            /* We need to now search for a free transfer object. It is possible that
             * the USB_HOST_DeviceTransfer() function will be called from the
             * transfer event. It is also possible that this function is called from
             * another thread. We will use a mutual exclusion to protect the transfer
             * object pool. But the mutual exclusion should be take only if the host if
             * this function is not being called in an interrupt context.*/

            if(!hostObj->isInInterruptContext)
            {
                /* We can take mutual exclusion only if we are not in interrupt context */
                osalResult = OSAL_MUTEX_Lock(&hostObj->mutexTransferObj, OSAL_WAIT_FOREVER);
            }
            else
            {
                /* We are in interrupt context. So the rest of the code flow
                 * should continue. */
                osalResult = OSAL_RESULT_TRUE;
            }

            /* The execution reaches this point (in an RTOS setting) if the
             * mutual exclusion lock was taken, or if the mutual exclusion failed. We must disable all
             * root hub events so this operation is atomic */
            
            if(osalResult == OSAL_RESULT_TRUE)
            {
                _USB_HOST_RootHubEventDisable();
                
                for(search = 0; search < USB_HOST_TRANSFERS_NUMBER; search ++)
                {
                    if(!gUSBHostTransferObj[search].inUse)
                    {
                        /* We found a transfer object. Allocate it and then update
                         * the transfer handle. */

                        transferObj = &gUSBHostTransferObj[search];
                        transferObj->inUse = true;
                        *transferHandle = (USB_HOST_TRANSFER_HANDLE)(transferObj);
                            break;
                    }
                }
                
                _USB_HOST_RootHubEventEnable();

                /* Return the mutual exclusion */
                if(!hostObj->isInInterruptContext)
                {
                    OSAL_MUTEX_Unlock(&(hostObj->mutexTransferObj));
                }

                if(*transferHandle == USB_HOST_TRANSFER_HANDLE_INVALID)
                {
                    /* This means we did not find a transfer object */
                    result = USB_HOST_RESULT_REQUEST_BUSY;
                }
                else
                {
                    /* We have found a transfer object. Initialize it */

                    transferObj->irp.data = (void *)data;
                    transferObj->irp.size = size ;
                    transferObj->irp.userData = (uintptr_t)(transferObj);
                    transferObj->irp.callback = _USB_HOST_DataTransferIRPCallback;
                    transferObj->context = context;
                    transferObj->interfaceInfoObj = interfaceInfo;

                    /* Try submitting the IRP */
                    if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( drvPipeHandle, & (transferObj->irp)))
                    {
                        /* If the IRP submit was not successful, then return the
                         * transferObject to the pool and update the result. */

                        result = USB_HOST_RESULT_FAILURE;
                        *transferHandle = USB_HOST_TRANSFER_HANDLE_INVALID;
                        transferObj->inUse = false;
                    }
                }
            }
            else
            {
                /* Could not take the mutual exclusion lock*/
                result = USB_HOST_RESULT_REQUEST_BUSY;
            }
        }
    }

    return result;
}

// *****************************************************************************
/* Function:
    USB_HOST_PIPE_HANDLE USB_HOST_DevicePipeOpen 
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
        USB_ENDPOINT_ADDRESS endpointAddress
    );

  Summary:
    Opens a pipe to the specified endpoint.

  Description:
    This function opens a pipe on the specified endpoint. The pipe parameters
    will be defined by the endpoint attributes specified in the endpoint
    descriptor that the attached device reports to the host.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
*/

USB_HOST_PIPE_HANDLE USB_HOST_DevicePipeOpen
(
    USB_HOST_DEVICE_INTERFACE_HANDLE deviceInterfaceHandle,
    USB_ENDPOINT_ADDRESS endpointAddress
)
{
    uint8_t  interval;
    uint8_t  pipeNumber;
    uint32_t deviceIndex;
    uint8_t  interfaceIndex;
    uint32_t maxPacketSize;

    USB_HOST_PIPE_HANDLE result;
    USB_TRANSFER_TYPE transferType;
    USB_HOST_PIPE_OBJ * pipeObj = NULL;
    USB_HOST_INTERFACE_DESC_INFO *interfaceInfo;
    USB_ENDPOINT_DESCRIPTOR * endpointDescriptor;
    USB_HOST_DEVICE_OBJ  * deviceObj;
    USB_HOST_ENDPOINT_DESCRIPTOR_QUERY endpointDescQuery;
    USB_HOST_INTERFACE_DESCRIPTOR_QUERY interfaceQuery;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;

    /* Get the index of the device from the interface handle. We need this to
     * get the device object. */
    deviceIndex =  USB_HOST_DEVICE_INDEX( deviceInterfaceHandle );

    /* Get the index of the interface. We need this to get information about
     * the endpoint */
    interfaceIndex =  USB_HOST_INTERFACE_INDEX ( deviceInterfaceHandle );

    /* Get the device object because we need the pointer to the HCD pipe open
     * function. */
    deviceObj = &gUSBHostDeviceList[deviceIndex];

    /* Get the interface information object to get the pointer to the interface
     * descriptor */
    interfaceInfo = &(deviceObj->configDescriptorInfo.interfaceInfo[interfaceIndex]);

    /* Set default value to return in case of error */
    result = USB_HOST_PIPE_HANDLE_INVALID;

    /* We must first find the interface descriptor with the current alternate
     * setting */

    USB_HOST_DeviceInterfaceQueryContextClear(&interfaceQuery);
    interfaceQuery.bInterfaceNumber = interfaceIndex;
    interfaceQuery.bAlternateSetting = interfaceInfo->currentAlternateSetting;
    interfaceQuery.flags = USB_HOST_INTERFACE_QUERY_BY_NUMBER|USB_HOST_INTERFACE_QUERY_ALT_SETTING;
    interfaceDescriptor = USB_HOST_DeviceGeneralInterfaceDescriptorQuery(interfaceInfo->interfaceDescriptor, &interfaceQuery);

    if(interfaceDescriptor != NULL)
    {

        /* We want to search for the endpoint descriptor so that we can find out the
         * pipe attributes. Set up the query object to search by endpoint number and
         * then search for the end point descriptor */

        USB_HOST_DeviceEndpointQueryContextClear(&endpointDescQuery);
        endpointDescQuery.endpointAddress = endpointAddress;
        endpointDescQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_ENDPOINT_ADDRESS;
        endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor,&endpointDescQuery);

        if(deviceObj->inUse) 
        {
            /* Device object is valid */
            if((USB_HOST_PNP_IDENTIFIER(deviceInterfaceHandle)) == (USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier)))
            {
                /* PlugNPlay identifiers are matching */

                if ( endpointDescriptor != NULL )
                {
                    /* Transfer type */
                    transferType = endpointDescriptor->transferType;

                    /* Interval for periodic transfers */
                    if ( transferType ==  USB_TRANSFER_TYPE_BULK )
                    {
                        /* Bulk endpoints we ignore interval value
                         * reported by endpoint descriptor */
                        interval = 0x00;
                    }
                    else
                    {
                        interval = endpointDescriptor->bInterval;
                    }

                    /* Endpoint maximum packet size */
                    maxPacketSize = endpointDescriptor->wMaxPacketSize;

                    /* The pipe open function should never be called in an event handler or
                     * in an interrupt context. We don't check for interrupt safety here,
                     * only thread safety. The safety is needed for protected the pipe pool
                     * */
                    if(OSAL_MUTEX_Lock(&(gUSBHostObj.mutexPipeObj), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
                    {
                        for ( pipeNumber = 0 ; pipeNumber < USB_HOST_PIPES_NUMBER ; pipeNumber++)
                        {
                            if( gUSBHostPipeObj[pipeNumber].inUse == false )
                            {
                                pipeObj= & gUSBHostPipeObj[pipeNumber];
                                result = (USB_HOST_PIPE_HANDLE)(pipeObj);
                                pipeObj->inUse = true;
                                break;
                            }
                        }

                        /* Release the mutual exclusion lock */
                        OSAL_MUTEX_Unlock(&(gUSBHostObj.mutexPipeObj));
                    }
                    else
                    {
                        /* If the mutual exclusion lock failed for unknown reason, we don't do anything
                         * here as we will any ways check for a valid handle in result.
                         * result has an invalid handle at this point */
                    }

                    if(result != USB_HOST_PIPE_HANDLE_INVALID)
                    {
                        /* This means the pipeObj is valid. We can try opening the pipe by
                         * using the HCD pipe open function */

                        pipeObj->pipeHandle = deviceObj->hcdInterface->hostPipeSetup( deviceObj->hcdHandle,
                                deviceObj->deviceAddress,   /* Address of the device */ 
                                endpointAddress,            /* Endpoint */
                                deviceObj->hubAddress,       /* Hub address and port */
                                deviceObj->devicePort,         
                                transferType,               /* Pipe type */ 
                                interval,                   /* bInterval */
                                maxPacketSize,              /* Pipe size */ 
                                deviceObj->speed            /* Speed of the pipe */
                                );     

                        if(pipeObj->pipeHandle == DRV_USB_HOST_PIPE_HANDLE_INVALID)
                        {
                            /* The HCD pipe open function failed. Release the pipe object
                             * back and then update result. */

                            pipeObj->inUse = false;
                            result = USB_HOST_PIPE_HANDLE_INVALID;
                        }
                        else
                        {
                            /* HCD pipe open function worked. Update the pipe object */
                            pipeObj->endpointAddress = endpointAddress;
                            pipeObj->interfaceHandle = deviceInterfaceHandle ;

                        }
                    }
                }
            }
        }
    }

    return result;
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DevicePipeClose 
    (
        USB_HOST_PIPE_HANDLE pipeHandle
    );

  Summary:
    Closes an existing pipe.

  Description:
    This function closes an existing pipe. Any transfers that are queued or
    in-progress on the pipe will be aborted and the transfer complete events
    will be generated. Once closed, no transfers can be queued on the pipe.
 
  Remarks:
     Refer to usb_host_client_driver.h for usage details.
*/

USB_HOST_RESULT USB_HOST_DevicePipeClose
(
    USB_HOST_PIPE_HANDLE pipeHandle
)
{
    USB_HOST_RESULT result;
    USB_HOST_PIPE_OBJ * pipeObj;
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle;
    USB_HOST_DEVICE_OBJ * deviceObj;
    int deviceIndex;

    if((pipeHandle == USB_HOST_PIPE_HANDLE_INVALID) || (pipeHandle == 0))
    {
        result = USB_HOST_RESULT_PIPE_HANDLE_INVALID;
    }
    else
    {
        /* The pipe handle is a pointer to the pipe object */
        pipeObj = (USB_HOST_PIPE_OBJ *)(pipeHandle);

        /* Get the handle to the interface to which this pipe belongs */
        interfaceHandle = pipeObj->interfaceHandle;

        /* From the interface handle get the device index and hence the device
         * object */
        deviceIndex = USB_HOST_DEVICE_INDEX(interfaceHandle);
        deviceObj = &gUSBHostDeviceList[deviceIndex];

        if((USB_HOST_PNP_IDENTIFIER(interfaceHandle)) == (USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier)))
        {
            /* Use the HCD routine to close the pipe and deallocate the pipe
             * object */

            deviceObj->hcdInterface->hostPipeClose(pipeObj->pipeHandle);
            pipeObj->inUse = false;
            pipeObj->pipeHandle = DRV_USB_HOST_PIPE_HANDLE_INVALID;
            result = USB_HOST_RESULT_SUCCESS;
        }
        else
        {
            result = USB_HOST_RESULT_FAILURE;
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DevicePipeHaltClear
    (
        USB_HOST_PIPE_HANDLE pipeHandle,
        USB_HOST_REQUEST_HANDLE * requestHandle,
        uintptr_t context
    );

  Summary:
    Clears the halt condition on a pipe.

  Description:
    This function clears the halt condition on the specified pipe. This
    function will cause a CLEAR FEATURE control request to be sent to the
    device. The completion of the control transfer will be indicated by the
    USB_HOST_DEVICE_EVENT_DEVICE_HALT_CLEAR_COMPLETE event.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
*/

USB_HOST_RESULT USB_HOST_DevicePipeHaltClear
(
    USB_HOST_PIPE_HANDLE pipeHandle,
    USB_HOST_REQUEST_HANDLE * requestHandle,
    uintptr_t context
)
{
    USB_HOST_RESULT result = USB_HOST_RESULT_FAILURE;
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle;
    USB_HOST_DEVICE_OBJ * deviceObj;
    USB_HOST_CONTROL_TRANSFER_OBJ * controlTransferObj; 
    USB_HOST_PIPE_OBJ * pipeObj;
    int deviceIndex;


    /* Check if we have a valid pipe */
    if((USB_HOST_PIPE_HANDLE_INVALID == pipeHandle) || (0 == pipeHandle))
    {
        result = USB_HOST_RESULT_PIPE_HANDLE_INVALID;
    }
    else
    {
        /* Request Handle pointer cannot be NULL */
        if(requestHandle == NULL)
        {
            result = USB_HOST_RESULT_PARAMETER_INVALID;
        }
        else
        {
            /* Get the index of the device to which this pipe belongs. This can
             * be obtained from the interface handle. The interface handle is
             * stored in the pipe object */

            pipeObj = (USB_HOST_PIPE_OBJ *)(pipeHandle);
            interfaceHandle = pipeObj->interfaceHandle;
            deviceIndex = USB_HOST_DEVICE_INDEX(interfaceHandle);
            deviceObj = &gUSBHostDeviceList[deviceIndex];

            /* The interface handle  PNP identifier does not match the device PNP
             * identifier. The device is not the same */
            if(USB_HOST_PNP_IDENTIFIER(interfaceHandle) != 
                    USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier))
            {
                result = USB_HOST_RESULT_FAILURE; 
            }
            else
            {
                /* Set the request handle default to invalid */
                *requestHandle = USB_HOST_REQUEST_HANDLE_INVALID;

                /* Get a mutual exclusion lock as the the control transfer object is a global resource */
                if(OSAL_MUTEX_Lock(&(gUSBHostObj.mutexControlTransferObj), OSAL_WAIT_FOREVER) 
                        == OSAL_RESULT_TRUE)
                {
                    if(!deviceObj->controlTransferObj.inUse)
                    {
                        /* This means that there no control request in progress. We can assign
                         * request now. The request handle is updated to point to the device
                         * control transfer object. */

                        deviceObj->controlTransferObj.inUse = true;
                        *requestHandle = (USB_HOST_REQUEST_HANDLE)(&deviceObj->controlTransferObj);
                    }
                    else
                    {
                        /* A control transfer is in progress. */
                        result = USB_HOST_RESULT_REQUEST_BUSY;
                    }

                    /* Unlock the mutual exclusion lock */
                    OSAL_MUTEX_Unlock(&(gUSBHostObj.mutexControlTransferObj));
                }
                else
                {
                    /* The mutual exclusion lock could not be obtained */
                    result = USB_HOST_RESULT_REQUEST_BUSY;
                }

                if(*requestHandle != USB_HOST_REQUEST_HANDLE_INVALID)
                {
                    /* Set up the control transfer object. The endpoint halt
                     * clear request does not have a data stage. */
                    controlTransferObj = &deviceObj->controlTransferObj;
                    controlTransferObj->requestType = USB_HOST_CONTROL_REQUEST_TYPE_PIPE_HALT_CLEAR;

                    /* Create the setup packet */
                    _USB_HOST_FillSetupPacket(
                            &(deviceObj->setupPacket),
                            ( USB_SETUP_DIRN_HOST_TO_DEVICE |
                              USB_SETUP_TYPE_STANDARD |
                              USB_SETUP_RECIPIENT_ENDPOINT ),
                            USB_REQUEST_CLEAR_FEATURE ,
                            USB_FEATURE_SELECTOR_ENDPOINT_HALT , pipeObj->endpointAddress  ,0 ) ;

                    controlTransferObj->requestType = USB_HOST_CONTROL_REQUEST_TYPE_PIPE_HALT_CLEAR;
                    controlTransferObj->controlIRP.data = NULL;
                    controlTransferObj->controlIRP.setup = &deviceObj->setupPacket;
                    controlTransferObj->controlIRP.size = deviceObj->setupPacket.wLength;
                    controlTransferObj->controlIRP.callback = _USB_HOST_DeviceControlTransferCallback;
                    controlTransferObj->controlIRP.userData = interfaceHandle;
                    controlTransferObj->context = context;
                    controlTransferObj->callback = NULL;

                    if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, &(controlTransferObj->controlIRP)))
                    {
                        /* There was a problem while submitting the IRP. Update the result and
                         * the transfer handle. Return the control transfer object back to the
                         * device object */

                        result = USB_HOST_RESULT_FAILURE;
                        controlTransferObj->inUse = false;
                        *requestHandle = USB_HOST_REQUEST_HANDLE_INVALID;
                    }
                    else
                    {
                        result = USB_HOST_RESULT_SUCCESS;
                    }
                }
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceRelease
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    );

  Summary:
    Releases an the device ownership.

  Description:
    This function allows the client driver to release device level ownership.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
 */

USB_HOST_RESULT USB_HOST_DeviceRelease
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle
)
{
    int deviceIndex;
    unsigned int pnpIdentifier;
    USB_HOST_RESULT result;
    USB_HOST_DEVICE_OBJ * deviceObj;
    
    deviceIndex = USB_HOST_DEVICE_INDEX(deviceHandle);
    pnpIdentifier = USB_HOST_PNP_IDENTIFIER(deviceHandle);
    
    /* Get the device object */
    deviceObj = &gUSBHostDeviceList[deviceIndex];
    
    if(!deviceObj->inUse)
    {
        /* This device object is not in use. May have been disconnected */
        result = USB_HOST_RESULT_DEVICE_UNKNOWN;
    }
    else 
    {
        if(pnpIdentifier != USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier))
        {
            /* This device is not the same that client driver thinks it is */
            result = USB_HOST_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            if(deviceObj->deviceClientDriver == NULL)
            {
                /* This is should not happen */
                result = USB_HOST_RESULT_DEVICE_UNKNOWN;
            }
            else
            {
                /* Release the interface */
                deviceObj->deviceClientDriver = NULL;
                result = USB_HOST_RESULT_SUCCESS;
            }
        }
    }    
        
    return(result);
}

// *****************************************************************************
/* Function:
    USB_INTERFACE_ASSOCIATION_DESCRIPTOR * USB_HOST_DeviceIADQuery
    (
        USB_CONFIGURATION_DESCRIPTOR * configuration
        USB_HOST_IAD_QUERY * query,
    );

  Summary:
    Queries the configuration for the specified IAD.

  Description:
    This function queries the configuration for the specified IAD and returns a
    pointer to the interface association descriptor if found. The return pointer
    will point to the standard interface association descriptor.  The search
    criteria can specified by using the flags. 
    
  Remarks:
    This function is optional and may not be available on all implementations of
    the USB Host Layer.
*/

USB_INTERFACE_ASSOCIATION_DESCRIPTOR * USB_HOST_DeviceIADQuery
(
    USB_CONFIGURATION_DESCRIPTOR * configuration,
    USB_HOST_IAD_QUERY * query
)
{
    USB_INTERFACE_ASSOCIATION_DESCRIPTOR * result = NULL;
    USB_DESCRIPTOR_HEADER * descriptorHeader;
    uint8_t * search;
    uint8_t * lastLocation;

    if(configuration == NULL)
    {
        /* Cannot proceed. Note that result is already NULL */
    }
    else
    {
        /* Keep track of the last location of this descriptor */
        search = (uint8_t *)(configuration);
        lastLocation = (uint8_t *)(_USB_HOST_FindEndOfDescriptor(search));

        if((query->context != 0) && (query->context < (uintptr_t)(lastLocation)))
        {
            /* This is a continuing search. We start the search from the last
             * saved location */
            search = (uint8_t *)(query->context);
        }

        while(search < lastLocation)
        {
            descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
            if(descriptorHeader->descType == USB_DESCRIPTOR_INTERFACE_ASSOCIATION)
            {
                /* This means we found an IAD. Update result and stop searching */
                result = (USB_INTERFACE_ASSOCIATION_DESCRIPTOR *)(search);
                
                /* Setup the context to point to the next descriptor else
                 * we will always loop at this descriptor the next time the 
                 * search function is called. */
                query->context = (uintptr_t)(search + descriptorHeader->size);
                break;
            }
            else
            {
                /* Go to the next descriptor */
                search += descriptorHeader->size;
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    void USB_HOST_DeviceIADQueryContextClear
    (
        USB_HOST_IAD_QUERY * query
    );

  Summary:
    Clear the query object.

  Description:
    This function clears the query object. Using the query after it has been
    clear will cause the USB_HOST_DeviceIADQuery() and function to reset the
    search location to the start of the configuration descriptor.

  Remarks:
    This function is optional and may not be available on all implementations of
    the USB Host Layer.
*/

void USB_HOST_DeviceIADQueryContextClear
(
    USB_HOST_IAD_QUERY * query
)
{
    if(query != NULL)
    {
        query->context = 0;
    }
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceConfigurationSet 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
        USB_HOST_REQUEST_HANDLE * requestHandle,
        uint8_t configurationIndex,
        uintptr_t context
    );

  Summary:
    Sets the active configuration for the device.

  Description:
    This function sets the configuration that the host layer must set for this
    device. A handle to the request is returned in requestHandle. The completion
    of this request is indicated by the USB_HOST_DEVICE_EVENT_CONFIGURATION_SET
    complete event.

  Remarks:
    Refer to usb_host_client_driver.h for usage details.
*/

USB_HOST_RESULT USB_HOST_DeviceConfigurationSet 
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    USB_HOST_REQUEST_HANDLE * requestHandle,
    uint8_t configurationIndex,
    uintptr_t context
)
{
    USB_HOST_RESULT result;
    USB_HOST_DEVICE_OBJ * deviceObj;
    unsigned int deviceIndex;
    unsigned int pnpIdentifier;

    if(deviceHandle == USB_HOST_DEVICE_CLIENT_HANDLE_INVALID)
    {
        /* Device client handle is not valid */
        result = USB_HOST_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* We have a valid client handle. Get the device index from the client
         * handle */

        deviceIndex = USB_HOST_DEVICE_INDEX(deviceHandle);
        deviceObj = &gUSBHostDeviceList[deviceIndex];
        pnpIdentifier = USB_HOST_PNP_IDENTIFIER(deviceHandle);

        if((!deviceObj->inUse) || (pnpIdentifier != USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier)))
        {
            /* This device is not valid */
            result = USB_HOST_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            /* Check if this is valid configuration. */
            if(configurationIndex >= deviceObj->nConfiguration)
            {
                /* The device does not support this configuration */
                result = USB_HOST_RESULT_CONFIGURATION_UNKNOWN;
            }
            else
            {
                if((!deviceObj->controlTransferObj.inUse) &&
                        (deviceObj->configurationState == USB_HOST_DEVICE_CONFIG_STATE_READY_FOR_CONFIG) &&
                        (deviceObj->deviceState == USB_HOST_DEVICE_STATE_READY))
                {
                    /* The configuration can be set */
                    deviceObj->controlTransferObj.inUse = true;
                    deviceObj->controlTransferObj.context = context;
                    deviceObj->requestedConfigurationNumber = configurationIndex;
                    deviceObj->configurationState = USB_HOST_DEVICE_CONFIG_STATE_START;
                    result = USB_HOST_RESULT_SUCCESS;
                }
                else
                {
                    /* The device is not ready for any requests at this point */
                    result = USB_HOST_RESULT_REQUEST_BUSY;
                }
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    USB_INTERFACE_DESCRIPTOR * USB_HOST_DeviceGeneralInterfaceDescriptorQuery
    (
        void * descriptor
        USB_HOST_INTERFACE_DESCRIPTOR_QUERY * query,
    );

  Summary:
    Queries the IAD group for the specified query.

  Description:
    This function will query will search for an interface starting from the
    location pointed to by the descriptor parameter. This descriptor parameter
    could be a pointer to an IAD or a interface descriptor.  The return pointer
    will point to the standard interface descriptor and class specific interface
    descriptors for that interface. The search criteria can specified by using
    the flags. 
    
    In a case where the interface has more than one alternate settings, the
    function can be called repetitively to continue the search till the end of the
    configuration descriptor is reached or till the search fails. The query flag
    in such should be set to ignore the alternate setting field.  The query
    object maintains the last point where the search was successful and
    continues the search from that point onwards. Resetting the query object
    (through the USB_HOST_DeviceInterfaceQueryContextClear()) function will
    reset the search object and cause the search to start from the top.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
*/

USB_INTERFACE_DESCRIPTOR * USB_HOST_DeviceGeneralInterfaceDescriptorQuery
(
    void * descriptor,
    USB_HOST_INTERFACE_DESCRIPTOR_QUERY * query
)
{
    USB_INTERFACE_DESCRIPTOR * result = NULL;
    USB_DESCRIPTOR_HEADER * descriptorHeader;
    uint8_t * search;
    uint8_t * lastLocation;
    USB_HOST_INTERFACE_QUERY_FLAG matchedCriteria = 0;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;

    if(descriptor != NULL)
    {
        /* We have a non null descritpor. Find where this descriptor ends */
        search = (uint8_t *)(descriptor);
        lastLocation = (uint8_t *)(_USB_HOST_FindEndOfDescriptor(search));

        if((query->context != 0) && (query->context < (uintptr_t)(lastLocation)))
        {
            /* This is a continuing search. We start the search from the last
             * saved location */
            search = (uint8_t *)(query->context);
        }

        while(search < lastLocation)
        {
            /* Reset the matching criteria as this is a new search */
            matchedCriteria = 0;
            descriptorHeader = (USB_DESCRIPTOR_HEADER *)(search);
            
            if(descriptorHeader->descType == USB_DESCRIPTOR_INTERFACE)
            {
                interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)(search);

                /* This means we found a interface descriptor. We need to check
                 * if it meets the criteria */

                if(query->flags == USB_HOST_INTERFACE_QUERY_ANY)
                {
                    /* This means any interface descriptor is fine. We should
                     * stop searching. Save the query location in the context */

                    query->context = (uintptr_t)(search + descriptorHeader->size);
                    result = (USB_INTERFACE_DESCRIPTOR *)(search);
                    break;
                }
                else
                {
                    /* Need to apply the specified criteria */

                    if(query->flags & USB_HOST_INTERFACE_QUERY_BY_NUMBER)
                    {
                        if(interfaceDescriptor->bInterfaceNumber == query->bInterfaceNumber)
                        {
                            /* Matches by number */
                            matchedCriteria |= USB_HOST_INTERFACE_QUERY_BY_NUMBER;
                        }
                    }

                    if(query->flags & USB_HOST_INTERFACE_QUERY_ALT_SETTING)
                    {
                        if(interfaceDescriptor->bAlternateSetting == query->bAlternateSetting)
                        {
                            /* Matches by alternate setting */
                            matchedCriteria |= USB_HOST_INTERFACE_QUERY_ALT_SETTING;
                        }
                    }

                    if(query->flags & USB_HOST_INTERFACE_QUERY_BY_CLASS)
                    {
                        if(interfaceDescriptor->bInterfaceClass == query->bInterfaceClass)
                        {
                            /* Matches by interface class */
                            matchedCriteria |= USB_HOST_INTERFACE_QUERY_BY_CLASS;
                        }
                    }

                    if(query->flags & USB_HOST_INTERFACE_QUERY_BY_SUBCLASS)
                    {
                        if(interfaceDescriptor->bInterfaceSubClass == query->bInterfaceSubClass)
                        {
                            /* Matches by interface subclass */
                            matchedCriteria |= USB_HOST_INTERFACE_QUERY_BY_SUBCLASS;
                        }
                    }

                    if(query->flags & USB_HOST_INTERFACE_QUERY_BY_PROTOCOL)
                    {
                        if(interfaceDescriptor->bInterfaceProtocol == query->bInterfaceProtocol)
                        {
                            /* Matches by interface protocol */
                            matchedCriteria |= USB_HOST_INTERFACE_QUERY_BY_PROTOCOL;
                        }
                    }

                    /* Now check if we have met all the criteria */

                    if(matchedCriteria == query->flags)
                    {
                        /* Yes we have. Save the search location and exit */
                        query->context = (uintptr_t)(search + descriptorHeader->size);
                        result = (USB_INTERFACE_DESCRIPTOR *)(search);
                        break;
                    }
                    else
                    {
                        /* We did not match the criteria */
                        search += descriptorHeader->size;
                    }
                }
            }
            else
            {
                /* Go to the next descriptor */
                search += descriptorHeader->size;
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceInterfaceSet
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
        USB_HOST_REQUEST_HANDLE * requestHandle,
        uint8_t alternateSetting,
        uintptr_t context
    );

  Summary:
    Activates an alternate setting for the specified interface.

  Description:
    This function activates an alternate setting for the specified interface.
    This will cause the host layer to send a SET INTERFACE request to the
    device.  The specified interface should have been claimed. The completion of
    the SET INTERFACE function will be indicated by the
    USB_HOST_DEVICE_EVENT_SET_INTERFACE_COMPLETE event.

  Remarks:
    Refer to usb_host_client_driver.h for usage information.
*/

USB_HOST_RESULT USB_HOST_DeviceInterfaceSet
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_REQUEST_HANDLE * requestHandle,
    uint8_t alternateSetting,
    uintptr_t context
)
{
    USB_HOST_RESULT result = USB_HOST_RESULT_FAILURE;
    USB_HOST_DEVICE_OBJ * deviceObj;
    USB_HOST_CONTROL_TRANSFER_OBJ * controlTransferObj; 
    int deviceIndex, interfaceIndex, pnpIdentifier;
    USB_HOST_INTERFACE_DESC_INFO * interfaceDescInfo;
    USB_HOST_INTERFACE_DESCRIPTOR_QUERY interfaceQueryObject;

    /* Get the device index and then the pointer to the device object */

    deviceIndex = USB_HOST_DEVICE_INDEX(interfaceHandle);
    deviceObj = &gUSBHostDeviceList[deviceIndex];

    if(!deviceObj->inUse)
    {
        /* Device object is not valid anymore */
        result = USB_HOST_RESULT_DEVICE_UNKNOWN;
    }
    else
    {
        /* Check the Plug N Play identifier */
        pnpIdentifier = USB_HOST_PNP_IDENTIFIER(interfaceHandle);
        if(pnpIdentifier != USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier))
        {
            /* This is not the same device the client driver thinks it is */
            result = USB_HOST_RESULT_DEVICE_UNKNOWN;

        }
        else
        {
            interfaceIndex = USB_HOST_INTERFACE_INDEX(interfaceHandle);
            interfaceDescInfo = &deviceObj->configDescriptorInfo.interfaceInfo[interfaceIndex];
            if(interfaceDescInfo->interfaceDescriptor == NULL)
            {
                /* This means this interface is not valid */
                result = USB_HOST_RESULT_INTERFACE_UNKNOWN;
            }
            else
            {
                /* We must search for this interface */
                USB_HOST_DeviceInterfaceQueryContextClear(&interfaceQueryObject);
                interfaceQueryObject.bInterfaceNumber = interfaceIndex;
                interfaceQueryObject.bAlternateSetting = alternateSetting;
                interfaceQueryObject.flags = USB_HOST_INTERFACE_QUERY_ALT_SETTING|USB_HOST_INTERFACE_QUERY_BY_NUMBER;
                if(USB_HOST_DeviceGeneralInterfaceDescriptorQuery(interfaceDescInfo->interfaceDescriptor,
                            &interfaceQueryObject) == NULL)
                {
                    /* This alternate setting does not exist */
                    result = USB_HOST_RESULT_INTERFACE_UNKNOWN;
                }
                else
                {
                    /* Set the request handle default to invalid */
                    *requestHandle = USB_HOST_REQUEST_HANDLE_INVALID;

                    /* Get a mutual exclusion lock as the the control transfer object is a global resource */
                    if(OSAL_MUTEX_Lock(&(gUSBHostObj.mutexControlTransferObj), OSAL_WAIT_FOREVER) 
                            == OSAL_RESULT_TRUE)
                    {
                        if(!deviceObj->controlTransferObj.inUse)
                        {
                            /* This means that there no control request in progress. We can assign
                             * request now. The request handle is updated to point to the device
                             * control transfer object. */

                            deviceObj->controlTransferObj.inUse = true;
                            *requestHandle = (USB_HOST_REQUEST_HANDLE)(&deviceObj->controlTransferObj);
                        }
                        else
                        {
                            /* A control transfer is in progress. */
                            result = USB_HOST_RESULT_REQUEST_BUSY;
                        }

                        /* Unlock the mutual exclusion lock */
                        OSAL_MUTEX_Unlock(&(gUSBHostObj.mutexControlTransferObj));
                    }
                    else
                    {
                        /* The mutual exclusion lock could not be obtained */
                        result = USB_HOST_RESULT_REQUEST_BUSY;
                    }

                    if(*requestHandle != USB_HOST_REQUEST_HANDLE_INVALID)
                    {
                        /* Set up the control transfer object. The endpoint halt
                         * clear request does not have a data stage. */
                        controlTransferObj = &deviceObj->controlTransferObj;
                        controlTransferObj->requestType = USB_HOST_CONTROL_REQUEST_TYPE_INTERFACE_SET;
                        
                        /* Remember which alternate setting was requested. This
                         * will be needed after we get the event */
                        deviceObj->requestedAlternateSetting = alternateSetting;

                        _USB_HOST_FillSetupPacket(
                                &(deviceObj->setupPacket),
                                ( USB_SETUP_DIRN_HOST_TO_DEVICE |
                                  USB_SETUP_TYPE_STANDARD |
                                  USB_SETUP_RECIPIENT_INTERFACE ),
                                USB_REQUEST_SET_INTERFACE,
                                alternateSetting , interfaceIndex  ,0 ) ;

                        controlTransferObj->controlIRP.data = NULL;
                        controlTransferObj->controlIRP.setup = &deviceObj->setupPacket;
                        controlTransferObj->controlIRP.size = deviceObj->setupPacket.wLength;
                        controlTransferObj->controlIRP.callback = _USB_HOST_DeviceControlTransferCallback;
                        controlTransferObj->controlIRP.userData = interfaceHandle;
                        controlTransferObj->context = context;
                        controlTransferObj->callback = NULL;

                        if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, &(controlTransferObj->controlIRP)))
                        {
                            /* There was a problem while submitting the IRP. Update the result and
                             * the transfer handle. Return the control transfer object back to the
                             * device object */

                            result = USB_HOST_RESULT_FAILURE;
                            controlTransferObj->inUse = false;
                            *requestHandle = USB_HOST_REQUEST_HANDLE_INVALID;
                        }
                        else
                        {
                            result = USB_HOST_RESULT_SUCCESS;
                        }
                    }
                }
            }
        }
    }

    return(result);
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_DeviceStringDescriptorGet
    (
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        USB_HOST_DEVICE_STRING stringType,
        uint16_t languageID,
        void * stringDescriptor,
        size_t length,
        USB_HOST_REQUEST_HANDLE * requestHandle,
        USB_HOST_STRING_REQUEST_COMPLETE_CALLBACK callback,
        uintptr_t context
    );

  Summary:
    Retrieves specified string descriptor from the device

  Description:
    This function retrieves the specified string descriptor from the device.
    This function will cause the host layer to issue a control transfer to the
    device. When the string descriptor is available, the host layer will call
    the callback function to let the application know that the request has
    completed. 
    
    The function will return a valid request handle in requestHandle, if the
    request was successful. This request handle will be returned in the callback
    function. The size of the stringDescriptor buffer is specified by the length
    parameter.  Only length number of bytes will be retrieved. The type of
    device string descriptor to be retrieved is specified by the stringType
    parameter. The supported language IDs, manufacturer, product and serial
    number strings can be obtained. While obtaining the supported language IDs,
    the languageID parameter will be ignored.

  Remarks:
    None.
*/

USB_HOST_RESULT USB_HOST_DeviceStringDescriptorGet
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_DEVICE_STRING stringType,
    uint16_t languageID,
    void * stringDescriptor,
    size_t length,
    USB_HOST_REQUEST_HANDLE * requestHandle,
    USB_HOST_STRING_REQUEST_COMPLETE_CALLBACK callback,
    uintptr_t context
)
{
    int deviceIndex, pnpIdentifier;
    USB_HOST_DEVICE_OBJ * deviceObj;
    USB_HOST_RESULT result = USB_HOST_RESULT_FAILURE;
    USB_HOST_CONTROL_TRANSFER_OBJ * controlTransferObj;
    uint8_t stringIndex;

    /* Get the device index and then the pointer to the device object */

    deviceIndex = USB_HOST_DEVICE_INDEX(deviceObjHandle);
    deviceObj = &gUSBHostDeviceList[deviceIndex];

    if(!deviceObj->inUse)
    {
        /* Device object is not valid anymore */
        result = USB_HOST_RESULT_DEVICE_UNKNOWN;
    }
    else
    {
        /* Check the Plug N Play identifier */
        pnpIdentifier = USB_HOST_PNP_IDENTIFIER(deviceObjHandle);
        if(pnpIdentifier != USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier))
        {
            /* This is not the same device as the application thinks it is */
            result = USB_HOST_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            if((stringDescriptor == NULL) || (length == 0) || (requestHandle == NULL) || (callback == NULL))
            {
                /* One of the required parameter is not valid */
                result = USB_HOST_RESULT_PARAMETER_INVALID;
            }
            else
            {
                /* Need to check if the device is in a failure state */
                if(deviceObj->deviceState == USB_HOST_DEVICE_STATE_ERROR)
                {
                    result = USB_HOST_RESULT_FAILURE;
                }
                else
                {
                    if(deviceObj->deviceState != USB_HOST_DEVICE_STATE_READY)
                    {
                        /* The device must be in a ready state */
                        result = USB_HOST_RESULT_REQUEST_BUSY;
                    }
                    else
                    {
                        /* Map the string type to string index. Set the default
                         * string index to 0. */
                        
                        stringIndex = 0;
                        switch(stringType)
                        {
                            case USB_HOST_DEVICE_STRING_LANG_ID:

                                /* Setting string index to zero will get the
                                 * the language ID */
                                stringIndex = 0;
                                break;

                            case USB_HOST_DEVICE_STRING_MANUFACTURER:

                                /* Set the string index to the manufacture string
                                   index */
                                stringIndex = deviceObj->deviceDescriptor.iManufacturer;
                                break;

                            case USB_HOST_DEVICE_STRING_PRODUCT:

                                /* Set the string index to the manufacture string
                                   index */
                                stringIndex = deviceObj->deviceDescriptor.iProduct;
                                break;

                            case USB_HOST_DEVICE_STRING_SERIAL_NUMBER:

                                /* Set the string index to the manufacture string
                                   index */
                                stringIndex = deviceObj->deviceDescriptor.iSerialNumber;
                                break;

                            default:
                                break;
                        }

                        if((stringIndex == 0) && (stringType != USB_HOST_DEVICE_STRING_LANG_ID))
                        {
                            /* This means that the device does not support the
                             * requested string. */

                            result = USB_HOST_RESULT_STRING_DESCRIPTOR_UNSUPPORTED;
                        }
                        else
                        {
                            /* Set the request handle default to invalid */
                            *requestHandle = USB_HOST_REQUEST_HANDLE_INVALID;

                            /* Get a mutual exclusion lock as the control transfer object is a global
                             * resource */

                            if(OSAL_MUTEX_Lock(&(gUSBHostObj.mutexControlTransferObj), OSAL_WAIT_FOREVER) 
                                    == OSAL_RESULT_TRUE)
                            {
                                if(!deviceObj->controlTransferObj.inUse)
                                {
                                    /* This means that there no control request in progress. We can assign
                                     * request now. The request handle is updated to point to the device
                                     * control transfer object. */

                                    deviceObj->controlTransferObj.inUse = true;
                                    *requestHandle = (USB_HOST_REQUEST_HANDLE)(&deviceObj->controlTransferObj);
                                }
                                else
                                {
                                    /* A control transfer is in progress. */
                                    result = USB_HOST_RESULT_REQUEST_BUSY;
                                }

                                /* Unlock the mutual exclusion lock */
                                OSAL_MUTEX_Unlock(&(gUSBHostObj.mutexControlTransferObj));
                            }
                            else
                            {
                                /* The mutual exclusion lock could not be obtained */
                                result = USB_HOST_RESULT_REQUEST_BUSY;
                            }

                            if(*requestHandle != USB_HOST_REQUEST_HANDLE_INVALID)
                            {
                                /* Set up the control transfer object. The request type
                                 * allows the one control transfer handler to identify the
                                 * type of the request. In this case this is a string
                                 * descriptor request.  */
                                controlTransferObj = &deviceObj->controlTransferObj;
                                controlTransferObj->requestType = USB_HOST_CONTROL_REQUEST_TYPE_STRING_DESCRIPTOR;

                                _USB_HOST_FillSetupPacket(
                                        &(deviceObj->setupPacket),
                                        ( USB_SETUP_DIRN_DEVICE_TO_HOST |
                                          USB_SETUP_TYPE_STANDARD |
                                          USB_SETUP_RECIPIENT_DEVICE ),
                                        USB_REQUEST_GET_DESCRIPTOR,
                                        ((USB_DESCRIPTOR_STRING << 8)|stringIndex), languageID, length ) ;

                                /* The userData filed in the IRP is set to the device object
                                 * handle. This will allow the control transfer callback to
                                 * identify the device which submitted the control
                                 * transfer. */

                                controlTransferObj->controlIRP.data = stringDescriptor;
                                controlTransferObj->controlIRP.setup = &deviceObj->setupPacket;
                                controlTransferObj->controlIRP.size = deviceObj->setupPacket.wLength;
                                controlTransferObj->controlIRP.callback = _USB_HOST_DeviceControlTransferCallback;
                                controlTransferObj->controlIRP.userData = deviceObjHandle ;
                                controlTransferObj->context = context;
                                controlTransferObj->callback = callback;

                                if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, &(controlTransferObj->controlIRP)))
                                {
                                    /* There was a problem while submitting the IRP. Update the result and
                                     * the transfer handle. Return the control transfer object back to the
                                     * device object */

                                    result = USB_HOST_RESULT_FAILURE;
                                    controlTransferObj->inUse = false;
                                    *requestHandle = USB_HOST_REQUEST_HANDLE_INVALID;
                                }
                                else
                                {
                                    result = USB_HOST_RESULT_SUCCESS;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    return(result);
}

// *****************************************************************************
/* Function:
    void USB_HOST_OverCurrentDetected
    (
        USB_HOST_DEVICE_OBJ_HANDLE parentDeviceObjHandle,
        uint8_t port,
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle
    );

  Summary:
    This function provides indication to the host layer that an over-current
    event has occurred.
    
  Description:
    This function provides indication to the host layer that an over-current
    event has occurred. The host layer will in turn forward the event to the
    application. This function is called exclusively by the root hub or the
    external hub driver. The root hub or the external driver will de-enumerate
    this device after the function returns.

  Remarks:
    This function is optional and may not be available on all implementations of
    the USB Host Layer. 
*/

void USB_HOST_OverCurrentDetected
(
    USB_HOST_DEVICE_OBJ_HANDLE parentDeviceObjHandle,
    uint8_t port,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle
)
{
    /* This function is called when an overcurrent condition has occurred. The
     * root hub or the external hub driver would de-enumerate the device after
     * this function exits. So the only thing we should do is to call the
     * application event handler and let the application know that the over
     * current event has occurred. */

    if(gUSBHostObj.hostEventHandler != NULL)
    {
        /* In this version of the host layer, we do not send any event data with
         * this event */

        gUSBHostObj.hostEventHandler(USB_HOST_EVENT_PORT_OVERCURRENT_DETECTED, NULL, gUSBHostObj.context);
    }
}

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_ConfigurationDescriptorGet
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
        USB_HOST_REQUEST_HANDLE * requestHandle
        uint8_t configurationIndex,
        void * buffer,
        size_t size,
        uintptr_t context
    );

  Summary:
    Requests for a configuration descriptor.

  Description:
    This function places a USB Host request to obtain a device configuration
    descriptor. The function is non blocking. A pointer to the configuration
    descriptor data will be available in event data when the
    USB_HOST_DEVICE_EVENT_CONFIGURATION_DESCRIPTOR_GET_COMPLETE event occurs.
    The size of the configuration descriptor will be available in the event
    data.

  Remarks:
    None.
*/

USB_HOST_RESULT USB_HOST_DeviceConfigurationDescriptorGet
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    USB_HOST_REQUEST_HANDLE * requestHandle,
    uint8_t configurationValue,
    void * buffer,
    size_t size,
    uintptr_t context
)
{
    int deviceIndex, pnpIdentifier;
    USB_HOST_DEVICE_OBJ * deviceObj;
    USB_HOST_RESULT result = USB_HOST_RESULT_FAILURE;
    USB_HOST_CONTROL_TRANSFER_OBJ * controlTransferObj;

    /* Get the device index and then the pointer to the device object */

    deviceIndex = USB_HOST_DEVICE_INDEX(deviceHandle);
    deviceObj = &gUSBHostDeviceList[deviceIndex];

    if(!deviceObj->inUse)
    {
        /* Device object is not valid anymore */
        result = USB_HOST_RESULT_DEVICE_UNKNOWN;
    }
    else
    {
        /* Check the Plug N Play identifier */
        pnpIdentifier = USB_HOST_PNP_IDENTIFIER(deviceHandle);
        if(pnpIdentifier != USB_HOST_PNP_IDENTIFIER(deviceObj->deviceIdentifier))
        {
            /* This is not the same device as the application thinks it is */
            result = USB_HOST_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            if((size == 0) || (requestHandle == NULL) || (buffer == NULL))
            {
                /* One of the required parameter is not valid */
                result = USB_HOST_RESULT_PARAMETER_INVALID;
            }
            else
            {
                /* Need to check if the device is in a failure state */
                if(deviceObj->deviceState == USB_HOST_DEVICE_STATE_ERROR)
                {
                    result = USB_HOST_RESULT_FAILURE;
                }
                else
                {
                    if(deviceObj->deviceState != USB_HOST_DEVICE_STATE_READY)
                    {
                        /* The device must be in a ready state */
                        result = USB_HOST_RESULT_REQUEST_BUSY;
                    }
                    else
                    {     
                        /* Set the request handle default to invalid */
                        *requestHandle = USB_HOST_REQUEST_HANDLE_INVALID;

                        /* Get a mutual exclusion lock as the control transfer object is a global
                         * resource */

                        if(OSAL_MUTEX_Lock(&(gUSBHostObj.mutexControlTransferObj), OSAL_WAIT_FOREVER) 
                                == OSAL_RESULT_TRUE)
                        {
                            if(!deviceObj->controlTransferObj.inUse)
                            {
                                /* This means that there no control request in progress. We can assign
                                 * request now. The request handle is updated to point to the device
                                 * control transfer object. */

                                deviceObj->controlTransferObj.inUse = true;
                                *requestHandle = (USB_HOST_REQUEST_HANDLE)(&deviceObj->controlTransferObj);
                            }
                            else
                            {
                                /* A control transfer is in progress. */
                                result = USB_HOST_RESULT_REQUEST_BUSY;
                            }

                            /* Unlock the mutual exclusion lock */
                            OSAL_MUTEX_Unlock(&(gUSBHostObj.mutexControlTransferObj));
                        }
                        else
                        {
                            /* The mutual exclusion lock could not be obtained */
                            result = USB_HOST_RESULT_REQUEST_BUSY;
                        }

                        if(*requestHandle != USB_HOST_REQUEST_HANDLE_INVALID)
                        {

                            /* Create the Setup packet */
                            _USB_HOST_FillSetupPacket(
                                    &(deviceObj->setupPacket),
                                    ( USB_SETUP_DIRN_DEVICE_TO_HOST |
                                      USB_SETUP_TYPE_STANDARD |
                                      USB_SETUP_RECIPIENT_DEVICE ),
                                    USB_REQUEST_GET_DESCRIPTOR,
                                    ( USB_DESCRIPTOR_CONFIGURATION << 8 ) + configurationValue , 0 , size ) ;

                            /* Set up the control transfer object. The endpoint halt
                             * clear request does not have a data stage. */
                            controlTransferObj = &deviceObj->controlTransferObj;
                            controlTransferObj->requestType = USB_HOST_CONTROL_REQUEST_TYPE_CONFIGURATION_DESCRIPTOR_GET;

                            controlTransferObj->controlIRP.data = buffer;
                            controlTransferObj->controlIRP.setup = &deviceObj->setupPacket;
                            controlTransferObj->controlIRP.size = deviceObj->setupPacket.wLength;
                            controlTransferObj->controlIRP.callback = _USB_HOST_DeviceControlTransferCallback;
                            controlTransferObj->controlIRP.userData = deviceHandle;
                            controlTransferObj->context = context;
                            controlTransferObj->callback = NULL;

                            if(USB_ERROR_NONE != deviceObj->hcdInterface->hostIRPSubmit( deviceObj->controlPipeHandle, &(controlTransferObj->controlIRP)))
                            {
                                /* There was a problem while submitting the IRP. Update the result and
                                 * the transfer handle. Return the control transfer object back to the
                                 * device object */

                                result = USB_HOST_RESULT_FAILURE;
                                controlTransferObj->inUse = false;
                                *requestHandle = USB_HOST_REQUEST_HANDLE_INVALID;
                            }
                            else
                            {
                                result = USB_HOST_RESULT_SUCCESS;
                            }

                        }

                    }
                }
            }
        }
    }
    return result;
}

