/*******************************************************************************
  USB Device Driver Implementation of device mode operation routines

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhs_device.c

  Summary:
    USB Device Driver Dynamic Implementation of device mode operation routines

  Description:
    The USB device driver provides a simple interface to manage the USB
    modules on Microchip microcontrollers.  This file Implements the 
    interface routines for the USB driver when operating in device mode.

    While building the driver from source, ALWAYS use this file in the build if
    device mode operation is required.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip  Technology  Inc.   All  rights  reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
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

#include "system_config.h"
#include "driver/usb/usbhs/drv_usbhs.h"
#include "driver/usb/usbhs/src/drv_usbhs_local.h"
#include "system/debug/sys_debug.h"

/*****************************************************
 * This structure is a pointer to a set of USB Driver
 * Device mode functions. This set is exported to the
 * device layer when the device layer must use the
 * PIC32MX USB Controller.
 ******************************************************/

DRV_USB_DEVICE_INTERFACE gDrvUSBHSDeviceInterface =
{
    .open = DRV_USBHS_Open,
    .close = DRV_USBHS_Close,
    .eventHandlerSet = DRV_USBHS_ClientEventCallBackSet,
    .deviceAddressSet = DRV_USBHS_DEVICE_AddressSet,
    .deviceCurrentSpeedGet = DRV_USBHS_DEVICE_CurrentSpeedGet,
    .deviceSOFNumberGet = DRV_USBHS_DEVICE_SOFNumberGet,
    .deviceAttach = DRV_USBHS_DEVICE_Attach,
    .deviceDetach = DRV_USBHS_DEVICE_Detach,
    .deviceEndpointEnable = DRV_USBHS_DEVICE_EndpointEnable,
    .deviceEndpointDisable = DRV_USBHS_DEVICE_EndpointDisable,
    .deviceEndpointStall = DRV_USBHS_DEVICE_EndpointStall,
    .deviceEndpointStallClear = DRV_USBHS_DEVICE_EndpointStallClear,
    .deviceEndpointIsEnabled = DRV_USBHS_DEVICE_EndpointIsEnabled,
    .deviceEndpointIsStalled = DRV_USBHS_DEVICE_EndpointIsStalled,
    .deviceIRPSubmit = DRV_USBHS_DEVICE_IRPSubmit,
    .deviceIRPCancelAll = DRV_USBHS_DEVICE_IRPCancelAll,
    .deviceRemoteWakeupStop = DRV_USBHS_DEVICE_RemoteWakeupStop,
    .deviceRemoteWakeupStart = DRV_USBHS_DEVICE_RemoteWakeupStart,
    .deviceTestModeEnter = DRV_USBHS_DEVICE_TestModeEnter
               
            
};

uint16_t _DRV_USBHS_ProcessIRPFIFO
(
    DRV_USBHS_OBJ * hDriver,
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObj,
    uint8_t endpoint,
    uint8_t direction,
    USB_DEVICE_IRP_LOCAL * irp,
    bool * pisDMAUsed,
    bool tryDma
)
{
    /* This function only loads or unloads the FIFO
     * It does not set the Tx Packet Ready or clear'
     * the RX Packet Ready bit. The function
     * returns the amount of data that was written
     * or read from the FIFO */

    /* Start of local variables */
    uint32_t count = 0;
    unsigned int offset;
    uint8_t dmaChannelGrabbed = 0;
    USBHS_MODULE_ID usbID;
    uint8_t * data;
    /* end of local variables */

    data = (uint8_t *)irp->data;
    usbID = hDriver->usbID;

    if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
    {
        /* This means data has to move from device
         * to host. We should write to the FIFO */

        count = (irp->nPendingBytes < endpointObj->maxPacketSize)
                ? irp->nPendingBytes : endpointObj->maxPacketSize;

        offset = (irp->size - irp->nPendingBytes);

        if(endpoint == 0)
        {
            PLIB_USBHS_DeviceEPFIFOLoad(usbID, endpoint, &data[offset], count);
            *pisDMAUsed = false;
            irp->nPendingBytes -= count;
        }
        else
        {
            /* Non EP0 handling */
            if(tryDma == true)
            {
                /* tryDma flag is true. So we will try to get a free DMA channel
                 * If no channel was found, then we process with manual FIFO
                 * access. */
                dmaChannelGrabbed = _DRV_USBHS_DEVICE_Get_FreeDMAChannel(hDriver, USB_DATA_DIRECTION_DEVICE_TO_HOST, endpoint);
                if((dmaChannelGrabbed == 0))
                {
                    /* NO DMA channel available. So do normal FIFO load */
                    PLIB_USBHS_DeviceEPFIFOLoad(usbID, endpoint, &data[offset], count);
                    *pisDMAUsed = false;
                    irp->nPendingBytes -= count;
                }
                else
                {
                    /* DMA CHANNEL has been grabbed and DMA buffer address is OK
                       (divisible by 4). So program the DMA registers now to start
                        DMA operation
                     */

                     hDriver->gDrvUSBDMAPool[dmaChannelGrabbed].count = count;
                     irp->nPendingBytes -= count;
                     *pisDMAUsed = true;
                     PLIB_USBHS_DMAOperationEnable(usbID, endpoint, dmaChannelGrabbed,(void *)&data[offset], count, 0);
                }
            }
            else
            {
                /* Do a manual FIFO access as tryDma flag is false */
                PLIB_USBHS_DeviceEPFIFOLoad(usbID, endpoint, &data[offset], count);
                *pisDMAUsed = false;
                irp->nPendingBytes -= count;
            }
        }
    }
    else
    {
        /* This means data is moving from host to device */

        if((endpoint != 0))
        {
            if(tryDma == true)
            {
                dmaChannelGrabbed = _DRV_USBHS_DEVICE_Get_FreeDMAChannel(hDriver, USB_DATA_DIRECTION_HOST_TO_DEVICE, endpoint);

                if((dmaChannelGrabbed == 0))
                {
                    /* NO DMA channel available. So do normal FIFO unload */
                    count = PLIB_USBHS_DeviceEPFIFOUnload(usbID, endpoint, &data[irp->nPendingBytes]);
                    *pisDMAUsed = false;
                    irp->nPendingBytes += count;
                }
                else
                {
                    /* DMA CHANNEL has been grabbed and DMA buffer address is OK
                       (divisible by 4). So program the DMA registers now to start
                        DMA operation
                     */

                     /* Load the endpoint in INDEX register to access and obtain
                      * Received data count in bytes
                      */
                     count = (uint32_t) PLIB_USBHS_GetReceiveDataCount(usbID, endpoint);

                     hDriver->gDrvUSBDMAPool[dmaChannelGrabbed].count = count;

                     offset = irp->nPendingBytes;

                     irp->nPendingBytes += count;

                     *pisDMAUsed = true;
                     
                     PLIB_USBHS_DMAOperationEnable(usbID, endpoint, dmaChannelGrabbed, 
                             (void *)((uint8_t *)(irp->data) + offset), count, 1);

                }
            }
            else
            {
                /* tryDma flag is false. So we perform manual FIFO access */
                count = PLIB_USBHS_DeviceEPFIFOUnload(usbID, endpoint, &data[irp->nPendingBytes]);
                *pisDMAUsed = false;
                irp->nPendingBytes += count;
            }
        }
        else
        {
            /* For endpoint 0 */
            count = PLIB_USBHS_DeviceEPFIFOUnload(usbID, endpoint, &data[irp->nPendingBytes]);
            *pisDMAUsed = false;
            irp->nPendingBytes += count;
        }
    }

    return(count);
}/* end of _DRV_USBHS_ProcessIRPFIFO() */

void _DRV_USBHS_DEVICE_Initialize(DRV_USBHS_OBJ * drvObj, SYS_MODULE_INDEX index)
{
    /* Start of local variables */
    USBHS_MODULE_ID usbID;
    /* end of local variables */

    usbID = drvObj->usbID;

    /* Disable all endpoint interrupts 
       Enable the reset, the SOF, resume and suspend interrupt */

    PLIB_USBHS_InterruptEnableSet(usbID, 0xF, 0x0, 0x0);
    
    /* In device mode endpoint 0 FIFO size is always 64.
     * So any FIFO allocation should start from 64. The
     * actual value stored in this variable is 64/8 */

    drvObj->consumedFIFOSize = 8;
}/* end of _DRV_USBHS_DEVICE_Initialize() */

// *****************************************************************************
/* Function:
      void DRV_USBHS_DEVICE_AddressSet(DRV_HANDLE handle, uint8_t address)

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_AddressSet client interface function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_AddressSet client
    interface initiaization function for USB device.
    Function checks the input handle validity and on success updates the USB
    Address Register FADDR<6:0> with the address value.

  Remarks:
    See drv_usbhs.h for usage information.
*/

void DRV_USBHS_DEVICE_AddressSet(DRV_HANDLE handle, uint8_t address)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
    /* End of local variables */

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return;
    }

    /* Set the device address */
    hDriver = (DRV_USBHS_OBJ *)handle;
    usbID = hDriver->usbID;

    PLIB_USBHS_DeviceAddressSet(usbID, address);
}/* end of DRV_USBHS_DEVICE_AddressSet() */

// *****************************************************************************
/* Function:
      USB_SPEED DRV_USBHS_DEVICE_CurrentSpeedGet(DRV_HANDLE handle)

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_CurrentSpeedGet client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_CurrentSpeedGet client
    interface initiaization function for USB device.
    Function checks the input handle validity and on success returns value to
    indicate HIGH/FULL speed operation.

  Remarks:
    See drv_usbhs.h for usage information.
*/

USB_SPEED DRV_USBHS_DEVICE_CurrentSpeedGet(DRV_HANDLE handle)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    /* end of local variables */

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
    }

    hDriver = (DRV_USBHS_OBJ *)handle;

    /* The current speed in contained in the
     * device speed member of the driver object */

    return(hDriver->deviceSpeed);
}/* end of DRV_USBHS_DEVICE_CurrentSpeedGet() */

// *****************************************************************************
/* Function:
      void DRV_USBHS_DEVICE_RemoteWakeupStart(DRV_HANDLE handle)

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_RemoteWakeupStart client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_RemoteWakeupStart client
    interface function for USB device.
    Function checks the input handle validity and on success enables the USB
    device to drive resume signalling. RESUME is enabled by programming LPMRES
    bit in LPM_CNTRL register(self clearing bit).

  Remarks:
    See drv_usbhs.h for usage information.
*/

void DRV_USBHS_DEVICE_RemoteWakeupStart(DRV_HANDLE handle)
{
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return;
    }
    /* Commented till PLIB implementation is completed */
    //PLIB_USB_ResumeSignalingEnable(((DRV_USBHS_OBJ *)handle)->usbID);

}/* end of DRV_USBHS_DEVICE_RemoteWakeupStart() */

void DRV_USBHS_DEVICE_RemoteWakeupStop(DRV_HANDLE handle)
{
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return;
    }
    /* Commented till PLIB implementation is completed */
    //PLIB_USB_ResumeSignalingDisable(((DRV_USBHS_OBJ *)handle)->usbID);

}/* end of DRV_USBHS_DEVICE_RemoteWakeupStop() */

void DRV_USBHS_DEVICE_Attach(DRV_HANDLE handle)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
    /* end of local variables */

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return;
    }

    hDriver = (DRV_USBHS_OBJ *)handle;
    usbID = hDriver->usbID;

    /* Attach at the speed at which the driver
     * was initialized */

    PLIB_USBHS_DeviceAttach(usbID, hDriver->operationSpeed);
        
    return; 
}/* end of DRV_USBHS_DEVICE_Attach() */

void DRV_USBHS_DEVICE_Detach(DRV_HANDLE handle)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
    /* end of local variables */

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return;
    }

    hDriver = (DRV_USBHS_OBJ *)handle;
    usbID = hDriver->usbID;

    /* Detach the device */

    PLIB_USBHS_DeviceDetach(usbID);
}/* end of DRV_USBHS_DEVICE_Detach() */

// *****************************************************************************
/* Function:
      void _DRV_USBHS_DEVICE_EndpointObjectEnable
      (
          DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject,
          uint16_t endpointSize,
          USB_TRANSFER_TYPE endpointType,
          USB_BUFFER_DATA01 dataToggle
       )

  Summary:
    Dynamic impementation of _DRV_USBHS_DEVICE_EndpointObjectEnable function.

  Description:
    This is the dynamic impementation of _DRV_USBHS_DEVICE_EndpointObjectEnable
    function for USB device.
    Function populates the endpoint object data structure and sets it to enabled
    state.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_USBHS_DEVICE_EndpointObjectEnable
(
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject,
    uint16_t endpointSize,
    USB_TRANSFER_TYPE endpointType
   
)
{
    /* This is a helper function */
   
    endpointObject->irpQueue        = NULL;
    endpointObject->maxPacketSize   = endpointSize;
    endpointObject->endpointType    = endpointType;
    endpointObject->endpointState  |= DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED;
    
}/* end of _DRV_USBHS_DEVICE_EndpointObjectEnable() */

// *****************************************************************************
/* Function:
      uint16_t DRV_USBHS_DEVICE_SOFNumberGet(DRV_HANDLE client)

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_SOFNumberGet client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_SOFNumberGet client
    interface function for USB device.
    Function checks the validity of the input arguments and on success returns
    the Frame count value.

  Remarks:
    See drv_usbhs.h for usage information.
*/

uint16_t DRV_USBHS_DEVICE_SOFNumberGet(DRV_HANDLE handle)
{   
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "USB Driver: Invalid Handle");
        return 0;
    }

    return(0);
    
}/* end of DRV_USBHS_DEVICE_SOFNumberGet() */

// *****************************************************************************
/* Function:
    USB_ERROR DRV_USBHS_DEVICE_EndpointEnable
    (
      DRV_HANDLE handle,
      USB_ENDPOINT endpointAndDirection,
      USB_TRANSFER_TYPE endpointType,
      uint16_t endpointSize
    )

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_EndpointEnable client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_EndpointEnable client
    interface function for USB device.
    Function enables the specified endpoint.

  Remarks:
    See drv_usbhs.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_EndpointEnable
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpointAndDirection, 
    USB_TRANSFER_TYPE endpointType,
    uint16_t endpointSize
)
{
    /* Start of local variables */
    int     direction;
    int     fifoSize;
    uint8_t endpoint;
    unsigned int shiftWord;
    bool    interruptWasEnabled = false;
    bool    interruptWasEnabledUSBDMA = false;
    bool    mutexLock = false;

    DRV_USBHS_OBJ     * hDriver;
    USBHS_MODULE_ID   usbID;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject;
    uint16_t adjustedEndpointSize =1;
    /* end of local variables */

    /* Enable the endpoint */

    endpoint     = endpointAndDirection & 0xF;
    direction    = ((endpointAndDirection & 0x80) != 0);
    
    if(endpoint >= DRV_USBHS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return(USB_ERROR_PARAMETER_INVALID);
    }

    hDriver  = (DRV_USBHS_OBJ *)handle;
    usbID    = hDriver->usbID;

    /* Get the pointer to the endpoint object */
    
    endpointObject = (hDriver->endpointTable + (2 * endpoint) + 0);

    if(endpoint == 0)
    {

        /* There are two endpoint objects for a control 
         * endpoint */
        
        _DRV_USBHS_DEVICE_EndpointObjectEnable(endpointObject, endpointSize, endpointType);
   
        endpointObject ++;

        _DRV_USBHS_DEVICE_EndpointObjectEnable(endpointObject, endpointSize, endpointType);

        PLIB_USBHS_TxInterruptEnable(usbID, USBHS_TXRXINT_EP0);

        /* EP0 does not require any configuration 
         * It is ready to receive packets when the device is attached */
    }
    else
    {
        /* Non EP0 endpoints require configuration */

        /* Update the endpoint database */
        endpointObject += direction;

        _DRV_USBHS_DEVICE_EndpointObjectEnable(endpointObject, endpointSize, endpointType);
        
        if((hDriver->isInInterruptContext == false) &&
            (hDriver->isInInterruptContextUSBDMA == false))
        {
            /* OSAL: Get mutex */
            if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
                return USB_ERROR_OSAL_FUNCTION;
            }
            mutexLock = true;
        }
        if(hDriver->isInInterruptContext == false)
        {
            interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
        }
        if(hDriver->isInInterruptContextUSBDMA == false)
        {
            interruptWasEnabledUSBDMA = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSourceUSBDma);
        }

        /* The following code maps the endpoint size to the value 
         * that should be loaded in the FIFOSZ register */

        fifoSize = 0;

        /* Find upper 2 power number of endpointSize */
        if (endpointSize)
        {
            while(adjustedEndpointSize < endpointSize)
            {
                adjustedEndpointSize <<= 1;
            }
        }
        else
        {
            adjustedEndpointSize = 0;
        }
        /* endpointSize */
        shiftWord = adjustedEndpointSize;

        while((shiftWord & 0x1) != 1)
        {
            shiftWord = (shiftWord >> 1);
            fifoSize ++;
        }

        fifoSize -= 3;

        if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
        {
            /* Set up the maxpacket size, fifo start address
             * fifosize and enable the interrupt. CLear the data
             * toggle. */
            PLIB_USBHS_DeviceTxEndpointConfigure(usbID, endpoint, endpointSize, hDriver->consumedFIFOSize, fifoSize, endpointType);
        }
        else
        {
            /* Setup the RX endpoint. Disable NYET handshakes. */
            PLIB_USBHS_DeviceRxEndpointConfigure(usbID, endpoint, endpointSize, hDriver->consumedFIFOSize, fifoSize, endpointType);
        }

        hDriver->consumedFIFOSize += (endpointSize/8);

        if(hDriver->isInInterruptContext == false)
        {
            if(interruptWasEnabled)
            {
                /* Enable the interrupt only if it was disabled */
                _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
            }
        }

        if(hDriver->isInInterruptContextUSBDMA == false)
        {
            if(interruptWasEnabledUSBDMA)
            {
                /* Enable the interrupt only if it was disabled */
                _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);
            }
        }

        if(mutexLock == true)
        {
            /* OSAL: Return mutex */
            if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
            }
        }
    }

    return(USB_ERROR_NONE);
}/* end of DRV_USBHS_DEVICE_EndpointEnable() */

// *****************************************************************************
/* Function:
      USB_ERROR DRV_USBHS_DEVICE_EndpointDisable
      (
        DRV_HANDLE handle,
        USB_ENDPOINT endpointAndDirection
      )

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_EndpointDisable client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_EndpointDisable client
    interface function for USB device.
    Function disables the specified endpoint.

  Remarks:
    See drv_usbhs.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_EndpointDisable
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpointAndDirection
)
{
    /* This routine disables the specified endpoint.
     * It does not check if there is any ongoing 
     * communication on the bus through the endpoint
     */

    /* Start of local variables */
    uint8_t endpoint;
    int     direction, iEntry;
    DRV_USBHS_OBJ * hDriver;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject;
    USBHS_MODULE_ID usbID;
    bool interruptWasEnabled = false;
    bool interruptWasEnabledUSBDMA = false;
    bool mutexLock = false;
    /* end of local variables */
   
    /* Check if the handle is valid */
    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    /* Get the pointer to associated endpoint object table */

    hDriver         = (DRV_USBHS_OBJ *)handle;
    usbID           = hDriver->usbID;
    endpointObject  = hDriver->endpointTable;
    endpoint        = endpointAndDirection & 0xF;
    direction       = ((endpointAndDirection & 0x80) != 0);

    if((endpointAndDirection != DRV_USBHS_DEVICE_ENDPOINT_ALL)
            && (endpoint >= DRV_USBHS_ENDPOINTS_NUMBER))
    {
        /* The endpoint number must be either _DRV_USBHS_DEVICE_ENDPOINT_ALL
         * or a valid endpoint */

        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if((hDriver->isInInterruptContext == false) &&
            (hDriver->isInInterruptContextUSBDMA == false))
    {
        /* OSAL: Get Mutex */
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }
        mutexLock = true;
    }
    if(hDriver->isInInterruptContext == false)
    {
        interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        interruptWasEnabledUSBDMA = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSourceUSBDma);
    }


    /* If the endpointAndDirection is _DRV_USBHS_DEVICE_ENDPOINT_ALL
     * then this means that the DRV_USBHS_DEVICE_EndpointDisableAll()
     * function was called */

    if(endpointAndDirection == DRV_USBHS_DEVICE_ENDPOINT_ALL)
    {
        /* Reset the consumed FIFO size */
        hDriver->consumedFIFOSize = 8; 

        /* Disable all the interrupts */

        PLIB_USBHS_TxInterruptDisable(usbID, USBHS_TXRXINT_ALL);
        PLIB_USBHS_RxInterruptDisable(usbID, USBHS_TXRXINT_ALL);

        
        for(iEntry = 0; iEntry < DRV_USBHS_ENDPOINTS_NUMBER; iEntry ++)
        {
            /* Update the endpoint database */
            
            endpointObject->endpointState  &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject ++;
            endpointObject->endpointState  &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject ++;

        }

        if(hDriver->isInInterruptContext == false)
        {
            if(interruptWasEnabled)
            {
                /* IF the interrupt was enabled when entering the routine
                 * re-enable it now */
                _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
            }
        }
        if(hDriver->isInInterruptContextUSBDMA == false)
        {
            if(interruptWasEnabledUSBDMA)
            {
                /* Enable the interrupt only if it was disabled */
                _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);
            }
        }
        if(mutexLock == true)
        {
            /* OSAL: Return mutex */
            if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
            }
        }
        return (USB_ERROR_NONE);
    }

    /* Setup the endpointObj to point to the correct
     * endpoint object Interrupt is still disabled */

    endpointObject += (endpoint * 2) + direction;

    if(((endpointObject->endpointState) & DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED))
    {
        if(endpointObject->endpointType == USB_TRANSFER_TYPE_CONTROL)
        {
            /* Disable a control endpoint and update the
             * endpoint database. */
            endpointObject->endpointState  &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject += 1;
            endpointObject->endpointState  &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED;
        }
        else
        {
            /* Disable a specific endpoint direction for non
             * control endpoints. Also reduce the hDriver consumedFIFOSize
             * member by the size of the disable endpoint. */

            endpointObject->endpointState  &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED;

            /* Release the FIFO only if all the endpoints has not been disabled
             * yet. Otherwise  consumedFIFOSize variable will overflow */
            hDriver->consumedFIFOSize -= (endpointObject->maxPacketSize/8);

            if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
            {
                /* Disable the TX interrupt */
                PLIB_USBHS_TxInterruptDisable(usbID, (1 << endpoint));
            }
            else
            {
                /* Disable the RX interrupt */
                PLIB_USBHS_RxInterruptDisable(usbID, (1 << endpoint));
            }            
        }
    }

    /* Release interrupts and mutex */
    if(hDriver->isInInterruptContext == false)
    {
        if(interruptWasEnabled)
        {
            /* IF the interrupt was enabled when entering the routine
             * re-enable it now */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
        }
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        if(interruptWasEnabledUSBDMA)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);
        }
    }
    if(mutexLock == true)
    {
        /* OSAL: Return mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }
    return(USB_ERROR_NONE);
}/* end of DRV_USBHS_DEVICE_EndpointDisable() */

// *****************************************************************************
/* Function:
    bool DRV_USBHS_DEVICE_EndpointIsEnabled(DRV_HANDLE client,
                                        USB_ENDPOINT endpointAndDirection)

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_EndpointIsEnabled client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_EndpointIsEnabled client
    interface function for USB device.
    Function returns the state of specified endpoint(true\false) signifying
    whether the endpoint is enabled or not.

  Remarks:
    See drv_usbhs.h for usage information.
*/

bool DRV_USBHS_DEVICE_EndpointIsEnabled(DRV_HANDLE handle,
                                        USB_ENDPOINT endpointAndDirection)
{
    /* Return the state of the endpoint */

    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObj;

    uint8_t endpoint = endpointAndDirection & 0xF;
    int direction = ((endpointAndDirection & 0x80) != 0);
    /* end of local variables */

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return false;
    }

    hDriver = (DRV_USBHS_OBJ *)handle;
    endpointObj = hDriver->endpointTable + 
            (2 * endpoint) + direction;
    
    if((endpointObj->endpointState & 
                DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED) != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}/* end of DRV_USBHS_DEVICE_EndpointIsEnabled() */

// *****************************************************************************
/* Function:
    bool DRV_USBHS_DEVICE_EndpointIsStalled(DRV_HANDLE client,
                                        USB_ENDPOINT endpointAndDirection)

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_EndpointIsStalled client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_EndpointIsStalled client
    interface function for USB device.
    Function returns the state of specified endpoint(true\false) signifying
    whether the endpoint is STALLed or not.

  Remarks:
    See drv_usbhs.h for usage information.
*/

bool DRV_USBHS_DEVICE_EndpointIsStalled(DRV_HANDLE handle,
                                        USB_ENDPOINT endpointAndDirection)
{
    /* Return the state of the endpoint */

    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObj;
    uint8_t endpoint = endpointAndDirection & 0xF;
    int direction = ((endpointAndDirection & 0x80) != 0);
    /* end of local variables */

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return false;
    }

    hDriver = (DRV_USBHS_OBJ *)handle;
    endpointObj = hDriver->endpointTable + 
            (2 * endpoint) + direction;
    
    if((endpointObj->endpointState & 
                DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED) != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}/* end of DRV_USBHS_DEVICE_EndpointIsStalled() */

// *****************************************************************************
/* Function:
      USB_ERROR DRV_USBHS_DEVICE_IRPSubmit
      (
        DRV_HANDLE client,
        USB_ENDPOINT endpointAndDirection,
        USB_DEVICE_IRP * inputIRP
       )

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_IRPSubmit client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_IRPSubmit client
    interface function for USB device.
    Function checks the validity of the input arguments and on success adds the
    IRP to endpoint object queue linked list.

  Remarks:
    See drv_usbhs.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_IRPSubmit
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection, 
    USB_DEVICE_IRP * inputIRP
)
{
    /* Start of local variables */
    uint8_t endpoint;
    int direction;
    USBHS_MODULE_ID usbID;
    DRV_USBHS_OBJ * hDriver;
    USB_DEVICE_IRP_LOCAL * irp = (USB_DEVICE_IRP_LOCAL *)inputIRP;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObj;
    bool interruptWasEnabled = false;
    bool interruptWasEnabledUSBDMA = false;
    bool mutexLock = false;
    unsigned int count;
    uint8_t ep0Status;
    uint8_t rxEPStatus;
    bool isDmaUsed = false;
    bool tryDma = true;
    int remainder;
    /* end of local variables */

    /* Check for a valid client */

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Invalid handle");
        return USB_ERROR_PARAMETER_INVALID;
    }

    if(irp->status > USB_DEVICE_IRP_STATUS_SETUP)
    {
        /* This means that the IRP is in use */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Device IRP is already in use");
        return(USB_ERROR_DEVICE_IRP_IN_USE);
    }

    /* Check for a valid endpoint */
    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USBHS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Endpoint is not provisioned for");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    /* Get the driver object, the module ID and the
     * endpoint and direction specific BDT entry and
     * the endpoint object. */

    hDriver     = (DRV_USBHS_OBJ*)handle;
    usbID       = hDriver->usbID;
    endpointObj = hDriver->endpointTable + (2 * endpoint) + direction;

    if(!(endpointObj->endpointState & DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED))
    {
        /* This means the endpoint is not enabled */
        return(USB_ERROR_ENDPOINT_NOT_CONFIGURED);
    }

    /* Check the size of the IRP. If the endpoint receives
     * data from the host, then IRP size must be 
     * multiple of maxPacketSize. If the send ZLP flag is 
     * set, then size must be multiple of endpoint size. */

    remainder = irp->size % endpointObj->maxPacketSize;

    if(remainder == 0)
    {
        /* The IRP size is either 0 or a 
         * exact multiple of maxPacketSize */

        if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
        {
            if(((irp->flags & USB_DEVICE_IRP_FLAG_DATA_COMPLETE) 
                    == USB_DEVICE_IRP_FLAG_DATA_COMPLETE) &&
                (irp->size != 0))
            {
                /* This means a ZLP should be sent after the data
                 * is sent */

                irp->flags |= USB_DEVICE_IRP_FLAG_SEND_ZLP;
            }
        }
    }
    else
    {
        /* Not exact multiple of maxPacketSize */
        if(USB_DATA_DIRECTION_HOST_TO_DEVICE == direction)
        {
            /* For receive IRP it needs to exact multiple of maxPacketSize.
             * Hence this is an error condition. */
            return USB_ERROR_PARAMETER_INVALID;
        }
    }

    /* Now we check if the interrupt context is active. If so
     * the we dont need to get a mutex or disable interrupts.
     * If this were being done in non interrupt context, we, then
     * we would disable the interrupt. In which case we would
     * get the mutex and then disable the interrupt */

    if((hDriver->isInInterruptContext == false) &&
            (hDriver->isInInterruptContextUSBDMA == false))
    {
        /* OSAL: Get mutex */
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }
        mutexLock = true;
    }
    if(hDriver->isInInterruptContext == false)
    {
        interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        interruptWasEnabledUSBDMA = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSourceUSBDma);
    }

    irp->next = NULL;

    /* If the data is moving from device to host
     * then pending bytes is data remaining to be
     * sent to the host. If the data is moving from
     * host to device, nPendingBytes tracks the
     * amount of data received so far */

    if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
    {
        irp->nPendingBytes = irp->size;
    }
    else
    {
        irp->nPendingBytes = 0;
    }

    /* Mark the IRP status as pending */
    irp->status = USB_DEVICE_IRP_STATUS_PENDING;

    /* Get the last object in the endpoint object
     * IRP Queue */
    if(endpointObj->irpQueue == NULL)
    {
        /* Queue is empty */
        endpointObj->irpQueue = irp;
        irp->previous = NULL;

        /* Because this is the first IRP in the queue
         * then we we must arm the endpoint entry in
         * the BDT. */

        irp->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
        
        if(endpoint == 0)
        {
            /* Endpoint zero IRPs require special handling */

            if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
            {
                /* Because we allow the client to provide three/two different
                 * IRPs for the three/two stages of the control transfer. we
                 * need to check what stage we are in when we get an IRP. The
                 * endpoint0State is manipulated by both the main thread and
                 * the interrupt thread. */

                /* Check if the endpoint is waiting for a IRP */
                if(hDriver->endpoint0State == DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT)
                {
                    /* In this state, the USB module is waiting for transmit
                     * data from the client.  This would be a Control Read
                     * transfer. And client has provided it */

                    count = _DRV_USBHS_ProcessIRPFIFO(hDriver, endpointObj, (uint8_t)0,
                            (uint8_t)USB_DATA_DIRECTION_DEVICE_TO_HOST, irp, &isDmaUsed, tryDma);
                    hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS;
                    if(count < endpointObj->maxPacketSize)
                    {
                        /* This means that this is the last packet
                         * in the control read transfer. Set the
                         * transmit ready and the data end bit */
                        PLIB_USBHS_EP0TxPktRdyDataEnd(usbID);
                    }
                    else
                    {
                        /* This is a multi data stage transfer. Dont
                         * set the data end bit. Only set the
                         * transmit ready bit. */
                        PLIB_USBHS_EP0TxPktRdy(usbID);
                    }
                }
                else if(hDriver->endpoint0State == DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT)
                {
                    /* In this state, the module is waiting for the client to
                     * complete the handshake stage of the control write
                     * transfer. The client needs to let the module know that
                     * it has received the data. */

                    hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST;
                    PLIB_USBHS_EP0RxPktRdyServicedDataEnd(usbID);

                }
            }
            else
            {
                /* Receive direction handling here */

                if(hDriver->endpoint0State == DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_IRP_FROM_CLIENT)
                {
                    ep0Status = PLIB_USBHS_EP0StatusGet(usbID);
                    
                    /* Check if data is already present in the FIFO */
                    if(ep0Status & USBHS_EP0_RXPKTRDY)
                    {
                        /* This means we already have data in the FIFO
                         * We will need to check if the IRP is completed */

                        count = _DRV_USBHS_ProcessIRPFIFO(hDriver, endpointObj, 
                                0, USB_DATA_DIRECTION_HOST_TO_DEVICE, irp, &isDmaUsed, tryDma);

                        /* Check if the irp is completed */
                        if((count < endpointObj->maxPacketSize)
                                || (irp->nPendingBytes >= irp->size))
                        {
                            /* This means we received a short packet
                             * or the amount of data received is the amount
                             * that was expected */

                            if(irp->size < irp->nPendingBytes)
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                            }
                            else
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            }

                            irp->size = irp->nPendingBytes;

                            endpointObj->irpQueue = irp->next;
                            if(irp->callback)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }

                            /* We can now move to the handshake stage. Check
                             * if a TX IRP is available */

                            if((endpointObj + 1)->irpQueue != NULL)
                            {
                                hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST;

                                /* Data stage is complete. Clear the Rx Pkt Ready
                                 * and set the data end bit. */
                                PLIB_USBHS_EP0RxPktRdyServicedDataEnd(usbID);
                            }
                            else
                            {
                                hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT;
                            }
                        }
                        else
                        {
                            /* We are expecting more data. Let the module know
                             * that we have completed reading the data */
                            PLIB_USBHS_EP0RxPktRdyServiced(usbID);
                        }
                    }
                    else
                    {
                        /* There is no data in the FIFO yet */
                        hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_STAGE_FROM_HOST;
                    }
                }
                else if(hDriver->endpoint0State == DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_IRP_FROM_CLIENT)
                {
                    /* This IRP will receive the status stage ZLP for 
                     * a control read transfer. The ZLP in this case 
                     * comes from the host. */

                    hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_STAGE_FROM_HOST;
                    PLIB_USBHS_EP0DataEndSet(usbID);
                }
            }
        }
        else
        {
            /* Non zero endpoint irp */


            if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
            {
                /* Data is moving from device to host. So this
                 * a TX IRP. We dont have to check for the TXxPktRdy
                 * bit because the fact that the queue is empty
                 * means there is nothing in the queue */

               _DRV_USBHS_ProcessIRPFIFO(hDriver, endpointObj, endpoint, direction, irp, &isDmaUsed, tryDma);

               if(isDmaUsed == false)
               {
                   /* Set the TX packet Ready bit */

                   PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, endpoint);
               }

               /* The rest of the IRP processing takes place in ISR */

            }
            else
            {
                /* We have an IRP to process. Lets check if the RxPktRdy
                 * bit is already set on the endpoint. If so we can start
                 * unloading the fifo */

                rxEPStatus = PLIB_USBHS_RxEPStatusGet(usbID, endpoint);

                if(rxEPStatus & USBHS_RXEP_PKTRDY)
                {
                    /* This means the FIFO already contains data */
                    tryDma = false;
                    count = _DRV_USBHS_ProcessIRPFIFO(hDriver, endpointObj,
                            endpoint, direction, irp, &isDmaUsed, tryDma);

                    if(isDmaUsed == false)
                    {
                        /* Clear the Rx Packet Ready Bit */
                         PLIB_USBHS_RxEPStatusClear(usbID, endpoint, USBHS_RXEP_PKTRDY);

                        /* Check if the irp is completed */

                        if((count < endpointObj->maxPacketSize)
                                || (irp->nPendingBytes >= irp->size))
                        {
                            /* This means we received a short packet
                             * or the amount of data received is the amount
                             * that was expected */

                            if(irp->size < irp->nPendingBytes)
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                            }
                            else
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            }

                            irp->size = irp->nPendingBytes;
                            
                            irp->nPendingBytes = 0;

                            endpointObj->irpQueue = irp->next;
                            if(irp->callback)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }
                        }
                    } /* End of DMA not used */
                }
            }/* End of non zero RX IRP submit */
        }/* End of non zero IRP submit */
    }
    else
    {
        /* This means we should surf the linked list
         * to get to the last entry . */
        USB_DEVICE_IRP_LOCAL * iterator;
        iterator = endpointObj->irpQueue;
        while(iterator->next != NULL)
        {
            iterator = iterator->next;
        }
        iterator->next = irp;
        irp->previous = iterator;
        irp->status = USB_DEVICE_IRP_STATUS_PENDING;
    }

    if(hDriver->isInInterruptContext == false)
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
        }
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        if(interruptWasEnabledUSBDMA)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);
        }
    }
    if(mutexLock == true)
    {
        /* OSAL: Return mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }
    return(USB_ERROR_NONE);

}/* end of DRV_USBHS_DEVICE_IRPSubmit() */

// *****************************************************************************
/* Function:
      void _DRV_USBHS_DEVICE_IRPQueueFlush
      (
         DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject,
         USB_DEVICE_IRP_STATUS status

      )

  Summary:
    Dynamic impementation of _DRV_USBHS_DEVICE_IRPQueueFlush function.

  Description:
    This is the dynamic impementation of _DRV_USBHS_DEVICE_IRPQueueFlush
    function for USB device.
    Function scans for all the IRPs on the endpoint queue and cancels them all.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_USBHS_DEVICE_IRPQueueFlush
(
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject,
    USB_DEVICE_IRP_STATUS status

)
{
    /* Start of local variables */
    USB_DEVICE_IRP_LOCAL * iterator;
    /* end of local variables */
    
    /* Check if any IRPs are assigned on this endpoint and 
     * abort them */

    if(endpointObject->irpQueue != NULL)
    {
        /* Cancel the IRP and deallocate driver IRP 
         * objects */

        iterator = endpointObject->irpQueue;

        while(iterator != NULL)
        {
            iterator->status = status;
            if(iterator->callback != NULL)
            {
                iterator->callback((USB_DEVICE_IRP *)iterator);
            }
            iterator = iterator->next;
        }
    }
    
    /* Set the head pointer to NULL */
    endpointObject->irpQueue = NULL;
}/* end of _DRV_USBHS_DEVICE_IRPQueueFlush() */

// *****************************************************************************
/* Function:
      USB_ERROR DRV_USBHS_DEVICE_IRPCancelAll
      (
         DRV_HANDLE client,
         USB_ENDPOINT endpointAndDirection
      )

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_IRPCancelAll client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_IRPCancelAll client
    interface function for USB device.
    Function checks the validity of the input arguments and on success cancels
    all the IRPs on the specific endpoint object queue.

  Remarks:
    See drv_usbhs.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_IRPCancelAll
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)
{
    /* Start of local variables */
    int     direction;
    uint8_t endpoint;
    DRV_USBHS_OBJ     * hDriver;
    USBHS_MODULE_ID   usbID;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject;
    bool interruptWasEnabled = false;
    bool interruptWasEnabledUSBDMA = false;
    bool mutexLock = false;
    /* end of local variables */

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USBHS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver      = (DRV_USBHS_OBJ*)handle;
    usbID        = hDriver->usbID;

    /* Get the endpoint object */
    endpointObject = hDriver->endpointTable + (2 * endpoint)
        + direction;

    if((hDriver->isInInterruptContext == false) &&
            (hDriver->isInInterruptContextUSBDMA == false))
    {
        /* OSAL: Get mutex */
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }
        mutexLock = true;
    }
    if(hDriver->isInInterruptContext == false)
    {
        interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        interruptWasEnabledUSBDMA = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSourceUSBDma);
    }

    /* Flush the endpoint */
    _DRV_USBHS_DEVICE_IRPQueueFlush(endpointObject, USB_DEVICE_IRP_STATUS_ABORTED);

    /* We need to flush the TXFIFO if any of them are
     * loaded. Endpoint zero does not have a direction.
     * For other endpoints we check if the direction
     * is transmit */

    if((endpoint == 0) || 
            ((endpoint != 0) && (direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)))
    {
        if(endpoint == 0)
        {
 //           PLIB_USBHS_Endpoint0FIFOFlush(usbID);
        }  
        else
        {
            PLIB_USBHS_EndpointTxFIFOFlush(usbID, endpoint);
        }
    }
    if(hDriver->isInInterruptContext == false)
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
        }
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        if(interruptWasEnabledUSBDMA)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);
        }
    }
    if(mutexLock == true)
    {
        /* OSAL: Return mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }
    return(USB_ERROR_NONE);
}/* end of DRV_USBHS_DEVICE_IRPCancelAll() */

// *****************************************************************************
/* Function:
    USB_ERROR DRV_USBHS_DEVICE_IRPCancel
    (
        DRV_HANDLE client, 
        USB_DEVICE_IRP * irp
    )

  Summary:
    Dynamic implementation of DRV_USBHS_DEVICE_IRPCancel client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHS_DEVICE_IRPCancel client
    interface function for USB device.  Function checks the validity of the
    input arguments and on success cancels  the specific IRP.
    An IRP that was in the queue but that has been processed yet will be
    cancelled successfully and the IRP callback function will be called from
    this function with USB_DEVICE_IRP_STATUS_ABORTED status. The application can
    release the data buffer memory used by the IRP when this callback occurs.
    If the IRP was in progress (a transaction in on the bus) when the cancel
    function was called, the IRP will be cancelled only when an ongoing or the
    next transaction has completed. The IRP callback function will then be
    called in an interrupt context. The application should not release the
    related data buffer unless the IRP callback has occurred.

  Remarks:
    See drv_usbhs.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_IRPCancel
(
    DRV_HANDLE client, 
    USB_DEVICE_IRP * irp
)
{
    /* Start of local variables */
    USB_ERROR returnValue = USB_ERROR_NONE;
    DRV_USBHS_OBJ * hDriver = NULL;
    USB_DEVICE_IRP_LOCAL * irpToCancel = (USB_DEVICE_IRP_LOCAL *)irp;
    bool interruptWasEnabled = false;
    bool interruptWasEnabledUSBDMA = false;
    bool mutexLock = false;
    /* End of local variables */
    
    /* Check if the handle is valid */
    if(DRV_HANDLE_INVALID == client)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        returnValue = USB_ERROR_PARAMETER_INVALID;
    }
    /* Check if the IRP is NULL */
    else if(irpToCancel == NULL)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "IRP is invalid");
        returnValue = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        hDriver = ((DRV_USBHS_OBJ *)client);
        
        if((hDriver->isInInterruptContext == false) &&
            (hDriver->isInInterruptContextUSBDMA == false))
        {
            /* OSAL: Get mutex */
            if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                    OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
                returnValue = USB_ERROR_OSAL_FUNCTION;
            }
            mutexLock = true;
        }
        if(hDriver->isInInterruptContext == false)
        {
            interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
        }
        if(hDriver->isInInterruptContextUSBDMA == false)
        {
            interruptWasEnabledUSBDMA = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSourceUSBDma);
        }

        if(returnValue == USB_ERROR_NONE)
        {
            if(irpToCancel->status <= USB_DEVICE_IRP_STATUS_COMPLETED_SHORT)
            {
                /* This IRP has either completed or has been aborted.*/
                returnValue = USB_ERROR_PARAMETER_INVALID;
            }
            else
            {
                /* The code will come here both when the IRP is NOT the 1st
                 * in queue as well as when it is at the HEAD. We will change
                 * the IRP status for either scenario but will give the callback
                 * only if it is NOT at the HEAD of the queue.
                 * 
                 * What it means for HEAD IRP case is it will be caught in USB
                 * ISR and will be further processed in ISR. This is done to
                 * make sure that the user cannot release the IRP buffer before
                 * ABORT callback*/
                
                /* Mark the IRP status as aborted */
                irpToCancel->status = USB_DEVICE_IRP_STATUS_ABORTED;
                
                /* No data for this IRP was sent or received */
                irpToCancel->size = 0;
                
                if(irpToCancel->previous != NULL)
                {
                    /* This means this is not the HEAD IRP in the IRP queue.
                     Can be removed from the endpoint object queue safely.*/
                    irpToCancel->previous->next = irpToCancel->next;

                    if(irpToCancel->next != NULL)
                    {
                        /* If this is not the last IRP in the queue then update
                         the previous link connection for the next IRP */
                        irpToCancel->next->previous = irpToCancel->previous;
                    }

                    irpToCancel->previous = NULL;
                    irpToCancel->next = NULL;
                    
                    if(irpToCancel->callback != NULL)
                    {
                        irpToCancel->callback((USB_DEVICE_IRP *)irpToCancel);
                    }
                }
            }
        }
        
        if(( (returnValue == USB_ERROR_NONE) || 
                (returnValue == USB_ERROR_PARAMETER_INVALID) ))
        {
            if(hDriver->isInInterruptContext == false)
            {
                if(interruptWasEnabled)
                {
                    /* Enable the interrupt only if it was disabled */
                    _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
                }
            }
            if(hDriver->isInInterruptContextUSBDMA == false)
            {
                if(interruptWasEnabledUSBDMA)
                {
                    /* Enable the interrupt only if it was disabled */
                    _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);
                }
            }
            if(mutexLock == true)
            {
                /* OSAL: Return mutex */
                if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
                {
                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
                }
            }
        }
    }
    
    return returnValue;
    
}/* End of DRV_USBHS_DEVICE_IRPCancel() */

// *****************************************************************************
/* Function:
      USB_ERROR DRV_USBHS_DEVICE_EndpointStall
      (
        DRV_HANDLE client,
        USB_ENDPOINT endpointAndDirection
      )

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_EndpointStall client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_EndpointStall client
    interface function for USB device.
    Function sets the STALL state of the specified endpoint.

  Remarks:
    See drv_usbhs.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_EndpointStall(DRV_HANDLE handle,
                                    USB_ENDPOINT endpointAndDirection)
{
    /* Start of local variables */
    int     direction;
    bool    interruptWasEnabled = false;
    bool    interruptWasEnabledUSBDMA = false;
    bool    mutexLock = false;
    uint8_t endpoint;
    DRV_USBHS_OBJ     * hDriver;
    USBHS_MODULE_ID   usbID;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject;
    /* end of local variables */

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0); 

    if(endpoint >= DRV_USBHS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver      = (DRV_USBHS_OBJ*)handle;
    usbID        = hDriver->usbID;

    /* Get the endpoint object */
    endpointObject = hDriver->endpointTable + (2 * endpoint);
    
    if((hDriver->isInInterruptContext == false) &&
            (hDriver->isInInterruptContextUSBDMA == false))
    {
        /* OSAL: Get mutex */
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }
        mutexLock = true;
    }
    if(hDriver->isInInterruptContext == false)
    {
        interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        interruptWasEnabledUSBDMA = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSourceUSBDma);
    }

    if(endpoint == 0)
    {
        PLIB_USBHS_EP0StallEnable(usbID);

        /* While stalling endpoint 0 we stall both directions */
        endpointObject->endpointState |= DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED;
        _DRV_USBHS_DEVICE_IRPQueueFlush(endpointObject, USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT);
        endpointObject ++;
        endpointObject->endpointState |= DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED;
        _DRV_USBHS_DEVICE_IRPQueueFlush(endpointObject, USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT);
    }
    else
    {
        /* Stalling a non zero endpoint */
        if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
        {
            /* Stall the TX direction */
            PLIB_USBHS_DeviceTxEndpointStallEnable(usbID, endpoint);
        }
        else
        {
            /* Stall the RX direction */
            PLIB_USBHS_DeviceRxEndpointStallEnable(usbID, endpoint);
        }

        endpointObject += direction;
        endpointObject->endpointState |= DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED;
        _DRV_USBHS_DEVICE_IRPQueueFlush(endpointObject, USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT);
    }

    if(hDriver->isInInterruptContext == false)
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
        }
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        if(interruptWasEnabledUSBDMA)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);
        }
    }
    if(mutexLock == true)
    {
        /* OSAL: Return mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }

    return(USB_ERROR_NONE);
}/* end of DRV_USBHS_DEVICE_EndpointStall() */

// *****************************************************************************
/* Function:
      USB_ERROR DRV_USBHS_DEVICE_EndpointStallClear
      (
        DRV_HANDLE client,
        USB_ENDPOINT endpointAndDirection
      )

  Summary:
    Dynamic impementation of DRV_USBHS_DEVICE_EndpointStallClear client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBHS_DEVICE_EndpointStallClear
    client interface function for USB device.  Function clears the STALL state
    of the specified endpoint and resets the data toggle value.

  Remarks:
    See drv_usbhs.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_EndpointStallClear
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)
{
    /* Start of local variables */
    int     direction;
    bool    interruptWasEnabled = false;
    bool    interruptWasEnabledUSBDMA = false;
    bool    mutexLock = false;
    uint8_t endpoint;
    DRV_USBHS_OBJ     * hDriver;
    USBHS_MODULE_ID   usbID;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObject;
    /* end of local variables */

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USBHS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if((DRV_HANDLE_INVALID == handle))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = (DRV_USBHS_OBJ *)handle;
    usbID = hDriver->usbID;

    if(endpoint == 0)
    {
        /* For endpoint zero, the STALL is automatically
         * cleared when the STALL is sent to the host */
        return(USB_ERROR_NONE);
    }

    if((hDriver->isInInterruptContext == false) &&
            (hDriver->isInInterruptContextUSBDMA == false))
    {
        /* OSAL: Mutex Get */
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }
        mutexLock = true;
    }
    if(hDriver->isInInterruptContext == false)
    {
        interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        interruptWasEnabledUSBDMA = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSourceUSBDma);
    }

    if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
    {
        /* Clear the stall on the TX direction
         * and the stall sent bit */

        PLIB_USBHS_DeviceTxEndpointStallDisable(usbID, endpoint);
    }
    else
    {
        /* Clear the stall on the RX direction 
         * and the stall sent bit */

        PLIB_USBHS_DeviceRxEndpointStallDisable(usbID, endpoint);
    }

    /* Get the endpoint object */
    endpointObject = hDriver->endpointTable + (2 * endpoint)
                        + direction;
    endpointObject->endpointState &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED;

    _DRV_USBHS_DEVICE_IRPQueueFlush(endpointObject, USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST);

    if(hDriver->isInInterruptContext == false)
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
        }
    }
    if(hDriver->isInInterruptContextUSBDMA == false)
    {
        if(interruptWasEnabledUSBDMA)
        {
            /* Enable the interrupt only if it was disabled */
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);
        }
    }
    if(mutexLock == true)
    {
        /* OSAL: Return mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }

    return(USB_ERROR_NONE);
}/* end of DRV_USBHS_DEVICE_EndpointStallClear() */



// *****************************************************************************
/* Function:
      void _DRV_USBHS_DEVICE_Tasks_ISR(DRV_USBHS_OBJ * hDriver)

  Summary:
    Dynamic impementation of _DRV_USBHS_DEVICE_Tasks_ISR ISR handler function.

  Description:
    This is the dynamic impementation of _DRV_USBHS_DEVICE_Tasks_ISR ISR handler
    function for USB device.
    Function will get called automatically due to USB interrupts in interrupt mode.
    In polling mode this function will be routinely called from USB driver
    DRV_USBHS_Tasks() function.
    This function performs necessary action based on the interrupt and clears the
    interrupt after that. The USB device layer callback is called with the
    interrupt event details, if callback function is registered.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/


void _DRV_USBHS_DEVICE_Tasks_ISR(DRV_USBHS_OBJ * hDriver)
{
    /* Start of local variables */
    uint8_t usbInterrupts;
    uint16_t endpointMask;
    uint8_t  iEndpoint;
    uint8_t  ep0Status;
    uint8_t  txEPStatus;
    uint8_t  rxEPStatus;
    uint8_t endpointTXInterrupts;
    uint8_t endpointRXInterrupts;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObjReceive;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObjTransmit;
    USBHS_MODULE_ID usbID;
    USB_DEVICE_IRP_LOCAL * irp;
    uint16_t endpoint0DataStageSize;
    unsigned int endpoint0DataStageDirection;

    unsigned int count;
    uint8_t * data;
    bool isDmaUsed = false;
    bool tryDma = true;
    /* end of local variables */

    _DRV_USBHS_NonPersistentInterruptSourceClear(hDriver->interruptSource);

    if(!hDriver->isOpened)
    {
        /* We need a valid client */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver does not have a client");
        return;
    }

    if(hDriver->pEventCallBack == NULL)
    {
        /* We need a valid event handler */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver needs a event handler");
        return;
    }

    usbID = hDriver->usbID;

    /* Reading the interrupt flags will cause the
     * flags to get cleared */

    usbInterrupts = PLIB_USBHS_GenInterruptFlagsGet(usbID);

    if(usbInterrupts & USBHS_GENINT_RESET)
    {
        /* This means that RESET signalling was detected 
         * on the bus. This means the packet that we should
         * get on the bus for EP0 should be a setup packet */

        hDriver->endpoint0State = 
            DRV_USBHS_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;


        /* Default speed is full speed */
        hDriver->deviceSpeed = USB_SPEED_FULL;

        if(hDriver->operationSpeed == USB_SPEED_HIGH)
        {
            /* If high speed operation was specified
               and we were able to connect at high speed
               then update the speed.
               */
            if(PLIB_USBHS_HighSpeedIsConnected(usbID))
            {
                /* This means high speed was successful*/
                hDriver->deviceSpeed = USB_SPEED_HIGH;
            }
        }

        hDriver->pEventCallBack(hDriver->hClientArg,
                DRV_USBHS_EVENT_RESET_DETECT,  NULL);
    }

    if(usbInterrupts & USBHS_GENINT_SUSPEND)
    {
        /* This means that the bus was SUSPENDED. */

        hDriver->pEventCallBack(hDriver->hClientArg,
                DRV_USBHS_EVENT_IDLE_DETECT, NULL);
    }

    if(usbInterrupts & USBHS_GENINT_RESUME)
    {
        /* This means that RESUME was detected on the bus. */

        hDriver->pEventCallBack(hDriver->hClientArg,
                DRV_USBHS_EVENT_RESUME_DETECT, NULL);
    }        

    if(usbInterrupts & USBHS_GENINT_SOF)
    {
        /* This measn that there was a SOF. */

        hDriver->pEventCallBack(hDriver->hClientArg,
                DRV_USBHS_EVENT_SOF_DETECT, NULL);
    }        

    /* Read the endpoint interrupts */

    endpointTXInterrupts = PLIB_USBHS_TxInterruptFlagsGet(usbID);
    endpointRXInterrupts = PLIB_USBHS_RxInterruptFlagsGet(usbID);

    /* Setup the endpoint objects */

    endpointObjReceive = hDriver->endpointTable; 
    endpointObjTransmit = hDriver->endpointTable + 1;

    if(endpointTXInterrupts & USBHS_TXRXINT_EP0)
    {
        /* This means this is EP0 interrupt */
        PLIB_USBHS_LoadEPInIndex(usbID, 0);
        ep0Status = PLIB_USBHS_EP0StatusGet(usbID);

        if(ep0Status & USBHS_EP0_SENTSTALL)
        {
            /* This means a stall was sent. Reset the 
             * endpoint 0 state.  Clear the sent stall bit */
            hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;
            PLIB_USBHS_EP0SentStallClear(usbID);
            endpointObjReceive->endpointState &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED;
            endpointObjTransmit->endpointState &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED;
        }

        else if(ep0Status & USBHS_EP0_SETUPEND)
        {
            /* This means the current control transfer
             * end prematurely. We dont need to end any 
             * transfers. The device layer will manage
             * the premature transfer end. */
            PLIB_USBHS_EP0SetupEndServiced(usbID);
            if(!hDriver->isZeroLengthControlTransfer)
            {
                /* The setup end bit gets set for zero length transfers. If
                 * this was not a zero length transfer and the setup end bit
                 * is set, then we have genuine control transfer abort. */
                hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;
            }
        }

        /* We have to check for these conditions before we check for the
           endpoint 0 receive packet because these conditions could potentially
           clear up the IRP queue. These conditions do not have any explicit
           status bit associated with them, but the module will generate an
           interrupt anyways*/

        if(hDriver->endpoint0State == DRV_USBHS_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS)
        {

            irp = endpointObjTransmit->irpQueue;

            if(irp != NULL)
            {
                /* Check if the Transmit IRP is done */
                if(irp->nPendingBytes == 0)
                {
                    if((irp->flags & USB_DEVICE_IRP_FLAG_SEND_ZLP) != 0)
                    {
                        /* This means we have to send a ZLP. Clear the
                         * flag */

                        irp->flags &= ~USB_DEVICE_IRP_FLAG_SEND_ZLP;
                        /* Only set the transmit ready bit. */
                        PLIB_USBHS_EP0TxPktRdy(usbID);
                    }
                    else
                    {
                        /* This means we got done with the IRP */
                        endpointObjTransmit->irpQueue = irp->next;
                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                        if(irp->callback != NULL)
                        {
                            irp->callback((USB_DEVICE_IRP *)irp);
                        }

                        /* We should check if the status stage IRP is available */

                        if(endpointObjReceive->irpQueue != NULL)
                        {
                            /* This is the status IRP */

                            irp = endpointObjReceive->irpQueue;
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            irp->size = 0;
                            endpointObjReceive->irpQueue = irp->next;
                            if(irp->callback != NULL)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }
                        }

                        else
                        {
                            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Was expecting status IRP");
                        }

                        /* We will just move on to the next control transfer */
                        hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;
                    }
                }
                else
                {
                    /* There is more data in the IRP. 
                     * Continue to process it */

                    count =  _DRV_USBHS_ProcessIRPFIFO(hDriver,endpointObjTransmit,
                            0, USB_DATA_DIRECTION_DEVICE_TO_HOST, irp, &isDmaUsed, tryDma);

                    if(count < endpointObjTransmit->maxPacketSize)
                    {
                        /* This means that this is the last packet
                         * in the control read transfer. Set the
                         * transmit ready and the data end bit */
                        *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0xA;

                    }
                    else
                    {
                        /* This is a multi data stage transfer. Dont
                         * set the data end bit. Only set the
                         * transmit ready bit. */
                        *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0x2;
                    }
                }
            }
        } /* End of checking for TX Data Stage processing */

        else if(hDriver->endpoint0State == DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST)
        {
            /* This means the TX status stage has completed */

            irp = endpointObjTransmit->irpQueue;
            if(irp != NULL)
            {
                irp->size = 0;
                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                endpointObjTransmit->irpQueue = irp->next;
                if(irp->callback != NULL)
                {
                    irp->callback((USB_DEVICE_IRP *)irp);
                }
                hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;
            }
        }

        else if(ep0Status & USBHS_EP0_RXPKTRDY)
        {
            /* This means we have received data on the
             * endpoint */

            if (hDriver->endpoint0State ==
                    DRV_USBHS_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST)
            {
                /* This means we were expecting a SETUP packet
                 * and we got one. Get the endpoint 0 RX endpoint
                 * object and get the IRP at top of the queue */

                irp = endpointObjReceive->irpQueue;

                /* If we have received a setup packet, this means that if a 
                 * stall was scheduled, then it has been sent. So we can 
                 * clear the stall status of the endpoint object */
                endpointObjReceive->endpointState &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED;
                endpointObjTransmit->endpointState &= ~DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED;

                if(irp == NULL)
                {
                    /* This means the device layer did not
                     * schedule a IRP to receive a SETUP
                     * packet! */

                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"No IRP to receive setup packet!");
                    Nop();
                    Nop();
                }
                else
                {

                    /* We have an IRP for the setup packet. Transfer
                     * the data from the FIFO to the IRP */

                    data = (uint8_t *)irp->data;
                    *((unsigned int *)(data)) =
                        *((unsigned int *)PLIB_USBHS_GetEP0FIFOAddress(usbID));
                    *((unsigned int *)(data + 4)) =
                        *((unsigned int *)PLIB_USBHS_GetEP0FIFOAddress(usbID));

                    /* Remember the size and direction of the data stage */
                    endpoint0DataStageSize = *((unsigned short int *)(data + 6));
                    endpoint0DataStageDirection = ((data[0] & 0x80) != 0);

                    if(endpoint0DataStageSize > 0)
                    {
                        /* Set the Serviced RX packet Ready bit which
                         * will then clear the RX packet ready bit. This
                         * let the hardware know that FIFO has been read */
                        hDriver->isZeroLengthControlTransfer = false;

                    }

                    /* Indicate that this is a setup IRP */
                    irp->status = USB_DEVICE_IRP_STATUS_SETUP;
                    irp->size = 8;

                    /* Update the endpoint IRP queue. If
                     * irp->next is NULL, then this means
                     * at this point we dont have anything
                     * in the queue */

                    endpointObjReceive->irpQueue = irp->next;
                    if(irp->callback != NULL)
                    {
                        irp->callback((USB_DEVICE_IRP *)irp);
                    }

                    /* Check the size and the direction of the
                     * control transfer */

                    if(endpoint0DataStageSize == 0)
                    {
                        if(!(endpointObjReceive->endpointState & DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED))
                        {
                            /* Set a flag to remember that this is a
                             * zero length control transfer */
                            hDriver->isZeroLengthControlTransfer = true;

                            /* This means data was moving from
                             * host to device. The status stage ZLP
                             * will come from device. Service the
                             * TX endpoint on device */

                            if(endpointObjTransmit->irpQueue != NULL)
                            {
                                /* That means we have a IRP to be processed */
                                endpointObjTransmit->irpQueue->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
                                hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST;

                                /* Set the data end bit and the serviced packet
                                 * ready bit */
                                *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0x48;

                            }
                            else
                            {
                                /* If here means we have to wait for an IRP submit
                                 * to complete this transfer. The RxPktRdyServiced
                                 * and the data end bit is set in the IRP submit
                                 * function */
                                hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT;
                            }
                        }

                    }
                    else
                    {
                        if(!(endpointObjReceive->endpointState & DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED))
                        {
                            /* This means that there is a data stage
                            */
                            if(endpoint0DataStageDirection == USB_DATA_DIRECTION_DEVICE_TO_HOST)
                            {
                                /* Clear the Rx packet ready bit */
                                *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0x40;

                                /* This means the data stage is moving from device
                                 * to host which means a transmit endpoint is used
                                 * */

                                if(endpointObjTransmit->irpQueue != NULL)
                                {
                                    /* This means we have a TX IRP to process */

                                    hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS;
                                    endpointObjTransmit->irpQueue->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;

                                    count = _DRV_USBHS_ProcessIRPFIFO(hDriver,endpointObjTransmit,
                                            0, USB_DATA_DIRECTION_DEVICE_TO_HOST,
                                            endpointObjTransmit->irpQueue, &isDmaUsed, tryDma);

                                    if(count < endpointObjTransmit->maxPacketSize)
                                    {
                                        /* This means that this is the last packet
                                         * in the control read transfer. Set the
                                         * transmit ready and the data end bit */
                                        *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0xA;
                                    }
                                    else
                                    {
                                        /* This is a multi data stage transfer. Dont
                                         * set the data end bit. Only set the
                                         * transmit ready bit. */
                                        *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0x2;
                                    }
                                }
                                else
                                {
                                    /* This means we dont have a IRP ready */
                                    hDriver->endpoint0State =
                                        DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT;
                                }
                            }
                            else
                            {
                                /* This means the data stage is moving from host
                                 * to device which means a receive endpoint is used
                                 */
                                *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0x40;
                                if(endpointObjReceive->irpQueue != NULL)
                                {
                                    /* This means we have a RX IRP to process */

                                    hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_STAGE_FROM_HOST;
                                    endpointObjReceive->irpQueue->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
                                }
                                else
                                {
                                    /* This means we dont have a IRP ready */
                                    hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_IRP_FROM_CLIENT;
                                }
                            }
                        }
                    } /* End of data stage available else condition */
                } /* End of valid setup irp avaialble handling */
            } /* End of expecting Setup Packet Handling state */

            else if(hDriver->endpoint0State == DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_STAGE_FROM_HOST)
            {
                /* This means were expecting some data from the host
                 * and have received it */

                irp = endpointObjReceive->irpQueue;
                if(irp != NULL)
                {
                    /* This means we have a irp to process */
                    count = _DRV_USBHS_ProcessIRPFIFO(hDriver, endpointObjReceive,
                            0, USB_DATA_DIRECTION_HOST_TO_DEVICE, irp, &isDmaUsed, tryDma);

                    /* Clear the Rx Packet Ready Bit */
                    *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0x40;

                    /* Check if the irp is completed */
                    if((count < endpointObjReceive->maxPacketSize)
                            || (irp->nPendingBytes >= irp->size))
                    {
                        /* This means we received a short packet
                         * or the amount of data received is the amount
                         * that was expected */

                        if(irp->size < irp->nPendingBytes)
                        {
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                        }
                        else
                        {
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                        }

                        irp->size = irp->nPendingBytes;

                        endpointObjReceive->irpQueue = irp->next;
                        if(irp->callback)
                        {
                            irp->callback((USB_DEVICE_IRP *)irp);
                        }

                        /* We can now move to the handshake stage. Check
                         * if a TX IRP is available */

                        if(endpointObjTransmit->irpQueue != NULL)
                        {
                            hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST;
                            /* Set the data end bit */
                            *(PLIB_USBHS_GetEP0CSRAddress(usbID)) = 0x8;
                        }
                        else
                        {
                            hDriver->endpoint0State = DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT;
                        }
                    }
                }
            }
        }
    }

    /* Here we check if any other endpoint interrupts
     * have occurred. Start the endpoint mask with
     * endpoint 1 as the endpoint 0 is already serviced
     * */

    endpointMask = 0x2;
    for(iEndpoint = 1; iEndpoint < DRV_USBHS_ENDPOINTS_NUMBER; iEndpoint ++)
    {
        endpointObjReceive = hDriver->endpointTable + (2 * iEndpoint);
        endpointObjTransmit = hDriver->endpointTable + (2 * iEndpoint) + 1;

        if(endpointTXInterrupts & endpointMask)
        {
            /* Check if the endpoint is enabled */

            if(endpointObjTransmit->endpointState & DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED)
            {
                /* This means the endpoint is enabled. We got a TX interrupt 
                 * because a IRP was scheduled */

                txEPStatus = PLIB_USBHS_TxEPStatusGet(usbID, iEndpoint);

                if(txEPStatus & USBHS_TXEP_SENTSTALL)
                {
                    /* This means a stall was sent. Clear this 
                     * the sent stall bit
                     */

                    PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_SENTSTALL);
                }

                if(endpointObjTransmit->irpQueue != NULL)
                {
                    /* We have a valid IRP at the head
                     * of the queue */

                    irp = endpointObjTransmit->irpQueue;
                    if(irp->status == USB_DEVICE_IRP_STATUS_ABORTED)
                    {
                        endpointObjTransmit->irpQueue = irp->next;
                        if(irp->callback != NULL)
                        {
                            /* Invoke the IRP */
                            irp->callback((USB_DEVICE_IRP *) irp);
                        }
                    }
                    else if(irp->nPendingBytes == 0)
                    {
                        /* We don't have any more bytes to send in this 
                         * IRP. Check if the ZLP flag is set else end 
                         * the IRP */

                        if((irp->flags & USB_DEVICE_IRP_FLAG_SEND_ZLP) != 0)
                        {
                            /* This means we have to send a ZLP. Clear the
                             * flag */

                            irp->flags &= ~USB_DEVICE_IRP_FLAG_SEND_ZLP;
                            PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_UNDERRUN);
                            PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, iEndpoint);
                        }
                        else
                        {

                            /* This means this IRP is processed */
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            endpointObjTransmit->irpQueue = irp->next;
                            if(irp->callback != NULL)
                            {
                                /* Invoke the IRP */
                                irp->callback((USB_DEVICE_IRP *) irp);
                            }


                            if(endpointObjTransmit->irpQueue != NULL)
                            {
                                /* This means we have one more IRP in the
                                 * queue */
                                
                                _DRV_USBHS_ProcessIRPFIFO(hDriver, 
                                        endpointObjTransmit, iEndpoint, 
                                        USB_DATA_DIRECTION_DEVICE_TO_HOST,
                                        endpointObjTransmit->irpQueue, &isDmaUsed, tryDma);

                                if(isDmaUsed == false)
                                {
                                    /* Set the TX packet Ready bit. Clear the
                                     * FIFO underun bit if it is set. */
                                    PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_UNDERRUN);
                                    PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, iEndpoint);
                                }
                            }
                        }
                    }/* End of nPending bytes equal to zero */
                    else
                    {
                        /* This means are more pending bytes in this
                         * IRP */

                        _DRV_USBHS_ProcessIRPFIFO(hDriver, 
                                endpointObjTransmit, iEndpoint, 
                                USB_DATA_DIRECTION_DEVICE_TO_HOST,
                                irp, &isDmaUsed, tryDma);

                        if(isDmaUsed == false)
                        {
                            /* Set the TX packet Ready bit. Clear the
                             * FIFO underun bit if it is set. */
                            PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_UNDERRUN);
                            PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, iEndpoint);
                        }

                    }

                }
            }
        }/* End of TX Endpoint Interrupt Handling */

        if(endpointRXInterrupts & endpointMask)
        {
            /* Check if the endpoint is enabled */

            if(endpointObjReceive->endpointState & DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED)
            {
                /* This means the endpoint is enabled */

                rxEPStatus = PLIB_USBHS_RxEPStatusGet(usbID, iEndpoint);
                if(rxEPStatus & USBHS_RXEP_SENTSTALL)
                {
                    /* This means a stall was sent. Clear this 
                     * the sent stall bit
                     */

                    PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint, USBHS_RXEP_SENTSTALL);
                }

                if(endpointObjReceive->irpQueue != NULL)
                {
                    /* This means we have a IRP to process */

                    if(rxEPStatus & USBHS_RXEP_PKTRDY)
                    {
                        /* This means the FIFO already contains data */

                        irp = endpointObjReceive->irpQueue;
                        
                        if(irp->status == USB_DEVICE_IRP_STATUS_ABORTED)
                        {
                            /* Move the IRP queue HEAD */
                            endpointObjReceive->irpQueue = irp->next;
                            
                            /* Flush the data from FIFO */
                            PLIB_USBHS_EndpointRxFIFOFlush(usbID, iEndpoint);
                            /* Clear the Rx Packet Ready Bit */
                            PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint, USBHS_RXEP_PKTRDY);

                            /* Callback with ABORT status */
                            if(irp->callback)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }
                        }
                        
                        else
                        {
                            
							{
								/* This means there is data in the FIFO and this data is presently not
								 * being unloaded. So we can go ahead and try to unload the FIFO.*/
								count = _DRV_USBHS_ProcessIRPFIFO(hDriver, 
                                    endpointObjReceive, iEndpoint,
                                    USB_DATA_DIRECTION_HOST_TO_DEVICE, irp, &isDmaUsed, tryDma);
							
								if(isDmaUsed == false)
								{
									/* Clear the Rx Packet Ready Bit */

									PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint, USBHS_RXEP_PKTRDY);

									/* Check if the irp is completed */

									if((count < endpointObjReceive->maxPacketSize)
											|| (irp->nPendingBytes >= irp->size))
									{
										/* This means we received a short packet
										 * or the amount of data received is the amount
										 * that was expected */

										if(irp->size > irp->nPendingBytes)
										{
											/* This means that we received less data
											 * than what was expected. */
											irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
										}
										else
										{
											irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
										}

										irp->size = irp->nPendingBytes;

										irp->nPendingBytes = 0;

										endpointObjReceive->irpQueue = irp->next;

										if(irp->callback)
										{
											irp->callback((USB_DEVICE_IRP *)irp);
										}
									}
								}
							}
                        }
                    }
                }
            } /* Check of RX endpoint state enabled */
        }/* End of receive interrupt */

        endpointMask = endpointMask << 1;

    }/* End of endpoint interrupt checking for loop */
}/* end of _DRV_USBHS_DEVICE_Tasks_ISR() */


// *****************************************************************************
/* Function:
      void _DRV_USBHS_DEVICE_Get_FreeDMAChannel(DRV_USBHS_OBJ * hDriver,
                                                bool endpointDir,
                                                uint8_t iEndpoint)

  Summary:
      Get a free DMA channel and use it, if available.

  Returns:  
      Return dma channel, if available;
      otherwise return 0
*/
uint8_t _DRV_USBHS_DEVICE_Get_FreeDMAChannel
(
    DRV_USBHS_OBJ * hDriver,
    bool endpointDir,
    uint8_t iEndpoint
)
{
    /* Start of local variables */
    uint8_t dmaChannel = 0;
    uint8_t channelCount = 0;
    /* end of local variables */
    
    for(channelCount = 1; channelCount < 9; channelCount++)
    {
        if((hDriver->gDrvUSBDMAPool[channelCount]).inUse == false)
        {
            /* Found Not used DMA Channel - Use this channel */
            dmaChannel = channelCount;
            hDriver->gDrvUSBDMAPool[dmaChannel].inUse = true;
            hDriver->gDrvUSBDMAPool[dmaChannel].endpointDir = endpointDir;
            hDriver->gDrvUSBDMAPool[dmaChannel].iEndpoint = iEndpoint;
            break;
        }
     }

     return dmaChannel;
}/* end of _DRV_USBHS_DEVICE_Get_FreeDMAChannel() */

// *****************************************************************************
/* Function:
      USB_ERROR DRV_USBHS_DEVICE_TestModeEnter
      (
          DRV_HANDLE handle,
          USB_TEST_MODE_SELECTORS testMode
      )

  Summary:
    Dynamic implementation of DRV_USBHS_DEVICE_TestModeEnter client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHS_DEVICE_TestModeEnter client
    interface function for USB device. Function set the test mode requested.
    Only 1 test mode can remain set at any given point of time.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_TestModeEnter(DRV_HANDLE handle, USB_TEST_MODE_SELECTORS testMode)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
	uint8_t testModeData[53] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
                                0xAA, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
                                0xEE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xBF, 0xDF,
                                0xEF, 0xF7, 0xFB, 0xFD, 0xFC, 0x7E, 0xBF, 0xDF,
								0xEF, 0xF7, 0xFB, 0xFD, 0x7E};
    /* end of local variables */

    if((handle == DRV_HANDLE_INVALID))
    {
		SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = (DRV_USBHS_OBJ *)handle;
    usbID = hDriver->usbID;
	
	if( USB_TEST_MODE_SELCTOR_TEST_PACKET == testMode )
	{
	    PLIB_USBHS_DeviceEPFIFOLoad(usbID, 0, &testModeData[0], 53);
	}
	
    if(PLIB_USBHS_TestModeEnter(usbID, (uint8_t)testMode) < 0)
    {
	    /* Failure */
        return USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
	    if( USB_TEST_MODE_SELCTOR_TEST_PACKET == testMode )
		{
	        PLIB_USBHS_EP0TxPktRdy(usbID);
		}
	    /* Success */
        return USB_ERROR_NONE;
    }

}/* end of DRV_USBHS_DEVICE_TestModeEnter() */

// *****************************************************************************
/* Function:
      USB_ERROR DRV_USBHS_DEVICE_TestModeExit
      (
          DRV_HANDLE handle,
          USB_TEST_MODE_SELECTORS testMode
      )

  Summary:
    Dynamic implementation of DRV_USBHS_DEVICE_TestModeExit client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHS_DEVICE_TestModeExit client
    interface function for USB device. Function clears the test mode set.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR DRV_USBHS_DEVICE_TestModeExit(DRV_HANDLE handle, USB_TEST_MODE_SELECTORS testMode)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
    /* end of local variables */

    if((handle == DRV_HANDLE_INVALID))
    {
		SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = (DRV_USBHS_OBJ *)handle;
    usbID = hDriver->usbID;
	
    if(PLIB_USBHS_TestModeExit(usbID, (uint8_t)testMode) < 0)
	{
	    return USB_ERROR_PARAMETER_INVALID;
	}
	/* Success */
    return USB_ERROR_NONE;

}/* end of DRV_USBHS_DEVICE_TestModeExit() */


void _DRV_USBHS_DEVICE_Tasks_ISR_USBDMA(DRV_USBHS_OBJ * hDriver)
{
    /* Start of local variables */
    uint8_t usbDMAInterrupts;
    uint8_t  iEndpoint;
    USBHS_MODULE_ID usbID;
    bool usbDMAError = false;
    uint8_t usbDMAChannel = 0;
    bool endpointDir;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObjReceive;
    DRV_USBHS_DEVICE_ENDPOINT_OBJ * endpointObjTransmit;
    USB_DEVICE_IRP_LOCAL * irp;
    
    /* end of local variables */

    if(!hDriver->isOpened)
    {
        /* We need a valid client */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver does not have a client");
        return;
    }

    if(hDriver->pEventCallBack == NULL)
    {
        /* We need a valid event handler */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver needs a event handler");
        return;
    }

    usbID = hDriver->usbID;

        /* Reading the interrupt flags will cause the
         * flags to get cleared */
        usbDMAInterrupts = PLIB_USBHS_DMAInterruptGet(usbID);

        for(usbDMAChannel = 1; usbDMAChannel < 9; usbDMAChannel++)
        {
            if((usbDMAInterrupts >> (usbDMAChannel - 1)) & 0x01)
            {
                /* Check if there is DMA error */
                usbDMAError =
                        PLIB_USBHS_DMAErrorGet(usbID, usbDMAChannel);
                if(usbDMAError == true)
                {
                    continue;
                }
                /* DMA interrupt set for this channel.
                 * Map the EP it is assigned for
                 */
                iEndpoint = hDriver->gDrvUSBDMAPool[usbDMAChannel].iEndpoint;
                endpointDir = hDriver->gDrvUSBDMAPool[usbDMAChannel].endpointDir;

                endpointObjReceive = hDriver->endpointTable + (2 * iEndpoint);
                endpointObjTransmit = hDriver->endpointTable + (2 * iEndpoint) + 1;

                

                /* Check the EP direction */
                if(endpointDir == USB_DATA_DIRECTION_HOST_TO_DEVICE)
                {
                    /* This is RX Endpoint */
                    irp = endpointObjReceive->irpQueue;
                }
                else
                {
                    /* This is TX Endpoint */
                    irp = endpointObjTransmit->irpQueue;
                    
                    if(irp->status == USB_DEVICE_IRP_STATUS_ABORTED)
                    {
                        PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_UNDERRUN);
                        PLIB_USBHS_EndpointTxFIFOFlush(usbID, iEndpoint);
                    }
                    else
                    {
                        /* Set the TX packet Ready bit. Clear the
                         * FIFO underrun bit if it is set. */
                        PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_UNDERRUN);
                        PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, iEndpoint);
                    }
                    hDriver->gDrvUSBDMAPool[usbDMAChannel].inUse = false;
                }

                /* Check if the irp is completed */
                
                if(endpointDir == USB_DATA_DIRECTION_DEVICE_TO_HOST)
                {
                    /* TX PATH */
                    /* No need to do anything */
                }

                else
                {
                    /* The IRP can be NULL if the endpoint was disabled. The
                     * endpoint can get disabled asynchronously if the alternate
                     * settings has changed or when ever the end point disable
                     * function is called. At the point, there could still be a
                     * IRP that the USB module has already started processing.
                     * */

                    if(irp != NULL && (irp->status != USB_DEVICE_IRP_STATUS_ABORTED))
                    {
                        /* RX PATH */
                        if(((hDriver->gDrvUSBDMAPool[usbDMAChannel].count) <
                                    (endpointObjReceive->maxPacketSize))
                                || (irp->nPendingBytes >= irp->size))
                        {
                            /* This means we received a short packet
                             * or the amount of data received is the amount
                             * that was expected */
                            if(irp->size > irp->nPendingBytes)
                            {
                                /* This means that we received less data
                                 * than what was expected. */
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                            }
                            else
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            }

                            irp->size = irp->nPendingBytes;
                            
                            irp->nPendingBytes = 0;

                            endpointObjReceive->irpQueue = irp->next;
                            
                            /* Return DMA channel back to the pool.*/
							hDriver->gDrvUSBDMAPool[usbDMAChannel].inUse = false;
							
                            /* Clear RXPktRDY bit in the corresponding register.
                               Doing this will ensure that if IRP submit is called
                               in the IRP callback, it will NOT receive a false
                               RX ready status */
                            PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint,
                                    USBHS_RXEP_PKTRDY);
                            if(irp->callback)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }
                        }
                        else
                        {
                            /* This means we have received max packet size of
                             data but the transfer is not yet complete. So we
                             clear the RX ready flag and release the DMA channel
                             to receive the next packet */
                            hDriver->gDrvUSBDMAPool[usbDMAChannel].inUse = false;
                            /* Clear RXPktRDY bit in the corresponding register */
                            PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint,
                                    USBHS_RXEP_PKTRDY);
                        }
                    }
                    if(irp!= NULL && (irp->status == USB_DEVICE_IRP_STATUS_ABORTED))
                    {
                        /* Release the DMA channel */
                        hDriver->gDrvUSBDMAPool[usbDMAChannel].inUse = false;
                        /* Move IRP queue HEAD */
                        endpointObjReceive->irpQueue = irp->next;
                        /* Clear RXPktRDY bit in the corresponding register */
                        PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint,
                                USBHS_RXEP_PKTRDY);
                        /* ABORT callback */
                        if(irp->callback)
                        {
                            irp->callback((USB_DEVICE_IRP *)irp);
                        }
                    }
                }
            }
    }
    
}/* end of _DRV_USBHS_DEVICE_Tasks_ISR() */

