/*******************************************************************************
  USB Device Driver Implementation of device mode operation routines

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhsv1_device.c

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
#include "driver/usb/usbhsv1/src/drv_usbhsv1_local.h"
#include "driver/usb/usbhsv1/drv_usbhsv1.h"
#include "system/debug/sys_debug.h"


/* Array of endpoint objects. Two objects per endpoint */
DRV_USBHSV1_DEVICE_ENDPOINT_OBJ gDrvUSBControlEndpoints[DRV_USBHSV1_INSTANCES_NUMBER] [2];

/* Array of endpoint objects. Two objects per endpoint */
DRV_USBHSV1_DEVICE_ENDPOINT_OBJ gDrvUSBNonControlEndpoints[DRV_USBHSV1_INSTANCES_NUMBER] [DRV_USBHSV1_ENDPOINTS_NUMBER - 1];

/* Array of device speeds. To map the speed as per bit values */

USB_SPEED gDrvUSBHSV1DeviceSpeedMap[4] =
{
    USB_SPEED_FULL,
    USB_SPEED_HIGH,
    USB_SPEED_LOW,
    USB_SPEED_ERROR

};

/* Array of endpoint types. To map the types as per bit values */

USB_TRANSFER_TYPE gDrvUSBHSV1DeviceTransferTypeMap[4] =
{
    USBHS_DEVEPTCFG_EPTYPE_CTRL_Val,
    USBHS_DEVEPTCFG_EPTYPE_ISO_Val,
    USBHS_DEVEPTCFG_EPTYPE_BLK_Val,
    USBHS_DEVEPTCFG_EPTYPE_INTRPT_Val

};

/******************************************************************************
 * This structure is a pointer to a set of USB Driver Device mode functions. 
 * This set is exported to the device layer when the device layer must use the
 * USB Controller.
 *****************************************************************************/

DRV_USB_DEVICE_INTERFACE gDrvUSBHSV1DeviceInterface =
{
    .open = DRV_USBHSV1_Open,
    .close = DRV_USBHSV1_Close,
    .eventHandlerSet = DRV_USBHSV1_ClientEventCallBackSet,
    .deviceAddressSet = DRV_USBHSV1_DEVICE_AddressSet,
    .deviceCurrentSpeedGet = DRV_USBHSV1_DEVICE_CurrentSpeedGet,
    .deviceSOFNumberGet = DRV_USBHSV1_DEVICE_SOFNumberGet,
    .deviceAttach = DRV_USBHSV1_DEVICE_Attach,
    .deviceDetach = DRV_USBHSV1_DEVICE_Detach,
    .deviceEndpointEnable = DRV_USBHSV1_DEVICE_EndpointEnable,
    .deviceEndpointDisable = DRV_USBHSV1_DEVICE_EndpointDisable,
    .deviceEndpointStall = DRV_USBHSV1_DEVICE_EndpointStall,
    .deviceEndpointStallClear = DRV_USBHSV1_DEVICE_EndpointStallClear,
    .deviceEndpointIsEnabled = DRV_USBHSV1_DEVICE_EndpointIsEnabled,
    .deviceEndpointIsStalled = DRV_USBHSV1_DEVICE_EndpointIsStalled,
    .deviceIRPSubmit = DRV_USBHSV1_DEVICE_IRPSubmit,
    .deviceIRPCancelAll = DRV_USBHSV1_DEVICE_IRPCancelAll,
    .deviceRemoteWakeupStop = DRV_USBHSV1_DEVICE_RemoteWakeupStop,
    .deviceRemoteWakeupStart = DRV_USBHSV1_DEVICE_RemoteWakeupStart,
    .deviceTestModeEnter = DRV_USBHSV1_DEVICE_TestModeEnter

};


// *****************************************************************************

/* Function:
    void _DRV_USBHSV1_DEVICE_Initialize
    (
        DRV_USBHSV1_OBJ * drvObj,
        SYS_MODULE_INDEX index
    )

  Summary:
    Dynamic implementation of _DRV_USBHSV1_DEVICE_Initialize client
    interface function.

  Description:
    This is the dynamic implementation of _DRV_USBHSV1_DEVICE_Initialize
    client interface initialization function for USB device.
    Function checks the input handle validity and on success initializes the
    driver object. It also freezes the clock, disables all DMA and Endpoint
    interrupts. The endpoint objects are updated with respective pointers

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

void _DRV_USBHSV1_DEVICE_Initialize
(
    DRV_USBHSV1_OBJ * drvObj,
    SYS_MODULE_INDEX index
)

{

    usbhs_registers_t * usbID;      /* USB instance pointer */
    uint8_t count;                  /* Loop Counter */

    /* Get the USB H/W instance pointer */
    usbID = drvObj->usbID;

    /* Point the objects for control endpoint. It is a bidirectional
     * endpoint, so only one object is needed */

    drvObj->deviceEndpointObj[0] = &gDrvUSBControlEndpoints[index][0];

    /* Point the objects for non control endpoints.
     * They are unidirectional endpoints, so multidimensional
     * array with one object per endpoint direction */

    for(count = 1; count < DRV_USBHSV1_ENDPOINTS_NUMBER ; count++)
    {
        drvObj->deviceEndpointObj[count] = &gDrvUSBNonControlEndpoints[index][count - 1];
    }

    /* Freeze USB clock */
    usbID->USBHS_CTRL.w |= USBHS_CTRL_FRZCLK_Msk;

    /* Disable all DMA interrupts */
    usbID->USBHS_DEVIDR.w = USBHS_DEVIDR_DMA__Msk;

    /* Disable all endpoint interrupts */
    usbID->USBHS_DEVIDR.w = USBHS_DEVIDR_PEP__Msk;

    /* In device mode endpoint 0 FIFO size is always 64.
     * So any FIFO allocation should start from 64. The
     * actual value stored in this variable is 64/8 */

    drvObj->consumedFIFOSize = 8;
    
}/* end of _DRV_USBHSV1_DEVICE_Initialize() */

// *****************************************************************************

/* Function:
      void DRV_USBHSV1_DEVICE_AddressSet(DRV_HANDLE handle, uint8_t address)

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_AddressSet client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_AddressSet
    client interface initialization function for USB device. Function checks
    the input handle validity and on success updates the Device General Control
    Register USBHS_DEVCTRL.UADD with the address value and enables the address.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

void DRV_USBHSV1_DEVICE_AddressSet
(
    DRV_HANDLE handle,
    uint8_t address
)

{

    usbhs_registers_t * usbID;          /* USB instance pointer */
    DRV_USBHSV1_OBJ * hDriver;          /* USB driver object pointer */

    /* Check if the handle is invalid, if so return without any action */
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_AddressSet().");
    }
    else
    {
        /* Set the device address */
        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;

        /*Disable the device address */
        usbID->USBHS_DEVCTRL.ADDEN = 0;

        /* Configure the device address */
        usbID->USBHS_DEVCTRL.UADD = address;

        /* Enable the device address */
        usbID->USBHS_DEVCTRL.ADDEN = 1;

    }

}/* end of DRV_USBHSV1_DEVICE_AddressSet() */

// *****************************************************************************

/* Function:
      USB_SPEED DRV_USBHSV1_DEVICE_CurrentSpeedGet(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_CurrentSpeedGet client
    interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_CurrentSpeedGet
    client interface initialization function for USB device.
    Function checks the input handle validity and on success returns value to
    indicate HIGH/FULL speed operation.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

USB_SPEED DRV_USBHSV1_DEVICE_CurrentSpeedGet(DRV_HANDLE handle)
{

    DRV_USBHSV1_OBJ * hDriver;              /* USB driver object pointer */
    USB_SPEED retVal = USB_SPEED_ERROR;     /* Return value */

    /* Check if the handle is invalid, if so return without any action */
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_CurrentSpeedGet().");
    }
    else
    {
        hDriver = (DRV_USBHSV1_OBJ *) handle;

        /* The current speed in contained in the
         * device speed member of the driver object */
         retVal = hDriver->deviceSpeed;
    }

    return (retVal);

}/* end of DRV_USBHSV1_DEVICE_CurrentSpeedGet() */

// *****************************************************************************

/* Function:
      void DRV_USBHSV1_DEVICE_RemoteWakeupStart(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_RemoteWakeupStart client
    interface function.

  Description:
    This is dynamic implementation of DRV_USBHSV1_DEVICE_RemoteWakeupStart
    client interface function for USB device.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

void DRV_USBHSV1_DEVICE_RemoteWakeupStart(DRV_HANDLE handle)
{

    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_RemoteWakeupStart().");
    }
    else
    {
        /* Commented till PLIB implementation is completed */
    }

}/* end of DRV_USBHSV1_DEVICE_RemoteWakeupStart() */

// *****************************************************************************

/* Function:
      void DRV_USBHSV1_DEVICE_RemoteWakeupStop(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_RemoteWakeupStop client
    interface function.

  Description:
    This is dynamic implementation of DRV_USBHSV1_DEVICE_RemoteWakeupStop
    client interface function for USB device.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

/* TODO:Sundar This is not yet completed */
void DRV_USBHSV1_DEVICE_RemoteWakeupStop(DRV_HANDLE handle)
{
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_RemoteWakeupStop().");
    }
    else
    {
        /* Commented till PLIB implementation is completed */
    }

}/* end of DRV_USBHSV1_DEVICE_RemoteWakeupStop() */

// *****************************************************************************

/* Function:
      void DRV_USBHSV1_DEVICE_Attach(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_Attach client
    interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_Attach
    client interface function for USB device.
    This function checks if the handle passed is valid and if so, performs the
    device attach operation. EOR, SUSP, WAKEUP, & SOF interrupts are enabled and
    EOR, WAKEUP, & SOF interrupts are cleared.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

void DRV_USBHSV1_DEVICE_Attach(DRV_HANDLE handle)
{

    usbhs_registers_t * usbID;                  /* USB instance pointer */
    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */

    /* Check if the handle is invalid, if so return without any action */
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_Attach().");
    }
    else
    {
        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;

        /* Unfreeze USB clock */
        usbID->USBHS_CTRL.w &= ~USBHS_CTRL_FRZCLK_Msk;

        /* Wait to unfreeze clock */
        while(!(usbID->USBHS_SR.CLKUSABLE));

        /* Attach the device */
        usbID->USBHS_DEVCTRL.DETACH = 0;

        /* Enable the End Of Reset, Suspend, SOF & Wakeup interrupts */
        usbID->USBHS_DEVIER.w = (USBHS_DEVIER_EORSTES_Msk | USBHS_DEVIER_SUSPES_Msk | USBHS_DEVIER_SOFES_Msk | USBHS_DEVIER_WAKEUPES_Msk);

        /* Clear the End Of Reset, SOF & Wakeup interrupts */
        usbID->USBHS_DEVICR.w = (USBHS_DEVICR_EORSTC_Msk | USBHS_DEVICR_SOFC_Msk | USBHS_DEVICR_WAKEUPC_Msk);

        /* Manually set the Suspend Interrupt */
        usbID->USBHS_DEVIFR.SUSPS = 1;

        /* Ack the Wakeup Interrupt */
        usbID->USBHS_DEVICR.WAKEUPC = 1;

        /* Freeze USB clock */
        usbID->USBHS_CTRL.w |= USBHS_CTRL_FRZCLK_Msk;

    }

}/* end of DRV_USBHSV1_DEVICE_Attach() */

// *****************************************************************************

/* Function:
      void DRV_USBHSV1_DEVICE_Detach(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_Detach client
    interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_Detach
    client interface function for USB device.
    This function checks if the passed handle is valid and if so, performs a
    device detach operation.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

void DRV_USBHSV1_DEVICE_Detach(DRV_HANDLE handle)
{

    usbhs_registers_t * usbID;                  /* USB instance pointer */
    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */

    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_Detach().");
    }
    else
    {
        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;

        /* Detach the device */
        usbID->USBHS_DEVCTRL.DETACH = 1;
    }

}/* end of DRV_USBHSV1_DEVICE_Detach() */

// *****************************************************************************

/* Function:
      uint16_t DRV_USBHSV1_DEVICE_SOFNumberGet(DRV_HANDLE client)

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_SOFNumberGet client
    interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_SOFNumberGet
    client interface function for USB device.
    Function checks the validity of the input arguments and on success returns
    the Frame count value.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

uint16_t DRV_USBHSV1_DEVICE_SOFNumberGet(DRV_HANDLE handle)
{
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Invalid Handle in DRV_USBHSV1_DEVICE_SOFNumberGet().");
    }

    return (0);

}/* end of DRV_USBHSV1_DEVICE_SOFNumberGet() */

// *****************************************************************************

/* Function:
    void _DRV_USBHSV1_DEVICE_IRPQueueFlush
    (
        DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj,
        USB_DEVICE_IRP_STATUS status
    )

  Summary:
    Dynamic implementation of _DRV_USBHSV1_DEVICE_IRPQueueFlush function.

  Description:
    This is the dynamic implementation of _DRV_USBHSV1_DEVICE_IRPQueueFlush
    function for USB device.
    Function scans for all the IRPs on the endpoint queue and cancels them all.

  Remarks:
    This is a local function and should not be called directly by the
    application.
 */

void _DRV_USBHSV1_DEVICE_IRPQueueFlush
(
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj,
    USB_DEVICE_IRP_STATUS status
)

{

    USB_DEVICE_IRP_LOCAL * iterator;                  /* Local IRP object for iterations */

    /* Check if any IRPs are assigned on this endpoint and
     * abort them */

    if(endpointObj->irpQueue != NULL)
    {
        /* Cancel the IRP and deallocate driver IRP
         * objects */

        iterator = endpointObj->irpQueue;

        while (iterator != NULL)
        {
            iterator->status = status;
            if(iterator->callback != NULL)
            {
                iterator->callback((USB_DEVICE_IRP *) iterator);
            }
            iterator = iterator->next;
        }
    }

    /* Set the head pointer to NULL */
    endpointObj->irpQueue = NULL;

}/* end of _DRV_USBHSV1_DEVICE_IRPQueueFlush() */

// *****************************************************************************

/* Function:
    void _DRV_USBHSV1_DEVICE_EndpointObjectEnable
    (
        DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj,
        uint16_t endpointSize,
        USB_TRANSFER_TYPE endpointType,
        USB_DATA_DIRECTION endpointDirection
    )

  Summary:
    Dynamic implementation of _DRV_USBHSV1_DEVICE_EndpointObjectEnable
    function.

  Description:
    This is the dynamic implementation of
    _DRV_USBHSV1_DEVICE_EndpointObjectEnable function for USB device.
    Function populates the endpoint object data structure and sets it to
    enabled state.

  Remarks:
    This is a local function and should not be called directly by the
    application.
 */

void _DRV_USBHSV1_DEVICE_EndpointObjectEnable
(
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj,
    uint16_t endpointSize,
    USB_TRANSFER_TYPE endpointType,
    USB_DATA_DIRECTION endpointDirection
)

{
    /* This is a helper function */

    endpointObj->irpQueue = NULL;
    endpointObj->maxPacketSize = endpointSize;
    endpointObj->endpointType = endpointType;
    endpointObj->endpointState |= DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED;
    endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;
    endpointObj->endpointDirection = endpointDirection;

}/* end of _DRV_USBHSV1_DEVICE_EndpointObjectEnable() */

// *****************************************************************************

/* Function:
    USB_ERROR DRV_USBHSV1_DEVICE_EndpointEnable
    (
        DRV_HANDLE handle,
        USB_ENDPOINT endpointAndDirection,
        USB_TRANSFER_TYPE endpointType,
        uint16_t endpointSize
    )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_EndpointEnable client
    interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_EndpointEnable
    client interface function for USB device.
    Function enables the specified endpoint.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

USB_ERROR DRV_USBHSV1_DEVICE_EndpointEnable
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection,
    USB_TRANSFER_TYPE endpointType,
    uint16_t endpointSize
)

{

    usbhs_registers_t * usbID;                  /* USB instance pointer */
    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */
    uint8_t direction;                          /* Endpoint Direction */
    uint8_t endpoint;                           /* Endpoint Number */
    uint8_t fifoSize = 0;                       /* FIFO size */
    uint16_t defaultEndpointSize = 8;           /* Default size of Endpoint */
    bool mutexLock = false;                     /* OSAL: for mutex lock */
    USB_ERROR retVal = USB_ERROR_NONE;          /* Return value */

    /* Endpoint object pointer */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj;

    /* Extract the Endpoint number and its direction */
    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USBHSV1_ENDPOINTS_NUMBER)
    {
        /* Endpoint number is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Unsupported endpoint in DRV_USBHSV1_DEVICE_EndpointEnable().");

        retVal = USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }
    else if(endpointSize < 8 || endpointSize > 1024)
    {
        /* Endpoint size is invalid, return with appropriate error message */
        retVal = USB_ERROR_HOST_ENDPOINT_INVALID;
    }
    else if(DRV_HANDLE_INVALID == handle)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_EndpointEnable().");

        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        /* Handle and Endpoint formation are valid, Enable endpoint */

        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;

        /* Find upper 2 power number of endpointSize */
        if(endpointSize)
        {
            while (defaultEndpointSize < endpointSize)
            {
                fifoSize++;
                defaultEndpointSize <<= 1;
            }
        }

        /* Get the endpoint object */
        endpointObj = hDriver->deviceEndpointObj[endpoint];

        if(endpoint == 0)
        {
            /* There are two endpoint objects for a control endpoint.
             * Enable the first endpoint object */

            _DRV_USBHSV1_DEVICE_EndpointObjectEnable
            (
                endpointObj, endpointSize, endpointType, USB_DATA_DIRECTION_HOST_TO_DEVICE
            );

            endpointObj++;

             /* Enable the second endpoint object */

            _DRV_USBHSV1_DEVICE_EndpointObjectEnable
            (
                endpointObj, endpointSize, endpointType, USB_DATA_DIRECTION_DEVICE_TO_HOST
            );

            /* Enable the control endpoint - Endpoint 0 */
            usbID->USBHS_DEVEPT.EPEN0 = 1;

            /* Configure the Endpoint 0 configuration register */
            usbID->USBHS_DEVEPTCFG[0].w =
            (
                USBHS_DEVEPTCFG_EPSIZE(fifoSize) |
                USBHS_DEVEPTCFG_EPTYPE(USB_TRANSFER_TYPE_CONTROL) |
                USBHS_DEVEPTCFG_EPBK(USBHS_DEVEPTCFG_EPBK_1_BANK) |
                USBHS_DEVEPTCFG_ALLOC_Msk
            );
            
            usbID->USBHS_DEVEPTIER[0].RSTDTS = 1;
			
			usbID->USBHS_DEVEPTIDR[0].STALLRQC = 1;
            

            if(!(usbID->USBHS_DEVEPTISR[0].CFGOK))
            {
                /* Endpoint configuration is not successful */
                retVal = USB_ERROR_ENDPOINT_NOT_CONFIGURED;
            }
            else
            {
                /* Endpoint configuration is successful */
                usbID->USBHS_DEVEPTIER[0].w = USBHS_DEVEPTIER_RXSTPES_Msk | USBHS_DEVEPTIER_RXOUTES_Msk;

                /* Enable Endpoint 0 Interrupts */
                usbID->USBHS_DEVIER.PEP_0 = 1;
            }
        }
        else
        {
            /* Enable the non-zero endpoint object */

            _DRV_USBHSV1_DEVICE_EndpointObjectEnable(endpointObj, endpointSize, endpointType, direction);

            /* Enable the endpoint */
            usbID->USBHS_DEVEPT.w |= USBHS_DEVEPT_EPEN(0x01 << endpoint);

            /* Set up the maxpacket size, fifo start address fifosize
             * and enable the interrupt. CLear the data toggle. */

            usbID->USBHS_DEVEPTCFG[endpoint].w =
            (
                USBHS_DEVEPTCFG_EPSIZE(fifoSize) | ((direction & 0x01) << USBHS_DEVEPTCFG_EPDIR_Pos) |
                USBHS_DEVEPTCFG_EPTYPE(gDrvUSBHSV1DeviceTransferTypeMap[endpointType]) |
                USBHS_DEVEPTCFG_EPBK(USBHS_DEVEPTCFG_EPBK_1_BANK) | USBHS_DEVEPTCFG_ALLOC_Msk
            );

            usbID->USBHS_DEVEPTIER[endpoint].RSTDTS = 1;
			
			usbID->USBHS_DEVEPTIDR[endpoint].STALLRQC = 1;
            
            if(!(usbID->USBHS_DEVEPTISR[endpoint].CFGOK))
            {
                /* Endpoint configuration is not successful */
                retVal = USB_ERROR_ENDPOINT_NOT_CONFIGURED;
            }
            else
            {
                /* Endpoint configuration is successful.
                 * Enable Endpoint Interrupts */

                if(direction == USB_DATA_DIRECTION_HOST_TO_DEVICE)
                {
                    usbID->USBHS_DEVEPTIER[endpoint].RXOUTES = 1;
                }

                usbID->USBHS_DEVIER.w = USBHS_DEVIER_PEP_(0x01 << endpoint);
            }

            if(mutexLock == true)
            {
                /* OSAL: Return mutex */
                if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
                {
                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Mutex unlock failed in DRV_USBHSV1_DEVICE_EndpointEnable().");
                }
            }
        }
    }

    return (retVal);

}/* end of DRV_USBHSV1_DEVICE_EndpointEnable() */

// *****************************************************************************

/* Function:
    USB_ERROR DRV_USBHSV1_DEVICE_EndpointDisable
    (
        DRV_HANDLE handle,
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_EndpointDisable client
    interface function.

  Description:
    This is dynamic implementation of DRV_USBHSV1_DEVICE_EndpointDisable
    client interface function for USB device.
    Function disables the specified endpoint.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

USB_ERROR DRV_USBHSV1_DEVICE_EndpointDisable
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)

{
    /* This routine disables the specified endpoint.
     * It does not check if there is any ongoing
     * communication on the bus through the endpoint
     */

    usbhs_registers_t * usbID;                  /* USB instance pointer */
    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */
    uint8_t endpoint;                           /* Endpoint Number */
    uint8_t count;                              /* Loop Counter */
    bool interruptWasEnabled = false;           /* To track interrupt state */
    bool mutexLock = false;                     /* OSAL: for mutex lock */
    USB_ERROR retVal = USB_ERROR_NONE;          /* Return value */

    /* Endpoint object pointer */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj;


    endpoint = endpointAndDirection & 0xF;

    if(
        (endpointAndDirection != DRV_USBHSV1_DEVICE_ENDPOINT_ALL) &&
        (endpoint >= DRV_USBHSV1_ENDPOINTS_NUMBER)
        )
    {
        /* The endpoint number must be either
         * _DRV_USBHSV1_DEVICE_ENDPOINT_ALL or a valid endpoint */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Unsupported endpoint in DRV_USBHSV1_DEVICE_EndpointDisable().");
        
        retVal = USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }
    else if(DRV_HANDLE_INVALID == handle)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_EndpointDisable().");

        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        /* Handle and Endpoint formation are valid, Enable endpoint */

        /* Get the driver object handle pointer and USB HW instance*/

        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;

        if(hDriver->isInInterruptContext == false)
        {
            interruptWasEnabled = SYS_INT_SourceDisable(hDriver->interruptSource);
        }

        /* If the endpointAndDirection is _DRV_USBHSV1_DEVICE_ENDPOINT_ALL
         * then this means that the DRV_USBHSV1_DEVICE_EndpointDisableAll()
         * function was called */

        if(endpointAndDirection == DRV_USBHSV1_DEVICE_ENDPOINT_ALL)
        {
            /* Disable all endpoints */

            usbID->USBHS_DEVEPT.w &= (~USBHS_DEVEPT_EPEN(0x3FF));

            endpointObj = hDriver->deviceEndpointObj[0];

            endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED;

            endpointObj++;

            endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED;

            endpointObj = hDriver->deviceEndpointObj[1];

            for(count = 1; count < DRV_USBHSV1_ENDPOINTS_NUMBER ; count++)
            {
                endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED;
                
                endpointObj++;
            }
        }
        else
        {
            if(endpoint == 0)
            {
                /* Disable a control endpoint and update the
                 * endpoint database. */

                endpointObj = hDriver->deviceEndpointObj[0];

                endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED;

                endpointObj++;

                endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED;

                /* Disable the Control Endpoint */
                usbID->USBHS_DEVEPT.EPEN0 = 0;
            }
            else
            {
                /* Disable a specific endpoint direction for non
                 * control endpoints. */                                    
                endpointObj = hDriver->deviceEndpointObj[endpoint];

                endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED;

                /* Disable the respective Endpoint  */
                usbID->USBHS_DEVEPT.w &= ~USBHS_DEVEPT_EPEN(0x01 << endpoint);
            }
        }

        /* Release interrupts and mutex */
        if(hDriver->isInInterruptContext == false)
        {
            if(interruptWasEnabled)
            {
                /* IF the interrupt was enabled when entering the routine
                 * re-enable it now */
                SYS_INT_SourceEnable(hDriver->interruptSource);
            }
        }
        if(mutexLock == true)
        {
            /* OSAL: Return mutex */
            if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Mutex unlock failed in DRV_USBHSV1_DEVICE_EndpointDisable().");
            }
        }
    }
    return (retVal);
}/* end of DRV_USBHSV1_DEVICE_EndpointDisable() */

// *****************************************************************************

/* Function:
    bool DRV_USBHSV1_DEVICE_EndpointIsEnabled
    (
        DRV_HANDLE handle,
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_EndpointIsEnabled client
    interface function.

  Description:
    This is the dynamic implementation of
    DRV_USBHSV1_DEVICE_EndpointIsEnabled client interface function for
    USB device.
    Function returns the state of specified endpoint(true\false) signifying
    whether the endpoint is enabled or not.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

bool DRV_USBHSV1_DEVICE_EndpointIsEnabled
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)

{

    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */
    uint8_t endpoint;                           /* Endpoint Number */
    bool retVal = true;                         /* Return value */

    /* Endpoint object pointer */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj;


    /* Extract the Endpoint number */
    endpoint = endpointAndDirection & 0xF;

    if(endpoint >= DRV_USBHSV1_ENDPOINTS_NUMBER)
    {
        /* Endpoint number is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Unsupported endpoint in DRV_USBHSV1_DEVICE_EndpointIsEnabled().");

        retVal = false;
    }
    else if(DRV_HANDLE_INVALID == handle)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_EndpointIsEnabled().");

        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        hDriver = (DRV_USBHSV1_OBJ *) handle;
        endpointObj = hDriver->deviceEndpointObj[endpoint];

        if((endpointObj->endpointState & DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED) == 0)
        {
            retVal = false;
        }
        else
        {
            /* return true */
        }
    }

    return (retVal);

}/* end of DRV_USBHSV1_DEVICE_EndpointIsEnabled() */


// *****************************************************************************

/* Function:
      USB_ERROR DRV_USBHSV1_DEVICE_EndpointStall
      (
        DRV_HANDLE client,
        USB_ENDPOINT endpointAndDirection
      )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_EndpointStall client
    interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_EndpointStall 
    client interface function for USB device.
    Function sets the STALL state of the specified endpoint.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

USB_ERROR DRV_USBHSV1_DEVICE_EndpointStall
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)

{

    usbhs_registers_t * usbID;                  /* USB instance pointer */
    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */
    uint8_t endpoint;                           /* Endpoint Number */
    bool interruptWasEnabled = false;           /* To track interrupt state */
    bool mutexLock = false;                     /* OSAL: for mutex lock */
    USB_ERROR retVal = USB_ERROR_NONE;          /* Return value */

    /* Endpoint object pointer */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj;

    /* Extract the Endpoint number and its direction */
    endpoint = endpointAndDirection & 0xF;

    if(endpoint >= DRV_USBHSV1_ENDPOINTS_NUMBER)
    {
        /* Endpoint number is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Unsupported endpoint in DRV_USBHSV1_DEVICE_EndpointStall().");

        retVal = USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }
    else if(DRV_HANDLE_INVALID == handle)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_EndpointStall().");

        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        /* Handle and Endpoint formation are valid, Stall the endpoint */

        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;

        /* Get the endpoint object */
        endpointObj = hDriver->deviceEndpointObj[endpoint];

        if(hDriver->isInInterruptContext == false)
        {
            interruptWasEnabled = 
                SYS_INT_SourceDisable(hDriver->interruptSource);
        }

        /* Stall the endpoint 0 */
        usbID->USBHS_DEVEPTIER[endpoint].STALLRQS = 1;

        if(endpoint == 0)
        {

            /* While stalling endpoint 0 we stall both directions */
            endpointObj->endpointState |= DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;

            _DRV_USBHSV1_DEVICE_IRPQueueFlush(endpointObj, USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT);

            endpointObj++;

            endpointObj->endpointState |= DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;

            _DRV_USBHSV1_DEVICE_IRPQueueFlush(endpointObj, USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT);
        }
        else
        {
            /* Stalling a non zero endpoint object */
            endpointObj->endpointState |= DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;

            _DRV_USBHSV1_DEVICE_IRPQueueFlush(endpointObj, USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT);
        }

        if(hDriver->isInInterruptContext == false)
        {
            if(interruptWasEnabled)
            {
                /* Enable the interrupt only if it was disabled */
                SYS_INT_SourceEnable(hDriver->interruptSource);
            }
        }
        if(mutexLock == true)
        {
            /* OSAL: Return mutex */
            if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Mutex unlock failed in DRV_USBHSV1_DEVICE_EndpointStall().");
            }
        }
    }

    return (retVal);
}/* end of DRV_USBHSV1_DEVICE_EndpointStall() */

// *****************************************************************************

/* Function:
      USB_ERROR DRV_USBHSV1_DEVICE_EndpointStallClear
      (
        DRV_HANDLE client,
        USB_ENDPOINT endpointAndDirection
      )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_EndpointStallClear client 
    interface function.

  Description:
    This is the dynamic implementation of 
    DRV_USBHSV1_DEVICE_EndpointStallClear client interface function for 
    USB device.  
    Function clears the STALL state of the specified endpoint and resets the 
    data toggle value.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

USB_ERROR DRV_USBHSV1_DEVICE_EndpointStallClear
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)

{

    usbhs_registers_t * usbID;                  /* USB instance pointer */
    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */
    uint8_t endpoint;                           /* Endpoint Number */
    bool interruptWasEnabled = false;           /* To track interrupt state */
    bool mutexLock = false;                     /* OSAL: for mutex lock */
    USB_ERROR retVal = USB_ERROR_NONE;          /* Return value */

    /* Endpoint object pointer */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj;

    endpoint = endpointAndDirection & 0xF;

    if(endpoint >= DRV_USBHSV1_ENDPOINTS_NUMBER)
    {
        /* Endpoint number is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Unsupported endpoint in DRV_USBHSV1_DEVICE_EndpointStallClear().");

        retVal = USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }
    else if(DRV_HANDLE_INVALID == handle)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_EndpointStallClear().");

        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        /* Handle and Endpoint formation are valid, clear the Stall */

        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;
        
        /* Get the endpoint object */
        endpointObj = hDriver->deviceEndpointObj[endpoint];

        /* Clear the stall request for the endpoint */
        usbID->USBHS_DEVEPTIDR[endpoint].STALLRQC = 1;
        
        usbID->USBHS_DEVEPTIER[endpoint].RSTDTS = 1;

        if(hDriver->isInInterruptContext == false)
        {
            interruptWasEnabled = SYS_INT_SourceDisable(hDriver->interruptSource);
        }

        if(endpoint == 0)
        {
            /* Update the endpoint object with stall Clear for endpoint 0 */
            endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;
                
            _DRV_USBHSV1_DEVICE_IRPQueueFlush(endpointObj, USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST);

            endpointObj++;

            endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;
                
            _DRV_USBHSV1_DEVICE_IRPQueueFlush(endpointObj, USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST);
            
        }
        else
        {
            
            /* Update the objects with stall Clear for non-zero endpoint */            
            endpointObj->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;
                
            _DRV_USBHSV1_DEVICE_IRPQueueFlush(endpointObj, USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST);
        }

        if(hDriver->isInInterruptContext == false)
        {
            if(interruptWasEnabled)
            {
                /* Enable the interrupt only if it was disabled */
                SYS_INT_SourceEnable(hDriver->interruptSource);
            }
        }
        if(mutexLock == true)
        {
            /* OSAL: Return mutex */
            if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Mutex unlock failed in DRV_USBHSV1_DEVICE_EndpointStallClear().");
            }
        }
    }

    return (retVal);
}/* end of DRV_USBHSV1_DEVICE_EndpointStallClear() */



// *****************************************************************************

/* Function:
    bool DRV_USBHSV1_DEVICE_EndpointIsStalled(DRV_HANDLE client,
                                        USB_ENDPOINT endpointAndDirection)

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_EndpointIsStalled client 
    interface function.

  Description:
    This is the dynamic implementation of 
    DRV_USBHSV1_DEVICE_EndpointIsStalled client interface function for 
    USB device.
    Function returns the state of specified endpoint(true\false) signifying
    whether the endpoint is STALLed or not.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

bool DRV_USBHSV1_DEVICE_EndpointIsStalled
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)

{
    /* Return the state of the endpoint */
    
    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */
    uint8_t endpoint;                           /* Endpoint Number */
    bool retVal = true;                         /* Return value */
    
    /* Endpoint object pointer */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj;
    
    if(DRV_HANDLE_INVALID == handle)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_EndpointIsStalled().");

        retVal = false;
    }
    else
    {
        endpoint = endpointAndDirection & 0xF;
        hDriver = (DRV_USBHSV1_OBJ *) handle;
        
        endpointObj = hDriver->deviceEndpointObj[endpoint];

        if((endpointObj->endpointState & DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED) == 0)
        {
            retVal = false;
        }
        else
        {
            /* return true */
        }
    }

    return (retVal);

}/* end of DRV_USBHSV1_DEVICE_EndpointIsStalled() */

// *****************************************************************************

/* Function:
    USB_ERROR DRV_USBHSV1_DEVICE_IRPSubmit
    (
        DRV_HANDLE handle,
        USB_ENDPOINT endpointAndDirection,
        USB_DEVICE_IRP * inputIRP
    )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_IRPSubmit client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_IRPSubmit 
    client interface function for USB device.
    Function checks the validity of the input arguments and on success adds the
    IRP to endpoint object queue linked list.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */
USB_ERROR DRV_USBHSV1_DEVICE_IRPSubmit
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection,
    USB_DEVICE_IRP * inputIRP
)
{

    usbhs_registers_t * usbID;                  /* USB instance pointer */
    DRV_USBHSV1_OBJ * hDriver;                  /* USB driver object pointer */
    USB_DEVICE_IRP_LOCAL * irp;                 /* Pointer to irp data structure */
    uint8_t direction;                      /* Endpoint Direction */
    uint8_t endpoint;                       /* Endpoint Number */
    uint16_t count;                         /* Loop Counter */
    uint8_t * ptr;                          /* pointer variable for local use */
    uint8_t * data;                         /* pointer to irp data array */
    uint16_t byteCount = 0;                 /* To hold received byte count */
    bool interruptWasEnabled = false;       /* To track interrupt state */
    USB_ERROR retVal = USB_ERROR_NONE;      /* Return value */

    /* Direction of Endpoint 0 Data Stage */
    uint16_t endpoint0DataStageDirection;
    
    /* Size of Endpoint 0 Data Stage */
    uint16_t endpoint0DataStageSize;
    
    /* Endpoint object pointer */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj;


    /* Get the endpoint number and direction */
    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);
    irp = (USB_DEVICE_IRP_LOCAL *) inputIRP;
    
    /* Check for a valid client */
    if(DRV_HANDLE_INVALID == handle)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Invalid handle in DRV_USBHSV1_DEVICE_IRPSubmit().");

        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else if(irp->status > USB_DEVICE_IRP_STATUS_SETUP)
    {
        /* This means that the IRP is in use */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Device IRP is already in use in DRV_USBHSV1_DEVICE_IRPSubmit().");
        
        retVal = USB_ERROR_DEVICE_IRP_IN_USE;
    }
    else if(endpoint >= DRV_USBHSV1_ENDPOINTS_NUMBER)
    {
        /* Endpoint number is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Endpoint is not provisioned for in DRV_USBHSV1_DEVICE_IRPSubmit().");
        
        retVal = USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }
    else
    {
        /* Get the driver object, the module ID and the endpoint and direction
         * specific BDT entry and the endpoint object. */

        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;
        endpointObj = hDriver->deviceEndpointObj[endpoint];

        if(endpoint == 0)
        {
            endpointObj += direction;
        }

        if((endpointObj->endpointState & DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED) == 0)
        {
            /* This means the endpoint is not enabled */
            retVal = USB_ERROR_ENDPOINT_NOT_CONFIGURED;
        }
        else
        {
            /* Check the size of the IRP. If the endpoint receives data from
             * the host, then IRP size must be multiple of maxPacketSize. If 
             * the send ZLP flag is set, then size must be multiple of 
             * endpoint size. */

            if((irp->size % endpointObj->maxPacketSize) == 0)
            {
                /* The IRP size is either 0 or a exact multiple of maxPacketSize */

                if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
                {
                    /* If the IRP size is an exact multiple of endpoint size and
                     * size is not 0 and if data complete flag is set,
                     * then we must send a ZLP */
                    if(((irp->flags & USB_DEVICE_IRP_FLAG_DATA_COMPLETE) == USB_DEVICE_IRP_FLAG_DATA_COMPLETE) && (irp->size != 0))
                    {
                        /* This means a ZLP should be sent after the data is sent */

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
                    retVal = USB_ERROR_PARAMETER_INVALID;
                }
            }

            /* Now we check if the interrupt context is active. If so the we dont need
             * to get a mutex or disable interrupts.  If this were being done in non
             * interrupt context, we, then we would disable the interrupt. In which case
             * we would get the mutex and then disable the interrupt */

            if(hDriver->isInInterruptContext == false)
            {
                /* Disable  the USB Interrupt as this is not called inside ISR */
                interruptWasEnabled = SYS_INT_SourceDisable(hDriver->interruptSource);

            }
            irp->next = NULL;

            /* Mark the IRP status as pending */
            irp->status = USB_DEVICE_IRP_STATUS_PENDING;

            /* If the data is moving from device to host then pending bytes is data
             * remaining to be sent to the host. If the data is moving from host to
             * device, nPendingBytes tracks the amount of data received so far */

            if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
            {
                irp->nPendingBytes = irp->size;
            }
            else
            {
                irp->nPendingBytes = 0;
            }

            /* Get the last object in the endpoint object IRP Queue */
            if(endpointObj->irpQueue == NULL)
            {
                /* Queue is empty */
                endpointObj->irpQueue = irp;
                irp->previous = NULL;

                /* Because this is the first IRP in the queue then we we must arm the
                 * endpoint entry in the BDT. */

                irp->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;

                if(endpoint == 0)
                {
                    if(direction == USB_DATA_DIRECTION_HOST_TO_DEVICE)
                    {
                        switch(hDriver->endpoint0State)
                        {
                            case DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST:

                                /* This is the default initialization value of Endpoint
                                 * 0.  In this state EPO is waiting for the setup packet
                                 * from the host. The IRP is already added to the queue.
                                 * When the host send the Setup packet, this IRP will be
                                 * processed in the interrupt. This means we dont have
                                 * to do anything in this state. */
                                break;

                            case DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_SETUP_IRP_FROM_CLIENT:
                                /* In this state, the driver has received the Setup
                                 * packet from the host, but was waiting for an IRP from
                                 * the client. The driver now has the IRP. We can unload
                                 * the setup packet into the IRP */

                                /* Get 8-bit access to endpoint 0 FIFO from USB RAM address
                                 * and copy the data into the IRP data buffer */

                                ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[0];

                                data = (uint8_t *)irp->data;

                                for(count = 0; count < 8; count++)
                                {
                                    *((uint8_t *)(data + count)) = *ptr++;
                                }

                                irp->nPendingBytes += count;

                                /* Clear the Setup Interrupt flag and also re-enable the
                                 * setup interrupt. */

                                usbID->USBHS_DEVEPTICR[0].RXSTPIC = 1;
                                usbID->USBHS_DEVEPTIER[0].RXSTPES = 1;

                                /* Analyze the setup packet. We need to check if the
                                 * control transfer contains a data stage and if so,
                                 * what is its direction. */

                                endpoint0DataStageSize = *((unsigned short int *) (data + 6));
                                endpoint0DataStageDirection = ((data[0] & 0x80) != 0);

                                if(endpoint0DataStageSize == 0)
                                {
                                    /* This means there is no data stage. We wait for
                                     * the client to submit the status IRP. */
                                    hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT;
                                }
                                else
                                {
                                    /* This means there is a data stage. Analyze the
                                     * direction. */

                                    if(endpoint0DataStageDirection == USB_DATA_DIRECTION_DEVICE_TO_HOST)
                                    {
                                        /* If data is moving from device to host, then
                                         * we wait for the client to submit an transmit
                                         * IRP */

                                        hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT;
                                    }
                                    else
                                    {
                                        /* Data is moving from host to device. We wait
                                         * for the host to send the data. */
                                        hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_RX_DATA_STAGE_FROM_HOST;
                                    }
                                }

                                /* Update the IRP queue so that the client can submit an
                                 * IRP in the IRP callback. */
                                endpointObj->irpQueue = irp->next;

                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                                /* IRP callback */
                                if(irp->callback != NULL)
                                {
                                    irp->callback((USB_DEVICE_IRP *)irp);
                                }

                                break;

                            case DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_RX_DATA_STAGE_FROM_HOST:
                            case DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_COMPLETE:

                                /* In both of these states, an IRP has been submitted,
                                 * and the driver is now waiting for data from the host.
                                 * When the data arrives, the IRPs will be processed in
                                 * the interrupts. */

                                break;

                            case DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_IRP_FROM_CLIENT:

                                /* In this state, the host sent a data stage packet, an
                                    * interrupt occurred but there was no RX data stage
                                    * IRP. The RX IRP is now being submitted. We should
                                    * unload the fifo. */

                                byteCount = usbID->USBHS_DEVEPTISR[0].BYCT;

                                data = (uint8_t *) irp->data;

					            /* Get 8-bit access to endpoint 0 FIFO from USB RAM address */
					            ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[0];

					            data = (uint8_t *)&data[irp->nPendingBytes];

					            if((irp->nPendingBytes + byteCount) > irp->size)
					            {
						            /* This is not acceptable as it may corrupt the ram location */
                                    byteCount = irp->size - irp->nPendingBytes;
					            }
					            else
					            {
						            for(count = 0; count < byteCount; count++)
						            {
							            *((uint8_t *)(data + count)) = *ptr++;
						            }

                                    /* Update the pending byte count */
                                    irp->nPendingBytes += byteCount;

                                    if(irp->nPendingBytes >= irp->size)
                                    {
                                        /* This means we have received all the data that
                                         * we were supposed to receive */
                                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                                        /* Change endpoint state to waiting to the
                                         * status stage */
                                        hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_COMPLETE;

                                        /* Clear and re-enable the interrupt */
                                        usbID->USBHS_DEVEPTICR[0].RXOUTIC = 1;

                                        usbID->USBHS_DEVEPTIER[0].RXOUTES = 1;

                                        /* Update the queue, update irp-size to indicate
                                         * how much data was received from the host. */
                                        irp->size = irp->nPendingBytes;

                                        endpointObj->irpQueue = irp->next;

                                        if(irp->callback != NULL)
                                        {
                                            irp->callback((USB_DEVICE_IRP *)irp);
                                        }
                                    }
                                    else
                                    {
                                        //Chk state alone is different - others same as top.
                                        if(byteCount < endpointObj->maxPacketSize)
                                        {
                                            /* This means we received a short packet. We
                                             * should end the transfer. */
                                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;

                                            /* The data stage is complete. We now wait
                                             * for the status stage. */
                                            hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_COMPLETE;

                                            /* Clear and enable the interrupt. */
                                            usbID->USBHS_DEVEPTICR[0].RXOUTIC = 1;

                                            usbID->USBHS_DEVEPTIER[0].RXOUTES = 1;

                                            irp->size = irp->nPendingBytes;

                                            endpointObj->irpQueue = irp->next;

								            if(irp->callback != NULL)
								            {
    								            irp->callback((USB_DEVICE_IRP *)irp);
								            }
							            }
							            else
							            {
                                            usbID->USBHS_DEVEPTICR[0].RXOUTIC = 1;

								            usbID->USBHS_DEVEPTIER[0].RXOUTES = 1;
							            }

                                    }
                                }
                                break;

                            case DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_IRP_FROM_CLIENT:

                                /* This means the host has already sent an RX status
                                 * stage but there was not IRP to receive this. We have
                                 * the IRP now. We change the EP0 state to waiting for
                                 * the next setup from the host. */

                                hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;

                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                                usbID->USBHS_DEVEPTICR[0].RXOUTIC = 1;

                                usbID->USBHS_DEVEPTIER[0].RXOUTES = 1;

                                endpointObj->irpQueue = irp->next;

                                if(irp->callback != NULL)
                                {
                                    irp->callback((USB_DEVICE_IRP *)irp);
                                }

                                break;

                            default:

                                break;
                        }
                    }
                    else
                    {
                        /* This means data is moving device to host. */
                        switch(hDriver->endpoint0State)
                        {
                            case DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT:

                                /* Driver is waiting for an IRP from the client and has
                                 * received it. Determine the transaction size. */

                                if(irp->nPendingBytes < endpointObj->maxPacketSize)
                                {
                                    /* This is the last transaction in the transfer. */
                                    byteCount = irp->nPendingBytes;
                                }
                                else
                                {
                                    /* This is first or a continuing transaction in the
                                     * transfer and the transaction size must be
                                     * maxPacketSize */

                                    byteCount = endpointObj->maxPacketSize;
                                }

                                /* Copy data to the FIFO */
                                ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[0];

                                data = (uint8_t *)irp->data;

                                for(count = 0; count < byteCount; count++)
                                {
                                    *ptr++ = *((uint8_t *)(data + count));
                                }

                                irp->nPendingBytes -= byteCount;

                                hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS;

                                /* Clear the flag and enable the interrupt. The rest of
                                 * the IRP should really get processed in the ISR.
                                 * */
                                usbID->USBHS_DEVEPTICR[0].TXINIC = 1;
                                
                                usbID->USBHS_DEVEPTIDR[0].FIFOCONC = 1;

                                usbID->USBHS_DEVEPTIER[0].TXINES = 1;

                                break;

                            case DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT:

                                /* This means the driver is expecting the client to
                                 * submit a TX status stage IRP. */
                                hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_COMPLETE;

                                usbID->USBHS_DEVEPTICR[0].TXINIC = 1;

                                usbID->USBHS_DEVEPTIER[0].TXINES = 1;

                                break;

                            default:

                                break;

                        }
                    }
                }
                else
                {
                    /* Non zero endpoint irp */

                    if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
                    {
                        usbID->USBHS_DEVEPTICR[endpoint].TXINIC = 1;

                        /* Sending from Device to Host */
                        if(irp->nPendingBytes < endpointObj->maxPacketSize)
                        {
                            byteCount = irp->nPendingBytes;
                        }
                        else
                        {
                            byteCount = endpointObj->maxPacketSize;
                        }

                        /* Copy data to the FIFO */
                        ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[endpoint];

                        data = (uint8_t *)irp->data;

                        for(count = 0; count < byteCount; count++)
                        {
                            *ptr++ = *((uint8_t *)(data + count));
                        }

                        irp->nPendingBytes -= byteCount;

                        /* Enable the TXINI interrupt and clear the interrupt flag
                         * to initiate a Tx the packet */

                        usbID->USBHS_DEVEPTIDR[endpoint].FIFOCONC = 1;

                        usbID->USBHS_DEVEPTIER[endpoint].TXINES = 1;

                        /* The rest of the IRP processing takes place in ISR */
                    }
                    else
                    {
                        /* direction is Host to Device */
                        if(
                            (usbID->USBHS_DEVEPTISR[endpoint].RXOUTI) &&
                            (usbID->USBHS_DEVEPTIMR[endpoint].RXOUTE)
                          )
                        {
                            /* Data is already available in the FIFO */
                            byteCount = usbID->USBHS_DEVEPTISR[endpoint].BYCT;
                            
                            /* Get FIFO Address */
                            ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[endpoint];

                            if((irp->nPendingBytes + byteCount) > irp->size)
                            {
                                /* This is not acceptable as it may corrupt the ram location */
                                byteCount = irp->size - irp->nPendingBytes;
                            }

                            data = (uint8_t *)irp->data;

					        for(count = 0; count < byteCount; count++)
					        {
						        *((uint8_t *)(data + count)) = *ptr++;
					        }

                            /* Update the pending byte count */
                            irp->nPendingBytes += byteCount;

                            if((irp->nPendingBytes >= irp->size) || (byteCount < endpointObj->maxPacketSize))
                            {
                                if(byteCount < endpointObj->maxPacketSize)
                                {
                                    /* This means we have received a short packet */
                                    irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                                }
                                else
                                {
                                    /* This means we have received all the data that
                                    * we were supposed to receive */
                                    irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                                }
                                /* Update the queue, update irp-size to indicate
                                    * how much data was received from the host. */
                                irp->size = irp->nPendingBytes;

                                endpointObj->irpQueue = irp->next;

                                if(irp->callback != NULL)
                                {
                                    irp->callback((USB_DEVICE_IRP *)irp);
                                }
                            }
                            /* Clear and re-enable the interrupt */
                            usbID->USBHS_DEVEPTICR[endpoint].RXOUTIC = 1;

                            usbID->USBHS_DEVEPTIER[endpoint].RXOUTES = 1;
                        }
                        else
                        {
                            /* Host has not sent any data and IRP is already added
                             * to the queue. IRP will be processed in the ISR */
                        }
                    }/* End of non zero RX IRP submit */
                }/* End of non zero IRP submit */
            }
            else
            {
                /* This means we should surf the linked list to get to the last entry . */

                USB_DEVICE_IRP_LOCAL * iterator;
                iterator = endpointObj->irpQueue;
                while (iterator->next != NULL)
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
                    SYS_INT_SourceEnable(hDriver->interruptSource);
                }
            }        
        }
    }
   
    return (retVal);

}/* end of DRV_USBHSV1_DEVICE_IRPSubmit() */

// *****************************************************************************

/* Function:
    USB_ERROR DRV_USBHSV1_DEVICE_IRPCancelAll
    (
        DRV_HANDLE handle,
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_IRPCancelAll client 
    interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_IRPCancelAll 
    client interface function for USB device.
    Function checks the validity of the input arguments and on success cancels
    all the IRPs on the specific endpoint object queue.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

USB_ERROR DRV_USBHSV1_DEVICE_IRPCancelAll
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)

{

    DRV_USBHSV1_OBJ * hDriver;              /* USB driver object pointer */
    USB_ERROR retVal = USB_ERROR_NONE;      /* Return value */
    bool mutexLock = false;                 /* OSAL: for mutex lock */
    bool interruptWasEnabled = false;       /* To track interrupt state */
    uint8_t endpoint;                       /* Endpoint Number */

    /* Endpoint object pointer */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObj;


    /* Get the endpoint number and direction */
    endpoint = endpointAndDirection & 0xF;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_IRPCancelAll().");

        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else if(endpoint >= DRV_USBHSV1_ENDPOINTS_NUMBER)
    {
        /* Endpoint number is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Unsupported endpoint in DRV_USBHSV1_DEVICE_IRPCancelAll().");
        
        retVal = USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }
    else
    {
        
        hDriver = (DRV_USBHSV1_OBJ*) handle;
        
        /* Get the endpoint object */
        endpointObj = hDriver->deviceEndpointObj[endpoint];

        if(hDriver->isInInterruptContext == false)
        {
            interruptWasEnabled = SYS_INT_SourceDisable(hDriver->interruptSource);
        }

        /* Flush the endpoint */
        _DRV_USBHSV1_DEVICE_IRPQueueFlush
        (
            endpointObj, 
            USB_DEVICE_IRP_STATUS_ABORTED
        );
        
        if(hDriver->isInInterruptContext == false)
        {
            if(interruptWasEnabled)
            {
                /* Enable the interrupt only if it was disabled */
                SYS_INT_SourceEnable(hDriver->interruptSource);
            }
        }
        if(mutexLock == true)
        {
            /* OSAL: Return mutex */
            if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Mutex unlock failed in DRV_USBHSV1_DEVICE_IRPCancelAll().");
            }
        }
    }

    return (retVal);
    
}/* end of DRV_USBHSV1_DEVICE_IRPCancelAll() */

// *****************************************************************************

/* Function:
    USB_ERROR DRV_USBHSV1_DEVICE_IRPCancel
    (
        DRV_HANDLE client,
        USB_DEVICE_IRP * irp
    )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_IRPCancel client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_IRPCancel 
    client interface function for USB device.  Function checks the validity of 
    the input arguments and on success cancels  the specific IRP.
    An IRP that was in the queue but that has been processed yet will be
    canceled successfully and the IRP callback function will be called from
    this function with USB_DEVICE_IRP_STATUS_ABORTED status. The application 
    can release the data buffer memory used by the IRP when this callback 
    occurs. If the IRP was in progress (a transaction in on the bus) when the 
    cancel function was called, the IRP will be canceled only when an ongoing 
    or the next transaction has completed. The IRP callback function will then 
    be called in an interrupt context. The application should not release the
    related data buffer unless the IRP callback has occurred.

  Remarks:
    See drv_usbhsv1.h for usage information.
 */

USB_ERROR DRV_USBHSV1_DEVICE_IRPCancel
(
    DRV_HANDLE client,
    USB_DEVICE_IRP * irp
)

{

    DRV_USBHSV1_OBJ * hDriver;          /* USB driver object pointer */
    USB_DEVICE_IRP_LOCAL * irpToCancel;     /* Pointer to irp data structure */
    bool interruptWasEnabled = false;       /* To track interrupt state */
    bool mutexLock = false;                 /* OSAL: for mutex lock */
    USB_ERROR retVal = USB_ERROR_NONE;      /* Return value */

    
    irpToCancel = (USB_DEVICE_IRP_LOCAL *) irp;

    /* Check if the handle is valid */
    if(DRV_HANDLE_INVALID == client)
    {
        /* The handle is invalid, return with appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_IRPCancel().");
        
        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else if(irpToCancel == NULL)
    {
        /* IRP is NULL, send appropriate error message */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: IRP is invalid in DRV_USBHSV1_DEVICE_IRPCancel().");
        
        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        hDriver = ((DRV_USBHSV1_OBJ *) client);
        
        if(hDriver->isInInterruptContext == false)
        {
            interruptWasEnabled = SYS_INT_SourceDisable(hDriver->interruptSource);
        }

        if(irpToCancel->status <= USB_DEVICE_IRP_STATUS_COMPLETED_SHORT)
        {
            /* This IRP has either completed or has been aborted.*/
            retVal = USB_ERROR_PARAMETER_INVALID;
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
                    irpToCancel->callback((USB_DEVICE_IRP *) irpToCancel);
                }
            }
        }
        
        if(hDriver->isInInterruptContext == false)
        {
            if(interruptWasEnabled)
            {
                /* Enable the interrupt only if it was disabled */
                SYS_INT_SourceEnable(hDriver->interruptSource);
            }
        }

        if(mutexLock == true)
        {
            /* OSAL: Return mutex */
            if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Mutex unlock failed in DRV_USBHSV1_DEVICE_IRPCancel().");
            }
        }
    }
    
    return (retVal);

}/* End of DRV_USBHSV1_DEVICE_IRPCancel() */

// *****************************************************************************
/* Function:
      void _DRV_USBHSV1_DEVICE_Tasks_ISR(DRV_USBHSV1_OBJ * hDriver)

  Summary:
    Dynamic implementation of _DRV_USBHSV1_DEVICE_Tasks_ISR ISR handler 
    function.

  Description:
    This is the dynamic implementation of _DRV_USBHSV1_DEVICE_Tasks_ISR ISR 
    handler function for USB device.
    Function will get called automatically due to USB interrupts in interrupt 
    mode.
    In polling mode this function will be routinely called from USB driver
    DRV_USBHSV1_Tasks() function.
    This function performs necessary action based on the interrupt and clears 
    the interrupt after that. The USB device layer callback is called with the
    interrupt event details, if callback function is registered.

  Remarks:
    This is a local function and should not be called directly by the
    application.
 */

void _DRV_USBHSV1_DEVICE_Tasks_ISR(DRV_USBHSV1_OBJ * hDriver)
{
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObjReceive;
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObjTransmit;
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * endpointObjNonZero;
    usbhs_registers_t * usbID;
    USB_DEVICE_IRP_LOCAL * irp;
    uint16_t endpoint0DataStageSize;
    uint8_t endpointIndex;
    unsigned int endpoint0DataStageDirection;
    uint8_t *ptr;
    uint16_t count;
    uint8_t * data;
    uint32_t offset;
    uint32_t ep0Status = 0;
    uint32_t ep0MaskStatus = 0;
    uint32_t byteCount = 0;
    uint32_t epNonZeroStatus = 0;
    uint32_t epNonZeroMaskStatus = 0;

    if(!hDriver->isOpened)
    {
        /* We need a valid client */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver does not have a client in _DRV_USBHSV1_DEVICE_Tasks_ISR().");
    }
    else if(hDriver->pEventCallBack == NULL)
    {
        /* We need a valid event handler */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver needs a event handler in _DRV_USBHSV1_DEVICE_Tasks_ISR().");
    }

    usbID = hDriver->usbID;

    /* Check for SOF Interrupt Enable and SOF Interrupt Flag */
    if(usbID->USBHS_DEVISR.SOF && usbID->USBHS_DEVIMR.SOFE)
    {
        /* This means that there was a SOF. */
        usbID->USBHS_DEVICR.SOFC = 1;

        hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBHSV1_EVENT_SOF_DETECT, NULL);

    }

    /* Check for MSOF Interrupt Enable and MSOF Interrupt Flag */
    if(usbID->USBHS_DEVISR.MSOF && usbID->USBHS_DEVIMR.MSOFE)
    {
        /* Just acknowledge and Do nothing */
        usbID->USBHS_DEVICR.MSOFC = 1;

    }

    /* Check for Suspend Interrupt Enable and Suspend Interrupt Flag */
    if(usbID->USBHS_DEVISR.SUSP && usbID->USBHS_DEVIMR.SUSPE)
    {
        /* This means that the bus was SUSPENDED. */
        hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBHSV1_EVENT_IDLE_DETECT, NULL);

        /* Unfreeze USB clock */
        usbID->USBHS_CTRL.w &= ~USBHS_CTRL_FRZCLK_Msk;

        /* Disable Suspend Interrupt */
        usbID->USBHS_DEVIDR.SUSPEC = 1;

        /* Enable Wakeup Interrupt */
        usbID->USBHS_DEVIER.WAKEUPES = 1;

        /* Acknowledge the suspend interrupt */
        usbID->USBHS_DEVICR.SUSPC = 1;

        /* Freeze USB clock */
        usbID->USBHS_CTRL.w |= USBHS_CTRL_FRZCLK_Msk;

    }

    /* Check for Wakeup Interrupt Enable and Wakeup Interrupt Flag */
    if(usbID->USBHS_DEVISR.WAKEUP && usbID->USBHS_DEVIMR.WAKEUPE)
    {

        /* Unfreeze USB clock */
        usbID->USBHS_CTRL.w &= ~USBHS_CTRL_FRZCLK_Msk;

        /* Acknowledge the Wakeup interrupt */
        usbID->USBHS_DEVICR.WAKEUPC = 1;

        /* Disable Wakeup Interrupt */
        usbID->USBHS_DEVIDR.WAKEUPEC = 1;

        /* Enable Suspend Interrupt */
        usbID->USBHS_DEVIER.SUSPES = 1;

    }

    /* Check for End Of Reset Interrupt Enable and End Of Reset Interrupt Flag */
    if(usbID->USBHS_DEVISR.EORST && usbID->USBHS_DEVIMR.EORSTE)
    {
        /* This means that RESET signaling was detected
         * on the bus. This means the packet that we should
         * get on the bus for EP0 should be a setup packet */

        hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;

        hDriver->deviceSpeed = gDrvUSBHSV1DeviceSpeedMap[usbID->USBHS_SR.SPEED];

        hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBHSV1_EVENT_RESET_DETECT, NULL);

        /* Acknowledge the End of Resume interrupt */
        usbID->USBHS_DEVICR.EORSTC = 1;

        /* Acknowledge the Wakeup interrupt */
        usbID->USBHS_DEVICR.WAKEUPC = 1;

        /* Acknowledge the suspend interrupt */
        usbID->USBHS_DEVICR.SUSPC = 1;

        /* Enable Suspend Interrupt */
        usbID->USBHS_DEVIER.SUSPES = 1;

        /* Unfreeze USB clock */
        usbID->USBHS_CTRL.w &= ~USBHS_CTRL_FRZCLK_Msk;

    }

    if(usbID->USBHS_DEVISR.PEP_0)
    {
        /* This means this is EP0 interrupt. Read the endpoint 0 status
         * register. */
        ep0Status = usbID->USBHS_DEVEPTISR[0].w & 0x7FF7F3FFUL;
        ep0MaskStatus = usbID->USBHS_DEVEPTIMR[0].w & 0x000F70FFUL;

        /* Get the pointer to the endpoint 0 object */
        endpointObjReceive = hDriver->deviceEndpointObj[0];
        endpointObjTransmit = endpointObjReceive + 1;

        if(ep0Status & USBHS_DEVEPTISR_STALLEDI_Msk)
        {
            /* This means the endpoint stall was sent. We can clear the
             * the stall condition on the endpoint */

            /* Clear the stall request for the endpoint */
            usbID->USBHS_DEVEPTIDR[0].STALLRQC = 1;
            
            endpointObjReceive->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;
            endpointObjTransmit->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;
        }

        if(ep0MaskStatus & (ep0Status & USBHS_DEVEPTISR_RXSTPI_Msk))
        {
            /* This means we have received a setup packet. Let's clear the
             * stall condition on the endpoint. */
            endpointObjReceive->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;
            endpointObjTransmit->endpointState &= ~DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED;

            irp = endpointObjReceive->irpQueue;

            if(irp != NULL)
            {

                /* Get 8-bit access to endpoint 0 FIFO from USB RAM address */
                ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[0];

                data = (uint8_t *)irp->data;

                for(count = 0; count < 8; count++)
                {
                    *((uint8_t *)(data + count)) = *ptr++;
                }

                usbID->USBHS_DEVEPTICR[0].RXSTPIC = 1;

                endpoint0DataStageSize = *((unsigned short int *) (data + 6));
                endpoint0DataStageDirection = ((data[0] & 0x80) != 0);

                /* Indicate that this is a setup IRP */
                irp->status = USB_DEVICE_IRP_STATUS_SETUP;
                irp->size = 8;

                if(endpoint0DataStageSize == 0)
                {
                    hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT;
                }
                else
                {
                    if(endpoint0DataStageDirection == USB_DATA_DIRECTION_DEVICE_TO_HOST)
                    {
                        hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT;
                    }
                    else
                    {
                        hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_RX_DATA_STAGE_FROM_HOST;
                    }
                }

                endpointObjReceive->irpQueue = irp->next;

                if(irp->callback != NULL)
                {
                    irp->callback((USB_DEVICE_IRP *)irp);
                }

            }
            else
            {
                hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_SETUP_IRP_FROM_CLIENT;

                usbID->USBHS_DEVEPTIDR[0].RXSTPEC = 1;
            }
        }
        if(ep0MaskStatus & (ep0Status & USBHS_DEVEPTISR_TXINI_Msk))
        {
            irp = endpointObjTransmit->irpQueue;

            data = (uint8_t *) irp->data;

            if(hDriver->endpoint0State == DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_COMPLETE)
            {

                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;

                endpointObjTransmit->irpQueue = irp->next;

                irp->size = 0;

                if(irp->callback != NULL)
                {
                    irp->callback((USB_DEVICE_IRP *)irp);
                }

                usbID->USBHS_DEVEPTIDR[0].TXINEC = 1;
            }
            else
            {
                if(irp->nPendingBytes == 0)
                {
                    if(irp->flags & USB_DEVICE_IRP_FLAG_SEND_ZLP)
                    {
                        irp->flags &= ~USB_DEVICE_IRP_FLAG_SEND_ZLP;

                        usbID->USBHS_DEVEPTICR[0].TXINIC = 1;

                        usbID->USBHS_DEVEPTIER[0].TXINES = 1;
                    }
                    else
                    {
                        hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_COMPLETE;

                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                        endpointObjTransmit->irpQueue = irp->next;

                        if(irp->callback != NULL)
                        {
                            irp->callback((USB_DEVICE_IRP *)irp);
                        }

                        usbID->USBHS_DEVEPTIDR[0].TXINEC = 1;

                    }
                }
                else
                {
                    if(irp->nPendingBytes <= endpointObjTransmit->maxPacketSize)
                    {
                        byteCount = irp->nPendingBytes;
                    }
                    else
                    {
                        byteCount = endpointObjTransmit->maxPacketSize;
                    }

                    offset = irp->size - irp->nPendingBytes;

                    ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[0];

                    data = (uint8_t *)&data[offset];

                    for(count = 0; count < byteCount; count++)
                    {
                        *ptr++ = *((uint8_t *)(data + count));
                    }

                    irp->nPendingBytes -= byteCount;

                    usbID->USBHS_DEVEPTICR[0].TXINIC = 1;

                    usbID->USBHS_DEVEPTIER[0].TXINES = 1;

                }
            }
        }
        if(ep0MaskStatus & (ep0Status & USBHS_DEVEPTISR_RXOUTI_Msk))
        {
            /* This means we have received data from the host in the
             * data stage of the control transfer */
            irp = endpointObjReceive->irpQueue;

            if(hDriver->endpoint0State == DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_COMPLETE)
            {
                if(irp != NULL)
                {
                    irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                    hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;

                    usbID->USBHS_DEVEPTICR[0].RXOUTIC = 1;

                    endpointObjReceive->irpQueue = irp->next;

                    irp->size = 0;

                    if(irp->callback != NULL)
                    {
                        irp->callback((USB_DEVICE_IRP *)irp);
                    }
                }
                else
                {
                    hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_IRP_FROM_CLIENT;

                    usbID->USBHS_DEVEPTIDR[0].RXOUTEC = 1;

                }
            }
            else
            {
                if(irp == NULL)
                {
                    hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_IRP_FROM_CLIENT;

                    usbID->USBHS_DEVEPTIDR[0].RXOUTEC = 1;
                }
                else
                {
                    byteCount = usbID->USBHS_DEVEPTISR[0].BYCT;

                    data = (uint8_t *) irp->data;

                    /* Get 8-bit access to endpoint 0 FIFO from USB RAM address */
                    ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[0];

                    data = (uint8_t *)&data[irp->nPendingBytes];

                    if((irp->nPendingBytes + byteCount) > irp->size)
                    {
                        /* This is not acceptable as it may corrupt the ram location */
                        byteCount = irp->size - irp->nPendingBytes;
                    }
                    else
                    {
                        for(count = 0; count < byteCount; count++)
                        {
                            *((uint8_t *)(data + count)) = *ptr++;
                        }

                        irp->nPendingBytes += byteCount;

                        if((irp->nPendingBytes < irp->size) && (byteCount >= endpointObjReceive->maxPacketSize))
                        {
                            usbID->USBHS_DEVEPTICR[0].RXOUTIC = 1;
                        }
                        else
                        {
                            if(irp->nPendingBytes >= irp->size)
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            }
                            else
                            {
                                /* Short Packet */
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                            }

                            hDriver->endpoint0State = DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT;

                            usbID->USBHS_DEVEPTICR[0].RXOUTIC = 1;

                            endpointObjReceive->irpQueue = irp->next;

                            irp->size = irp->nPendingBytes;

                            if(irp->callback != NULL)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        for(endpointIndex = 1; endpointIndex < DRV_USBHSV1_ENDPOINTS_NUMBER; endpointIndex++)
        {
            /* This means this is non-EP0 interrupt. Read the endpoint status
             * register. */

            if(!(usbID->USBHS_DEVISR.w & USBHS_DEVISR_PEP_(0x01 << endpointIndex)))
            {
                continue;
            }

            epNonZeroStatus = usbID->USBHS_DEVEPTISR[endpointIndex].w & 0x7FF7F3FFUL;
            epNonZeroMaskStatus = usbID->USBHS_DEVEPTIMR[endpointIndex].w & 0x000F70FFUL;

            /* Get the pointer to the endpoint object */
            endpointObjNonZero = hDriver->deviceEndpointObj[endpointIndex];

            if(epNonZeroMaskStatus & (epNonZeroStatus & USBHS_DEVEPTISR_RXOUTI_Msk))
            {
                /* This means we have received RXOUTI interrupt */
                /* This means we have received data from the host */

                if(endpointObjNonZero->irpQueue == NULL)
                {
                    usbID->USBHS_DEVEPTIDR[endpointIndex].RXOUTEC = 1;
                }
                else
                {
                    irp = endpointObjNonZero->irpQueue;

                    byteCount = usbID->USBHS_DEVEPTISR[endpointIndex].BYCT;

                    data = (uint8_t *) irp->data;

                    /* Get 8-bit access to endpoint 0 FIFO from USB RAM address */
                    ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[endpointIndex];

                    data = (uint8_t *)&data[irp->nPendingBytes];

                    if((irp->nPendingBytes + byteCount) > irp->size)
                    {
                        byteCount = irp->size - irp->nPendingBytes;
                    }

                    for(count = 0; count < byteCount; count++)
                    {
                        *((uint8_t *)(data + count)) = *ptr++;
                    }

                    irp->nPendingBytes += byteCount;

                    if((irp->nPendingBytes < irp->size) && (byteCount >= endpointObjNonZero->maxPacketSize))
                    {
                        usbID->USBHS_DEVEPTICR[endpointIndex].RXOUTIC = 1;
                    }
                    else
                    {
                        if(irp->nPendingBytes >= irp->size)
                        {
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                        }
                        else
                        {
                            /* Short Packet */
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;

                            usbID->USBHS_DEVEPTICR[endpointIndex].SHORTPACKETC = 1;
                        }

                        endpointObjNonZero->irpQueue = irp->next;

                        irp->size = irp->nPendingBytes;

                        if(irp->callback != NULL)
                        {
                            irp->callback((USB_DEVICE_IRP *)irp);
                        }

                        usbID->USBHS_DEVEPTICR[endpointIndex].RXOUTIC = 1;

                        usbID->USBHS_DEVEPTIDR[endpointIndex].FIFOCONC = 1;
                    }
                }
            }
            else if(epNonZeroMaskStatus & (epNonZeroStatus & USBHS_DEVEPTISR_TXINI_Msk))
            {

                if(endpointObjNonZero->irpQueue != NULL)
                {

                    irp = endpointObjNonZero->irpQueue;

                    if((irp->nPendingBytes == 0) && (irp->flags & USB_DEVICE_IRP_FLAG_SEND_ZLP))
                    {
                        irp->flags &= ~USB_DEVICE_IRP_FLAG_SEND_ZLP;

                        usbID->USBHS_DEVEPTICR[endpointIndex].TXINIC = 1;

                        usbID->USBHS_DEVEPTIER[endpointIndex].TXINES = 1;
                    }
                    else if (irp->nPendingBytes != 0)
                    {
					
                        if(irp->nPendingBytes >= endpointObjNonZero->maxPacketSize)
                        {
                            byteCount = endpointObjNonZero->maxPacketSize;
                        }
                        else
                        {
                            byteCount = irp->nPendingBytes;
                        }

                        data = (uint8_t *) irp->data;

                        offset = irp->size - irp->nPendingBytes;
                        
                        ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[endpointIndex];

                        data = (uint8_t *)&data[offset];

                        for(count = 0; count < byteCount; count++)
                        {
                            *ptr++ = *((uint8_t *)(data + count));
                        }

                        irp->nPendingBytes -= byteCount;

                        usbID->USBHS_DEVEPTICR[endpointIndex].TXINIC = 1;

                        usbID->USBHS_DEVEPTIDR[endpointIndex].FIFOCONC = 1;

                        usbID->USBHS_DEVEPTIER[endpointIndex].TXINES = 1;
                    }
                    else 
                    {
                        if(!(irp->flags & USB_DEVICE_IRP_FLAG_SEND_ZLP))
                        {
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                            endpointObjNonZero->irpQueue = irp->next;

                            if(irp->callback != NULL)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }

                            if(endpointObjNonZero->irpQueue == NULL)
                            {
                                usbID->USBHS_DEVEPTIDR[endpointIndex].TXINEC = 1;
                            }
                            else
                            {
                                irp = endpointObjNonZero->irpQueue;

                                if(irp->nPendingBytes >= endpointObjNonZero->maxPacketSize)
                                {
                                    byteCount = irp->nPendingBytes;
                                }
                                else
                                {
                                    byteCount = endpointObjNonZero->maxPacketSize;
                                }

                                data = (uint8_t *) irp->data;

                                offset = irp->size - irp->nPendingBytes;

                                ptr = (uint8_t *) & ((volatile uint8_t (*)[0x8000])USBHSV1_RAM_ADDR)[endpointIndex];

                                data = (uint8_t *)&data[offset];

                                for(count = 0; count < byteCount; count++)
                                {
                                    *ptr++ = *((uint8_t *)(data + count));
                                }

                                irp->nPendingBytes -= byteCount;

                                usbID->USBHS_DEVEPTICR[endpointIndex].TXINIC = 1;

                                usbID->USBHS_DEVEPTIDR[endpointIndex].FIFOCONC = 1;

                                usbID->USBHS_DEVEPTIER[endpointIndex].TXINES = 1;

                            }
                        }                        
                    }
                }
            }
        }
    }
}

// *****************************************************************************

/* Function:
      void _DRV_USBHSV1_DEVICE_Get_FreeDMAChannel(DRV_USBHSV1_OBJ * hDriver,
                                                bool endpointDir,
                                                uint8_t iEndpoint)

  Summary:
      Get a free DMA channel and use it, if available.

  Returns:
      Return dma channel, if available;
      otherwise return 0
 */
uint8_t _DRV_USBHSV1_DEVICE_Get_FreeDMAChannel
(
    DRV_USBHSV1_OBJ * hDriver,
    bool endpointDir,
    uint8_t iEndpoint
)

{
    uint8_t dmaChannel = 0;
    uint8_t channelCount = 0;

    for (channelCount = 1; channelCount < 9; channelCount++)
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
}/* end of _DRV_USBHSV1_DEVICE_Get_FreeDMAChannel() */

// *****************************************************************************

/* Function:
      USB_ERROR DRV_USBHSV1_DEVICE_TestModeEnter
      (
          DRV_HANDLE handle,
          USB_TEST_MODE_SELECTORS testMode
      )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_TestModeEnter client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_TestModeEnter client
    interface function for USB device. Function set the test mode requested.
    Only 1 test mode can remain set at any given point of time.

  Remarks:
    See drv_usb.h for usage information.
 */

USB_ERROR DRV_USBHSV1_DEVICE_TestModeEnter
(
    DRV_HANDLE handle,
    USB_TEST_MODE_SELECTORS testMode
)

{
    DRV_USBHSV1_OBJ * hDriver;
    usbhs_registers_t * usbID;
    USB_ERROR retVal = USB_ERROR_NONE;

    if((handle == DRV_HANDLE_INVALID))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_TestModeEnter().");
        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;
        
        usbID = usbID;
        if(USB_TEST_MODE_SELCTOR_TEST_PACKET == testMode)
        {
            //TODO: PLIB_USBHSV1_DeviceEPFIFOLoad(usbID, 0, &testModeData[0], 53);
        }
        //TODO: if(PLIB_USBHSV1_TestModeEnter(usbID, (uint8_t)testMode) < 0)
        bool tempValue = 1;

        if(tempValue)
        {
            /* Failure */
            retVal = USB_ERROR_PARAMETER_INVALID;
        }
        else
        {
            if(USB_TEST_MODE_SELCTOR_TEST_PACKET == testMode)
            {
                //TODO: PLIB_USBHSV1_EP0TxPktRdy(usbID);
            }
            /* Success */
        }
    }

    return (retVal);

}/* end of DRV_USBHSV1_DEVICE_TestModeEnter() */

// *****************************************************************************

/* Function:
      USB_ERROR DRV_USBHSV1_DEVICE_TestModeExit
      (
          DRV_HANDLE handle,
          USB_TEST_MODE_SELECTORS testMode
      )

  Summary:
    Dynamic implementation of DRV_USBHSV1_DEVICE_TestModeExit client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_DEVICE_TestModeExit client
    interface function for USB device. Function clears the test mode set.

  Remarks:
    See drv_usb.h for usage information.
 */

USB_ERROR DRV_USBHSV1_DEVICE_TestModeExit
(
    DRV_HANDLE handle,
    USB_TEST_MODE_SELECTORS testMode
)

{
    DRV_USBHSV1_OBJ * hDriver;
    usbhs_registers_t * usbID;
    USB_ERROR retVal = USB_ERROR_NONE;


    if((handle == DRV_HANDLE_INVALID))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Device Driver: Driver Handle is invalid in DRV_USBHSV1_DEVICE_TestModeExit().");
        retVal = USB_ERROR_PARAMETER_INVALID;
    }
    else
    {
        hDriver = (DRV_USBHSV1_OBJ *) handle;
        usbID = hDriver->usbID;

        usbID = usbID;
        //TODO: if(PLIB_USBHSV1_TestModeExit(usbID, (uint8_t)testMode) < 0)
        {
            retVal = USB_ERROR_PARAMETER_INVALID;
        }
    }
    /* Success */
    return (retVal);

}/* end of DRV_USBHSV1_DEVICE_TestModeExit() */
