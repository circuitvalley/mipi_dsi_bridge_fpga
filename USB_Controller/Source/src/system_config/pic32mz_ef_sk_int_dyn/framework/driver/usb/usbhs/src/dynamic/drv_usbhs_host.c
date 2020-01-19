/*******************************************************************************
  USB Host Controller Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhs_host.c

  Summary:
    USB Host Driver Implementation for PIC32MZ.

  Description:
    This file implements the Host mode operation of the High Speed USB Driver.
    This file should be included in the application if USB Host mode operation
    is desired.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#include "system_config.h"
#include "driver/usb/drv_usb.h"
#include "driver/usb/usbhs/drv_usbhs.h"
#include "driver/usb/usbhs/src/drv_usbhs_local.h"
#include "usb/usb_host.h"
#include "usb/usb_host_client_driver.h"
#include "peripheral/usbhs/plib_usbhs.h"

/**********************************************************
 * This structure is a set of pointer to the USBFS driver
 * functions. It is provided to the host and device layer
 * as the interface to the driver.
 * *******************************************************/

DRV_USB_HOST_INTERFACE gDrvUSBHSHostInterface =
{
    .open = DRV_USBHS_Open,
    .close = DRV_USBHS_Close,
    .eventHandlerSet = DRV_USBHS_ClientEventCallBackSet,
    .hostIRPSubmit = DRV_USBHS_HOST_IRPSubmit,
    .hostIRPCancel = DRV_USBHS_HOST_IRPCancel,
    .hostPipeSetup = DRV_USBHS_HOST_PipeSetup,
    .hostPipeClose = DRV_USBHS_HOST_PipeClose,
    .hostEventsDisable = DRV_USBHS_HOST_EventsDisable,
    .hostEventsEnable = DRV_USBHS_HOST_EventsEnable,
    .rootHubInterface.rootHubPortInterface.hubPortReset = DRV_USBHS_HOST_ROOT_HUB_PortReset,
    .rootHubInterface.rootHubPortInterface.hubPortSpeedGet = DRV_USBHS_HOST_ROOT_HUB_PortSpeedGet,
    .rootHubInterface.rootHubPortInterface.hubPortResetIsComplete = DRV_USBHS_HOST_ROOT_HUB_PortResetIsComplete,
    .rootHubInterface.rootHubPortInterface.hubPortSuspend = DRV_USBHS_HOST_ROOT_HUB_PortSuspend,
    .rootHubInterface.rootHubPortInterface.hubPortResume = DRV_USBHS_HOST_ROOT_HUB_PortResume,
    .rootHubInterface.rootHubMaxCurrentGet = DRV_USBHS_HOST_ROOT_HUB_MaximumCurrentGet,
    .rootHubInterface.rootHubPortNumbersGet = DRV_USBHS_HOST_ROOT_HUB_PortNumbersGet,
    .rootHubInterface.rootHubSpeedGet = DRV_USBHS_HOST_ROOT_HUB_BusSpeedGet,
    .rootHubInterface.rootHubInitialize = DRV_USBHS_HOST_ROOT_HUB_Initialize,
    .rootHubInterface.rootHubOperationEnable = DRV_USBHS_HOST_ROOT_HUB_OperationEnable,
    .rootHubInterface.rootHubOperationIsEnabled = DRV_USBHS_HOST_ROOT_HUB_OperationIsEnabled,
};

/*****************************************
 * Pool of pipe objects that is used by
 * all driver instances.
 *****************************************/
DRV_USBHS_HOST_PIPE_OBJ gDrvUSBHostPipeObj[DRV_USBHS_HOST_PIPES_NUMBER];

/****************************************
 * The driver object
 ****************************************/
extern DRV_USBHS_OBJ gDrvUSBObj[];

// ****************************************************************************
// ****************************************************************************
// Local Functions
// ****************************************************************************
// ****************************************************************************

// *****************************************************************************
/* Function:
    void _DRV_USBHS_HOST_TimerCallback
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

void _DRV_USBHS_HOST_TimerCallback(uint32_t context, uint32_t currtick)
{
    DRV_USBHS_OBJ * hDriver = ((DRV_USBHS_OBJ *)context);
    hDriver->timerExpired = true;
}


void _DRV_USBHS_HOST_AttachDetachStateMachine (DRV_USBHS_OBJ * hDriver)
{
    /* In the host mode, we perform attach de-bouncing */
    
    bool interruptWasEnabled;

    switch(hDriver->attachState)
    {
        case DRV_USBHS_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH:
            /* If no device is attached, then there is nothing to do
             * If device is attached, then move to next state */
            if(hDriver->deviceAttached)
            {   
                /* Explicitely try to delete the Timer object if it is not
                 * running */
                SYS_TMR_ObjectDelete(hDriver->timerHandle);
                
                hDriver->timerExpired = false;
                
                /* Start the de-bouncing timer */
                hDriver->timerHandle = SYS_TMR_CallbackSingle( DRV_USBHS_HOST_ATTACH_DEBOUNCE_DURATION,
                        (uintptr_t ) hDriver, _DRV_USBHS_HOST_TimerCallback);
                
                if(hDriver->timerHandle != SYS_TMR_HANDLE_INVALID)
                {
                    /* The de-bounce timer has started */
                    hDriver->attachState = DRV_USBHS_HOST_ATTACH_STATE_DEBOUNCING;
                }
            }
            else
            {
                /* If a device is not attached. We need to check if the device
                 * has slipped into OTG B device mode. The USB module does this
                 * sometimes. While operating in host mode, the USB module
                 * should always be a OTG A device.  */
                
                if(PLIB_USBHS_IsBDevice(hDriver->usbID) && 
                        (PLIB_USBHS_VBUSLevelGet(hDriver->usbID) == USBHS_VBUS_VALID))
                {
                    /* This means the module is not operating as host mode.
                     * We should restart the session */
                    hDriver->attachState = DRV_USBHS_HOST_ATTACH_STATE_RESTART_SESSION;
                }
            }
            break;
            
        case DRV_USBHS_HOST_ATTACH_STATE_RESTART_SESSION:
            
            /* We reach this state if the USB module has slipped out of host
             * mode and must be placed back into host mode. Restart the session
             */
            
            PLIB_USBHS_SessionEnable(hDriver->usbID);
            hDriver->attachState = DRV_USBHS_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH;
          
            break;

        case DRV_USBHS_HOST_ATTACH_STATE_DEBOUNCING:

            /* Check if the de-bounce delay has completed. If so then
             * check if the device is still attached. */
            if(hDriver->timerExpired)
            {
                hDriver->timerExpired = false;
                /* The de-bounce period has ended. Is the device
                 * still attached. Disable the driver interrupt as
                 * we do not want this section to be interrupted. */
                interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);

                if(hDriver->deviceAttached)
                {
                    /* Yes the device is still attached. Enumerate
                     * this device. usbHostDeviceInfo is the ID of
                     * this root hub. */
                    hDriver->attachedDeviceObjHandle = USB_HOST_DeviceEnumerate(hDriver->usbHostDeviceInfo, 0);
                    hDriver->attachState = DRV_USBHS_HOST_ATTACH_STATE_READY;

                }
                else
                {
                    /* The device is not attached any more. This was
                     * a false attach */
                    hDriver->attachState = DRV_USBHS_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH;
                }

                if(interruptWasEnabled)
                {
                    /* Re-enable the interrupt if it was originally
                     * enabled. */
                    _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
                }

            }
            break;

        case DRV_USBHS_HOST_ATTACH_STATE_READY:

            /* De-bouncing is done and device ready. We can check
             * here if the device is detached */
            if(!hDriver->deviceAttached)
            {
                /* Device is not attached */
                hDriver->attachState = DRV_USBHS_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH;
            }
            
            break;

        default:
            break;
    }
}

void _DRV_USBHS_HOST_ResetStateMachine(DRV_USBHS_OBJ * hDriver)
{
    /* Check if reset is needed */
    switch(hDriver->resetState)
    {
        case DRV_USBHS_HOST_RESET_STATE_NO_RESET:

            /* No reset signaling is request */
            break;

        case DRV_USBHS_HOST_RESET_STATE_START:

            /* We should start reset signaling. First try to get a
             * timer */
            hDriver->timerHandle = SYS_TMR_CallbackSingle( DRV_USBHS_HOST_RESET_DURATION,
                    (uintptr_t ) hDriver, _DRV_USBHS_HOST_TimerCallback);
            
            if(hDriver->timerHandle != SYS_TMR_HANDLE_INVALID)
            {
                PLIB_USBHS_ResetEnable(hDriver->usbID);
                hDriver->resetState = DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_COMPLETE;
            }
            break;

        case DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_COMPLETE:

            /* Check if the reset duration has completed */
            if(hDriver->timerExpired)
            {
                hDriver->timerExpired = false;
                /* The reset has completed */
                PLIB_USBHS_ResetDisable(hDriver->usbID);
                
                /* Though we de assert Reset signaling above but the isResetting
                 flag is not set to false here. In this way we make sure that
                 USB Host layer does not start submitting transfer requests when
                 the USB module is not completely ready for enumeration.
                 The USB module is made to enter Suspend mode after Bus reset
                 de assert and is resumed after some time. In Suspend state
                 the USB lines are in forced idle state and therefore does not
                 detect any noise in line state lines from USB PHY.
                 By this way we negate devices which have quick pull up on 
                 D+/D- lines. */ 
                
                /* Enable Suspend */
                PLIB_USBHS_SuspendEnable(hDriver->usbID);
                
                hDriver->timerHandle = SYS_TMR_CallbackSingle( 100,(uintptr_t ) hDriver,
                        _DRV_USBHS_HOST_TimerCallback);
                if(hDriver->timerHandle != SYS_TMR_HANDLE_INVALID)
                {
                    hDriver->resetState = DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_SUSPEND_COMPLETE;
                }
            }
            
            break;
            
        case DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_SUSPEND_COMPLETE:
            
            if(hDriver->timerExpired)
            {
                hDriver->timerExpired = false;
                /* Enable Resume. This will clear Suspend as well */
                PLIB_USBHS_ResumeEnable(hDriver->usbID);
                
                hDriver->timerHandle = SYS_TMR_CallbackSingle( 20, (uintptr_t ) hDriver, _DRV_USBHS_HOST_TimerCallback);
                if(hDriver->timerHandle != SYS_TMR_HANDLE_INVALID)
                {
                    hDriver->resetState = DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_RESUME_COMPLETE;
                }
            }
            break;
            
        case DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_RESUME_COMPLETE:
            
            if(hDriver->timerExpired)
            {
                hDriver->timerExpired = false;
                
                PLIB_USBHS_ResumeDisable(hDriver->usbID);
                hDriver->resetState = DRV_USBHS_HOST_RESET_STATE_NO_RESET;

                /* Clear the flag */
                hDriver->isResetting = false;

                /* Now that reset is complete, we can find out the
                 * speed of the attached device. */
                if(PLIB_USBHS_HighSpeedIsConnected(hDriver->usbID))
                {
                    /* This means the device attached at high speed */
                    hDriver->deviceSpeed = USB_SPEED_HIGH;
                }
                else if(PLIB_USBHS_FullOrHighSpeedIsConnected(hDriver->usbID))
                {
                    hDriver->deviceSpeed = USB_SPEED_FULL;
                }
                else
                {
                    hDriver->deviceSpeed = USB_SPEED_LOW;
                }
            }
            break;

        default:
            break;
    }
}

DRV_USBHS_HOST_PIPE_HANDLE _DRV_USBHS_HOST_FifoTableAllocate
(
    DRV_USBHS_OBJ * hDriver,
    DRV_USBHS_HOST_PIPE_OBJ * pipe,
    uint16_t wMaxPacketSize
)
{
    uint8_t *pFifoTable = (uint8_t *)(hDriver->gDrvUSBFifoTable);
    uint8_t *pFifoTableSecondary = (uint8_t *)(hDriver->gDrvUSBFifoTableSecondary);
    uint8_t indexFifoTable = 1;
    uint8_t noOfSlotsReq = 0;
    uint8_t noOfSlotsReqTemp = 0;
    uint8_t counter = 0;
    uint8_t byteLoop;
    uint8_t bitLoop;
    uint8_t byteStarting = 0;
    uint8_t bitStarting = 0;
    bool slotObtained = false;
    bool firstObtained = false;


    if(wMaxPacketSize <= 64)
    {
        /* For 9KB FIFO RAM we need total 144 bit slots where each bit slot
         * represents 64 bytes of FIFO RAM memory. So that translates to 18
         * byte slots.
         */
        while(indexFifoTable <= 18)
        {
            /* Search whole byte bit by bit */
            while(counter <= 7)
            {
                if((*pFifoTable & (0x01 << counter)) == 0)
                {
                    /* Free slot obtained. Now grab as per the max packet size */
                    /* 1 slot will represent 64 bytes in the fifo.
                     * So for endpoints having maxpacket size less than
                     * 64 also we will allocate 64 bytes memory EP FIFO
                     */
                    pipe->startingOffset = (((indexFifoTable - 1) * 8) + 1 + counter) * 8;

                    pipe->noOfSlots = 1;
                    *pFifoTable = (*pFifoTable | (0x01 << counter));
                    slotObtained = true;
                    break;
                }
            
                counter++;

            }/* end of (counter != 7) */

            if(slotObtained == true)
            {
                /* Exit from the main while() loop if FIFOTable obtained */
                break;
            }

            indexFifoTable++;
            pFifoTable++;
            
            /* Reset the counter to 0 for the next FIFOTable byte scan */
            counter = 0;

        }/* end of (indexFifoTable <= 18) */
    }
    else
    {
        /* Here noOfSlotsReq is number of continous bits required in the FIFO
           table */
        noOfSlotsReq = (wMaxPacketSize / 64);

        if((wMaxPacketSize % 64) != 0)
        {
            noOfSlotsReq++;
        }

        /* Keeping a backup of the number of slots required */
        noOfSlotsReqTemp = noOfSlotsReq;

        /* 
         * Copy the latest content of the FIFOTable. The design idea is to
         * set the  gDrvUSBFifoTableSecondary data structure and then copy the
         * whole gDrvUSBFifoTableSecondary data to gDrvUSBFifoTable at the end.
         *
         * There is no chance that gDrvUSBFifoTable can get changed in the middle
         * as all are protected by Mutex lock on a per USBHCD driver instance level.
         */

        memcpy(hDriver->gDrvUSBFifoTableSecondary, hDriver->gDrvUSBFifoTable, sizeof(hDriver->gDrvUSBFifoTableSecondary));

        for(byteLoop = 1; byteLoop <=18; byteLoop++)
        {
            for(bitLoop = 0; bitLoop <=7; bitLoop++)
            {
                if((*pFifoTable & (0x01 << bitLoop)) == 0)
                {
                    if(firstObtained == false)
                    {
                        firstObtained = true;
                        byteStarting = byteLoop;
                        bitStarting = bitLoop;
                        *pFifoTableSecondary = (*pFifoTableSecondary | (0x01 << bitLoop));
                    }
                    else
                    {
                        *pFifoTableSecondary = (*pFifoTableSecondary | (0x01 << bitLoop));
                    }

                    noOfSlotsReqTemp--;
                }
                else
                {
                    firstObtained = false;
                    noOfSlotsReqTemp = noOfSlotsReq;
                    /*
                     * Reset the secondary array  to original array. This is
                     * required because it may happen that at initial we got some
                     * free slots but not enough slots as required. In this case
                     * we need to recopy again the original FifoTable data.
                     */
                    memcpy(hDriver->gDrvUSBFifoTableSecondary, hDriver->gDrvUSBFifoTable, sizeof(hDriver->gDrvUSBFifoTable));
                }

                if(noOfSlotsReqTemp == 0)
                {
                    /* Obtained all continuous slots */
                    pipe->startingOffset = (((byteStarting - 1) * 8) + 1 + bitStarting) * 8;
                    pipe->noOfSlots = noOfSlotsReq;
                    slotObtained = true;
                    break;
                }

            }/* end of for(bitLoop) */

            if(slotObtained == true)
            {
                memcpy(hDriver->gDrvUSBFifoTable, hDriver->gDrvUSBFifoTableSecondary, sizeof(hDriver->gDrvUSBFifoTable));
                /* Exit from the main for() loop if FIFOTable obtained */
                break;
            }

            pFifoTable++;
            pFifoTableSecondary++;

        }/* end of for(byteLoop) */
    }/*end of else(wmaxpacket > 64)*/

    if(slotObtained ==  false)
    {
        /* Slot NOT obtained */
        return DRV_USBHS_HOST_PIPE_HANDLE_INVALID;
    }
    else
    {
        return ((DRV_USBHS_HOST_PIPE_HANDLE)pipe);
    }

}/* end of _DRV_USBHS_HOST_FifoTableAllocate() */

void _DRV_USBHS_HOST_IRPTransmitFIFOLoad
(
    USBHS_MODULE_ID usbID, 
    USB_HOST_IRP_LOCAL * irp,
    uint8_t endpoint
)
{
    /* This function will copy data from the irp to the fifo
     * base on the number of bytes that were completed and
     * then trigger the transmit */

    uint8_t * data;
    unsigned int count, pendingBytes;
    DRV_USBHS_HOST_PIPE_OBJ * pipe = (DRV_USBHS_HOST_PIPE_OBJ *)(irp->pipe);
    uint8_t dmaChannelGrabbed = 0;

    /* Load the fifo */
    pendingBytes = irp->size - irp->completedBytes;
    count = (pendingBytes > pipe->endpointSize) ? pipe->endpointSize : pendingBytes;
    data = (uint8_t *)((uint8_t *)irp->data + irp->completedBytes);

    if(endpoint == 0)
    {
        irp->completedBytes += count;
        PLIB_USBHS_EndpointFIFOLoad(usbID, endpoint, data, count);
    }
    else
    {
        /* NON CONTROL transfer */
        dmaChannelGrabbed = _DRV_USBHS_HOST_GetFreeDMAChannel((DRV_USBHS_OBJ *)(pipe->hClient), USB_DATA_DIRECTION_HOST_TO_DEVICE, endpoint);
        if((dmaChannelGrabbed == 0))
        {
            /* NO DMA channel available. So do normal FIFO load */
            irp->completedBytes += count;
            PLIB_USBHS_EndpointFIFOLoad(usbID, endpoint, data, count);
        }
        else
        {
            /* DMA CHANNEL has been grabbed and DMA buffer address is OK
               (divisible by 4). So program the DMA registers now to start
               DMA operation
               */
            ((DRV_USBHS_OBJ *)(pipe->hClient))->gDrvUSBDMAPool[dmaChannelGrabbed] .count = count;

            PLIB_USBHS_DMAOperationEnable(usbID, endpoint, dmaChannelGrabbed, (void *)data, count, 0);
        }
    }
}/* end of _DRV_USBHS_HOST_IRPTransmitFIFOLoad() */

void _DRV_USBHS_HOST_IRPTransmitSetupPacket
(
    USBHS_MODULE_ID usbID,
    USB_HOST_IRP_LOCAL * irp
)
{
    /* This function will load the irp setup packet into endpoint 
     * 0 fifo and will then transmit the setup packet. */

    DRV_USBHS_HOST_PIPE_OBJ * pipe = (DRV_USBHS_HOST_PIPE_OBJ *)(irp->pipe);
    uint8_t * data = (uint8_t *)irp->setup;

    PLIB_USBHS_Endpoint0SetupPacketLoad(usbID, data, pipe->deviceAddress, pipe->hubAddress, pipe->hubPort, pipe->speed);

}/* end of _DRV_USBHS_HOST_IRPTransmitSetupPacket() */

unsigned int _DRV_USBHS_HOST_IRPReceiveFIFOUnload 
(
    USBHS_MODULE_ID usbID,
    USB_HOST_IRP_LOCAL * irp,
    uint8_t endpoint,
    bool * pisDMAUsed
)
{
    /* This function will recover the count of the received data/
     * and will then unload the endpoint fifo. Finally the rx packet
     * ready bit will be cleared */

    uint32_t  count;
    uint8_t * data;
    DRV_USBHS_HOST_PIPE_OBJ * pipe = (DRV_USBHS_HOST_PIPE_OBJ *)(irp->pipe);
    uint8_t dmaChannelGrabbed = 0;

    /* Copy the data from the FIFO0 to the application
     * buffer and then update the complete byte count
     * and clear the RX packet ready bit */

    data = (uint8_t *)((uint8_t *)irp->data + irp->completedBytes);
    if(endpoint == 0)
    {
        count = PLIB_USBHS_EndpointFIFOUnload(usbID, endpoint, data);
        irp->completedBytes += count;
        *pisDMAUsed = false;
    }
    else
    {
        dmaChannelGrabbed = _DRV_USBHS_HOST_GetFreeDMAChannel((DRV_USBHS_OBJ *)(pipe->hClient), USB_DATA_DIRECTION_DEVICE_TO_HOST, endpoint);
        if((dmaChannelGrabbed == 0))
        {
            /* NO DMA channel available. So do normal FIFO unload */
            count = PLIB_USBHS_EndpointFIFOUnload(usbID, endpoint, data);
            irp->completedBytes += count;
            *pisDMAUsed = false;
        }
        else
        {
            /* DMA CHANNEL has been grabbed and DMA buffer address is OK
             * (divisible by 4). So program the DMA registers now to start
             * DMA operation
             */

            /* Obtain the Recieved data count in bytes */
            count = (uint32_t) PLIB_USBHS_GetReceiveDataCount(usbID, endpoint);

            ((DRV_USBHS_OBJ *)(pipe->hClient))->gDrvUSBDMAPool[dmaChannelGrabbed] .count = count;

            PLIB_USBHS_DMAOperationEnable(usbID, endpoint,dmaChannelGrabbed, (void *)(data), count, 1);
            *pisDMAUsed = true;
        }
    }

    return (count);
}/* end of _DRV_USBHS_HOST_IRPReceiveFIFOUnload() */

void _DRV_USBHS_HOST_Initialize
(
    DRV_USBHS_OBJ * drvObj, 
    SYS_MODULE_INDEX index
  
)
{
    USBHS_MODULE_ID usbID = drvObj->usbID;
   
    /* No device attached */
    drvObj->deviceAttached = false;

    /* Initialize the device handle */
    drvObj->attachedDeviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;

    /* Disable all interrupts. Interrupts will be enabled when the root hub is
     * enabled */
    PLIB_USBHS_InterruptEnableSet(usbID, 0x0, 0x0, 0x0);

    /* Based on the speed that we are initializing
     * set up the HS Enable bit in the Power register */

    if(drvObj->operationSpeed == USB_SPEED_HIGH)
    {
        /* Enable high speed */
        PLIB_USBHS_HighSpeedEnable(usbID);
    }
    else
    {
        /* Enable full speed */
        PLIB_USBHS_HighSpeedDisable(usbID);
    }

    /* Initialize the host specific members in the driver object */
    drvObj->isResetting     = false;
    drvObj->usbHostDeviceInfo = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
   
}/* end of _DRV_USBHS_HOST_Initialize() */

USB_ERROR DRV_USBHS_HOST_IRPSubmit
(
    DRV_USBHS_HOST_PIPE_HANDLE  hPipe, 
    USB_HOST_IRP * inputIRP
)
{
    USB_HOST_IRP_LOCAL * irpIterator;
    DRV_USBHS_HOST_TRANSFER_GROUP * controlTransferGroup;
    bool interruptWasEnabled = false;
    unsigned int direction;
    uint8_t endpoint;

    USB_HOST_IRP_LOCAL * irp        = (USB_HOST_IRP_LOCAL *)inputIRP;
    DRV_USBHS_HOST_PIPE_OBJ * pipe = (DRV_USBHS_HOST_PIPE_OBJ *)(hPipe);
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;

    if((pipe == NULL) || (hPipe == (DRV_USBHS_HOST_PIPE_HANDLE_INVALID)))
    {
        /* This means an invalid pipe was specified. 
         * Return with an error */

        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Pipe handle is not valid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = (DRV_USBHS_OBJ *)(pipe->hClient);
    usbID = hDriver->usbID;
    controlTransferGroup = &hDriver->controlTransferGroup;

    /* Assign owner pipe */
    irp->pipe = hPipe;
    irp->status = USB_HOST_IRP_STATUS_PENDING;
    irp->tempState = DRV_USBHS_HOST_IRP_STATE_PROCESSING;
    endpoint = pipe->hostEndpoint;
    direction = (pipe->endpointAndDirection & 0x80) >> 7;

    /* We need to disable interrupts was the queue state
     * does not change asynchronously */

    if(!hDriver->isInInterruptContext)
    {
        // OSAL: Get Mutex
        if(OSAL_MUTEX_Lock(&(hDriver->mutexID), OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }
        interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);;

    }

    /* This needs to be done for all irp irrespective
     * of type or if there IRP is immediately processed */

    irp->next = NULL;
    irp->completedBytes = 0;
    irp->status = USB_HOST_IRP_STATUS_PENDING;

    if(pipe->irpQueueHead == NULL)
    {
        /* This means that there are no
         * IRPs on this pipe. We can add
         * this IRP directly */

        irp->previous = NULL;
        pipe->irpQueueHead = irp;

        if(pipe->pipeType == USB_TRANSFER_TYPE_CONTROL)
        {
            /* Set the initial stage of the IRP */
            irp->tempState = DRV_USBHS_HOST_IRP_STATE_SETUP_STAGE;

            /* We need to update the flags parameter of the IRP
             * to indicate the direction of the control transfer. */

            if(*((uint8_t*)(irp->setup)) & 0x80)
            {
                /* This means the data stage moves from device to 
                 * host. Set bit 15 of the flags parameter */

                irp->flags |= 0x8000;
            }
            else
            {
                /* This means the data stage moves from host to
                 * device. Clear bit 15 of the flags parameter. */

                irp->flags &= 0x7FFF;
            }

            /* We need to check if the endpoint 0 is free and if so
             * then start processing the IRP */

            if(controlTransferGroup->currentIRP == NULL)
            {
                /* This means that no IRPs are being processed
                 * So we should start the IRP processing. Else
                 * the IRP processing will start in interrupt.
                 * We start by copying the setup command */

                controlTransferGroup->currentIRP = irp;
                controlTransferGroup->currentPipe = pipe;
                irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;

                /* Send the setup packet to device */
                _DRV_USBHS_HOST_IRPTransmitSetupPacket(usbID, irp);
            }
        }
        else
        {
            /* For non control transfers, if this is the first
             * irp in the queue, then we can start the irp */

            irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;

            if(USB_DATA_DIRECTION_HOST_TO_DEVICE == direction)
            {
                /* Data is moving from host to device. We
                 * need to copy data into the FIFO and
                 * then and set the TX request bit. If the
                 * IRP size is greater than endpoint size then
                 * we must packetize. */

                _DRV_USBHS_HOST_IRPTransmitFIFOLoad(usbID, irp, endpoint);
            }
            else
            {
                /* Data is moving from device to host
                 * We need to set the Rx Packet Request
                 * bit */

                PLIB_USBHS_EndpointRxRequestEnable(usbID, endpoint);
            }
        }
    }
    else
    {
        /* We need to add the irp to the last irp
         * in the pipe queue (which is a linked list) */
        irpIterator = pipe->irpQueueHead;

        /* Find the last IRP in the linked list*/
        while(irpIterator->next != 0)
        {
            irpIterator = irpIterator->next;
        }

        /* Add the item to the last irp in the linked list */
        irpIterator->next = irp;
        irp->previous = irpIterator;
    }

    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
        }
        //OSAL: Return Mutex
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }

    return USB_ERROR_NONE;
}/* end of DRV_USBHS_HOST_IRPSubmit() */

void DRV_USBHS_HOST_IRPCancel(USB_HOST_IRP * inputIRP)
{
    /* This function cancels an IRP */

    USB_HOST_IRP_LOCAL * irp = (USB_HOST_IRP_LOCAL *) inputIRP;
    DRV_USBHS_OBJ * hDriver;
    DRV_USBHS_HOST_PIPE_OBJ * pipe;
    bool interruptWasEnabled = false;

    if(irp->pipe == DRV_USBHS_HOST_PIPE_HANDLE_INVALID)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Invalid pipe");
        return;
    }

    if(irp->status <= USB_HOST_IRP_STATUS_COMPLETED_SHORT)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "IRP is not pending or in progress");
        return;
    }

    pipe = (DRV_USBHS_HOST_PIPE_OBJ *)irp->pipe;
    hDriver = (DRV_USBHS_OBJ *) pipe->hClient;

    if(!hDriver->isInInterruptContext)
    {
        //OSAL: Get Mutex
        if(OSAL_MUTEX_Lock(&(hDriver->mutexID), OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
        }
        interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
    }

    if(irp->previous == NULL)
    {
        /* This means this was the first
         * irp in the queue. Update the pipe
         * queue head directly */

        pipe->irpQueueHead = irp->next;
        if(irp->next != NULL)
        {
            irp->next->previous = NULL;
        }
    }
    else
    {
        /* Remove the IRP from the linked
         * list */
        irp->previous->next = irp->next;

        if(irp->next != NULL)
        {
            /* This applies if this is not the last
             * irp in the linked list */
            irp->next->previous = irp->previous;
        }
    }

    if(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)
    {
        /* If the irp is already in progress then
         * we set the temporary state. This will get
         * caught in _DRV_USBHS_HOST_ControlXferProcess()
         * and _DRV_USBHS_HOST_NonControlIRPProcess()
         * functions. */

        irp->tempState = DRV_USBHS_HOST_IRP_STATE_ABORTED;
    }
    else
    {
        irp->status = USB_HOST_IRP_STATUS_ABORTED;
        if(irp->callback != NULL)
        {
            irp->callback((USB_HOST_IRP *)irp);
        }
    }

    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
        }
        //OSAL: Release Mutex
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }
}/* end of DRV_USBHS_HOST_IRPCancel() */

void DRV_USBHS_HOST_PipeClose
(
    DRV_USBHS_HOST_PIPE_HANDLE pipeHandle
)
{
    /* This function closes an open pipe */

    bool                        interruptWasEnabled = false;
    DRV_USBHS_OBJ                 * hDriver;
    USB_HOST_IRP_LOCAL          * irp;
    DRV_USBHS_HOST_PIPE_OBJ       * pipe;
    DRV_USBHS_HOST_TRANSFER_GROUP * transferGroup;
    DRV_USBHS_HOST_ENDPOINT_OBJ   * endpointObj;
    USBHS_MODULE_ID               usbID;
    unsigned int direction;

    uint8_t *pFifoTable;
    uint8_t startingSlot;
    uint8_t byteLoop;
    uint8_t bitLoop;
    bool slotObtained = false;

    /* Make sure we have a valid pipe */
    if( ( pipeHandle == 0 )  || pipeHandle == DRV_USBHS_HOST_PIPE_HANDLE_INVALID)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Invalid pipe handle");
        return;
    }

    pipe = (DRV_USBHS_HOST_PIPE_OBJ*) pipeHandle;
    direction = (pipe->endpointAndDirection & 0x80) >> 7;

    /* Make sure tha we are working with a pipe in use */
    if(pipe->inUse != true)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Pipe is not in use");
        return;
    }

    hDriver = (DRV_USBHS_OBJ *)pipe->hClient;
    usbID = hDriver->usbID;

    /* Disable the interrupt */

    if(!hDriver->isInInterruptContext)
    {
        //OSAL: Get Mutex
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
        }
        interruptWasEnabled = _DRV_USBHS_InterruptSourceDisable(hDriver->interruptSource);
    }

    if(USB_TRANSFER_TYPE_CONTROL == pipe->pipeType)
    {
        transferGroup = &hDriver->controlTransferGroup;

        if(pipe->previous == NULL)
        {
            /* The previous pipe could be null if this was the first pipe in the
             * transfer group */

            transferGroup->pipe = pipe->next;
            if(pipe->next != NULL)
            {
                pipe->next->previous = NULL;
            }
        }
        else
        {
            /* Remove this pipe from the linked
             * list */

            pipe->previous->next = pipe->next;
            if(pipe->next != NULL)
            {
                pipe->next->previous = pipe->previous;
            }
        }

        if(transferGroup->nPipes != 0)
        {
            /* Reduce the count only if its
             * not zero already */

            transferGroup->nPipes --;
        }
    }
    else
    {
        /* Non control transfer pipes are not stored as groups.  We deallocate
         * the endpoint object that this pipe used */

        endpointObj = &hDriver->hostEndpointTable[pipe->hostEndpoint];
        endpointObj->endpoints[direction].inUse = false;
        endpointObj->endpoints[direction].pipe = NULL;


        if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
        {
            /* Clear the error status on and flush the fifo
             * on receive endpoint */


            PLIB_USBHS_EndpointRxFIFOFlush(usbID, pipe->hostEndpoint);
            PLIB_USBHS_RxEPStatusClear(usbID, pipe->hostEndpoint, USBHS_RXEP_ERROR_ALL);
            PLIB_USBHS_EndpointRxRequestClear(usbID, pipe->hostEndpoint);
        }
        else
        {
            /* Clear the error status on and flush the fifo
             * on transmit endpoint */
            PLIB_USBHS_EndpointTxFIFOFlush(usbID, pipe->hostEndpoint);
            PLIB_USBHS_TxEPStatusClear(usbID, pipe->hostEndpoint, USBHS_TXEP_ERROR_ALL);

        }
    }

    /* Now we invoke the call back for each IRP in this pipe and say that it is
     * aborted.  If the IRP is in progress, then that IRP will be actually
     * aborted on the next SOF This will allow the USB module to complete any
     * transaction that was in progress. */

    irp = (USB_HOST_IRP_LOCAL *)pipe->irpQueueHead;
    while(irp != NULL)
    {
        irp->pipe = DRV_USBHS_HOST_PIPE_HANDLE_INVALID;

        if(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)
        {
            /* If the IRP is in progress, then we set the temp IRP state. This
             * will be caught in the _DRV_USBHS_HOST_NonControlIRPProcess() and
             * _DRV_USBHS_HOST_ControlXferProcess() functions */
            irp->status = USB_HOST_IRP_STATUS_ABORTED;
            if(irp->callback != NULL)
            {
                irp->callback((USB_HOST_IRP*)irp);
            }
            irp->tempState = DRV_USBHS_HOST_IRP_STATE_ABORTED;
        }
        else
        {
            /* IRP is pending */
            irp->status = USB_HOST_IRP_STATUS_ABORTED;
            if(irp->callback != NULL)
            {
                irp->callback((USB_HOST_IRP*)irp);
            }
        }
        irp = irp->next;
    }

    /* Update the consumed FIFO size for non control transfers. Control
     * transfers have a dedicated FIFO that is the first 64 bytes */

    if(USB_TRANSFER_TYPE_CONTROL != pipe->pipeType)
    {
        pFifoTable = (uint8_t *)(hDriver->gDrvUSBFifoTable);
        startingSlot = pipe->startingOffset/8;

        for(byteLoop = 1; byteLoop <=18; byteLoop++)
        {
            for(bitLoop=0; bitLoop <=7; bitLoop++)
            {
                /*
                 * The design here is to find the starting slot in the
                 * FiFoTable.  Once found, we will start reseting all the slots
                 * allocated.  We need not check for the slot value as slots
                 * during allocation needs to be continous. No segmented slots
                 * for a particluar pipe is possible.
                 */
                if( (slotObtained == false) && (( ((byteLoop - 1) * 8) + 1 + bitLoop) == startingSlot) )
                {
                    /* Starting Slot found. Reset the slot. Make sure it never
                     * enters this if() loop again for this pipe close operation
                     */
                    *pFifoTable = ( *pFifoTable & ~(0x01 << bitLoop) );
                    slotObtained = true;
                    pipe->noOfSlots--;
                }
                else if(slotObtained == true)
                {
                    /* Slot already found before.  We can just continuing
                     * resetting all the slots allocated for the pipe.*/
                    *pFifoTable = ( *pFifoTable & ~(0x01 << bitLoop) );
                    pipe->noOfSlots--;
                }
                if(pipe->noOfSlots == 0)
                {
                    /* Done releasing slots. Exit the inner loop */
                    break;
                }
            }/* end of bitLoop */
            if((pipe->noOfSlots == 0))
            {
                /* Done releasing slots. Exit the outer loop */
                break;
            }
            pFifoTable++;
        }/* end of byteloop */
        if((pipe->noOfSlots != 0))
        {
            /* This is error scenaario */
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Error in releasing Fifo Table");
        }
    }

    /* Now we return the pipe back to the driver */
    pipe->inUse = false ;

    /* Enable the interrupts */
    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
        }
        //OSAL: Return Mutex
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }
}/* end of DRV_USBHS_HOST_PipeClose() */

DRV_USBHS_HOST_PIPE_HANDLE DRV_USBHS_HOST_PipeSetup
(
    DRV_HANDLE client,
    uint8_t deviceAddress,
    USB_ENDPOINT endpointAndDirection,
    uint8_t hubAddress,
    uint8_t hubPort,
    USB_TRANSFER_TYPE pipeType,
    uint8_t bInterval,
    uint16_t wMaxPacketSize,
    USB_SPEED speed
)
{
    int pipeIter;
    int fifoSize;
    bool epFound;
    uint8_t epIter;
    uint8_t endpoint;
    USBHS_MODULE_ID usbID;
    unsigned int epDirection;
    unsigned int shiftWord;
    unsigned int i, j, accumulate;

    DRV_USBHS_OBJ * hDriver;
    DRV_USBHS_HOST_PIPE_OBJ * pipe, * iteratorPipe;
    DRV_USBHS_HOST_TRANSFER_GROUP * transferGroup;
  
    if(client == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Invalid client handle");
        return DRV_USBHS_HOST_PIPE_HANDLE_INVALID;
    }

    if((speed == USB_SPEED_LOW) || (speed == USB_SPEED_FULL) || (speed == USB_SPEED_HIGH))
    {
        if(pipeType != USB_TRANSFER_TYPE_CONTROL)
        {
            if(wMaxPacketSize < 8)
            {
                wMaxPacketSize = 8;
            }
        }
    }

    if((wMaxPacketSize < 8) ||(wMaxPacketSize > 4096))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Invalid pipe endpoint size");
        return DRV_USBHS_HOST_PIPE_HANDLE_INVALID;
    }

    /* Need to make sure that wMaxPacketSize is equal to 2 to the power n where
     * n is 3 >= n <= 12*/

    for(i = 3; i <= 12; i ++)
    {
        accumulate = 1;

        for (j = 0; j < i; j ++)
        {
            accumulate *= 2;
        }

        /* At this point accumulate will be either 8 or 16 or 32...4096. Check
         * specified wMaxPacketSize against this. */

        if(wMaxPacketSize == accumulate)
        {
            /* This means the we dont have to do any adjustment*/

            break;
        }

        if((wMaxPacketSize > accumulate) &&
                (wMaxPacketSize < (accumulate * 2)))
        {
            /* This means the wMaxPacketSize is between two valid ranges. Set
             * wMaxPacketSize to the higher range. */

            wMaxPacketSize = accumulate * 2;
            break;
        }
    }

    hDriver = (DRV_USBHS_OBJ *)client;
    endpoint = (endpointAndDirection & 0x7F);
    epDirection = (endpointAndDirection & 0x80) >> 7;

    usbID = hDriver->usbID;

    /* There are two things that we need to check before we allocate a pipe. We
     * check if have a free pipe object and check if we have a free endpoint
     * that we can use */

    /* All control transfer pipe are grouped together as a linked list of pipes.
     * Non control transfer pipes are organized individually */

    /* We start by searching for a free pipe */

    //OSAL - mutex lock */
    if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
    {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex lock failed");
    }

    for(pipeIter = 0; pipeIter < DRV_USBHS_HOST_PIPES_NUMBER; pipeIter ++)
    {
        if(gDrvUSBHostPipeObj[pipeIter].inUse == false)
        {
            /* This means we have found a free pipe object */

            pipe = &gDrvUSBHostPipeObj[pipeIter];

            if(pipeType != USB_TRANSFER_TYPE_CONTROL)
            {
                /* For non control transfer we need a free
                 * non zero endpoint object. */

                epFound = false;
                for(epIter = 1; 
                        epIter < DRV_USBHS_HOST_MAXIMUM_ENDPOINTS_NUMBER;
                        epIter ++)
                {
                    if(hDriver->hostEndpointTable[epIter]
                            .endpoints[epDirection].inUse == false)
                    {
                        /* This means we have found an endpoint. Capture this
                         * endpoint and indicate that we have found an endpoint
                         * */

                        epFound = true;
                        hDriver->hostEndpointTable[epIter].endpoints[epDirection].inUse = true;
                        hDriver->hostEndpointTable[epIter].endpoints[epDirection].pipe = pipe;

                        /* The following code maps the endpoint size to the
                         * value that should be loaded in the FIFOSZ register */

                        fifoSize = 0;
                        shiftWord = wMaxPacketSize;

                        while((shiftWord & 0x1) != 1)
                        {
                            shiftWord = (shiftWord >> 1);
                            fifoSize ++;
                        }
                        fifoSize -= 3;

                        if(_DRV_USBHS_HOST_FifoTableAllocate(hDriver,pipe,wMaxPacketSize)
                                    == DRV_USBHS_HOST_PIPE_HANDLE_INVALID)
                        {
                            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Could not obtain FIFO");
                            return DRV_USBHS_HOST_PIPE_HANDLE_INVALID;
                        }
                        if(USB_DATA_DIRECTION_DEVICE_TO_HOST == epDirection)
                        {
                            /* This means host is receiving data. We need to
                             * setup all receive related registers. Setup the
                             * maximum packet size, fifo size, fifo start
                             * address,  target function address, rx type
                             * register, hub address and hub port  */

                            PLIB_USBHS_HostRxEndpointConfigure
                            (
                                usbID,          /* USB Module ID */
                                epIter,         /* Host Endpoint */
                                speed,          /* Endpoint speed */
                                pipeType,       /* Pipe Type */
                                wMaxPacketSize, /* Endpoint Size */
                                pipe->startingOffset,/* FIFO Address */
                                fifoSize,       /* FIFO size */
                                endpoint,       /* Target Endpoint */
                                deviceAddress,  /* Target Address */
                                hubAddress,     /* Target Hub Address */
                                hubPort,        /* Target Hub Port */
                                bInterval       /* NAK or period interval */
                            );
                            /* Clear the Data toggle */
                            PLIB_USBHS_HostRxEndpointDataToggleClear(usbID, epIter);
                        }
                        else
                        {
                            /* This means data flows host to device. We must
                             * setup TX registers */

                            PLIB_USBHS_HostTxEndpointConfigure
                            (
                                usbID,              /* USB Module ID */
                                epIter,             /* Host Endpoint */
                                speed,              /* Endpoint speed */
                                pipeType,           /* Pipe Type */
                                wMaxPacketSize,     /* Endpoint Size */
                                pipe->startingOffset,/* FIFO Address */
                                fifoSize,       /* FIFO size */
                                endpoint,       /* Target Endpoint */
                                deviceAddress,  /* Target Address */
                                hubAddress,     /* Target Hub Address */
                                hubPort,        /* Target Hub Port */
                                bInterval       /* NAK or period interval */
                            );
                            /* Clear the Data toggle */
                            PLIB_USBHS_HostTxEndpointDataToggleClear(usbID, epIter);
                        }
                        break;
                    }
                } /* End of search for host endpoint */

                if(!epFound)
                {
                    /* This means we could not find a spare endpoint for this
                     * non control transfer. */

                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Could not allocate endpoint");
                    //OSAL mutex unlock
                    if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
                    {
                        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
                    }
                    return DRV_USBHS_HOST_PIPE_HANDLE_INVALID;
                }
                else
                {
                    /* Clear all the previous error condition that may be there
                     from the last device connect */
                    if(USB_DATA_DIRECTION_HOST_TO_DEVICE == epDirection)
                    {
                        PLIB_USBHS_TxEPStatusClear(usbID, epIter,
                                USBHS_TXEP_ERROR_ALL);
                    }
                    else
                    {
                        PLIB_USBHS_RxEPStatusClear(usbID, epIter, 
                                USBHS_RXEP_ERROR_ALL);
                    }
                }
                
            } /* End of non zero endpoint handling */
            else
            {
                /* Set epIter to zero to indicate that we must use endpoint 0
                 * for control transfers. We also add the control transfer pipe
                 * to the control transfer group. */

                epIter = 0;

                transferGroup = &hDriver->controlTransferGroup;

                if(transferGroup->pipe == NULL)
                {
                    /* This if the first pipe to be setup */

                    transferGroup->pipe = pipe;
                    transferGroup->currentPipe = pipe;
                    pipe->previous = NULL;
                }
                else
                {
                    /* This is not the first pipe. Find the last pipe in the
                     * linked list */

                    iteratorPipe = transferGroup->pipe;
                    while(iteratorPipe->next != NULL)
                    {
                        /* This is not the last pipe in this transfer group */
                        iteratorPipe = iteratorPipe->next;
                    }
                    iteratorPipe->next = pipe;
                    pipe->previous = iteratorPipe;
                }
                pipe->next = NULL;

                /* Update the pipe count in the transfer group */

                transferGroup->nPipes ++;

            }

            /* Setup the pipe object */
            pipe->inUse         = true;
            pipe->deviceAddress = deviceAddress;
            pipe->irpQueueHead  = NULL;
            pipe->bInterval     = bInterval;
            pipe->speed         = speed;
            pipe->hubAddress    = hubAddress;
            pipe->hubPort       = hubPort;
            pipe->pipeType      = pipeType;
            pipe->hClient       = client;
            pipe->endpointSize  = wMaxPacketSize;
            pipe->intervalCounter = bInterval;
            pipe->hostEndpoint  = epIter;
            pipe->endpointAndDirection   = endpointAndDirection;

            // OSAL release mutex
            if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
            }
            return((DRV_USBHS_HOST_PIPE_HANDLE)pipe);
        }
    }

    /* This means that we could not find a free pipe object */
    //OSAL mutex release
    if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
    }
    return(DRV_USBHS_HOST_PIPE_HANDLE_INVALID);
}/* end of DRV_USBHS_HOST_PipeSetup() */

void _DRV_USBHS_HOST_ControlTransferProcess(DRV_USBHS_OBJ * hDriver)
{
    /* This function is called every time there is an endpoint 0
     interrupt. This means that a stage of the current control IRP has been
     completed. This function is called from an interrupt context */

    uint8_t status;
    USB_HOST_IRP_LOCAL * irp;
    DRV_USBHS_HOST_PIPE_OBJ * pipe, * iterPipe;
    DRV_USBHS_HOST_TRANSFER_GROUP * transferGroup;
    bool endIRP = false;
    unsigned int count, i;
    bool foundIRP = false;
    bool isDmaUsed = false;
    USBHS_MODULE_ID usbID;

    transferGroup = &hDriver->controlTransferGroup;
    usbID = hDriver->usbID;

    /* First check if the IRP was aborted */
    irp = transferGroup->currentIRP;
    pipe = transferGroup->currentPipe;

    /* If current IRP is null, or current pipe is null then we have unknow
     * failure. We just quit.  Nothing we can do.*/

    if((irp == NULL) || (pipe == NULL) ||
            (pipe == (DRV_USBHS_HOST_PIPE_OBJ *)DRV_USBHS_HOST_PIPE_HANDLE_INVALID))
    {
        /* Unknown error */
        return;
    }

    /* If here means, we have a valid IRP and pipe.  Check the status register.
     * The IRP could have been aborted. This would be known in the temp state.
     * */

    status = PLIB_USBHS_EP0StatusGet(usbID);
    if(irp->tempState == DRV_USBHS_HOST_IRP_STATE_ABORTED)
    {
        /* This means the application has aborted this
         IRP.*/

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ABORTED;

        /* Clear up the error status and we must also flush the fifo */

        PLIB_USBHS_EP0StatusClear(usbID, USBHS_EP0_ERROR_ALL);
        PLIB_USBHS_Endpoint0FIFOFlush(usbID);
    }
    else if(status & USBHS_EP0_ERROR_RXSTALL)
    {
        /* This means the packet was stalled. Set the error status and then
         * clear the stall bit */

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
        PLIB_USBHS_EP0StatusClear(usbID, USBHS_EP0_ERROR_RXSTALL);
    }
    else if(status & USBHS_EP0_ERROR_NAK_TIMEOUT)
    {
        /* This means a NAK Time Out has occurred. Clear the error status.  */

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT;
        PLIB_USBHS_EP0StatusClear(usbID, USBHS_EP0_ERROR_NAK_TIMEOUT);
    }
    else if(status & USBHS_EP0_ERROR_BUS)
    {
        /* This means there was an bus error. The packet was tried three times
         * and was not successfully processed */

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ERROR_DATA;
        PLIB_USBHS_EP0StatusClear(usbID, USBHS_EP0_ERROR_BUS);
    }
    else
    {
        /* This means there was no error. We should check the current state of
         * the IRP and then proceed accordingly */

       switch(irp->tempState)
       {
           case DRV_USBHS_HOST_IRP_STATE_SETUP_STAGE:

               /* We got an interrupt after this stage. This means that the
                * setup stage has completed.  We need to check if a data stage
                * is required and then start the data stage. */

               if((irp->data == NULL) || (irp->size == 0))
               {
                   /* This means there is no data stage. We can proceed to the
                    * handshake stage. In a case where there isnt a data stage,
                    * we need to send an IN token to the device to the device */

                   irp->tempState = DRV_USBHS_HOST_IRP_STATE_HANDSHAKE_SENT;
                   PLIB_USBHS_EP0INHandshakeSend(usbID);
               }
               else
               {
                   /* This means that a data stage is required. We can find out
                    * the direction of the data stage by investigating the flags
                    * parameter of the  IRP. */

                   irp->tempState = DRV_USBHS_HOST_IRP_STATE_DATA_STAGE_SENT;
                   if(irp->flags & 0x8000)

                   {
                       /* This means the data stage moves from device to host.
                        * So the host would have to send an IN token.  */

                       PLIB_USBHS_EP0INTokenSend(usbID);
                   }
                   else
                   {
                       /* This function loads the fifo and sends the packet. The
                        * completed bytes field in the IRP will be updated. */

                       _DRV_USBHS_HOST_IRPTransmitFIFOLoad(usbID, irp, 0);
                   }
               }
               break;

           case DRV_USBHS_HOST_IRP_STATE_DATA_STAGE_SENT:

               /* We reach here after an interrupt which means that a data stage
                * interaction was completed. Find out what was the direction the
                * data stage */

               if(irp->flags & 0x8000)
               {
                   /* This means the data was moving from device to host. We got
                    * an interrupt, which means we have received data. Start by
                    * checking how much data we received from the device */

                   count = _DRV_USBHS_HOST_IRPReceiveFIFOUnload(usbID, irp, 0,
                           &isDmaUsed);

                   if((count < pipe->endpointSize) ||
                           (irp->completedBytes >= irp->size))
                   {
                       /* This means that we either received a short packet or
                        * we received the amount of data that we needed. We
                        * should move to the handshake stage. Send the  */

                       irp->tempState = DRV_USBHS_HOST_IRP_STATE_HANDSHAKE_SENT;
                       PLIB_USBHS_EP0OUTHandshakeSend(usbID);

                   }
                   else
                   {
                       /* This means this is a multi stage control read
                        * transfer. Issue another IN token */

                       PLIB_USBHS_EP0INTokenSend(usbID);
                       break;
                   }
               }
               else
               {
                   /* Data stage was moving from host to device.  Check if we
                    * need to send more data */

                   if(irp->completedBytes < irp->size)
                   {
                       _DRV_USBHS_HOST_IRPTransmitFIFOLoad(usbID, irp, 0);
                   }
                   else
                   {
                       /* We can move to the status stage */
                       irp->tempState = DRV_USBHS_HOST_IRP_STATE_HANDSHAKE_SENT;
                       PLIB_USBHS_EP0INHandshakeSend(usbID);
                   }
               }
               break;

           case DRV_USBHS_HOST_IRP_STATE_HANDSHAKE_SENT:

               /* If we have reached here, it means that status stage has
                * completed. Check the direction of the data stage, update the
                * irp status flag and then end the irp. */

               irp->status = USB_HOST_IRP_STATUS_COMPLETED;
               
               if(irp->flags & 0x8000)
               {
                   /* This means the data stage moved from device to host. We
                    * need to check the number of bytes the host sent. If it was
                    * less than expected, then irp status should say so */

                   if(irp->completedBytes < irp->size)
                   {
                       irp->size = irp->completedBytes;
                       irp->status = USB_HOST_IRP_STATUS_COMPLETED_SHORT;
                   }
               }
               else
               {
                   /* We need to clear the Status Pakcet bit and
                    the Rx packet ready bit */
                   PLIB_USBHS_EP0INHandshakeClear(usbID);
               }
               endIRP = true;
               break;

           default:
               break;
       }
    }

    if(endIRP)
    {
        /* This means this IRP needs to be terminated and new one started. */

        if(irp->callback != NULL)
        {
            irp->callback((USB_HOST_IRP *)irp);
        }

        /* Expire the IRP */

        pipe->irpQueueHead = irp->next;

        /* Now we need to check if any more IRPs are in this group are pending.
         * We start searching from the current pipe onwards. If we dont find
         * another pipe with an IRP, we should land back on the current pipe and
         * check if we have a IRP to process */

        iterPipe = transferGroup->currentPipe->next;
        for(i = 0; i < transferGroup->nPipes; i ++)
        {
            if(iterPipe == NULL)
            {
                /* We have reached the end of the pipe group. Rewind the pipe
                 * iterator to the start of the pipe group. */

                iterPipe = transferGroup->pipe;
            }

            /* This means that we have a valid pipe.  Now check if there is irp
             * to be processed. */

            if(iterPipe->irpQueueHead != NULL)
            {
                foundIRP = true;
                transferGroup->currentPipe = iterPipe;
                transferGroup->currentIRP = iterPipe->irpQueueHead;
                break;
            }

            iterPipe = iterPipe->next;
        }

        if(foundIRP)
        {
            /* This means we have found another IRP to process. We must load the
             * endpoint. */
            
            irp = transferGroup->currentIRP;
            pipe = transferGroup->currentPipe;
            irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
            irp->tempState = DRV_USBHS_HOST_IRP_STATE_SETUP_STAGE;

            /* We need to update the flags parameter of the IRP to indicate the
             * direction of the control transfer. */

            if(*((uint8_t*)(irp->setup)) & 0x80)
            {
                /* This means the data stage moves from device to host. Set bit
                 * 15 of the flags parameter */
               irp->flags |= 0x8000;
            }
            else
            {
                /* This means the data stage moves from host to device. Clear
                 * bit 15 of the flags parameter. */
                irp->flags &= 0x7FFF;
            }

            /* Send the setup packet to the device */
            _DRV_USBHS_HOST_IRPTransmitSetupPacket(usbID, irp);
        }
        else
        {
            /* This means we dont have an IRP. Set the current IRP and current
             * pipe to NULL to indicate that we dont have any active IRP */

            transferGroup->currentPipe = NULL;
            transferGroup->currentIRP = NULL;
        }
    }

    return;
}/* end of _DRV_USBHS_HOST_ControlTransferProcess() */

void _DRV_USBHS_HOST_NonControlTransferProcess
(
    DRV_USBHS_OBJ * hDriver,
    uint8_t endpoint, 
    unsigned int direction
)
{
    /* This function processes non zero endpoint transfers which
     would bulk, interrupt and isochronous transfers */

    DRV_USBHS_HOST_ENDPOINT_OBJ * endpointTable = &(hDriver->hostEndpointTable[0]);
    DRV_USBHS_HOST_PIPE_OBJ * pipe;
    USB_HOST_IRP_LOCAL * irp;
    uint8_t status;
    bool endIRP = false;
    bool isDmaUsed = false;
    unsigned int count;
    USBHS_MODULE_ID usbID;

    endpointTable += endpoint;
    usbID = hDriver->usbID;

    pipe = endpointTable->endpoints[direction].pipe;

    if((!endpointTable->endpoints[direction].inUse) ||
       (pipe == NULL) ||
       (pipe == (DRV_USBHS_HOST_PIPE_OBJ *)(DRV_USBHS_HOST_PIPE_HANDLE_INVALID)))
    {
        /* This means the pipe was closed. We dont do anything */
        return;
    }

    irp = pipe->irpQueueHead;

    if(USB_DATA_DIRECTION_HOST_TO_DEVICE == direction)
    {
        /* We got an interrupt for data moving from host to to device. Check if
         * there were any transmission errors. Then check if there is any more
         * data to be sent in the IRP. If we are done, then end the IRP and
         * start a new one. */

        status = PLIB_USBHS_TxEPStatusGet(usbID, endpoint);
        /* Check if the IRP was aborted */
        if(irp->tempState == DRV_USBHS_HOST_IRP_STATE_ABORTED)
        {
            endIRP = true;
            irp->status = USB_HOST_IRP_STATUS_ABORTED;

            /* Clear the error bits and the FIFO */
            PLIB_USBHS_TxEPStatusClear(usbID, endpoint, USBHS_TXEP_ERROR_ALL);
            PLIB_USBHS_EndpointTxFIFOFlush(usbID, endpoint);
        }
        else if(status & USBHS_TXEP_ERROR_RXSTALL)
        {
            /* This means the packet was stalled. Set the error status and then
             * clear the stall bit */

            endIRP = true;
            irp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
            PLIB_USBHS_TxEPStatusClear(usbID, endpoint, USBHS_TXEP_ERROR_RXSTALL);

            /* Clear the Data toggle */
            PLIB_USBHS_HostTxEndpointDataToggleClear(usbID, endpoint);

        }
        else if(status & USBHS_TXEP_ERROR_NAK_TIMEOUT)
        {
            /* This means a NAK Time Out has occurred. Clear the error status.
             * */

            endIRP = true;
            irp->status = USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT;
            PLIB_USBHS_TxEPStatusClear(usbID, endpoint, USBHS_TXEP_ERROR_NAK_TIMEOUT);
        }
        else if(status & USBHS_TXEP_ERROR_BUS)
        {
            /* This means there was an bus error. The packet was tried three
             * times and was not successfully processed */

            endIRP = true;
            irp->status = USB_HOST_IRP_STATUS_ERROR_DATA;
            PLIB_USBHS_TxEPStatusClear(usbID, endpoint, USBHS_TXEP_ERROR_BUS);
        }
        else
        {
            /* This means this transaction completed successfully.  We should
             * check if there are any spare bytes remaining to be sent and then
             * send it */

            if(irp->completedBytes >= irp->size)
            {
                endIRP = true;
                irp->status = USB_HOST_IRP_STATUS_COMPLETED;
            }
            else
            {
                /* This means we have more data to send */
                endIRP = false;

                /* This function will load the next packet for this IRP into the
                 * endpoint FIFO and then transmit it. */
                _DRV_USBHS_HOST_IRPTransmitFIFOLoad(usbID, irp, endpoint);
            }
        }

        if(endIRP)
        {
            /* This means we need to end the IRP */

            pipe->irpQueueHead = irp->next;

            if(irp->callback)
            {
                /* Invoke the call back*/
                irp->callback((USB_HOST_IRP *)irp);
            }

            irp = pipe->irpQueueHead;

            if((irp != NULL) && (!(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)))
            {
                /* We do have another IRP to process. */
                irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
                _DRV_USBHS_HOST_IRPTransmitFIFOLoad(usbID, irp, endpoint);
            }
        }
    }
    else
    {
         /* We got an interrupt for data moving from device to to host. Check if
          * there were any receptions errors. Then check if there is any more
          * data to be received in the IRP. If we are done, then end the IRP and
          * start a new one. */

        status = PLIB_USBHS_RxEPStatusGet(usbID, endpoint);

        /* Check if the IRP was aborted */
        if(irp->tempState == DRV_USBHS_HOST_IRP_STATE_ABORTED)
        {
            endIRP = true;
            irp->status = USB_HOST_IRP_STATUS_ABORTED;

            /* Clear the error flags and then flush the fifo if a data is
             * available */
            PLIB_USBHS_RxEPStatusClear(usbID, endpoint, USBHS_RXEP_ERROR_ALL);
            PLIB_USBHS_EndpointRxFIFOFlush(usbID, endpoint);
        }
        else if(status & USBHS_RXEP_ERROR_RXSTALL)
        {
            /* This means the packet was stalled. Set the error status and then
             * clear the stall bit */

            endIRP = true;
            irp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
            PLIB_USBHS_RxEPStatusClear(usbID, endpoint, USBHS_RXEP_ERROR_RXSTALL);

            /* Clear the Data toggle */
            PLIB_USBHS_HostRxEndpointDataToggleClear(usbID, endpoint);
            
            PLIB_USBHS_EndpointRxFIFOFlush(usbID, endpoint);
        }
        else if(status & USBHS_RXEP_ERROR_NAK_TIMEOUT)
        {
            /* This means a NAK Time Out has occurred. Clear the error status.
             * */

            endIRP = true;
            irp->status = USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT;
            PLIB_USBHS_RxEPStatusClear(usbID, endpoint, USBHS_RXEP_ERROR_NAK_TIMEOUT);
            PLIB_USBHS_EndpointRxFIFOFlush(usbID, endpoint);
        }
        else if(status & USBHS_RXEP_ERROR_BUS)
        {
            /* This means there was an bus error. The packet was tried three
             * times and was not successfully processed */

            endIRP = true;
            irp->status = USB_HOST_IRP_STATUS_ERROR_DATA;
            PLIB_USBHS_RxEPStatusClear(usbID, endpoint, USBHS_RXEP_ERROR_BUS);
            PLIB_USBHS_EndpointRxFIFOFlush(usbID, endpoint);
        }
        else
        {
            /* This means that data was received without errors. */

            count = _DRV_USBHS_HOST_IRPReceiveFIFOUnload(usbID, irp, endpoint, &isDmaUsed);
            if(isDmaUsed == false)
            {
                if((count < pipe->endpointSize) ||
                        (irp->completedBytes >= irp->size))
                {
                    endIRP = true;
                    irp->status = USB_HOST_IRP_STATUS_COMPLETED;

                    if(irp->completedBytes < irp->size)
                    {
                        /* This means the device ended the transfer and and we
                         * have a short transfer */
                        irp->status = USB_HOST_IRP_STATUS_COMPLETED_SHORT;
                    }

                    /* Update the actual amount of data received */
                    irp->size = irp->completedBytes;
                }
                else
                {
                    /* This means we have more data to send We request another
                     * packet */

                    endIRP = false;
                    PLIB_USBHS_RxEPINTokenSend(usbID, endpoint);
                }
            }
            else
            {
                /* DMA has been used. Do not end the IRP here.
                 It will be done in DMA ISR handler */
                endIRP = false;
            }
        }

        if(endIRP)
        {
            /* This means we need to end the IRP */

            pipe->irpQueueHead = irp->next;
          
            if(irp->callback)
            {
                /* Invoke the call back*/
                irp->callback((USB_HOST_IRP *)irp);
            }

            irp = pipe->irpQueueHead;

            /* A IRP could have been submitted in the callback. If that is the
             * case and the IRP status would indicate that it already in
             * progress. If the IRP in the queue head is not in progress then we
             * shold initiate the transaction */

            if((irp != NULL) && (!(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)))
            {
                irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
                
                /* We do have another IRP to process. Request for
                 * an IN packet. */

                PLIB_USBHS_RxEPINTokenSend(usbID, endpoint);
            }
        }
    }
}/* end of _DRV_USBHS_HOST_NonControlTransferProcess() */

void _DRV_USBHS_HOST_Tasks_ISR(DRV_USBHS_OBJ * hDriver)
{
    unsigned int i;
    USBHS_GEN_INTERRUPT interruptStatus;
    USBHS_EPTXRX_INTERRUPT interruptRxStatus;
    USBHS_EPTXRX_INTERRUPT interruptTxStatus;

    _DRV_USBHS_NonPersistentInterruptSourceClear(hDriver->interruptSource);
    USBHS_MODULE_ID usbID = hDriver->usbID;
    

    interruptStatus = PLIB_USBHS_GenInterruptFlagsGet(usbID);

    if(interruptStatus & USBHS_GENINT_DEVCONN) 
    {
        hDriver->deviceAttached = true;

    }
    else if(interruptStatus & USBHS_GENINT_DEVDISCONN)
    {
        /* We go a detach interrupt. The detach interrupt could have occurred
         * while the attach de-bouncing is in progress. We just set a flag saying
         * the device is detached; */

        hDriver->deviceAttached = false;
        
        if(hDriver->attachedDeviceObjHandle != USB_HOST_DEVICE_OBJ_HANDLE_INVALID)
        {
            /* Ask the host layer to de-enumerate this device. The de-enumeration
             * must be done in the interrupt context. */
            USB_HOST_DeviceDenumerate (hDriver->attachedDeviceObjHandle);
        }

        hDriver->attachedDeviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
    }
    
    if(interruptStatus & USBHS_GENINT_VBUSERR)
    {
        /* This interrupt occurs if a VBUS error has occurred. The fact that
         * that the code has reached means the flag was cleared (reading the
         * interrupt status clears the flag). */
        
        /* Device De enumerate is required here because the client drivers needs
         to reset their data structures. Asserting NRSTX Reset from ISR here
         will lead to Session disable. Though the Session will be enabled from
         task context later on but no disconnect will be generated. */
        
        hDriver->deviceAttached = false;
        
        if(hDriver->attachedDeviceObjHandle != USB_HOST_DEVICE_OBJ_HANDLE_INVALID)
        {
            /* Ask the host layer to de-enumerate this device. The de-enumeration
             * must be done in the interrupt context. */
            USB_HOST_DeviceDenumerate (hDriver->attachedDeviceObjHandle);
            hDriver->attachedDeviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
        }
        
        PLIB_USBHS_SoftResetNRSTXEnable(usbID);
    }
    
    if(interruptStatus & USBHS_GENINT_BABBLE)
    {
        /* This interrupt occurs if a Babble error has occurred. The fact that
         * that the code has reached means the flag was cleared (reading the
         * interrupt status clears the flag). */
        
        hDriver->deviceAttached = false;
        
        if(hDriver->attachedDeviceObjHandle != USB_HOST_DEVICE_OBJ_HANDLE_INVALID)
        {
            /* Ask the host layer to de-enumerate this device. The de-enumeration
             * must be done in the interrupt context. */
            USB_HOST_DeviceDenumerate (hDriver->attachedDeviceObjHandle);
            hDriver->attachedDeviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
        }
        
        PLIB_USBHS_SoftResetEnable(usbID);
    }


    interruptTxStatus = PLIB_USBHS_TxInterruptFlagsGet(usbID);
    interruptRxStatus = PLIB_USBHS_RxInterruptFlagsGet(usbID);

    /* Check if we have an endpoint interrupt */

    if(interruptTxStatus & USBHS_TXRXINT_EP0)
    {
        /* Process control transfers */
        _DRV_USBHS_HOST_ControlTransferProcess(hDriver);
    }

    /* Process non control transfers */
    for(i = 1; i < DRV_USBHS_HOST_MAXIMUM_ENDPOINTS_NUMBER; i++ )
    {
        if(interruptTxStatus & (1 << i))
        {
            /* This means an interrupt occurred on a non zero endpoint. */

            _DRV_USBHS_HOST_NonControlTransferProcess(hDriver, i,
                    USB_DATA_DIRECTION_HOST_TO_DEVICE);

        }

        if(interruptRxStatus & (1 << i))
        {
            /* This means an receive endpoint interrupt occurred on a non zero
             * endpoint. */

            _DRV_USBHS_HOST_NonControlTransferProcess(hDriver, i,
                    USB_DATA_DIRECTION_DEVICE_TO_HOST);
        }
    }
}/* end of _DRV_USBHS_HOST_Tasks_ISR() */

void _DRV_USBHS_HOST_Tasks_ISR_USBDMA(DRV_USBHS_OBJ * hDriver)
{
    USBHS_MODULE_ID usbID = hDriver->usbID;
    uint8_t usbDMAInterrupts;
    uint8_t  iEndpoint;
    uint8_t usbDMAChannel = 0;
    bool endpointDir;
    bool usbDMAError = false;
    bool endIRP = false;
    USB_HOST_IRP_LOCAL * irp = (USB_HOST_IRP_LOCAL *)NULL;

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

    /* Reading the interrupt flags will cause the
     * flags to get cleared */
    usbDMAInterrupts = PLIB_USBHS_DMAInterruptGet(usbID);

    for(usbDMAChannel = 1; usbDMAChannel < 9; usbDMAChannel++)
    {
        if((usbDMAInterrupts >> (usbDMAChannel - 1)) & 0x01)
        {
            /* DMA interrupt set for this channel.
             * Map the EP it is assigned for
             */
            usbDMAError = PLIB_USBHS_DMAErrorGet(usbID, usbDMAChannel);
            if(usbDMAError == true)
            {
                continue;
            }
            iEndpoint = hDriver->gDrvUSBDMAPool[usbDMAChannel].iEndpoint;
            endpointDir = hDriver->gDrvUSBDMAPool[usbDMAChannel].endpointDir;

            /* Check the EP direction */
            if(endpointDir == USB_DATA_DIRECTION_HOST_TO_DEVICE)
            {
                /* This is TX Endpoint */
                irp = hDriver->hostEndpointTable[iEndpoint] .endpoints[endpointDir].pipe->irpQueueHead;
                irp->completedBytes += hDriver->gDrvUSBDMAPool[usbDMAChannel].count;
                PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, iEndpoint);
                /* Release the DMA channel */
                hDriver->gDrvUSBDMAPool[usbDMAChannel].inUse = false;
            }
            else
            {
                /* This is RX Endpoint */

                /* Release the DMA channel */
                hDriver->gDrvUSBDMAPool[usbDMAChannel].inUse = false;

                irp = hDriver->hostEndpointTable[iEndpoint].endpoints[endpointDir].pipe->irpQueueHead;
                irp->completedBytes += hDriver->gDrvUSBDMAPool[usbDMAChannel].count;

                /* Clear RXPktRDY bit in the corresponding register */
                PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint, USBHS_RXEP_PKTRDY);

                if(((hDriver->gDrvUSBDMAPool[usbDMAChannel].count) < 
                            (hDriver->hostEndpointTable[iEndpoint] .endpoints[endpointDir].pipe->endpointSize)) ||
                        (irp->completedBytes >= irp->size))
                {
                    endIRP = true;
                    irp->status = USB_HOST_IRP_STATUS_COMPLETED;

                    if(irp->completedBytes < irp->size)
                    {
                        /* This means the device ended the transfer and
                         * and we have a short transfer */
                        irp->status = USB_HOST_IRP_STATUS_COMPLETED_SHORT;
                    }
                    irp->size = irp->completedBytes;
                }
                else
                {
                    /* This means we have more data to send
                     * We request another packet */

                    endIRP = false;
                    PLIB_USBHS_RxEPINTokenSend(usbID, iEndpoint);
                }

                if(endIRP)
                {
                    /* This means we need to end the IRP */

                    hDriver->hostEndpointTable[iEndpoint].endpoints[endpointDir].pipe->irpQueueHead = irp->next;

                    if(irp->callback)
                    {
                        /* Invoke the call back*/
                        irp->callback((USB_HOST_IRP *)irp);
                    }

                    irp = hDriver->hostEndpointTable[iEndpoint].endpoints[endpointDir].pipe->irpQueueHead;

                    /* A IRP could have been submitted in the callback. If that is the case
                     * and the IRP status would indicate that it already in progress. If
                     * the IRP in the queue head is not in progress then we shold initiate
                     * the transaction */

                    if((irp != NULL) && (!(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)))
                    {
                        irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;

                        /* We do have another IRP to process. Request for
                         * an IN packet. */

                        PLIB_USBHS_RxEPINTokenSend(usbID, iEndpoint);
                    }
                }
            }
        }
    }

}/* end of _DRV_USBHS_HOST_Tasks_ISR_USBDMA() */

void DRV_USBHS_HOST_BusResetControl(DRV_HANDLE client, bool control)
{
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;

    if((client == DRV_HANDLE_INVALID) || (((DRV_USBHS_OBJ *)client) == NULL))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Invalid client");
        return;
    }

    hDriver = (DRV_USBHS_OBJ *)client;
    usbID = hDriver->usbID;

    /* If control is true, then enable reset */

    if(control == true)
    {
        /* Enable the rest signaling */
        PLIB_USBHS_ResetEnable(usbID);
    }
    else
    {
        /* Clear the reset signaling. At this point we
         * can check the speed of the connected device. */
        PLIB_USBHS_ResetDisable(usbID);

        if(PLIB_USBHS_HighSpeedIsConnected(usbID))
        {
            /* This means the device attached at high speed */
            hDriver->deviceSpeed = USB_SPEED_HIGH;
        }
        else if(PLIB_USBHS_FullOrHighSpeedIsConnected(usbID))
        {
            hDriver->deviceSpeed = USB_SPEED_FULL;
        }
        else
        {
            hDriver->deviceSpeed = USB_SPEED_LOW;
        }
    }
}/* end of DRV_USBHS_HOST_BusResetControl() */

void DRV_USBHS_HOST_StartOfFrameControl(DRV_HANDLE client, bool control)
{
    /* At the point this function does not do any thing.
     * The Start of frame signaling in this HCD is controlled
     * automatically by the module. */
}/* end of DRV_USBHS_HOST_StartOfFrameControl() */

USB_SPEED DRV_USBHS_HOST_DeviceCurrentSpeedGet(DRV_HANDLE client)
{
    /* This function returns the current device speed */

    DRV_USBHS_OBJ * hDriver;

    if((client == DRV_HANDLE_INVALID) || (((DRV_USBHS_OBJ *)client) == NULL))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Invalid client");
    }

    hDriver = (DRV_USBHS_OBJ *)client;
    return(hDriver->deviceSpeed);

}/* end of DRV_USBHS_HOST_DeviceCurrentSpeedGet() */

void DRV_USBHS_HOST_OperationEnable(DRV_HANDLE hClient, bool enable)
{
    /* For this driver, this function does not do anything. The
     * PIC32MZ USB module does not have a explicit USB HOST mode
     * enable bit */

}

uint8_t _DRV_USBHS_HOST_GetFreeDMAChannel
(
    DRV_USBHS_OBJ * hDriver,
    bool endpointDir,
    uint8_t iEndpoint
)
{
    /* Start of local variables */
    uint8_t dmaChannel = 0;
    /* end of local variables */

    for(dmaChannel = 1; dmaChannel < 9; dmaChannel++)
    {
        if((hDriver->gDrvUSBDMAPool[dmaChannel]).inUse == false)
        {
            /* Found Not used DMA Channel */
            hDriver->gDrvUSBDMAPool[dmaChannel].inUse = true;
            hDriver->gDrvUSBDMAPool[dmaChannel].endpointDir = endpointDir;
            hDriver->gDrvUSBDMAPool[dmaChannel].iEndpoint = iEndpoint;
            break;
        }
    }
    /* DMA operation blocked */
    return 0;
}/* end of _DRV_USBHS_HOST_GetFreeDMAChannel() */

// ****************************************************************************
/* Function:
    bool DRV_USBHS_HOST_EventDisable
    (
        DRV_HANDLE handle
    );
    
  Summary:
    Disables host mode events.
	
  Description:
    This function disables the host mode events. This function is called by the
    Host Layer when it wants to execute code atomically. 
	
  Remarks:
    Refer to drv_usbfs.h for usage information.
*/

bool DRV_USBHS_HOST_EventsDisable
(
    DRV_HANDLE handle
)
{
    DRV_USBHS_OBJ * pUSBDrvObj;
    bool result = false;

    if((DRV_HANDLE_INVALID != handle) && (0 != handle))
    {
        pUSBDrvObj = (DRV_USBHS_OBJ *)(handle);
        result = _DRV_USBHS_InterruptSourceDisable(pUSBDrvObj->interruptSource);
    }

    return(result);
}


// ****************************************************************************
/* Function:
    void DRV_USBHS_HOST_EventsDisable
    (
        DRV_HANDLE handle
        bool eventRestoreContext
    );
    
  Summary:
    Restores the events to the specified the original value.
	
  Description:
    This function will restore the enable disable state of the events.
    eventRestoreContext should be equal to the value returned by the
    DRV_USBHS_HOST_EventsDisable() function.
	
  Remarks:
    Refer to drv_usbfs.h for usage information.
*/

void DRV_USBHS_HOST_EventsEnable
(
    DRV_HANDLE handle, 
    bool eventContext
)
{
    DRV_USBHS_OBJ * pUSBDrvObj;
   
    if((DRV_HANDLE_INVALID != handle) && (0 != handle))
    {
        pUSBDrvObj = (DRV_USBHS_OBJ *)(handle);
        if(eventContext == true)
        {
            _DRV_USBHS_InterruptSourceEnable(pUSBDrvObj->interruptSource);
        }
        else
        {
            _DRV_USBHS_InterruptSourceDisable(pUSBDrvObj->interruptSource);
        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Root Hub Driver Routines
// *****************************************************************************
// *****************************************************************************

void DRV_USBHS_HOST_ROOT_HUB_OperationEnable(DRV_HANDLE handle, bool enable)
{
    /* Start of local variable */
    DRV_USBHS_OBJ * pUSBDrvObj = (DRV_USBHS_OBJ *)handle;
    /* End of local variable */

    /* Check if the handle is valid or opened */
    if((handle == DRV_HANDLE_INVALID) || (!(pUSBDrvObj->isOpened)))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        if(true == enable)
        {
            /* The USB Global interrupt and USB module is already enabled at
             * this point. We enable the attach interrupt to detect attach
             * First we enable port power */

            if(pUSBDrvObj->rootHubInfo.portPowerEnable != NULL)
            {
                /* This USB module has only one port. So we call this function
                 * once to enable the port power on port 0*/

                pUSBDrvObj->rootHubInfo.portPowerEnable(0 /* Port 0 */, true); 
            }

            /* Enable the attach and detach interrupt and EP0 interrupt. */
            PLIB_USBHS_InterruptEnableSet(pUSBDrvObj->usbID, (USBHS_GENINT_BABBLE|USBHS_GENINT_VBUSERR|USBHS_GENINT_DEVCONN|USBHS_GENINT_DEVDISCONN), 0x1, 0x0);
            
        }
        else
        {
            /* If the root hub operation is disable, we disable detach and
             * attached event and switch off the port power. */

            SYS_INT_SourceStatusClear(pUSBDrvObj->interruptSource);
            PLIB_USBHS_InterruptEnableSet(pUSBDrvObj->usbID, 0x0, 0x0, 0x0);
            if(pUSBDrvObj->rootHubInfo.portPowerEnable != NULL)
            {
                /* This USB module has only one port. So we call this function
                 * once to disable the port power on port 0*/

                pUSBDrvObj->rootHubInfo.portPowerEnable(0 /* Port 0 */, false); 
            }
        }
    }
}

bool DRV_USBHS_HOST_ROOT_HUB_OperationIsEnabled(DRV_HANDLE hClient)
{
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;

    if((hClient == DRV_HANDLE_INVALID) || (((DRV_USBHS_OBJ *)hClient) == NULL))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Invalid client");
    }

    hDriver = (DRV_USBHS_OBJ *)hClient;
    usbID = hDriver->usbID;
  
    /* This function sets the session bit if the VBUS voltage
     * level is correct and then returns true. */

    if(PLIB_USBHS_VBUSLevelGet(usbID) == USBHS_VBUS_VALID)
    {
        PLIB_USBHS_SessionEnable(usbID);
        return(true);
     
    }
    /* The VBUS voltage level is not yet set. */
    return(false);
    
}/* end of DRV_USBHS_HOST_OperationIsEnabled() */

// ****************************************************************************
/* Function:
    void DRV_USBHS_HOST_ROOT_HUB_Initialize
    (
        DRV_HANDLE handle,
        USB_HOST_DEVICE_OBJ_HANDLE usbHostDeviceInfo,
    )

  Summary:
    This function instantiates the root hub driver.

  Description:
    This function initializes the root hub driver. It is called by the host
    layer at the time of processing root hub devices. The host layer assigns a
    USB_HOST_DEVICE_OBJ_HANDLE reference to this root hub driver. This
    identifies the relationship between the root hub and the host layer.

  Remarks:
    None.
*/

void DRV_USBHS_HOST_ROOT_HUB_Initialize
(
    DRV_HANDLE handle,
    USB_HOST_DEVICE_OBJ_HANDLE usbHostDeviceInfo
)
{
    DRV_USBHS_OBJ * pUSBDrvObj = (DRV_USBHS_OBJ *)handle;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        pUSBDrvObj->usbHostDeviceInfo = usbHostDeviceInfo;
    }
}

// ****************************************************************************
/* Function:
    uint8_t DRV_USBHS_HOST_ROOT_HUB_PortNumbersGet(DRV_HANDLE handle);

  Summary:
    Returns the number of ports this root hub contains.

  Description:
    This function returns the number of ports that this root hub contains.

  Remarks:
    None.
*/

uint8_t DRV_USBHS_HOST_ROOT_HUB_PortNumbersGet(DRV_HANDLE handle)
{
    DRV_USBHS_OBJ * pUSBDrvObj = (DRV_USBHS_OBJ *)handle;
    uint8_t result = 0;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        result = 1;
    }

    return(result);
}

// ****************************************************************************
/* Function:
    uint32_t DRV_USBHS_HOST_ROOT_HUB_MaximumCurrentGet(DRV_HANDLE);

  Summary:
    Returns the maximum amount of current that this root hub can provide on the
    bus.

  Description:
    This function returns the maximum amount of current that this root hubn can
    provide on the bus.

  Remarks:
    Refer to drv_usbfs.h for usage information.
*/

uint32_t DRV_USBHS_HOST_ROOT_HUB_MaximumCurrentGet(DRV_HANDLE handle)
{
    DRV_USBHS_OBJ * pUSBDrvObj = (DRV_USBHS_OBJ *)handle;
    uint32_t result = 0;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        result = pUSBDrvObj->rootHubInfo.rootHubAvailableCurrent;
    }

    return(result);
}

// ****************************************************************************
/* Function:
    USB_SPEED DRV_USBHS_HOST_ROOT_HUB_BusSpeedGet(DRV_HANDLE handle);

  Summary:
    Returns the speed at which the bus to which this root hub is connected is
    operating.

  Description:
    This function returns the speed at which the bus to which this root hub is
    connected is operating.

 Remarks:
    None.
*/

USB_SPEED DRV_USBHS_HOST_ROOT_HUB_BusSpeedGet(DRV_HANDLE handle)
{
    DRV_USBHS_OBJ * pUSBDrvObj = (DRV_USBHS_OBJ *)handle;
    USB_SPEED speed = USB_SPEED_ERROR;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");

    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        /* Return the bus speed. This is speed at which the root hub is
         * operating. */
        speed = pUSBDrvObj->operationSpeed;
    }

    return(speed);
}

// ****************************************************************************
/* Function:
    void DRV_USBHS_ROOT_HUB_PortResume(DRV_HANDLE handle, uint8_t port );

  Summary:
    Resumes the specified root hub port.

  Description:
    This function resumes the root hub. The resume duration is defined by
    DRV_USBHS_ROOT_HUB_RESUME_DURATION. The status of the resume signalling can
    be checked using the DRV_USBHS_ROOT_HUB_PortResumeIsComplete() function.

  Remarks:
    The root hub on this particular hardware only contains one port - port 0.
*/

USB_ERROR DRV_USBHS_HOST_ROOT_HUB_PortResume(DRV_HANDLE handle, uint8_t port)
{
    /* The functionality is yet to be implemented. */
    return(USB_ERROR_NONE);
}

// ****************************************************************************
/* Function:
    void DRV_USBHS_ROOT_HUB_PortSuspend(DRV_HANDLE handle, uint8_t port );

  Summary:
    Suspends the specified root hub port.

  Description:
    This function suspends the root hub port.

  Remarks:
    The root hub on this particular hardware only contains one port - port 0.
*/

USB_ERROR DRV_USBHS_HOST_ROOT_HUB_PortSuspend(DRV_HANDLE handle, uint8_t port)
{
    /* The functionality is yet to be implemented. */
    return (USB_ERROR_NONE);
}

// ****************************************************************************
/* Function:
    void DRV_USBHS_ROOT_HUB_PortResetIsComplete
    (
        DRV_HANDLE handle,
        uint8_t port
    );

  Summary:
    Returns true if the root hub has completed the port reset operation.

  Description:
    This function returns true if the port reset operation has completed. It
    should be called after the DRV_USB_HOST_ROOT_HUB_PortReset() function to
    check if the reset operation has completed.

  Remarks:
    Refer to drv_usbfs.h for usage information.
 */

bool DRV_USBHS_HOST_ROOT_HUB_PortResetIsComplete
(
    DRV_HANDLE handle,
    uint8_t port
)
{
    DRV_USBHS_OBJ * pUSBDrvObj = (DRV_USBHS_OBJ *)handle;
    bool result = true;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");

    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        /* Return false if the driver is still resetting*/
        result = (pUSBDrvObj->isResetting) ? false : true;
    }

    return(result);
}

// ****************************************************************************
/* Function:
    void DRV_USBHS_ROOT_HUB_PortReset(DRV_HANDLE handle, uint8_t port );
    
  Summary:
    Resets the specified root hub port.
	
  Description:
    This function resets the root hub port. The reset duration is defined by
    DRV_USBHS_ROOT_HUB_RESET_DURATION. The status of the reset signalling can be
    checked using the DRV_USBHS_ROOT_HUB_PortResetIsComplete() function.
	
  Remarks:
    Refer to drv_usbfs.h for usage information.
*/

USB_ERROR DRV_USBHS_HOST_ROOT_HUB_PortReset(DRV_HANDLE handle, uint8_t port)
{
    DRV_USBHS_OBJ * pUSBDrvObj = (DRV_USBHS_OBJ *)handle;
    USB_ERROR result = USB_ERROR_NONE;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
        result = USB_ERROR_PARAMETER_INVALID;

    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
        result = USB_ERROR_PARAMETER_INVALID;
    }
    else if(pUSBDrvObj->isResetting)
    {
        /* This means a reset is alredy in progress. Lets not do anything. */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Reset already in progress");

    }
    else
    {
        /* Start the reset signal. Set the driver flag to indicate the reset
         * signal is in progress. Enable the 1 millisecond timer interrupt to
         * count the reset duration. Clear the reset duration counter. This
         * counter will be updated in the interrupt. Start generating the reset
         * signal. */
        
        pUSBDrvObj->isResetting = true;
        pUSBDrvObj->resetState = DRV_USBHS_HOST_RESET_STATE_START;
        PLIB_USBHS_ResetEnable(pUSBDrvObj->usbID);
    }

    return(result);
}

// ****************************************************************************
/* Function:
    USB_SPEED DRV_USBHS_HOST_ROOT_HUB_PortSpeedGet
    (
        DRV_HANDLE handle,
        uint8_t port
    );

  Summary:
    Returns the speed of at which the port is operating.

  Description:
    This function returns the speed at which the port is operating.

  Remarks:
    Refer to drv_usbhs.h for usage information.
*/

USB_SPEED DRV_USBHS_HOST_ROOT_HUB_PortSpeedGet(DRV_HANDLE handle, uint8_t port)
{
    DRV_USBHS_OBJ * pUSBDrvObj = (DRV_USBHS_OBJ *)handle;
    USB_SPEED speed = USB_SPEED_ERROR;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        /* The driver will not check if a device is connected. It is assumed
         * that the client has issued a port reset before calling this function
         */

        speed = pUSBDrvObj->deviceSpeed;
    }

    return(speed);
}

