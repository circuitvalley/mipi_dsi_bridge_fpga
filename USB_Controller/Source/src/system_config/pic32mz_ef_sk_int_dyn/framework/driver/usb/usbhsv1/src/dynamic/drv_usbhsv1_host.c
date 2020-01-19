/*******************************************************************************
  USB Host Controller Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhsv1_host.c

  Summary:
    USB Host Driver Implementation for PIC32C.

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
#include "driver/usb/usbhsv1/drv_usbhsv1.h"
#include "driver/usb/usbhsv1/src/drv_usbhsv1_local.h"
#include "usb/usb_host.h"
#include "usb/usb_host_client_driver.h"

/**********************************************************
 * This structure is a set of pointer to the USBFS driver
 * functions. It is provided to the host and device layer
 * as the interface to the driver.
 * *******************************************************/

DRV_USB_HOST_INTERFACE gDrvUSBHSV1HostInterface =
{
    .open = DRV_USBHSV1_Open,
    .close = DRV_USBHSV1_Close,
    .eventHandlerSet = DRV_USBHSV1_ClientEventCallBackSet,
    .hostIRPSubmit = DRV_USBHSV1_HOST_IRPSubmit,
    .hostIRPCancel = DRV_USBHSV1_HOST_IRPCancel,
    .hostPipeSetup = DRV_USBHSV1_HOST_PipeSetup,
    .hostPipeClose = DRV_USBHSV1_HOST_PipeClose,
    .hostEventsDisable = DRV_USBHSV1_HOST_EventsDisable,
    .hostEventsEnable = DRV_USBHSV1_HOST_EventsEnable,
    .rootHubInterface.rootHubPortInterface.hubPortReset = DRV_USBHSV1_HOST_ROOT_HUB_PortReset,
    .rootHubInterface.rootHubPortInterface.hubPortSpeedGet = DRV_USBHSV1_HOST_ROOT_HUB_PortSpeedGet,
    .rootHubInterface.rootHubPortInterface.hubPortResetIsComplete = DRV_USBHSV1_HOST_ROOT_HUB_PortResetIsComplete,
    .rootHubInterface.rootHubPortInterface.hubPortSuspend = DRV_USBHSV1_HOST_ROOT_HUB_PortSuspend,
    .rootHubInterface.rootHubPortInterface.hubPortResume = DRV_USBHSV1_HOST_ROOT_HUB_PortResume,
    .rootHubInterface.rootHubMaxCurrentGet = DRV_USBHSV1_HOST_ROOT_HUB_MaximumCurrentGet,
    .rootHubInterface.rootHubPortNumbersGet = DRV_USBHSV1_HOST_ROOT_HUB_PortNumbersGet,
    .rootHubInterface.rootHubSpeedGet = DRV_USBHSV1_HOST_ROOT_HUB_BusSpeedGet,
    .rootHubInterface.rootHubInitialize = DRV_USBHSV1_HOST_ROOT_HUB_Initialize,
    .rootHubInterface.rootHubOperationEnable = DRV_USBHSV1_HOST_ROOT_HUB_OperationEnable,
    .rootHubInterface.rootHubOperationIsEnabled = DRV_USBHSV1_HOST_ROOT_HUB_OperationIsEnabled,
};

/*****************************************
 * Pool of pipe objects that is used by
 * all driver instances.
 *****************************************/
DRV_USBHSV1_HOST_PIPE_OBJ gDrvUSBHostPipeObj[10];

/****************************************
 * The driver object
 ****************************************/
extern DRV_USBHSV1_OBJ gDrvUSBObj[];

// ****************************************************************************
// ****************************************************************************
// Local Functions
// ****************************************************************************
// ****************************************************************************

void _DRV_USBHSV1_HOST_AttachDetachStateMachine (DRV_USBHSV1_OBJ * hDriver)
{
    /* In the host mode, we perform attach de-bouncing */
    
    bool interruptWasEnabled;
	
    switch(hDriver->attachState)
    {
        case DRV_USBHSV1_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH:
            /* If no device is attached, then there is nothing to do
             * If device is attached, then move to next state */
            if(hDriver->deviceAttached)
            {
                hDriver->attachState = DRV_USBHSV1_HOST_ATTACH_STATE_DETECTED;
            }

            break;
            
        case DRV_USBHSV1_HOST_ATTACH_STATE_DETECTED:
                /* Disable the driver interrupt as
                 * we do not want this section to be interrupted. */
                interruptWasEnabled = _DRV_USBHSV1_InterruptSourceDisable(hDriver->interruptSource);

                if(hDriver->deviceAttached)
                {
                    /* Yes the device is still attached. Enumerate
                     * this device. usbHostDeviceInfo is the ID of
                     * this root hub. */
                    hDriver->attachedDeviceObjHandle = USB_HOST_DeviceEnumerate(hDriver->usbHostDeviceInfo, 0);
                    hDriver->attachState = DRV_USBHSV1_HOST_ATTACH_STATE_READY;

                }
                else
                {
                    /* The device is not attached any more. This was a false attach 
                     */
                    hDriver->attachState = DRV_USBHSV1_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH;
                }

                if(interruptWasEnabled)
                {
                    /* Re-enable the interrupt if it was originally
                     * enabled. */
                    _DRV_USBHSV1_InterruptSourceEnable(hDriver->interruptSource);
                }

            break;

        case DRV_USBHSV1_HOST_ATTACH_STATE_READY:

            /* De-bouncing is done and device ready. We can check
             * here if the device is detached */
            if(!hDriver->deviceAttached)
            {
                /* Device is not attached */
                hDriver->attachState = DRV_USBHSV1_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH;
            }
            break;

        default:
            break;
    }
}

void _DRV_USBHSV1_HOST_ResetStateMachine(DRV_USBHSV1_OBJ * hDriver)
{
    /* Check if reset is needed */
    switch(hDriver->resetState)
    {
        case DRV_USBHSV1_HOST_RESET_STATE_NO_RESET:

            /* No reset signaling is request */
        break;

        case DRV_USBHSV1_HOST_RESET_STATE_START:
            /* Trigger USB Reset */
            hDriver->usbID->USBHS_HSTCTRL.RESET = 1;
            hDriver->resetState = DRV_USBHSV1_HOST_RESET_STATE_WAIT_FOR_COMPLETE;
        break;

        case DRV_USBHSV1_HOST_RESET_STATE_WAIT_FOR_COMPLETE:

            /* Check if the reset has completed */
            if(0 == hDriver->usbID->USBHS_HSTCTRL.RESET)
            {
                /* Reset has completed */
                hDriver->resetState = DRV_USBHSV1_HOST_RESET_STATE_NO_RESET;

                /* Clear the flag */
                hDriver->isResetting = false;

                /* Now that reset is complete, we can find out the
                 * speed of the attached device. */
                switch (hDriver->usbID->USBHS_SR.SPEED) {
        	        case 0x0:
        	            hDriver->deviceSpeed = USB_SPEED_FULL;
        	        break;
        	        case 0x1:
            	        hDriver->deviceSpeed = USB_SPEED_HIGH;
        	        break;
        	        case 0x2:
        	            hDriver->deviceSpeed = USB_SPEED_LOW;
                    break;    
        	        default:
        	        break;
    	        }
            }
        break;

        default:
        break;
    }
}

void _DRV_USBHSV1_HOST_IRPTransmitFIFOLoad
(
    volatile usbhs_registers_t * usbID, 
    USB_HOST_IRP_LOCAL * irp,
    uint8_t hPipe
)
{
    /* This function will copy data from the irp to the fifo
     * base on the number of bytes that were completed and
     * then trigger the transmit */

    uint8_t *data, *ptrEPData;
    unsigned int count, pendingBytes;
    DRV_USBHSV1_HOST_PIPE_OBJ * pipe = (DRV_USBHSV1_HOST_PIPE_OBJ *)(irp->pipe);
    uint32_t hstPipeCfg;
    
    /* Load the FIFO */
    pendingBytes = irp->size - irp->completedBytes;
    count = (pendingBytes > pipe->endpointSize) ? pipe->endpointSize : pendingBytes;
    data = (uint8_t *)((uint8_t *)irp->data + irp->completedBytes);
    ptrEPData = (uint8_t *) &drv_usbhsv1_get_pipe_fifo_access(hPipe);
   
    if(0 == hPipe)
    {
        /* Configure OUT Token for Pipe 0 */
        hstPipeCfg = usbID->USBHS_HSTPIPCFG[0].w & ~(uint32_t)(USBHS_HSTPIPCFG_PTOKEN_Msk);
        hstPipeCfg |= USBHS_HSTPIPCFG_PTOKEN(2);
        usbID->USBHS_HSTPIPCFG[0].w = hstPipeCfg;
    }

    /* Load the endpoint FIFO with the user data */
    for(uint16_t i = 0; i < count; i ++)
    {
	    *ptrEPData++ = *data++;
    }
    /* Update the irp with the byte count loaded */
    irp->completedBytes += count;

    /* Enable Pipe out ready interrupt */
    usbID->USBHS_HSTPIPIER[hPipe].w = USBHS_HSTPIPIER_TXOUTES_Msk;
    /* Clear FIFOCON and Unfreeze pipe */
    usbID->USBHS_HSTPIPIDR[hPipe].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);

}/* end of _DRV_USBHSV1_HOST_IRPTransmitFIFOLoad() */

void _DRV_USBHSV1_HOST_IRPTransmitSetupPacket
(
    volatile usbhs_registers_t * usbID,
    USB_HOST_IRP_LOCAL * irp
)
{
    /* This function will load the irp setup packet into 
     * pipe0 FIFO and then transmit the setup packet. */

    uint8_t * data = (uint8_t *)irp->setup;
	volatile uint8_t *ptrEPData;
    uint32_t hstPipeCfg;
    
    /* Configure Setup Token for Pipe 0 */
    hstPipeCfg = usbID->USBHS_HSTPIPCFG[0].w & ~(uint32_t)(USBHS_HSTPIPCFG_PTOKEN_Msk);
    hstPipeCfg |= USBHS_HSTPIPCFG_PTOKEN(0);
    usbID->USBHS_HSTPIPCFG[0].w = hstPipeCfg;
    /* Clear Setup Ready interrupt */
	usbID->USBHS_HSTPIPICR[0].TXSTPIC = 1; 
	ptrEPData =   (volatile uint8_t *)&drv_usbhsv1_get_pipe_fifo_access(0);

   /* Load the endpoint FIFO with the user data */
    for(uint16_t i = 0; i < 8; i ++)
    {
        *ptrEPData++ = *data++;
    }
   
    /* Enable setup ready interrupt */
    usbID->USBHS_HSTPIPIER[0].w = USBHS_HSTPIPIER_TXSTPES_Msk;
    /* Clear FIFOCON and Unfreeze pipe */
    usbID->USBHS_HSTPIPIDR[0].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);

}/* end of _DRV_USBHSV1_HOST_IRPTransmitSetupPacket() */

unsigned int _DRV_USBHSV1_HOST_IRPReceiveFIFOUnload 
(
    volatile usbhs_registers_t * usbID,
    USB_HOST_IRP_LOCAL * irp,
    uint8_t hPipe,
    bool * pisDMAUsed
)
{
    /* This function will recover the count of the received data/
     * and then unload the pipe FIFO. */

    uint32_t  count;
    uint8_t * data;
    uint8_t * ptrEPData;

    /* Copy the data from the FIFO0 to the application
     * buffer and then update the complete byte count
     * and clear the RX packet ready bit */
	ptrEPData = (uint8_t *) &drv_usbhsv1_get_pipe_fifo_access(hPipe);
    data = (uint8_t *)((uint8_t *)irp->data + irp->completedBytes);
	/* Get byte count to read data */
	count = usbID->USBHS_HSTPIPISR[hPipe].PBYCT;

    for(uint16_t i = 0; i < count; i ++)
    {
	    *data++ = *ptrEPData++;
    }
    irp->completedBytes += count;
    *pisDMAUsed = false;
    
    /* Clear FIFO Status */
    usbID->USBHS_HSTPIPIDR[hPipe].w = USBHS_HSTPIPIDR_FIFOCONC_Msk;
    return (count);
}/* end of _DRV_USBHSV1_HOST_IRPReceiveFIFOUnload() */

void _DRV_USBHSV1_HOST_Initialize
(
    DRV_USBHSV1_OBJ * drvObj, 
    SYS_MODULE_INDEX index
)
{
    volatile usbhs_registers_t * usbMod = drvObj->usbID;
   
    /* No device attached */
    drvObj->deviceAttached = false;
    /* Initialize the device handle */
    drvObj->attachedDeviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
    /* Set VBUS Hardware Control */
    usbMod->USBHS_CTRL.VBUSHWC = 1;
    /* Initialize the host specific members in the driver object */
    drvObj->isResetting     = false;
    drvObj->usbHostDeviceInfo = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;
    drvObj->operationEnabled = false;
   
}/* end of _DRV_USBHSV1_HOST_Initialize() */

USB_ERROR DRV_USBHSV1_HOST_IRPSubmit
(
    DRV_USBHSV1_HOST_PIPE_HANDLE  hPipe, 
    USB_HOST_IRP * inputIRP
)
{
    USB_HOST_IRP_LOCAL * irpIterator;
    DRV_USBHSV1_HOST_TRANSFER_GROUP * controlTransferGroup;
    bool interruptWasEnabled = false;
    unsigned int direction;
    uint8_t hostPipe;

    USB_HOST_IRP_LOCAL * irp        = (USB_HOST_IRP_LOCAL *)inputIRP;
    DRV_USBHSV1_HOST_PIPE_OBJ * pipe = (DRV_USBHSV1_HOST_PIPE_OBJ *)(hPipe);
    DRV_USBHSV1_OBJ * hDriver;
    volatile usbhs_registers_t * usbMod;

    if((pipe == NULL) || (hPipe == (DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID)))
    {
        /* This means an invalid pipe was specified. 
         * Return with an error */

        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Pipe handle is not valid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = (DRV_USBHSV1_OBJ *)(pipe->hClient);
    usbMod = hDriver->usbID;
    controlTransferGroup = &hDriver->controlTransferGroup;

    /* Assign owner pipe */
    irp->pipe = hPipe;
    irp->status = USB_HOST_IRP_STATUS_PENDING;
    irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_PROCESSING;
    hostPipe = pipe->hostPipeN;
    direction = (pipe->endpointAndDirection & 0x80) >> 7;

    /* We need to disable interrupts was the queue state
     * does not change asynchronously */

    if(!hDriver->isInInterruptContext)
    {
        /* OSAL: Get Mutex */
        if(OSAL_MUTEX_Lock(&(hDriver->mutexID), OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }
        interruptWasEnabled = _DRV_USBHSV1_InterruptSourceDisable(hDriver->interruptSource);
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
            irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_SETUP_STAGE;

            /* We need to update the flags parameter of the IRP
             * to indicate the direction of the control transfer. */

            if(*((uint8_t*)(irp->setup)) & 0x80)
            {
                /* This means the data stage moves from device to 
                 * host. Set bit 15 of the flags parameter */

                irp->flags |= 0x80;
            }
            else
            {
                /* This means the data stage moves from host to
                 * device. Clear bit 15 of the flags parameter. */

                irp->flags &= 0x7F;
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
                _DRV_USBHSV1_HOST_IRPTransmitSetupPacket(usbMod, irp);
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
                
                /* Clear Tx Out Ready Interrupt */
                usbMod->USBHS_HSTPIPICR[hostPipe].w = USBHS_HSTPIPICR_TXOUTIC_Msk;
                _DRV_USBHSV1_HOST_IRPTransmitFIFOLoad(usbMod, irp, hostPipe);
            }
            else
            {
                /* Data is moving from device to host
                 * We need to set the Rx Packet Request
                 * bit */
                
                /* Perform 1 IN requests before freezing the pipe. */
                usbMod->USBHS_HSTPIPINRQ[hostPipe].w = 0;
                /* Clear RX IN Interrupt */
                usbMod->USBHS_HSTPIPICR[hostPipe].w = USBHS_HSTPIPICR_RXINIC_Msk;
                /* Enable Rx IN Interrupt */
                usbMod->USBHS_HSTPIPIER[hostPipe].w = USBHS_HSTPIPIER_RXINES_Msk;
                /* Clear FIFOCON and Unfreeze pipe */
                usbMod->USBHS_HSTPIPIDR[hostPipe].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);
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
            _DRV_USBHSV1_InterruptSourceEnable(hDriver->interruptSource);
        }
        /* OSAL: Return Mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex unlock failed");
        }
    }

    return USB_ERROR_NONE;
}/* end of DRV_USBHSV1_HOST_IRPSubmit() */

void DRV_USBHSV1_HOST_IRPCancel(USB_HOST_IRP * inputIRP)
{
    /* This function cancels an IRP */

    USB_HOST_IRP_LOCAL * irp = (USB_HOST_IRP_LOCAL *) inputIRP;
    DRV_USBHSV1_OBJ * hDriver;
    DRV_USBHSV1_HOST_PIPE_OBJ * pipe;
    bool interruptWasEnabled = false;

    if(irp->pipe == DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Invalid pipe");
        return;
    }

    if(irp->status <= USB_HOST_IRP_STATUS_COMPLETED_SHORT)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: IRP is not pending or in progress");
        return;
    }

    pipe = (DRV_USBHSV1_HOST_PIPE_OBJ *)irp->pipe;
    hDriver = (DRV_USBHSV1_OBJ *) pipe->hClient;

    if(!hDriver->isInInterruptContext)
    {
        /* OSAL: Get Mutex */
        if(OSAL_MUTEX_Lock(&(hDriver->mutexID), OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex lock failed");
        }
        interruptWasEnabled = _DRV_USBHSV1_InterruptSourceDisable(hDriver->interruptSource);
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
         * caught in _DRV_USBHSV1_HOST_ControlXferProcess()
         * and _DRV_USBHSV1_HOST_NonControlIRPProcess()
         * functions. */

        irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_ABORTED;
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
            _DRV_USBHSV1_InterruptSourceEnable(hDriver->interruptSource);
        }

        /* OSAL: Release Mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex unlock failed");
        }
    }
}/* end of DRV_USBHSV1_HOST_IRPCancel() */

void DRV_USBHSV1_HOST_PipeClose
(
    DRV_USBHSV1_HOST_PIPE_HANDLE pipeHandle
)
{
    /* This function closes an open pipe */

    bool interruptWasEnabled = false;

    DRV_USBHSV1_OBJ * hDriver;
    USB_HOST_IRP_LOCAL  * irp;
    DRV_USBHSV1_HOST_PIPE_OBJ       * pipe;
    DRV_USBHSV1_HOST_TRANSFER_GROUP * transferGroup;
    DRV_USBHSV1_HOST_ENDPOINT_OBJ   * endpointObj;
    volatile usbhs_registers_t * usbMod;

    /* Make sure we have a valid pipe */
    if( ( pipeHandle == 0 )  || pipeHandle == DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Invalid pipe handle");
        return;
    }

    pipe = (DRV_USBHSV1_HOST_PIPE_OBJ*) pipeHandle;

    /* Make sure that we are working with a pipe in use */
    if(pipe->inUse != true)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Pipe is not in use");
        return;
    }

    hDriver = (DRV_USBHSV1_OBJ *)pipe->hClient;
    usbMod = hDriver->usbID;

    /* Disable the interrupt */

    if(!hDriver->isInInterruptContext)
    {
        /* OSAL: Get Mutex */
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex lock failed");
        }
        interruptWasEnabled = _DRV_USBHSV1_InterruptSourceDisable(hDriver->interruptSource);
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

        endpointObj = &hDriver->hostEndpointTable[pipe->hostPipeN];
        endpointObj->endpoint.inUse = false;
        endpointObj->endpoint.pipe = NULL;

        /* Clear the error status */
        usbMod->USBHS_HSTPIPERR[pipe->hostPipeN].w = 0;
    }

    /* Now we invoke the call back for each IRP in this pipe and say that it is
     * aborted.  If the IRP is in progress, then that IRP will be actually
     * aborted on the next SOF This will allow the USB module to complete any
     * transaction that was in progress. */

    irp = (USB_HOST_IRP_LOCAL *)pipe->irpQueueHead;
    while(irp != NULL)
    {
        irp->pipe = DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID;

        if(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)
        {
            /* If the IRP is in progress, then we set the temp IRP state. This
             * will be caught in the _DRV_USBHSV1_HOST_NonControlTransferProcess() and
             * _DRV_USBHSV1_HOST_ControlTransferProcess() functions */
            irp->status = USB_HOST_IRP_STATUS_ABORTED;
            if(irp->callback != NULL)
            {
                irp->callback((USB_HOST_IRP*)irp);
            }
            irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_ABORTED;
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

    /* Now we return the pipe back to the driver */
    pipe->inUse = false ;

    /* Enable the interrupts */
    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USBHSV1_InterruptSourceEnable(hDriver->interruptSource);
        }

        /* OSAL: Return Mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex unlock failed");
        }
    }
}/* end of DRV_USBHSV1_HOST_PipeClose() */

DRV_USBHSV1_HOST_PIPE_HANDLE DRV_USBHSV1_HOST_PipeSetup
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
    bool epFound = false;
    uint8_t endpoint;
    unsigned int epDirection;
    int pipeIter;
    uint32_t hstPipeCfg;

    DRV_USBHSV1_OBJ * hDriver;
    DRV_USBHSV1_HOST_PIPE_OBJ * pipe;
  
    if(client == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Invalid client handle");
        return DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID;
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
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Invalid pipe endpoint size");
        return DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID;
    }

    hDriver = (DRV_USBHSV1_OBJ *)client;
    endpoint = (endpointAndDirection & 0x0F);
    epDirection = (endpointAndDirection & 0x80) >> 7;

    /* OSAL: Mutex Lock */
    if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex lock failed");
        return USB_ERROR_OSAL_FUNCTION;
    }
    
    
    if(pipeType == USB_TRANSFER_TYPE_CONTROL)
    {
        /* Set pipeIter to zero to indicate that we must use pipe 0
         * for control transfers. We also add the control transfer pipe
         * to the control transfer group. */
        pipeIter = 0;

	    if (wMaxPacketSize < 8)
        {
    	    return DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID;
	    }
        /* Reset Pipe */
        Set_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (pipeIter)));
        Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (pipeIter)));
                
        /* Enable Pipe */
        Set_bits(hDriver->usbID->USBHS_HSTPIP.w, (1 << (pipeIter)));

        /* Configure Pipe */
        hstPipeCfg = ((0 << 2)|((drv_usbhsv1_format_pipe_size(wMaxPacketSize) & 7) << 4)|
                      (0 << 8)|(0 << 10)|(0 << 12)|(pipeIter << 16)|(0 << 24));

        /* Enable PINGEN for HS Control Pipe */ 
        if((USB_TRANSFER_TYPE_CONTROL == pipeType) && \
           (USB_SPEED_HIGH == hDriver->deviceSpeed))
        {
            hstPipeCfg |= (1<<20);
        }
        
        /* Allocate the Pipe Memory */
        hstPipeCfg |= (USBHS_HSTPIPCFG_ALLOC_Msk);
        hDriver->usbID->USBHS_HSTPIPCFG[pipeIter].w  = hstPipeCfg;

        /* Check if Pipe configuration status is OK */
        if(false == hDriver->usbID->USBHS_HSTPIPISR[pipeIter].CFGOK)
        {
            /* Disable Pipe */
            Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, (1 << (pipeIter)));
            return DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID;
        }
        /* Configure address of Pipe */
        Wr_bitfield((&hDriver->usbID->USBHS_HSTADDR1)[(pipeIter)>>2].w, 0x7F << (((pipeIter)&0x03)<<3), deviceAddress);

        /* Always enable stall and error interrupts of control endpoint */
        /* Enable Stall Interrupt */
        hDriver->usbID->USBHS_HSTPIPIER[pipeIter].RXSTALLDES = 1;
        /* Enable Pipe Error Interrupt */
        hDriver->usbID->USBHS_HSTPIPIER[pipeIter].PERRES = 1;
        /* Enable Pipe Interrupt */
        Set_bits(hDriver->usbID->USBHS_HSTIER.w, ((1 << 8) << (pipeIter)));
        epFound = true;
        pipe = &gDrvUSBHostPipeObj[pipeIter];
    }
    else
    {
        /* Pipe allocation for non-control transfer */
        for(pipeIter = 1; pipeIter < 10; pipeIter ++)
        {
            if((false == gDrvUSBHostPipeObj[pipeIter].inUse) && \
               (false == Tst_bits(hDriver->usbID->USBHS_HSTPIP.w, (1 << (pipeIter)))))
            {
                /* This means we have found a free pipe object */
                uint16_t ep_dir;
                ep_dir = (epDirection == USB_DATA_DIRECTION_DEVICE_TO_HOST)?(0x1):(0x2);
                /* Reset Pipe*/
                Set_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (pipeIter)));
                Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (pipeIter)));
        
                /* Enable Pipe */
                Set_bits(hDriver->usbID->USBHS_HSTPIP.w, (1 << (pipeIter)));
                
                /* Configure Pipe */
                hstPipeCfg = ((0 << 2)|((drv_usbhsv1_format_pipe_size(wMaxPacketSize) & 7) << 4)|
                (ep_dir << 8)|(0 << 10)|(pipeType << 12)|(endpoint << 16)|(bInterval << 24));
  
                /* Enable PINGEN for HS BULK-OUT Pipe */ 
                if((0x2 == ep_dir) && (USB_TRANSFER_TYPE_BULK == pipeType) && \
                   (USB_SPEED_HIGH == hDriver->deviceSpeed))
                {
                    hstPipeCfg |= (1<<20);
                }
        
                /* Allocate the Pipe Memory */
                hstPipeCfg |= (USBHS_HSTPIPCFG_ALLOC_Msk);
                hDriver->usbID->USBHS_HSTPIPCFG[pipeIter].w  = hstPipeCfg;                

                /* Check if Pipe configuration status is OK */
                if(false == hDriver->usbID->USBHS_HSTPIPISR[pipeIter].CFGOK)
                {
                    /* Disable Pipe */
                    Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, (1 << (pipeIter)));
                    return DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID;
                }
                
                /* Configure address of Pipe */
                Wr_bitfield((&hDriver->usbID->USBHS_HSTADDR1)[(pipeIter)>>2].w, 0x7F << (((pipeIter)&0x03)<<3), deviceAddress);
                
                /* Enable Stall Interrupt */
                hDriver->usbID->USBHS_HSTPIPIER[pipeIter].RXSTALLDES = 1;
                /* Enable Pipe Error Interrupt */
                hDriver->usbID->USBHS_HSTPIPIER[pipeIter].PERRES = 1;
                /* Enable Pipe Interrupt */
                Set_bits(hDriver->usbID->USBHS_HSTIER.w, ((1 << 8) << (pipeIter)));
                
                epFound = true;
                pipe = &gDrvUSBHostPipeObj[pipeIter];
                hDriver->hostEndpointTable[pipeIter].endpoint.inUse = true;
                hDriver->hostEndpointTable[pipeIter].endpoint.pipe = pipe;                
                break;
            }                            
        }            
    }    

    if(!epFound)
    {
        /* This means we could not find a spare endpoint for this
         * non control transfer. */

        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Could not allocate endpoint");
        /* OSAL: Mutex Unlock */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex unlock failed");
        }
        return DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID;
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
    pipe->hostPipeN     = pipeIter;
    pipe->endpointAndDirection   = endpointAndDirection;

    /* OSAL: Release Mutex */
    if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Mutex unlock failed");
    }
    return((DRV_USBHSV1_HOST_PIPE_HANDLE)pipe);

}/* end of DRV_USBHSV1_HOST_PipeSetup() */

void _DRV_USBHSV1_HOST_ControlTransferProcess(DRV_USBHSV1_OBJ * hDriver)
{
    /* This function is called every time there is an endpoint 0
     * interrupt. This means that a stage of the current control IRP has been
     * completed. This function is called from an interrupt context */

    USB_HOST_IRP_LOCAL * irp;
    DRV_USBHSV1_HOST_PIPE_OBJ * pipe, * iterPipe;
    DRV_USBHSV1_HOST_TRANSFER_GROUP * transferGroup;
    bool endIRP = false;
    unsigned int count, i;
    bool foundIRP = false;
    bool isDmaUsed = false;
    volatile usbhs_registers_t * usbMod;
    uint32_t hstPipeCfg;

    transferGroup = &hDriver->controlTransferGroup;
    usbMod = hDriver->usbID;

    /* First check if the IRP was aborted */
    irp = transferGroup->currentIRP;
    pipe = transferGroup->currentPipe;

    /* If current IRP is null, or current pipe is null then we have unknown
     * failure. We just quit.  Nothing we can do.*/

    if((irp == NULL) || (pipe == NULL) ||
            (pipe == (DRV_USBHSV1_HOST_PIPE_OBJ *)DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID))
    {
        /* Unknown error */
        return;
    }

	/* Disable setup, IN and OUT interrupts of control endpoint */
	usbMod->USBHS_HSTPIPIDR[0].w = (USBHS_HSTPIPIDR_TXSTPEC_Msk | USBHS_HSTPIPIDR_RXINEC_Msk | USBHS_HSTPIPIDR_TXOUTEC_Msk);

    /* If here means, we have a valid IRP and pipe.  Check the status register.
     * The IRP could have been aborted. This would be known in the temp state.
     */

    if(irp->tempState == DRV_USBHSV1_HOST_IRP_STATE_ABORTED)
    {
        /* This means the application has aborted this
         IRP.*/

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ABORTED;
        /* Reset Pipe*/
        Set_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (0)));
        Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (0)));			
    }
    else if(false != usbMod->USBHS_HSTPIPISR[0].RXSTALLDI)
    {
        /* This means the packet was stalled. Set the error status and then
         * clear the stall bit */

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
        /* Clear Stall Interrupt */
        usbMod->USBHS_HSTPIPICR[0].RXSTALLDIC = 1;
	    /* Reset DATA Toggle */
        usbMod->USBHS_HSTPIPIER[0].RSTDTS = 1;		
        /* Reset Pipe*/
        Set_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (0)));
        Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (0)));		
    }
    else if(false != usbMod->USBHS_HSTPIPISR[0].PERRI)
    {
        /* This means there was a pipe error. Set the error status and then
		 * clear the error bits */

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ERROR_DATA;
		/* Ack all errors */
        usbMod->USBHS_HSTPIPERR[0].w = 0;
        /* Reset Pipe */
        Set_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (0)));
        Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (0)));			
    }
    else
    {
        /* This means there was no error. We should check the current state of
         * the IRP and then proceed accordingly */

        switch(irp->tempState)
        {
             case DRV_USBHSV1_HOST_IRP_STATE_SETUP_STAGE:
				if (false != usbMod->USBHS_HSTPIPISR[0].TXSTPI)
                {
					 /* SETUP packet sent */
                     /* Freeze Pipe */
					 usbMod->USBHS_HSTPIPIER[0].w = USBHS_HSTPIPIER_PFREEZES_Msk;
                     /* Clear Tx Setup Ready Interrupt */
                     usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_TXSTPIC_Msk;
					 
				}
				else
				{
					return;
				}
 
               /* We got an interrupt after this stage. This means that the
                * setup stage has completed.  We need to check if a data stage
                * is required and then start the data stage. */

               if((irp->data == NULL) || (irp->size == 0))
               {
                   /* This means there is no data stage. We can proceed to the
                    * handshake stage. In a case where there isnt a data stage,
                    * we need to send an IN token to the device */

                    irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_HANDSHAKE_SENT;
				   
                    /* Configure pipe for IN token */
                    hstPipeCfg = usbMod->USBHS_HSTPIPCFG[0].w & ~(uint32_t)(USBHS_HSTPIPCFG_PTOKEN_Msk);
                    hstPipeCfg |= USBHS_HSTPIPCFG_PTOKEN(1);
                    usbMod->USBHS_HSTPIPCFG[0].w = hstPipeCfg;
					/* Clear IN Rx Interrupt */
                    usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_RXINIC_Msk;
 					/* Enable IN Rx Interrupt */
                    usbMod->USBHS_HSTPIPIER[0].w = USBHS_HSTPIPIER_RXINES_Msk;
                    /* Clear FIFOCON and Unfreeze pipe */
                    usbMod->USBHS_HSTPIPIDR[0].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);    
               }
               else
               {
                   /* This means that a data stage is required. We can find out
                    * the direction of the data stage by investigating the flags
                    * parameter of the  IRP. */

                   irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_DATA_STAGE_SENT;
                   if(irp->flags & 0x80)
                   {
                       /* This means the data stage moves from device to host.
                        * So the host would have to send an IN token.  */
                       /* Configure pipe for IN token */
                       hstPipeCfg = usbMod->USBHS_HSTPIPCFG[0].w & ~(uint32_t)(USBHS_HSTPIPCFG_PTOKEN_Msk);
                       hstPipeCfg |= USBHS_HSTPIPCFG_PTOKEN(1);
                       usbMod->USBHS_HSTPIPCFG[0].w = hstPipeCfg;                
                       /* Clear IN Rx Interrupt */
                       usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_RXINIC_Msk;
                       /* Enable IN Rx Interrupt */
                       usbMod->USBHS_HSTPIPIER[0].w = USBHS_HSTPIPIER_RXINES_Msk;
                       /* Clear FIFOCON and Unfreeze pipe */
                       usbMod->USBHS_HSTPIPIDR[0].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);
                   }
                   else
                   {
                       /* This function loads the fifo and sends the packet. The
                        * completed bytes field in the IRP will be updated. */
                       _DRV_USBHSV1_HOST_IRPTransmitFIFOLoad(usbMod, irp, 0);
                   }
               }
               break;

           case DRV_USBHSV1_HOST_IRP_STATE_DATA_STAGE_SENT:

				if (false != usbMod->USBHS_HSTPIPISR[0].RXINI) 
                {
					/* IN packet received */
                    /* In case of low USB speed and with a high CPU frequency,
		             * a ACK from host can be always running on USB line
		             * then wait end of ACK on IN pipe */
                    if (usbMod->USBHS_SR.SPEED == 0x2)
                    {    
                        while(!usbMod->USBHS_HSTPIPIMR[0].PFREEZE);
                    }    
                    /* Clear IN Rx Interrupt */        
					usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_RXINIC_Msk;
				}
				else if (false != usbMod->USBHS_HSTPIPISR[0].TXOUTI) 
                {
					/* OUT packet sent */
                    /* Clear Tx Setup Ready Interrupt */
                    usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_TXOUTIC_Msk;
				}
				else
				{
					return;
				}
	

               /* We reach here after an interrupt which means that a data stage
                * interaction was completed. Find out what was the direction the
                * data stage */

               if(irp->flags & 0x80)
               {
                   /* This means the data was moving from device to host. We got
                    * an interrupt, which means we have received data. Start by
                    * checking how much data we received from the device */

                   count = _DRV_USBHSV1_HOST_IRPReceiveFIFOUnload(usbMod, irp, 0, &isDmaUsed);
                        
                   if((count < pipe->endpointSize) ||
                           (irp->completedBytes >= irp->size))
                   {
                       /* This means that we either received a short packet or
                         * we received the amount of data that we needed. We
                         * should move to the handshake stage. */
                        irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_HANDSHAKE_SENT;
                        /* Configure pipe for OUT token */
                        hstPipeCfg = usbMod->USBHS_HSTPIPCFG[0].w & ~(uint32_t)(USBHS_HSTPIPCFG_PTOKEN_Msk);
                        hstPipeCfg |= USBHS_HSTPIPCFG_PTOKEN(2);
                        usbMod->USBHS_HSTPIPCFG[0].w = hstPipeCfg;
                        /* Clear Tx Out Ready Interrupt */
                        usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_TXOUTIC_Msk;
                        /* Enable Pipe out ready interrupt */
                        usbMod->USBHS_HSTPIPIER[0].w = USBHS_HSTPIPIER_TXOUTES_Msk;
                        /* Clear FIFOCON and Unfreeze pipe */
                        usbMod->USBHS_HSTPIPIDR[0].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);
                   }
                   else
                   {
                        /* This means this is a multi stage control read
                         * transfer. Issue another IN token */
					    /* Configure pipe for IN token */
                        hstPipeCfg = usbMod->USBHS_HSTPIPCFG[0].w & ~(uint32_t)(USBHS_HSTPIPCFG_PTOKEN_Msk);
                        hstPipeCfg |= USBHS_HSTPIPCFG_PTOKEN(1);
                        usbMod->USBHS_HSTPIPCFG[0].w = hstPipeCfg;
					    /* Clear IN Rx Interrupt */
					    usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_RXINIC_Msk;
					    /* Enable IN Rx Interrupt */
					    usbMod->USBHS_HSTPIPIER[0].w = USBHS_HSTPIPIER_RXINES_Msk;
                        /* Clear FIFOCON and Unfreeze pipe */
                        usbMod->USBHS_HSTPIPIDR[0].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);
                   }
               }
               else
               {
                   /* Data stage was moving from host to device.  Check if we
                    * need to send more data */

                   if(irp->completedBytes < irp->size)
                   {
                        _DRV_USBHSV1_HOST_IRPTransmitFIFOLoad(usbMod, irp, 0);
                   }
                   else
                   {
                        /* We can move to the status stage */
                        irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_HANDSHAKE_SENT;
                        hstPipeCfg = usbMod->USBHS_HSTPIPCFG[0].w & ~(uint32_t)(USBHS_HSTPIPCFG_PTOKEN_Msk);
                        hstPipeCfg |= USBHS_HSTPIPCFG_PTOKEN(1);
                        usbMod->USBHS_HSTPIPCFG[0].w = hstPipeCfg;
                        /* Clear IN Rx Interrupt */
                        usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_RXINIC_Msk;
                        /* Enable IN Rx Interrupt */
                        usbMod->USBHS_HSTPIPIER[0].w = USBHS_HSTPIPIER_RXINES_Msk;
                        /* Clear FIFOCON and Unfreeze pipe */
                        usbMod->USBHS_HSTPIPIDR[0].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);
                   }
               }
               break;

           case DRV_USBHSV1_HOST_IRP_STATE_HANDSHAKE_SENT:

               /* If we have reached here, it means that status stage has
                * completed. Check the direction of the data stage, update the
                * irp status flag and then end the irp. */

               	if (false != usbMod->USBHS_HSTPIPISR[0].RXINI) 
                {
					/* IN packet received */
					/* IN packet received */
                    /* In case of low USB speed and with a high CPU frequency,
		             * a ACK from host can be always running on USB line
		             * then wait end of ACK on IN pipe */
                    if (usbMod->USBHS_SR.SPEED == 0x2)
                    {    
                        while(!usbMod->USBHS_HSTPIPIMR[0].PFREEZE);
                    }                     
                    /* Clear IN Rx Interrupt */
					usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_RXINIC_Msk;
				}
				else if (false != usbMod->USBHS_HSTPIPISR[0].TXOUTI) 
                {
					/* OUT packet sent */
                    /* Clear Tx Setup Ready Interrupt */
                    usbMod->USBHS_HSTPIPICR[0].w = USBHS_HSTPIPICR_TXOUTIC_Msk;
				}
				else
				{
					return;
				}
               irp->status = USB_HOST_IRP_STATUS_COMPLETED;
               
               if(irp->flags & 0x80)
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
                   /* We need to clear the Status Packet bit and
                    * the Rx packet ready bit */
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
            irp->tempState = DRV_USBHSV1_HOST_IRP_STATE_SETUP_STAGE;

            /* We need to update the flags parameter of the IRP to indicate the
             * direction of the control transfer. */

            if(*((uint8_t*)(irp->setup)) & 0x80)
            {
                /* This means the data stage moves from device to host. Set bit
                 * 15 of the flags parameter */
               irp->flags |= 0x80;
            }
            else
            {
                /* This means the data stage moves from host to device. Clear
                 * bit 15 of the flags parameter. */
                irp->flags &= 0x7F;
            }

            /* Send the setup packet to the device */
            _DRV_USBHSV1_HOST_IRPTransmitSetupPacket(usbMod, irp);
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
}/* end of _DRV_USBHSV1_HOST_ControlTransferProcess() */

void _DRV_USBHSV1_HOST_NonControlTransferProcess
(
    DRV_USBHSV1_OBJ * hDriver,
    uint8_t hostPipe
)
{
    /* This function processes non-zero endpoint transfers which
     * could be any of bulk, interrupt and isochronous transfers */

    DRV_USBHSV1_HOST_ENDPOINT_OBJ * endpointTable;
    DRV_USBHSV1_HOST_PIPE_OBJ * pipe;
    USB_HOST_IRP_LOCAL * irp;
    volatile usbhs_registers_t * usbMod;
    bool endIRP = false;
    bool isDmaUsed = false;
	bool endIRPOut = false;
    unsigned int count;

    endpointTable = &(hDriver->hostEndpointTable[hostPipe]);
    usbMod = hDriver->usbID;
    pipe = endpointTable->endpoint.pipe; 
	
    if((!endpointTable->endpoint.inUse) ||
       (pipe == NULL) ||
       (pipe == (DRV_USBHSV1_HOST_PIPE_OBJ *)(DRV_USBHSV1_HOST_PIPE_HANDLE_INVALID)))
    {
        /* This means the pipe was closed. We don't do anything */
        return;
    }

    irp = pipe->irpQueueHead;
	
    /* We got an interrupt for data moving from host to to device. Check if
     * there were any transmission errors. Then check if there is any more
     * data to be sent in the IRP. If we are done, then end the IRP and
     * start a new one. */

    /* Check if the IRP was aborted */
    if(irp->tempState == DRV_USBHSV1_HOST_IRP_STATE_ABORTED)
    {
        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ABORTED;
    }
    else if(false != usbMod->USBHS_HSTPIPISR[hostPipe].RXSTALLDI)
    {
        /* This means the packet was stalled. Set the error status and then
         * clear the stall bit */

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
        /* Clear Stall Interrupt */
        usbMod->USBHS_HSTPIPICR[hostPipe].RXSTALLDIC = 1;
	    /* Reset DATA Toggle */
        usbMod->USBHS_HSTPIPIER[hostPipe].RSTDTS = 1;
        /* Reset Pipe*/
        Set_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (hostPipe)));
        Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (hostPipe)));			
    }
    else if(false != usbMod->USBHS_HSTPIPISR[hostPipe].PERRI)
    {
        /* This means there was an bus error. The packet was tried three
         * times and was not successfully processed */

        endIRP = true;
        irp->status = USB_HOST_IRP_STATUS_ERROR_DATA;
		/* Ack all errors */
		usbMod->USBHS_HSTPIPERR[hostPipe].w = 0;
        /* Reset Pipe*/
        Set_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (hostPipe)));
        Clr_bits(hDriver->usbID->USBHS_HSTPIP.w, ((1 << 16) << (hostPipe)));		
    }
    else if (false != usbMod->USBHS_HSTPIPISR[hostPipe].TXOUTI)
    {
        /* This means this transaction completed successfully.  We should
         * check if there are any spare bytes remaining to be sent and then
         * send it */
            
        /* Clear Tx Setup Ready Interrupt */
        usbMod->USBHS_HSTPIPICR[hostPipe].w = USBHS_HSTPIPICR_TXOUTIC_Msk;
        if(irp->completedBytes >= irp->size)
        {
            endIRP = true;
		    endIRPOut = true;
            irp->status = USB_HOST_IRP_STATUS_COMPLETED;
        }
        else
        {
            /* This means we have more data to send */
            endIRP = false;

            /* This function will load the next packet for this IRP into the
             * endpoint FIFO and then transmit it. */
            /* Clear Tx Setup Ready Interrupt */
            usbMod->USBHS_HSTPIPICR[hostPipe].w = USBHS_HSTPIPICR_TXOUTIC_Msk;
            _DRV_USBHSV1_HOST_IRPTransmitFIFOLoad(usbMod, irp, hostPipe);
        }
    }
    else if (false != usbMod->USBHS_HSTPIPISR[hostPipe].RXINI)
    {
        /* In case of low USB speed and with a high CPU frequency,
		* a ACK from host can be always running on USB line
		* then wait end of ACK on IN pipe */
        if (usbMod->USBHS_SR.SPEED == 0x2)
        {    
            while(!usbMod->USBHS_HSTPIPIMR[hostPipe].PFREEZE);
        }  
        usbMod->USBHS_HSTPIPICR[hostPipe].w = USBHS_HSTPIPICR_RXINIC_Msk;
        /* This means that data was received without errors. */
        count = _DRV_USBHSV1_HOST_IRPReceiveFIFOUnload(usbMod, irp, hostPipe, &isDmaUsed);
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
				/* Enable IN Rx Interrupt */
				usbMod->USBHS_HSTPIPIER[hostPipe].w = USBHS_HSTPIPIER_RXINES_Msk;
                /* Clear FIFOCON and Unfreeze pipe */
                usbMod->USBHS_HSTPIPIDR[hostPipe].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);
            }
        }
        else
        {
            /* DMA has been used. Do not end the IRP here.
             * It will be done in DMA ISR handler */
            endIRP = false;
        }
    }
    else
    {
	    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Unknown Pipe Interrupt");
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
        if((irp != NULL) && (!(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)) && (endIRPOut != false) )
        {
            /* We do have another IRP to process. */
            irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
            /* Clear Tx Setup Ready Interrupt */
            usbMod->USBHS_HSTPIPICR[hostPipe].w = USBHS_HSTPIPICR_TXOUTIC_Msk;
            _DRV_USBHSV1_HOST_IRPTransmitFIFOLoad(usbMod, irp, hostPipe);
        }

        /* A IRP could have been submitted in the callback. If that is the
         * case and the IRP status would indicate that it already in
         * progress. If the IRP in the queue head is not in progress then we
         * should initiate the transaction */

        if((irp != NULL) && (!(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)) && (endIRPOut == false) )
        {
            irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
                
            /* We do have another IRP to process. Request for
             * an IN packet. */
			/* Clear IN Rx Interrupt */
			usbMod->USBHS_HSTPIPICR[hostPipe].w = USBHS_HSTPIPICR_RXINIC_Msk;
			/* Enable IN Rx Interrupt */
			usbMod->USBHS_HSTPIPIER[hostPipe].w = USBHS_HSTPIPIER_RXINES_Msk;
            /* Clear FIFOCON and Unfreeze pipe */
            usbMod->USBHS_HSTPIPIDR[hostPipe].w = (USBHS_HSTPIPIDR_FIFOCONC_Msk | USBHS_HSTPIPIDR_PFREEZEC_Msk);
        }
    }
}/* end of _DRV_USBHSV1_HOST_NonControlTransferProcess() */

void _DRV_USBHSV1_HOST_Tasks_ISR(DRV_USBHSV1_OBJ * hDriver)
{
    uint8_t intPipe;
	   
    if ((false != hDriver->usbID->USBHS_HSTISR.DDISCI) && \
        (false != hDriver->usbID->USBHS_HSTIMR.DDISCIE)) 
    {
        /* Manage Device Disconnection Interrupt 
         */
        /* Clear Device Disconnection Interrupt */
        hDriver->usbID->USBHS_HSTICR.w = USBHS_HSTICR_DDISCIC_Msk;
        /* Disable Device Disconnection Interrupt */
        hDriver->usbID->USBHS_HSTIDR.w = USBHS_HSTIDR_DDISCIEC_Msk;
        /* Enable Device Connection and Host Wakeup Interrupt */
        hDriver->usbID->USBHS_HSTIER.w = (USBHS_HSTIER_DCONNIES_Msk | USBHS_HSTIER_HWUPIES_Msk);
        hDriver->deviceAttached = false;
        if(hDriver->attachedDeviceObjHandle != USB_HOST_DEVICE_OBJ_HANDLE_INVALID)
        {
            /* Ask the host layer to de-enumerate this device. The de-enumeration
             * must be done in the interrupt context. */
            USB_HOST_DeviceDenumerate (hDriver->attachedDeviceObjHandle);
        }
        hDriver->attachedDeviceObjHandle = USB_HOST_DEVICE_OBJ_HANDLE_INVALID;        
    }
    else if((false != hDriver->usbID->USBHS_HSTISR.HWUPI) && \
            (false != hDriver->usbID->USBHS_HSTIMR.HWUPIE)) 
    {
    	/* Manage Host Wakeup Interrupt. This interrupt is generated 
         * even if the clock is frozen
         */
    	/* Clear Wake-up Interrupt */
        hDriver->usbID->USBHS_HSTICR.w = USBHS_HSTICR_HWUPIC_Msk;
        /* Disable Wake-up Interrupt */
        hDriver->usbID->USBHS_HSTIDR.w = USBHS_HSTIDR_HWUPIEC_Msk;
        /* Enable Device Connection Interrupt */
        hDriver->usbID->USBHS_HSTIER.w = USBHS_HSTIER_DCONNIES_Msk;
        /* Unfreeze clock */
        hDriver->usbID->USBHS_CTRL.FRZCLK = 0;
    	/* Requests VBus activation */
        hDriver->usbID->USBHS_SFR.VBUSRQS= 1;
	}
    else if((false != hDriver->usbID->USBHS_HSTISR.DCONNI) && \
            (false != hDriver->usbID->USBHS_HSTIMR.DCONNIE)) 
    {
        /* Manage Device Connection Interrupt
         */
        /* Clear Connection Interrupt*/
        hDriver->usbID->USBHS_HSTICR.w = USBHS_HSTICR_DCONNIC_Msk;
        /* Disable Connection Interrupt */
        hDriver->usbID->USBHS_HSTIDR.w = USBHS_HSTIDR_DCONNIEC_Msk;
        /* Enable Disconnection Interrupt */
        hDriver->usbID->USBHS_HSTIER.w = USBHS_HSTIER_DDISCIES_Msk;
     	hDriver->deviceAttached = true;
 	}
    else if((false != hDriver->usbID->USBHS_HSTISR.RSTI) && \
            (false != hDriver->usbID->USBHS_HSTIMR.RSTIE))
	{
        /* Manage USB Reset Sent Interrupt  */
        /* Clear USB Reset Sent Interrupt */
        hDriver->usbID->USBHS_HSTICR.w = USBHS_HSTICR_RSTIC_Msk;
        /* Disable USB Reset Sent Interrupt */
        hDriver->usbID->USBHS_HSTIDR.w = USBHS_HSTIDR_RSTIEC_Msk;
        /* Clear the flag */
        hDriver->isResetting = false;
        /* Now that reset is complete, we can find out the
         * speed of the attached device. */
        switch (hDriver->usbID->USBHS_SR.SPEED) 
        {
            case 0x0:
                hDriver->deviceSpeed = USB_SPEED_FULL;
            break;
            case 0x1:
                hDriver->deviceSpeed = USB_SPEED_HIGH;
            break;
            case 0x2:
                hDriver->deviceSpeed = USB_SPEED_LOW;
            break;
            default:
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Unknown Speed at Reset");
            break;
        }
	}
	else if(0 != (hDriver->usbID->USBHS_HSTISR.w & 0x3FF00))
    {
        /* Manage pipe interrupts */
        /* Get the lowest Pipe number generating
         * a Pipe Interrupt */
        intPipe = ctz(((hDriver->usbID->USBHS_HSTISR.w & hDriver->usbID->USBHS_HSTIMR.w) >> 8) | (1 << 10));
	    if (intPipe == 0) 
        {
		     /* Manage control pipe */
		    _DRV_USBHSV1_HOST_ControlTransferProcess(hDriver);
	    }
        else
        {
		    /* Manage Non-control pipe */
		    _DRV_USBHSV1_HOST_NonControlTransferProcess(hDriver, intPipe);
        }
    } 
    else
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Unknown Interrupt");
    }   

}/* end of _DRV_USBHSV1_HOST_Tasks_ISR() */

void DRV_USBHSV1_HOST_StartOfFrameControl(DRV_HANDLE client, bool control)
{
    /* At the point this function does not do any thing.
     * The Start of frame signaling in this HCD is controlled
     * automatically by the module. */
}/* end of DRV_USBHSV1_HOST_StartOfFrameControl() */

USB_SPEED DRV_USBHSV1_HOST_DeviceCurrentSpeedGet(DRV_HANDLE client)
{
    /* This function returns the current device speed */

    DRV_USBHSV1_OBJ * hDriver;

    if((client == DRV_HANDLE_INVALID) || (((DRV_USBHSV1_OBJ *)client) == NULL))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Invalid client");
    }

    hDriver = (DRV_USBHSV1_OBJ *)client;
    return(hDriver->deviceSpeed);

}/* end of DRV_USBHSV1_HOST_DeviceCurrentSpeedGet() */

// ****************************************************************************
/* Function:
    bool DRV_USBHSV1_HOST_EventDisable
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

bool DRV_USBHSV1_HOST_EventsDisable
(
    DRV_HANDLE handle
)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj;
    bool result = false; 
	
    if((DRV_HANDLE_INVALID != handle) && (0 != handle))
    {
        pUSBDrvObj = (DRV_USBHSV1_OBJ *)(handle);
        result = _DRV_USBHSV1_InterruptSourceDisable(pUSBDrvObj->interruptSource);
    }

    return(result);
}


// ****************************************************************************
/* Function:
    void DRV_USBHSV1_HOST_EventsDisable
    (
        DRV_HANDLE handle
        bool eventRestoreContext
    );
    
  Summary:
    Restores the events to the specified the original value.
	
  Description:
    This function will restore the enable disable state of the events.
    eventRestoreContext should be equal to the value returned by the
    DRV_USBHSV1_HOST_EventsDisable() function.
	
  Remarks:
    Refer to drv_usbfs.h for usage information.
*/

void DRV_USBHSV1_HOST_EventsEnable
(
    DRV_HANDLE handle, 
    bool eventContext
)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj;
   
    if((DRV_HANDLE_INVALID != handle) && (0 != handle))
    {
        pUSBDrvObj = (DRV_USBHSV1_OBJ *)(handle);
        if(false == eventContext)
        {
            _DRV_USBHSV1_InterruptSourceDisable(pUSBDrvObj->interruptSource);
        }
        else
        {
            _DRV_USBHSV1_InterruptSourceEnable(pUSBDrvObj->interruptSource);
        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Root Hub Driver Routines
// *****************************************************************************
// *****************************************************************************

void DRV_USBHSV1_HOST_ROOT_HUB_OperationEnable(DRV_HANDLE handle, bool enable)
{
    /* Start of local variable */
    DRV_USBHSV1_OBJ * pUSBDrvObj = (DRV_USBHSV1_OBJ *)handle;
    /* End of local variable */
    
    volatile usbhs_registers_t * usbMod = pUSBDrvObj->usbID;

    /* Check if the handle is valid or opened */
    if((handle == DRV_HANDLE_INVALID) || (!(pUSBDrvObj->isOpened)))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else
    {
        if(false == enable)
        {
             /* If the root hub operation is disable, we disable detach and
             * attached event and switch off the port power. */

            _DRV_USBHSV1_InterruptSourceClear(pUSBDrvObj->interruptSource);
            pUSBDrvObj->operationEnabled = false;

        }
        else
        {
            /* The USB Global interrupt and USB module is already enabled at
             * this point. We enable the attach interrupt to detect attach
             */

            pUSBDrvObj->operationEnabled = true;
            /* Enable Device Connection Interrupt */
            usbMod->USBHS_HSTIER.w = USBHS_HSTIER_DCONNIES_Msk;   
            /* Requests VBus activation */
            usbMod->USBHS_SFR.VBUSRQS = 1;            
            /* Unfreeze clock */
            usbMod->USBHS_CTRL.FRZCLK = 0;
        }
    }
}

bool DRV_USBHSV1_HOST_ROOT_HUB_OperationIsEnabled(DRV_HANDLE hClient)
{
    DRV_USBHSV1_OBJ * hDriver;
    if((hClient == DRV_HANDLE_INVALID) || (((DRV_USBHSV1_OBJ *)hClient) == NULL))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Invalid client");
    }
    hDriver = (DRV_USBHSV1_OBJ *)hClient;
    return(hDriver->operationEnabled);

}/* end of DRV_USBHSV1_HOST_OperationIsEnabled() */

// ****************************************************************************
/* Function:
    void DRV_USBHSV1_HOST_ROOT_HUB_Initialize
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

void DRV_USBHSV1_HOST_ROOT_HUB_Initialize
(
    DRV_HANDLE handle,
    USB_HOST_DEVICE_OBJ_HANDLE usbHostDeviceInfo
)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj = (DRV_USBHSV1_OBJ *)handle;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else
    {
        pUSBDrvObj->usbHostDeviceInfo = usbHostDeviceInfo;
    }
}

// ****************************************************************************
/* Function:
    uint8_t DRV_USBHSV1_HOST_ROOT_HUB_PortNumbersGet(DRV_HANDLE handle);

  Summary:
    Returns the number of ports this root hub contains.

  Description:
    This function returns the number of ports that this root hub contains.

  Remarks:
    None.
*/

uint8_t DRV_USBHSV1_HOST_ROOT_HUB_PortNumbersGet(DRV_HANDLE handle)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj = (DRV_USBHSV1_OBJ *)handle;
    uint8_t result = 0;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else
    {
        result = 1;
    }

    return(result);
}

// ****************************************************************************
/* Function:
    uint32_t DRV_USBHSV1_HOST_ROOT_HUB_MaximumCurrentGet(DRV_HANDLE);

  Summary:
    Returns the maximum amount of current that this root hub can provide on the
    bus.

  Description:
    This function returns the maximum amount of current that this root hubn can
    provide on the bus.

  Remarks:
    Refer to drv_usbfs.h for usage information.
*/

uint32_t DRV_USBHSV1_HOST_ROOT_HUB_MaximumCurrentGet(DRV_HANDLE handle)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj = (DRV_USBHSV1_OBJ *)handle;
    uint32_t result = 0;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else
    {
        result = pUSBDrvObj->rootHubInfo.rootHubAvailableCurrent;
    }

    return(result);
}

// ****************************************************************************
/* Function:
    USB_SPEED DRV_USBHSV1_HOST_ROOT_HUB_BusSpeedGet(DRV_HANDLE handle);

  Summary:
    Returns the speed at which the bus to which this root hub is connected is
    operating.

  Description:
    This function returns the speed at which the bus to which this root hub is
    connected is operating.

 Remarks:
    None.
*/

USB_SPEED DRV_USBHSV1_HOST_ROOT_HUB_BusSpeedGet(DRV_HANDLE handle)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj = (DRV_USBHSV1_OBJ *)handle;
    USB_SPEED speed = USB_SPEED_ERROR;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");

    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
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
    void DRV_USBHSV1_ROOT_HUB_PortResume(DRV_HANDLE handle, uint8_t port );

  Summary:
    Resumes the specified root hub port.

  Description:
    This function resumes the root hub. The resume duration is defined by
    DRV_USBHSV1_ROOT_HUB_RESUME_DURATION. The status of the resume signalling can
    be checked using the DRV_USBHSV1_ROOT_HUB_PortResumeIsComplete() function.

  Remarks:
    The root hub on this particular hardware only contains one port - port 0.
*/

USB_ERROR DRV_USBHSV1_HOST_ROOT_HUB_PortResume(DRV_HANDLE handle, uint8_t port)
{
    /* The functionality is yet to be implemented. */
    return(USB_ERROR_NONE);
}

// ****************************************************************************
/* Function:
    void DRV_USBHSV1_ROOT_HUB_PortSuspend(DRV_HANDLE handle, uint8_t port );

  Summary:
    Suspends the specified root hub port.

  Description:
    This function suspends the root hub port.

  Remarks:
    The root hub on this particular hardware only contains one port - port 0.
*/

USB_ERROR DRV_USBHSV1_HOST_ROOT_HUB_PortSuspend(DRV_HANDLE handle, uint8_t port)
{
    /* The functionality is yet to be implemented. */
    return (USB_ERROR_NONE);
}

// ****************************************************************************
/* Function:
    void DRV_USBHSV1_ROOT_HUB_PortResetIsComplete
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

bool DRV_USBHSV1_HOST_ROOT_HUB_PortResetIsComplete
(
    DRV_HANDLE handle,
    uint8_t port
)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj = (DRV_USBHSV1_OBJ *)handle;
    bool result = true;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
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
    void DRV_USBHSV1_ROOT_HUB_PortReset(DRV_HANDLE handle, uint8_t port );
    
  Summary:
    Resets the specified root hub port.
	
  Description:
    This function resets the root hub port. The reset duration is defined by
    DRV_USBHSV1_ROOT_HUB_RESET_DURATION. The status of the reset signaling can be
    checked using the DRV_USBHSV1_ROOT_HUB_PortResetIsComplete() function.
	
  Remarks:
    Refer to drv_usbfs.h for usage information.
*/

USB_ERROR DRV_USBHSV1_HOST_ROOT_HUB_PortReset(DRV_HANDLE handle, uint8_t port)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj = (DRV_USBHSV1_OBJ *)handle;
    USB_ERROR result = USB_ERROR_NONE;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
        result = USB_ERROR_PARAMETER_INVALID;

    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
        result = USB_ERROR_PARAMETER_INVALID;
    }
    else if(pUSBDrvObj->isResetting)
    {
        /* This means a reset is already in progress. Lets not do anything. */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Reset already in progress");

    }
    else
    {
        /* Start the reset signal. Set the driver flag to indicate the reset
         * signal is in progress. Start generating the reset signal.
         */
        
        pUSBDrvObj->isResetting = true;
        pUSBDrvObj->resetState = DRV_USBHSV1_HOST_RESET_STATE_START;
        /* Enable Reset sent interrupt */
        pUSBDrvObj->usbID->USBHS_HSTIER.RSTIES = 1;
        /* Start Reset */
        pUSBDrvObj->usbID->USBHS_HSTCTRL.RESET = 1;
    }

    return(result);
}

// ****************************************************************************
/* Function:
    USB_SPEED DRV_USBHSV1_HOST_ROOT_HUB_PortSpeedGet
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

USB_SPEED DRV_USBHSV1_HOST_ROOT_HUB_PortSpeedGet(DRV_HANDLE handle, uint8_t port)
{
    DRV_USBHSV1_OBJ * pUSBDrvObj = (DRV_USBHSV1_OBJ *)handle;
    USB_SPEED speed = USB_SPEED_ERROR;

    if(DRV_HANDLE_INVALID == handle)
    {
        /* Driver handle is not valid */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
    }
    else if(!(pUSBDrvObj->isOpened))
    {
        /* Driver has not been opened. Handle could be stale */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nDRV USBHSV1: Bad Client or client closed");
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
