/*******************************************************************************
  USB Mass Storage Client Driver Implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_msd.c

  Summary:
    This file contains implementations of both private and public functions
    of the USB Host MSD client driver.

  Description:
    This file contains the USB host MSD client driver implementation. This file 
    should be included in the project if USB MSD devices are to be supported.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sub license terms in the accompanying license agreement).

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

#include "usb/usb_host_msd.h"
#include "usb/src/usb_host_msd_local.h"
#include "usb/usb_host_client_driver.h"
#include "system/debug/sys_debug.h"
#include "usb/usb_host_scsi.h"
//#include <p32xxxx.h>

/*************************************************
 * Driver interface that is provide to the 
 * host layer.
 *************************************************/

USB_HOST_CLIENT_DRIVER gUSBHostMSDClientDriver = 
{
    .initialize = _USB_HOST_MSD_Initialize,
    .deinitialize = _USB_HOST_MSD_Deinitialize,
    .reinitialize = _USB_HOST_MSD_Reinitialize,
    .interfaceAssign = _USB_HOST_MSD_InterfaceAssign,
    .interfaceRelease = _USB_HOST_MSD_InterfaceRelease,
    .interfaceEventHandler = _USB_HOST_MSD_InterfaceEventHandler,
    .interfaceTasks = _USB_HOST_MSD_InterfaceTasks,
    .deviceEventHandler = NULL,
    .deviceAssign = NULL,
    .deviceEventHandler = NULL,
    .deviceRelease = NULL
         
};

/**************************************************
 * If the error callback function has been enabled
 * then declare it here.
 **************************************************/
_USB_HOST_MSD_ERROR_CALLBACK_DECLARE

/**************************************************
 * Global array of MSD Instance Objects. Each for
 * one MSD device attached.
 ***************************************************/
USB_HOST_MSD_INSTANCE  gUSBHostMSDInstance[USB_HOST_MSD_INSTANCES_NUMBER];

/***********************************************
 * CBW and CSW structure needed by for the MSD
 * function driver instance.
 ***********************************************/
uint8_t gUSBHostMSDCBW[USB_HOST_MSD_INSTANCES_NUMBER][32] COHERENT_ATTRIBUTE __attribute__((aligned(16)));
uint8_t gUSBHostMSDCSW[USB_HOST_MSD_INSTANCES_NUMBER][16] COHERENT_ATTRIBUTE __attribute__((aligned(16)));

// *****************************************************************************
// *****************************************************************************
// USB Host MSD Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_TransferTasks
   (
        uintptr msdInstanceIndex,
        USB_HOST_RESULT result,
        size_t size
   );

  Summary:
    This function is called when a transfer event has occurred. This updates the
    transfer state of an ongoing MSD transfer.

  Description:
    This function is called when a transfer event has occurred. This updates the
    transfer state of an ongoing MSD transfer.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_TransferTasks
(
    uintptr_t msdInstanceIndex,
    USB_HOST_RESULT result,
    size_t size
)
{
    USB_HOST_TRANSFER_HANDLE transferHandle;
    USB_HOST_MSD_RESULT msdResult = USB_HOST_MSD_RESULT_FAILURE;
    USB_HOST_PIPE_HANDLE pipeHandle;
    USB_MSD_CSW * msdCSW;
    size_t processedBytes = 0;
    USB_HOST_MSD_TRANSFER_OBJ * transferObj;
    bool transferIsDone = false;
    USB_HOST_MSD_INSTANCE * msdInstanceInfo = &gUSBHostMSDInstance[msdInstanceIndex];

    if(msdInstanceInfo->msdState == USB_HOST_MSD_STATE_READY)
    {
        /* This means this transfer event is related to a regular BOT transfer
         * */

        transferObj = &msdInstanceInfo->transferObj;

        switch(msdInstanceInfo->transferState)
        {
            case USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CBW:

                /* This means the CBW stage has completed */
                if(result == USB_HOST_RESULT_SUCCESS)
                {
                    /* Check if the command needs a data stage */
                    if(transferObj->size > 0)
                    {
                        /* Data stage is needed. Find out the direction */
                        if(transferObj->transferDirection == USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST)
                        {
                            /* We need the in pipe */
                            pipeHandle = msdInstanceInfo->bulkInPipeHandle;
                        }
                        else
                        {
                            /* Else we need the out pipe */
                            pipeHandle = msdInstanceInfo->bulkOutPipeHandle;
                        }

                        /* Update the state to indicate that we are waiting for data
                         * */
                        msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_DATA;

                        /* Send the transfer */
                        USB_HOST_DeviceTransfer(pipeHandle, &transferHandle, transferObj->buffer, transferObj->size,msdInstanceIndex);
                    }
                    else
                    {
                        /* Data stage is not needed. We can launch CSW */
                        msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CSW;

                        /* Note that we are not checking for return value here
                         * because we are in an interrupt context. If the function
                         * fails here, there isn't much that we can do. */
                        USB_HOST_DeviceTransfer(msdInstanceInfo->bulkInPipeHandle, &transferHandle, msdInstanceInfo->msdCSW, 13, msdInstanceIndex);
                    }
                }
                else if(result == USB_HOST_RESULT_REQUEST_STALLED)
                {
                    /* The CBW was stalled. The specification is not clear on
                     * what should be done in such a case. In this
                     * implementation we will request reset recovery procedure.
                     * Because the reset recovery cannot be completed in this
                     * transfer event handler context, we defer this to the
                     * transfer error tasks routine */

                    msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RESET_RECOVERY;
                    msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_ERROR;
                    _USB_HOST_MSD_ERROR_CALLBACK(msdInstanceIndex, USB_HOST_MSD_ERROR_CODE_CBW_STALL_RESET_RECOVERY);
                }
                else 
                {
                    /* An unknown failure occurred on the bus. We move this MSD
                     * instance to an error state. We must let the client know
                     * the transfer will not complete. */
                    msdInstanceInfo->msdErrorCode = USB_HOST_MSD_ERROR_CODE_FAILED_BOT_TRANSFER;

                    msdInstanceInfo->msdState = USB_HOST_MSD_STATE_ERROR;
                    transferIsDone = true;
                    msdResult = USB_HOST_MSD_RESULT_FAILURE;
                }

                break;

            case USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_DATA:

                /* We were waiting for the data stage to complete */
                if (result == USB_HOST_RESULT_SUCCESS)
                {
                    /* We got the data stage. Go to CSW stage */
                    msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CSW;

                    /* Note that we are not checking for return value here
                     * because we are in an interrupt context. If the function
                     * fails here, there isn't much that we can do. */
                    USB_HOST_DeviceTransfer(msdInstanceInfo->bulkInPipeHandle, &transferHandle, msdInstanceInfo->msdCSW, 13, msdInstanceIndex);
                }
                else if(result == USB_HOST_RESULT_REQUEST_STALLED)
                {
                    /* If the data stage of the BOT was stalled, sections 6.7.2
                     * and 6.7.3 define the action to be taken. The rest of the
                     * transfer must complete in the error tasks routine. */

                    if(transferObj->transferDirection == USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST)
                    {
                        /* Clear the halt on the in pipe */
                        msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_IN_PIPE_STALLED;
                    }
                    else
                    {
                        /* Clear the halt on the out pipe */
                        msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_OUT_PIPE_STALLED;
                    }
                    
                    msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_ERROR;
                }
                else
                {
                    /* End the command. This is an error that we cannot handle */
                    msdInstanceInfo->msdErrorCode = USB_HOST_MSD_ERROR_CODE_FAILED_BOT_TRANSFER;
                    msdInstanceInfo->msdState = USB_HOST_MSD_STATE_ERROR;
                    transferIsDone = true;
                    msdResult = USB_HOST_MSD_RESULT_FAILURE;
                }

                break;

            case USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CSW:
                /* This means the CSW has completed. Check if we have a valid
                 * and meaningful CSW */

                msdCSW = msdInstanceInfo->msdCSW;

                if(result == USB_HOST_RESULT_SUCCESS) 
                { 
                    if(msdInstanceInfo->transferErrorTaskState == USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_RETRY)
                    {
                        /* This means this CSW was re-attempted after the
                         * previous CSW attempt that was stalled. Clear
                         * the transfer error task state to let the error
                         * task know that nothing needs to be done now. */
                        
                        msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_NO_ERROR;
                    }
                    
                    if ((msdCSW->dCSWSignature == USB_MSD_VALID_CSW_SIGNATURE) &&
                            (msdCSW->dCSWTag == USB_MSD_VALID_CBW_TAG) && (size == 13))
                    {
                        /* This means the CSW is valid as defined in section
                         * 6.3.1 of the BOT specification. Now we check if it is
                         * meaningful.
                         * */

                        if(((msdCSW->bCSWStatus == USB_MSD_CSW_STATUS_GOOD) || (msdCSW->bCSWStatus == USB_MSD_CSW_STATUS_FAIL)) &&
                                (msdCSW->dCSWDataResidue <= msdInstanceInfo->msdCBW->dCBWDataTransferLength))
                        {
                            /* This means the CSW is meaningful. We must let the
                             * MSD client know if the command has failed or
                             * passed. This is know in the bCSWStatus of CSW. */
                            processedBytes = msdInstanceInfo->msdCBW->dCBWDataTransferLength - msdCSW->dCSWDataResidue;
                            transferIsDone = true;
                            msdResult = msdCSW->bCSWStatus;
                        }
                        else if (msdCSW->bCSWStatus == USB_MSD_CSW_STATUS_PHASE_ERROR)
                        {
                            /* This means the CSW is valid but a phase error has
                             * occurred. Reset recovery should be performed.
                             * This is done in the transfer error tasks routine. */
                            msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_ERROR;
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RESET_RECOVERY;
                            msdInstanceInfo->cswPhaseError = true;
                            _USB_HOST_MSD_ERROR_CALLBACK(msdInstanceIndex, USB_HOST_MSD_ERROR_CODE_CSW_PHASE_ERROR);
                        }
                        else
                        {
                            /* The specification does not define how the host
                             * must react in such a case. The specification
                             * requires the device to return a valid and
                             * meaningful. Because this is something we cannot
                             * handle, we end the transfer and move the device
                             * to error state. */

                            msdInstanceInfo->msdErrorCode = USB_HOST_MSD_ERROR_CODE_FAILED_BOT_TRANSFER;
                            msdInstanceInfo->msdState = USB_HOST_MSD_STATE_ERROR;
                            transferIsDone = true;
                            processedBytes = 0;
                            msdResult = USB_HOST_MSD_RESULT_FAILURE;
                        }
                    }
                    else
                    {
                        /* If the CSW is not valid, then Reset recovery must be
                         * performed. See Figure 2 - Status transport flow in
                         * the BOT specification. */

                        msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_ERROR;
                        msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RESET_RECOVERY;
                    }
                }
                else if(result == USB_HOST_RESULT_REQUEST_STALLED)
                {
                    /* We need to check if the this the first or the second time
                     * the CSW got stalled. If the transfer error task state
                     * machine is in a CSW retry state, then we know that this
                     * is the second time. */

                    msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_ERROR;
                    if(msdInstanceInfo->transferErrorTaskState == USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_RETRY)
                    {
                        /* This means the CSW was stalled once before and has
                         * been stalled again. We should move to reset recovery.
                         * */
                        
                        msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RESET_RECOVERY;
                    }
                    else
                    {
                        /* The CSW request got stalled ! Lets try to clear the IN
                         * pipe stall condition. This is an transfer error condition
                         * which must cleared in the transfer error tasks routine. */
                        
                        msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_STALLED;
                    }
                }
                else
                {
                    /* Figure 2 of the BOT specification states that the host
                     * should try to do an endpoint stall clear on bulk error.
                     * */
           
                    msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_ERROR;
                    msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_STALLED;
                    _USB_HOST_MSD_ERROR_CALLBACK(msdInstanceIndex, USB_HOST_MSD_ERROR_CODE_CSW_UNKNOWN_ERROR);
                }

                break;

            default:
                break;
        }

        if(transferIsDone)
        {
            if(msdInstanceInfo->transferObj.callback != NULL)
            {
                /* Let the caller who initiated the command know that this is
                 * done */
                msdInstanceInfo->transferObj.callback(msdInstanceInfo->transferObj.lunHandle,
                        (USB_HOST_MSD_TRANSFER_HANDLE)(&msdInstanceInfo->transferObj), 
                        msdResult, processedBytes, msdInstanceInfo->transferObj.context);
            }

            /* Return the transfer object back */
            msdInstanceInfo->transferObj.inUse = false;

            /* Make the transfer state ready for another transfer */
            msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_READY;
        }
    }
}

// *****************************************************************************
/* Function:
   USB_HOST_MSD_RESULT _USB_HOST_MSD_HostResultToMSDResultMap
   (
        USB_HOST_RESULT result
   )

  Summary:
    Maps USB_HOST_RESULT to USB_HOST_MSD_RESULT.

  Description:
    Maps USB_HOST_RESULT to USB_HOST_MSD_RESULT.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_MSD_RESULT _USB_HOST_MSD_HostResultToMSDResultMap
(
    USB_HOST_RESULT result
)
{
    USB_HOST_MSD_RESULT msdResult;

    switch(result)
    {
        case USB_HOST_RESULT_SUCCESS:
            msdResult = USB_HOST_MSD_RESULT_SUCCESS;
            break;
        case USB_HOST_RESULT_FAILURE:
            /* Note the fall through here. This is intentional */
        case USB_HOST_RESULT_PARAMETER_INVALID:
        case USB_HOST_RESULT_TRANSFER_ABORTED:
        case USB_HOST_RESULT_PIPE_HANDLE_INVALID:
            msdResult = USB_HOST_MSD_RESULT_FAILURE;
            break;
        case USB_HOST_RESULT_REQUEST_BUSY:
            msdResult = USB_HOST_MSD_RESULT_BUSY;
            break;
        case USB_HOST_RESULT_REQUEST_STALLED:
            msdResult = USB_HOST_MSD_RESULT_COMMAND_STALLED;
            break;
        default:
            msdResult = USB_HOST_MSD_RESULT_FAILURE;
            break;
    }

    return(msdResult);
}

// *****************************************************************************
/* Function:
   int_USB_HOST_MSD_InterfaceHandleToMSDInstance
   ( 
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
   )

  Summary:
    This function will return the MSD instance object that is associated with
    this interface.

  Description:
    This function will return the MSD instance object that is associated with
    this interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_MSD_InterfaceHandleToMSDInstance
( 
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    int iterator;
    int msdInstanceInfo = -1;

    /* Find the MSD Instance object that owns this interface */
    for (iterator = 0; iterator < USB_HOST_MSD_INSTANCES_NUMBER; iterator ++)
    {
        if(gUSBHostMSDInstance[iterator].assigned)
        {
            if(gUSBHostMSDInstance[iterator].interfaceHandle == interfaceHandle)
            {
                /* Found it */
                msdInstanceInfo = iterator;
                break;
            }
        }
    }
    return(msdInstanceInfo);
}

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_InstanceRelease
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    );

  Summary:
    This function will release the device interface. It will close any open
    pipes. It will de initialize any SCSI instances.

  Description:
    This function will release the device interface. It will close any open
    pipes. It will de initialize any SCSI instances.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_InstanceRelease
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    int msdInstanceIndex;
    int iterator;
    USB_HOST_MSD_INSTANCE * msdInstanceInfo;

    /* Find the MSD instance for this interface */
    msdInstanceIndex = _USB_HOST_MSD_InterfaceHandleToMSDInstance(interfaceHandle);

    if(msdInstanceIndex >= 0)
    {
        /* Get the object pointer */
        msdInstanceInfo = &(gUSBHostMSDInstance[msdInstanceIndex]);

        /* Close any open pipes */
        if(msdInstanceInfo->bulkOutPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
        {
            /* Close the pipe */
            USB_HOST_DevicePipeClose(msdInstanceInfo->bulkOutPipeHandle);
            msdInstanceInfo->bulkOutPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        }

        if(msdInstanceInfo->bulkInPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
        {
            /* Close the pipe */
            USB_HOST_DevicePipeClose(msdInstanceInfo->bulkInPipeHandle);
            msdInstanceInfo->bulkInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        }

        /* The interface is release after the pipes are closed. This will allow
         * the IRP callback functions to be valid when the pipes are closed
         * in the statement above. */

        USB_HOST_DeviceInterfaceRelease(interfaceHandle);

        /* Note that we are not checking the msdState before calling
         * USB_HOST_SCSI_Deinitialize for this LUN. A valid concern could be
         * that what if we are deinitialising on a LUN for which a initialize
         * was never called.  The answer lies in how the
         * USB_HOST_SCSI_Deinitialize function works.  It will check if the
         * input LUN was assigned any SCSI instance. If not then it will not
         * have any effect. The msdState cannot be qualified because we dont
         * know in what state the device may have detached. */

        for(iterator = 0; iterator < msdInstanceInfo->logicalUnitNumber; iterator ++)
        {
            USB_HOST_SCSI_Deinitialize(USB_HOST_MSD_LUNHandleGet(iterator, msdInstanceIndex));
        }

        msdInstanceInfo->assigned = false;
    }
}

// *****************************************************************************
/* Function:
    void USB_HOST_MSD_TransferErrorTasks
    (
        USB_HOST_MSD_LUN_HANDLE lunHandle,
    );

  Summary:
    This function maintains the MSD transfer error handling state machine.

  Description:
    This function maintains the MSD transfer error handling state machine. This
    function should be called periodically after the USB_HOST_MSD_Transfer
    function has been called to schedule a transfer. The function should be
    called periodically at least till the transfer completion event has been
    received. Calling this function while a BOT transfer is in progress allows
    the MSD Host Client driver to perform BOT error handling in a non-blocking
    manner.  

    Calling this function when there is no BOT transfer in progress will not
    have any effect. In case of BOT error handling, calling this function will
    eventually result in a BOT transfer event. It is not necessary to call this
    function after this event has occurred (till the next BOT transfer has been
    scheduled).

  Remarks:
    While running in an RTOS application, this function should be called in the
    same thread that requested the BOT Transfer.
*/

void USB_HOST_MSD_TransferErrorTasks
(
    USB_HOST_MSD_LUN_HANDLE lunHandle
)
{
    USB_HOST_MSD_RESULT msdResult = USB_HOST_MSD_RESULT_FAILURE;
    bool transferIsDone = false;
    USB_HOST_MSD_INSTANCE * msdInstanceInfo;
    int msdInstanceIndex;
    size_t processedBytes = 0;
    USB_HOST_TRANSFER_HANDLE transferHandle, requestHandle;

    if(USB_HOST_MSD_LUN_HANDLE_INVALID == lunHandle)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "\r\nUSB Host MSD: LUN Handle %x in USB_HOST_MSD_Transfer is not valid.",lunHandle);
    }
    else
    {
        /* Get the MSD Instance Index from the LUN Handle */
        msdInstanceIndex = USB_HOST_MSD_INDEX(lunHandle);

        /* Get the pointer to the MSD instance */
        msdInstanceInfo = &gUSBHostMSDInstance[msdInstanceIndex];

        if(!msdInstanceInfo->assigned)
        {
            /* This object is not valid */
            SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "\r\nUSB Host MSD: MSD Instance %d in USB_HOST_MSD_TransferErrorTasks is not valid.", msdInstanceIndex);
        }
        else
        {
            if(msdInstanceInfo->msdState == USB_HOST_MSD_STATE_READY)
            {
                /* Check the error state of the transfer */
                switch(msdInstanceInfo->transferErrorTaskState)
                {
                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_NO_ERROR:

                        /* In this state there is no error. There is nothing for
                         * this task routine to do. */
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_RESET_RECOVERY:

                        /* In this state, driver must perform reset recovery as
                         * described 5.3.4 of the BOT specification. The client
                         * driver enters this state if the CBW was stalled
                         * (behaviour not defined in the specification, CSW was
                         * stalled twice, if the CSW was not valid or if an
                         * phase error has occurred. */

                        /* Note that if we are in reset recovery, then the
                         * amount of data processed will be ignored. We should
                         * let the client know that no data was processed */
                        processedBytes = 0;

                        /* Create the control transfer setup packet */
                        _USB_HOST_MSD_ResetPacketCreate(&msdInstanceInfo->setupPacket, msdInstanceInfo->bInterfaceNumber);
                        msdInstanceInfo->controlTransferDone = false;

                        /* Try sending the control transfer */
                        if(USB_HOST_DeviceControlTransfer(msdInstanceInfo->controlPipeHandle, &transferHandle,
                                    &msdInstanceInfo->setupPacket, NULL, _USB_HOST_MSD_ControlTransferCallback,
                                    (uintptr_t)(msdInstanceInfo)) == USB_HOST_RESULT_SUCCESS)
                        {
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_WAIT_MSD_RESET;
                        }
                        else
                        {
                            /* If the control transfer could not be scheduled, then
                             * we wait in the same state */
                        }

                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_WAIT_MSD_RESET:

                        /* Here we check if the control transfer has completed */
                        if(msdInstanceInfo->controlTransferDone)
                        {
                            /* This means the control transfer has completed */
                            if(USB_HOST_RESULT_SUCCESS == msdInstanceInfo->controlTransferResult)
                            {
                                /* The control transfer completed successfully */
                                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d MSD Reset done.", msdInstanceIndex);
                                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d clearing Halt on Bulk IN endpoint", msdInstanceIndex);
                                msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_IN;
                            }
                            else
                            {
                                /* We should not face errors with the MSD reset. We
                                 * cannot handle such error. Place the device in
                                 * error state */

                                transferIsDone = true;
                                msdInstanceInfo->msdErrorCode = USB_HOST_MSD_ERROR_CODE_FAILED_RESET_RECOVERY;
                                msdInstanceInfo->msdState = USB_HOST_MSD_STATE_ERROR;
                                msdResult = USB_HOST_MSD_RESULT_FAILURE;
                            }
                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_IN:

                        /* In this state, we send a clear halt to the IN
                         * endpoint */

                        msdInstanceInfo->standardRequestDone = false;
                        if(USB_HOST_DevicePipeHaltClear(msdInstanceInfo->bulkInPipeHandle, &requestHandle, 
                                    (uintptr_t)(msdInstanceIndex)) == USB_HOST_RESULT_SUCCESS)
                        {
                            /* The request was accepted. Wait for completion */
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_IN_WAIT;
                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_IN_WAIT:

                        /* Here we are waiting for the standard request to complete */
                        if(msdInstanceInfo->standardRequestDone)
                        {
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d MSD IN endpoint Clear Feature Done done.\r\n", msdInstanceIndex);
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d clearing Halt on Bulk Out endpoint\r\n", msdInstanceIndex);
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_OUT;
                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_OUT:

                        /* In this state, we send a clear halt to the OUT
                         * endpoint */

                        msdInstanceInfo->standardRequestDone = false;
                        if(USB_HOST_DevicePipeHaltClear(msdInstanceInfo->bulkOutPipeHandle, &requestHandle, 
                                    (uintptr_t)(msdInstanceIndex)) == USB_HOST_RESULT_SUCCESS)
                        {
                            /* The request was accepted. Wait for completion */
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_OUT_WAIT;
                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_OUT_WAIT:

                        /* Here we are waiting for the standard request to complete */
                        if(msdInstanceInfo->standardRequestDone)
                        {
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d MSD OUT endpoint Clear Feature Done done.", msdInstanceIndex);
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d Ending Transfer.", msdInstanceIndex);
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_NO_ERROR;
                            transferIsDone = true;
                            if(msdInstanceInfo->cswPhaseError)
                            {
                                /* This means we performed reset recovery
                                 * because of a phase error */
                                msdResult = USB_HOST_MSD_RESULT_COMMAND_PHASE_ERROR;
                            }
                            else
                            {
                                /* The CSW was not valid or we have dont know
                                 * the reason for the failure */
                                msdResult = USB_HOST_MSD_RESULT_FAILURE;
                            }
                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_IN_PIPE_STALLED:

                        /* This error can occur when the data stage of the BOT
                         * has stalled. As sub-item 5 of 6.7.2 of the BOT
                         * specification, we have clear the stall and then try
                         * to get the CSW. */

                        msdInstanceInfo->standardRequestDone = false;
                        if(USB_HOST_DevicePipeHaltClear(msdInstanceInfo->bulkInPipeHandle, &requestHandle, (uintptr_t)(msdInstanceIndex)) == USB_HOST_RESULT_SUCCESS)
                        {
                            /* The request was accepted. Wait for completion */
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_IN_PIPE_STALLED_CLEAR_WAIT;
                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_IN_PIPE_STALLED_CLEAR_WAIT:

                        /* Here we are waiting for the standard request to complete */
                        if(msdInstanceInfo->standardRequestDone)
                        {
                            /* The request had completed. Now we try to get the
                             * CSW. */
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d MSD IN endpoint Clear Feature Done.", msdInstanceIndex);
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d Trying to get CSW.", msdInstanceIndex);
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_NO_ERROR;

                            /* The CSW completion will be completed in the
                             * transfer tasks. So we set the state of the
                             * transfer tasks to indicate this. */
                            msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CSW;

                            /* Launch the CSW */
                            USB_HOST_DeviceTransfer(msdInstanceInfo->bulkInPipeHandle, &transferHandle, msdInstanceInfo->msdCSW, 13, msdInstanceIndex);

                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_OUT_PIPE_STALLED:

                        /* This error can occur when the data stage of the BOT
                         * has stalled. As sub-item 5 of 6.7.3 of the BOT
                         * specification, we have clear the stall and then try
                         * to get the CSW. */

                        msdInstanceInfo->standardRequestDone = false;
                        if(USB_HOST_DevicePipeHaltClear(msdInstanceInfo->bulkOutPipeHandle, &requestHandle, (uintptr_t)(msdInstanceIndex)) == USB_HOST_RESULT_SUCCESS)
                        {
                            /* The request was accepted. Wait for completion */
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_OUT_PIPE_STALLED_CLEAR_WAIT;
                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_OUT_PIPE_STALLED_CLEAR_WAIT:

                        /* Here we are waiting for the standard request to complete */
                        if(msdInstanceInfo->standardRequestDone)
                        {
                            /* The request had completed. Now we try to get the
                             * CSW. */
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d MSD OUT endpoint Clear Feature Done.", msdInstanceIndex);
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d Trying to get CSW.", msdInstanceIndex);
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_NO_ERROR;

                            /* The CSW completion will be completed in the
                             * transfer tasks. So we set the state of the
                             * transfer tasks to indicate this. */
                            msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CSW;

                            /* Launch the CSW */
                            USB_HOST_DeviceTransfer(msdInstanceInfo->bulkInPipeHandle, &transferHandle, msdInstanceInfo->msdCSW, 13, msdInstanceIndex);

                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_STALLED:

                        /* This means a CSW request has stalled. Figure 2 of the
                         * BOT specification specifies how this should be
                         * handled */

                        msdInstanceInfo->standardRequestDone = false;
                        if(USB_HOST_DevicePipeHaltClear(msdInstanceInfo->bulkInPipeHandle, &requestHandle, (uintptr_t)(msdInstanceIndex)) == USB_HOST_RESULT_SUCCESS)
                        {
                            /* The request was accepted. Wait for completion */
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_STALLED_IN_PIPE_CLEAR_WAIT;
                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_STALLED_IN_PIPE_CLEAR_WAIT:

                        /* In this state, we are waiting for clear feature
                         * on the IN pipe to complete.
                         * */

                        if(msdInstanceInfo->standardRequestDone)
                        {
                            /* The request had completed. Now we try to get the
                             * CSW again. */
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d CSW Stall Bulk IN endpoint Clear Feature Done.", msdInstanceIndex);
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d Trying to get CSW.", msdInstanceIndex);

                            /* The CSW completion will be completed in the
                             * transfer tasks. So we set the state of the
                             * transfer tasks to indicate this. */
                            msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CSW;
                            msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_RETRY;

                            /* Launch the CSW */
                            USB_HOST_DeviceTransfer(msdInstanceInfo->bulkInPipeHandle, &transferHandle, msdInstanceInfo->msdCSW, 13, msdInstanceIndex);

                        }
                        break;

                    case USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_RETRY:

                        /* We dont do any thing in this state. This is an
                         * indication state only. The transfer tasks state
                         * machine will check if the error tasks state machine
                         * is in this state. If so it will know that this is the
                         * second attempt at getting the CSW. */

                        break;

                    default:
                        break;
                }

                if(transferIsDone)
                {
                    if(msdInstanceInfo->transferObj.callback != NULL)
                    {
                        /* Let the caller who initiated the command know that this is
                         * done */
                        msdInstanceInfo->transferObj.callback(msdInstanceInfo->transferObj.lunHandle,
                                (USB_HOST_MSD_TRANSFER_HANDLE)(&msdInstanceInfo->transferObj), 
                                msdResult, processedBytes, 
                                msdInstanceInfo->transferObj.context);
                    }

                    /* Return the transfer object back */
                    msdInstanceInfo->transferObj.inUse = false;

                    /* Make the transfer state ready for another transfer */
                    msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_READY;
                }
            }
        }
    }
}

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_ControlTransferCallback
    (
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        USB_HOST_REQUEST_HANDLE requestHandle,
        USB_HOST_RESULT result,
        size_t size,
        uintptr_t context
    );

  Summary:
    This function is called when a control transfer completes.

  Description:
    This function is called when a control transfer completes.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_ControlTransferCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
)
{
    USB_HOST_MSD_INSTANCE * msdInstanceInfo;
    
    /* The context will be a pointer to the MSD instance */
    msdInstanceInfo = (USB_HOST_MSD_INSTANCE *)(context);
    
    /* Update the request object with the result */
    msdInstanceInfo->controlTransferResult = result;
    msdInstanceInfo->controlTransferSize = size;
    msdInstanceInfo->controlTransferDone = true; 

    return;
}

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_GetMaxLUNPacketCreate
   ( 
       USB_SETUP_PACKET * setupPacket,
       uint8_t bInterfaceNumber
   )

  Summary:
    This function will create the Get MAX LUN

  Description:
    This function will create the Get MAX LUN

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_GetMaxLUNPacketCreate
(
   USB_SETUP_PACKET * setupPacket,
   uint8_t bInterfaceNumber
)
{
    /* Fill setup packet. The Get MAX LUN is target to the interface */
    setupPacket->bmRequestType  = ( USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_INTERFACE );

    /* Setup the other setup packet values */
    setupPacket->bRequest =  USB_MSD_GET_MAX_LUN ;
    setupPacket->wValue = 0x0000;
    setupPacket->wIndex = bInterfaceNumber;
    setupPacket->wLength = 0x01;
}

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_ResetPacketCreate
   ( 
       USB_SETUP_PACKET * setupPacket,
       uint8_t bInterfaceNumber
   )

  Summary:
    This function will create the Reset Packet.

  Description:
    This function will create the Reset Packet.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_ResetPacketCreate
( 
   USB_SETUP_PACKET * setupPacket,
   uint8_t bInterfaceNumber
)
{
    /* Create the setup packet */
    setupPacket->bmRequestType  = ( USB_SETUP_DIRN_HOST_TO_DEVICE | USB_SETUP_TYPE_CLASS | USB_SETUP_RECIPIENT_INTERFACE ); 
    setupPacket->bRequest =  USB_MSD_RESET ;
    setupPacket->wValue = 0x0000;
    setupPacket->wIndex = bInterfaceNumber;
    setupPacket->wLength = 0x00;
}

// *****************************************************************************
/* Function:
    void _USB_HOST_MSD_Initialize(void * msdInitData)

  Summary:
    This function is called when the Host Layer is initializing.

  Description:
    This function is called when the Host Layer is initializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_Initialize(void * msdInitData)
{
    /* Make sure all the pipe handles are invalid */
    
    int iterator;
    USB_HOST_MSD_INSTANCE * msdInstanceInfo;
    
    for (iterator = 0; iterator < USB_HOST_MSD_INSTANCES_NUMBER; iterator ++)
    {
        msdInstanceInfo = &gUSBHostMSDInstance[iterator];
        msdInstanceInfo->controlPipeHandle = USB_HOST_CONTROL_PIPE_HANDLE_INVALID;
        msdInstanceInfo->bulkInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        msdInstanceInfo->bulkOutPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
        msdInstanceInfo->msdCBW = (USB_MSD_CBW *)(&gUSBHostMSDCBW[iterator][0]);
        msdInstanceInfo->msdCSW = (USB_MSD_CSW *)(&gUSBHostMSDCSW[iterator][0]);

        /* We create mutexes at initialization time. This way we dont have to
         * deal with having to delete the mutex when the interface is released.
         * A mutex should not be deleted in the interrupt context and the
         * interface release function could be called from an interrupt context.
         * */

        if(OSAL_RESULT_TRUE != OSAL_MUTEX_Create(&(msdInstanceInfo->mutexMSDInstanceObject)))
        {
            /* The mutex could not be created. We cannot
             * continue with using the instance object. */
            SYS_DEBUG_PRINT(SYS_ERROR_FATAL, "\r\nUSB Host MSD: Could not create Mutex for MSD instance %d",iterator);
        }
    }
}

// *****************************************************************************
/* Function:
    void _USB_HOST_MSD_Deinitialize(void)

  Summary:
    This function is called when the Host Layer is deinitializing.

  Description:
    This function is called when the Host Layer is deinitializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_Deinitialize(void)
{
    /* This function is not implemented in this release of the driver */
}

// *****************************************************************************
/* Function:
    void _USB_HOST_MSD_Reinitialize(void)

  Summary:
    This function is called when the Host Layer is reinitializing.

  Description:
    This function is called when the Host Layer is reinitializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_Reinitialize(void * msdInitData)
{
    /* This function is not implemented in this release of the driver */
}

// *****************************************************************************
/* Function:
    void _USB_HOST_MSD_InterfaceAssign 
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

void _USB_HOST_MSD_InterfaceAssign 
(
    USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    size_t nInterfaces,
    uint8_t * descriptor
)
{
    int driverIndex;
    USB_HOST_MSD_INSTANCE * msdInstanceInfo = NULL;
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle;
    USB_HOST_ENDPOINT_DESCRIPTOR_QUERY endpointQuery;
    USB_ENDPOINT_DESCRIPTOR * endpointDescriptor;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;
    USB_HOST_PIPE_HANDLE controlPipeHandle, bulkInPipeHandle, bulkOutPipeHandle;
    bool result = false;

    /* We first try to open a control pipe to the device. Also initialize the
     * local pipe handles  */

    controlPipeHandle = USB_HOST_DeviceControlPipeOpen(deviceObjHandle);
    bulkInPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
    bulkOutPipeHandle = USB_HOST_PIPE_HANDLE_INVALID;
    interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)(descriptor);
    interfaceHandle = interfaces[0];

    if(controlPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
    {
        /* The control pipe could be opened. Now we check if the interface
         * descriptors contain the endpoints needed to operate the device. We
         * first find the Bulk IN endpoint and then try to open a pipe. To do
         * this, a query must be setup first. */

        USB_HOST_DeviceEndpointQueryContextClear(&endpointQuery);
        endpointQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_DIRECTION|USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE;
        endpointQuery.direction  = USB_DATA_DIRECTION_DEVICE_TO_HOST;
        endpointQuery.transferType = USB_TRANSFER_TYPE_BULK;

        /* Now find the endpoint */
        endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointQuery);
        if(endpointDescriptor != NULL)
        {
            /* We have found the IN bulk endpoint. Try opening a pipe on this 
             * endpoint */

            bulkInPipeHandle = USB_HOST_DevicePipeOpen(interfaceHandle, endpointDescriptor->bEndpointAddress);
            if(bulkInPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
            {
                /* Now open the the bulk out pipe */

                USB_HOST_DeviceEndpointQueryContextClear(&endpointQuery);
                endpointQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_DIRECTION|USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE;
                endpointQuery.direction  = USB_DATA_DIRECTION_HOST_TO_DEVICE;
                endpointQuery.transferType = USB_TRANSFER_TYPE_BULK;

                /* Now find the endpoint */
                endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointQuery);
                if(endpointDescriptor != NULL)
                {
                    /* Found the bulk out endpoint. Now open the pipe */
                    bulkOutPipeHandle = USB_HOST_DevicePipeOpen(interfaceHandle, endpointDescriptor->bEndpointAddress);
                    if(bulkOutPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
                    {
                        /* All pipes open. Now we search for a MSD Client
                         * driver instance object that can be assigned to
                         * this device */
                        for ( driverIndex = 0 ; driverIndex < USB_HOST_MSD_INSTANCES_NUMBER ; driverIndex++ )
                        {
                            if (!gUSBHostMSDInstance[driverIndex].assigned)
                            {
                                /* Found a free instance object. We
                                 * assing the pipe handles, the
                                 * interface handle and the device
                                 * object handle. */

                                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Host Instance %d Assigned.",driverIndex);
                                gUSBHostMSDInstance[driverIndex].assigned = true;
                                msdInstanceInfo = &gUSBHostMSDInstance[driverIndex];
                                msdInstanceInfo->transferObj.inUse = false;
                                msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_READY;
                                msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_NO_ERROR;
                                msdInstanceInfo->msdState = USB_HOST_MSD_STATE_GET_MAX_LUN;
                                msdInstanceInfo->controlPipeHandle = controlPipeHandle;
                                msdInstanceInfo->bulkInPipeHandle = bulkInPipeHandle;
                                msdInstanceInfo->bulkOutPipeHandle = bulkOutPipeHandle;
                                msdInstanceInfo->interfaceHandle = interfaces[0];
                                msdInstanceInfo->deviceObjHandle = deviceObjHandle;
                                interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)(descriptor);
                                msdInstanceInfo->bInterfaceNumber = interfaceDescriptor->bInterfaceNumber;
                                result = true;
                                break;
                            }
                        }
                    }
                    else
                    {
                        /* If an error callback function is defined, then call it with the error
                         * code */
                        _USB_HOST_MSD_ERROR_CALLBACK(deviceObjHandle, USB_HOST_MSD_ERROR_CODE_FAILED_PIPE_OPEN);

                        SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "\r\nUSB Host MSD: Could not open Bulk OUT pipe.");
                    }
                }
                else
                {
                    /* If an error callback function is defined, then call it with the error
                     * code */
                    _USB_HOST_MSD_ERROR_CALLBACK(deviceObjHandle, USB_HOST_MSD_ERROR_CODE_NOT_FOUND_BULK_OUT_ENDPOINT);
                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host MSD: Could not find OUT endpoint in interface descriptor.");
                }
            }
            else
            {
                /* If an error callback function is defined, then call it with the error
                 * code */
                _USB_HOST_MSD_ERROR_CALLBACK(deviceObjHandle, USB_HOST_MSD_ERROR_CODE_FAILED_PIPE_OPEN);
                SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "\r\nUSB Host MSD: Could not open Bulk IN pipe.");
            }
        }
        else
        {
            /* If an error callback function is defined, then call it with the error
             * code */
            _USB_HOST_MSD_ERROR_CALLBACK(deviceObjHandle, USB_HOST_MSD_ERROR_CODE_NOT_FOUND_BULK_IN_ENDPOINT);
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host MSD: Could not find IN endpoint in interface descriptor.");
        }
    }

    if (result == false)
    {
        /* Let the host know that this interface cannot be processed */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host MSD: Could not allocate MSD object.");

        /* If an error callback function is defined, then call it with the error
         * code */
        _USB_HOST_MSD_ERROR_CALLBACK(deviceObjHandle, USB_HOST_MSD_ERROR_CODE_INSUFFICIENT_INSTANCES);
        
        /* Close any pipes that were opened */
        if(USB_HOST_PIPE_HANDLE_INVALID != bulkInPipeHandle)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host MSD: Closing Bulk IN pipe.");
            USB_HOST_DevicePipeClose(bulkInPipeHandle);
        }

        if(USB_HOST_PIPE_HANDLE_INVALID != bulkOutPipeHandle)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host MSD: Closing Bulk OUT pipe.");
            USB_HOST_DevicePipeClose(bulkOutPipeHandle);
        }

        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host MSD: Releasing Interface.");
        _USB_HOST_MSD_InterfaceRelease(interfaceHandle);
    }

    return;
}

// *****************************************************************************
/* Function:
    void USB_HOST_MSD_InterfaceRelease
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

void _USB_HOST_MSD_InterfaceRelease
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    /* The device is detached or the configuration has changed. Shut down
     * everything for this interface */
    _USB_HOST_MSD_InstanceRelease(interfaceHandle);
}

// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_MSD_InterfaceEventHandler
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

USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_MSD_InterfaceEventHandler
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_DEVICE_INTERFACE_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    USB_HOST_MSD_INSTANCE * msdInstanceInfo;
    USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA * transferCompleteEventData;

    /* The context at the time of scheduling a transfer is the MSD Instance
     * Index */

    msdInstanceInfo = &gUSBHostMSDInstance[context];
    
    switch(event)
    {
        case USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE:
    
            /* This means a transfer completed. Update the transfer state
             * machine. */
            transferCompleteEventData = (USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA *)(eventData);
            _USB_HOST_MSD_TransferTasks (context, transferCompleteEventData->result, transferCompleteEventData->length);
            break;
            
        case USB_HOST_DEVICE_INTERFACE_EVENT_SET_INTERFACE_COMPLETE:
            
            break;
            
        case USB_HOST_DEVICE_INTERFACE_EVENT_PIPE_HALT_CLEAR_COMPLETE:
            /* Let the main state machine know that the standard request is done
             * */
            msdInstanceInfo->standardRequestDone = true;
            break;
            
        default:
            break;
    }


    return(USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE);
}

// *****************************************************************************
/* Function:
    void USB_HOST_MSD_InterfaceTasks
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

void _USB_HOST_MSD_InterfaceTasks
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    int msdInstanceIndex;
    bool interruptIsEnabled;
    int iterator;
    USB_HOST_MSD_INSTANCE * msdInstanceInfo;
    USB_HOST_TRANSFER_HANDLE  transferHandle;
     
    /* Get the MSD instance for this interface */
    msdInstanceIndex = _USB_HOST_MSD_InterfaceHandleToMSDInstance(interfaceHandle);

    if(msdInstanceIndex >= 0)
    {
        msdInstanceInfo = &gUSBHostMSDInstance[msdInstanceIndex];
        
        switch(msdInstanceInfo->msdState)
        {
            case USB_HOST_MSD_STATE_NOT_READY:

                /* This object is in the process of getting initialized. Dont do
                 * anything yet */
                break;

            case USB_HOST_MSD_STATE_GET_MAX_LUN:

                /* In this state we launch the Get Max LUN request. Create the
                 * Get Max LUN packet */
                _USB_HOST_MSD_GetMaxLUNPacketCreate(&msdInstanceInfo->setupPacket, msdInstanceInfo->bInterfaceNumber);

                /* Set the flag indicating we are waiting for the control
                 * request to complete */
                msdInstanceInfo->controlTransferDone = false;

                /* Launch the request. We give a pointer to the
                 * logicalUnitNumber member of the instance object. The received
                 * data should be stored there. */

                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d Trying to send Get Max LUN Request.", msdInstanceIndex);
                
                if(USB_HOST_DeviceControlTransfer(msdInstanceInfo->controlPipeHandle, 
                        &transferHandle,
                        &msdInstanceInfo->setupPacket, 
                        &msdInstanceInfo->logicalUnitNumber,
                        _USB_HOST_MSD_ControlTransferCallback,
                        (uintptr_t)(msdInstanceInfo)) == USB_HOST_RESULT_SUCCESS)
                {
                    /* Update state. We will wait for the Get Max LUN request to
                     * complete. */
                    msdInstanceInfo->msdState = USB_HOST_MSD_STATE_WAITING_GET_MAX_LUN;
                }
                else
                {
                    /* Wait in the same state */
                }

                break;
                
            case USB_HOST_MSD_STATE_WAITING_GET_MAX_LUN:
                
                /* Here we wait for the Get Max Lun to complete */
                if(msdInstanceInfo->controlTransferDone)
                {
                    /* This means the control transfer completed. Check the 
                     * result */
                    if(msdInstanceInfo->controlTransferResult == USB_HOST_RESULT_FAILURE)
                    {
                        /* This means an unknown error has occurred. For now we
                         * will move the client driver to an error state. The
                         * device will not be accessible in the error state. */
                        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host MSD: MSD Instance %d Get Max LUN Request Unknown Failure.", msdInstanceIndex);
                        msdInstanceInfo->logicalUnitNumber = 0;
                        msdInstanceInfo->msdErrorCode = USB_HOST_MSD_ERROR_CODE_FAILED_GET_MAX_LUN;
                        msdInstanceInfo->msdState = USB_HOST_MSD_STATE_ERROR;
                    }
                    else if(msdInstanceInfo->controlTransferResult == USB_HOST_RESULT_REQUEST_STALLED)
                    {
                        /* Device can stall the Get Max LUN request when it does
                         * not support multiple LUNs. We set the number of LUNs
                         * to 1 */

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d Get Max LUN Request was stalled. Setting LUNs to 1.", msdInstanceIndex);
                        msdInstanceInfo->logicalUnitNumber = 1;
                    }
                    else if (msdInstanceInfo->controlTransferResult == USB_HOST_RESULT_SUCCESS) 
                    {
                        /* The Get Max LUN request passed. logicalUnitNumber
                         * should contain the number of LUNs (which could be 0).
                         * */

                        /* While the BOT specification (in section 3.2) states that
                         * the Get Max LUN request will return 0 if there are no
                         * LUNs associated with the device, device do return 0 if no
                         * LUNs are supported. We increment the LUN count by 1. */

                        msdInstanceInfo->logicalUnitNumber ++;
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: Get Max LUN Request was passed. LUNs = %d.", msdInstanceInfo->logicalUnitNumber);
                    }

                    if(msdInstanceInfo->msdState != USB_HOST_MSD_STATE_ERROR)
                    {
                        /* The process of initializing the SCSI instances for each
                         * LUN should not be interrupted, specially by a USB Detach
                         * interrupt. We make this process atomic. The
                         * USB_HOST_SCSI_Initialize function does not perform
                         * hardware access and is not a blocking function. So we are
                         * safe in that sense. */

                        interruptIsEnabled = SYS_INT_Disable();
                        for (iterator = 0; iterator < msdInstanceInfo->logicalUnitNumber; iterator ++)
                        {
                            /* Initialize the SCSI driver for every LUN */
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: Initializing SCSI for LUN %d.", iterator);
                            USB_HOST_SCSI_Initialize(USB_HOST_MSD_LUNHandleGet(iterator,msdInstanceIndex));
                        }

                        msdInstanceInfo->msdState = USB_HOST_MSD_STATE_READY;

                        if(interruptIsEnabled)
                        {
                            /* Re-enable the global interrupt */
                            SYS_INT_Enable();
                        }

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d is Ready.", msdInstanceIndex);
                    }
                    else
                    {
                        /* Get Max LUN failed. Note that the msdState becomes
                         * USB_HOST_MSD_STATE_ERROR in the Get Max LUN failure
                         * case. */
                    }
                }
                break;

            case USB_HOST_MSD_STATE_READY:
                
                /* Device is in a ready state. BOT protocol can be supported now
                 * Run the SCSI task routines for each LUN. These task routines
                 * perform the SCSI commands required to make sure that the SCSI
                 * device is ready. */
                for (iterator = 0; iterator < (msdInstanceInfo->logicalUnitNumber); iterator ++)
                {
                    USB_HOST_SCSI_Tasks(USB_HOST_MSD_LUNHandleGet(iterator,msdInstanceIndex));
                }
                break;

            case USB_HOST_MSD_STATE_ERROR:

                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d is entering error state", msdInstanceIndex);
                _USB_HOST_MSD_ERROR_CALLBACK(msdInstanceIndex, msdInstanceInfo->msdErrorCode); 
                msdInstanceInfo->msdState = USB_HOST_MSD_STATE_ERROR_HOLDING;
                break;

            case USB_HOST_MSD_STATE_ERROR_HOLDING:
                /* Device is in an error state. It must be unplugged. */
                break;

            default:
                break;
        }
    }
}

// *****************************************************************************
/* Function:
   USB_HOST_MSD_RESULT USB_HOST_MSD_Transfer
   (
       uint8_t * cdb,
       uint8_t cdbLength,
       void * data,
       size_t size,
       USB_HOST_MSD_TRANSFER_DIRECTION transferDirection,
       USB_HOST_MSD_TRANSFER_CALLBACK callback,
       uintptr_t context
   )

  Summary:
    This function schedules a MSD BOT transfer.

  Description:
    This function schedules a MSD BOT transfer. The command to be executed is
    specified in the cdb. This should be pointer to a 16 byte command descriptor
    block. The actual length of the command is specified by cdbLength. If there
    is data to be transferred, the pointer to the buffer is specified by data.
    The size of the buffer is specified in size. When the transfer completes,
    the callback function will be called. The context will be returned in the
    callback function.

  Remarks:
    None.
*/

USB_HOST_MSD_RESULT USB_HOST_MSD_Transfer
(
    USB_HOST_MSD_LUN_HANDLE lunHandle,
    uint8_t * cdb,
    uint8_t cdbLength,
    void * data,
    size_t size,
    USB_HOST_MSD_TRANSFER_DIRECTION transferDirection,
    USB_HOST_MSD_TRANSFER_CALLBACK callback,
    uintptr_t context
)
{
    USB_HOST_MSD_RESULT result;
    int iterator;
    USB_HOST_MSD_INSTANCE * msdInstanceInfo;
    int msdInstanceIndex;
    USB_HOST_RESULT hostResult;
    USB_HOST_TRANSFER_HANDLE transferHandle;

    if(USB_HOST_MSD_LUN_HANDLE_INVALID == lunHandle)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "\r\nUSB Host MSD: LUN Handle in USB_HOST_MSD_Transfer is not valid.");
        result = USB_HOST_MSD_RESULT_LUN_HANDLE_INVALID;
    }
    else
    {
        /* Get the MSD Instance Index from the LUN Handle */
        msdInstanceIndex = USB_HOST_MSD_INDEX(lunHandle);

        /* Get the pointer to the MSD instance */
        msdInstanceInfo = &gUSBHostMSDInstance[msdInstanceIndex];

        if(!msdInstanceInfo->assigned)
        {
            /* This object is not valid */
            SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "\r\nUSB Host MSD: MSD Instance %d in USB_HOST_MSD_Transfer is not valid.", msdInstanceIndex);
            result = USB_HOST_MSD_RESULT_FAILURE;
        }
        else
        {
            /* Try obtaining the mutex. In an RTOS application, the thread could
             * block at this point */
            
            if(OSAL_MUTEX_Lock(&(msdInstanceInfo->mutexMSDInstanceObject), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
            {
                /* We got the mutex. Now check if the BOT transfer object is
                 * free and if the MSD state machine can accept transfer
                 * requests. */


                if((msdInstanceInfo->msdState == USB_HOST_MSD_STATE_READY) && 
                        (!msdInstanceInfo->transferObj.inUse) &&
                        (msdInstanceInfo->transferState == USB_HOST_MSD_TRANSFER_STATE_READY))
                {
                    /* We can proceed with the request. Grab the transfer object */
                    msdInstanceInfo->transferObj.inUse = true;

                    /* We can release the mutex now */
                    OSAL_MUTEX_Unlock(&(msdInstanceInfo->mutexMSDInstanceObject));
                    
                    /* Setup the CBW */
                    msdInstanceInfo->msdCBW->dCBWSignature = USB_MSD_VALID_CBW_SIGNATURE;
                    msdInstanceInfo->msdCBW->dCBWTag = USB_MSD_VALID_CBW_TAG;
                    msdInstanceInfo->msdCBW->bCBWCBLength = cdbLength;
                    msdInstanceInfo->msdCBW->dCBWDataTransferLength = size;
                    msdInstanceInfo->msdCBW->bmCBWFlags.value = transferDirection;
                    
                    /* Reset the phase error flag. This flag gets set is a phase
                     * error has occurred. */
                    msdInstanceInfo->cswPhaseError = false;

                    msdInstanceInfo->msdCBW->bCBWLUN = USB_HOST_MSD_LUN(lunHandle);

                    /* Copy the cdb. It should be zero padded */
                    for(iterator = 0; iterator < 16; iterator ++)
                    {
                        /* Clear the command block */
                        msdInstanceInfo->msdCBW->CBWCB[iterator] = 0;
                    }

                    /* Now copy the command */
                    for(iterator = 0; iterator < cdbLength; iterator ++)
                    {
                        msdInstanceInfo->msdCBW->CBWCB[iterator] = cdb[iterator];
                    }

                    /* Save the caller data in the transfer object */
                    msdInstanceInfo->transferObj.callback = callback;
                    msdInstanceInfo->transferObj.context = context;
                    msdInstanceInfo->transferObj.size = size;
                    msdInstanceInfo->transferObj.transferDirection = transferDirection;
                    msdInstanceInfo->transferObj.cdb = cdb;
                    msdInstanceInfo->transferObj.cdbLength = cdbLength;
                    msdInstanceInfo->transferObj.lunHandle = lunHandle;
                    msdInstanceInfo->transferObj.buffer = data;

                    msdInstanceInfo->transferState = USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CBW;
                    msdInstanceInfo->transferErrorTaskState = USB_HOST_MSD_TRANSFER_ERROR_STATE_NO_ERROR;

                    /* CBW must go out on the bulk out pipe handle */
                    hostResult = USB_HOST_DeviceTransfer(msdInstanceInfo->bulkOutPipeHandle, &transferHandle, msdInstanceInfo->msdCBW, 31, (uintptr_t)(msdInstanceIndex));

                    /* Map the result */
                    result = _USB_HOST_MSD_HostResultToMSDResultMap(hostResult); 
                }
                else
                {
                    /* Un-commenting this line could result in too many messages on the console */
                    /*SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d is busy. Cannot schedule BOT.", msdInstanceIndex);*/
                    OSAL_MUTEX_Unlock(&(msdInstanceInfo->mutexMSDInstanceObject));
                    _USB_HOST_MSD_ERROR_CALLBACK(msdInstanceIndex, USB_HOST_MSD_ERROR_CODE_TRANSFER_BUSY);
                    result = USB_HOST_MSD_RESULT_BUSY;
                }
            }
            else
            {
                /* Could not get the mutex lock */
                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host MSD: MSD Instance %d mutex lock failed in BOT transfer request.", msdInstanceIndex);
                result = USB_HOST_MSD_RESULT_BUSY;
            }
        }
    }

    return(result);
}
