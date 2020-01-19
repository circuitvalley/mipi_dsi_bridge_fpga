/*******************************************************************************
  USB Host SCSI Driver Implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_scsi.c

  Summary:
    This file contains implementations of both private and public functions
    of the USB Host MSD SCSI client driver.

  Description:
    This file contains the USB Host MSD SCSI client driver implementation. This
    file should be included in the project if USB MSD devices are to be
    supported.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sub-license terms in the accompanying license agreement).

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

#include "usb/usb_host_msd.h"
#include "usb/usb_host_scsi.h"
#include "usb/scsi.h"
#include "usb/src/usb_host_scsi_local.h"
#include "system/fs/sys_fs_media_manager.h"
#include "system/debug/sys_debug.h"

/******************************************************
 * USB HOST MSD SCSI Instance object. One for each LUN
 ******************************************************/

USB_HOST_SCSI_INSTANCE_OBJ gUSBHostSCSIObj[USB_HOST_MSD_LUN_NUMBERS];

/******************************************************
 * USB HOST MSD SCSI buffers needed for SCSI operation
 ******************************************************/

uint8_t gUSBSCSIBuffer[USB_HOST_MSD_LUN_NUMBERS][256] SCSI_COHERENT_ATTRIBUTE __attribute__((aligned(16)));

/*****************************************************
 * USB HOST SCSI Attach Listeners.
 *****************************************************/
USB_HOST_SCSI_ATTACH_LISTENER_OBJ gUSBHostSCSIAttachListener[USB_HOST_SCSI_ATTACH_LISTENERS_NUMBER];

/*****************************************************
 * Media functions table that is exported to the
 * file system. Structure is defined by the file
 * system.
 *****************************************************/
SYS_FS_MEDIA_FUNCTIONS gUSBHostMSDSCSIMediaFunctions =
{
    .mediaStatusGet     = USB_HOST_SCSI_MediaStatusGet,
    .mediaGeometryGet   = USB_HOST_SCSI_MediaGeometryGet,
    .sectorRead         = USB_HOST_SCSI_SectorRead,
    .sectorWrite        = USB_HOST_SCSI_SectorWrite,
    .eventHandlerset    = USB_HOST_SCSI_EventHandlerSet,
    .open               = USB_HOST_SCSI_Open,
    .close              = USB_HOST_SCSI_Close,
    .tasks              = USB_HOST_SCSI_TransferTasks
};

/****************************************************
 * If the USB_HOST_SCSI_ERROR_CALLBACK option is set
 * to true, then this declares a callback function.
 ****************************************************/
_USB_HOST_SCSI_ERROR_CALLBACK_DECLARE

// ******************************************************************************
// ******************************************************************************
// Local Functions
// ******************************************************************************
// ******************************************************************************

// *****************************************************************************
/* Function:
    void * _USB_HOST_SCSI_TimerCallback
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

void _USB_HOST_SCSI_TimerCallback(uintptr_t context, uint32_t currtick)
{
    /* The handle is actually a pointer to the SCSI object */
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj = ((USB_HOST_SCSI_INSTANCE_OBJ *)(context));
    scsiObj->timerExpired = true;
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_Transfer 
    (
        USB_HOST_SCSI_HANDLE scsiHandle,
        USB_HOST_MSD_TRANSFER_HANDLE * transferHandle,
        uint32_t startSector
        uint32_t nSectors
        void * buffer
        USB_HOST_MSD_TRANSFER_DIRECTION direction
    )

  Summary:
    This function performs a block read or block write.

  Description:
    This function performs a block read or block write.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_Transfer 
(
    USB_HOST_SCSI_HANDLE scsiHandle,
    USB_HOST_MSD_TRANSFER_HANDLE * transferHandle,
    uint32_t startSector,
    uint32_t nSectors,
    void * buffer,
    USB_HOST_MSD_TRANSFER_DIRECTION direction
)
{
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    USB_HOST_SCSI_COMMAND_OBJ * commandObj;
    USB_HOST_MSD_RESULT result;
    USB_HOST_MSD_TRANSFER_HANDLE temp, * commandHandle;

    /* If the transfer handle parameter is NULL, we set up a local variable to
     * temporarily store the transfer handle */

    if(transferHandle == NULL)
    {
        commandHandle = &temp;
    }
    else
    {
        commandHandle = transferHandle;
    }

    /* Validate the handle */
    if((scsiHandle == USB_HOST_SCSI_HANDLE_INVALID) || (scsiHandle == 0))
    {
        *commandHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    }
    else
    {
        /* The handle is actually a pointer to the SCSI object */
        scsiObj = (USB_HOST_SCSI_INSTANCE_OBJ *)(scsiHandle);

        if((scsiObj->inUse) && (scsiObj->state == USB_HOST_SCSI_STATE_READY) &&
                (!scsiObj->commandObj.inUse) && (scsiObj->isMediaReady))
        {

            /* Check if the media is write protected */
            if((direction == USB_HOST_MSD_TRANSFER_DIRECTION_HOST_TO_DEVICE) && (scsiObj->isWriteProtected))
            {
                *commandHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
            }
            else
            {
                /* This means the SCSI instance is valid and the command object is
                 * available. Start by grabbing the command object */

                scsiObj->commandObj.inUse = true;
                commandObj = &scsiObj->commandObj;

                /* Set up the command based on direction */
                if(direction == USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST)
                {
                    commandObj->cdb[0] = USB_HOST_SCSI_READ10_COMMAND;
                    commandObj->cdb[1] = 0x00;
                }
                else
                {
                    commandObj->cdb[0] = USB_HOST_SCSI_WRITE10_COMMAND;

                    /* Set the FUA bit in the command so that the media will
                     * the completed the command only when the data has been 
                     * written to the media. */
                    commandObj->cdb[1] = 0x04;
                }

                /* Set up the sector address */
                commandObj->cdb[2] = (uint8_t)(startSector >> 24);
                commandObj->cdb[3] = (uint8_t)(startSector >> 16);
                commandObj->cdb[4] = (uint8_t)(startSector >> 8);
                commandObj->cdb[5] = (uint8_t)(startSector);

                /* The number of sectors to read or write */
                commandObj->cdb[6] = 0x00;
                commandObj->cdb[7] = (uint8_t)(nSectors >> 8);
                commandObj->cdb[8] = (uint8_t)(nSectors);
                commandObj->cdb[9] = 0x00;

                /* Initialize the command handle */
                *commandHandle = (SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE)(commandObj);

                /* Make sure the transfer state is set to a know value */
                scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_IDLE;

                /* Set the command delay to 0 */
                scsiObj->nCommandFailureTestUnitReadyAttempts = 0;

                /* Initialize the other members of the command object. These
                 * would be needed if an error has occurred and the block
                 * command needs to retried. */

                scsiObj->commandObj.nSectors = nSectors;
                scsiObj->commandObj.direction = direction;
                scsiObj->commandObj.buffer = buffer;

                /* Schedule the transfer */
                result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                        scsiObj->commandObj.cdb, 0x0A, buffer , (nSectors << 9) /* The left shift multiplies by 512 */, 
                        direction, _USB_HOST_SCSI_BlockTransferCallback, (uintptr_t)(scsiObj));

                if(result == USB_HOST_MSD_RESULT_SUCCESS)
                {
                    /* This means the transfer was scheduled. We dont have to do
                     * anything. */
                }
                else if(result == USB_HOST_MSD_RESULT_BUSY)
                {
                    /* The transfer could not be scheduled because the MSD
                     * driver is busy completing another transfer. We let the
                     * SCSI transfer tasks continue to try scheduling the
                     * transfer. The caller will still get a valid command
                     * handle */

                    scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_BLOCK_COMMAND_RETRY;
                    _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiHandle, USB_HOST_SCSI_ERROR_CODE_BOT_REQUEST_DEFERRED);
                }
                else 
                {
                    /* The request failed. Return the command object continue
                     * and return an invalid handle */

                    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host SCSI: Could not schedule BOT transfer");
                    *commandHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
                    commandObj->inUse = false;
                }
            }
        }
        else
        {
            /* The SCSI instance is not ready */
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host SCSI: SCSI Instance is not ready");
            *commandHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
            _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiHandle, USB_HOST_SCSI_ERROR_CODE_INSTANCE_BUSY);
        }
    }
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_CommandCallback 
    (
        USB_HOST_MSD_LUN_HANDLE lunHandle,
        USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
        USB_HOST_MSD_RESULT result,
        size_t size,
        uintptr_t context
    )

  Summary:
    This function is called when a SCSI command related MSD transfer has
    completed.

  Description:
    This function is called when a SCSI command related MSD transfer has
    completed.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_CommandCallback 
(
    USB_HOST_MSD_LUN_HANDLE lunHandle,
    USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
    USB_HOST_MSD_RESULT result,
    size_t size,
    uintptr_t context
)
{
    USB_HOST_SCSI_COMMAND_OBJ * commandObj;

    /* The context for this callback is the pointer to the command object
     * related to the callback. */
    commandObj = (USB_HOST_SCSI_COMMAND_OBJ *)(context);

    /* The processed size */
    commandObj->size = size;

    /* The result of the command */
    commandObj->result = result;
    
    /* Let the main state machine know that the command is completed */
    commandObj->commandCompleted = true;

    /* Release the command object */
    commandObj->inUse = false;
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_BlockTransferCallback 
    (
        USB_HOST_MSD_LUN_HANDLE lunHandle,
        USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
        USB_HOST_MSD_RESULT result,
        size_t size,
        uintptr_t context
    )

  Summary:
    This function is called when a MSD SCSI Block transfer has completed.

  Description:
    This function is called when a MSD SCSI Block transfer has completed.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_BlockTransferCallback 
(
    USB_HOST_MSD_LUN_HANDLE lunHandle,
    USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
    USB_HOST_MSD_RESULT result,
    size_t size,
    uintptr_t context
)
{
    int scsiObjIndex;
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    USB_HOST_SCSI_COMMAND_OBJ * commandObj;

    /* Get the SCSI object index from the lunHandle */
    scsiObjIndex = _USB_HOST_SCSI_LUNHandleToSCSIInstance(lunHandle);

    /* Get the pointer to the SCSI object */
    scsiObj = &gUSBHostSCSIObj[scsiObjIndex];

    /* Pointer to the command object */
    commandObj = &scsiObj->commandObj;

    /* The processed size */
    commandObj->size = size;

    /* The result of the command */
    commandObj->result = result;

    /* Let the main state machine know that the command is completed */
    commandObj->commandCompleted = true;

    if((result == USB_HOST_MSD_RESULT_SUCCESS) || (result == USB_HOST_MSD_RESULT_COMMAND_PASSED))
    {
        /* If there is an event handler registered, then call the event handler
         * */
        if(scsiObj->eventHandler != NULL)
        {
            /* Generate the event */
            (scsiObj->eventHandler)(USB_HOST_SCSI_EVENT_COMMAND_COMPLETE, (USB_HOST_SCSI_COMMAND_HANDLE)(commandObj), scsiObj->context);
        }
        
        /* Return the command object */
        commandObj->inUse = false;
    }
    else
    {
        /* The command failed. Transfer control to the transfer state machine to
         * find out why */

        scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_REQUEST_SENSE;
    }
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_TestUnitReadyCommand 
    (
        uint8_t * scsiCommand
    )

  Summary:
    Sets up the Test Unit Ready command.

  Description:
    This function sets up the Test Unit Ready Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_TestUnitReadyCommand (uint8_t * scsiCommand )
{
    /* Set up the Test Unit Ready command */
    scsiCommand[0] = SCSI_TEST_UNIT_READY;
    scsiCommand[1] = 0x00;
    scsiCommand[2] = 0x00;
    scsiCommand[3] = 0x00;
    scsiCommand[4] = 0x00;
    scsiCommand[5] = 0x00;

}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_RequestSenseCommand 
    (
        uint8_t * scsiCommand
    )

  Summary:
    Sets up the Request Sense command.

  Description:
    This function sets up the Request Sense Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_RequestSenseCommand (uint8_t * scsiCommand )
{
    /* Set up the Request Sense Command */
    scsiCommand[0] = SCSI_REQUEST_SENSE;
    scsiCommand[1] = 0x00;
    scsiCommand[2] = 0x00;
    scsiCommand[3] = 0x00;
    scsiCommand[4] = 0xFF;
    scsiCommand[5] = 0x00;
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_ReadCapacityCommand
    (
        uint8_t * scsiCommand
    )

  Summary:
    Sets up the Read Capacity Command.

  Description:
    This function sets up the Read Capacity Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_ReadCapacityCommand (uint8_t * scsiCommand )
{
    /* Set up the Read Capacity Command */
    scsiCommand[0] = SCSI_READ_CAPACITY;
    scsiCommand[1] = 0x00;
    scsiCommand[2] = 0x00;
    scsiCommand[3] = 0x00;
    scsiCommand[4] = 0x00;
    scsiCommand[5] = 0x00;
    scsiCommand[6] = 0x00;
    scsiCommand[7] = 0x00;
    scsiCommand[8] = 0x00;
    scsiCommand[9] = 0x00;
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_InquiryResponseCommand 
    (
        uint8_t * scsiCommand
    )

  Summary:
    Sets up the SCSI Inquiry Response command.

  Description:
    This function sets up the SCSI Inquiry Response Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_InquiryResponseCommand (uint8_t * scsiCommand )
{
    /* Set up the SCSI Inquiry command */
    scsiCommand[0] = SCSI_INQUIRY;
    scsiCommand[1] = 0x00;
    scsiCommand[2] = 0x00;
    scsiCommand[3] = 0x00;
    scsiCommand[4] = 0x24;
    scsiCommand[5] = 0x00;
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_ModeSense
    (
        uint8_t * scsiCommand
    )

  Summary:
    Sets up the SCSI Mode Sense command.

  Description:
    This function sets up the SCSI Mode Sense Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_ModeSense (uint8_t * scsiCommand )
{
    /* Set up the Read Format Capacity Command's CDB. */
    scsiCommand[0] = SCSI_MODE_SENSE;
    scsiCommand[1] = 0x00;
    scsiCommand[2] = 0x1C;
    scsiCommand[3] = 0x00;
    scsiCommand[4] = 192;
    scsiCommand[5] = 0x00;
    scsiCommand[9] = 0x00;
}

// ******************************************************************************
/* Function:
    int _USB_HOST_SCSI_LUNHandleToSCSIInstance 
    (
        USB_HOST_MSD_LUN_HANDLE lunHandle
    )

  Summary:
    This function maps a LUN Handle to the SCSI instance.

  Description:
    This function maps a LUN Handle to the SCSI instance. It returns the index
    of the SCSI object. It will return -1 if the object is not found.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_SCSI_LUNHandleToSCSIInstance
(
    USB_HOST_MSD_LUN_HANDLE lunHandle
)
{
    int iterator;
    int result = -1;
    
    for(iterator = 0;iterator < USB_HOST_MSD_LUN_NUMBERS; iterator ++)
    {
        if(gUSBHostSCSIObj[iterator].inUse)
        {
            if(gUSBHostSCSIObj[iterator].lunHandle == lunHandle)
            {
                /* Got a match */
                result = iterator;
                break;
            }
        }
    }

    return(result);
}

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_Initialize (USB_HOST_MSD_LUN_HANDLE lunHandle)

  Summary:
    This function is called by the MSD Host Client Driver for detected LUN.

  Description:
    This function is called by the MSD Host Client Driver for detected LUN. The
    function will then register this LUN as drive with the file system media
    manager, thus allowing the media manager to access this drive.

  Remarks:
    The application will never need to call this function. This function is
    always called the USB Host MSD Client Driver.
*/

void USB_HOST_SCSI_Initialize(USB_HOST_MSD_LUN_HANDLE lunHandle)
{
    int iterator;
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj = NULL;

    /* Search for a free SCSI object */
    for(iterator = 0; iterator < USB_HOST_MSD_LUN_NUMBERS; iterator ++)
    {
        if(!gUSBHostSCSIObj[iterator].inUse)
        {
            scsiObj = &gUSBHostSCSIObj[iterator];

            /* Clear this object */
            memset(scsiObj, 0, sizeof(USB_HOST_SCSI_INSTANCE_OBJ));

            /* Setup the rest of the object members whose initial values are
             * non-zero. */
            scsiObj->inUse = true;
            scsiObj->lunHandle = lunHandle;
            scsiObj->buffer = &gUSBSCSIBuffer[iterator][0];
            scsiObj->state = USB_HOST_SCSI_STATE_INQUIRY_RESPONSE;
            scsiObj->fsHandle = SYS_FS_MEDIA_HANDLE_INVALID;
            break;
        }
    }

    if(iterator >= USB_HOST_MSD_LUN_NUMBERS)
    {
        /* This means a SCSI instance object could not be allocated. We call the
         * Error callback function if this is defined. */

        _USB_HOST_SCSI_ERROR_CALLBACK(lunHandle, USB_HOST_SCSI_ERROR_CODE_INSUFFICIENT_INSTANCES);
    }
}

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_DeInitialize (USB_HOST_MSD_LUN_HANDLE lunHandle)

  Summary:
    This function is called by the MSD Host Client Driver for detached LUN.

  Description:
    This function is called by the MSD Host Client Driver for detached LUN. The
    function will then de-register this LUN from the file system media manager.

  Remarks:
    The application will never need to call this function. This function is
    always called the USB Host MSD Client Driver.
*/

void USB_HOST_SCSI_Deinitialize(USB_HOST_MSD_LUN_HANDLE lunHandle)
{
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    int scsiObjIndex;

    /* Find the SCSI object that owns this LUN */
    scsiObjIndex = _USB_HOST_SCSI_LUNHandleToSCSIInstance(lunHandle);

    if(scsiObjIndex >= 0)
    {
        /* Valid index. Get the pointer to the SCSI object */
        scsiObj = &gUSBHostSCSIObj[scsiObjIndex];
        
        /* Here we check if a SCSI transfer was in progress. It is possible that
         * SCSI driver would have received a request from the file system but
         * the device got detached before the SCSI driver could submit this 
         * request to the MSD Host Client driver. In such a case, the IRP would
         * not be scheduled and hence the SCSI driver would not get a transfer
         * failure callback when the underlying pipes are closed. The SCSI
         * command object in such case would still indicate that a command was
         * requested by the file system. In such a case we should let the file
         * system know that command has failed */
        
        if(scsiObj->commandObj.inUse)
        {
            /* This means a command was requested */
            
            if(scsiObj->eventHandler != NULL)
            {
                /* Generate the error event */
                (scsiObj->eventHandler)(USB_HOST_SCSI_EVENT_COMMAND_ERROR, (USB_HOST_SCSI_COMMAND_HANDLE)(&scsiObj->commandObj), scsiObj->context);
            }
        }
        
        if(scsiObj->eventHandler != NULL)
        {
            /* Let the client know that device has been detached. */
            scsiObj->eventHandler(USB_HOST_SCSI_EVENT_DETACH, USB_HOST_SCSI_COMMAND_HANDLE_INVALID, scsiObj->context);
        }

        /* De-register from the file system */
        if(scsiObj->fsHandle != SYS_FS_MEDIA_HANDLE_INVALID)
        {
            /* It may be possible that this specific LUN may have not registered
             * with the file system. Hence the check. */
            _USB_HOST_SCSI_FILE_SYSTEM_DEREGISTER(scsiObj->fsHandle);
            scsiObj->fsHandle = SYS_FS_MEDIA_HANDLE_INVALID;
        }

        /* Set the state to not ready */
        scsiObj->state = USB_HOST_SCSI_STATE_NOT_READY;

        /* Release this object back */
        scsiObj->inUse = false;
    }
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_TasksByIndex (int scsiIndex)

  Summary:
    This function is called by the USB_HOST_SCSI_Tasks to update the driver
    state machine.

  Description:
    This function is called by the USB_HOST_SCSI_Tasks to update the driver
    state machine.

  Remarks:
    This is local function and should not be called by the application.
*/

void _USB_HOST_SCSI_TasksByIndex(int scsiObjIndex)
{
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    USB_HOST_MSD_RESULT result;
    SCSI_SENSE_DATA * requestSenseResponse;
    int attachListenerIndex = 0;

    scsiObj = &gUSBHostSCSIObj[scsiObjIndex];

    switch(scsiObj->state)
    {
        case USB_HOST_SCSI_STATE_NOT_READY:
            /* We are not ready to do anything yet */
            break;

        case USB_HOST_SCSI_STATE_INQUIRY_RESPONSE:

            /* We get the SCSI Enquiry response. Although there isn't much
             * that we can do with this data */
            _USB_HOST_SCSI_InquiryResponseCommand(scsiObj->taskCommandObj.cdb);

            /* The commandCompleted flag will be updated in the callback.
             * Update the state and send the command.   */
            scsiObj->taskCommandObj.inUse = true;
            scsiObj->taskCommandObj.commandCompleted = false;

            result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                    scsiObj->taskCommandObj.cdb, 6, scsiObj->buffer, 36, 
                    USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST,
                    _USB_HOST_SCSI_CommandCallback, (uintptr_t)(&scsiObj->taskCommandObj));

            if(result == USB_HOST_MSD_RESULT_SUCCESS)
            {
                scsiObj->state = USB_HOST_SCSI_STATE_WAIT_INQUIRY_RESPONSE;
            }

            break;

        case USB_HOST_SCSI_STATE_WAIT_INQUIRY_RESPONSE:

            /* Here we wait for the Inquiry Response */
            if(scsiObj->taskCommandObj.commandCompleted)
            {
                /* This means the command has completed. We can launch the
                 * next command which is the Read Capacities command
                 * */
                SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Inquiry Response Successful", scsiObjIndex);
                scsiObj->state = USB_HOST_SCSI_STATE_READ_CAPACITY;
            }
            break;

        case USB_HOST_SCSI_STATE_READ_CAPACITY:

            /* Here we send the read capacity command */
            _USB_HOST_SCSI_ReadCapacityCommand(scsiObj->taskCommandObj.cdb);

            /* The commandCompleted flag will be updated in the callback.
             * Update the state and send the command.   */
            scsiObj->taskCommandObj.inUse = true;
            scsiObj->taskCommandObj.commandCompleted = false;

            result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                    scsiObj->taskCommandObj.cdb, 0xA, scsiObj->buffer, 0x8 , 
                    USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST,
                    _USB_HOST_SCSI_CommandCallback, (uintptr_t)(&scsiObj->taskCommandObj));

            if(result == USB_HOST_MSD_RESULT_SUCCESS)
            {
                /* Go to the next state only if the request was placed
                 * successfully. */
                scsiObj->state = USB_HOST_SCSI_STATE_WAIT_READ_CAPACITY;
            }

            break;

        case USB_HOST_SCSI_STATE_WAIT_READ_CAPACITY:

            /* Here we wait for the read capacity to complete */
            if(scsiObj->taskCommandObj.commandCompleted)
            {
                if(scsiObj->taskCommandObj.result == USB_HOST_MSD_RESULT_COMMAND_PASSED)
                {
                    /* We have the capacity information. Update the geometry
                     * object */

                    uint8_t * buffer = scsiObj->buffer;

                    /* The read and write will be blocking */
                    scsiObj->mediaGeometry.mediaProperty = (SYS_FS_MEDIA_WRITE_IS_BLOCKING|SYS_FS_MEDIA_READ_IS_BLOCKING);

                    /* There is one read, write and erase region */
                    scsiObj->mediaGeometry.numReadRegions = 1;
                    scsiObj->mediaGeometry.numWriteRegions = 1;
                    scsiObj->mediaGeometry.numEraseRegions = 1;

                    /* The size of the read region and size of the read block */
                    scsiObj->mediaRegionGeometry[0].blockSize = (buffer[7])|(buffer[6] << 8)|(buffer[5]<<16)| (buffer[4] << 24);
                    scsiObj->mediaRegionGeometry[0].numBlocks = ((buffer[3])|(buffer[2] << 8)|(buffer[1]<<16)| (buffer[0] << 24)) + 1;

                    /* The size of the write region and size of the read block */
                    scsiObj->mediaRegionGeometry[1].blockSize = scsiObj->mediaRegionGeometry[0].blockSize;
                    scsiObj->mediaRegionGeometry[1].numBlocks = scsiObj->mediaRegionGeometry[0].numBlocks;

                    /* The size of the erase region and size of the read block */
                    scsiObj->mediaRegionGeometry[2].blockSize = scsiObj->mediaRegionGeometry[0].blockSize;
                    scsiObj->mediaRegionGeometry[2].numBlocks = scsiObj->mediaRegionGeometry[0].numBlocks;

                    /* Adding the region specific geometry table */
                    scsiObj->mediaGeometry.geometryTable = scsiObj->mediaRegionGeometry;
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Read Capacity Successful", scsiObjIndex);
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Capacity is %d blocks", scsiObjIndex, scsiObj->mediaRegionGeometry[1].numBlocks);

                    /* Now we can check if the device is write protected.
                     * */
                    scsiObj->state = USB_HOST_SCSI_STATE_MODE_SENSE;
                }
                else
                {
                    /* The Read capacity command failed. We must do a
                     * Request sense to see why */

                    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host SCSI: SCSI Instance %d Read Capacity Failed. Requesting Sense Data", scsiObjIndex);
                    scsiObj->failedCommandState = USB_HOST_SCSI_STATE_READ_CAPACITY;
                    scsiObj->state = USB_HOST_SCSI_STATE_REQUEST_SENSE;
                    _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_READ_CAPACITY_FAILED);
                }
            }
            else
            {
                /* We must run the Transfer Error tasks while we are waiting
                 * for the transfer to complete. */                   
                USB_HOST_MSD_TransferErrorTasks(scsiObj->lunHandle);
            }

            break;

        case USB_HOST_SCSI_STATE_MODE_SENSE:

            /* In this state we send the mode sense command to find out if
             * the device is write protected */

            _USB_HOST_SCSI_ModeSense(scsiObj->taskCommandObj.cdb);

            /* The commandCompleted flag will be updated in the callback.
             * Update the state and send the command.   */
            scsiObj->taskCommandObj.inUse = true;
            scsiObj->taskCommandObj.commandCompleted = false;

            result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                    scsiObj->taskCommandObj.cdb, 0x06, scsiObj->buffer, 192 , 
                    USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST,
                    _USB_HOST_SCSI_CommandCallback, (uintptr_t)(&scsiObj->taskCommandObj));

            if(result == USB_HOST_MSD_RESULT_SUCCESS)
            {
                /* Go to the next state only if the request was placed
                 * successfully. */
                scsiObj->state = USB_HOST_SCSI_STATE_WAIT_MODE_SENSE;
            }

            break;

        case USB_HOST_SCSI_STATE_WAIT_MODE_SENSE:

            /* Here we are waiting for the mode sense request to complete */

            if(scsiObj->taskCommandObj.commandCompleted)
            {
                if(scsiObj->taskCommandObj.result == USB_HOST_MSD_RESULT_COMMAND_PASSED)
                {
                    /* Check if the device is write protected. This will be
                     * indicated by bit 7 of the third byte in the mode
                     * sense response. The third byte can be ready only if the
                     * device type reported back in byte 1 is SBC device type
                     * (0x00). There are some device which report a different
                     * device type. For such devices, we assume that write
                     * protect is disabled. */

                    if( ( scsiObj->buffer[1] == 0x00  ) && (scsiObj->buffer[2] & 0x80) )
                    {
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Mode Sense Passed. Media is write protected.", scsiObjIndex);
                        scsiObj->isWriteProtected = true;
                    }
                    else
                    {
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Mode Sense Passed. Media is not write protected.", scsiObjIndex);
                        scsiObj->isWriteProtected = false;
                    }

                    /* We are done with the Mode Sense command are now ready
                     * to move to the ready state */
                    scsiObj->state = USB_HOST_SCSI_STATE_MODE_SENSE_DONE;
                }
                else
                {
                    /* The mode sense command failed. Use the Request Sense
                     * state to find out why */
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Mode Sense Failed. Requesting Sense Data.", scsiObjIndex);
                    scsiObj->failedCommandState = USB_HOST_SCSI_STATE_MODE_SENSE;
                    scsiObj->state = USB_HOST_SCSI_STATE_REQUEST_SENSE;
                    _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_MODE_SENSE_FAILED);
                }
            }
            else
            {
                /* We must run the Transfer Error tasks while we are waiting
                 * for the transfer to complete. */                   
                USB_HOST_MSD_TransferErrorTasks(scsiObj->lunHandle);
            }

            break;

        case USB_HOST_SCSI_STATE_MODE_SENSE_DONE:

            /* This state is prior to get the ready state. In this state, we
             * register the media with the file system, initialize the state
             * of the detach state machine to start checking for detach. */

            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d is ready for block commands", scsiObjIndex);
            scsiObj->fsHandle = _USB_HOST_SCSI_FILE_SYSTEM_REGISTER((SYS_MODULE_OBJ)(scsiObj->lunHandle), 
                    scsiObjIndex, &gUSBHostMSDSCSIMediaFunctions, SYS_FS_MEDIA_TYPE_MSD);
            
            for (attachListenerIndex = 0; attachListenerIndex < USB_HOST_SCSI_ATTACH_LISTENERS_NUMBER; attachListenerIndex ++)
            {
                /* If there are any attach listeners registered, then call 
                 * each of these listeners */
                if(gUSBHostSCSIAttachListener[attachListenerIndex].inUse)
                {
                    gUSBHostSCSIAttachListener[attachListenerIndex].eventHandler((USB_HOST_SCSI_OBJ)(scsiObjIndex), 
                            gUSBHostSCSIAttachListener[attachListenerIndex].context);
                }
            }
            
            scsiObj->state = USB_HOST_SCSI_STATE_READY;
            scsiObj->detachTaskState = USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_SEND;

            /* Indicate that the media is ready to accept block
             * transfer commands. Reset the detach time out. */
            scsiObj->isMediaReady = true;
            scsiObj->detachTimeOut = 0;
            break;

        case USB_HOST_SCSI_STATE_READY:

            /* In this state the driver checks periodically if the media is
             * ready. This is needed for cases where the media is a card reader.
             * The card reader would always be plugged in but the card itself
             * could be plugged in and out. The driver uses the SCSI test unit
             * ready command to check if the card is responding. When running in
             * a RTOS, this check could block on the MSD BOT transfer object
             * which may be in use by a SCSI client thread. Another possible
             * case that could occur is when a media error has occurred. In such
             * a case, the driver will de-register the media from the file
             * system and will return to a state where it tries to bring up the
             * card again */

            if(scsiObj->isMediaError)
            {
                /* This means that some tpye of media error has occurred. We
                 * deregister the media from the file system and then try to
                 * bring up the media again. */
                SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host SCSI: SCSI Instance %d Media Error has occurred", scsiObjIndex);
                
                /* Send the detach event to all client event handlers */
                if(scsiObj->eventHandler != NULL)
                {
                    /* Let the client know that device has been detached. */
                    scsiObj->eventHandler(USB_HOST_SCSI_EVENT_DETACH, USB_HOST_SCSI_COMMAND_HANDLE_INVALID, scsiObj->context);
                }
                
                _USB_HOST_SCSI_FILE_SYSTEM_DEREGISTER(scsiObj->fsHandle);
                    
                scsiObj->fsHandle = SYS_FS_MEDIA_HANDLE_INVALID;
                scsiObj->state = USB_HOST_SCSI_STATE_INQUIRY_RESPONSE;
            }
            else
            {
                /* Continue to check if the media is present */
                _USB_HOST_SCSI_DetachDetectTasks(scsiObjIndex);
            }

            break;

        case USB_HOST_SCSI_STATE_REQUEST_SENSE:

            /* In this state, we send the request sense command */

            _USB_HOST_SCSI_RequestSenseCommand(scsiObj->taskCommandObj.cdb);

            /* The commandCompleted flag will be updated in the callback.
             * Update the state and send the command.   */
            scsiObj->taskCommandObj.inUse = true;
            scsiObj->taskCommandObj.commandCompleted = false;

            /* Send the command */
            result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                    scsiObj->taskCommandObj.cdb, 0x6, scsiObj->buffer, 18 , 
                    USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST,
                    _USB_HOST_SCSI_CommandCallback, (uintptr_t)(&scsiObj->taskCommandObj));

            if(result == USB_HOST_MSD_RESULT_SUCCESS)
            {
                /* Go to the next state only if the request was placed
                 * successfully. */
                scsiObj->state = USB_HOST_SCSI_STATE_WAIT_REQUEST_SENSE;
            }

            break;

        case USB_HOST_SCSI_STATE_WAIT_REQUEST_SENSE:

            /* In this state we wait for the command to complete */

            if(scsiObj->taskCommandObj.commandCompleted)
            {
                if(scsiObj->taskCommandObj.result == USB_HOST_MSD_RESULT_COMMAND_PASSED)
                {
                    /* Check the sense data */
                    requestSenseResponse = (SCSI_SENSE_DATA *)(scsiObj->buffer);

                    if(scsiObj->failedCommandState == USB_HOST_SCSI_STATE_MODE_SENSE)
                    {
                        /* This means the device does not support the mode sense
                         * command. That is okay. We assume that device is not
                         * write protected. Because the mode sense command
                         * occurs after the read capacity command, at this point
                         * we have all the data needed for the file system to
                         * open the device. */

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Sense Key Mode Sense not supported.", scsiObjIndex);
                        scsiObj->isWriteProtected = false;
                        scsiObj->state = USB_HOST_SCSI_STATE_MODE_SENSE_DONE;
                    }
                    else if((requestSenseResponse->SenseKey == SCSI_SENSE_UNIT_ATTENTION) ||
                            (requestSenseResponse->SenseKey == SCSI_SENSE_NOT_READY) ||
                            (requestSenseResponse->SenseKey == SCSI_SENSE_NO_SENSE))
                    {
                        /* This means something has changed or is changing on
                         * the device. Wait for the unit to be ready.  The
                         * Transcend card reader reports no sense on the flash
                         * reader when no media is inserted. */

                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Sense Key Unit Not Ready.", scsiObjIndex);
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Check if Unit is Ready.", scsiObjIndex);
                        scsiObj->state = USB_HOST_SCSI_STATE_TEST_UNIT_READY;
                        _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_REQUEST_SENSE_TEST_UNIT_READY);
                    }
                    else if(requestSenseResponse->SenseKey == SCSI_SENSE_MEDIUM_ERROR)
                    {
                        /* This could mean there is something terribly wrong
                         * with the device. We let the ready state catch this
                         * error. */
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Sense Key Medium Error.", scsiObjIndex);
                        scsiObj->isMediaError = true;
                        scsiObj->state = USB_HOST_SCSI_STATE_READY;
                        _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_REQUEST_SENSE_MEDIUM_ERROR);
                    }
                }
                else
                {
                    /* The SCSI Request Sense command must pass */
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Sense Command failed!", scsiObjIndex);
                    scsiObj->state = USB_HOST_SCSI_STATE_ERROR;
                    _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_REQUEST_SENSE_FAILURE);
                }
            }
            else
            {
                USB_HOST_MSD_TransferErrorTasks(scsiObj->lunHandle);
            }

            break;

        case USB_HOST_SCSI_STATE_TEST_UNIT_READY_DELAY_START:

            /* In this state we start a delay before launching the Test Unit
             * Ready command */

            scsiObj->timerExpired = false;
            scsiObj->commandDelayHandle = SYS_TMR_CallbackSingle( USB_HOST_SCSI_DETACH_TEST_UNIT_READY_INTERVAL, 
                    (uintptr_t )scsiObj, _USB_HOST_SCSI_TimerCallback);
            
            if (scsiObj->commandDelayHandle != SYS_TMR_HANDLE_INVALID)
            {
                /* The delay has started. Now wait for the delay to complete
                 * */

                scsiObj->state = USB_HOST_SCSI_STATE_TEST_UNIT_READY_DELAY_WAIT;
            }
            else
            {
                /* Continue to stay in this state till the delay could be
                 * scheduled */
            }
            break;

        case USB_HOST_SCSI_STATE_TEST_UNIT_READY_DELAY_WAIT:

            /* Check if the delay has completed */

            if(scsiObj->timerExpired)
            {
                /* Delay has completed. Send the test unit ready command */
                scsiObj->state = USB_HOST_SCSI_STATE_TEST_UNIT_READY;
                scsiObj->commandDelayHandle = SYS_TMR_HANDLE_INVALID ;
                
            }
            else
            {
                /* Stay in this state. Continue to check if the delay has
                 * completed */
            }
            break;

        case USB_HOST_SCSI_STATE_TEST_UNIT_READY:

            /* In this state the driver will check if the device is ready.
             * Grab the command object. Populate the command block with
             * command */
            
            _USB_HOST_SCSI_TestUnitReadyCommand(scsiObj->taskCommandObj.cdb);

            /* Send the command. The commandCompleted flag will be update in
             * the event handler  */
            
            scsiObj->taskCommandObj.inUse = true;
            scsiObj->taskCommandObj.commandCompleted = false;

            result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, scsiObj->taskCommandObj.cdb, 6, NULL, 0, 
                    USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST, _USB_HOST_SCSI_CommandCallback, 
                    (uintptr_t)(&scsiObj->taskCommandObj));

            if(result == USB_HOST_MSD_RESULT_SUCCESS)
            {
                /* The request was scheduled successfully. Wait for the
                 * request to complete */

                scsiObj->state = USB_HOST_SCSI_STATE_WAIT_TEST_UNIT_READY;
            }

            break;

        case USB_HOST_SCSI_STATE_WAIT_TEST_UNIT_READY:

            /* Here we are waiting for the test unit command to complete */

            if(scsiObj->taskCommandObj.commandCompleted)
            {
                /* The status of the test unit ready command can be read
                 * from the CSW */

                if(scsiObj->taskCommandObj.result == USB_HOST_MSD_RESULT_COMMAND_PASSED)
                {
                    /* The test unit ready command must have been called
                     * because some command failed. Try the command again */
                    scsiObj->state = scsiObj->failedCommandState;
                }
                else
                {
                    scsiObj->state = USB_HOST_SCSI_STATE_TEST_UNIT_READY_DELAY_START;
                }
            }
            else
            {
                /* Continue to run the transfer error tasks while waiting
                 * for the transfer to complete */
                USB_HOST_MSD_TransferErrorTasks(scsiObj->lunHandle);
            }

            break;

        case USB_HOST_SCSI_STATE_ERROR:

            /* We are in the state because a media error has occurred. There
             * is really nothing to be done here. The media is assumed
             * un-usable and should be un-plugged. */
            break;

        default:
            break;
    }
}

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_Tasks (USB_HOST_MSD_LUN_HANDLE lunHandle)

  Summary:
    This function is called by the MSD Host Client Driver to update the driver
    state machine.

  Description:
    This function is called by the MSD Host Client Driver to update the driver
    state machine.

  Remarks:
    The application will never need to call this function. This function is
    always called the USB Host MSD Client Driver.
*/

void USB_HOST_SCSI_Tasks(USB_HOST_MSD_LUN_HANDLE lunHandle)
{
    int scsiObjIndex;

    /* Find the SCSI object that own this LUN */
    scsiObjIndex = _USB_HOST_SCSI_LUNHandleToSCSIInstance(lunHandle);

    if(scsiObjIndex >= 0)
    {
        _USB_HOST_SCSI_TasksByIndex(scsiObjIndex);
    }
}

// ******************************************************************************
/* Function:
    USB_HOST_SCSI_HANDLE USB_HOST_SCSI_Open 
    (
        const SYS_MODULE_INDEX index, 
        const DRV_IO_INTENT intent
    );

  Summary:
    This function will open the specified instance of the SCSI driver.

  Description:
    This function will open the specified instance of the SCSI driver. The
    handle will allow the client to read and write data to the SCSI device.

  Remarks:
    This function is typically called by the file system media manager.
*/

USB_HOST_SCSI_HANDLE USB_HOST_SCSI_Open 
(
    SYS_MODULE_INDEX index, 
    DRV_IO_INTENT intent
)
{
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    USB_HOST_SCSI_HANDLE result;

    if(index >= USB_HOST_MSD_LUN_NUMBERS)
    {
        /* Invalid driver index */
        result = USB_HOST_SCSI_HANDLE_INVALID;
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB Host SCSI: Invalid LUN index in USB_HOST_SCSI_Open().");
    }
    else
    {
        scsiObj = &gUSBHostSCSIObj[index];
        if((scsiObj->inUse) && (scsiObj->state == USB_HOST_SCSI_STATE_READY))
        {
            /* This means this object is ready for use */
            result = (USB_HOST_SCSI_HANDLE)(scsiObj);
        }
        else
        {
            /* Not ready to be opened */
            _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)index, USB_HOST_SCSI_ERROR_CODE_OPEN_FAIL_ON_BUSY);
            result = USB_HOST_SCSI_HANDLE_INVALID;
        }
    }

    return(result);
}

// ******************************************************************************
/* Function:
    USB_HOST_SCSI_HANDLE USB_HOST_SCSI_Close 
    (
        USB_HOST_SCSI_HANDLE scsiHandle
    );

  Summary:
    This function will close the specified instance of the SCSI driver.

  Description:
    This function will close the specified instance of the SCSI driver. The
    handle in scsiHandle will not be valid any more. The driver must be opened
    once again.

  Remarks:
    This function is typically called by the file system media manager.
*/

void USB_HOST_SCSI_Close 
(
    USB_HOST_SCSI_HANDLE scsiHandle
)
{
    /* This function is not implemented in this release of the driver */
}

// ******************************************************************************
/* Function:
    bool USB_HOST_SCSI_MediaStatusGet 
    (
        USB_HOST_SCSI_HANDLE scsiHandle
    );

  Summary:
    This function will return true if the SCSI media is attached and ready to
    use.

  Description:
    This function will return true if the SCSI media is attached and ready to
    use.

  Remarks:
    This function is typically called by the file system media manager.
*/

bool USB_HOST_SCSI_MediaStatusGet 
(
    USB_HOST_SCSI_HANDLE scsiHandle
)
{
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    bool result = false;

    /* Validate the SCSI handle */
    if((USB_HOST_SCSI_HANDLE_INVALID != scsiHandle) &&
            (0 != scsiHandle))
    {
        /* The handle is actually a pointer to the SCSI object */
        scsiObj = (USB_HOST_SCSI_INSTANCE_OBJ *)scsiHandle;

        if((scsiObj->inUse) && (scsiObj->state == USB_HOST_SCSI_STATE_READY) && (scsiObj->isMediaReady))
        {
            /* Device is attached and ready to use */
            result = true;
        }
    }

    return(result);
}

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_SectorRead 
    (
        USB_HOST_SCSI_HANDLE scsiHandle,
        SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE * commandHandle,
        void * buffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Performs a block read operation.

  Description:
    This function will perform a block read operation. The operation will not
    complete when the function returns. Instead a handle to the operation will
    be returned in commandHandle and driver will generate an event when the
    operation has completed. The command handle will be returned along with the
    event. The data will be stored in buffer. Data will be read from the
    blockStart block and nBlock buffers will be read.

  Remarks:
    This function is typically called by the file system media manager.
*/

void USB_HOST_SCSI_SectorRead 
(
    USB_HOST_SCSI_HANDLE scsiHandle,
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE * commandHandle,
    void * buffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
     /* Perform a read transfer */
    _USB_HOST_SCSI_Transfer(scsiHandle, commandHandle, blockStart, nBlock, buffer, USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST);
}

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_SectorWrite 
    (
        USB_HOST_SCSI_HANDLE scsiHandle,
        SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE * commandHandle,
        void * buffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Performs a block write operation.

  Description:
    This function will perform a block write operation. The operation will not
    complete when the function returns. Instead a handle to the operation will
    be returned in commandHandle and driver will generate an event when the
    operation has completed. The command handle will be returned along with the
    event. The data to be written is specified by buffer. Data will be written
    to the blockStart block and nBlock buffers will be written.

  Remarks:
    This function is typically called by the file system media manager.
*/

void USB_HOST_SCSI_SectorWrite 
(
    USB_HOST_SCSI_HANDLE scsiHandle,
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE * commandHandle,
    void * buffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    /* Perform a write transfer */
    _USB_HOST_SCSI_Transfer(scsiHandle, commandHandle, blockStart, nBlock, buffer, USB_HOST_MSD_TRANSFER_DIRECTION_HOST_TO_DEVICE);
}

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_EventHandlerSet 
    (
        USB_HOST_SCSI_HANDLE scsiHandle,
        const void * eventHandler,
        const uintptr_t context
    );

  Summary:
    Registers an event handler with media driver.

  Description:
    This function will register an event handler with the SCSI driver. When read
    or a write command has completed, the SCSI driver will call this function
    with the relevant event. The context parameter will returned with this
    event.

  Remarks:
    This function is typically called by the file system media manager.
*/

void USB_HOST_SCSI_EventHandlerSet 
(
    USB_HOST_SCSI_HANDLE scsiHandle,
    const void * eventHandler,
    const uintptr_t context
)
{
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;

    if((scsiHandle != USB_HOST_SCSI_HANDLE_INVALID) && (scsiHandle != 0))
    {
        /* Set the event handler and the context */
        scsiObj = (USB_HOST_SCSI_INSTANCE_OBJ *)(scsiHandle);
        if(scsiObj->inUse)
        {
            /* Only if the object is valid */
            scsiObj->eventHandler = eventHandler;
            scsiObj->context = context;
        }
    }
}

// ******************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * USB_HOST_SCSI_MediaGeometryGet 
    (
        USB_HOST_SCSI_HANDLE scsiHandle
    );

  Summary:
    Return the media geometry of this media.

  Description:
    This function will return the media geometry of this media. 

  Remarks:
    This function is typically called by the file system media manager.
*/

SYS_FS_MEDIA_GEOMETRY * USB_HOST_SCSI_MediaGeometryGet 
(
    USB_HOST_SCSI_HANDLE scsiHandle
)
{
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    SYS_FS_MEDIA_GEOMETRY * result;

    result = NULL;
    if((scsiHandle != USB_HOST_SCSI_HANDLE_INVALID) && (scsiHandle != 0))
    {
        /* Set the event handler and the context */
        scsiObj = (USB_HOST_SCSI_INSTANCE_OBJ *)(scsiHandle);
        if((scsiObj->inUse) && (scsiObj->state == USB_HOST_SCSI_STATE_READY))
        {
            /* Only if the object is valid */
            result = &scsiObj->mediaGeometry;
        }
    }
    
    return(result);
}

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_DetachDetectTasks(int scsiObjIndex)

  Summary:
    This function is called periodically by the USB_HOST_SCSI_Tasks() function
    to check if the media is still attached.

  Description:
    This function is called periodically by the USB_HOST_SCSI_Tasks() function
    to check if the media is still attached. It uses the Test Unit ready command
    to check if the media responding. If the media is not responding the
    function moves the main state machine to initial state.

  Remarks:
    None.
*/

void _USB_HOST_SCSI_DetachDetectTasks(int scsiObjIndex)
{
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    USB_HOST_MSD_RESULT result;

    scsiObj = &gUSBHostSCSIObj[scsiObjIndex];

    switch(scsiObj->detachTaskState)
    {
        case USB_HOST_SCSI_DETACH_TASK_STATE_IDLE:

            /* Nothing to be done in this state */
            break;

        case USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_SEND:

            /* In this state, the driver prepares and send the Test Unit
             * ready command */
            _USB_HOST_SCSI_TestUnitReadyCommand(scsiObj->taskCommandObj.cdb);

            /* Send the command. The commandCompleted flag will be update in
             * the event handler  */
            scsiObj->taskCommandObj.inUse = true;
            scsiObj->taskCommandObj.commandCompleted = false;

            result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                    scsiObj->taskCommandObj.cdb, 6, NULL, 0, 
                    USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST,
                    _USB_HOST_SCSI_CommandCallback, (uintptr_t)(&scsiObj->taskCommandObj));

            if(result == USB_HOST_MSD_RESULT_SUCCESS)
            {
                /* The request was scheduled successfully. Wait for the
                 * request to complete */

                scsiObj->detachTaskState = USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_WAIT;
                _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_IDLE_TEST_UNIT_READY);
            }
            else if(result == USB_HOST_MSD_RESULT_BUSY)
            {
                /* If MSD is busy then we should really retry this after some time
                 * */
                scsiObj->detachTaskState = USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_DELAY;
            }


            break;

        case USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_WAIT:

            /* Here we are waiting for the test unit command to complete */

            if(scsiObj->taskCommandObj.commandCompleted)
            {
                /* The status of the test unit ready command can be read
                 * from the CSW */

                if(scsiObj->taskCommandObj.result == USB_HOST_MSD_RESULT_COMMAND_PASSED)
                {
                    /* Indicate that media is ready and reset the
                     * detachTimeOut counter */
                    scsiObj->isMediaReady = true;
                    scsiObj->detachTimeOut = 0;
                    scsiObj->detachTaskState = USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_DELAY;
                }
                else
                {
                    /* The test unit ready command failed. Increment the detach count */
                    scsiObj->isMediaReady = false;
                    scsiObj->detachTimeOut += USB_HOST_SCSI_DETACH_TEST_UNIT_READY_INTERVAL;
                    if(scsiObj->detachTimeOut >= USB_HOST_SCSI_DETACH_TIME_OUT)
                    {
                        /* This means that the media has most probably
                         * detached. Deregister the media from the file
                         * system and then try re-start the media bring up
                         * process. */
                        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "\r\nUSB Host SCSI: SCSI Instance %d Media was detached", scsiObjIndex);
                        
                        if(scsiObj->eventHandler != NULL)
                        {
                            /* Let the client know that device has been detached. */
                            scsiObj->eventHandler(USB_HOST_SCSI_EVENT_DETACH, USB_HOST_SCSI_COMMAND_HANDLE_INVALID, scsiObj->context);
                        }
                        
                        _USB_HOST_SCSI_FILE_SYSTEM_DEREGISTER(scsiObj->fsHandle);
                        scsiObj->fsHandle = SYS_FS_MEDIA_HANDLE_INVALID;
                        scsiObj->state = USB_HOST_SCSI_STATE_INQUIRY_RESPONSE;
                        scsiObj->detachTaskState = USB_HOST_SCSI_DETACH_TASK_STATE_IDLE;
                        _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_DETACH_TIME_OUT);
                    }
                    else
                    {
                        /* We need to keep sending the test unit ready.
                         * Start the delay and then send the command again
                         * */
                        scsiObj->detachTaskState = USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_DELAY;
                    }
                }
            }
            else
            {
                /* Continue to run the transfer error tasks while waiting
                 * for the transfer to complete */
                USB_HOST_MSD_TransferErrorTasks(scsiObj->lunHandle);
            }

            break;

        case USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_DELAY:
            /* In this state we start a delay before launching the Test Unit
             * Ready command */
            scsiObj->timerExpired = false;
            scsiObj->commandDelayHandle = SYS_TMR_CallbackSingle( USB_HOST_SCSI_DETACH_TEST_UNIT_READY_INTERVAL, (uintptr_t )scsiObj, _USB_HOST_SCSI_TimerCallback );
            if (scsiObj->commandDelayHandle != SYS_TMR_HANDLE_INVALID)
            {
                /* The delay has started. Now wait for the delay to complete
                 * */

                scsiObj->detachTaskState = USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_DELAY_WAIT;
            }
            else
            {
                /* Continue to stay in this state till the delay could be
                 * scheduled */
            }
            break;

        case USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_DELAY_WAIT:
            /* Check if the delay has completed */

            if(scsiObj->timerExpired )
            {
                /* Delay has completed. Send the test unit ready command */
                scsiObj->detachTaskState = USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_SEND;
                scsiObj->commandDelayHandle = SYS_TMR_HANDLE_INVALID ;
               
            }
            else
            {
                /* Stay in this state. Continue to check if the delay has
                 * completed */
            }
            break;

        default:
            break;
    }
}

USB_HOST_MSD_LUN_HANDLE USB_HOST_SCSI_MSDLUNHandleGet(USB_HOST_SCSI_OBJ scsiObj)
{
    /* This function return the MSD LUN Handle associated with this SCSI object */
    USB_HOST_MSD_LUN_HANDLE result = USB_HOST_MSD_LUN_HANDLE_INVALID;
    int scsiObjIndex = (int)scsiObj;
    
    if(gUSBHostSCSIObj[scsiObjIndex].inUse)
    {
        result = gUSBHostSCSIObj[scsiObjIndex].lunHandle;
    }
            
    return(result);
    
}

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_TransferTasks (USB_HOST_MSD_LUN_HANDLE lunHandle)

  Summary:
    This function is should be called periodically by the client that has opened
    the logical unit.

  Description:
    This function is should be called periodically by the client that has opened
    the logical unit. It should be called when the client is waiting for read or
    This function maintains the read/write command state machine.  write command
    to complete.

  Remarks:
    None.
*/

void USB_HOST_SCSI_TransferTasks(USB_HOST_MSD_LUN_HANDLE lunHandle)
{
    int scsiObjIndex;
    USB_HOST_SCSI_INSTANCE_OBJ * scsiObj;
    USB_HOST_MSD_RESULT result;
    SCSI_SENSE_DATA * requestSenseResponse;
    USB_HOST_SCSI_COMMAND_OBJ * commandObj;
    bool doEventCallback = false;
    int iterator;

    /* Run the MSD transfer error tasks. This will update the BOT transfer error
     * state machine */
    USB_HOST_MSD_TransferErrorTasks(lunHandle);    

    /* Run the SCSI tasks for all the LUNs. This is only needed when the
     * application is running in Bare-metal mode. */
    
    for(iterator = 0; iterator < USB_HOST_MSD_LUN_NUMBERS; iterator++)
    {
        if(gUSBHostSCSIObj[iterator].inUse)
        {
            /* Run the task routine for all objects that are valid */
            _USB_HOST_SCSI_TasksByIndex(iterator);
        }
    }

    /* Find the SCSI object that own this LUN */
    scsiObjIndex = _USB_HOST_SCSI_LUNHandleToSCSIInstance(lunHandle);

    if(scsiObjIndex >= 0)
    {
        scsiObj = &gUSBHostSCSIObj[scsiObjIndex];
        commandObj = &scsiObj->commandObj;

        switch(scsiObj->transferTaskState)
        {
            case USB_HOST_SCSI_TRANSFER_STATE_IDLE:

                /* In this state we don't have anything to do. */
                break;

            case USB_HOST_SCSI_TRANSFER_STATE_REQUEST_SENSE:

                /* We get into this state when a block command (READ10 or WRITE
                 * 10) has failed. We have to find out why. Note that we are not
                 * using the commandObj command object for these commands.
                 * commandObj will still contain the original command requested
                 * by the client. This will be needed just in case we need to
                 * retry the command. */

                _USB_HOST_SCSI_RequestSenseCommand(scsiObj->transferErrorCommandObj.cdb);

                /* The commandCompleted flag will be updated in the callback.
                 * Update the state and send the command.   */
                scsiObj->transferErrorCommandObj.inUse = true;
                scsiObj->transferErrorCommandObj.commandCompleted = false;

                /* Send the command */
                result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                        scsiObj->transferErrorCommandObj.cdb, 0x6, scsiObj->buffer, 18 , 
                        USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST,
                        _USB_HOST_SCSI_CommandCallback, (uintptr_t)(&scsiObj->transferErrorCommandObj));

                if(result == USB_HOST_MSD_RESULT_SUCCESS)
                {
                    /* Go to the next state only if the request was placed
                     * successfully. */
                    scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_WAIT_REQUEST_SENSE;
                }
                else if(result == USB_HOST_MSD_RESULT_FAILURE  ||
                        result == USB_HOST_MSD_RESULT_LUN_HANDLE_INVALID)
                {
                    /* These errors return due to a bad handle, invalid parameter, and so on;
                       things that no amount of spinning and retrying will fix.  In this case
                       just set the callback and get out. */
                    doEventCallback = true;
                }


                break;

            case USB_HOST_SCSI_TRANSFER_STATE_WAIT_REQUEST_SENSE:

                /* Here we check if the request sense command has completed */

                if(scsiObj->transferErrorCommandObj.commandCompleted)
                {
                    if(scsiObj->transferErrorCommandObj.result == USB_HOST_MSD_RESULT_COMMAND_PASSED)
                    {
                        /* Check the sense data */
                        requestSenseResponse = (SCSI_SENSE_DATA *)(scsiObj->buffer);

                        if(requestSenseResponse->SenseKey == SCSI_SENSE_NOT_READY)
                        {
                            /* This means the unit is not ready to serve the
                             * command. We have to send the Test Unit ready
                             * command to check when the unit becomes ready */
 
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Block command failed on Unit not ready.", scsiObjIndex);
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Testing if unit is ready.", scsiObjIndex);
                            scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_TEST_UNIT_READY;
                            _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_READ_WRITE_TEST_UNIT_READY);
                        }
                        else if((requestSenseResponse->SenseKey == SCSI_SENSE_UNIT_ATTENTION) &&
                                (requestSenseResponse->ASC == SCSI_ASC_MEDIUM_NOT_PRESENT))
                        {
                            /* This could probably mean that the medium was
                             * detached. We set the isMediaReady flag to false
                             * and then let the main SCSI state machine continue
                             * to check if the media is ready. We should also
                             * let the client know that block command has
                             * failed. */

                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Block command failed on Media not present.", scsiObjIndex);
                            _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_READ_WRITE_MEDIUM_NOT_PRESENT);
                            scsiObj->isMediaReady = false;
                            doEventCallback = true;
                        }
                        else if(requestSenseResponse->SenseKey == SCSI_SENSE_MEDIUM_ERROR)
                        {
                            /* This means there was an error on the medium */
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Block command failed on Media Error.", scsiObjIndex);
                            _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_READ_WRITE_MEDIUM_ERROR);
                            scsiObj->isMediaError = true;
                            doEventCallback = true;
                        }
                        else if(requestSenseResponse->ASCQ == SCSI_ASCQ_WRITE_PROTECTED)
                        {
                            /* This means the medium is Write protected */
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Block command failed on Media Write Protect Error.", scsiObjIndex);
                            doEventCallback = true;
                            _USB_HOST_SCSI_ERROR_CALLBACK((uintptr_t)scsiObjIndex, USB_HOST_SCSI_ERROR_CODE_MEDIA_WRITE_PROTECTED);
                        }
                    }

                }

                break;

            case USB_HOST_SCSI_TRANSFER_STATE_TEST_UNIT_READY:

                /* We get into this state if a SCSI block command failed and the
                 * request sense returned a sense code an media not ready. In
                 * this state we will periodically send the Test Unit ready
                 * command. */

                _USB_HOST_SCSI_TestUnitReadyCommand(scsiObj->transferErrorCommandObj.cdb);

                /* Send the command. The commandCompleted flag will be update in
                 * the event handler  */
                scsiObj->transferErrorCommandObj.inUse = true;
                scsiObj->transferErrorCommandObj.commandCompleted = false;

                result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                        scsiObj->transferErrorCommandObj.cdb, 6, NULL, 0, 
                        USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST,
                        _USB_HOST_SCSI_CommandCallback, (uintptr_t)(&scsiObj->transferErrorCommandObj));

                if(result == USB_HOST_MSD_RESULT_SUCCESS)
                {
                    /* The request was scheduled successfully. Wait for the
                     * request to complete */

                    scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_WAIT_TEST_UNIT_READY;
                }
                
                else if(result == USB_HOST_MSD_RESULT_FAILURE  ||
                        result == USB_HOST_MSD_RESULT_LUN_HANDLE_INVALID)
                {
                    /* The USB_HOST_MSD_Transfer may fail here if the device was
                     * disconnected or if some fatal error has occurred on the
                     * bus. In such a case, we must stop the transfer. */

                    doEventCallback = true;
                }


                break;

            case USB_HOST_SCSI_TRANSFER_STATE_WAIT_TEST_UNIT_READY:

                /* Here we are waiting for the test unit command to complete */

                if(scsiObj->transferErrorCommandObj.commandCompleted)
                {
                    /* The status of the test unit ready command can be read
                     * from the CSW */

                    if(scsiObj->transferErrorCommandObj.result == USB_HOST_MSD_RESULT_COMMAND_PASSED)
                    {
                        /* This means the unit is ready. The block command
                         * should be re-tried. */
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Test Unit is ready. Retrying block command.", scsiObjIndex);
                        scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_BLOCK_COMMAND_RETRY;
                    }
                    else
                    {
                        /* The test unit ready failed.  Before trying, we check
                         * how many time we have tried this already. If this has
                         * exceeded the retry count, then we fail the transfer.
                         * */

                        if(scsiObj->nCommandFailureTestUnitReadyAttempts >= USB_HOST_SCSI_COMMAND_FAILURE_TEST_UNIT_READY_NUMBER)
                        {
                            /* This means we have exceeded the re-try the count
                             * */
                            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nUSB Host SCSI: SCSI Instance %d Test Unit retry count exceeded. Failing the block command.", scsiObjIndex);
                            doEventCallback = true;
                        }
                        else
                        {
                            /* Increment the delay and try again */
                            scsiObj->nCommandFailureTestUnitReadyAttempts ++;
                            scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_TEST_UNIT_READY;
                        }
                    }
                }

                break;

            case USB_HOST_SCSI_TRANSFER_STATE_BLOCK_COMMAND_RETRY:

                /* In this state, we retry the block command that caused the
                 * failure in the first place. The block command parameter are a
                 * part of the command object */

                result = USB_HOST_MSD_Transfer(scsiObj->lunHandle, 
                        scsiObj->commandObj.cdb, 0x0A, scsiObj->commandObj.buffer , (scsiObj->commandObj.nSectors << 9) /* The left shift multiplies by 512 */, 
                        scsiObj->commandObj.direction, _USB_HOST_SCSI_BlockTransferCallback, (uintptr_t)(scsiObj));

                /* Update the transfer handle */
                if(result == USB_HOST_MSD_RESULT_SUCCESS)
                {
                    /* This means we can return the transfer task state to idle
                     * */
                    scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_IDLE; 
                }
                else if (result == USB_HOST_MSD_RESULT_BUSY)
                {
                    /* Continue to try sending this command */
                }
                else if(result == USB_HOST_MSD_RESULT_FAILURE)
                {
                    /* An unknown failure has occurred. We should terminate the
                     * transfer. We should let the client know that transfer
                     * has failed. The transfer error tasks routine must return
                     * back to not doing anything. */

                    doEventCallback = true;

                }

                break;

            default:
                break;
        }

        if(doEventCallback)
        {
            /* We should let the client know that transfer has failed.
             * The transfer error tasks routine must return back to not
             * doing anything. */
            if(scsiObj->eventHandler != NULL)
            {
                /* Generate the event */
                (scsiObj->eventHandler)(USB_HOST_SCSI_EVENT_COMMAND_ERROR, 
                                        (USB_HOST_SCSI_COMMAND_HANDLE)(commandObj), 
                                        scsiObj->context);
            }
            
            /* Release the command object and reset the transfer state machine since 
               the command failed. */
            scsiObj->commandObj.inUse = false;
            scsiObj->transferTaskState = USB_HOST_SCSI_TRANSFER_STATE_IDLE;
        }    
    }
}

// ****************************************************************************
/* Function:
    USB_HOST_SCSI_RESULT USB_HOST_SCSI_AttachEventHandlerSet
    (
        USB_HOST_SCSI_ATTACH_EVENT_HANDLER eventHandler,
        uintptr_t context
    );
           
  Summary:
    This function will set an attach event handler.

  Description:
    This function will set an attach event handler. The attach event handler
    will be called when a Mass Storage Device that supports a SCSI command set
    is attached. The context will be returned in the event handler. 
    This function should be called before the bus has been enabled.

  Remarks:
    Refer to usb_host_scsi.h for usage information.
*/

USB_HOST_SCSI_RESULT USB_HOST_SCSI_AttachEventHandlerSet
(
    USB_HOST_SCSI_ATTACH_EVENT_HANDLER eventHandler,
    uintptr_t context
)
{
    int iterator;

    USB_HOST_SCSI_RESULT result = USB_HOST_SCSI_RESULT_FAILURE;
    USB_HOST_SCSI_ATTACH_LISTENER_OBJ * attachListener;
    
    if(eventHandler == NULL)
    {
        result = USB_HOST_SCSI_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Search for free listener object */
        for(iterator = 0; iterator < USB_HOST_SCSI_ATTACH_LISTENERS_NUMBER; iterator ++)
        {
            if(!gUSBHostSCSIAttachListener[iterator].inUse)
            {
                /* Found a free object */
                attachListener = &gUSBHostSCSIAttachListener[iterator];
                attachListener->inUse = true;
                attachListener->eventHandler = eventHandler;
                attachListener->context = context;
                result = USB_HOST_SCSI_RESULT_SUCCESS;
                break;
            }
        }
    }

    return(result);
}

