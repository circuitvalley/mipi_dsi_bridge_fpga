/******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_msd.c

  Summary:
    USB Device Mass Storage Class Function Driver Implementation.

  Description:
    This file contains implementations of both private and public functions of
    USB Device MSD function driver. It should be included in any project that
    requires USB Device MSD functionality.
********************************************************************************/

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system/common/sys_module.h"
#include "driver/driver_common.h"
#include "usb/usb_device_msd.h"
#include "usb/src/usb_device_msd_local.h"

/*************************************
 * USB device MSD instance objects.
 *************************************/
static USB_DEVICE_MSD_INSTANCE gUSBDeviceMSDInstance [USB_DEVICE_INSTANCES_NUMBER];

static SCSI_SENSE_DATA gUSBDeviceMSDSenseData[USB_DEVICE_MSD_LUNS_NUMBER] MSD_COHERENT_ATTRIBUTE __attribute__((aligned(16)));

/***************************************
 * USB device MSD init objects.
 ***************************************/

USB_DEVICE_MSD_INIT gUSBDeviceMSDInit[USB_DEVICE_MSD_INSTANCES_NUMBER];

/****************************************
 * MSD Device function driver structure
 ****************************************/
  
USB_DEVICE_FUNCTION_DRIVER msdFunctionDriver = 
{
    /* MSD init function */
    .initializeByDescriptor = _USB_DEVICE_MSD_InitializeByDescriptorType ,

    /* MSD de-init function */
    .deInitialize = _USB_DEVICE_MSD_Deinitialization , 
      
    /* MSD set-up packet handler */
    .controlTransferNotification = _USB_DEVICE_MSD_ControlTransferHandler ,

    /* MSD tasks function */
    .tasks = _USB_DEVICE_MSD_Tasks
};

// ******************************************************************************
/* Function:
    void _USB_DEVICE_MSD_InitializeEndpoint
    (
        USB_DEVICE_MSD_INSTANCE * msdInstance,
        DRV_HANDLE usbDevHandle,
        USB_ENDPOINT_DESCRIPTOR * epDescriptor
    )

  Summary:
    USB Device MSD endpoint initialization.

  Description:
    Initialize endpoints and endpoint related variables here.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_DEVICE_MSD_InitializeEndpoint
(
    USB_DEVICE_MSD_INSTANCE * msdInstance,
    DRV_HANDLE usbDevHandle,
    USB_ENDPOINT_DESCRIPTOR * epDescriptor
)
{
    /* This function is called from _USB_DEVICE_MSD_InitializeByDescriptorType
     * when the descriptor type is an endpoint. The bulk endpoints associated 
     * with this interface are enabled in this function */

    if( epDescriptor->transferType == USB_TRANSFER_TYPE_BULK )
    {
        if(epDescriptor->dirn == USB_DATA_DIRECTION_DEVICE_TO_HOST)
        {
            /* Save the TX endpoint information */
            msdInstance->bulkEndpointTx = epDescriptor->bEndpointAddress;
            msdInstance->bulkEndpointTxSize =  epDescriptor->wMaxPacketSize;
            
            /* Enable the TX endpoint */
            USB_DEVICE_EndpointEnable(usbDevHandle, 0, msdInstance->bulkEndpointTx, epDescriptor->transferType, epDescriptor->wMaxPacketSize);
        }
        else
        {
            /* Enable the receive endpoint */
            msdInstance->bulkEndpointRx = epDescriptor->bEndpointAddress;
            msdInstance->bulkEndpointRxSize = epDescriptor->wMaxPacketSize;

            /* Enable the endpoint */
            USB_DEVICE_EndpointEnable(usbDevHandle, 0, msdInstance->bulkEndpointRx, epDescriptor->transferType,epDescriptor->wMaxPacketSize);

            /* Now since device layer has already opened the bulk endpoint we can submit an
             * IRP to receive the CBW. */
            msdInstance->msdMainState = USB_DEVICE_MSD_STATE_WAIT_FOR_CBW;
        }
    }
    else
    {
        SYS_ASSERT( false, "USB DEVICE MSD: MSD does not support anything other than Bulk endpoints. Please check the descriptors.");
    }    
}

// *****************************************************************************
/* Function:
    void _USB_DEVICE_MSD_InitializeInterface
    (
        USB_DEVICE_MSD_OBJ * msdDeviceObj,
        DRV_HANDLE usbDeviceHandle,
        void * funcDriverInit,
        USB_INTERFACE_DESCRIPTOR * intfDesc
    )

  Summary:
    USB Device MSD Interface Initialization.

  Description:
    Initialize only the interface related variables here.

  Remarks:
    This is a local function and should not be called directly by the
    applicaiton.
*/

void _USB_DEVICE_MSD_InitializeInterface
(
    USB_DEVICE_MSD_INSTANCE * msdDeviceObj,
    DRV_HANDLE usbDeviceHandle,
    void * funcDriverInit,
    USB_INTERFACE_DESCRIPTOR * intfDesc
)
{
    /* This function is called from _USB_DEVICE_MSD_InitializeByDescriptorType
     * when the descriptor type is interface */

    int count;
    USB_DEVICE_MSD_INIT * msdInitializationData;

    /* Access the MSD Function Driver Initialization data */
    msdInitializationData = (USB_DEVICE_MSD_INIT *)funcDriverInit;

    /* Update the MSD function driver instance with the 
     * initialization data. The media data is a pointer
     * to table containing data about each media
     * that this MSD instance is managing. */
    msdDeviceObj->numberOfLogicalUnits = msdInitializationData->numberOfLogicalUnits;
    msdDeviceObj->mediaData = msdInitializationData->mediaInit;
    msdDeviceObj->msdCSW = msdInitializationData->msdCSW;
    msdDeviceObj->msdCBW = msdInitializationData->msdCBW;

    /* Initialize all media handles to an initial value i.e invalid handles. */
    for(count = 0; count < msdDeviceObj->numberOfLogicalUnits; count++)
    {
        msdDeviceObj->mediaDynamicData[count].mediaHandle = DRV_HANDLE_INVALID;
        /* Initialize the Sense data pointer */
        msdDeviceObj->mediaDynamicData[count].senseData = &gUSBDeviceMSDSenseData[count];
        _USB_DEVICE_MSD_ResetSenseData (msdDeviceObj->mediaDynamicData[count].senseData);

        SYS_ASSERT(msdDeviceObj->mediaData->mediaFunctions[count].open != NULL, "This function pointer cannot be NULL");
        SYS_ASSERT(msdDeviceObj->mediaData->mediaFunctions[count].close != NULL, "This function pointer cannot be NULL");
        SYS_ASSERT(msdDeviceObj->mediaData->mediaFunctions[count].isAttached != NULL, "This function pointer cannot be NULL");
        SYS_ASSERT(msdDeviceObj->mediaData->mediaFunctions[count].isWriteProtected != NULL, "This function pointer cannot be NULL");
        SYS_ASSERT(msdDeviceObj->mediaData->mediaFunctions[count].geometryGet != NULL, "This function pointer cannot be NULL");
        SYS_ASSERT(msdDeviceObj->mediaData->mediaFunctions[count].blockEventHandlerSet != NULL, "This function pointer cannot be NULL");
        SYS_ASSERT(msdDeviceObj->mediaData->mediaFunctions[count].blockRead != NULL, "This function pointer cannot be NULL");
    }

    /* Remember the USB Device Layer handle */
    msdDeviceObj->hUsbDevHandle = usbDeviceHandle;

    /* Initialize some of the IRP members to an intial value. Note that the
     * user data of each IRP is set to point to the owner MSD Device Instance */

    msdDeviceObj->irpRx.userData = (uintptr_t)msdDeviceObj;
    msdDeviceObj->irpTx.userData = (uintptr_t)msdDeviceObj;
    msdDeviceObj->irpRx.status = USB_DEVICE_IRP_STATUS_COMPLETED;
    msdDeviceObj->irpTx.status = USB_DEVICE_IRP_STATUS_COMPLETED;
    msdDeviceObj->irpTx.callback = &_USB_DEVICE_MSD_CallBackBulkTxTransfer;
    msdDeviceObj->irpRx.callback = &_USB_DEVICE_MSD_CallBackBulkRxTransfer;

    /* The Host may set an alternate inteface on this instance. Intialize the
     * alternate setting to zero */
    msdDeviceObj->alternateSetting = 0;
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_MSD_InitializeByDescriptorType
    (
        SYS_MODULE_INDEX iMSD, 
        DRV_HANDLE usbDeviceHandle,
        void* funcDriverInit, 
        uint8_t intfNumber, 
        uint8_t altSetting,
        uint8_t descriptorType, 
        uint8_t * pDescriptor
    )

  Summary:
    USB Device MSD Initialization. The device layer calls this function to
    initialize the MSD based on the descriptor found (from the configurations
    descriptor).  

  Description:
    This function initializes MSD function driver based on the descriptor.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_DEVICE_MSD_InitializeByDescriptorType
(
    SYS_MODULE_INDEX iMSD, 
    DRV_HANDLE usbDeviceHandle,
    void* funcDriverInit, 
    uint8_t intfNumber, 
    uint8_t altSetting,
    uint8_t descriptorType, 
    uint8_t * pDescriptor
)
{
    /* This function is called by the Device Layer when it comes across an MSD
     * interface descriptor or an endpoint belonging to a MSD interface */

    SYS_ASSERT(altSetting == 0, "USB Device MSD: MSD supports only one setting and does not support alternate settings ");

    USB_DEVICE_MSD_INSTANCE * msdDeviceObj = &gUSBDeviceMSDInstance[iMSD];

    switch(descriptorType )
    {
        case USB_DESCRIPTOR_ENDPOINT:
            
            /* Device layer came across an MSD endpoint. Initialize the endpoint */
            _USB_DEVICE_MSD_InitializeEndpoint(msdDeviceObj, usbDeviceHandle,(USB_ENDPOINT_DESCRIPTOR *)pDescriptor);
            break;

        case USB_DESCRIPTOR_INTERFACE:

            /* Device Layer came across an MSD interface. Initialize the
             * interface */
            _USB_DEVICE_MSD_InitializeInterface(msdDeviceObj, usbDeviceHandle, funcDriverInit, (USB_INTERFACE_DESCRIPTOR *)pDescriptor);
            break;

        default:
            SYS_ASSERT( false, "USB DEVICE MSD: MSD doesnot support this descriptor type. Please check the descriptors");
            break;
    }
}

// ******************************************************************************
/* Function: 
   void _USB_DEVICE_MSD_ControlTransferHandler
    (
        SYS_MODULE_INDEX MSDIndex,
        USB_DEVICE_EVENT transferState,
        USB_SETUP_PACKET * setupPkt
    )
  
  Summary:
    MSD control transfer handler.

  Description:
    MSD control transfer handler. This is the callback the device layer calls
    when there is a setup packet that is targeted to this particular instance
    of MSD.

  Returns:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_DEVICE_MSD_ControlTransferHandler
(
    SYS_MODULE_INDEX MSDIndex,
    USB_DEVICE_EVENT controlTransferEvent,
    USB_SETUP_PACKET * setupPkt
)
{
    USB_DEVICE_MSD_INSTANCE * msdThisInstance = &gUSBDeviceMSDInstance[MSDIndex] ;

    if(controlTransferEvent == USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST)
    {
        if(( setupPkt->Recipient == 0x01) && (setupPkt->RequestType == 0x00))
        {
            /* This means the recipient of the control transfer is interface
             * and this is a standard request type */

            switch(setupPkt->bRequest)
            {
                case USB_REQUEST_SET_INTERFACE:

                    /* If the host does a Set Interface, we simply acknowledge
                     * it. We also remember the interface that was set */
                    msdThisInstance->alternateSetting = setupPkt->W_Value.byte.LB;
                    USB_DEVICE_ControlStatus( msdThisInstance->hUsbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);
                    break;

                case USB_REQUEST_GET_INTERFACE:

                    /* The host is requesting for the current interface setting
                     * number. Return the one that the host set */
                    USB_DEVICE_ControlSend( msdThisInstance->hUsbDevHandle, &msdThisInstance->alternateSetting, 1);
                    break;

                default:
                    break; 
            }
        }
        else if( setupPkt->bmRequestType & USB_MSD_REQUEST_CLASS_SPECIFIC )
        {
            /* We have got setup request */
            switch ( setupPkt->bRequest )
            {
                case USB_MSD_GET_MAX_LUN:

                    /* First make sure all request parameters are correct:
                     * MSD BOT specs require wValue to be == 0x0000, and wLengh == 1 */
                    if((setupPkt->wValue != 0) || (setupPkt->wLength != 1))
                    {
                        USB_DEVICE_ControlStatus( msdThisInstance->hUsbDevHandle,
                                USB_DEVICE_CONTROL_STATUS_ERROR );
                        return ;
                    }

                    /* Return the number of LUNs in this device less one. */

                    msdThisInstance->msdBuffer  = msdThisInstance->numberOfLogicalUnits - 1;
                    USB_DEVICE_ControlSend( msdThisInstance->hUsbDevHandle, &msdThisInstance->msdBuffer, 1 );

                    break;

                case USB_MSD_RESET:

                    /* First make sure all request parameters are correct:
                     * MSD BOT specs require wValue to be == 0x0000, and wLength == 0 */
                   if((setupPkt->wValue != 0) || (setupPkt->wLength != 0))
                    {
                        USB_DEVICE_ControlStatus( msdThisInstance->hUsbDevHandle,
                                USB_DEVICE_CONTROL_STATUS_ERROR );
                        return ;
                    }

                   /* Cancel all the IRPs on the BULK IN and OUT Endpoints. 
                      Stall both the endpoints.
                      */
                   USB_DEVICE_IRPCancelAll(msdThisInstance->hUsbDevHandle, msdThisInstance->bulkEndpointTx);
                   USB_DEVICE_IRPCancelAll(msdThisInstance->hUsbDevHandle, msdThisInstance->bulkEndpointRx);

                   USB_DEVICE_EndpointStall(msdThisInstance->hUsbDevHandle, msdThisInstance->bulkEndpointTx);
                   USB_DEVICE_EndpointStall(msdThisInstance->hUsbDevHandle, msdThisInstance->bulkEndpointRx);

                   msdThisInstance->msdMainState = USB_DEVICE_MSD_STATE_WAIT_FOR_CBW;
                   USB_DEVICE_ControlStatus( msdThisInstance->hUsbDevHandle, USB_DEVICE_CONTROL_STATUS_OK );
                   break;

                default:

                   /* Stall other requests. */
                   USB_DEVICE_ControlStatus( msdThisInstance->hUsbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR );
                   break;
            } 
        }
    }
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_MSD_CallBackBulkRxTransfer( void * handle )

  Summary:
    This is a callback function that gets called by controller driver,
    after the completion of bulk-out transfer.

  Description:
    This is a callback function that gets called by controller driver,
    after the completion of bulk-out transfer.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

void _USB_DEVICE_MSD_CallBackBulkRxTransfer( USB_DEVICE_IRP *  handle )
{
    /* Code to be add if required in a future release */
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_MSD_CallBackBulkTxTransfer( void *  handle )

  Summary:
    This is a callback function that gets called by controller driver,
    after the completion of bulk-in transfer.

  Description:
    This is a callback function that gets called by controller driver,
    after the completion of bulk-in transfer.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

void _USB_DEVICE_MSD_CallBackBulkTxTransfer( USB_DEVICE_IRP *  handle )
{
    /* Code to be add if required in a future release */
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_MSD_PostDataStageRoutine ( SYS_MODULE_INDEX iMSD )

  Summary:
    Do some post data stage routines (Before we initiate CSW).

  Description:
    Do some post data stage routines (Before we initiate CSW).
  
  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

bool _USB_DEVICE_MSD_PostDataStageRoutine
(
    SYS_MODULE_INDEX iMSD
)
{
    /* This function is called int function driver tasks routine when
     * the CSW is being prepared. */
    bool isStalled = false;
    uint32_t residueLength;

    USB_DEVICE_MSD_INSTANCE * msdInstance = &gUSBDeviceMSDInstance[iMSD];
    /* Build the CSW */
    msdInstance->msdCSW->dCSWSignature = USB_MSD_VALID_CSW_SIGNATURE;
    msdInstance->msdCSW->dCSWTag = msdInstance->msdCBW->dCBWTag;

    residueLength = (msdInstance->msdCBW->dCBWDataTransferLength - msdInstance->rxTxTotalDataByteCount);

    /* Update the Residue length in the CSW */
    msdInstance->msdCSW->dCSWDataResidue = residueLength;

    if ((msdInstance->msdCSW->bCSWStatus == USB_MSD_CSW_COMMAND_PASSED) &&
            residueLength)
    {
        /* The device has sent less data than the host had expected. The transfer
        was successful. But stall the IN EP. */
        USB_DEVICE_EndpointStall( msdInstance->hUsbDevHandle, msdInstance->bulkEndpointTx );
        isStalled = true;
    }

    if ((msdInstance->msdCSW->bCSWStatus == USB_MSD_CSW_COMMAND_FAILED) &&
            (msdInstance->msdCBW->dCBWDataTransferLength))
    {
        if (msdInstance->msdCBW->bmCBWFlags.value & USB_MSD_CBW_DIRECTION_BITMASK)
        {
            USB_DEVICE_EndpointStall(msdInstance->hUsbDevHandle, msdInstance->bulkEndpointTx);
            isStalled = true;
        }
        else
        {
            USB_DEVICE_EndpointStall(msdInstance->hUsbDevHandle, msdInstance->bulkEndpointRx);
        }
    }

    return isStalled;
}

void _USB_DEVICE_MSD_SendDataToUsb 
(
    SYS_MODULE_INDEX iMSD,
    uint8_t *data,
    uint16_t length
)
{
    USB_DEVICE_MSD_INSTANCE * msdInstance = &gUSBDeviceMSDInstance[iMSD];
    msdInstance->irpTx.data = (void *)data;
    msdInstance->irpTx.size = length;
    msdInstance->irpTx.flags = USB_DEVICE_IRP_FLAG_DATA_PENDING;

    USB_DEVICE_IRPSubmit(msdInstance->hUsbDevHandle, msdInstance->bulkEndpointTx, &msdInstance->irpTx);
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_MSD_Tasks ( SYS_MODULE_INDEX iMSD )

  Summary:
    This function handles the main MSD state machine.

  Description:
    This function handles the main MSD state machine. It process the CBW, data
    stage and the CSW. This function is called by the Device Layer tasks routine.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

void _USB_DEVICE_MSD_Tasks 
(
    SYS_MODULE_INDEX iMSD
)
{
    uint8_t commandStatus = USB_MSD_CSW_COMMAND_PASSED; 
    USB_DEVICE_MSD_INSTANCE * msdObj = &gUSBDeviceMSDInstance[iMSD];

    switch (msdObj->msdMainState)
    {
        case USB_DEVICE_MSD_STATE_STALL_IN_OUT:
            {
                /* Stall both BULK IN and OUT EPs. 
                   The msdMainState state change will be done once
                   the MSD BOT RESET Request is processed.
                 */
                USB_DEVICE_EndpointStall(msdObj->hUsbDevHandle, msdObj->bulkEndpointTx);
                USB_DEVICE_EndpointStall(msdObj->hUsbDevHandle, msdObj->bulkEndpointRx);
                break;
            }

        case USB_DEVICE_MSD_STATE_WAIT_FOR_CBW:
            {
                /* Queue an IRP to receive the CBW if the BULK OUT and IN EPs are not stalled. */
                if ((!USB_DEVICE_EndpointIsStalled(msdObj->hUsbDevHandle, msdObj->bulkEndpointRx)) &&
                    (!USB_DEVICE_EndpointIsStalled(msdObj->hUsbDevHandle, msdObj->bulkEndpointTx)))
                {
                    /* Get ready to receive a CBW */
                    msdObj->irpRx.data = (void *)msdObj->msdCBW;
                    msdObj->irpRx.size = msdObj->bulkEndpointRxSize;
                    msdObj->irpRx.flags = USB_DEVICE_IRP_FLAG_DATA_PENDING;

                    USB_DEVICE_IRPSubmit (msdObj->hUsbDevHandle, msdObj->bulkEndpointRx, &msdObj->irpRx);
                    msdObj->msdMainState = USB_DEVICE_MSD_STATE_CBW;
                }
                break;
            }

      
          case USB_DEVICE_MSD_STATE_CBW:
            {
                if (( (msdObj->irpRx.status == USB_DEVICE_IRP_STATUS_COMPLETED) || (msdObj->irpRx.status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
                        && (!USB_DEVICE_EndpointIsStalled(msdObj->hUsbDevHandle, msdObj->bulkEndpointRx)))
                {
                    /* Received the CBW from the HOST. Check whether the CBW is valid and meaningful. */
                    msdObj->msdMainState = _USB_DEVICE_MSD_VerifyCommand (iMSD, &commandStatus);

                    if (msdObj->msdMainState == USB_DEVICE_MSD_STATE_PROCESS_CBW)
                    {
                        msdObj->msdMainState = _USB_DEVICE_MSD_ProcessNonRWCommand(iMSD, &commandStatus);
                    }
                    else if (msdObj->msdMainState == USB_DEVICE_MSD_STATE_DATA_IN)
                    {
                        msdObj->msdMainState = _USB_DEVICE_MSD_ProcessRead(iMSD, &commandStatus);
                    }
                    else if (msdObj->msdMainState == USB_DEVICE_MSD_STATE_DATA_OUT)
                    {
                        msdObj->msdMainState = _USB_DEVICE_MSD_ProcessWrite(iMSD, &commandStatus);
                    }

                    msdObj->msdCSW->bCSWStatus = commandStatus;
                }
                else if (msdObj->irpRx.status < USB_DEVICE_IRP_STATUS_COMPLETED)
                {
                    /* This could happen if the IRP was aborted due to some
                     * reason. We need to ignore the IRP and re-schedule a new
                     * IRP. */

                    msdObj->msdMainState = USB_DEVICE_MSD_STATE_WAIT_FOR_CBW;
                }
                break;
            }


            case USB_DEVICE_MSD_STATE_DATA_IN:
            {
                if ((msdObj->irpTx.status <= USB_DEVICE_IRP_STATUS_COMPLETED_SHORT)
                        && (!USB_DEVICE_EndpointIsStalled(msdObj->hUsbDevHandle, msdObj->bulkEndpointTx)))
                {
                    /* This means we have to send or have to continue sending data.
                     * Check if we have any/more data to send. The return value of
                     * this function indicates what the next state should be. */
                    msdObj->msdMainState = _USB_DEVICE_MSD_ProcessRead(iMSD, &commandStatus);
                    msdObj->msdCSW->bCSWStatus = commandStatus;
                }
                break;
            }

            case USB_DEVICE_MSD_STATE_DATA_OUT:
            {
                if ((msdObj->irpRx.status <= USB_DEVICE_IRP_STATUS_COMPLETED_SHORT)
                        && (!USB_DEVICE_EndpointIsStalled(msdObj->hUsbDevHandle, msdObj->bulkEndpointRx)))
                {
                    /* Check if we have any/more data to receive. */
                    msdObj->msdMainState = _USB_DEVICE_MSD_ProcessWrite(iMSD, &commandStatus);
                    /* Update the CSW command status */
                    msdObj->msdCSW->bCSWStatus = commandStatus;
                }
                break;
            }

            case USB_DEVICE_MSD_STATE_CSW:
            {
                if (msdObj->irpTx.status <= USB_DEVICE_IRP_STATUS_COMPLETED_SHORT)
                {
                    _USB_DEVICE_MSD_PostDataStageRoutine(iMSD);
                    msdObj->msdMainState = USB_DEVICE_MSD_STATE_SEND_CSW;
                }
                else
                {
                    break;
                }
            }

            case USB_DEVICE_MSD_STATE_SEND_CSW:
            {
                if ((msdObj->irpTx.status <= USB_DEVICE_IRP_STATUS_COMPLETED_SHORT)
                        && (!USB_DEVICE_EndpointIsStalled(msdObj->hUsbDevHandle, msdObj->bulkEndpointTx)))
                {
                    /* Submit IRP to send CSW */
                    msdObj->irpTx.data = (void *)msdObj->msdCSW;
                    msdObj->irpTx.size = sizeof(USB_MSD_CSW);
                    msdObj->irpTx.flags = USB_DEVICE_IRP_FLAG_DATA_PENDING;
                    USB_DEVICE_IRPSubmit (msdObj->hUsbDevHandle, msdObj->bulkEndpointTx, &msdObj->irpTx);

                    msdObj->msdMainState = USB_DEVICE_MSD_STATE_WAIT_FOR_CBW;
                }
                break;
            }

            case USB_DEVICE_MSD_STATE_IDLE:
            default:
                break;
    }
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_MSD_BlockEventHandler
    (
        SYS_FS_MEDIA_BLOCK_EVENT event,
        SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )

  Summary:
    This is a callback function called by the media after the completion of
    media operation.

  Description:
    This is a callback function called by the media after the completion of
    media operation.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

void _USB_DEVICE_MSD_BlockEventHandler
(
    SYS_FS_MEDIA_BLOCK_EVENT event,
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE commandHandle,
    uintptr_t context
)
{
    USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA * mediaDynamicData = (USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA *)context;
    switch(event)
    {
        case SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE:
            mediaDynamicData->mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_COMPLETE;
            break;
        case SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR:
            mediaDynamicData->mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_ERROR;
            break;
    }
 }


// ******************************************************************************
/* Function:
    void _USB_DEVICE_MSD_CheckAndUpdateMediaState
    (
        SYS_MODULE_INDEX iMSD,
        uint8_t logicalUnit
    )

  Summary:
     This function must be called periodically in the MSD task to update the
     media state.

  Description:
     This function must be called periodically in the MSD task to update the
     media state.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

void _USB_DEVICE_MSD_CheckAndUpdateMediaState
(
    SYS_MODULE_INDEX iMSD,
    uint8_t logicalUnit
)
{
    /* This function is called by the _USB_DEVICE_MSD_VerifyCommand() 
     * function. The mediaPresent flag in this function is checked
     * by the _USB_DEVICE_MSD_ProcessNonRWCommand() function to decide
     * whether it should be continue processing the command */

    USB_DEVICE_MSD_MEDIA_FUNCTIONS * mediaFunctions;
    USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA * mediaDynamicData;
    SYS_MODULE_INDEX mediaInstanceIndex;
    DRV_HANDLE drvHandle;

    USB_DEVICE_MSD_INSTANCE * msdThisInstance = &gUSBDeviceMSDInstance[iMSD];

    /* Get data about this LUN. The dynamic information about this media */
    mediaDynamicData = &msdThisInstance->mediaDynamicData[logicalUnit];

    /* Pointer to the media functions */
    mediaFunctions = &msdThisInstance->mediaData[logicalUnit].mediaFunctions;

    /* Harmony module index for this media */
    mediaInstanceIndex = msdThisInstance->mediaData[logicalUnit].instanceIndex;

    /* Check if this media has been opened. If not can it be opened */        
    if( mediaDynamicData->mediaHandle == DRV_HANDLE_INVALID )
    {
        /* Try to open the media */
        drvHandle = mediaFunctions->open( mediaInstanceIndex, DRV_IO_INTENT_READWRITE| DRV_IO_INTENT_NONBLOCKING);

        /* If the driver could be opened, then we need to do a few things */
        if (drvHandle != DRV_HANDLE_INVALID)
        {
            /* Update the media dynamic data with the valid handle */
            mediaDynamicData->mediaHandle = drvHandle;

            /* Get the sector size */
            if(msdThisInstance->mediaData[logicalUnit].sectorSize != 0)
            {
                /* This means the sector size is specified in the media init table
                 * */
                mediaDynamicData->sectorSize = msdThisInstance->mediaData[logicalUnit].sectorSize;
            }

            /* Set the block start address if available */
            if((msdThisInstance->mediaData[logicalUnit].block0StartAddress != NULL)
                    && (mediaFunctions->blockStartAddressSet != NULL))
            {
                mediaFunctions->blockStartAddressSet(drvHandle, msdThisInstance->mediaData[logicalUnit].block0StartAddress);
            }

            /* We should set an event handler with the media driver */
            mediaFunctions->blockEventHandlerSet(drvHandle, _USB_DEVICE_MSD_BlockEventHandler, (uintptr_t)mediaDynamicData);
            
            /* Get a pointer to the media geomtery */
            mediaDynamicData->mediaGeometry = mediaFunctions->geometryGet(drvHandle);
            
        }
    }

    /* The driver is open. Update the media status */
    if (mediaDynamicData->mediaHandle != DRV_HANDLE_INVALID)
    {
        /* The media driver could be opended. Now check if media is present. */
        mediaDynamicData->mediaPresent = mediaFunctions->isAttached(mediaDynamicData->mediaHandle);
    }
    else
    {
        /* Media is not yet ready to be opened. So consider it as media not
         * being present.*/
        mediaDynamicData->mediaPresent = false;
    }
}

// ******************************************************************************
/* Function:
    USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_VerifyCommand
    (
        SYS_MODULE_INDEX iMSD,
        uint8_t *commandStatus
    )

  Summary:
    This function verifies the recieved CBW.

  Description:
    This function verifies the recieved CBW.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_VerifyCommand
(
    SYS_MODULE_INDEX iMSD,
    uint8_t *commandStatus
)
{
    uint8_t logicalUnit = 0;
    uint32_t length = 0;

    /* This function is called when the device has recieved a CBW. It updates
     * the media state and then lets the main state machine know what the next
     * state should be based on the CBW verfication. */
    USB_MSD_CBW * lCBW;
    USB_DEVICE_MSD_INSTANCE * msdInstance = &gUSBDeviceMSDInstance[iMSD];

    USB_DEVICE_MSD_MEDIA_FUNCTIONS * mediaFunctions;
    USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA * mediaDynamicData;

    *commandStatus = USB_MSD_CSW_COMMAND_PASSED; 
    /* Obtain the pointer to the CBW data structure */
    lCBW = (USB_MSD_CBW *)msdInstance->msdCBW;

    logicalUnit = msdInstance->msdCBW->bCBWLUN;
    /* Reset media state */
    msdInstance->mediaDynamicData[logicalUnit].mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_IDLE;

    /* Reset the counters. */
    msdInstance->rxTxTotalDataByteCount = 0;
    msdInstance->bufferOffset = 0;
    msdInstance->numPendingIrps = 0;
    msdInstance->numUsbSectors = 0;
    msdInstance->numSectorsToWrite = 0;

    /* Make sure we have received an integral CBW with the 
     * right size and signature */
    if ((msdInstance->irpRx.size != sizeof(USB_MSD_CBW))
            || (lCBW->dCBWSignature != USB_MSD_VALID_CBW_SIGNATURE))
    {
        /* Invalid CBW. Stall both the endpoints until the host performs a reset
            recovery.
            */
        return USB_DEVICE_MSD_STATE_STALL_IN_OUT;
    }

    /* Check and update the media state */
    _USB_DEVICE_MSD_CheckAndUpdateMediaState(iMSD, lCBW->bCBWLUN);

    /* See if last command was request sense. If so host has already read the
     * sense data. Now we should reset sense data. */
    if (msdInstance->mediaDynamicData[logicalUnit].resetSenseData)
    {
        msdInstance->mediaDynamicData[logicalUnit].resetSenseData = false;
        _USB_DEVICE_MSD_ResetSenseData(msdInstance-> mediaDynamicData[logicalUnit].senseData);
    }

    mediaDynamicData = &msdInstance->mediaDynamicData[logicalUnit];
    mediaFunctions = &msdInstance->mediaData[logicalUnit].mediaFunctions;

    /* Find the number of bytes to be transferred. */
    length = ((lCBW->CBWCB[7] << 8) | lCBW->CBWCB[8]);
    length <<= 9;
    
    /* Do one time error checking for the read/write commands. */
    if ((lCBW->CBWCB[0] == SCSI_READ_10) || (lCBW->CBWCB[0] == SCSI_WRITE_10))
    {
        if (mediaDynamicData->mediaPresent == false)
        {
            *commandStatus = USB_MSD_CSW_COMMAND_FAILED; 
            return USB_DEVICE_MSD_STATE_CSW;
        }

        /* Fail the command if there is a mismatch in the amount of data
        to be transferred for read and write commands. */
        if (lCBW->dCBWDataTransferLength != length)
        {
            *commandStatus = USB_MSD_CSW_COMMAND_FAILED; 
            return USB_DEVICE_MSD_STATE_CSW;
        }

        /* zero block transfer. */
        if (lCBW->dCBWDataTransferLength == 0)
        {
            *commandStatus = USB_MSD_CSW_COMMAND_PASSED; 
            return USB_DEVICE_MSD_STATE_CSW;
        }

        if (lCBW->CBWCB[0] == SCSI_READ_10)
        {
            if (!(lCBW->bmCBWFlags.value & USB_MSD_CBW_DIRECTION_BITMASK))
            {
                *commandStatus = USB_MSD_CSW_COMMAND_FAILED; 
                return USB_DEVICE_MSD_STATE_CSW;
            }
            return USB_DEVICE_MSD_STATE_DATA_IN;
        }
        else
        {
            if (lCBW->bmCBWFlags.value & USB_MSD_CBW_DIRECTION_BITMASK)
            {
                *commandStatus = USB_MSD_CSW_COMMAND_FAILED; 
                return USB_DEVICE_MSD_STATE_CSW;
            }

            if (mediaFunctions->isWriteProtected(mediaDynamicData->mediaHandle))
            {
                /* Media is write protected. Set sense keys so the host knows
                 * what caused the error. */
                mediaDynamicData->senseData->SenseKey = SCSI_SENSE_DATA_PROTECT;
                mediaDynamicData->senseData->ASC = SCSI_ASC_WRITE_PROTECTED;
                mediaDynamicData->senseData->ASCQ = SCSI_ASCQ_WRITE_PROTECTED;

                /* Indicate in CSW that command failed. */
                *commandStatus = USB_MSD_CSW_COMMAND_FAILED; 
                return USB_DEVICE_MSD_STATE_CSW;
            }

            return USB_DEVICE_MSD_STATE_DATA_OUT;
        }
    }

    return USB_DEVICE_MSD_STATE_PROCESS_CBW;
}

// ******************************************************************************
/* Function:
    void  _USB_DEVICE_MSD_GetBlockAddressAndLength
    (
        USB_MSD_CBW * lCBW,
        USB_DEVICE_MSD_DWORD_VAL * logicalBlockAddress,
        USB_DEVICE_MSD_DWORD_VAL * logicalBlockLength
    )

  Summary:
    This function retrieves the logical unit's block address and block length
    from CBW.

  Description:
    This function retrieves the logical unit's block address and block length
    from CBW.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

void  _USB_DEVICE_MSD_GetBlockAddressAndLength
(
    USB_MSD_CBW * lCBW,
    USB_DEVICE_MSD_DWORD_VAL * logicalBlockAddress,
    USB_DEVICE_MSD_DWORD_VAL * logicalBlockLength
)
{
    /* Get block address and block length from CBW command structure.*/
    logicalBlockAddress->v[3] =  lCBW->CBWCB[2];
    logicalBlockAddress->v[2] =  lCBW->CBWCB[3];
    logicalBlockAddress->v[1] =  lCBW->CBWCB[4];
    logicalBlockAddress->v[0] =  lCBW->CBWCB[5];        
    logicalBlockLength->byte.HB = lCBW->CBWCB[7];   /* MSB of Transfer Length (in number of blocks, not bytes) */
    logicalBlockLength->byte.LB = lCBW->CBWCB[8];   /* LSB of Transfer Length (in number of blocks, not bytes) */   
}    

// ******************************************************************************
/* Function:
    void  _USB_DEVICE_MSD_SaveBlockAddressAndLength
    (
        USB_MSD_CBW * lCBW,
        USB_DEVICE_MSD_DWORD_VAL * logicalBlockAddress,
        USB_DEVICE_MSD_DWORD_VAL * logicalBlockLength
    )

  Summary:
    This function saves the logical unit's block address and block length back
    into the CBW structure.

  Description:
    This function saves the logical unit's block address and block length back
    into the CBW structure.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

void  _USB_DEVICE_MSD_SaveBlockAddressAndLength
(
    USB_MSD_CBW * lCBW,
    USB_DEVICE_MSD_DWORD_VAL * logicalBlockAddress,
    USB_DEVICE_MSD_DWORD_VAL * logicalBlockLength
)
{
    /* Save back the updated address and logical block into the CBW
     * command structure. */
    lCBW->CBWCB[2] = logicalBlockAddress->v[3];
    lCBW->CBWCB[3] = logicalBlockAddress->v[2];
    lCBW->CBWCB[4] = logicalBlockAddress->v[1];
    lCBW->CBWCB[5] = logicalBlockAddress->v[0];

    /* MSB of Transfer Length (in number of blocks, not bytes) */
    lCBW->CBWCB[7] = logicalBlockLength->byte.HB;
    /* LSB of Transfer Length (in number of blocks, not bytes) */
    lCBW->CBWCB[8] = logicalBlockLength->byte.LB;   

}    

USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_ProcessRead
(
    SYS_MODULE_INDEX iMSD,
    uint8_t *commandStatus
)
{
    USB_MSD_CBW *lCBW;
    uint8_t *msdBuffer;
    size_t mediaReadBlockSize = 0;
    uint8_t logicalUnit;

    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE mediaReadWriteHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    USB_DEVICE_MSD_MEDIA_FUNCTIONS * mediaFunctions;
    USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA * mediaDynamicData;

    USB_DEVICE_MSD_INSTANCE * msdInstance = &gUSBDeviceMSDInstance[iMSD];
    USB_DEVICE_MSD_DWORD_VAL logicalBlockLength;
    USB_DEVICE_MSD_DWORD_VAL logicalBlockAddress;

    DRV_HANDLE drvHandle;

    /* Pointer to the CBW */ 
    lCBW = (USB_MSD_CBW *)msdInstance->msdCBW; // Pointer to CBW

    /* Logical unit being addressed */
    logicalUnit = lCBW->bCBWLUN;

    /* Get the media dynamic data */
    mediaDynamicData = &msdInstance->mediaDynamicData[logicalUnit];

    /* Pointer to the media functions for this LUN */
    mediaFunctions = &msdInstance->mediaData[logicalUnit].mediaFunctions;

    /* Pointer to the working buffer for this LUN */
    msdBuffer = msdInstance->mediaData[logicalUnit].sectorBuffer;
    drvHandle = mediaDynamicData->mediaHandle;

    *commandStatus = USB_MSD_CSW_COMMAND_PASSED;
    logicalBlockAddress.Val = 0;
    logicalBlockLength.Val = 0;

    _USB_DEVICE_MSD_GetBlockAddressAndLength(lCBW, &logicalBlockAddress, &logicalBlockLength);

    if (mediaDynamicData->mediaState == USB_DEVICE_MSD_MEDIA_OPERATION_COMPLETE)
    {
        /* Keep queuing IRPs as long as there is buffered data available with the MSD. */
        if (msdInstance->numPendingIrps != msdInstance->bufferOffset)
        {
            msdInstance->rxTxTotalDataByteCount += mediaDynamicData->sectorSize;
            msdInstance->irpTx.size = mediaDynamicData->sectorSize;
            msdInstance->irpTx.data = (void *)&msdBuffer[msdInstance->numPendingIrps * 512];
            msdInstance->irpTx.flags = USB_DEVICE_IRP_FLAG_DATA_PENDING;

            /* Submit the endpoint */
            USB_DEVICE_IRPSubmit( msdInstance->hUsbDevHandle, msdInstance->bulkEndpointTx, &msdInstance->irpTx);
            msdInstance->numPendingIrps ++;

            /* There is still data to be transferred. Continue to be in the IN state. */
            return USB_DEVICE_MSD_STATE_DATA_IN;
        }

        /* Reset the counters */
        msdInstance->numPendingIrps = 0;
        msdInstance->bufferOffset = 0;

        mediaDynamicData->mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_IDLE;
        if (logicalBlockLength.Val == 0)
        {
            /* End the data stage and move to CSW state */
            return USB_DEVICE_MSD_STATE_CSW;
        }
    }

    if (mediaDynamicData->mediaState == USB_DEVICE_MSD_MEDIA_OPERATION_IDLE)
    {
        mediaDynamicData->mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_PENDING;

        /* Find the amount of buffering available with the MSD driver. */
        if (logicalBlockLength.Val > _DRV_MSD_NUM_SECTORS_BUFFERING)
        {
            msdInstance->bufferOffset = _DRV_MSD_NUM_SECTORS_BUFFERING;
        }
        else
        {
            msdInstance->bufferOffset = logicalBlockLength.Val;
        }

        /* Find the media read block size */
        mediaReadBlockSize = mediaDynamicData->mediaGeometry->geometryTable[0].blockSize;

        /* Read bufferOffset number of sectors data from the media. */
        mediaFunctions->blockRead (drvHandle, 
                        &mediaReadWriteHandle, 
                        (uint8_t*)&msdBuffer[0],
                        (logicalBlockAddress.Val * (mediaDynamicData->sectorSize/mediaReadBlockSize)),
                        msdInstance->bufferOffset * (mediaDynamicData->sectorSize/mediaReadBlockSize));

        if (mediaReadWriteHandle == SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID)
        {
            /* Media Read Failed. */
            *commandStatus = USB_MSD_CSW_COMMAND_FAILED;
            return USB_DEVICE_MSD_STATE_CSW;
        }

        /* Update the amount of data read and the sector address
         * read. */
        logicalBlockLength.Val -= msdInstance->bufferOffset;
        logicalBlockAddress.Val += msdInstance->bufferOffset;

        _USB_DEVICE_MSD_SaveBlockAddressAndLength(lCBW, &logicalBlockAddress, &logicalBlockLength);

        msdInstance->numPendingIrps = 0;
    }
    else if (mediaDynamicData->mediaState == USB_DEVICE_MSD_MEDIA_OPERATION_ERROR)
    {
        *commandStatus = USB_MSD_CSW_COMMAND_FAILED;
        return USB_DEVICE_MSD_STATE_CSW;
    }

    return USB_DEVICE_MSD_STATE_DATA_IN;
}

USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_ProcessWrite
(
    SYS_MODULE_INDEX iMSD,
    uint8_t * commandStatus
)
{
    /* This function processes the command received in the CBW */
    USB_MSD_CBW *lCBW;
    uint8_t * msdBuffer;
    uint8_t * writeBlockBackupBuffer;
    size_t mediaReadBlockSize = 0;
    size_t mediaWriteBlockSize = 0;
    uint8_t sectorsPerBlock = 0;
    uint32_t memoryBlock = 0;
    uint8_t logicalUnit;

    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE mediaReadWriteHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    USB_DEVICE_MSD_MEDIA_FUNCTIONS * mediaFunctions;
    USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA * mediaDynamicData;

    USB_DEVICE_MSD_INSTANCE * msdInstance = &gUSBDeviceMSDInstance[iMSD];
    USB_DEVICE_MSD_DWORD_VAL logicalBlockLength;
    USB_DEVICE_MSD_DWORD_VAL logicalBlockAddress;

    DRV_HANDLE              drvHandle;

    /* Pointer to the CBW */ 
    lCBW = (USB_MSD_CBW *)msdInstance->msdCBW; // Pointer to CBW

    /* Logical unit being addressed */
    logicalUnit = lCBW->bCBWLUN;   

    /* Get the media dynamic data */
    mediaDynamicData = &msdInstance->mediaDynamicData[logicalUnit] ;

    /* Pointer to the media functions for this LUN */
    mediaFunctions = &msdInstance->mediaData[logicalUnit].mediaFunctions;

    /* Pointer to the working buffer for this LUN */
    msdBuffer = msdInstance->mediaData[logicalUnit].sectorBuffer;
    writeBlockBackupBuffer = msdInstance->mediaData[logicalUnit].blockBuffer;

    /* Assume that the command will pass */ 
    *commandStatus = USB_MSD_CSW_COMMAND_PASSED; 

    /* Reset some of the local variables */
    logicalBlockAddress.Val = 0;
    logicalBlockLength.Val = 0;
    drvHandle = mediaDynamicData->mediaHandle;

    /* Media is not write protected. Go ahead writing the media 
     * Get the logical block address, transfer length fields from
     * Command Block Wrapper */
    _USB_DEVICE_MSD_GetBlockAddressAndLength(lCBW, &logicalBlockAddress, &logicalBlockLength);

    /* This is the media write block size in bytes. We will need to make
     * a translation from media block size to the sector size */
    mediaWriteBlockSize = mediaDynamicData->mediaGeometry->geometryTable[1].blockSize;
    mediaReadBlockSize  = mediaDynamicData->mediaGeometry->geometryTable[0].blockSize;

    if (mediaDynamicData->sectorSize > mediaWriteBlockSize)
    {
        sectorsPerBlock = 1;
    }
    else
    {
        sectorsPerBlock = mediaWriteBlockSize/mediaDynamicData->sectorSize;
    }

    memoryBlock = logicalBlockAddress.Val/sectorsPerBlock;

    if (mediaDynamicData->mediaState == USB_DEVICE_MSD_MEDIA_OPERATION_COMPLETE)
    {
        if (logicalBlockLength.Val == 0)
        {
            /* Done writing all the blocks. Move on to the CSW Stage. */
            return USB_DEVICE_MSD_STATE_CSW;
        }

        if (msdInstance->numSectorsToWrite == 0)
        {
            /* The amount of data as computed in the previous iteration is transferred.
               There is still data to be transferred. Compute the next set of data to be transferred.
             */
            mediaDynamicData->mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_IDLE;
        }
    }
    else if (mediaDynamicData->mediaState == USB_DEVICE_MSD_MEDIA_OPERATION_ERROR)
    {
        /* There was an error while writing the data. */
        (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED;
        return USB_DEVICE_MSD_STATE_CSW;
    }

    /* operation state is idle */
    if (mediaDynamicData->mediaState == USB_DEVICE_MSD_MEDIA_OPERATION_IDLE)
    {
        /* This is the initial condition. Number of sectors to be written in this block is zero. */
        if (msdInstance->numSectorsToWrite == 0)
        {
            /* Find the number of sectors to be written in this block */
            if (sectorsPerBlock == 1)
            {
                /* Media sector size and the USB MSD Sector size are the same. */
                if (logicalBlockLength.Val > _DRV_MSD_NUM_SECTORS_BUFFERING)
                {
                    /* The number of blocks to be transferred is more than the buffer size
                       available at the MSD. Read buffer size worth of data from the USB Host.
                       Mark this as the number of blocks to be written to the media. */
                    msdInstance->numUsbSectors = _DRV_MSD_NUM_SECTORS_BUFFERING;
                }
                else
                {
                    /* The number of blocks to be transferred is less than the buffer size available
                       at the MSD. Read all requested number of blocks from the USB Host.
                       Mark this as the number of blocks to be written to the media. */
                    msdInstance->numUsbSectors = logicalBlockLength.Val;
                }
            }
            else
            {
                /* Media sector and the USB MSD Sector size is not the same. Use the 
                   the read-modify-write concept if a partial media sector is to be 
                   updated. */
                uint8_t sectorOffsetInBlock = (logicalBlockAddress.Val - (memoryBlock * sectorsPerBlock));
                uint8_t numRemainingSectorsInBlock = sectorsPerBlock - sectorOffsetInBlock;

                if (logicalBlockLength.Val > numRemainingSectorsInBlock)
                {
                    /* Number of sectors to be written > numRemainingSectorsInBlock
                       number of sectors to read from usb = numRemainingSectorsInBlock
                       number of sectors to be written in this block = numRemainingSectorsInBlock */
                    if (numRemainingSectorsInBlock > _DRV_MSD_NUM_SECTORS_BUFFERING)
                    {
                        msdInstance->numUsbSectors = _DRV_MSD_NUM_SECTORS_BUFFERING;
                    }
                    else
                    {
                        msdInstance->numUsbSectors = numRemainingSectorsInBlock;
                    }
                }
                else
                {
                    /* Number of sectors to write is less than that of the available buffer
                       capacity.
                       Read all the sectors from the USB.
                       Update the number of sectors to be written to the media. */

                    if (logicalBlockLength.Val > _DRV_MSD_NUM_SECTORS_BUFFERING)
                    {
                        msdInstance->numUsbSectors = _DRV_MSD_NUM_SECTORS_BUFFERING;
                    }
                    else
                    {
                        msdInstance->numUsbSectors = logicalBlockLength.Val;
                    }
                }
            }

            msdInstance->numSectorsToWrite = msdInstance->numUsbSectors;
            msdInstance->bufferOffset = 0;
        }

        if (msdInstance->numUsbSectors > 0)
        {
            /* There is still data that needs to be read from the USB. 
               Find the correct offset in the buffer for the new data.
             */
            msdInstance->irpRx.data = (void *)&msdBuffer[msdInstance->bufferOffset * 512];
            msdInstance->irpRx.size = mediaDynamicData->sectorSize;
            msdInstance->irpRx.flags = USB_DEVICE_IRP_FLAG_DATA_PENDING;

            /* Submit IRP to receive more data */
            USB_DEVICE_IRPSubmit (msdInstance->hUsbDevHandle, msdInstance->bulkEndpointRx, &msdInstance->irpRx);

            /* Increment the data buffer offset */
            msdInstance->bufferOffset++;
            /* Decrement the number of sectors to read from usb */
            msdInstance->numUsbSectors--;
        }
        else
        {
            /* There is no need to use the read-modify-write cycle if the media sector size
               and the usb msd sector size are the same or if the whole of the media sector is being programmed. */
            if ((sectorsPerBlock == 1) || (sectorsPerBlock == msdInstance->numSectorsToWrite))
            {
                /* set the operation status to complete */
                mediaDynamicData->mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_COMPLETE;
            }
            else
            {
                /* set the operation status to pending */
                mediaDynamicData->mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_PENDING;

                /* Read one media sector worth of data. */
                mediaFunctions->blockRead(drvHandle, &mediaReadWriteHandle,
                        writeBlockBackupBuffer, (memoryBlock * (mediaWriteBlockSize/mediaReadBlockSize)),
                        (mediaWriteBlockSize/mediaReadBlockSize));

                if (mediaReadWriteHandle == SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID)
                {
                    /* Media read failed. */
                    *commandStatus = USB_MSD_CSW_COMMAND_FAILED;
                    return USB_DEVICE_MSD_STATE_CSW;
                }
            }
        }
    }

    if (mediaDynamicData->mediaState == USB_DEVICE_MSD_MEDIA_OPERATION_COMPLETE)
    {
        uint8_t *data;
        uint32_t blockAddress;
        uint32_t numBlocks;

        if ((sectorsPerBlock == 1) || (sectorsPerBlock == msdInstance->numSectorsToWrite))
        {
            data = msdBuffer;
            numBlocks = msdInstance->numSectorsToWrite / sectorsPerBlock;
        }
        else
        {
            uint16_t i = 0;
            uint16_t sectorOffsetWithinBlock = (logicalBlockAddress.Val - (memoryBlock * sectorsPerBlock)) * mediaDynamicData->sectorSize;
            /*
               find the offset in the write block buffer
               update with the data from the usb
             */

            /* Update the sector in the write block */
            for (i = 0; i < (mediaDynamicData->sectorSize * msdInstance->numSectorsToWrite); i ++)
            {
                writeBlockBackupBuffer[sectorOffsetWithinBlock + i] = msdBuffer[i];
            }

            data = writeBlockBackupBuffer;
            numBlocks = 1;
        }

        blockAddress = logicalBlockAddress.Val / sectorsPerBlock;

        if (mediaDynamicData->sectorSize > mediaWriteBlockSize)
        {
            uint32_t numBlocksInSector = 0;
            numBlocksInSector = (mediaDynamicData->sectorSize / mediaWriteBlockSize);
            numBlocks *= numBlocksInSector;
            blockAddress = logicalBlockAddress.Val * numBlocksInSector;
        }

        mediaDynamicData->mediaState = USB_DEVICE_MSD_MEDIA_OPERATION_PENDING;

        /* number of sectors to be written in this block != 0 */
        /* Write data to the media */
        mediaFunctions->blockWrite (drvHandle, &mediaReadWriteHandle, 
                (uint8_t*)data, blockAddress, numBlocks);

        if (mediaReadWriteHandle == SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID)
        {
            /* Media write failed. */
            *commandStatus = USB_MSD_CSW_COMMAND_FAILED;
            return USB_DEVICE_MSD_STATE_CSW;
        }

        /* Update the total byte count */
        msdInstance->rxTxTotalDataByteCount += (msdInstance->numSectorsToWrite * 512);

        /* Updated the block address and the length values */
        logicalBlockAddress.Val += msdInstance->numSectorsToWrite;
        logicalBlockLength.Val -= msdInstance->numSectorsToWrite;

        /* Save back the updated address and logical block */
        _USB_DEVICE_MSD_SaveBlockAddressAndLength(lCBW, &logicalBlockAddress, &logicalBlockLength);

        msdInstance->numSectorsToWrite = 0;
    }

    return USB_DEVICE_MSD_STATE_DATA_OUT;
}

// *****************************************************************************
/* Function:
    USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_ProcessNonRWCommand
    (
        SYS_MODULE_INDEX iMSD,
        uint8_t * commandStatus
    )  

  Summary:
    Processes the command.

  Description:
    Processes the command.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_ProcessNonRWCommand
(
    SYS_MODULE_INDEX iMSD,
    uint8_t * commandStatus
)
{
    /* This function processes the command received in the CBW */
    USB_MSD_CBW *lCBW;
    uint8_t logicalUnit;
    uint8_t * msdBuffer;
    uint32_t length = 0;

    USB_DEVICE_MSD_MEDIA_FUNCTIONS * mediaFunctions;
    USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA * mediaDynamicData;
    USB_DEVICE_MSD_STATE msdNextState = USB_DEVICE_MSD_STATE_CSW;

    USB_DEVICE_MSD_INSTANCE * msdInstance = &gUSBDeviceMSDInstance[iMSD];
    USB_DEVICE_MSD_DWORD_VAL capacity = {.Val = 0};
    USB_DEVICE_MSD_DWORD_VAL sectorSize = {.Val = 0};

    DRV_HANDLE              drvHandle;
    SYS_FS_MEDIA_GEOMETRY * mediaGeometry;

    /* Pointer to the CBW */ 
    lCBW = (USB_MSD_CBW *)msdInstance->msdCBW; // Pointer to CBW

    /* Logical unit being addressed */
    logicalUnit = lCBW->bCBWLUN;   

    /* Get the media dynamic data */
    mediaDynamicData = &msdInstance->mediaDynamicData[logicalUnit];

    /* Pointer to the media functions for this LUN */
    mediaFunctions = &msdInstance->mediaData[logicalUnit].mediaFunctions;

    /* Pointer to the working buffer for this LUN */
    msdBuffer = msdInstance->mediaData[logicalUnit].sectorBuffer;

    /* Assume that the command will pass */ 
    (* commandStatus) = USB_MSD_CSW_COMMAND_PASSED; 

    drvHandle = mediaDynamicData->mediaHandle;

    /* Process the command */
    switch(lCBW->CBWCB[0])
    {
        case SCSI_INQUIRY:
            {
                uint8_t inquiryLen = 0;
                if ((lCBW->dCBWDataTransferLength == 0) ||
                        (!(lCBW->bmCBWFlags.value & USB_MSD_CBW_DIRECTION_BITMASK)))
                {
                    /* Fail the command if
                       1. Host does not want to receive any data.
                       2. If the data transfer direction is not IN.
                       */
                    (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED; 
                    break;
                }

                inquiryLen = sizeof(SCSI_INQUIRY_RESPONSE);
                if (inquiryLen > lCBW->dCBWDataTransferLength)
                {
                    /* Transfer the amount of data that the host is expecting. */
                    length = lCBW->dCBWDataTransferLength;
                }
                else
                {
                    /* Host is expecting more data. But transfer just inquiryLen data */
                    length = inquiryLen;
                }

                msdInstance->rxTxTotalDataByteCount = length;

                _USB_DEVICE_MSD_SendDataToUsb (iMSD,
                        (uint8_t *)&msdInstance->mediaData[logicalUnit].inquiryResponse,
                        length);
            }
            break;

        case SCSI_MODE_SENSE:
            {
                uint8_t modeSenseLen = 0;
                if ((lCBW->dCBWDataTransferLength == 0) ||
                        (!(lCBW->bmCBWFlags.value & USB_MSD_CBW_DIRECTION_BITMASK)))
                {
                    /* Fail the command if
                       1. Host does not want to receive any data.
                       2. If the data transfer direction is not IN.
                       */
                    (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED; 
                    break;
                }

                if (mediaDynamicData->mediaPresent == false)
                {
                    /* Fail the command if the media is not present. */
                    (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED;
                    break;
                }

                modeSenseLen = 4;
                if (modeSenseLen > lCBW->dCBWDataTransferLength)
                {
                    /* Transfer the amount of data that the host is expecting. */
                    length = lCBW->dCBWDataTransferLength;
                }
                else
                {
                    /* Host is expecting more data. But transfer just modeSenseLen data */
                    length = modeSenseLen;
                }

                msdInstance->rxTxTotalDataByteCount = length;

                /* Pad the bytes with zeroes first */
                memset(msdBuffer, 0, length);

                /* Add required information */
                msdBuffer[0] = 0x03;
                msdBuffer[1] = 0x00;
                msdBuffer[2] = 0x00;

                if(mediaFunctions->isWriteProtected(drvHandle))
                {
                    /* Media is write protected */
                    msdBuffer[2] = 0x80;
                }

                msdBuffer[3]= 0x00;

                _USB_DEVICE_MSD_SendDataToUsb (iMSD, &msdBuffer[0], length);
            }
            break;

        case SCSI_REQUEST_SENSE:
            {
                uint8_t requestSenseLen = 0;
                if(mediaDynamicData->mediaPresent == false)
                {
                    /* Media is not present */
                    mediaDynamicData->senseData->SenseKey = SCSI_SENSE_NOT_READY;
                    mediaDynamicData->senseData->ASC = SCSI_ASC_MEDIUM_NOT_PRESENT;
                    mediaDynamicData->senseData->ASCQ = SCSI_ASCQ_MEDIUM_NOT_PRESENT;
                }

                if ((lCBW->dCBWDataTransferLength == 0) ||
                        (!(lCBW->bmCBWFlags.value & USB_MSD_CBW_DIRECTION_BITMASK)))
                {
                    /* Fail the command if
                       1. Host does not want to receive any data.
                       2. If the data transfer direction is not IN.
                       */
                    (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED; 
                    break;
                }

                requestSenseLen = sizeof(SCSI_SENSE_DATA);
                if (requestSenseLen > lCBW->dCBWDataTransferLength)
                {
                    /* Transfer the amount of data that the host is expecting. */
                    length = lCBW->dCBWDataTransferLength;
                }
                else
                {
                    /* Host is expecting more data. But transfer just requestSenseLen data */
                    length = requestSenseLen;
                }

                msdInstance->rxTxTotalDataByteCount = length;

                _USB_DEVICE_MSD_SendDataToUsb (iMSD, (uint8_t *)mediaDynamicData->senseData, length);

                /* Now the host is reading the sense data, we are good to reset in
                 * the next CBW stage */
                mediaDynamicData->resetSenseData = true;
            }
            break;

        case SCSI_TEST_UNIT_READY:
            {
                if (lCBW->dCBWDataTransferLength != 0)
                {
                    /* Fail the command if
                       1. Host is expecting to receive data.
                       */
                    (* commandStatus) = USB_MSD_CSW_COMMAND_FAILED; 
                    break;
                }

                if (mediaDynamicData->mediaPresent == false)
                {
                    /* Fail the command if the media is not present. */
                    (* commandStatus) = USB_MSD_CSW_COMMAND_FAILED; 
                    break;
                }
            }
            break;

        case SCSI_READ_CAPACITY:
            {
                uint8_t readCapacityLen = 0;
                if ((lCBW->dCBWDataTransferLength == 0) ||
                        (!(lCBW->bmCBWFlags.value & USB_MSD_CBW_DIRECTION_BITMASK)))
                {
                    /* Fail the command if
                       1. Host does not want to receive any data.
                       2. If the data transfer direction is not IN.
                       */
                    (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED; 
                    break;
                }

                if (mediaDynamicData->mediaPresent == false)
                {
                    /* Fail the command if the media is not present. */
                    (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED;
                    break;
                }

                readCapacityLen = 8;
                if (readCapacityLen > lCBW->dCBWDataTransferLength)
                {
                    /* Transfer the amount of data that the host is expecting. */
                    length = lCBW->dCBWDataTransferLength;
                }
                else
                {
                    /* Host is expecting more data. But transfer just readCapacityLen data */
                    length = readCapacityLen;
                }

                msdInstance->rxTxTotalDataByteCount = length;

                /* Get the information from the physical media */
                mediaGeometry = mediaDynamicData->mediaGeometry;
                if(mediaGeometry->numWriteRegions != 0)
                {
                    /* If there is a write region then return
                     * the size of the write region. The size of
                     * the write region will be available after
                     * read regions.  */
                    if (mediaGeometry->geometryTable[1].numBlocks > mediaDynamicData->sectorSize)
                    {
                        capacity.Val = ((mediaGeometry->geometryTable[1].numBlocks / mediaDynamicData->sectorSize) *
                                mediaGeometry->geometryTable[1].blockSize) - 1;
                    }
                    else
                    {
                        capacity.Val = (((mediaGeometry->geometryTable[1].numBlocks * mediaGeometry->geometryTable[1].blockSize) / 
                                mediaDynamicData->sectorSize) - 1);
                    }
                }
                else
                {
                    /* Return the number of read blocks. Only the first entry is
                     * considered. Refer to the geometry table structure for more
                     * details. */
                    if (mediaGeometry->geometryTable[0].numBlocks > mediaDynamicData->sectorSize)
                    {
                        capacity.Val = ((mediaGeometry->geometryTable[0].numBlocks / mediaDynamicData->sectorSize) *
                                mediaGeometry->geometryTable[0].blockSize) - 1;
                    }
                    else
                    {
                        capacity.Val = (((mediaGeometry->geometryTable[0].numBlocks * mediaGeometry->geometryTable[0].blockSize) / 
                                mediaDynamicData->sectorSize) - 1);
                    }
                }

                /* Sector size was udpated when the media was opened in
                 * the _USB_DEVICE_MSD_CheckAndUpdateMediaState() function */
                sectorSize.Val = mediaDynamicData->sectorSize;

                /* Copy the data to the buffer.  Host expects the response in big
                 * endian format */
                msdBuffer[0] = capacity.v[3];
                msdBuffer[1] = capacity.v[2];
                msdBuffer[2] = capacity.v[1];
                msdBuffer[3] = capacity.v[0];

                msdBuffer[4] = sectorSize.v[3];
                msdBuffer[5] = sectorSize.v[2];
                msdBuffer[6] = sectorSize.v[1];
                msdBuffer[7] = sectorSize.v[0];
                
                _USB_DEVICE_MSD_SendDataToUsb (iMSD, &msdBuffer[0], length);
            }
            break;

        case SCSI_VERIFY:
        case SCSI_STOP_START:
            if(mediaDynamicData->mediaPresent == false)
            {
                (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED;
            }
            break;

        case SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL:
            mediaDynamicData->senseData->SenseKey = SCSI_SENSE_ILLEGAL_REQUEST;
            mediaDynamicData->senseData->ASC = SCSI_ASC_INVALID_COMMAND_OPCODE;
            mediaDynamicData->senseData->ASCQ = SCSI_ASCQ_INVALID_COMMAND_OPCODE;
            (* commandStatus) = USB_MSD_CSW_COMMAND_FAILED;
            break;

        default:
            _USB_DEVICE_MSD_ResetSenseData(mediaDynamicData->senseData);
            // Set sense data.
            mediaDynamicData->senseData->SenseKey = SCSI_SENSE_ILLEGAL_REQUEST;
            mediaDynamicData->senseData->ASC = SCSI_ASC_INVALID_COMMAND_OPCODE;
            mediaDynamicData->senseData->ASCQ = SCSI_ASCQ_INVALID_COMMAND_OPCODE;
            (*commandStatus) = USB_MSD_CSW_COMMAND_FAILED;
            break;
    }

    return msdNextState;
}

// *****************************************************************************
/* Function:
    void    _USB_DEVICE_MSD_Deinitialization (SYS_MODULE_INDEX iMSD)

  Summary:
    MSD function driver deinitialization.

  Description:
    Deinitializes the given instance of the MSD function driver. This function
    is called by the USB DEVICE layer.

  Remarks:
    This is a local function and should not be called directly by an
    application.
 
*/

void _USB_DEVICE_MSD_Deinitialization ( SYS_MODULE_INDEX iMSD )
{
    USB_DEVICE_MSD_INSTANCE * msdInstance;
    USB_DEVICE_MSD_MEDIA_FUNCTIONS * mediaFunctions;
    uint8_t count;

    msdInstance = &gUSBDeviceMSDInstance[ iMSD ] ;
    

    // close all open logical units..
    for(count = 0; count < msdInstance->numberOfLogicalUnits; count++)
    {
        if( msdInstance->mediaDynamicData[count].mediaHandle != DRV_HANDLE_INVALID )
        {
			mediaFunctions = &msdInstance->mediaData[count].mediaFunctions;
            mediaFunctions->close(msdInstance->mediaDynamicData[count].mediaHandle);
        }
    }
    /* Cancel all RX IRPs */
    USB_DEVICE_IRPCancelAll( msdInstance->hUsbDevHandle, msdInstance->bulkEndpointRx );
    
    /* Cancel all TX IRPs */
    USB_DEVICE_IRPCancelAll( msdInstance->hUsbDevHandle, msdInstance->bulkEndpointTx );

    /* Close the endpoint */
    USB_DEVICE_EndpointDisable( msdInstance->hUsbDevHandle, msdInstance->bulkEndpointRx );
    USB_DEVICE_EndpointDisable( msdInstance->hUsbDevHandle, msdInstance->bulkEndpointTx );
}

// ******************************************************************************
/* Function:
     void _USB_DEVICE_MSD_ResetSenseData ( USB_DEVICE_MSD_SENSE_DATA * senseData )

  Summary:
    MSD function driver deinitialization.
  
  Description:
    This routine resets the Sense Data, initializing the structure
    USB_DEVICE_MSD_SENSE_DATA senseData.

  Remarks:
    This is a local function and should not be called directly by an
    application.
*/

void _USB_DEVICE_MSD_ResetSenseData( SCSI_SENSE_DATA * senseData )
{
   	senseData->ResponseCode = SCSI_SENSE_CURRENT;
	senseData->VALID = 0;			// no data in the information field
	senseData->Obsolete = 0x0;
	senseData->SenseKey = SCSI_SENSE_NO_SENSE;
	senseData->ILI = 0;
	senseData->EOM = 0;
	senseData->FILEMARK = 0;
	senseData->InformationB0 = 0x00;
	senseData->InformationB1 = 0x00;
	senseData->InformationB2 = 0x00;
	senseData->InformationB3 = 0x00;
	senseData->AddSenseLen = 0x0a;	// n-7 (n=17 (0..17))
	senseData->CmdSpecificInfo = 0x00000000;
	senseData->ASC = 0x0;
	senseData->ASCQ = 0x0;
	senseData->FRUC = 0x0;
	senseData->SenseKeySpecific[0] = 0x0;
	senseData->SenseKeySpecific[1] = 0x0;
	senseData->SenseKeySpecific[2] = 0x0;
}

