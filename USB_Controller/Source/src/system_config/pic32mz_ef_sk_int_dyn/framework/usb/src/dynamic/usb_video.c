/* ************************************************************************** */
/** Descriptive File Name
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "usb/usb_video.h"
#include "usb/src/usb_video_local.h"
#include "system/debug/sys_debug.h"

typedef uintptr_t USB_DEVICE_VIDEO_INDEX;


void _USB_DEVICE_VIDEO_IRPCancelAll(USB_DEVICE_VIDEO_INDEX iAudio );

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

// *****************************************************************************




const USB_DEVICE_FUNCTION_DRIVER videoFunctionDriver =
{

    /* VIDEO init function */
    .initializeByDescriptor             = &_USB_DEVICE_VIDEO_Initialize ,

    /* VIDEO de-init function */
    .deInitialize           = &_USB_DEVICE_VIDEO_Deinitialize ,

    /* VIDEO set-up packet handler */
    .controlTransferNotification     = &_USB_DEVICE_VIDEO_ControlTransferHandler,

    /* VIDEO tasks function */
    .tasks                  = NULL,

    /* Video Global Initialize function */
    .globalInitialize = _USB_DEVICE_VIDEO_GlobalInitialize
        
};

 
 
void _USB_DEVICE_VIDEO_ControlTransferHandler
(
    SYS_MODULE_INDEX iVideo,
    USB_DEVICE_EVENT controlEvent,
    USB_SETUP_PACKET * setupRequest
)
{
    /* Obtain pointer to the Video Instance that is being addressed*/
    USB_DEVICE_VIDEO_INSTANCE *thisVideoInstance = &gUsbDeviceVideoInstance[iVideo];

    /* check the validity of the function driver index */
    if ( USB_DEVICE_VIDEO_INSTANCES_NUMBER <= iVideo )
    {
        /* invalid handle */
        SYS_ASSERT (false, "invalid Video Index");
    }

    /* Check Control Event  */
    switch(controlEvent)
    {
        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:
            /* A new SETUP packet received from Host*/
            _USB_DEVICE_VIDEO_SetupPacketHandler(iVideo,setupRequest);
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
            /* A Control Transfer data stage completed. Notify the application */
            thisVideoInstance->appEventCallBack
            (
                iVideo,
                USB_DEVICE_VIDEO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED,
                NULL,
                0
            );        
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_SENT:
            /* The Control Transfer data was sent the Host. Notify the application */
            thisVideoInstance->appEventCallBack
            (
                iVideo,
                USB_DEVICE_VIDEO_EVENT_CONTROL_TRANSFER_DATA_SENT,
                NULL,
                0
            );
            break;
        default:
            break;
    }
}/* End of function _USB_DEVICE_VIDEO_ControlTransferHandler */ 



// ******************************************************************************
/* Function:
   void _USB_DEVICE_VIDEO_SetupPacketHandler
   (
       USB_DEVICE_VIDEO_INDEX iVideo ,
       USB_SETUP_PACKET * setupPkt
   )

  Summary:
    Handles a fresh SETUP packet received from Host.

  Description:
    Handles a fresh SETUP packet received from Host.

  Returns:
    This is a local function and should not be called directly by the
    application.
*/


void _USB_DEVICE_VIDEO_SetupPacketHandler 
(
    USB_DEVICE_VIDEO_INDEX iVideo , 
    USB_SETUP_PACKET * setupPkt
)
{
    uint8_t videoControlInterfaceId;
    uint8_t interfaceId;
    uint8_t curAlternateSetting;
    uint8_t prevAlternateSetting;
    uint8_t streamIntfcIndex;
    uint8_t noOfEndpoints;
    USB_ENDPOINT ep; 
    uint16_t maxPacketSize;
    USB_DEVICE_HANDLE usbDeviceHandle;
    USB_DEVICE_VIDEO_EVENT event;
    USB_DEVICE_VIDEO_EVENT_DATA_INTERFACE_SETTING_CHANGED interfaceInfo;
    USB_ERROR endpointEnableResult; 
    USB_DEVICE_VIDEO_STREAMING_INTERFACE_ALTERNATE_SETTING *pCurAlternateStng;
    USB_DEVICE_VIDEO_STREAMING_INTERFACE_ALTERNATE_SETTING *pPrevAlternateStng;
    USB_DEVICE_VIDEO_STREAMING_INTERFACE *pStreamingInterface;

    /* Obtain pointer to the Video Instance that is being addressed*/
    USB_DEVICE_VIDEO_INSTANCE* videoInstance =  &gUsbDeviceVideoInstance[iVideo];

    /* Obtain pointer to the Video Interface collection*/
    USB_DEVICE_VIDEO_INTERFACE_COLLECTION *curInfCollection =
            &(videoInstance->infCollection);

    /* Get the Device Layer handle */
    usbDeviceHandle = gUsbDeviceVideoInstance[iVideo].devLayerHandle;
    
    /* Check if the request is a standard interface request*/
    if (  (setupPkt->RequestType == USB_SETUP_REQUEST_TYPE_STANDARD)
       && (setupPkt->Recipient == USB_SETUP_REQUEST_RECIPIENT_INTERFACE))

    {
        /* We have received Standard Set request */

        /* Retrieve interface number from the Setup packet */
        interfaceId = setupPkt->bIntfID;

        /* Retrieve video Control interface number*/
        videoControlInterfaceId = curInfCollection->bControlInterfaceNum;
        
        switch(setupPkt->bRequest)
        {
            case USB_REQUEST_SET_INTERFACE:
                curAlternateSetting = setupPkt->bAltID;
                if (interfaceId == videoControlInterfaceId)
                {
                    /*SET INTERFACE command was received to Video Control
                      Interface */
                    curInfCollection->bControlAltSetng = curAlternateSetting;
                }
                else
                {
                    /*An Video Streaming interface has received SET INTERFACE
                      command */
                    streamIntfcIndex = interfaceId - videoControlInterfaceId - 1;

                    /* Get pointer to the current video streaming interface */
                    pStreamingInterface = &(curInfCollection->streamInf[streamIntfcIndex]);
                    
                    /* Get pointer to the Interface Alternate setting. */
                    pCurAlternateStng
                    = &(pStreamingInterface->alterntSetting[curAlternateSetting]);
                    
                    /* Find out how many endpoint are present for this Interface
                     * Alternate setting */
                    noOfEndpoints = pCurAlternateStng->numEndPoints;

                    if ((noOfEndpoints) && (curAlternateSetting))
                    {

                        /* We have to enable the endpoint only if this alternate
                         setting has at least one endpoint and the alternate
                         setting is a non zero value */
                        
                        /*Retrieve endpoint size */
                        ep = pCurAlternateStng->isoDataEp.epAddr;
                        
                        /* retrieve max packet size. */
                        maxPacketSize
                             = pCurAlternateStng->isoDataEp.epMaxPacketSize;

                        /* Enable Isochronous Data Endpoint */
                        endpointEnableResult = USB_DEVICE_EndpointEnable
                        (
                            usbDeviceHandle ,
                            0,
                            ep ,
                            USB_TRANSFER_TYPE_ISOCHRONOUS ,
                            maxPacketSize
                        );
                        if (endpointEnableResult != USB_ERROR_NONE)
                        {
                            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "_USB_DEVICE_VIDEO_SetupPacketHandler():  Endpoint not Enabled");
                        }

                        if (noOfEndpoints == 2)
                        {
                            /* If number of Endpoints is Two, then it is sure
                             * that this alternate setting reports a Isochronous
                             * Sync Endpoint. Enable the Sync Endpoint. */
                        }

                        /* Change Video Instance object state to Initialized */ 
                        pStreamingInterface->state
                                    = USB_DEVICE_VIDEO_STRMNG_INTFC_INITIALIZED;
                    }
                    else /* alternateSetting  = 0 */ 
                    {
                        if (pStreamingInterface->state
                                   == USB_DEVICE_VIDEO_STRMNG_INTFC_INITIALIZED)
                        {
                            /* Disable the Endpoint in the previous Active Alternate setting. */ 
                            prevAlternateSetting = pStreamingInterface->activeSetting;

                            /* Get a pointer to the previous alternate setting. */ 
                            pPrevAlternateStng = &(pStreamingInterface->alterntSetting[prevAlternateSetting]);

                            /* Get endpoint no of the previous alternate setting */ 
                            ep = pPrevAlternateStng->isoDataEp.epAddr;
                            
                            USB_DEVICE_IRPCancelAll(usbDeviceHandle,  ep);

                            /* Disable the Endpoint */ 
                            USB_DEVICE_EndpointDisable(usbDeviceHandle ,ep);


                            if (noOfEndpoints == 2)
                            {
                                /* If number of Endpoints is Two, then it is sure
                                * that this alternate setting reports a Isochronous
                                * Sync Endpoint. Disable the Sync Endpoint. */
                            }
                            curInfCollection->streamInf[streamIntfcIndex].state
                                = USB_DEVICE_VIDEO_STRMNG_INTFC_NOT_INITIALIZED;
                            
                        }  
                    }

                    /* Remember the new alternate setting received */
                    curInfCollection->streamInf[streamIntfcIndex].activeSetting 
                    = curAlternateSetting;


                    /* Notify application about the SET_INTERFACE request */
                    interfaceInfo.interfaceNumber = interfaceId;
                    interfaceInfo.interfaceAlternateSetting = curAlternateSetting;
                    videoInstance->appEventCallBack
                    (
                        iVideo,
                        USB_DEVICE_VIDEO_EVENT_INTERFACE_SETTING_CHANGED,
                        &interfaceInfo,
                        0
                    );
                }
                /* Send an Acknowledgement to the Host */ 
                USB_DEVICE_ControlStatus( usbDeviceHandle,
                                USB_DEVICE_CONTROL_STATUS_OK);
            break;
            case USB_REQUEST_GET_INTERFACE:
                if (interfaceId == videoControlInterfaceId)
                {
                    curAlternateSetting = curInfCollection->bControlAltSetng;
                }
                else
                {
                    streamIntfcIndex = interfaceId - videoControlInterfaceId - 1;
                    curAlternateSetting
                    = curInfCollection->streamInf[streamIntfcIndex].activeSetting;
                }

                USB_DEVICE_ControlSend( usbDeviceHandle,
                        (void *)&curAlternateSetting, 1);
                break;
            default:
                break;

        }      
    }/* End of if((setupPkt->bmRequestType == */ 
    else if ( (setupPkt->RequestType == USB_SETUP_REQUEST_TYPE_CLASS )
            && ((setupPkt->Recipient == USB_SETUP_REQUEST_RECIPIENT_INTERFACE) 
            || (setupPkt->Recipient == USB_SETUP_REQUEST_RECIPIENT_ENDPOINT)))
    {
        /* We have received a Video Class specific Interface request or
			Video Class specific Endpoint request */
        switch (setupPkt->bRequest)
        {
            case USB_VIDEO_CS_SET_CUR:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_SET_CUR;
                break;
            case USB_VIDEO_CS_GET_CUR:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_GET_CUR;
                break;
            case USB_VIDEO_CS_SET_MIN:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_SET_MIN;
                break;
            case USB_VIDEO_CS_GET_MIN:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_GET_MIN;
                break;
            case USB_VIDEO_CS_SET_MAX:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_SET_MAX;
                break;
            case USB_VIDEO_CS_GET_MAX:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_GET_MAX;
                break;
            case USB_VIDEO_CS_SET_RES:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_SET_RES;
                break;
            case USB_VIDEO_CS_GET_RES:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_GET_RES;
                break;
            case USB_VIDEO_CS_SET_MEM:
                event = USB_DEVICE_VIDEO_EVENT_ENTITY_SET_MEM;
                break;
            case USB_VIDEO_CS_GET_MEM:
                event = USB_DEVICE_VIDEO_EVENT_ENTITY_GET_MEM;
                break;
            case USB_VIDEO_CS_GET_INFO:
                event = USB_DEVICE_VIDEO_EVENT_ENTITY_GET_INFO;         
                break;
            case USB_VIDEO_CS_GET_DEF:
                event = USB_DEVICE_VIDEO_EVENT_CONTROL_GET_DEF;
                break;
            case USB_VIDEO_CS_GET_STAT:
                event = USB_DEVICE_VIDEO_EVENT_ENTITY_GET_STAT;
                break;
            default:
            /* Unknown request. Stall the request */
                event = 0;
                USB_DEVICE_ControlStatus(videoInstance->devLayerHandle,
                                               USB_DEVICE_CONTROL_STATUS_ERROR);
        }
        if (( videoInstance->appEventCallBack) && (event))
        {
            /* Inform the application about the request */
            /* The application needs to handle both EP and entity specific 
               requests */
           videoInstance->appEventCallBack ( iVideo, event, setupPkt, 0);
        }        
    }/* End of else if */ 
}/* End of function _USB_DEVICE_VIDEO_SetupPacketHandler */ 


//******************************************************************************


// ******************************************************************************
/* Function:
    void _USB_DEVICE_VIDEO_GlobalInitialize ( void )

  Summary:
    This function initializes resources required common to all instances of VIDEO
    function driver.

  Description:
    This function initializes resources common to all instances of VIDEO function
    driver. This function is called by the USB Device layer during Initialization.

  Remarks:
    This is local function and should not be called directly by the application.
*/
void _USB_DEVICE_VIDEO_GlobalInitialize (void)
{
    OSAL_RESULT osal_err;

    /* Create Mutex for CDC IRP objects if not created already */
    if (gUSBDeviceVideoCommonDataObj.isMutexVideoIrpInitialized == false)
    {
        /* This means that mutexes where not created. Create them. */
        osal_err = OSAL_MUTEX_Create(&gUSBDeviceVideoCommonDataObj.mutexVIDEOIRP);

        if(osal_err != OSAL_RESULT_TRUE)
        {
            /*do not proceed lock was not created, let user know about error*/
            return;
        }

         /* Set this flag so that global mutexes get allocated only once */
         gUSBDeviceVideoCommonDataObj.isMutexVideoIrpInitialized = true;
    }
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_VIDEO_Initialize 
    (
        SYS_MODULE_INDEX iVideo, 
        DRV_HANDLE usbDeviceHandle,
        void* funcDriverInit, 
        uint8_t interfaceNumber, 
        uint8_t alternateSetting,
        uint8_t descriptorType, 
        uint8_t * pDescriptor 
    )

  Summary:
    This function is called by the device layer. It gets called multiple times as 
    and when the USB device layer starts parsing the descriptors

  Description:
    This function is called by the device layer. It gets called multiple times as 
    and when the USB device layer starts parsing the descriptors

  Remarks:
    This is local function and should not be called directly by the application.
*/
//******************************************************************************
void _USB_DEVICE_VIDEO_Initialize 
(
    SYS_MODULE_INDEX iVideo, 
    DRV_HANDLE usbDeviceHandle,
    void* funcDriverInit, 
    uint8_t interfaceNumber, 
    uint8_t alternateSetting,
    uint8_t descriptorType, 
    uint8_t * pDescriptor 
)
{
    /* Pointer to Standard Endpoint Descriptor */
    USB_ENDPOINT_DESCRIPTOR *pEPDesc;

    /* Pointer to Standard Interface descriptor */
    USB_INTERFACE_DESCRIPTOR *pStdInfDesc;
    
    /* Pointer to Class Specific Interface Descriptor */
    USB_VIDEO_CS_AC_INTERFACE_HEADER_DESCRIPTOR *pCsInfDesc;

    USB_DEVICE_VIDEO_INIT * videoInit;
    uint8_t epAddressAndDirection;
    uint16_t maxPacketSize;
    uint8_t cnt;
    uint8_t videoControlInterfaceId;
    uint8_t strmIntrfcIndex;
    USB_DEVICE_VIDEO_STREAMING_INTERFACE_ALTERNATE_SETTING *curAlternateStng;
    USB_DEVICE_VIDEO_INSTANCE * videoInstance = &gUsbDeviceVideoInstance[iVideo];
    USB_DEVICE_VIDEO_INTERFACE_COLLECTION *curInfCollection =
            &(videoInstance->infCollection);
    videoInit = ((USB_DEVICE_VIDEO_INIT *)funcDriverInit);
    videoInstance->queueSizeRead = videoInit->queueSizeRead;
    videoInstance->queueSizeWrite = videoInit->queueSizeWrite;
    videoInstance->currentQSizeRead = 0;
    videoInstance->currentQSizeWrite = 0; 

     /* Check the type of descriptor passed by device layer */
    switch ( descriptorType )
    { 
        case USB_DESCRIPTOR_INTERFACE: 
            /* Interface Descriptor Received */ 
            
            /* Save the device handle to the corresponding Video Instance object. */ 
            videoInstance->devLayerHandle = usbDeviceHandle;

            /* Save the Video Index to the corresponding Video Instance object. */ 
            videoInstance->videoIndex = iVideo;

            pStdInfDesc = (USB_INTERFACE_DESCRIPTOR*) pDescriptor;

            /* Check if it is USB video interface descriptor */
            if (pStdInfDesc->bInterfaceClass == USB_VIDEO_CLASS_CODE)
            {
                /* Check if it is an Video Control Interface descriptor */
                switch (pStdInfDesc->bInterfaceSubClass)
                {
                    case USB_VIDEO_VIDEOCONTROL:
                        videoInstance->flags.allFlags = 0;
                        /* The program control reached here means that the device
                           layer has detected an Standard Video Control interface 
                           descriptor. Now we have to save all the fields present 
                           in the Standard Video Control interface descriptor to 
                           the corresponding Video Instance Object. */

                        /*Save the Interface number to the corresponding Video 
                            Instance Object.*/
                        curInfCollection->bControlInterfaceNum 
                                    = pStdInfDesc->bInterfaceNumber;

                        /* Save the no of endpoints present in the Video Control 
                        Interface. If there are endpoints present as part of AC 
                        interface descriptor that must be an Interrupt endpoint.*/
                        curInfCollection->isIntEpExists = pStdInfDesc->bNumEndPoints;
                     break;
                
                
                    case USB_VIDEO_VIDEOSTREAMING:
                        /* We have received an Video Streaming Interface descriptor. 
                           Save the interface number to an array of Video streaming 
                           interfaces. For an video function the interface numbers
                           starts from Video Control Interface ID.*/

                        /* Save number of endpoints present in the streaming interface */ 
                        videoControlInterfaceId 
                                    = curInfCollection-> bControlInterfaceNum;
                        /* Find out video streaming interface array index. */ 
                        strmIntrfcIndex = interfaceNumber-videoControlInterfaceId-1;

                        /* Save no of endpoints present in the streaming interface */
                        curAlternateStng 
                        = &(videoInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting]); 
                        curAlternateStng->numEndPoints = pStdInfDesc->bNumEndPoints;
                    break;
                    default :
                        break; 
                }
            }
        break; /* End of case USB_DESCRIPTOR_INTERFACE: */ 
    
        /* Class Specific Video Control Interface Descriptor. */ 
        case USB_VIDEO_CS_INTERFACE:
            /* If we have already received Video Control Interface related
            descriptors then ignore all further Class specific descriptors */ 
            if (videoInstance->flags.videoControlInterfaceReady ==  true)
                return;
            pCsInfDesc = (USB_VIDEO_CS_AC_INTERFACE_HEADER_DESCRIPTOR*) pDescriptor;

            /* Check if this Class specific descriptor belongs to an AC Interface*/
            if (interfaceNumber == videoInstance->infCollection.bControlInterfaceNum)
            {
                if  (USB_VIDEO_HEADER == pCsInfDesc->bDescriptorSubtype)
                {
                    SYS_ASSERT((pCsInfDesc->bInCollection != USB_DEVICE_VIDEO_MAX_STREAMING_INTERFACES ),
                        "Maximum number of streaming interfaces defined does not match descriptor value");

                    /* Save number of video steaming interfaces */ 
                    videoInstance->infCollection.numStreamingInf = pCsInfDesc->bInCollection;

                    /* Save interface numbers of the all available video streaming and Midi interfaces. */
                    for (cnt = 0; cnt < USB_DEVICE_VIDEO_MAX_STREAMING_INTERFACES; cnt++ )
                    {
                        videoInstance->infCollection.streamInf[cnt].interfaceNum
                            = *((uint8_t *)&(pCsInfDesc->bInCollection) + cnt + 1);
                    }

                    /* Save video specification number */ 
                    videoInstance->infCollection.bcdADC = pCsInfDesc->bcdADC;
                    videoInstance->flags.videoControlInterfaceReady = true;
                } /* End of "if  (USB_VIDEO_HEADER == pCsInfDesc->bDescriptorSubtype)" */
            } /* End of "if (interfaceNumber == videoInstance[iVideo].infCollection.bControlInterfaceNum)" */
        break; /* End of "case DEVICE_VIDEO_CS_INTERFACE" */

        case USB_DESCRIPTOR_ENDPOINT:
            /* We have received an Endpoint descriptor from device Layer*/
            pEPDesc = ( USB_ENDPOINT_DESCRIPTOR* ) pDescriptor;
            videoControlInterfaceId = videoInstance->infCollection.bControlInterfaceNum;
            strmIntrfcIndex = interfaceNumber - videoControlInterfaceId - 1;
       
            /* Check if this Endpoint belongs to an video Streaming interface */ 
            if ((interfaceNumber == videoInstance->infCollection.streamInf[strmIntrfcIndex].interfaceNum))
            {
                /* Save the ep address */
                epAddressAndDirection = pEPDesc->bEndpointAddress;

                /* Save max packet size */
                maxPacketSize = ( ( USB_ENDPOINT_DESCRIPTOR* ) pDescriptor )->wMaxPacketSize;
       
                if (pEPDesc->usageType == USB_USAGE_DATA_ENDPOINT)
                {
                    /* Save ep address to the data interface */
                    if(alternateSetting == 1)
                    {
                        volatile uint8_t abc= 0;
                        abc = abc +1;
                    }
                    videoInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting].isoDataEp.epAddr = epAddressAndDirection;

                    /* Save max packet size to the data interface */
                    videoInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting].isoDataEp.epMaxPacketSize = maxPacketSize;
                }
                else if (pEPDesc->usageType == USB_USAGE_FEEDBACK_ENDPOINT)
                {
                    /* Save ep address to the Sync interface */
                    videoInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting].isoSyncEp.epAddr = epAddressAndDirection;

                    /* Save max packet size to the Sync interface */
                    videoInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting].isoSyncEp.epMaxPacketSize = maxPacketSize;
                }
            }
            break;

        default:
        break; 
    }//end of switch ( descriptorType )
}

// *****************************************************************************
// ******************************************************************************
/* Function:
    void _USB_DEVICE_VIDEO_Deinitialize ( SYS_MODULE_INDEX iVideo )

  Summary:
    This function is called by the device layer when Video Device is detached 
    from the Host. 

  Description:
    This function is called by the device layer when Video Device is detached 
    from the Host. 

  Remarks:
    This is local function and should not be called directly by the application.
*/
//******************************************************************************
void _USB_DEVICE_VIDEO_Deinitialize ( SYS_MODULE_INDEX iVideo )
{
    /* Cancel all IRPs on the owned endpoints and then
     * disable the endpoint */

    if(iVideo >= USB_DEVICE_VIDEO_INSTANCES_NUMBER)
    {
        SYS_ASSERT(false," Invalid instance");
        return;
    }
   _USB_DEVICE_VIDEO_IRPCancelAll(iVideo);
}






//******************************************************************************
// ******************************************************************************
/* Function:
    void _USB_DEVICE_VIDEO_TransferIRPCallBack ( USB_DEVICE_IRP * irp )

  Summary:
    This function is called by the USB driver when an IRP is completed. 

  Description:
    This function is called by the USB driver when an IRP is completed.

  Remarks:
    This is local function and should not be called directly by the application.
*/
//******************************************************************************

void _USB_DEVICE_VIDEO_TransferIRPCallBack ( USB_DEVICE_IRP * irp )
{
    _USB_DEVICE_VIDEO_TransferCompleteCallback(irp); 
}


void _USB_DEVICE_VIDEO_TransferAbortPrevent(USB_DEVICE_IRP * irp)
{
    USB_DEVICE_VIDEO_EVENT_DATA_READ_COMPLETE readEventData;
    USB_DEVICE_VIDEO_INSTANCE *thisVideoInstance;
    USB_DEVICE_VIDEO_INDEX iVideo;
    uint8_t cnt;
    USB_DEVICE_VIDEO_IRP_DATA *videoIrpData;
    USB_DEVICE_VIDEO_EVENT event;

    /* Initialize Status */
    readEventData.status = USB_DEVICE_VIDEO_RESULT_ERROR;
    
    /* The user data field of the IRP contains the array index of the Video IRP
     * in the lower 16 bits and the and the unique Identifier in the upper 16 bits.
     * Mask the upper 16 bits to the Video IRP index associated with this IRP */

    cnt = (uint8_t)(irp->userData & 0xFFFF);

    /* Get a pointer to the Video IRP data */
    videoIrpData = &gUSBDeviceVideoIrpData[cnt];

    /* Retrieve Video Instance Number */
    iVideo = videoIrpData->iVideo;

    /* Get a pointer to the Video Instance */
    thisVideoInstance = &gUsbDeviceVideoInstance[iVideo];

     /* Populate the event handler for this transfer. This was stored in the
      * userData field of the IRP when the IRP was submitted. */
    readEventData.handle = ( USB_DEVICE_VIDEO_TRANSFER_HANDLE ) irp->userData;

    /* Update the size written */
    readEventData.length = irp->size;
    
    
    /* Get transfer status */
    if ((irp->status == USB_DEVICE_IRP_STATUS_COMPLETED) 
        || (irp->status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
    {
        /* Transfer completed successfully */
        readEventData.status = USB_DEVICE_VIDEO_RESULT_OK; 
    }
   
    /* Update Interface Number */ 
    readEventData.interfaceNum = videoIrpData->interfaceNum;

    if (videoIrpData->direction == USB_DEVICE_VIDEO_READ)
    {
        event = USB_DEVICE_VIDEO_EVENT_READ_COMPLETE;
        thisVideoInstance->currentQSizeRead--;

    }
    else
    {
        event = USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE;
        thisVideoInstance->currentQSizeWrite--; 
    }

    /* Send an event to the application */ 
    if ((thisVideoInstance->appEventCallBack)
        && (readEventData.status == USB_DEVICE_VIDEO_RESULT_OK))
    {
        thisVideoInstance->appEventCallBack(iVideo,
                                            event,
                                            &readEventData,
                                            thisVideoInstance->userData);
    }
}
void _USB_DEVICE_VIDEO_TransferAbortAllow(USB_DEVICE_IRP * irp)
{
    USB_DEVICE_VIDEO_EVENT_DATA_READ_COMPLETE readEventData;
    USB_DEVICE_VIDEO_INSTANCE *thisVideoInstance;
    USB_DEVICE_VIDEO_INDEX iVideo;
    uint8_t cnt;
    USB_DEVICE_VIDEO_IRP_DATA *videoIrpData;
    USB_DEVICE_VIDEO_EVENT event;

    /* Initialize Status */
    readEventData.status = USB_DEVICE_VIDEO_RESULT_ERROR;
    
    /* The user data field of the IRP contains the array index of the Video IRP
     * in the lower 16 bits and the and the unique Identifier in the upper 16 bits.
     * Mask the upper 16 bits to the Video IRP index associated with this IRP */

    cnt = (uint8_t)(irp->userData & 0xFFFF);

    /* Get a pointer to the Video IRP data */
    videoIrpData = &gUSBDeviceVideoIrpData[cnt];

    /* Retrieve Video Instance Number */
    iVideo = videoIrpData->iVideo;

    /* Get a pointer to the Video Instance */
    thisVideoInstance = &gUsbDeviceVideoInstance[iVideo];

     /* Populate the event handler for this transfer. This was stored in the
      * userData field of the IRP when the IRP was submitted. */
    readEventData.handle = ( USB_DEVICE_VIDEO_TRANSFER_HANDLE ) irp->userData;

    /* Update the size written */
    readEventData.length = irp->size;
    
    
    /* Get transfer status */
    if ((irp->status == USB_DEVICE_IRP_STATUS_COMPLETED) 
        || (irp->status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
    {
        /* Transfer completed successfully */
        readEventData.status = USB_DEVICE_VIDEO_RESULT_OK; 
    }
    else if (irp->status == USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT)
    {
        /* Transfer cancelled due to Endpoint Halt */
        readEventData.status = USB_DEVICE_VIDEO_RESULT_ERROR_ENDPOINT_HALTED; 
    }
    else if (irp->status == USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST)
    {
        /* Transfer Cancelled by Host (Host sent a Clear feature )*/
        readEventData.status = USB_DEVICE_VIDEO_RESULT_ERROR_TERMINATED_BY_HOST; 
    }
    else
    {
        /* Transfer was not completed successfully */
        readEventData.status = USB_DEVICE_VIDEO_RESULT_ERROR; 
    }

    /* Update Interface Number */ 
    readEventData.interfaceNum = videoIrpData->interfaceNum;

    if (videoIrpData->direction == USB_DEVICE_VIDEO_READ)
    {
        event = USB_DEVICE_VIDEO_EVENT_READ_COMPLETE;
        thisVideoInstance->currentQSizeRead--;

    }
    else
    {
        event = USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE;
        thisVideoInstance->currentQSizeWrite--; 
    }

    /* Send an event to the application */ 
    if (thisVideoInstance->appEventCallBack)
    {
        thisVideoInstance->appEventCallBack(iVideo,
                                            event,
                                            &readEventData,
                                            thisVideoInstance->userData);
    }
    
}
// ******************************************************************************
/* Function:
    void _USB_DEVICE_VIDEO_IRPCancelAll(USB_DEVICE_VIDEO_INDEX iVideo)

  Summary:
    This function cancels all pending IRPs on a Video Function driver instance. 

  Description:
    This function cancels all pending IRPs on a Video Function driver instance. 

  Remarks:
    This is local function and should not be called directly by the application.
*/
//******************************************************************************
void _USB_DEVICE_VIDEO_IRPCancelAll(USB_DEVICE_VIDEO_INDEX iVideo)
{
    USB_DEVICE_VIDEO_INSTANCE * thisVideoInstance =  &gUsbDeviceVideoInstance[iVideo];
    uint8_t noOfStreamingInterface; 
    uint8_t count;
    uint8_t activeInterfaceSetting;
    USB_DEVICE_VIDEO_STREAMING_INTERFACE* streamingInterface;
    uint8_t endpointAddress;

    /* Find out how many Streaming interfaces are supported by this Video Instance*/
    noOfStreamingInterface = thisVideoInstance->infCollection.numStreamingInf;

    /* We have to cancel IRPs submitted for all of the streams */
    for (count=0; count < noOfStreamingInterface; count++)
    {
        /* Get pointer current streaming interface */
        streamingInterface = &thisVideoInstance->infCollection.streamInf[count];

        /* Find out active alternate setting*/
        activeInterfaceSetting = streamingInterface->activeSetting;

        if (streamingInterface->alterntSetting[activeInterfaceSetting].numEndPoints)
        {
          /* Retrieve Isochronous Data Endpoint address */
          endpointAddress = streamingInterface->alterntSetting[activeInterfaceSetting].isoDataEp.epAddr;

          /* Double check if this not Endpoint 0*/
          if (endpointAddress !=0)
          {
            /* Cancel all IRPs on this Endpoint */
//            USB_DEVICE_IRPCancelAll(gUsbDeviceVideoInstance[iVideo].devLayerHandle,  endpointAddress);
          }
          if (streamingInterface->alterntSetting[activeInterfaceSetting].numEndPoints == 2)
          {
              /* If there are Two Endpoints, second Endpoint is a Sync Endpoint */
              endpointAddress = streamingInterface->alterntSetting[activeInterfaceSetting].isoSyncEp.epAddr;

              /* Double if this not Endpoint 0*/
              if (endpointAddress !=0)
              {
                /* Cancel all IRPs on this Endpoint */
              //  USB_DEVICE_IRPCancelAll(gUsbDeviceVideoInstance[iVideo].devLayerHandle,  endpointAddress);
              }
          }
        }
        
    }
}


// *****************************************************************************





/*******************************************************************************
 End of File
 */
