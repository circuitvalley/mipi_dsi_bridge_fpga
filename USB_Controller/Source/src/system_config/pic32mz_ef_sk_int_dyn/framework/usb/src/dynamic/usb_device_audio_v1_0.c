/*******************************************************************************
 USB Audio Class Function Driver

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_audio.c

  Summary:
    USB audio class function driver.

  Description:
    USB audio class function driver.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
 Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

 Microchip licenses to you the right to use, modify, copy and distribute
 Software only when embedded on a Microchip microcontroller or digital signal
 controller that is integrated into your product or third party product
 (pursuant to the sublicense terms in the accompanying license agreement).

 You should refer to the license agreement accompanying this Software for
 additional information regarding your rights and obligations.

 SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
#include "usb/usb_device_audio_v1_0.h"
#include "usb/src/usb_device_audio_local.h"
#include "system/debug/sys_debug.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
/* Function:
    void _USB_DEVICE_AUDIO_IRPCancelAll(USB_DEVICE_AUDIO_INDEX iAudio )
   Summary:
    This function cancels all pending Read and Write requests placed to
    USB Device Audio function driver.

   Description:
    This function cancels all pending Read and Write requests placed to
    USB Device Audio function driver.
   Precondition:
    The function driver should have been configured.
   Parameters:
    iAudio - USB Device Audio Function Driver instance.

   Returns:
    None
   Example:

   Remarks:
     None
 */
void _USB_DEVICE_AUDIO_IRPCancelAll(USB_DEVICE_AUDIO_INDEX iAudio );

// *****************************************************************************
/* AUDIO Device function driver structure

  Summary:
    Defines the function driver structure required for the device layer.

  Description:
    This data type defines the function driver structure required for the
    device layer.

  Remarks:
    This structure is private to the USB stack.
 */
const USB_DEVICE_FUNCTION_DRIVER audioFunctionDriver =
{

    /* AUDIO init function */
    .initializeByDescriptor             = &_USB_DEVICE_AUDIO_Initialize ,

    /* AUDIO de-init function */
    .deInitialize           = &_USB_DEVICE_AUDIO_Deinitialize ,

    /* AUDIO set-up packet handler */
    .controlTransferNotification     = &_USB_DEVICE_AUDIO_ControlTransferHandler,

    /* AUDIO tasks function */
    .tasks                  = NULL,

    /* Audio Global Initialize function */
    .globalInitialize = _USB_DEVICE_AUDIO_GlobalInitialize
        
};


//******************************************************************************
//******************************************************************************


// ******************************************************************************
/* Function:
   void _USB_DEVICE_AUDIO_ControlTransferHandler
   (
       SYS_MODULE_INDEX iAudio,
       USB_DEVICE_EVENT controlEvent,
       USB_SETUP_PACKET * setupRequest
   )

  Summary:
    Audio control transfer handler.

  Description:
    Audio control transfer handler. This is the callback the device layer calls
    when there is a SETUP packet that is targeted to this particular instance
    of Audio.

  Returns:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_DEVICE_AUDIO_ControlTransferHandler
(
    SYS_MODULE_INDEX iAudio,
    USB_DEVICE_EVENT controlEvent,
    USB_SETUP_PACKET * setupRequest
)
{
    /* Obtain pointer to the Audio Instance that is being addressed*/
    USB_DEVICE_AUDIO_INSTANCE *thisAudioInstance = &gUsbDeviceAudioInstance[iAudio];

    /* check the validity of the function driver index */
    if ( USB_DEVICE_AUDIO_INSTANCES_NUMBER <= iAudio )
    {
        /* invalid handle */
        SYS_ASSERT (false, "invalid Audio Index");
    }

    /* Check Control Event  */
    switch(controlEvent)
    {
        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:
            /* A new SETUP packet received from Host*/
            _USB_DEVICE_AUDIO_SetupPacketHandler(iAudio,setupRequest);
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
            /* A Control Transfer data stage completed. Notify the application */
            thisAudioInstance->appEventCallBack
            (
                iAudio,
                USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED,
                NULL,
                0
            );        
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_SENT:
            /* The Control Transfer data was sent the Host. Notify the application */
            thisAudioInstance->appEventCallBack
            (
                iAudio,
                USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_SENT,
                NULL,
                0
            );
            break;
        default:
            break;
    }
}/* End of function _USB_DEVICE_AUDIO_ControlTransferHandler */ 


// ******************************************************************************
/* Function:
   void _USB_DEVICE_AUDIO_SetupPacketHandler
   (
       USB_DEVICE_AUDIO_INDEX iAudio ,
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
void _USB_DEVICE_AUDIO_SetupPacketHandler 
(
    USB_DEVICE_AUDIO_INDEX iAudio , 
    USB_SETUP_PACKET * setupPkt
)
{
    uint8_t audioControlInterfaceId;
    uint8_t interfaceId;
    uint8_t curAlternateSetting;
    uint8_t prevAlternateSetting;
    uint8_t streamIntfcIndex;
    uint8_t noOfEndpoints;
    USB_ENDPOINT ep; 
    uint16_t maxPacketSize;
    USB_DEVICE_HANDLE usbDeviceHandle;
    USB_DEVICE_AUDIO_EVENT event;
    USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED interfaceInfo;
    USB_ERROR endpointEnableResult; 
    USB_DEVICE_AUDIO_STREAMING_INTERFACE_ALTERNATE_SETTING *pCurAlternateStng;
    USB_DEVICE_AUDIO_STREAMING_INTERFACE_ALTERNATE_SETTING *pPrevAlternateStng;
    USB_DEVICE_AUDIO_STREAMING_INTERFACE *pStreamingInterface;

    /* Obtain pointer to the Audio Instance that is being addressed*/
    USB_DEVICE_AUDIO_INSTANCE* audioInstance = &gUsbDeviceAudioInstance[iAudio];

    /* Obtain pointer to the Audio Interface collection*/
    USB_DEVICE_AUDIO_INTERFACE_COLLECTION *curInfCollection =
            &(audioInstance->infCollection);

    /* Get the Device Layer handle */
    usbDeviceHandle = gUsbDeviceAudioInstance[iAudio].devLayerHandle;
    
    /* Check if the request is a standard interface request*/
    if (  (setupPkt->RequestType == USB_SETUP_REQUEST_TYPE_STANDARD)
       && (setupPkt->Recipient == USB_SETUP_REQUEST_RECIPIENT_INTERFACE))

    {
        /* We have received Standard Set request */

        /* Retrieve interface number from the Setup packet */
        interfaceId = setupPkt->bIntfID;

        /* Retrieve audio Control interface number*/
        audioControlInterfaceId = curInfCollection->bControlInterfaceNum;
        
        switch(setupPkt->bRequest)
        {
            case USB_REQUEST_SET_INTERFACE:
                curAlternateSetting = setupPkt->bAltID;
                if (interfaceId == audioControlInterfaceId)
                {
                    /*SET INTERFACE command was received to Audio Control
                      Interface */
                    curInfCollection->bControlAltSetng = curAlternateSetting;
                }
                else
                {
                    /*An Audio Streaming interface has received SET INTERFACE
                      command */
                    streamIntfcIndex = interfaceId - audioControlInterfaceId - 1;

                    /* Get pointer to the current audio streaming interface */
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
                            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "_USB_DEVICE_AUDIO_SetupPacketHandler():  Endpoint not Enabled");
                        }

                        if (noOfEndpoints == 2)
                        {
                            /* If number of Endpoints is Two, then it is sure
                             * that this alternate setting reports a Isochronous
                             * Sync Endpoint. Enable the Sync Endpoint. */
                        }

                        /* Change Audio Instance object state to Initialized */ 
                        pStreamingInterface->state
                                    = USB_DEVICE_AUDIO_STRMNG_INTFC_INITIALIZED;
                    }
                    else /* alternateSetting  = 0 */ 
                    {
                        if (pStreamingInterface->state
                                   == USB_DEVICE_AUDIO_STRMNG_INTFC_INITIALIZED)
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
                                = USB_DEVICE_AUDIO_STRMNG_INTFC_NOT_INITIALIZED;
                            
                        }  
                    }

                    /* Remember the new alternate setting received */
                    curInfCollection->streamInf[streamIntfcIndex].activeSetting 
                    = curAlternateSetting;


                    /* Notify application about the SET_INTERFACE request */
                    interfaceInfo.interfaceNumber = interfaceId;
                    interfaceInfo.interfaceAlternateSetting = curAlternateSetting;
                    audioInstance->appEventCallBack
                    (
                        iAudio,
                        USB_DEVICE_AUDIO_EVENT_INTERFACE_SETTING_CHANGED,
                        &interfaceInfo,
                        0
                    );
                }
                /* Send an Acknowledgement to the Host */ 
                USB_DEVICE_ControlStatus( usbDeviceHandle,
                                USB_DEVICE_CONTROL_STATUS_OK);
            break;
            case USB_REQUEST_GET_INTERFACE:
                if (interfaceId == audioControlInterfaceId)
                {
                    curAlternateSetting = curInfCollection->bControlAltSetng;
                }
                else
                {
                    streamIntfcIndex = interfaceId - audioControlInterfaceId - 1;
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
        /* We have received a Audio Class specific Interface request or
			Audio Class specific Endpoint request */
        switch (setupPkt->bRequest)
        {
            case USB_AUDIO_CS_SET_CUR:
                event = USB_DEVICE_AUDIO_EVENT_CONTROL_SET_CUR;
                break;
            case USB_AUDIO_CS_GET_CUR:
                event = USB_DEVICE_AUDIO_EVENT_CONTROL_GET_CUR;
                break;
            case USB_AUDIO_CS_SET_MIN:
                event = USB_DEVICE_AUDIO_EVENT_CONTROL_SET_MIN;
                break;
            case USB_AUDIO_CS_GET_MIN:
                event = USB_DEVICE_AUDIO_EVENT_CONTROL_GET_MIN;
                break;
            case USB_AUDIO_CS_SET_MAX:
                event = USB_DEVICE_AUDIO_EVENT_CONTROL_SET_MAX;
                break;
            case USB_AUDIO_CS_GET_MAX:
                event = USB_DEVICE_AUDIO_EVENT_CONTROL_GET_MAX;
                break;
            case USB_AUDIO_CS_SET_RES:
                event = USB_DEVICE_AUDIO_EVENT_CONTROL_SET_RES;
                break;
            case USB_AUDIO_CS_GET_RES:
                event = USB_DEVICE_AUDIO_EVENT_CONTROL_GET_RES;
                break;
            case USB_AUDIO_CS_SET_MEM:
                event = USB_DEVICE_AUDIO_EVENT_ENTITY_SET_MEM;
                break;
            case USB_AUDIO_CS_GET_MEM:
                event = USB_DEVICE_AUDIO_EVENT_ENTITY_GET_MEM;
                break;
            case USB_AUDIO_CS_GET_STAT:
                event = USB_DEVICE_AUDIO_EVENT_ENTITY_GET_STAT;
                break;
            default:
            /* Unknown request. Stall the request */
                event = 0;
                USB_DEVICE_ControlStatus(audioInstance->devLayerHandle,
                                               USB_DEVICE_CONTROL_STATUS_ERROR);
        }
        if (( audioInstance->appEventCallBack) && (event))
        {
            /* Inform the application about the request */
            /* The application needs to handle both EP and entity specific 
               requests */
           audioInstance->appEventCallBack ( iAudio, event, setupPkt, 0);
        }        
    }/* End of else if */ 
}/* End of function _USB_DEVICE_AUDIO_SetupPacketHandler */ 


//******************************************************************************


// ******************************************************************************
/* Function:
    void _USB_DEVICE_AUDIO_GlobalInitialize ( void )

  Summary:
    This function initializes resources required common to all instances of AUDIO
    function driver.

  Description:
    This function initializes resources common to all instances of AUDIO function
    driver. This function is called by the USB Device layer during Initialization.

  Remarks:
    This is local function and should not be called directly by the application.
*/
void _USB_DEVICE_AUDIO_GlobalInitialize (void)
{
    OSAL_RESULT osal_err;

    /* Create Mutex for CDC IRP objects if not created already */
    if (gUSBDeviceAudioCommonDataObj.isMutexAudioIrpInitialized == false)
    {
        /* This means that mutexes where not created. Create them. */
        osal_err = OSAL_MUTEX_Create(&gUSBDeviceAudioCommonDataObj.mutexAUDIOIRP);

        if(osal_err != OSAL_RESULT_TRUE)
        {
            /*do not proceed lock was not created, let user know about error*/
            return;
        }

         /* Set this flag so that global mutexes get allocated only once */
         gUSBDeviceAudioCommonDataObj.isMutexAudioIrpInitialized = true;
    }
}

// ******************************************************************************
/* Function:
    void _USB_DEVICE_AUDIO_Initialize 
    (
        SYS_MODULE_INDEX iAudio, 
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
void _USB_DEVICE_AUDIO_Initialize 
(
    SYS_MODULE_INDEX iAudio, 
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
    USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR *pCsInfDesc;

    USB_DEVICE_AUDIO_INIT * audioInit;
    uint8_t epAddressAndDirection;
    uint16_t maxPacketSize;
    uint8_t cnt;
    uint8_t audioControlInterfaceId;
    uint8_t strmIntrfcIndex;
    USB_DEVICE_AUDIO_STREAMING_INTERFACE_ALTERNATE_SETTING *curAlternateStng;
    USB_DEVICE_AUDIO_INSTANCE * audioInstance = &gUsbDeviceAudioInstance[iAudio];
    USB_DEVICE_AUDIO_INTERFACE_COLLECTION *curInfCollection =
            &(audioInstance->infCollection);
    audioInit = ((USB_DEVICE_AUDIO_INIT *)funcDriverInit);
    audioInstance->queueSizeRead = audioInit->queueSizeRead;
    audioInstance->queueSizeWrite = audioInit->queueSizeWrite;
    audioInstance->currentQSizeRead = 0;
    audioInstance->currentQSizeWrite = 0; 

     /* Check the type of descriptor passed by device layer */
    switch ( descriptorType )
    { 
        case USB_DESCRIPTOR_INTERFACE: 
            /* Interface Descriptor Received */ 
            
            /* Save the device handle to the corresponding Audio Instance object. */ 
            audioInstance->devLayerHandle = usbDeviceHandle;

            /* Save the Audio Index to the corresponding Audio Instance object. */ 
            audioInstance->audioIndex = iAudio;

            pStdInfDesc = (USB_INTERFACE_DESCRIPTOR*) pDescriptor;

            /* Check if it is USB audio interface descriptor */
            if (pStdInfDesc->bInterfaceClass == USB_AUDIO_CLASS_CODE)
            {
                /* Check if it is an Audio Control Interface descriptor */
                switch (pStdInfDesc->bInterfaceSubClass)
                {
                    case USB_AUDIO_AUDIOCONTROL:
                        audioInstance->flags.allFlags = 0;
                        /* The program control reached here means that the device
                           layer has detected an Standard Audio Control interface 
                           descriptor. Now we have to save all the fields present 
                           in the Standard Audio Control interface descriptor to 
                           the corresponding Audio Instance Object. */

                        /*Save the Interface number to the corresponding Audio 
                            Instance Object.*/
                        curInfCollection->bControlInterfaceNum 
                                    = pStdInfDesc->bInterfaceNumber;

                        /* Save the no of endpoints present in the Audio Control 
                        Interface. If there are endpoints present as part of AC 
                        interface descriptor that must be an Interrupt endpoint.*/
                        curInfCollection->isIntEpExists = pStdInfDesc->bNumEndPoints;
                     break;
                
                
                    case USB_AUDIO_AUDIOSTREAMING:
                        /* We have received an Audio Streaming Interface descriptor. 
                           Save the interface number to an array of Audio streaming 
                           interfaces. For an audio function the interface numbers
                           starts from Audio Control Interface ID.*/

                        /* Save number of endpoints present in the streaming interface */ 
                        audioControlInterfaceId 
                                    = curInfCollection-> bControlInterfaceNum;
                        /* Find out audio streaming interface array index. */ 
                        strmIntrfcIndex = interfaceNumber-audioControlInterfaceId-1;

                        /* Save no of endpoints present in the streaming interface */
                        curAlternateStng 
                        = &(audioInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting]); 
                        curAlternateStng->numEndPoints = pStdInfDesc->bNumEndPoints;
                    break;
                    default :
                        break; 
                }
            }
        break; /* End of case USB_DESCRIPTOR_INTERFACE: */ 
    
        /* Class Specific Audio Control Interface Descriptor. */ 
        case USB_AUDIO_CS_INTERFACE:
            /* If we have already received Audio Control Interface related
            descriptors then ignore all further Class specific descriptors */ 
            if (audioInstance->flags.audioControlInterfaceReady ==  true)
                return;
            pCsInfDesc = (USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR*) pDescriptor;

            /* Check if this Class specific descriptor belongs to an AC Interface*/
            if (interfaceNumber == audioInstance->infCollection.bControlInterfaceNum)
            {
                if  (USB_AUDIO_HEADER == pCsInfDesc->bDescriptorSubtype)
                {
                    SYS_ASSERT((pCsInfDesc->bInCollection != USB_DEVICE_AUDIO_MAX_STREAMING_INTERFACES ),
                        "Maximum number of streaming interfaces defined does not match descriptor value");

                    /* Save number of audio steaming interfaces */ 
                    audioInstance->infCollection.numStreamingInf = pCsInfDesc->bInCollection;

                    /* Save interface numbers of the all available audio streaming and Midi interfaces. */
                    for (cnt = 0; cnt < USB_DEVICE_AUDIO_MAX_STREAMING_INTERFACES; cnt++ )
                    {
                        audioInstance->infCollection.streamInf[cnt].interfaceNum
                            = *((uint8_t *)&(pCsInfDesc->bInCollection) + cnt + 1);
                    }

                    /* Save audio specification number */ 
                    audioInstance->infCollection.bcdADC = pCsInfDesc->bcdADC;
                    audioInstance->flags.audioControlInterfaceReady = true;
                } /* End of "if  (USB_AUDIO_HEADER == pCsInfDesc->bDescriptorSubtype)" */
            } /* End of "if (interfaceNumber == audioInstance[iAudio].infCollection.bControlInterfaceNum)" */
        break; /* End of "case DEVICE_AUDIO_CS_INTERFACE" */

        case USB_DESCRIPTOR_ENDPOINT:
            /* We have received an Endpoint descriptor from device Layer*/
            pEPDesc = ( USB_ENDPOINT_DESCRIPTOR* ) pDescriptor;
            audioControlInterfaceId = audioInstance->infCollection.bControlInterfaceNum;
            strmIntrfcIndex = interfaceNumber - audioControlInterfaceId - 1;
       
            /* Check if this Endpoint belongs to an audio Streaming interface */ 
            if ((interfaceNumber == audioInstance->infCollection.streamInf[strmIntrfcIndex].interfaceNum))
            {
                /* Save the ep address */
                epAddressAndDirection = pEPDesc->bEndpointAddress;

                /* Save max packet size */
                maxPacketSize = ( ( USB_ENDPOINT_DESCRIPTOR* ) pDescriptor )->wMaxPacketSize;
       
                if (pEPDesc->usageType == USB_USAGE_DATA_ENDPOINT)
                {
                    /* Save ep address to the data interface */
                    audioInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting].isoDataEp.epAddr = epAddressAndDirection;

                    /* Save max packet size to the data interface */
                    audioInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting].isoDataEp.epMaxPacketSize = maxPacketSize;
                }
                else if (pEPDesc->usageType == USB_USAGE_FEEDBACK_ENDPOINT)
                {
                    /* Save ep address to the Sync interface */
                    audioInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting].isoSyncEp.epAddr = epAddressAndDirection;

                    /* Save max packet size to the Sync interface */
                    audioInstance->infCollection.streamInf[strmIntrfcIndex].alterntSetting[alternateSetting].isoSyncEp.epMaxPacketSize = maxPacketSize;
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
    void _USB_DEVICE_AUDIO_Deinitialize ( SYS_MODULE_INDEX iAudio )

  Summary:
    This function is called by the device layer when Audio Device is detached 
    from the Host. 

  Description:
    This function is called by the device layer when Audio Device is detached 
    from the Host. 

  Remarks:
    This is local function and should not be called directly by the application.
*/
//******************************************************************************
void _USB_DEVICE_AUDIO_Deinitialize ( SYS_MODULE_INDEX iAudio )
{
    /* Cancel all IRPs on the owned endpoints and then
     * disable the endpoint */

    if(iAudio >= USB_DEVICE_AUDIO_INSTANCES_NUMBER)
    {
        SYS_ASSERT(false," Invalid instance");
        return;
    }
   _USB_DEVICE_AUDIO_IRPCancelAll(iAudio);
}






//******************************************************************************
// ******************************************************************************
/* Function:
    void _USB_DEVICE_AUDIO_TransferIRPCallBack ( USB_DEVICE_IRP * irp )

  Summary:
    This function is called by the USB driver when an IRP is completed. 

  Description:
    This function is called by the USB driver when an IRP is completed.

  Remarks:
    This is local function and should not be called directly by the application.
*/
//******************************************************************************

void _USB_DEVICE_AUDIO_TransferIRPCallBack ( USB_DEVICE_IRP * irp )
{
    _USB_DEVICE_AUDIO_TransferCompleteCallback(irp); 
}


void _USB_DEVICE_AUDIO_TransferAbortPrevent(USB_DEVICE_IRP * irp)
{
    USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE readEventData;
    USB_DEVICE_AUDIO_INSTANCE *thisAudioInstance;
    USB_DEVICE_AUDIO_INDEX iAudio;
    uint8_t cnt;
    USB_DEVICE_AUDIO_IRP_DATA *audioIrpData;
    USB_DEVICE_AUDIO_EVENT event;

    /* Initialize Status */
    readEventData.status = USB_DEVICE_AUDIO_RESULT_ERROR;
    
    /* The user data field of the IRP contains the array index of the Audio IRP
     * in the lower 16 bits and the and the unique Identifier in the upper 16 bits.
     * Mask the upper 16 bits to the Audio IRP index associated with this IRP */

    cnt = (uint8_t)(irp->userData & 0xFFFF);

    /* Get a pointer to the Audio IRP data */
    audioIrpData = &gUSBDeviceAudioIrpData[cnt];

    /* Retrieve Audio Instance Number */
    iAudio = audioIrpData->iAudio;

    /* Get a pointer to the Audio Instance */
    thisAudioInstance = &gUsbDeviceAudioInstance[iAudio];

     /* Populate the event handler for this transfer. This was stored in the
      * userData field of the IRP when the IRP was submitted. */
    readEventData.handle = ( USB_DEVICE_AUDIO_TRANSFER_HANDLE ) irp->userData;

    /* Update the size written */
    readEventData.length = irp->size;
    
    
    /* Get transfer status */
    if ((irp->status == USB_DEVICE_IRP_STATUS_COMPLETED) 
        || (irp->status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
    {
        /* Transfer completed successfully */
        readEventData.status = USB_DEVICE_AUDIO_RESULT_OK; 
    }
   
    /* Update Interface Number */ 
    readEventData.interfaceNum = audioIrpData->interfaceNum;

    if (audioIrpData->direction == USB_DEVICE_AUDIO_READ)
    {
        event = USB_DEVICE_AUDIO_EVENT_READ_COMPLETE;
        thisAudioInstance->currentQSizeRead--;

    }
    else
    {
        event = USB_DEVICE_AUDIO_EVENT_WRITE_COMPLETE;
        thisAudioInstance->currentQSizeWrite--; 
    }

    /* Send an event to the application */ 
    if ((thisAudioInstance->appEventCallBack)
        && (readEventData.status == USB_DEVICE_AUDIO_RESULT_OK))
    {
        thisAudioInstance->appEventCallBack(iAudio,
                                            event,
                                            &readEventData,
                                            thisAudioInstance->userData);
    }
}
void _USB_DEVICE_AUDIO_TransferAbortAllow(USB_DEVICE_IRP * irp)
{
    USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE readEventData;
    USB_DEVICE_AUDIO_INSTANCE *thisAudioInstance;
    USB_DEVICE_AUDIO_INDEX iAudio;
    uint8_t cnt;
    USB_DEVICE_AUDIO_IRP_DATA *audioIrpData;
    USB_DEVICE_AUDIO_EVENT event;

    /* Initialize Status */
    readEventData.status = USB_DEVICE_AUDIO_RESULT_ERROR;
    
    /* The user data field of the IRP contains the array index of the Audio IRP
     * in the lower 16 bits and the and the unique Identifier in the upper 16 bits.
     * Mask the upper 16 bits to the Audio IRP index associated with this IRP */

    cnt = (uint8_t)(irp->userData & 0xFFFF);

    /* Get a pointer to the Audio IRP data */
    audioIrpData = &gUSBDeviceAudioIrpData[cnt];

    /* Retrieve Audio Instance Number */
    iAudio = audioIrpData->iAudio;

    /* Get a pointer to the Audio Instance */
    thisAudioInstance = &gUsbDeviceAudioInstance[iAudio];

     /* Populate the event handler for this transfer. This was stored in the
      * userData field of the IRP when the IRP was submitted. */
    readEventData.handle = ( USB_DEVICE_AUDIO_TRANSFER_HANDLE ) irp->userData;

    /* Update the size written */
    readEventData.length = irp->size;
    
    
    /* Get transfer status */
    if ((irp->status == USB_DEVICE_IRP_STATUS_COMPLETED) 
        || (irp->status == USB_DEVICE_IRP_STATUS_COMPLETED_SHORT))
    {
        /* Transfer completed successfully */
        readEventData.status = USB_DEVICE_AUDIO_RESULT_OK; 
    }
    else if (irp->status == USB_DEVICE_IRP_STATUS_ABORTED_ENDPOINT_HALT)
    {
        /* Transfer cancelled due to Endpoint Halt */
        readEventData.status = USB_DEVICE_AUDIO_RESULT_ERROR_ENDPOINT_HALTED; 
    }
    else if (irp->status == USB_DEVICE_IRP_STATUS_TERMINATED_BY_HOST)
    {
        /* Transfer Cancelled by Host (Host sent a Clear feature )*/
        readEventData.status = USB_DEVICE_AUDIO_RESULT_ERROR_TERMINATED_BY_HOST; 
    }
    else
    {
        /* Transfer was not completed successfully */
        readEventData.status = USB_DEVICE_AUDIO_RESULT_ERROR; 
    }

    /* Update Interface Number */ 
    readEventData.interfaceNum = audioIrpData->interfaceNum;

    if (audioIrpData->direction == USB_DEVICE_AUDIO_READ)
    {
        event = USB_DEVICE_AUDIO_EVENT_READ_COMPLETE;
        thisAudioInstance->currentQSizeRead--;

    }
    else
    {
        event = USB_DEVICE_AUDIO_EVENT_WRITE_COMPLETE;
        thisAudioInstance->currentQSizeWrite--; 
    }

    /* Send an event to the application */ 
    if (thisAudioInstance->appEventCallBack)
    {
        thisAudioInstance->appEventCallBack(iAudio,
                                            event,
                                            &readEventData,
                                            thisAudioInstance->userData);
    }
    
}
// ******************************************************************************
/* Function:
    void _USB_DEVICE_AUDIO_IRPCancelAll(USB_DEVICE_AUDIO_INDEX iAudio)

  Summary:
    This function cancels all pending IRPs on a Audio Function driver instance. 

  Description:
    This function cancels all pending IRPs on a Audio Function driver instance. 

  Remarks:
    This is local function and should not be called directly by the application.
*/
//******************************************************************************
void _USB_DEVICE_AUDIO_IRPCancelAll(USB_DEVICE_AUDIO_INDEX iAudio)
{
    USB_DEVICE_AUDIO_INSTANCE * thisAudioInstance = &gUsbDeviceAudioInstance[iAudio];
    uint8_t noOfStreamingInterface; 
    uint8_t count;
    uint8_t activeInterfaceSetting;
    USB_DEVICE_AUDIO_STREAMING_INTERFACE* streamingInterface;
    uint8_t endpointAddress;

    /* Find out how many Streaming interfaces are supported by this Audio Instance*/
    noOfStreamingInterface = thisAudioInstance->infCollection.numStreamingInf;

    /* We have to cancel IRPs submitted for all of the streams */
    for (count=0; count < noOfStreamingInterface; count++)
    {
        /* Get pointer current streaming interface */
        streamingInterface = &thisAudioInstance->infCollection.streamInf[count];

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
            USB_DEVICE_IRPCancelAll(gUsbDeviceAudioInstance[iAudio].devLayerHandle,  endpointAddress);
          }
          if (streamingInterface->alterntSetting[activeInterfaceSetting].numEndPoints == 2)
          {
              /* If there are Two Endpoints, second Endpoint is a Sync Endpoint */
              endpointAddress = streamingInterface->alterntSetting[activeInterfaceSetting].isoSyncEp.epAddr;

              /* Double if this not Endpoint 0*/
              if (endpointAddress !=0)
              {
                /* Cancel all IRPs on this Endpoint */
                USB_DEVICE_IRPCancelAll(gUsbDeviceAudioInstance[iAudio].devLayerHandle,  endpointAddress);
              }
          }
        }
        
    }
}


// *****************************************************************************





/*******************************************************************************
 End of File
 */
