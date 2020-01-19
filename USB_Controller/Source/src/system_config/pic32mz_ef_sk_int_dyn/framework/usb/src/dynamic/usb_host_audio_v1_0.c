/*******************************************************************************
 USB HOST Audio Class Driver

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_audio.c

  Summary:
    USB Audio Host class driver.

  Description:
    USB Audio Host class driver.
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

#include "system_config.h"
#include "usb/usb_host_client_driver.h"
#include "usb/usb_host.h"
#include "usb/src/usb_host_local.h"
#include "usb/src/usb_host_audio_local.h"
#include "usb/usb_audio_v1_0.h"
#include "usb/usb_host_audio_v1_0.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Audio Host Instance structure

  Summary:
    Defines the Audio instance(s).

  Description:
    This data type defines the Audio instance(s). The number of instances is
    defined by the application using USB_HOST_AUDIO_INSTANCES_NUMBER.

  Remarks:
    This structure is private to the Audio.
 */
USB_HOST_AUDIO_V1_INSTANCE  gUSBHostAudioInstance[USB_HOST_AUDIO_V1_INSTANCES_NUMBER];


// *****************************************************************************
/* Audio Host Common Object 

  Summary:
    Defines the Audio Host common object which is common to all instances of 
    Audio Host. 

  Description:
    Defines the Audio Host common object which is common to all instances of 
    Audio Host. 

  Remarks:
    This structure is private to the Audio Host Client driver.
 */
USB_HOST_AUDIO_V1_COMMON_OBJ  gUSBHostAudio1CommonObj; 


// *****************************************************************************
/* Audio v1.0 Host driver Interface structure

  Summary:
    Defines the Audio Host driver structure. 

  Description:
    Defines the Audio Host driver structure which is used by Host layer to
    communicate with the Audio v1.0 client driver. This structure should be 
    passed as the parameter in the TPL.  

  Remarks:
    None. 
 */
const USB_HOST_CLIENT_DRIVER gUSBHostAudioV1Driver =
{
    .initialize                 = _USB_HOST_AUDIO_V1_Initialize,
    .deinitialize               = _USB_HOST_AUDIO_V1_Deinitialize,
    .reinitialize               = _USB_HOST_AUDIO_V1_Reinitialize,
    .interfaceAssign            = _USB_HOST_AUDIO_V1_InterfaceAssign,
    .interfaceRelease           = _USB_HOST_AUDIO_V1_InterfaceRelease,
    .interfaceEventHandler      = _USB_HOST_AUDIO_V1_InterfaceEventHandler,
    .interfaceTasks             = _USB_HOST_AUDIO_V1_InterfaceTasks,
    .deviceEventHandler         = NULL,
    .deviceAssign               = NULL,
    .deviceEventHandler         = NULL,
    .deviceRelease              = NULL
 };


// *****************************************************************************
/* Function:
    void USB_HOST_AUDIO_V1_Initialize(void *init)

  Summary:
    This function is called when the Host Layer is initializing.

  Description:
    This function is called when the Host Layer is initializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/
void _USB_HOST_AUDIO_V1_Initialize (void *init)
{
    
}

void _USB_HOST_AUDIO_V1_Deinitialize(void)
{
    
}

void _USB_HOST_AUDIO_V1_Reinitialize (void * init)
{
    
}
/*************************************************************************/

/* Function:
   void USB_HOST_AUDIO_V1_InterfaceAssign
   (  
       USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
       USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
       size_t nInterfaces,
       uint8_t * descriptor
   )

  Summary:
    Initializes Audio Instance.  

  Description:
    This function is called by USB Host whenever an Audio v1.0 device is attached
    to the USB Bus.  

  Parameters:
     interfaces                           
     deviceObjHandle
     nInterfaces
     descriptor

  Returns:
    None
  
  Remarks:
    This is a local function and should be called directly by clients. 
*/
 void _USB_HOST_AUDIO_V1_InterfaceAssign
 (  
     USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
     USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
     size_t nInterfaces,
     uint8_t * descriptor
 )
 {
    int driverIndex;
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo = NULL;
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle  = interfaces[0];
    USB_HOST_ENDPOINT_DESCRIPTOR_QUERY endpointQuery;
    USB_HOST_INTERFACE_DESCRIPTOR_QUERY interfaceQuery; 
    USB_ENDPOINT_DESCRIPTOR * endpointDescriptor;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;
    USB_HOST_AUDIO_STREAM_SETTING* audioStream; 
    bool error = false;
    int strmIndex; 
    USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR* audioControlDesc;
    USB_AUDIO_CS_AS_INTERFACE_DESCRIPTOR* audioCsAsDescriptor; 
    USB_AUDIO_FORMAT_I_DESCRIPTOR_HEADER* audioFormatDesc; 
  
    /* This function is being called because an USB Audio v1.0 device was attached 
       and the driver has matched.*/ 
    
    interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR*)descriptor;  
    
    switch (interfaceDescriptor->bInterfaceSubClass)
    {
        case USB_AUDIO_AUDIOCONTROL:
            
            /* Check if there is a free Audio v1.0 object.*/
            for(driverIndex = 0 ; driverIndex < USB_HOST_AUDIO_V1_INSTANCES_NUMBER ; driverIndex++)
            {
                if (!gUSBHostAudioInstance[driverIndex].assigned)
                {
                    /* Found a free instance */
                    gUSBHostAudioInstance[driverIndex].assigned = true;
                    audioInstanceInfo = &gUSBHostAudioInstance[driverIndex];
                    audioInstanceInfo->index = driverIndex; 
                    break;
                }
            }
    
            if (audioInstanceInfo!= NULL)
            {
                /* Save these handles */
                audioInstanceInfo->deviceObjHandle = deviceObjHandle;
                audioInstanceInfo->acInterfaceHandle = interfaces[0];
                audioInstanceInfo->pAudioContorldescriptor = descriptor; 
            
                /* This means that we have found an Audio instance object and this 
                device can be processed. Open a control pipe to the device. */
                audioInstanceInfo->controlPipeHandle = USB_HOST_DeviceControlPipeOpen(deviceObjHandle);
           
                if(audioInstanceInfo->controlPipeHandle != USB_HOST_CONTROL_PIPE_HANDLE_INVALID)
                {
                    /* Get pointer to Audio Class Specific Descriptor */
                    audioControlDesc = (USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR*)((uint8_t *)descriptor +  ((USB_INTERFACE_DESCRIPTOR*)descriptor)->bLength);
             
                    /* Get Number of Audio Streaming interface in the attached Device */
                    audioInstanceInfo->nASInterfaces = audioControlDesc->bInCollection;
                    
                    /* Reset Audio Streaming Interfaces Counter. This variable will be 
                       incremented when an Audio Streaming interface is received. */
                    audioInstanceInfo->countASInterfaces = 0; 
             
                    /* Check if the Attached Audio Device exposes more streaming 
                    interfaces than user defined */  
                    if (audioInstanceInfo->nASInterfaces > USB_HOST_AUDIO_V1_STREAMING_INTERFACES_NUMBER)
                    {
                        SYS_DEBUG_MESSAGE  ( SYS_DEBUG_INFO ,
                        "USB_HOST_AUDIO_V1_InterfaceAssign():Attached Audio Device has more streaming interfaces than user expected" );
                    }
             
                    for (strmIndex = 0; strmIndex< audioInstanceInfo->nASInterfaces; strmIndex++)
                    {
                        /* Save Audio Streaming Interface Number */
                        audioInstanceInfo->streamInf[strmIndex].interfaceId = *((uint8_t*)&(audioControlDesc->bInCollection) + strmIndex + 1);
                
                        /* Set Alternate setting to Zero */
                        audioInstanceInfo->streamInf[strmIndex].activeInterfaceSetting = 0;
                    }
                    /* Save the pointer to Audio Instance for further processing  */
                    gUSBHostAudio1CommonObj.prevAudioInstance = audioInstanceInfo; 
                }
                else
                {
                    /* Control pipe could not be opened */
                    error = true;
                }
                
                if (error == true)
                {
                    /* Let the host know that this interface cannot be processed */
                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Could not find Audio object");
                    _USB_HOST_AUDIO_V1_InterfaceRelease(interfaceHandle);
                }
                return;
            }
                    
            break; 
        
        case USB_AUDIO_AUDIOSTREAMING:
            /* Get pointer to Audio Instance */
            audioInstanceInfo = gUSBHostAudio1CommonObj.prevAudioInstance; 
            
            /* Save Interface Handle */
            audioInstanceInfo->streamInf[audioInstanceInfo->countASInterfaces].asInterfaceHandle = interfaces[0]; 
            
            /* Reset number of Interface Settings */
            audioInstanceInfo->streamInf[audioInstanceInfo->countASInterfaces].nInterfaceSetting = 0; 
            
            /* Save Interface Id */
            audioInstanceInfo->streamInf[audioInstanceInfo->countASInterfaces].interfaceId = interfaceDescriptor->bInterfaceNumber; 
           
            /* Fill in Interface Query structure */  
            interfaceQuery.bInterfaceNumber = interfaceDescriptor->bInterfaceNumber; 
            interfaceQuery.bInterfaceClass = USB_AUDIO_CLASS_CODE; 
            interfaceQuery.bInterfaceSubClass = USB_AUDIO_AUDIOSTREAMING; 
            interfaceQuery.flags =   USB_HOST_INTERFACE_QUERY_BY_NUMBER 
                                   | USB_HOST_INTERFACE_QUERY_BY_CLASS
                                   | USB_HOST_INTERFACE_QUERY_BY_SUBCLASS 
                                   | USB_HOST_INTERFACE_QUERY_ALT_SETTING; 
            
            /* Start searching for Alternate Setting 0  */  
            interfaceQuery.bAlternateSetting = 0; 
            
            do
            {
                USB_HOST_DeviceInterfaceQueryContextClear(&interfaceQuery); 
                interfaceDescriptor = USB_HOST_DeviceGeneralInterfaceDescriptorQuery(descriptor, &interfaceQuery); 
                if (interfaceDescriptor != NULL)
                {
                    /* Get pointer to the current Audio Stream */
                    audioStream = (USB_HOST_AUDIO_STREAM_SETTING *)&audioInstanceInfo->streamInf[audioInstanceInfo->countASInterfaces].audioStreamSetting[interfaceQuery.bAlternateSetting];
                    
                    /* We found a new Alternate setting. Increment number of Alternate settings */
                    audioInstanceInfo->streamInf[audioInstanceInfo->countASInterfaces].nInterfaceSetting++;
                    
                    /* Save alternate setting number */
                    audioStream->interfaceAlternateSetting = interfaceQuery.bAlternateSetting; 
                    
                    /* Save Number of endpoints */
                    audioStream->nEndpoints = interfaceDescriptor->bNumEndPoints; 
                   
                    
                    /* Get AS Class Specific Descriptor */
                    audioCsAsDescriptor = (USB_AUDIO_CS_AS_INTERFACE_DESCRIPTOR*)((uint8_t*)interfaceDescriptor + interfaceDescriptor->bLength);
                    
                    /* Save ID of the terminal this AS interface is connected to */
                    audioStream->bTerminalLink  = audioCsAsDescriptor->bTerminalLink;
                    audioStream->wFormatTag = audioCsAsDescriptor->wFormatTag; 
                    
                    /* Get to Audio Format Descriptor */
                    audioFormatDesc = (USB_AUDIO_FORMAT_I_DESCRIPTOR_HEADER*)((uint8_t*)audioCsAsDescriptor + audioCsAsDescriptor->bLength);
                    
                    /* Save Audio Format Details */
                    audioStream->bNrChannels = audioFormatDesc->bNrChannels;
                    audioStream->bSubframeSize = audioFormatDesc->bSubframeSize; 
                    audioStream->bBitResolution = audioFormatDesc->bBitResolution; 
                    audioStream->bSamFreqType = audioFormatDesc->bSamFreqType;
                    
                    /* Check if the Audio Stream supports discrete sampling frequency */
                    if (audioStream->bSamFreqType != 0)
                    {
                        /* Get pointer to Sampling Frequencies */
                        audioStream->tSamFreq = (uint8_t *)((uint8_t*)audioFormatDesc + sizeof (USB_AUDIO_FORMAT_I_DESCRIPTOR_HEADER)); 
                    }
                  
                    if (audioStream->nEndpoints == 1)
                    { 
                        /* Clear Endpoint Query Context */
                        USB_HOST_DeviceEndpointQueryContextClear(&endpointQuery);
                        /* Fill endpoint Query */
                        endpointQuery.direction = USB_DATA_DIRECTION_DEVICE_TO_HOST; 
                        endpointQuery.transferType = USB_TRANSFER_TYPE_ISOCHRONOUS; 
                        endpointQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE | USB_HOST_ENDPOINT_QUERY_BY_DIRECTION; 

                        /* Submit Endpoint query */
                        endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointQuery); 
                        
                        if (endpointDescriptor != NULL)
                        {
                            /* We have received a valid descriptor. Copy that to our structure */
                            memcpy(&audioStream->isoDataEndpointDesc, endpointDescriptor, sizeof(USB_ENDPOINT_DESCRIPTOR) ); 
                            
                            /* Save stream direction */
                            audioStream->direction = USB_HOST_AUDIO_V1_DIRECTION_IN; 
                            
                            /* Change Audio Stream state to ready */
                            audioInstanceInfo->streamInf[audioInstanceInfo->countASInterfaces].state = USB_HOST_AUDIO_V1_STREAM_STATE_READY; 
                            
                        }
                        else
                        {
                            USB_HOST_DeviceEndpointQueryContextClear(&endpointQuery);
                            endpointQuery.direction = USB_DATA_DIRECTION_HOST_TO_DEVICE; 
                            endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery(interfaceDescriptor, &endpointQuery); 
                            if (endpointDescriptor != NULL)
                            {
                                /* We have received a valid descriptor. Copy that to our structure */
                                memcpy(&audioStream->isoDataEndpointDesc, endpointDescriptor, sizeof(USB_ENDPOINT_DESCRIPTOR) ); 
                                
                                /* Save stream direction */
                                audioStream->direction = USB_HOST_AUDIO_V1_DIRECTION_OUT; 
                                
                                /* Change Audio Stream state to ready */
                                audioInstanceInfo->streamInf[audioInstanceInfo->countASInterfaces].state = USB_HOST_AUDIO_V1_STREAM_STATE_READY; 
                            }
                            else
                            {
                                /* Error has occurred */
                                error = true; 
                                audioInstanceInfo->streamInf[audioInstanceInfo->countASInterfaces].state = USB_HOST_AUDIO_V1_STREAM_STATE_ERROR; 
                            }
                        }
                    }
                } 
                /* We need to query for next alternate setting */
                interfaceQuery.bAlternateSetting++; 
            }
            while (interfaceDescriptor != NULL);
            
            
             /* Increment Audio Streaming Interface Counter */ 
            audioInstanceInfo->countASInterfaces++; 
            break; 
    }
            
    if (error == true)
    {
        return; 
    }
    
    /* Sent attach event to the application */
    if ((gUSBHostAudio1CommonObj.attachEventHandler != NULL) 
        && (audioInstanceInfo != NULL)
        && (audioInstanceInfo->countASInterfaces == audioInstanceInfo->nASInterfaces))
    {
        gUSBHostAudio1CommonObj.attachEventHandler
        (
            (USB_HOST_AUDIO_V1_OBJ) audioInstanceInfo, 
            USB_HOST_AUDIO_V1_EVENT_ATTACH,
            gUSBHostAudio1CommonObj.context
        );
    }
    return;          
}
    
    
USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_AUDIO_V1_InterfaceEventHandler
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_DEVICE_INTERFACE_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    USB_HOST_AUDIO_STREAM_SETTING* audioStrm;
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA * dataTransferEvent;
    USB_HOST_DEVICE_INTERFACE_EVENT_SET_INTERFACE_COMPLETE_DATA *DataIntrfcSetEvent;
    USB_HOST_AUDIO_V1_STREAM_EVENT audioStreamEvent;
    USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA audioStrmXfrCmpltData;
    uint8_t alternateSetting; 
    USB_HOST_AUDIO_V1_STREAM_HANDLE strmHandle; 
    uint8_t audioIndex, asIntrfcIndex; 
    uint8_t versionFlag; 

    /* Find out to which Audio Stream this interface belongs */
    if (_USB_HOST_AUDIO_V1_IntrfcHndlToStrmIntrfcPtr
        (
            interfaceHandle,
            &asInterface,
            &audioIndex,
            &asIntrfcIndex
        ) == false) 
    {
        /* A matching interface could not be found. This is an error condition */
        return USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE; 
    }
    
    /* NULL check */
    if (asInterface == NULL)
    {
        /* A matching interface could not be found. This is an error condition */
        return USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE;
    }
            
    switch(event)
    {
        case USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE:
            /* This means a data transfer has completed */
            
            /* Get the Audio Stream Event, we saved this at the time of 
               submit request */
            audioStreamEvent = (USB_HOST_AUDIO_V1_STREAM_EVENT)(context) & 0x000000FF;
            versionFlag = (uint8_t) (context>>8); 
            
            /* Get event Data*/
            dataTransferEvent = (USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA *)(eventData);
            audioStrmXfrCmpltData.transferHandle = dataTransferEvent->transferHandle;
            audioStrmXfrCmpltData.result = dataTransferEvent->result;
            audioStrmXfrCmpltData.length = dataTransferEvent->length;
            alternateSetting = asInterface->activeInterfaceSetting; 
            if (alternateSetting == 0)
            {
                /* We should not have received an Audio transfer complete event 
                   when alternate setting is Zero. This is an error condition */
                SYS_DEBUG_MESSAGE(SYS_DEBUG_INFO, "USB_HOST_AUDIO_V1_InterfaceEventHandler: Data transfer event received but Alternate Setting is Zero"); 
                return USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE; 
            }
            
            /* Create Stream Handle */
            strmHandle = (USB_HOST_AUDIO_V1_STREAM_HANDLE ) (uint32_t)audioIndex|
                                                              (uint32_t)asIntrfcIndex<<8|
                                                              (uint32_t)alternateSetting<<16 ;
            
            if (versionFlag == USB_HOST_AUDIO_V1_API_VERSION_FLAG_V1)
            {
                /* Send event to the Audio Stream Handler */
                if(asInterface->streamEventHandler != NULL)
                {
                    asInterface->streamEventHandler(strmHandle, audioStreamEvent, 
                                        &audioStrmXfrCmpltData, asInterface->context);
                }
            }
            else if (versionFlag == USB_HOST_AUDIO_V1_API_VERSION_FLAG_V1_0_DEPRECIATED)
            {
                /* Get handle to the current active stream */
                audioStrm = &asInterface->audioStreamSetting[alternateSetting];
                if(audioStrm->streamEventHandler != NULL)
                {
                    audioStrm->streamEventHandler(strmHandle, audioStreamEvent, 
                                        &audioStrmXfrCmpltData, audioStrm->context);
                }
            }
            break;
            
        case USB_HOST_DEVICE_INTERFACE_EVENT_SET_INTERFACE_COMPLETE:
               
            DataIntrfcSetEvent = (USB_HOST_DEVICE_INTERFACE_EVENT_SET_INTERFACE_COMPLETE_DATA*)(eventData);
            
            /* Save event and event data in the Audio Streaming interface structure */
            asInterface->stateData.eventData.requestHandle = DataIntrfcSetEvent->requestHandle; 
            asInterface->stateData.eventData.result = DataIntrfcSetEvent->result;
            asInterface->stateData.context = context; 
            asInterface->stateData.event = event; 
            
            /* Change Audio Stream state to Pipe Action Pending. The Pipe open
               or close action will be performed in the Interface Event Handler
               routine. */
            asInterface->state = USB_HOST_AUDIO_V1_STREAM_STATE_PIPE_ACTION_PENDING;            
            break; 
            
        default:
            break;
    }

    
    return(USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE);
}

void _USB_HOST_AUDIO_V1_InterfaceTasks( USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle)
{
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioIndex, asIntrfcIndex;
    USB_ENDPOINT_ADDRESS endpointAddress; 
    USB_HOST_RESULT result;
    USB_HOST_AUDIO_STREAM_SETTING* audioStrmSetting;
    uint8_t alternateSetting; 
    uint8_t audioStreamEventLocal;
    USB_HOST_AUDIO_V1_STREAM_HANDLE strmHandle; 
    USB_HOST_AUDIO_V1_STREAM_EVENT audioStreamEvent;
    bool changeAlternateSetting = false; 
    
    /* Find out to which Audio Stream this Interface belongs */
    if (_USB_HOST_AUDIO_V1_IntrfcHndlToStrmIntrfcPtr
        (
            interfaceHandle,
            &asInterface,
            &audioIndex,
            &asIntrfcIndex
        ) == false) 
    {
        /* A matching interface could not be found. This is an error condition */
        return; 
    }
    
    /* NULL check */
    if (asInterface == NULL)
    {
        /* A matching interface could not be found. This is an error condition */
        return ;
    }
    
    switch (asInterface->state)    
    {
        case USB_HOST_AUDIO_V1_STREAM_STATE_PIPE_ACTION_PENDING: 
            
            asInterface->state = USB_HOST_AUDIO_V1_STREAM_STATE_READY; 
            
            /* Get the requested alternate setting, we saved this at the time of
               request submit */
            alternateSetting = (uint8_t)asInterface->stateData.context;
            
            /* Get the event */
            audioStreamEventLocal = (uint8_t) (asInterface->stateData.context>>8); 
            
            /* Get pointer to Audio Interface Setting  */
            audioStrmSetting = &asInterface->audioStreamSetting[alternateSetting]; 
                    
            if (asInterface->stateData.eventData.result == USB_HOST_RESULT_SUCCESS)
            {
                
                if (alternateSetting)
                {
                    /* Alternate Setting is a Non Zero Value. That means we have 
                       to Open pipe for all of the Endpoints present in this 
                       Alternate Setting. */
                    if (audioStrmSetting->nEndpoints)
                    {
                        /* Open pipe for Audio Data Endpoint*/
                        if (asInterface->isIsoDataPipeSet == false)
                        {
                            endpointAddress = audioStrmSetting->isoDataEndpointDesc.bEndpointAddress; 
                            asInterface->isoDataPipeHandle = USB_HOST_DevicePipeOpen( asInterface->asInterfaceHandle, endpointAddress);

                            if (asInterface->isoDataPipeHandle != USB_HOST_PIPE_HANDLE_INVALID)
                            {
                                asInterface->isIsoDataPipeSet = true; 
                                changeAlternateSetting = true; 
                                asInterface->state = USB_HOST_AUDIO_V1_STREAM_STATE_PIPE_OPEN_SUCCESS; 
                            }
                            else
                            {
                                asInterface->state = USB_HOST_AUDIO_V1_STREAM_PIPE_OPEN_FAILED; 
                                asInterface->isIsoDataPipeSet = false;
                                changeAlternateSetting = false; 
                                SYS_DEBUG_MESSAGE  ( SYS_DEBUG_INFO , "USB_HOST_AUDIO_V1_InterfaceEventHandler():Pipe Open Failed" );
                            }
                        }
                    }
                }
            }
            else 
            {
                if (asInterface->isIsoDataPipeSet == true)
                {
                    result = USB_HOST_DevicePipeClose(asInterface->isoDataPipeHandle); 
                    if (result != USB_HOST_RESULT_SUCCESS)
                    {
                        changeAlternateSetting = false; 
                        asInterface->state = USB_HOST_AUDIO_V1_STREAM_PIPE_CLOSE_FAILED; 
                        SYS_DEBUG_MESSAGE  ( SYS_DEBUG_INFO , "USB_HOST_AUDIO_V1_InterfaceEventHandler():Pipe Close Failed" );
                    }
                    else
                    {
                        changeAlternateSetting = true; 
                        asInterface->state = USB_HOST_AUDIO_V1_STREAM_STATE_PIPE_CLOSE_SUCCESS; 
                    }
                    
                    asInterface->isIsoDataPipeSet = false; 
                }
            }
            
            if (changeAlternateSetting == true)
            {
                /* Change in the alternate setting in the Audio Instance */
                asInterface->activeInterfaceSetting = alternateSetting;
            }
            
            if (audioStreamEventLocal == USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_INTERFACE_SET)
            {
                audioStreamEvent = USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE;
                
                /* Stream Handle */
                strmHandle = (USB_HOST_AUDIO_V1_STREAM_HANDLE ) (uint32_t)audioIndex|
                                                              (uint32_t)asIntrfcIndex<<8;
                if(asInterface->streamEventHandler != NULL)
                {
                    asInterface->streamEventHandler(strmHandle, audioStreamEvent, 
                                    &asInterface->stateData.eventData, asInterface->context);
                }
            }
            else if (audioStreamEventLocal == USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_DISABLE)
            {
                audioStreamEvent = USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE; 
                
                /* Stream Handle */
                strmHandle = (USB_HOST_AUDIO_V1_0_STREAM_HANDLE ) (uint32_t)audioIndex|
                                                              (uint32_t)asIntrfcIndex<<8|
                                                              (uint32_t)alternateSetting<<16 ;
                if(audioStrmSetting->streamEventHandler != NULL)
                {
                    audioStrmSetting->streamEventHandler(strmHandle, audioStreamEvent, 
                                        &asInterface->stateData.eventData, audioStrmSetting->context);
                }
            }
            else if (audioStreamEventLocal == USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_ENABLE)
            {
                audioStreamEvent = USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE;
                
                /* Stream Handle */
                strmHandle = (USB_HOST_AUDIO_V1_0_STREAM_HANDLE ) (uint32_t)audioIndex|
                                                              (uint32_t)asIntrfcIndex<<8|
                                                              (uint32_t)alternateSetting<<16 ;
                if(audioStrmSetting->streamEventHandler != NULL)
                {
                    audioStrmSetting->streamEventHandler(strmHandle, audioStreamEvent, 
                                        &asInterface->stateData.eventData, audioStrmSetting->context);
                }
            }
 
            
        default:
            break; 
    }    
    
                        
  
}

// ****************************************************************************
/* Function:
    bool _USB_HOST_AUDIO_V1_IntrfcHndlToStrmIntrfcPtr
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle, 
        USB_HOST_AUDIO_STREAMING_INTERFACE** strmIntfcPtr, 
        uint8_t *audioIndex, 
        uint8_t *asIntrfcIndex
    )

  Summary:
    This function converts an interface handle to following details. 
         1) Pointer to the Audio streaming interface. 
         2) Index of the Audio Instance
         3) Index of the Audio Streaming interface array. 

  Description:
    This function converts an interface handle to following details. 
         1) Pointer to the Audio streaming interface. 
         2) Index of the Audio Instance
         3) Index of the Audio Streaming interface array. 
   
  Precondition:
    None.

  Parameters:
    interfaceHandle - Interface Handle
    
    strmIntfcPtr    - Pointer to a pointer of Audio Streaming Interfacce. This is 
                      an OUT parameter. 
    
    audioIndex      - Pointer to Audio Instance Array Index. This is an OUT parameter. 
 
   asIntrfcIndex    - Pointer to Audio Streaming interface Array Index. This is an 
                      an OUT parameter. 

  Returns:
    true - Audio Streaming interface was found.  

    false - Audio Streaming interface was not found. 
  */ 
bool _USB_HOST_AUDIO_V1_IntrfcHndlToStrmIntrfcPtr
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle, 
    USB_HOST_AUDIO_STREAMING_INTERFACE** strmIntfcPtr, 
    uint8_t *audioIndex, 
    uint8_t *asIntrfcIndex
)
{
    int count;
    int asIntrfcCount; 
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 

    for(count = 0; count < USB_HOST_AUDIO_V1_INSTANCES_NUMBER; count++)
    {
        audioInstanceInfo = &gUSBHostAudioInstance[count]; 
        for (asIntrfcCount =0; asIntrfcCount <  audioInstanceInfo->nASInterfaces; asIntrfcCount++ )
        {
            *strmIntfcPtr = &audioInstanceInfo->streamInf[asIntrfcCount];
            if ((*strmIntfcPtr)->asInterfaceHandle == interfaceHandle)
            {   
                *audioIndex = count; 
                *asIntrfcIndex = asIntrfcCount; 
                return   true;   
            } 
        }
    }
    return false;
}

/*************************************************************************/

/* Function:
   void  _USB_HOST_AUDIO_DeInitialize(HC_DEVICE_ID id )

  Summary:
    DeInitialize the AUDIO host driver.

  Description:
    After deenumeration Host will Deinitialize the AUDIO driver

  Parameters:
     HC_DEVICE_ID                           id ,

  Returns:
        void
*/

void _USB_HOST_AUDIO_V1_InterfaceRelease( USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle )
 {
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    uint8_t audioIndex, asIntrfcIndex; 
    USB_AUDIO_SUBCLASS_CODE audioInterface = USB_AUDIO_SUBCLASS_UNDEFINED; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface; 
    int strmIndex; 
    
    
    audioIndex = _USB_HOST_AUDIO_V1_InterfaceHandleToAudioInstance(interfaceHandle); 
    if (audioIndex < USB_HOST_AUDIO_V1_INSTANCES_NUMBER)
    {
        audioInterface = USB_AUDIO_AUDIOCONTROL;
    }
    else
    {
        /* Find out to which Audio Stream this interface belongs */
            if (_USB_HOST_AUDIO_V1_IntrfcHndlToStrmIntrfcPtr
                (
                    interfaceHandle,
                    &asInterface,
                    &audioIndex,
                    &asIntrfcIndex
                ) == true) 
            {
                audioInterface = USB_AUDIO_AUDIOSTREAMING;
            }
    }
    
    
    switch (audioInterface)
    {
        case USB_AUDIO_AUDIOCONTROL:
            audioInstanceInfo = &gUSBHostAudioInstance[audioIndex]; 
            
            /* Close Control Pipe */ 
            //USB_HOST_DevicePipeClose(audioInstanceInfo->controlPipeHandle); 
            
            for (strmIndex = 0; strmIndex< audioInstanceInfo->nASInterfaces; strmIndex++)
            {
                asInterface = &audioInstanceInfo->streamInf[strmIndex]; 
                
                /* Save Audio Streaming Interface Number */
                asInterface->interfaceId = 0;

                /* Set Alternate setting to Zero */
                asInterface->activeInterfaceSetting = 0;
                
                if (asInterface->isIsoDataPipeSet == true)
                {
                    /* Close pipe */
                    USB_HOST_DevicePipeClose(asInterface->isoDataPipeHandle); 
                }
            }
             
            /* Notify client about detach event */
            if (gUSBHostAudio1CommonObj.attachEventHandler != NULL)
            {
                gUSBHostAudio1CommonObj.attachEventHandler
                (
                    (USB_HOST_AUDIO_V1_OBJ) audioInstanceInfo, 
                    USB_HOST_AUDIO_V1_EVENT_DETACH,
                    gUSBHostAudio1CommonObj.context
                ); 
            }
            audioInstanceInfo->assigned = false; 
            
            memset(audioInstanceInfo, 0, sizeof(USB_HOST_AUDIO_V1_INSTANCE)); 
            break; 
        case USB_AUDIO_AUDIOSTREAMING:
            
            Nop(); 
            break; 
        default:
        break; 
    }
 }


// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamWrite 
   (
       USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
       USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
       void * source, 
       size_t length
   ); 

  Summary:
    Schedules an  Audio stream Write request for the specified audio stream. 

  Description:
    This function schedules an  Audio stream Write request for the specified 
    audio stream. An event 
    USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE is generated when this 
    request is completed. 
    USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA returns 
    the status and request handle of the request.
    
  Precondition:
    Audio stream should have been opened and enabled. The direction of the 
    Audio Stream should be USB_HOST_AUDIO_V1_DIRECTION_OUT. 

  Parameters:
    streamHandle    - Handle to the Audio v1.0 Stream.

    transferHandle  - Handle to the Stream Write transfer request 

    source          - Pointer to the buffer containing data to be written to the 
                      device. 

    length          - Amount of data to written (in bytes).

  Returns:
    USB_HOST_AUDIO_V1_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT _USB_HOST_AUDIO_V1_StreamWrite 
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
    void * source, 
    size_t length, 
    uint8_t apiVersionFlag
)
{
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex;  
    USB_HOST_PIPE_HANDLE audioDataPipeHandle; 
    USB_HOST_AUDIO_V1_RESULT audioResult = USB_HOST_AUDIO_V1_RESULT_FAILURE;
    USB_HOST_RESULT hostResult; 
    
    
    /* Find Audio Stream from audioStreamObj */
    if (streamHandle == USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID)
    {
        return USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER; 
    }
   
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8);  

    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex];
    
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }

    if((length != 0) && (source == NULL))
    {
        /* Input parameters are not valid */
        return  USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER;
    }
    
    /* The context for the transfer is the event that needs to
     * be sent to the application. In this case the event to be
     * sent to the application when the transfer completes is
     * USB_HOST_AUDIO_V1_EVENT_READ_COMPLETE */
    if (audioInstanceInfo->streamInf[asIntrefaceIndex].isIsoDataPipeSet == true)
    {
        audioDataPipeHandle = audioInstanceInfo->streamInf[asIntrefaceIndex].isoDataPipeHandle; 
        hostResult = USB_HOST_DeviceTransfer(audioDataPipeHandle, transferHandle, source, length, (uintptr_t)(USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE | apiVersionFlag << 8 ));
    }
    else
    {
        return USB_HOST_AUDIO_V1_RESULT_FAILURE; 
    }
    switch (hostResult)
    {
        case USB_HOST_RESULT_SUCCESS:
            audioResult = USB_HOST_AUDIO_V1_RESULT_SUCCESS;
            break;
            
        case USB_HOST_RESULT_REQUEST_BUSY:
            audioResult = USB_HOST_AUDIO_V1_RESULT_BUSY; 
            break; 
            
        default:
            audioResult = USB_HOST_AUDIO_V1_RESULT_FAILURE; 
            break; 
    }
    return audioResult; 
}

// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_RESULT _USB_HOST_AUDIO_V1_StreamRead 
   (
       USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
       USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
       void * source, 
       size_t length
   ); 

  Summary:
    Schedules an  Audio stream Read request for the specified audio stream. 

  Description:
    This function schedules an  Audio stream Read request for the specified 
    audio stream. An event 
    USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE is generated when this 
    request is completed. 
    USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE_DATA returns 
    the status and request handle of the request.
    
  Precondition:
    Audio stream should have been opened and enabled. The direction of the 
    Audio Stream should be USB_HOST_AUDIO_V1_DIRECTION_IN. 

  Parameters:
    streamHandle    - Handle to the Audio v1.0 Stream.

    transferHandle  - Handle to the Stream Read transfer request 

    source          - Pointer to the buffer containing data to be read from the 
                      device. 

    length          - Amount of data to read (in bytes).

  Returns:
    USB_HOST_AUDIO_V1_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT _USB_HOST_AUDIO_V1_StreamRead 
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
    void * source, 
    size_t length, 
    uint8_t apiVersionFlag    
)
{
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex;  
    USB_HOST_PIPE_HANDLE audioDataPipeHandle; 
    USB_HOST_AUDIO_V1_RESULT audioResult = USB_HOST_AUDIO_V1_RESULT_FAILURE;
    USB_HOST_RESULT hostResult; 
    
    
    /* Find Audio Stream from audioStreamObj */
    if (streamHandle == USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID)
    {
        return USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER; 
    }
   
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8);  

    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex];

    if((length != 0) && (source == NULL))
    {
        /* Input parameters are not valid */
        return  USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER;
    }
    
    /* The context for the transfer is the event that needs to
     * be sent to the application. In this case the event to be
     * sent to the application when the transfer completes is
     * USB_HOST_AUDIO_V1_EVENT_READ_COMPLETE */
    if (audioInstanceInfo->streamInf[asIntrefaceIndex].isIsoDataPipeSet == true)
    {
        audioDataPipeHandle = audioInstanceInfo->streamInf[asIntrefaceIndex].isoDataPipeHandle; 
        hostResult = USB_HOST_DeviceTransfer(audioDataPipeHandle, transferHandle, source, length, (uintptr_t)(USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE) | apiVersionFlag << 8);
    }
    else
    {
        return USB_HOST_AUDIO_V1_RESULT_FAILURE; 
    }
    switch (hostResult)
    {
        case USB_HOST_RESULT_SUCCESS:
            audioResult = USB_HOST_AUDIO_V1_RESULT_SUCCESS;
            break;
            
        case USB_HOST_RESULT_REQUEST_BUSY:
            audioResult = USB_HOST_AUDIO_V1_RESULT_BUSY; 
            break; 
            
        default:
            audioResult = USB_HOST_AUDIO_V1_RESULT_FAILURE; 
            break; 
    }
    return audioResult; 
}

// ****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_AttachEventHandlerSet
    (
        USB_HOST_AUDIO_V1_ATTACH_EVENT_HANDLER eventHandler,
        uintptr_t context
    );

  Summary:
    This function will set an attach/detach event handler.

  Description:
    This function will set an attach event handler. The attach event handler
    will be called when a Audio v1.0 device has been attached or detached. The 
    context will be returned in the event handler. This function should be 
    called before the bus has been enabled.
    
  Precondition:
    None.

  Parameters:
    eventHandler - Pointer to the attach event handler
    
    context      - An application defined context that will be returned in the event
                   handler.

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - if the attach event handler was registered
    successfully. 

    USB_HOST_AUDIO_V1_RESULT_FAILURE - if the number of registered event 
    handlers has exceeded USB_HOST_AUDIO_V1_ATTACH_LISTENERS_NUMBER.
    
  Example:
    <code> 
    </code>

  Remarks:
    Function should be called before USB_HOST_BusEnable() function is called.

 */
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_AttachEventHandlerSet
(
    USB_HOST_AUDIO_V1_ATTACH_EVENT_HANDLER attachEventHandler,
    uintptr_t context
        
)
{
    /* validate callback handler */
    if( NULL == attachEventHandler )
    {
        return USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER;
    }
    gUSBHostAudio1CommonObj.attachEventHandler = attachEventHandler; 
    gUSBHostAudio1CommonObj.context = context; 
    return USB_HOST_AUDIO_V1_RESULT_SUCCESS; 
    
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_EntityRequestCallbackSet
    (
        USB_HOST_AUDIO_V1_OBJ audioDeviceObj, 
        USB_HOST_AUDIO_V1_ENTITY_REQUEST_CALLBACK appAudioEntityRequestCallback,
        uintptr_t context 
    ); 

   Summary:
    Registers an Audio Entity request callback function with the Audio v1.0
    Client Driver.

   Description:
    This function registers a callback function for the Audio v1.0 Control
    Entity requests. The Audio v1.0 Host Client Driver will invoke this
    callback function when an Audio Entity control request is completed.   
    
   Precondition:
    None.

   Parameters:
    audioDeviceObj  - Audio v1.0 device object.

    appAudioEntityRequestCallback - A pointer to event handler function. If NULL,
    then events will not be generated.
    
    context - Application specific context that is returned in the event handler.

   Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - The specified audio 
    Object does not exist.
    USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
    
   Example:
     <code>
     </code>

   Remarks:
     None.
*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_EntityRequestCallbackSet
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj, 
    USB_HOST_AUDIO_V1_ENTITY_REQUEST_CALLBACK appAudioEntityRequestCallback,
    uintptr_t context 
)
{
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo  = (USB_HOST_AUDIO_V1_INSTANCE*)audioDeviceObj; 
    
    /* NULL check */ 
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    if (appAudioEntityRequestCallback != NULL)
    {
        audioInstanceInfo->audioControlObj.callback = appAudioEntityRequestCallback; 
        audioInstanceInfo->audioControlObj.context = context; 
        return USB_HOST_AUDIO_V1_RESULT_SUCCESS; 
    }
    
    return USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER; 
}
// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_STREAM_RESULT USB_HOST_AUDIO_V1_StreamEventHandlerSet
    (
        USB_HOST_AUDIO_V1_STREAM_HANDLE handle,
        USB_HOST_AUDIO_V1_STREAM_EVENT_HANDLER appAudioHandler,
        uintptr_t context
    ); 

  Summary:
    Registers an event handler with the Audio v1.0 Client Driver Stream.

  Description:
    This function registers a client specific Audio v1.0 stream event handler.
    The Audio v1.0 Host Client Driver will call appAudioHandler function 
    specified as 2nd argument with relevant event and associate event data, in
    response to audio stream data transfers that have been scheduled by the 
    client.
    
  Precondition:
    None.

  Parameters:
    handle  - handle to the Audio v1.0 Stream.

    eventHandler - A pointer to event handler function. If NULL, then events
                   will not be generated.
    
    context - Application specific context that is returned in the event handler.

  Returns:
    USB_HOST_AUDIO_V1_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamEventHandlerSet
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_EVENT_HANDLER streamEventHandler,
    uintptr_t context
)
{ 
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx;   
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo ; 
    

    audioInstance = (uint8_t)streamHandle; 
    aSIntrfcIdx = (uint8_t) (streamHandle >> 8); 
    asIntrfcSettingIdx = (uint8_t)(streamHandle >> 16);  
    
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    /* NULL check */ 
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }

    /* Check if there is any Audio Streaming Interface present in the Audio Device */ 
    if (audioInstanceInfo->nASInterfaces == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Verify Audio Streaming Interface Object */ 
    if (aSIntrfcIdx >= audioInstanceInfo->nASInterfaces)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Verify Audio streaming interface Setting Index */ 
    if (asIntrfcSettingIdx >= audioInstanceInfo->streamInf[aSIntrfcIdx].nInterfaceSetting)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    } 
    
    /* Check if next Streaming Interface exists */ 
    if (aSIntrfcIdx == audioInstanceInfo->nASInterfaces)
    {
        return USB_HOST_AUDIO_V1_RESULT_END_OF_STREAMING_INTERFACE; 
    }

    if (streamEventHandler != (USB_HOST_AUDIO_V1_STREAM_EVENT_HANDLER)NULL)
    {
        audioInstanceInfo->streamInf[aSIntrfcIdx].streamEventHandler = streamEventHandler;
        audioInstanceInfo->streamInf[aSIntrfcIdx].context = context; 
        return USB_HOST_AUDIO_V1_RESULT_SUCCESS; 
    }
    
    return USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER; 
}



void _USB_HOST_AUDIO_V1_ControlRequestCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
)
{
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE *)context; 
    
    if (audioInstanceInfo->audioControlObj.callback != NULL)
    {
        audioInstanceInfo->audioControlObj.callback
        (
            audioInstanceInfo->deviceObjHandle,
            requestHandle,
            result, 
            size, 
            audioInstanceInfo->audioControlObj.context     
        ); 
    }
    audioInstanceInfo->audioControlObj.inUse = false; 
}


void _USB_HOST_AUDIO_V1_SetSampleFrequencyCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
)
{
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA samplingRateData; 
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle = (USB_HOST_AUDIO_V1_STREAM_HANDLE)context;
    
     /* Find Audio Stream from audioStreamObj */
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
   
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex];
    
    samplingRateData.requestHandle = requestHandle; 
    samplingRateData.requestStatus = result; 
    
    if (asInterface->streamEventHandler != NULL)
    {
        asInterface->streamEventHandler
        (
            streamHandle,
            USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_SET_COMPLETE,
            &samplingRateData, 
            asInterface->context     
        ); 
    }
    audioInstanceInfo->audioControlObj.inUse = false; 
}


void _USB_HOST_AUDIO_V1_GetSampleFrequencyCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
)
{
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_GET_COMPLETE_DATA samplingRateData; 
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle = (USB_HOST_AUDIO_V1_STREAM_HANDLE)context;
    
     /* Find Audio Stream from audioStreamObj */
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
   
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex];
    
    samplingRateData.requestHandle = requestHandle; 
    samplingRateData.requestStatus = result; 
    
    if (asInterface->streamEventHandler != NULL)
    {
        asInterface->streamEventHandler
        (
            streamHandle,
            USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_GET_COMPLETE,
            &samplingRateData, 
            asInterface->context     
        ); 
    }
    audioInstanceInfo->audioControlObj.inUse = false; 
}
void _USB_HOST_AUDIO_V1_SetSampleRateCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
)
{
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA samplingRateData; 
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    USB_HOST_AUDIO_STREAM_SETTING* audioStrm; 
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    uint8_t alternateSetting; 
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle = (USB_HOST_AUDIO_V1_STREAM_HANDLE)context; 
     /* Find Audio Stream from audioStreamObj */
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
    alternateSetting = (uint8_t)(streamHandle>>16);
   
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex];
    audioStrm = &asInterface->audioStreamSetting[alternateSetting]; 
            
    samplingRateData.requestHandle = requestHandle; 
    samplingRateData.requestStatus = result; 
    
    if (audioStrm->streamEventHandler != NULL)
    {
        audioStrm->streamEventHandler
        (
            streamHandle,
            USB_HOST_AUDIO_V1_0_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE,
            &samplingRateData, 
            audioStrm->context     
        ); 
    }
    audioInstanceInfo->audioControlObj.inUse = false; 
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT _USB_HOST_AUDIO_V1_ControlRequest
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
        USB_SETUP_PACKET *setupPacket,
        void * data,
        USB_HOST_AUDIO_V1_CONTROL_CALLBACK callback, 
        uintptr_t context 
    );

  Summary:
    Schedules an Audio v1.0 control transfer.

  Description:
    This function schedules an Audio v1.0 control transfer. audioObj is an 
    Object of Audio v1.0 class driver to which the audio control transfer is to
    be scheduled. setupPacket points to the setup command to be sent in the 
    Setup Stage of the control transfer. The size and the direction of the data
    stage is indicated by the setup packet. In case of control transfers where
    there is no data stage, data is ignored and can be NULL. In all other cases,
    data should point to the data to data be transferred in the data stage of 
    the control transfer. 
    
    If the transfer was scheduled successfully, requestHandle will contain a
    transfer handle that uniquely identifies this transfer. If the transfer
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.

    When the control transfer completes, the Audio v1.0 client driver will call
    the specified callback function. The context parameter specified here will 
    be returned in the callback.

  Precondition:
    Audio v1.0 Device should have attached. 

  Parameters:
    audioObj - Audio v1.0 client driver object 

    requestHandle - output parameter that will contain the handle to this
    transfer.

    setupPacket - Pointer to the setup packet to sent to the device in the setup
    stage of the control transfer.

    data -  For control transfer with a data stage, this should point to data to
    be sent to the device (for a control write transfer) or point to the buffer
    that will receive data from the device (for a control read transfer). For
    control transfers that do not require a data stage, this parameter is
    ignored and can be NULL.

    callback - pointer to the callback function that will be called when the
    control transfer completes. If the callback function is NULL, there will be
    no notification of when the control transfer will complete.

    context - user defined context that is returned with the callback function.

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - the transfer was scheduled successfully.
    requestHandle will contain a valid transfer handle.
    USB_HOST_AUDIO_V1_RESULT_FAILURE - an unknown failure occurred. requestHandle will
    contain USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
    USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or requestHandle pointer
    is NULL.

  Example:
    <code>
    </code>

  Remarks:
    This is a local function and should not be accessed by applications.  
*/
USB_HOST_AUDIO_V1_RESULT _USB_HOST_AUDIO_V1_ControlRequest
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_REQUEST_HANDLE * transferHandle,
    USB_SETUP_PACKET *setupPacket,
    void * data
)
{
    USB_HOST_AUDIO_V1_RESULT audioResult = USB_HOST_AUDIO_V1_RESULT_FAILURE; 
    USB_HOST_RESULT hostResult = USB_HOST_RESULT_FAILURE; 
    USB_HOST_AUDIO_V1_INSTANCE *audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE *)audioObj ;
    OSAL_CRITSECT_DATA_TYPE IntState;
    
    /* Check if the handle to the audio Instance in not NULL */
    if( audioInstanceInfo == NULL )
    {
        SYS_DEBUG_MESSAGE ( SYS_DEBUG_INFO, "USB_HOST_AUDIO_ControlSend: Not a valid instance " );
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID;
    }

    /* Check if this Audio Instance is initialized */
    if(audioInstanceInfo->assigned == false)
    {
        SYS_DEBUG_MESSAGE ( SYS_DEBUG_INFO , "USB_HOST_AUDIO_ControlSend: Invalid parameters " );
        return USB_HOST_AUDIO_V1_RESULT_FAILURE;
    }

    if (audioInstanceInfo->audioControlObj.inUse == true)
    {
        return USB_HOST_AUDIO_V1_RESULT_BUSY; 
    }
    
    /* Prevent other tasks pre-empting this sequence of code */ 
    IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
    
    memcpy (&audioInstanceInfo->setupPacket, setupPacket, sizeof (USB_SETUP_PACKET)); 
   
    audioInstanceInfo->audioControlObj.inUse = true; 
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
    
    /* Submit Request*/
    hostResult = USB_HOST_DeviceControlTransfer
             (
                 audioInstanceInfo->controlPipeHandle,
                 transferHandle,
                 &audioInstanceInfo->setupPacket,
                 (void *)data,
                 _USB_HOST_AUDIO_V1_ControlRequestCallback,
                 (uintptr_t)audioInstanceInfo
             ); 
    
    switch (hostResult)
    {
        case USB_HOST_RESULT_PARAMETER_INVALID:
            audioResult = USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER; 
            break; 
        case USB_HOST_RESULT_REQUEST_BUSY:
            audioResult = USB_HOST_AUDIO_V1_RESULT_BUSY; 
            break; 
        case USB_HOST_RESULT_SUCCESS:
            audioResult = USB_HOST_AUDIO_V1_RESULT_SUCCESS; 
            break; 
        default: 
            audioResult = USB_HOST_AUDIO_V1_RESULT_FAILURE; 
            break; 
        
    }
            
    return audioResult;
}
/*******************************************************************/

// *****************************************************************************
// *****************************************************************************
// USB Host Audio v1.0 Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
   int_USB_HOST_AUDIO_V1_InterfaceHandleToAUDIOInstance
   ( 
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
   )

  Summary:
    This function will return the Audio v1.0 instance object that is associated with
    this interface.

  Description:
    This function will return the Audio V1.0 instance object that is associated with
    this interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_AUDIO_V1_InterfaceHandleToAudioInstance
( 
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    int iterator;
    int audioIndex = -1;

    /* Find the Audio v1.0 Instance object that owns this interface */
    for (iterator = 0; iterator < USB_HOST_AUDIO_V1_INSTANCES_NUMBER; iterator ++)
    {
        if(gUSBHostAudioInstance[iterator].assigned)
        {
            if(gUSBHostAudioInstance[iterator].acInterfaceHandle== interfaceHandle)
            {
                /* Found it */
                audioIndex = iterator;
                break;
            }
        }
    }
    return(audioIndex);
}
// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceGetFirst
    (
        USB_HOST_AUDIO_V1_OBJ audioObj, 
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ* streamingInterfaceObj
    )

  Summary:
    
  Description:
    
    
  Precondition:
    
  Parameters:
    
  Returns:
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceGetFirst
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ* streamingInterfaceObj
)
{
    /* Get Pointer to Audio Device Instance  */
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE*)audioDeviceObj;
    
    /* Null check */
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* If there is any Audio Streaming Interface present in the Audio Device */ 
    if (audioInstanceInfo->nASInterfaces == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_END_OF_INTERFACE_SETTINGS; 
    }
    
    /* Get pointer to First Audio Streaming Interface */
    *streamingInterfaceObj = (USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ)audioInstanceInfo->index; 
    
    return USB_HOST_AUDIO_V1_RESULT_SUCCESS;     
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceGetNext
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,    
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObjCurrent 
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ* streamingInterfaceObjNext
    );

  Summary:
    This function will Get the next streaming interface in the attached 
    Audio Device.

  Description:
    This function will Get the next streaming interface in the attached 
    Audio Device. 
    
  Precondition:
    Audio v1.0 device should have been attached. 

  Parameters:
    audioObj - Audio Device Object
    streamingInterfaceObjCurrent - Current Audio Streaming Interface Object
    streamingInterfaceObj - Pointer to Audio Streaming Interface Object 

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request completed successfully
    USB_HOST_AUDIO_V1_RESULT_END_OF_STREAMING_INTERFACE - No more streaming 
    interfaces available 
    USB_HOST_AUDIO_V1_RESULT_DEVICE_UNKNOWN - Device is not attached currently
    USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - Audio Device object is invalid
    USB_HOST_AUDIO_V1_RESULT_FAILURE - An error has occured
    
     
  Example:
    <code> 
    </code>

  Remarks:

*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceGetNext
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObjCurrent, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ* streamingInterfaceObjNext
)
{
    /* Get Pointer to Audio Device Instance  */
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE*)audioDeviceObj;
    uint8_t aSIntrfcIdxCurrent; 
    uint8_t aSIntrfcIdxNext; 
    
    /* Get the Streaming Interface Index from the Streaming Interface Object */ 
    aSIntrfcIdxCurrent = (uint8_t)(streamingInterfaceObjCurrent>>8); 
    
    /* If there is any Audio Streaming Interface present in the Audio Device */ 
    if (audioInstanceInfo->nASInterfaces == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_END_OF_INTERFACE_SETTINGS; 
    }
    
    /* Verify Audio Streaming Interface Object */ 
    if (aSIntrfcIdxCurrent >= audioInstanceInfo->nASInterfaces)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Get the Index of the next Streaming Interface*/ 
    aSIntrfcIdxNext = aSIntrfcIdxCurrent + 1; 
    
    /* Check if next Streaming Interface exists */ 
    if (aSIntrfcIdxNext == audioInstanceInfo->nASInterfaces)
    {
        return USB_HOST_AUDIO_V1_RESULT_END_OF_STREAMING_INTERFACE; 
    }
    
    *streamingInterfaceObjNext = (USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ)(audioInstanceInfo->index | (uint32_t)aSIntrfcIdxNext<<8); 
    
    return USB_HOST_AUDIO_V1_RESULT_SUCCESS;  
}
// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetFirst
   (
       USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
       USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
       USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *interfaceSettingObj
   )

  Summary:
    Returns information about first audio stream in the specified Audio stream 
    group. 

  Description:
    This function returns information about first audio stream in the specified
    Audio stream group. The stream group index is parameter to this function 
    and it can be any value starting from Zero to number of stream groups minus
    one. Number of stream groups can be obtained by using 
    USB_HOST_AUDIO_V1_NumberOfStreamGroupsGet() function. 
    
    streamInfo object is an out parameter to this function. 
    
  Precondition:
    Audio v1.0 device should have been attached to the Host. 

  Parameters:
    audioDeviceObj   - Audio v1.0 client driver object

    streamGroupIndex - Stream Group Index.  
    
    streamInfo       -  Pointer to streamInfo object 

  Returns:
    USB_HOST_AUDIO_V1_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - The specified Audio v1.0 client
    driver object does not exist.
    USB_HOST_AUDIO_V1_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetFirst
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *interfaceSettingObj
)
{
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE*)audioDeviceObj;
    uint8_t aSIntrfcIdx;
    
    /* NULL check */ 
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    if (interfaceSettingObj == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER; 
    }
    
    /* Get the Streaming Interface Index from the Streaming Interface Object */ 
    aSIntrfcIdx = (uint8_t)(streamingInterfaceObj>>8); 
    
    /* Verify Audio Streaming Interface Object */ 
    if (aSIntrfcIdx >= audioInstanceInfo->nASInterfaces)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Fill the Audio streaming interface setting object */ 
    *interfaceSettingObj = (USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ)(audioInstanceInfo->index | aSIntrfcIdx<<8 );  
    
    /* Return success */ 
    return USB_HOST_AUDIO_V1_RESULT_SUCCESS;     
}
// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetNext 
   (
       USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
       USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
       USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObjCurrent, 
       USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *interfaceSettingObjNext
   )

  Summary:
    Returns information about next audio stream in the specified Audio stream 
    group.

  Description:
    This function returns information about next audio stream in the specified
    Audio stream group. USB_HOST_AUDIO_V1_StreamGetFirst() function should 
    have been called at least once on the same Audio stream group before calling 
    this function. Then calling this function repeatedly on the stream group 
    will return information about the next audio stream in the stream group. 
    When there are no more audio streams to report, the function returns 
    USB_HOST_AUDIO_V1_RESULT_END_OF_STREAM_LIST. 
    
    Calling the USB_HOST_AUDIO_V1_StreamGetFirst() function on the stream group
    index after the USB_HOST_AUDIO_V1_StreamGetNext() function has been called 
    will cause Audio v1.0 client driver to reset the audio stream group to point
    to the first stream in the stream group.
    
  Precondition:
    The USB_HOST_AUDIO_V1_StreamGetFirst() function must have been called
    before calling this function.

  Parameters:
    audioDeviceObj   - Audio v1.0 client driver object

    streamGroupIndex - Stream Group Index.  
    
    streamInfo       -  Pointer to streamInfo object 

  Returns:
    USB_HOST_AUDIO_V1_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - The specified Audio v1.0 client
    driver object does not exist.
    USB_HOST_AUDIO_V1_STREAM_RESULT_FAILURE - An unknown failure occurred.
    USB_HOST_AUDIO_V1_RESULT_END_OF_STREAM_LIST - There are no more audio 
    streams in the stream group. 
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/ 
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetNext 
(
   USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
   USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
   USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObjCurrent, 
   USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *interfaceSettingObjNext
)
{
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE*)audioDeviceObj;
    uint32_t aSIntrfcIdx;
    uint32_t aSIntrfcSettingIdxCurrent, aSIntrfcSettingIdxNext ;
    
    /* NULL check */ 
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    if (interfaceSettingObjNext == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    /* Get the Streaming Interface Index from the Streaming Interface Object */ 
    aSIntrfcIdx = (uint8_t)(interfaceSettingObjCurrent>>8); 
    
    /* Verify Audio Streaming Interface Object */ 
    if (aSIntrfcIdx >= audioInstanceInfo->nASInterfaces)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Get the Interface Setting Index */ 
    aSIntrfcSettingIdxCurrent = (uint8_t)(interfaceSettingObjCurrent>>16); 
    
    /* Verify Audio streaming interface Setting Index */ 
    if (aSIntrfcSettingIdxCurrent >= audioInstanceInfo->streamInf[aSIntrfcIdx].nInterfaceSetting)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    aSIntrfcSettingIdxNext = aSIntrfcSettingIdxCurrent + 1; 
    if (aSIntrfcSettingIdxNext == audioInstanceInfo->streamInf[aSIntrfcIdx].nInterfaceSetting)
    {
        return USB_HOST_AUDIO_V1_RESULT_END_OF_INTERFACE_SETTINGS; 
    }
    
    /* Fill the Audio streaming interface setting object */  
    *interfaceSettingObjNext = (USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ)(audioInstanceInfo->index | aSIntrfcIdx<<8 | aSIntrfcSettingIdxNext<<16 );  
    
    /* return success */ 
    return USB_HOST_AUDIO_V1_RESULT_SUCCESS;   
}

// ****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_STREAM_HANDLE USB_HOST_AUDIO_V1_StreamOpen
    ( 
        USB_HOST_AUDIO_V1_STREAM_OBJ audioStreamObj  
    );
           
  Summary:
    This function opens the specified Audio Stream. 

  Description:
    This function will open the specified Audio Stream. Once opened, the Audio
    stream can be accessed via the handle which this function returns. The
    audioStreamObj parameter is the value returned in the
    USB_HOST_AUDIO_V1_StreamGetFirst() or USB_HOST_AUDIO_V1_StreamGetNext()
    functions.

  Precondition:
    Audio stream object should be valid.

  Input:
    audioStreamObj - Audio Stream object. 

  Return:
    Will return a valid handle if the audio stream could be opened successfully,
    else will return USB_HOST_AUDIO_V1_STREAM_RESULT_HANDLE_INVALID. The 
    function will return a valid handle if the stream is ready to be opened.
    
  Example:
    <code>
    </code>

  Remarks:
    None.                                                                   
*/
USB_HOST_AUDIO_V1_STREAM_HANDLE USB_HOST_AUDIO_V1_StreamOpen
( 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ audiostreamingInterfaceObj  
)
{
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle = USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID;   
    
    /* The present implementation is a single client implementation only */
    streamHandle = (USB_HOST_AUDIO_V1_STREAM_HANDLE)audiostreamingInterfaceObj; 
       
    return streamHandle; 
}

// ****************************************************************************
/* Function:
    void USB_HOST_AUDIO_V1_StreamClose
    ( 
        USB_HOST_AUDIO_V1_STREAM_HANDLE audioSteamHandle
    );
           
  Summary:
    This function closes the Audio Stream. 

  Description:
    This function will close the open Audio Stream. This closes the association
    between the application entity that opened the audio stream and the audio 
    stream. The audio stream handle becomes invalid.

  Precondition:
    None.

  Input:
    audioSteamHandle - handle to the audio stream obtained from the
    USB_HOST_AUDIO_V1_StreamOpen() function.

  Return:
    None.
    
  Example:
    <code>
    </code>

  Remarks:
    The device handle becomes invalid after calling this function.                                                                   
*/
void USB_HOST_AUDIO_V1_StreamClose
( 
    USB_HOST_AUDIO_V1_STREAM_HANDLE audioSteamHandle
)
{
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex;  
    
    /* Find Audio Stream from audioStreamObj */
    if (audioSteamHandle != USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID)
    {
        audioInstanceIndex = (uint8_t)audioSteamHandle;
        asIntrefaceIndex = (uint8_t)(audioSteamHandle>>8);  
        
        audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex];
        asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex]; 

        if (asInterface->streamEventHandler != (USB_HOST_AUDIO_V1_STREAM_EVENT_HANDLER)NULL)
        {
            asInterface->streamEventHandler = NULL;
            asInterface->context = (uintptr_t)NULL; 
        }
        
    }
    
}
// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSet
    (
        USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle, 
        USB_INTERFACE_DESCRIPTOR* pInterfaceDesc,
        USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle
    ); 

  Summary:
    Schedules a SET_INTERFACE request to the specified audio stream. 

  Description:
    This function schedules an Audio stream enable request for the specified 
    audio stream. An audio stream must be enable before scheduling any data 
    transfer with the stream. An event 
    USB_HOST_AUDIO_V1_STREAM_EVENT_ENABLE_COMPLETE is generated when  this 
    request is completed. USB_HOST_AUDIO_V1_STREAM_EVENT_ENABLE_COMPLETE_DATA 
    returns the status and request handle of the request. 
        
       
  Precondition:
    Audio stream should have been opened. Only one audio stream from an audio
    stream group can be enabled at a time. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 Stream.
    requestHandle - Handle to the Stream Enable request. 

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSet
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle, 
    USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    uint8_t alternateSetting; 
    USB_HOST_RESULT hostResult; 
    USB_HOST_AUDIO_V1_RESULT result; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    
    /* Retrieve the Audio Instance Array Index from the Stream Handle */
    audioInstance = (uint8_t)interfaceSettingObj; 
    
    /* Retrieve the Audio Interface Array Index */
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8);
    
    /*Retrieve the Audio Interface Setting Array Index */
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    /* Get pointer to Audio Instance */
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    /* Perform NULL check on pointer to Audio Instance */ 
    if (audioInstanceInfo == NULL)
    {
        /* Pointer to Audio Instance is invalid. Cannot proceed.*/
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }

    /* Check if there is any Audio Streaming Interface present in the Audio Device */ 
    if (audioInstanceInfo->nASInterfaces == 0)
    {
        /* The Audio instance has no Streaming interfaces. Cannot proceed.*/
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Verify Audio Streaming Interface Object */ 
    if (aSIntrfcIdx >= audioInstanceInfo->nASInterfaces)
    {
        /* Audio streaming interface array index is invalid. Cannot proceed */
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Verify Audio streaming interface Setting Array Index */ 
    if (asIntrfcSettingIdx >= audioInstanceInfo->streamInf[aSIntrfcIdx].nInterfaceSetting)
    {
        /* Audio streaming interface array index is invalid. Cannot proceed.*/
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    } 

    /* Retrieve  the alternate Setting Number from the Audio Instance */ 
    alternateSetting = audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].interfaceAlternateSetting; 
    
    /* Place an Set Interface Request to the USB Host Layer */ 
    hostResult = USB_HOST_DeviceInterfaceSet 
                 (
                     audioInstanceInfo->streamInf[aSIntrfcIdx].asInterfaceHandle, 
                     requestHandle, 
                     alternateSetting,
                     (uintptr_t)alternateSetting | (uintptr_t)USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_INTERFACE_SET << 8
                 ); 
    
    switch (hostResult)
    {
        case USB_HOST_RESULT_SUCCESS:
            result = USB_HOST_AUDIO_V1_RESULT_SUCCESS;
            break; 
        case USB_HOST_RESULT_REQUEST_BUSY:
            result = USB_HOST_AUDIO_V1_RESULT_BUSY; 
            break; 
        default:
            result = USB_HOST_AUDIO_V1_RESULT_FAILURE; 
            break; 
    }
    return result; 
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_STREAM_RESULT USB_HOST_AUDIO_V1_StreamSamplingFrequencySet
    (
        USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
        USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
        uint32_t samplingRate
    ); 

  Summary:
    Schedules an  Audio stream Set Sampling rate request for the specified 
    audio stream. 

  Description:
    This function schedules an  Audio stream Set Sampling rate request for the
    specified audio stream. An event 
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE is generated 
    when this request is completed. 
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA returns 
    the status and request handle of the request. 
        
       
  Precondition:
    Audio stream should have been opened. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 Stream.
    requestHandle - Handle to the Stream Set Sampling rate request 

  Returns:
    USB_HOST_AUDIO_V1_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamSamplingFrequencySet
(  
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_REQUEST_HANDLE *requestHandle,
    uint32_t *samplingFrequency
)
{
    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAM_SETTING* audioStream; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    uint8_t alternateSetting; 
    USB_HOST_RESULT hostResult; 
    USB_HOST_AUDIO_V1_CONTROL_TRANSFER_OBJ* audioControlObj; 
    USB_AUDIO_ENDPOINT_CONTROL_REQUEST* setupPacket; 
    OSAL_CRITSECT_DATA_TYPE IntState;
    
    if (streamHandle == USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID)
    {
        return USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER; 
    }
    
    /* Find Audio Stream from audioStreamObj */
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID; 
    }
    
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex];
    if (asInterface == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID;
    }
    
    alternateSetting = asInterface->activeInterfaceSetting; 
    
    audioStream = &asInterface->audioStreamSetting[alternateSetting]; 
    if (audioStream == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID;
    }
    
    
    audioControlObj = &asInterface->audioControlObj; 
    if (audioControlObj == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID;
    }
    
    setupPacket = (USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)&asInterface->setupPacket; 
    
    if (audioControlObj->inUse == true)
    {
        return USB_HOST_AUDIO_V1_RESULT_BUSY; 
    }
    
    if (audioStream->nEndpoints == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_FAILURE; 
    }
    /* Prevent other tasks pre-empting this sequence of code */ 
    IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
   
    audioControlObj->inUse = true; 
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
    
    //Class Specific Endpoint Request, Direction Host to Device
    setupPacket->bmRequestType =   USB_SETUP_DIRN_HOST_TO_DEVICE
                                | USB_SETUP_TYPE_CLASS
                                | USB_SETUP_RECIPIENT_ENDPOINT ;

    setupPacket->bRequest = USB_AUDIO_CS_SET_CUR;
    setupPacket->controlSelector = USB_AUDIO_SAMPLING_FREQ_CONTROL;
    
    setupPacket->endpointNumber = audioStream->isoDataEndpointDesc.bEndpointAddress;
    
    setupPacket->wLength = 3;
    
    hostResult = USB_HOST_DeviceControlTransfer
             (
                 audioInstanceInfo->controlPipeHandle,
                 requestHandle,
                 (USB_SETUP_PACKET *)setupPacket,
                 (void *)samplingFrequency,
                 _USB_HOST_AUDIO_V1_SetSampleFrequencyCallback,
                 (uintptr_t)streamHandle
             ); 
    
    return hostResult;
    
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamSamplingFrequencyGet
    (
        USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
        USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
        uint32_t *samplingFrequency
    )
  Summary:
    Schedules an  Audio stream Get Sampling rate request for the specified 
    audio stream. 

  Description:
    This function schedules an  Audio stream Set Sampling rate request for the
    specified audio stream. An event 
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE is generated 
    when this request is completed. 
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA returns 
    the status and request handle of the request. 
        
       
  Precondition:
    Audio stream should have been opened. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 Stream.
    requestHandle - Handle to the Stream Set Sampling rate request
    samplingRate  - Pointer to Sampling Rate

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamSamplingFrequencyGet
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_REQUEST_HANDLE *requestHandle,
    uint32_t *samplingFrequency
)
{

    USB_HOST_AUDIO_V1_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAM_SETTING* audioStream; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    uint8_t alternateSetting; 
    USB_HOST_RESULT hostResult; 
    USB_HOST_AUDIO_V1_CONTROL_TRANSFER_OBJ* audioControlObj; 
    USB_AUDIO_ENDPOINT_CONTROL_REQUEST* setupPacket; 
    OSAL_CRITSECT_DATA_TYPE IntState;
    
    if (streamHandle == USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID)
    {
        return USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER; 
    }
    
    /* Find Audio Stream from audioStreamObj */
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID; 
    }
    
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex];
    if (asInterface == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID;
    }
    
    alternateSetting = asInterface->activeInterfaceSetting; 
    
    audioStream = &asInterface->audioStreamSetting[alternateSetting]; 
    if (audioStream == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID;
    }
    
    
    audioControlObj = &asInterface->audioControlObj; 
    if (audioControlObj == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID;
    }
    
    setupPacket = (USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)&asInterface->setupPacket; 
    
    if (audioControlObj->inUse == true)
    {
        return USB_HOST_AUDIO_V1_RESULT_BUSY; 
    }
    
    if (audioStream->nEndpoints == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_FAILURE; 
    }
    /* Prevent other tasks pre-empting this sequence of code */ 
    IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
   
    audioControlObj->inUse = true; 
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
    
    //Class Specific Endpoint Request, Direction Host to Device
    setupPacket->bmRequestType =   USB_SETUP_DIRN_DEVICE_TO_HOST
                                | USB_SETUP_TYPE_CLASS
                                | USB_SETUP_RECIPIENT_ENDPOINT ;

    setupPacket->bRequest = USB_AUDIO_CS_GET_CUR;
    setupPacket->controlSelector = USB_AUDIO_SAMPLING_FREQ_CONTROL;
    
    setupPacket->endpointNumber = audioStream->isoDataEndpointDesc.bEndpointAddress;
    
    setupPacket->wLength = 3;
    
    hostResult = USB_HOST_DeviceControlTransfer
             (
                 audioInstanceInfo->controlPipeHandle,
                 requestHandle,
                 (USB_SETUP_PACKET *)setupPacket,
                 (void *)samplingFrequency,
                 _USB_HOST_AUDIO_V1_SetSampleFrequencyCallback,
                 (uintptr_t)streamHandle
             ); 
    
    return hostResult;
    
}

// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_REESULT USB_HOST_AUDIO_V1_ControlEntityGetFirst
   (
       USB_HOST_AUDIO_V1_OBJ  audioObj, 
       USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ * pEntityHandle
   ); 

  Summary:
    Retrieves handle to the first Audio Control entity   

  Description:
    This function retrieves handle to first Audio Control entity. 
    
  Precondition:
    

  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    pEntityHandle - pointer to the Audio control entity handle

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_RESULT_END_OF_CONTROL_ENTITY - No more audio control
    entity. 
    USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>
  Remarks:
    None.

*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_ControlEntityGetFirst
(
    USB_HOST_AUDIO_V1_OBJ  audioDeviceObj, 
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ * pEntityHandle
)
{
    uint8_t * pDescriptor; 
    
    /* Get Pointer to Audio Device Instance  */
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE*)audioDeviceObj;
    
    /* Null check */
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Get pointer to Audio Control Descriptor */ 
    pDescriptor = audioInstanceInfo->pAudioContorldescriptor; 
    
    if (pDescriptor == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_FAILURE; 
    }
    
    /* First Descriptor in the Audio Control is always the Header. Get the next one */ 
    *pEntityHandle = (USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ)((uint8_t *)pDescriptor +  ((USB_INTERFACE_DESCRIPTOR*)pDescriptor)->bLength);
    
    /* Return Success */ 
    return USB_HOST_AUDIO_V1_RESULT_SUCCESS; 
}

// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_REESULT USB_HOST_AUDIO_V1_ControlEntityGetNext
   (
       USB_HOST_AUDIO_V1_OBJ  audioObj, 
       USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityHandleCurrent
       USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ * pEntityHandle
   ); 

  Summary:
    Retrieves handle to the next Audio Control entity.

  Description:
    This function retrieves handle to the next Audio Control entity. 
    
  Precondition:
    

  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityHandleCurrent - Handle to current Audio control entity. 

    pEntityHandle  -  pointer to Audio control entity Handle. 


  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful

    USB_HOST_AUDIO_V1_RESULT_END_OF_CONTROL_ENTITY - No more audio control
    entity. 
    USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_ControlEntityGetNext
(
    USB_HOST_AUDIO_V1_OBJ  audioDeviceObj, 
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObjectCurrent,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ * pEntityObject
)
{   
    USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR *pDescriptor = (USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR *) entityObjectCurrent; 
            
    /* Get Pointer to Audio Device Instance  */
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE*)audioDeviceObj;
    
    /* Null check */
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    if (pDescriptor == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    *pEntityObject = (USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ)((uint8_t *)pDescriptor +  ((USB_INTERFACE_DESCRIPTOR*)pDescriptor)->bLength);
    
    return USB_HOST_AUDIO_V1_RESULT_SUCCESS;
}


// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_EntityObjectGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        uint8_t entityId,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* entityObj
    ); 

  Summary:
    Retrieves Entity Object for Entity ID.

  Description:
    This function retrieves Entity Object for Entity ID.  
    
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 
 
    entityId - Entity ID

    entityObject - Audio control entity Object 

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    
    USB_HOST_AUDIO_V1_RESULT_FAILURE - Entity Id could not e found. Or 
    An unknown failure occurred.
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_EntityObjectGet
(
    USB_HOST_AUDIO_V1_OBJ  audioDeviceObj,
    uint8_t entityId,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* entityObj
)
{
    uint8_t *pDescriptor; 
    uint16_t descriptorCount; 
    USB_HOST_AUDIO_V1_RESULT audioResult = USB_HOST_AUDIO_V1_RESULT_FAILURE; 
    
   /* Get Pointer to Audio Device Instance  */
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE*)audioDeviceObj;
    
    /* Null check */
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    /* Get pointer to Audio Control Descriptor */ 
    pDescriptor = audioInstanceInfo->pAudioContorldescriptor;
    
    pDescriptor = pDescriptor + ((USB_INTERFACE_DESCRIPTOR*)pDescriptor)->bLength;  
    
    descriptorCount = ((USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR*)pDescriptor)->wTotalLength; 
    
    pDescriptor = pDescriptor + ((USB_INTERFACE_DESCRIPTOR*)pDescriptor)->bLength;
    
    while(descriptorCount)
    {
        if (entityId == ((USB_HOST_AUDIO_CONTROL_ENTITY_DESCRIPTOR_HEADER*)pDescriptor)->entityID)
        {
            *entityObj = (USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ)pDescriptor;
            audioResult = USB_HOST_AUDIO_V1_RESULT_SUCCESS; 
            break; 
        }

        pDescriptor = pDescriptor + ((USB_INTERFACE_DESCRIPTOR*)pDescriptor)->bLength; 
        descriptorCount = descriptorCount - ((USB_INTERFACE_DESCRIPTOR*)pDescriptor)->bLength; 
    }
    
    return audioResult; 
}
// *****************************************************************************
/* Function:
    USB_AUDIO_V1_ENTITY_TYPE USB_HOST_AUDIO_V1_EntityTypeGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj, 
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns Entity Type of the Audio Control entity.

  Description:
    This function returns Entity Type of the Audio Control entity. Prior to
    calling this function Entity Object should obtained by calling
    USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    USB_AUDIO_V1_ENTITY_TYPE 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_AUDIO_V1_ENTITY_TYPE USB_HOST_AUDIO_V1_EntityTypeGet
(
    USB_HOST_AUDIO_V1_OBJ  audioDeviceObj, 
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    USB_AUDIO_V1_ENTITY_TYPE entityType; 
    
    USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR *pDescriptor = (USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR *) entityObject; 
     
    if (pDescriptor == NULL)
    {
        return 0; 
    }

    entityType = pDescriptor->bDescriptorSubtype; 
    
    return entityType;  
}

// *****************************************************************************
/* Function:
    USB_AUDIO_V1_TERMINAL_TYPE USB_HOST_AUDIO_V1_TerminalTypeGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns Terminal Type of the Audio Control entity.

  Description:
    This function returns Terminal type of the Audio Control entity. Prior to
    calling this function Entity Object should obtained by calling
    USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    Terminal Type 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_AUDIO_V1_TERMINAL_TYPE USB_HOST_AUDIO_V1_TerminalTypeGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    USB_AUDIO_V1_TERMINAL_TYPE terminalType; 
    
    USB_HOST_AUDIO_V1_TERMINAL_HEADER_DESCRIPTOR *pDescriptor = (USB_HOST_AUDIO_V1_TERMINAL_HEADER_DESCRIPTOR *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return 0; 
    }
        
    terminalType = pDescriptor->wTerminalType; 
    
    return terminalType; 
}

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_TerminalIDGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns Terminal ID of the Audio Control entity.

  Description:
    This function returns Terminal ID of the Audio Control entity. Prior to
    calling this function Entity Object should obtained by calling
    USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    The terminal ID.
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
 
uint8_t USB_HOST_AUDIO_V1_TerminalIDGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    uint8_t terminalID; 
    
    USB_HOST_AUDIO_V1_TERMINAL_HEADER_DESCRIPTOR *pDescriptor = (USB_HOST_AUDIO_V1_TERMINAL_HEADER_DESCRIPTOR *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return 0; 
    }
        
    terminalID = pDescriptor->bTerminalID; 
    
    return terminalID;
}
// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_TerminalAssociationGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns Associated Terminal of the Audio Control Terminal.

  Description:
    This function returns ID Associated Terminal type of the Audio Control
    Terminal.  Prior to calling this function Entity Object should obtained by
    calling USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    ID of the Associated Terminal  
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_TerminalAssociationGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    uint8_t bAssocTerminal; 
    
    USB_HOST_AUDIO_V1_TERMINAL_HEADER_DESCRIPTOR *pDescriptor = (USB_HOST_AUDIO_V1_TERMINAL_HEADER_DESCRIPTOR *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return 0; 
    }
        
    bAssocTerminal = pDescriptor->bAssocTerminal; 
    
    return bAssocTerminal; 

}

// *****************************************************************************
/* Function:
    USB_AUDIO_CHANNEL_CONFIG USB_HOST_AUDIO_V1_TerminalInputChannelConfigGet
	(
		USB_HOST_AUDIO_V1_OBJ  audioObj,
		USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
	); 

  Summary:
    Returns a structure which describes the spatial location of the logical 
	channels of in the Terminal's output audio channel cluster.
	
  Description:
    This function returns a structure which describes the spatial location of 
	the logical channels of in the Terminal's output audio channel cluster. 
	This function is only applicable to Input Terminal.  Prior to calling this
	function Entity Object should obtained by calling 
	USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    Structure which describes the spatial location of the logical channels 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_AUDIO_CHANNEL_CONFIG USB_HOST_AUDIO_V1_TerminalInputChannelConfigGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    USB_AUDIO_CHANNEL_CONFIG channelConfig; 
    channelConfig.value = 0; 
    USB_AUDIO_INPUT_TERMINAL_DESCRIPTOR *pDescriptor = (USB_AUDIO_INPUT_TERMINAL_DESCRIPTOR *) entityObject;
    
    if (pDescriptor != NULL)
    {
        channelConfig.value = pDescriptor->wChannelConfig; 
    }
    return channelConfig; 
}
// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_TerminalInputChannelNumbersGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns Number of logical output channels in the Terminal's output audio 
    channel cluster.

  Description:
    This function returns Number of logical output channels in the Terminal's
    output audio channel cluster. This function is only applicable to Input
    Terminal.  Prior to calling this function Entity Object should obtained by
    calling USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    Number of logical output channels in the Terminal's output audio channel 
    cluster.  
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_TerminalInputChannelNumbersGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    uint8_t bNrChannels; 
    
    USB_AUDIO_INPUT_TERMINAL_DESCRIPTOR *pDescriptor = (USB_AUDIO_INPUT_TERMINAL_DESCRIPTOR *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return 0; 
    }
        
    bNrChannels = pDescriptor->bNrChannels; 
    
    return bNrChannels; 

}
// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_TerminalSourceIDGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns ID of the Unit or Terminal to which this Terminal is connected.

  Description:
    This function returns ID of the Unit or Terminal to which this Terminal is
    connected. This function is only applicable to Output Terminal.  Prior to
    calling this function Entity Object should obtained by calling
    USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    ID of the Unit or Terminal to which this Terminal is connected. 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_TerminalSourceIDGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    uint8_t bSourceID; 
    
    USB_AUDIO_OUTPUT_TERMINAL_DESCRIPTOR *pDescriptor = (USB_AUDIO_OUTPUT_TERMINAL_DESCRIPTOR *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return 0; 
    }
        
    bSourceID = pDescriptor->bSourceID; 
    
    return bSourceID; 
}

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_FeatureUnitIDGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns ID of the Feature Unit.

  Description:
    This function returns ID of the D of the Feature Unit.  This function is
    only applicable to Feature Unit. Prior to calling this function Entity
    Object should obtained by calling
    USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    ID of Feature Unit. 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_FeatureUnitIDGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
        uint8_t unitId; 
    
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return 0; 
    }
        
    unitId = pDescriptor->bUnitID; 
    
    return unitId;
}


// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_FeatureUnitSourceIDGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    );

  Summary:
    Returns ID of the Unit or Terminal to which this Feature Unit is connected.

  Description:
    This function returns ID of the Unit or Terminal to which this Feature Unit
    is connected. This function is only applicable to Feature Unit. Prior to
    calling this function Entity Object should obtained by calling
    USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    ID of the Unit or Terminal to which this Feature Unit is connected. 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_FeatureUnitSourceIDGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    uint8_t bSourceID; 
    
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return 0; 
    }
        
    bSourceID = pDescriptor->bSourceID; 
    
    return bSourceID; 

}

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_FeatureUnitChannelNumbersGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns Number of channels.

  Description:
    This function returns number of channels. This function is only applicable
    to Feature Unit. Prior to calling this function Entity Object should
    obtained by calling USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 

  Returns:
    Number of channels. 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_FeatureUnitChannelNumbersGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
)
{
    uint8_t nChannels; 
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return 0; 
    }
    
    if (pDescriptor->bControlSize == 0)
    {
        return 0; 
    }
    
    nChannels = (pDescriptor->bLength - sizeof(USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER) - 1)/pDescriptor->bControlSize - 1; 
    
    return nChannels; 
}

// *****************************************************************************
/* Function:
    bool USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists
    (
         USB_HOST_AUDIO_V1_OBJ  audioObj,
         USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
         uint8_t channel
    );

  Summary:
    Returns true if Mute Control exists for the specified channel of the Feature Unit. 

  Description:
    This function returns true if Mute Control exists on the specified channel
    of the Feature Unit.  Channel 0 would return indicates Master mute control.
    This function is only applicable to Feature Unit. Prior to calling this
    function Entity Object should obtained by calling
    USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 
 
    channel - Channel Number. 

  Returns:
    true - Mute control exists on the specified channel. 
    false - Mute control does not exist on the specified channel. 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
bool USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     uint8_t channel
)
{
    bool result = false; 
    uint8_t nChannels;
    uint8_t * pMuteControl; 
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return result; 
    }
    
    if (pDescriptor->bControlSize == 0)
    {
        return result; 
    }
    
    nChannels = (pDescriptor->bLength - sizeof(USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER) - 1)/pDescriptor->bControlSize - 1; 
    
    if (channel > nChannels )
    {
        return result; 
    }
    
    pMuteControl = (uint8_t*)pDescriptor + sizeof(USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER) + pDescriptor->bControlSize*channel; 
    
    if (((USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS*)pMuteControl)->mute == 1)
    {
        result = true; 
    }
    else
    {
        result = false; 
    }
    
    return result; 
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelMuteSet
    (
         USB_HOST_AUDIO_V1_OBJ  audioObj,
         USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
         USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
         uint8_t channelNumber,
         bool *muteStatus
    ); 

  Summary:
    Schedules a Set Mute control request to the specified channel.  

  Description:
    This function schedules a set Mute control request to the specified
    channel.  Prior to calling this function user should check if mute control
    exists on the specified channel by calling
    USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists() function. 
 
    If the request was scheduled successfully, requestHandle will contain a
    request handle that uniquely identifies this transfer. If the transfer
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 client driver will call
    callback function which was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet() function. The context
    parameter specified here will be returned in the callback.

  Parameters:
    audioObj     - USB Host Audio v1.0 Device Object. 

    entityObject - Audio Control Entity Object 
  
    requestHandle - Output Parameter that will contain the Handle to this request.
 
    channelNumber - Channel Number 
   
    muteStatus -  Value of Mute control, 1 Mutes the channel, 0 removes Mute. 

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - the request was scheduled successfully.
    requestHandle will contain a valid request handle.
 
    USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently busy. 
    Retry the request. 
 
    USB_HOST_AUDIO_V1_RESULT_FAILURE - an unknown failure occurred. requestHandle will
    contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
 
    USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or requestHandle pointer
    is NULL.
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelMuteSet
(
     USB_HOST_AUDIO_V1_OBJ  audioDeviceObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
     bool *muteStatus
)
{
    USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST setupPacket;
    USB_HOST_AUDIO_V1_RESULT result; 
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    USB_INTERFACE_DESCRIPTOR * audioControlInterfaceDescriptor; 
    USB_HOST_AUDIO_V1_INSTANCE *audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE *)audioDeviceObj;
    
    if (pDescriptor == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    if (pDescriptor->bControlSize == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    
    /* Fill in Setup Packet */
    setupPacket.bmRequestType = (  USB_SETUP_DIRN_HOST_TO_DEVICE
                   | USB_SETUP_TYPE_CLASS
                   | USB_SETUP_RECIPIENT_INTERFACE
                  ); //interface , Host to device , Standard;
    
    setupPacket.bRequest = USB_AUDIO_CS_SET_CUR;
    
    setupPacket.channelNumber = channelNumber; 
    
    setupPacket.controlSelector = USB_AUDIO_MUTE_CONTROL;
    
    setupPacket.featureUnitId = pDescriptor->bUnitID; 
    
    /* Fill in Interface Number */
    audioControlInterfaceDescriptor = (USB_INTERFACE_DESCRIPTOR*)audioInstanceInfo->pAudioContorldescriptor; 
    setupPacket.interfaceNumber = audioControlInterfaceDescriptor->bInterfaceNumber; 
    
    setupPacket.wLength = 1;
    
    result = _USB_HOST_AUDIO_V1_ControlRequest
    (
        audioDeviceObj,
        requestHandle,
        (USB_SETUP_PACKET *)&setupPacket,
        muteStatus
    ); 
    
    return result; 
}


// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelMuteGet
    (
         USB_HOST_AUDIO_V1_OBJ  audioObj,
         USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
         USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
         uint8_t channelNumber,
         bool *muteStatus
    ); 

  Summary:
    Schedules a Get Mute control request to the specified channel.  

  Description:
    This function schedules a Get Mute control request to the specified
    channel. Prior to calling this function user should check if mute control
    exists on the specified channel by calling
    USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists() function. 
 
    If the request was scheduled successfully, requestHandle will contain a
    request handle that uniquely identifies this request. If the transfer
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 client driver will call
    callback function which was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet() function. The context
    parameter specified here will be returned in the callback.
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device Object. 

    entityObject - Audio Control Entity Object 
  
    requestHandle - Output Parameter that will contain the Handle to this request.
 
    channelNumber - Channel Number
   
    muteStatus -  Output Parameter that will contain Current Mute status when
    Request is completed and a Callback is received.    

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - the request was scheduled successfully.
    requestHandle will contain a valid request handle.
 
    USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently
    busy.  Retry the request. 
 
    USB_HOST_AUDIO_V1_RESULT_FAILURE - an unknown failure occurred.
    requestHandle will contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
 
    USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or
    requestHandle pointer is NULL.
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelMuteGet
(
     USB_HOST_AUDIO_V1_OBJ  audioDeviceObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
     bool *muteStatus
)
{
    USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST setupPacket;
    USB_HOST_AUDIO_V1_RESULT result; 
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    USB_INTERFACE_DESCRIPTOR * audioControlInterfaceDescriptor; 
    USB_HOST_AUDIO_V1_INSTANCE *audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE *)audioDeviceObj;
    
    if (pDescriptor == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    if (pDescriptor->bControlSize == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    
    /* Fill in Setup Packet */
    setupPacket.bmRequestType = (  USB_SETUP_DIRN_DEVICE_TO_HOST
                   | USB_SETUP_TYPE_CLASS
                   | USB_SETUP_RECIPIENT_INTERFACE
                  ); //interface , Host to device , Standard;
    
    setupPacket.bRequest = USB_AUDIO_CS_GET_CUR;
    
    setupPacket.channelNumber = channelNumber; 
    
    setupPacket.controlSelector = USB_AUDIO_MUTE_CONTROL;
    
    setupPacket.featureUnitId = pDescriptor->bUnitID; 
    
    /* Fill in Interface Number */
    audioControlInterfaceDescriptor = (USB_INTERFACE_DESCRIPTOR*)audioInstanceInfo->pAudioContorldescriptor; 
    setupPacket.interfaceNumber = audioControlInterfaceDescriptor->bInterfaceNumber; 
    
    setupPacket.wLength = 1;
    
    result = _USB_HOST_AUDIO_V1_ControlRequest
    (
        audioDeviceObj,
        requestHandle,
        (USB_SETUP_PACKET *)&setupPacket,
        muteStatus
    ); 
    
    return result;
    
}

// *****************************************************************************
/* Function:
    bool USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists
    (
         USB_HOST_AUDIO_V1_OBJ  audioObj,
         USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
         uint8_t channel
    ); 

  Summary:
    Returns true if Volume Control exists for the specified channel of the
    Feature Unit. 

  Description:
    This function returns true if Volume Control exists on the specified
    channel of the Feature Unit. Channel 0 indicates Master Volume control.
    This function is only applicable to Feature Unit. Prior to calling this
    function Entity Object should obtained by calling
    USB_HOST_AUDIO_V1_ControlEntityGetFirst(),
    USB_HOST_AUDIO_V1_ControlEntityGetNext() or
    USB_HOST_AUDIO_V1_EntityObjectGet() function.
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 
 
    channel - Channel Number 

  Returns:
    true - Volume control exists on the specified channel. 
    false - Volume control does not exist on the specified channel. 
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/

bool USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     uint8_t channel
)
{
    bool result = false; 
    uint8_t nChannels;
    uint8_t * pMuteControl; 
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    
    if (pDescriptor == NULL)
    {
        return result; 
    }
    
    if (pDescriptor->bControlSize == 0)
    {
        return result; 
    }
    
    nChannels = (pDescriptor->bLength - sizeof(USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER) - 1)/pDescriptor->bControlSize - 1; 
    
    if (channel > nChannels )
    {
        return result; 
    }
    
    pMuteControl = (uint8_t*)pDescriptor + sizeof(USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER) + pDescriptor->bControlSize*channel; 
    
    if (((USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS*)pMuteControl)->volume== 1)
    {
        result = true; 
    }
    else
    {
        result = false; 
    }
    
    return result; 

}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet
    (
         USB_HOST_AUDIO_V1_OBJ  audioObj,
         USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
         USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
         uint8_t channelNumber,
         uint16_t *volume
    );

  Summary:
    Schedules a Set Current Volume control request to the specified channel.  

  Description:
    This function schedules a Set Current Volume request to the specified
    channel.  Prior to calling this function user should check if volume
    control exists on the specified channel by calling
    USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists() function. 
 
    If the request was scheduled successfully, requestHandle will contain a
    request handle that uniquely identifies this request. If the request
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 client driver will call
    callback function which was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet() function. The context
    parameter specified here will be returned in the callback. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device Object. 

    entityObject - Audio Control Entity Object 
  
    requestHandle - Output Parameter that will contain the Handle to this
    request.
 
    channelNumber - Channel Number to which the volume control is addressed to. 
   
    volume -  Current Volume control value that should be set in the Audio
    Device. 

  Returns:
    USB_HOST_AUDIO_V1_RESULT_SUCCESS - the request was scheduled successfully.
    requestHandle will contain a valid request handle.
 
    USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently
    busy.  Retry the request. 
 
    USB_HOST_AUDIO_V1_RESULT_FAILURE - an unknown failure occurred.
    requestHandle will contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
 
    USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or
    requestHandle pointer is NULL.
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet
(
     USB_HOST_AUDIO_V1_OBJ  audioDeviceObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
     uint16_t *volume
)
{
    USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST setupPacket;
    USB_HOST_AUDIO_V1_RESULT result; 
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    USB_INTERFACE_DESCRIPTOR * audioControlInterfaceDescriptor; 
    USB_HOST_AUDIO_V1_INSTANCE *audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE *)audioDeviceObj; 
    
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    
    if (pDescriptor == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    if (pDescriptor->bControlSize == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    
    /* Fill in Setup Packet */
    setupPacket.bmRequestType = (  USB_SETUP_DIRN_HOST_TO_DEVICE
                   | USB_SETUP_TYPE_CLASS
                   | USB_SETUP_RECIPIENT_INTERFACE
                  ); //interface , Host to device , Standard;
    
    setupPacket.bRequest = USB_AUDIO_CS_SET_CUR;
    
    setupPacket.channelNumber = channelNumber; 
    
    setupPacket.controlSelector = USB_AUDIO_VOLUME_CONTROL;
    
    setupPacket.featureUnitId = pDescriptor->bUnitID; 
   
    /* Fill in Interface Number */
    audioControlInterfaceDescriptor = (USB_INTERFACE_DESCRIPTOR*)audioInstanceInfo->pAudioContorldescriptor; 
    setupPacket.interfaceNumber = audioControlInterfaceDescriptor->bInterfaceNumber; 
    
    setupPacket.wLength = 2;
    
    result = _USB_HOST_AUDIO_V1_ControlRequest
    (
        audioDeviceObj,
        requestHandle,
        (USB_SETUP_PACKET *)&setupPacket,
        volume
    ); 
    
    return result;
}


// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeGet
    (
         USB_HOST_AUDIO_V1_OBJ  audioObj,
         USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
         USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
         uint8_t channelNumber,
         uint16_t *volume
    );

  Summary:
    Schedules a Get Current Volume control request to the specified Channel.  

  Description:
    This function schedules a Get Current Volume Control request to the
    specified channel. Prior to calling this function user should check if
    volume control exists on the specified channel by calling
    USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists() function. 
 
    If the request was scheduled successfully, requestHandle will contain a
    request handle that uniquely identifies this request. If the request
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 client driver will call
    callback function which was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet() function. The context
    parameter specified here will be returned in the callback. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 

    entityObject - Audio control entity Object 
  
    requestHandle - Output parameter that will contain the handle to this
    request.
 
    channelNumber - Channel Number to which the volume control is addressed to. 
   
    volume -  Output Parameter that will contain Current Volume when request is
    completed and a callback is received. 

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - the request was scheduled successfully.
      requestHandle will contain a valid request handle.
    - USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently
      busy.  Retry the request. 
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - an unknown failure occurred
      requestHandle will contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID
    - USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or
      requestHandle pointer is NULL
    
  Example:
    <code>
    </code>

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeGet
(
     USB_HOST_AUDIO_V1_OBJ  audioDeviceObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
     uint16_t *volume
)
{
    USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST setupPacket;
    USB_HOST_AUDIO_V1_RESULT result; 
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *pDescriptor = (USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER *) entityObject;
    USB_INTERFACE_DESCRIPTOR * audioControlInterfaceDescriptor; 
    USB_HOST_AUDIO_V1_INSTANCE *audioInstanceInfo = (USB_HOST_AUDIO_V1_INSTANCE *)audioDeviceObj; 
    
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    
    if (pDescriptor == NULL)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    if (pDescriptor->bControlSize == 0)
    {
        return USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID; 
    }
    
    
    /* Fill in Setup Packet */
    setupPacket.bmRequestType = (  USB_SETUP_DIRN_DEVICE_TO_HOST
                   | USB_SETUP_TYPE_CLASS
                   | USB_SETUP_RECIPIENT_INTERFACE
                  ); //interface , Host to device , Standard;
    
    setupPacket.bRequest = USB_AUDIO_CS_GET_CUR;
    
    setupPacket.channelNumber = channelNumber; 
    
    setupPacket.controlSelector = USB_AUDIO_VOLUME_CONTROL;
    
    setupPacket.featureUnitId = pDescriptor->bUnitID; 
   
    /* Fill in Interface Number */
    audioControlInterfaceDescriptor = (USB_INTERFACE_DESCRIPTOR*)audioInstanceInfo->pAudioContorldescriptor; 
    setupPacket.interfaceNumber = audioControlInterfaceDescriptor->bInterfaceNumber; 
    
    setupPacket.wLength = 2;
    
    result = _USB_HOST_AUDIO_V1_ControlRequest
    (
        audioDeviceObj,
        requestHandle,
        (USB_SETUP_PACKET *)&setupPacket,
        volume
    ); 
    
    return result;
}
// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceTerminalLinkGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    This function returns Terminal Link of the specified streaming interface
    setting. 
    
   Description:
    This function returns Terminal Link of the specified streaming interface
    setting. TODO Add more description. 
    
   Precondition:
    Audio v1.0 device should have been attached. 

   Parameters:
    audioObj - Audio Device Object

    streamingInterfaceObj - Audio Streaming Interface Object

    interfaceSettingObj - Audio Streaming Interface Setting Object 

   Returns:
     Terminal Link of the Audio Streaming Interface setting 
     
   Example:
    <code> 
    </code>

   Remarks:

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceTerminalLinkGet
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    
    
    audioInstance = (uint8_t)interfaceSettingObj; 
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8); 
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    if (audioInstanceInfo == NULL)
    {
        return 0; 
    }
    
    return (USB_AUDIO_V1_FORMAT_TAG)audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].bTerminalLink;
}

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceFormatTagGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    This function returns Format Tag of the specified streaming interface
    setting. 
    
   Description:
    This function returns Format Tag Link of the specified streaming interface
    setting. TODO Add more description. 
    
   Precondition:
    Audio v1.0 device should have been attached. 

   Parameters:
    audioObj - Audio Device Object

    streamingInterfaceObj - Audio Streaming Interface Object

    interfaceSettingObj - Audio Streaming Interface Setting Object 

   Returns:
     Format Tag of the Audio Streaming Interface setting 
     
   Example:
    <code> 
    </code>

   Remarks:

*/
USB_AUDIO_V1_FORMAT_TAG USB_HOST_AUDIO_V1_StreamingInterfaceFormatTagGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    
    
    audioInstance = (uint8_t)interfaceSettingObj; 
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8); 
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    if (audioInstanceInfo == NULL)
    {
        return USB_AUDIO_FORMAT_TYPE_I_UNDEFINED; 
    }
    
    return (USB_AUDIO_V1_FORMAT_TAG)audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].wFormatTag; 
}


// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceChannelNumbersGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    This function returns number of channels of the specified streaming interface
    setting. 
    
   Description:
    This function returns Number of channels of the specified streaming interface
    setting. TODO Add more description. 
    
   Precondition:
    Audio v1.0 device should have been attached. 

   Parameters:
    audioObj - Audio Device Object

    streamingInterfaceObj - Audio Streaming Interface Object

    interfaceSettingObj - Audio Streaming Interface Setting Object 

   Returns:
     Number of Channels present in the Audio Streaming Interface setting 
     
   Example:
    <code> 
    </code>

   Remarks:

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceChannelNumbersGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    uint8_t nChannels =  0; 
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    
    
    audioInstance = (uint8_t)interfaceSettingObj; 
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8); 
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    if (audioInstanceInfo != NULL)
    {
    
        nChannels =  audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].bNrChannels;
    }
    return nChannels; 
}

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceSubFrameSizeGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    This function returns Subframe size of the specified streaming interface
    setting. 
    
   Description:
    This function returns Subframe Size of the specified streaming interface
    setting. TODO Add more description. 
    
   Precondition:
    Audio v1.0 device should have been attached. 

   Parameters:
    audioObj - Audio Device Object

    streamingInterfaceObj - Audio Streaming Interface Object

    interfaceSettingObj - Audio Streaming Interface Setting Object 

   Returns:
     Subframe Size of the Audio Streaming Interface setting 
     
   Example:
    <code> 
    </code>

   Remarks:

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceSubFrameSizeGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    
    
    audioInstance = (uint8_t)interfaceSettingObj; 
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8); 
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    if (audioInstanceInfo == NULL)
    {
        return 0; 
    }
    
    return audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].bSubframeSize; 
}


// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceBitResolutionGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    This function returns Bit resolution of the specified streaming interface
    setting. 
    
   Description:
    This function returns Bit Resolution size of the specified streaming interface
    setting. TODO Add more description. 
    
   Precondition:
    Audio v1.0 device should have been attached. 

   Parameters:
    audioObj - Audio Device Object

    streamingInterfaceObj - Audio Streaming Interface Object

    interfaceSettingObj - Audio Streaming Interface Setting Object 

   Returns:
     Bit Resolution size of the Audio Streaming Interface setting 
     
   Example:
    <code> 
    </code>

   Remarks:

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceBitResolutionGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    uint8_t bitResolution = 0; 
      
    audioInstance = (uint8_t)interfaceSettingObj; 
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8); 
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    if (audioInstanceInfo != NULL)
    {
        bitResolution = audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].bBitResolution;
    }
    
    return  bitResolution; 
}

// *****************************************************************************
/* Function:
    uint8_t* USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequencyGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns Sampling Frequencies supported by the specified streaming interface
    setting. 
    
   Description:
    This function returns Sampling Frequencies supported by the specified streaming 
    interface setting. TODO Add more description. 
    
   Precondition:
    Audio v1.0 device should have been attached. 

   Parameters:
    audioObj - Audio Device Object

    streamingInterfaceObj - Audio Streaming Interface Object

    interfaceSettingObj - Audio Streaming Interface Setting Object 

   Returns:
     Pointer to Sampling Frequencies supported by the Audio Streaming Interface setting 
     
   Example:
    <code> 
    </code>

   Remarks:

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequencyTypeGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    
    
    audioInstance = (uint8_t)interfaceSettingObj; 
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8); 
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    if (audioInstanceInfo == NULL)
    {
        return 0; 
    }
    
    return audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].bSamFreqType; 
}

// *****************************************************************************
/* Function:
    uint8_t* USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequenciesGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns Sampling Frequencies supported by the specified streaming interface
    setting. 
    
   Description:
    This function returns Sampling Frequencies supported by the specified streaming 
    interface setting. TODO Add more description. 
    
   Precondition:
    Audio v1.0 device should have been attached. 

   Parameters:
    audioObj - Audio Device Object

    streamingInterfaceObj - Audio Streaming Interface Object

    interfaceSettingObj - Audio Streaming Interface Setting Object 

   Returns:
     Pointer to Sampling Frequencies supported by the Audio Streaming Interface setting 
     
   Example:
    <code> 
    </code>

   Remarks:

*/
uint8_t* USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequenciesGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    
    
    audioInstance = (uint8_t)interfaceSettingObj; 
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8); 
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    if (audioInstanceInfo == NULL)
    {
        return 0; 
    }
    
    return audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].tSamFreq; 
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_STREAM_DIRECTION USB_HOST_AUDIO_V1_StreamInterfaceDirectionGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj, 
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns Direction of the specified streaming interface
    setting. 
    
   Description:
    This function returns Direction of the specified streaming 
    interface setting. TODO Add more description. 
    
   Precondition:
    Audio v1.0 device should have been attached. 

   Parameters:
    audioObj - Audio Device Object

    streamingInterfaceObj - Audio Streaming Interface Object

    interfaceSettingObj - Audio Streaming Interface Setting Object 

   Returns:
     USB_HOST_AUDIO_V1_DIRECTION_OUT - Host to Device
     USB_HOST_AUDIO_V1_DIRECTION_IN - Device to Host
     
   Example:
    <code> 
    </code>

   Remarks:

*/
USB_HOST_AUDIO_V1_STREAM_DIRECTION USB_HOST_AUDIO_V1_StreamingInterfaceDirectionGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
)
{
    USB_ENDPOINT_DESCRIPTOR *dataEndpointDescriptor; 
    uint8_t audioInstance; 
    uint8_t aSIntrfcIdx; 
    uint8_t asIntrfcSettingIdx; 
    USB_HOST_AUDIO_V1_STREAM_DIRECTION direction = USB_HOST_AUDIO_V1_DIRECTION_OUT; 
    USB_HOST_AUDIO_V1_INSTANCE* audioInstanceInfo; 
    
    
    audioInstance = (uint8_t)interfaceSettingObj; 
    aSIntrfcIdx = (uint8_t) (interfaceSettingObj >> 8); 
    asIntrfcSettingIdx = (uint8_t)(interfaceSettingObj >> 16);  
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstance];
    
    if (audioInstanceInfo == NULL)
    {
        return direction; 
    }
    
    dataEndpointDescriptor = &audioInstanceInfo->streamInf[aSIntrfcIdx].audioStreamSetting[asIntrfcSettingIdx].isoDataEndpointDesc; 
    
    direction = dataEndpointDescriptor->dirn; 
       
    return direction; 
}

// *****************************************************************************
// Section: Depreciated APIs - Not recommended for new applications 
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_0_NumberOfStreamGroupsGet
    (
        USB_HOST_AUDIO_V1_0_OBJ audioObj
    );

  Summary:
    This function will Get number of stream groups present in the attached 
    Audio v1.0 Device. 

  Description:
    This function will Get number of stream groups present in the attached 
    Audio v1.0 Device. Audio stream with in a audio streams cannot be enabled
    at the same time. 
    
  Precondition:
    Audio v1.0 device should have been attached. 

  Parameters:
    audioObj - Audio v1.0 client driver object 

  Returns:
    Returned uint8_t indicates number of  audio stream groups present in the 
    attached Audio v1.0 device. 
    
  Example:
    <code> 
    </code>

  Remarks:

 */
uint8_t USB_HOST_AUDIO_V1_0_NumberOfStreamGroupsGet
(
    USB_HOST_AUDIO_V1_0_OBJ audioObj
)
{
    USB_HOST_AUDIO_V1_0_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_0_INSTANCE*)audioObj; 
    
    return audioInstanceInfo->nASInterfaces; 

}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamEnable
    (
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle, 
        USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * requestHandle
    ); 

  Summary:
    Schedules an  Audio stream enable request for the specified audio stream. 

  Description:
    This function schedules an Audio stream enable request for the specified 
    audio stream. An audio stream must be enable before scheduling any data 
    transfer with the stream. An event 
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE is generated when  this 
    request is completed. USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE_DATA 
    returns the status and request handle of the request. 
        
       
  Precondition:
    Audio stream should have been opened. Only one audio stream from an audio
    stream group can be enabled at a time. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 Stream.
    requestHandle - Handle to the Stream Enable request. 

  Returns:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamEnable
(
    
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle, 
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * requestHandle
)
{
    USB_HOST_AUDIO_V1_0_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    uint8_t alternateSetting; 
    USB_HOST_RESULT hostResult; 
    
    if (streamHandle == USB_HOST_AUDIO_V1_0_STREAM_HANDLE_INVALID)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_RESULT_PARAMETER_INVALID; 
    }
    
    /* Find Audio Stream from audioStreamObj */
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
    alternateSetting = (uint8_t)(streamHandle>>16);
       
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex];
    
    hostResult = USB_HOST_DeviceInterfaceSet 
                 (
                     asInterface->asInterfaceHandle, 
                     requestHandle, 
                     alternateSetting,
                     (uintptr_t)alternateSetting | (uintptr_t)USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_ENABLE << 8
                 ); 
    return hostResult; 
}
// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamDisable
    (
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle, 
        USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * requestHandle
    ); 

  Summary:
    Schedules an  Audio stream disable request for the specified audio stream. 

  Description:
    This function schedules an Audio stream disable request for the specified 
    audio stream. An event 
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE is generated when this
    request is completed. USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE_DATA 
    returns the status and request handle of the request. 
        
       
  Precondition:
    Audio stream should have been opened. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 Stream.
    requestHandle - Handle to the Stream Disable request. 

  Returns:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamDisable
(
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE *requestHandle
)
{
    USB_HOST_AUDIO_V1_0_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    uint8_t alternateSetting; 
    USB_HOST_RESULT hostResult; 
    
    /* Check if the stream handle is invalid */
    if (streamHandle == USB_HOST_AUDIO_V1_0_STREAM_HANDLE_INVALID)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_RESULT_PARAMETER_INVALID; 
    }
    
    /* Get Audio Instance Index, It is always the LSB */
    audioInstanceIndex = (uint8_t)streamHandle;
    
    /* Get the Streaming Interface Index. It always 2nd LSB */
    asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
    
    /* Get the Alternate Setting Number. It always 3rd LSB */
    alternateSetting = (uint8_t)(streamHandle>>16); 
        
    /* Get pointer to Audio Instance */
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    
    /* NULL check */
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_RESULT_PARAMETER_INVALID; 
    }
    
    /* Get pointer to Audio Streaming Interface */
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex]; 
    
    /* NULL check */
    if (asInterface == NULL)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_RESULT_PARAMETER_INVALID; 
    }
    
    /* Submit Set Interface request to Host Layer */
    hostResult = USB_HOST_DeviceInterfaceSet 
                 (
                      asInterface->asInterfaceHandle, 
                      requestHandle, 
                      0,
                      (uintptr_t)alternateSetting | (uintptr_t)USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_DISABLE << 8  
                  ); 
    return hostResult; 
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_ControlRequest
    (
        USB_HOST_AUDIO_V1_0_OBJ audioObj,
        USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * requestHandle,
        USB_SETUP_PACKET *setupPacket,
        void * data,
        USB_HOST_AUDIO_V1_0_CONTROL_CALLBACK callback, 
        uintptr_t context 
    );

  Summary:
    Schedules an Audio v1.0 control transfer.

  Description:
    This function schedules an Audio v1.0 control transfer. audioObj is an 
    Object of Audio v1.0 class driver to which the audio control transfer is to
    be scheduled. setupPacket points to the setup command to be sent in the 
    Setup Stage of the control transfer. The size and the direction of the data
    stage is indicated by the setup packet. In case of control transfers where
    there is no data stage, data is ignored and can be NULL. In all other cases,
    data should point to the data to data be transferred in the data stage of 
    the control transfer. 
    
    If the transfer was scheduled successfully, requestHandle will contain a
    transfer handle that uniquely identifies this transfer. If the transfer
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.

    When the control transfer completes, the Audio v1.0 client driver will call
    the specified callback function. The context parameter specified here will 
    be returned in the callback.

  Precondition:
    Audio v1.0 Device should have attached. 

  Parameters:
    audioObj - Audio v1.0 client driver object 

    requestHandle - output parameter that will contain the handle to this
    transfer.

    setupPacket - Pointer to the setup packet to sent to the device in the setup
    stage of the control transfer.

    data -  For control transfer with a data stage, this should point to data to
    be sent to the device (for a control write transfer) or point to the buffer
    that will receive data from the device (for a control read transfer). For
    control transfers that do not require a data stage, this parameter is
    ignored and can be NULL.

    callback - pointer to the callback function that will be called when the
    control transfer completes. If the callback function is NULL, there will be
    no notification of when the control transfer will complete.

    context - user defined context that is returned with the callback function.

  Returns:
    USB_HOST_AUDIO_V1_0_RESULT_SUCCESS - the transfer was scheduled successfully.
    requestHandle will contain a valid transfer handle.
    USB_HOST_AUDIO_V1_0_RESULT_FAILURE - an unknown failure occurred. requestHandle will
    contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
    USB_HOST_AUDIO_V1_0_RESULT_PARAMETER_INVALID - The data pointer or requestHandle pointer
    is NULL.

  Example:
    <code>
    </code>

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_ControlRequest
(
    USB_HOST_AUDIO_V1_0_OBJ audioObj,
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * transferHandle,
    USB_SETUP_PACKET *setupPacket,
    void * data,
USB_HOST_AUDIO_V1_0_CONTROL_CALLBACK callback, 
    uintptr_t context 
)
{
    USB_HOST_AUDIO_V1_0_RESULT result = USB_HOST_AUDIO_V1_0_RESULT_FAILURE; 
    USB_HOST_AUDIO_V1_0_INSTANCE *audioInstanceInfo = (USB_HOST_AUDIO_V1_0_INSTANCE *)audioObj ;
    OSAL_CRITSECT_DATA_TYPE IntState;
    

    /* Check if the handle to the audio Instance in not NULL */
    if( audioInstanceInfo == NULL )
    {
        SYS_DEBUG_MESSAGE ( SYS_DEBUG_INFO, "USB_HOST_AUDIO_ControlSend: Not a valid instance " );
        return USB_HOST_AUDIO_V1_0_RESULT_OBJ_INVALID;
    }

    /* Check if this Audio Instance is initialized */
    if(audioInstanceInfo->assigned == false)
    {
        SYS_DEBUG_MESSAGE ( SYS_DEBUG_INFO , "USB_HOST_AUDIO_ControlSend: Invalid parameters " );
        return USB_HOST_AUDIO_V1_0_RESULT_FAILURE;
    }

    if (audioInstanceInfo->audioControlObj.inUse == true)
    {
        return USB_HOST_AUDIO_V1_0_RESULT_BUSY; 
    }
    
    /* Prevent other tasks pre-empting this sequence of code */ 
    IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
    
    memcpy (&audioInstanceInfo->setupPacket, setupPacket, sizeof (USB_SETUP_PACKET)); 
   
    audioInstanceInfo->audioControlObj.inUse = true; 
    audioInstanceInfo->audioControlObj.context = context; 
    audioInstanceInfo->audioControlObj.callback = (USB_HOST_AUDIO_V1_ENTITY_REQUEST_CALLBACK)callback; 
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
    result = USB_HOST_DeviceControlTransfer
             (
                 audioInstanceInfo->controlPipeHandle,
                 transferHandle,
                 &audioInstanceInfo->setupPacket,
                 (void *)data,
                 _USB_HOST_AUDIO_V1_0_ControlRequestCallback,
                 (uintptr_t)audioInstanceInfo
             ); 
    
    return result;
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_StreamGetFirst 
    (
        USB_HOST_AUDIO_V1_0_OBJ audioDeviceObj, 
        uint8_t streamGroupIndex, 
        USB_HOST_AUDIO_V1_0_STREAM_INFO * streamInfo
    );

  Summary:
    Returns information about first audio stream in the specified Audio stream 
    group. 

  Description:
    This function returns information about first audio stream in the specified
    Audio stream group. The stream group index is parameter to this function 
    and it can be any value starting from Zero to number of stream groups minus
    one. Number of stream groups can be obtained by using 
    USB_HOST_AUDIO_V1_0_NumberOfStreamGroupsGet() function. 
    
    streamInfo object is an out parameter to this function. 
    
  Precondition:
    Audio v1.0 device should have been attached to the Host. 

  Parameters:
    audioDeviceObj   - Audio v1.0 client driver object

    streamGroupIndex - Stream Group Index.  
    
    streamInfo       -  Pointer to streamInfo object 

  Returns:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_0_RESULT_OBJ_INVALID - The specified Audio v1.0 client
    driver object does not exist.
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_StreamGetFirst 
(
    USB_HOST_AUDIO_V1_0_OBJ audioDeviceObj, 
    uint8_t streamGroupIndex, 
    USB_HOST_AUDIO_V1_0_STREAM_INFO * streamInfo
)
{
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface; 
    USB_HOST_AUDIO_STREAM_SETTING* audioStream; 
    USB_HOST_AUDIO_V1_0_INSTANCE* audioInstanceInfo = (USB_HOST_AUDIO_V1_0_INSTANCE*)audioDeviceObj;
    uint8_t alternateSetting; 
    int count; 
    
    if (audioInstanceInfo == NULL)
    {
        return USB_HOST_AUDIO_V1_0_RESULT_OBJ_INVALID; 
    }
    
    /* Get pointer to First Stream from the Stream Group */
    asInterface = &audioInstanceInfo->streamInf[streamGroupIndex]; 
    
    if (asInterface->nInterfaceSetting == 0)
    {
        return USB_HOST_AUDIO_V1_0_RESULT_FAILURE; 
    }
    /* First Stream is always the alternate setting 1 */
    alternateSetting = 1;  
    audioStream = &asInterface->audioStreamSetting[alternateSetting]; 
    /* Stream Object is made from 
       LSB = audio instance index 
       Second byte  = Stream Interface Index
       Third byte = Alternate Setting
       MSB = 0x01   */
    streamInfo->streamObj = (USB_HOST_AUDIO_V1_0_STREAM_OBJ ) (uint32_t)audioInstanceInfo->index|
                                                              ((uint32_t)streamGroupIndex)<<8|
                                                              ((uint32_t)alternateSetting<<16); 
    streamInfo->format = audioStream->wFormatTag; 
    streamInfo->nChannels = audioStream->bNrChannels; 
    streamInfo->streamDirection = audioStream->direction; 
    streamInfo->bitResolution = audioStream->bBitResolution; 
    streamInfo->subFrameSize = audioStream->bSubframeSize; 
   
    streamInfo->nSamplingRates = audioStream->bSamFreqType; 
    
    if (streamInfo->nSamplingRates > USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER )
    {
        streamInfo->nSamplingRates = USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER; 
    }
    
    /* Check if the Audio Stream supports discrete sampling frequency */
    if (streamInfo->nSamplingRates != 0)
    {
        /* Save Sampling frequencies */
        for (count = 0; count < audioStream->bSamFreqType; count++)
        {
            /* In the USB descriptors sampling frequency is 3 Bytes long. 
               We are copying it into a uint32_t */
            streamInfo->tSamFreq[count] =  (uint32_t)audioStream->tSamFreq[count*3]  |
                                            (uint32_t)audioStream->tSamFreq[count*3 + 1] << 8 |
                                            (uint32_t)audioStream->tSamFreq[count*3 + 2] << 16;                                
        }                                     
    }
    return USB_HOST_AUDIO_V1_0_RESULT_SUCCESS;     
}
// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_StreamGetNext 
    (
        USB_HOST_AUDIO_V1_0_OBJ audioDeviceObj, 
        uint8_t streamGroupIndex,
        USB_HOST_AUDIO_V1_0_STREAM_INFO * streamInfo
    );

  Summary:
    Returns information about next audio stream in the specified Audio stream 
    group.

  Description:
    This function returns information about next audio stream in the specified
    Audio stream group. USB_HOST_AUDIO_V1_0_StreamGetFirst() function should 
    have been called at least once on the same Audio stream group before calling 
    this function. Then calling this function repeatedly on the stream group 
    will return information about the next audio stream in the stream group. 
    When there are no more audio streams to report, the function returns 
    USB_HOST_AUDIO_V1_0_RESULT_END_OF_STREAM_LIST. 
    
    Calling the USB_HOST_AUDIO_V1_0_StreamGetFirst() function on the stream group
    index after the USB_HOST_AUDIO_V1_0_StreamGetNext() function has been called 
    will cause Audio v1.0 client driver to reset the audio stream group to point
    to the first stream in the stream group.
    
  Precondition:
    The USB_HOST_AUDIO_V1_0_StreamGetFirst() function must have been called
    before calling this function.

  Parameters:
    audioDeviceObj   - Audio v1.0 client driver object

    streamGroupIndex - Stream Group Index.  
    
    streamInfo       -  Pointer to streamInfo object 

  Returns:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_0_RESULT_OBJ_INVALID - The specified Audio v1.0 client
    driver object does not exist.
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred.
    USB_HOST_AUDIO_V1_0_RESULT_END_OF_STREAM_LIST - There are no more audio 
    streams in the stream group. 
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/ 
USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_StreamGetNext 
(
    USB_HOST_AUDIO_V1_0_STREAM_OBJ audioStreamObj, 
    USB_HOST_AUDIO_V1_0_STREAM_INFO * streamInfo
)
{
    USB_HOST_AUDIO_V1_0_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAM_SETTING* audioStream; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface; 
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    uint8_t alternateSetting; 
    // TODO uint8_t alternateSettingIndex; 
    uint8_t nextAlternateSetting; 
    uint8_t nextAlternateSettingIndex; 
    int count; 
    
    if (audioStreamObj == (USB_HOST_AUDIO_V1_0_STREAM_OBJ)NULL)
    {
        return USB_HOST_AUDIO_V1_0_RESULT_OBJ_INVALID; 
    }
    
    audioInstanceIndex = (uint8_t)audioStreamObj;
    asIntrefaceIndex = (uint8_t)(audioStreamObj>>8); 
    alternateSetting = (uint8_t)(audioStreamObj>>16);
    
    if (alternateSetting == 0)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_HANDLE_INVALID; 
    }
    // TODO alternateSettingIndex = alternateSetting;     
    
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex]; 
    
    if (asInterface->nInterfaceSetting == 0)
    {
        return USB_HOST_AUDIO_V1_0_RESULT_FAILURE; 
    }
    
    /* First Stream is always the alternate setting 1 */
    nextAlternateSetting = alternateSetting + 1;
    
    
    if (nextAlternateSetting >= asInterface->nInterfaceSetting)
    {
        return USB_HOST_AUDIO_V1_0_RESULT_END_OF_STREAM_LIST; 
    }
    
    nextAlternateSettingIndex = nextAlternateSetting-1; 
    audioStream =  &asInterface->audioStreamSetting[nextAlternateSettingIndex]; 
    /* Stream Object is made from 
       LSB = audio instance index 
       Second byte  = Stream Interface Index
       Third byte = Alternate Setting (not index)
       MSB = 0x01   */
    streamInfo->streamObj = (USB_HOST_AUDIO_V1_0_STREAM_OBJ ) (uint32_t)audioInstanceInfo->index|
                                                              ((uint32_t)asIntrefaceIndex)<<8|
                                                              ((uint32_t)nextAlternateSetting<<16); 
    streamInfo->format = audioStream->wFormatTag; 
    streamInfo->nChannels = audioStream->bNrChannels; 
    streamInfo->streamDirection = audioStream->direction; 
    streamInfo->bitResolution = audioStream->bBitResolution; 
    streamInfo->subFrameSize = audioStream->bSubframeSize; 
   
    streamInfo->nSamplingRates = audioStream->bSamFreqType; 
    
    if (streamInfo->nSamplingRates > USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER )
    {
        streamInfo->nSamplingRates = USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER; 
    }
    
    for (count = 0; count <= streamInfo->nSamplingRates; count++ )
    {
        streamInfo->tSamFreq[count] = audioStream->tSamFreq[count]; 
    }
    
    return USB_HOST_AUDIO_V1_0_RESULT_SUCCESS; 
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamEventHandlerSet
    (
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE handle,
        USB_HOST_AUDIO_V1_0_STREAM_EVENT_HANDLER appAudioHandler,
        uintptr_t context
    ); 

  Summary:
    Registers an event handler with the Audio v1.0 Client Driver Stream.

  Description:
    This function registers a client specific Audio v1.0 stream event handler.
    The Audio v1.0 Host Client Driver will call appAudioHandler function 
    specified as 2nd argument with relevant event and associate event data, in
    response to audio stream data transfers that have been scheduled by the 
    client.
    
  Precondition:
    None.

  Parameters:
    handle  - handle to the Audio v1.0 Stream.

    eventHandler - A pointer to event handler function. If NULL, then events
                   will not be generated.
    
    context - Application specific context that is returned in the event handler.

  Returns:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamEventHandlerSet
(
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_HANDLER streamEventHandler,
    uintptr_t context
)
{ 
    USB_HOST_AUDIO_V1_0_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    USB_HOST_AUDIO_STREAM_SETTING* audioStream; 
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    uint8_t alternateSetting; 
    uint8_t alternateSettingIndex; 
    
    
    /* Find Audio Stream from audioStreamObj */
    if (streamHandle != USB_HOST_AUDIO_V1_0_STREAM_HANDLE_INVALID)
    {
        audioInstanceIndex = (uint8_t)streamHandle;
        asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
        alternateSetting = (uint8_t)(streamHandle>>16); 
        
        alternateSettingIndex = alternateSetting; 
        
        audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex];
        asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex]; 
        audioStream =  &(asInterface->audioStreamSetting[alternateSettingIndex]);
        
        if (streamEventHandler != (USB_HOST_AUDIO_V1_0_STREAM_EVENT_HANDLER)NULL)
        {
            audioStream->streamEventHandler = streamEventHandler;
            audioStream->context = context; 
            return USB_HOST_AUDIO_V1_0_STREAM_SUCCESS; 
        }
    }
    return USB_HOST_AUDIO_V1_0_RESULT_PARAMETER_INVALID; 
}

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamSamplingRateSet
    (
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
        USB_HOST_AUDIO_V1_0_REQUEST_HANDLE requestHandle,
        uint32_t samplingRate
    ); 

  Summary:
    Schedules an  Audio stream Set Sampling rate request for the specified 
    audio stream. 

  Description:
    This function schedules an  Audio stream Set Sampling rate request for the
    specified audio stream. An event 
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE is generated 
    when this request is completed. 
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA returns 
    the status and request handle of the request. 
        
       
  Precondition:
    Audio stream should have been opened. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 Stream.
    requestHandle - Handle to the Stream Set Sampling rate request 

  Returns:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist.
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamSamplingRateSet
(  
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE *requestHandle,
    uint32_t *samplingRate
)
{
    USB_HOST_AUDIO_V1_0_INSTANCE * audioInstanceInfo; 
    USB_HOST_AUDIO_STREAM_SETTING* audioStream; 
    USB_HOST_AUDIO_STREAMING_INTERFACE* asInterface;
    uint8_t audioInstanceIndex;
    uint8_t asIntrefaceIndex; 
    uint8_t alternateSettingIndex; 
    uint8_t alternateSetting; 
    USB_HOST_RESULT hostResult; 
    USB_HOST_AUDIO_V1_0_CONTROL_TRANSFER_OBJ* audioControlObj; 
    USB_AUDIO_ENDPOINT_CONTROL_REQUEST* setupPacket; 
    OSAL_CRITSECT_DATA_TYPE IntState;
    
    if (streamHandle == USB_HOST_AUDIO_V1_0_STREAM_HANDLE_INVALID)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_RESULT_PARAMETER_INVALID; 
    }
    
    /* Find Audio Stream from audioStreamObj */
    audioInstanceIndex = (uint8_t)streamHandle;
    asIntrefaceIndex = (uint8_t)(streamHandle>>8); 
    alternateSetting = (uint8_t)(streamHandle>>16); 
    
    if (alternateSetting == 0)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_RESULT_PARAMETER_INVALID;
    }
    alternateSettingIndex = alternateSetting;     
    audioInstanceInfo = &gUSBHostAudioInstance[audioInstanceIndex]; 
    asInterface = &audioInstanceInfo->streamInf[asIntrefaceIndex];
    audioStream = &asInterface->audioStreamSetting[alternateSettingIndex];
    
    audioControlObj = &asInterface->audioControlObj; 
    setupPacket = (USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)&asInterface->setupPacket; 
    
    if (audioControlObj->inUse == true)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_RESULT_REQUEST_BUSY; 
    }
    
    if (audioStream->nEndpoints == 0)
    {
        return USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE; 
    }
    /* Prevent other tasks pre-empting this sequence of code */ 
    IntState = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
   
    audioControlObj->inUse = true; 
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, IntState);
    
    //Class Specific Endpoint Request, Direction Host to Device
    setupPacket->bmRequestType =   USB_SETUP_DIRN_HOST_TO_DEVICE
                                | USB_SETUP_TYPE_CLASS
                                | USB_SETUP_RECIPIENT_ENDPOINT ;

    setupPacket->bRequest = USB_AUDIO_CS_SET_CUR;
    setupPacket->controlSelector = USB_AUDIO_SAMPLING_FREQ_CONTROL;
    
    setupPacket->endpointNumber = audioStream->isoDataEndpointDesc.bEndpointAddress;
    
    setupPacket->wLength = 3;
    
    hostResult = USB_HOST_DeviceControlTransfer
             (
                 audioInstanceInfo->controlPipeHandle,
                 requestHandle,
                 (USB_SETUP_PACKET *)setupPacket,
                 (void *)samplingRate,
                 _USB_HOST_AUDIO_V1_SetSampleRateCallback,
                 (uintptr_t)streamHandle
             ); 
    
    return hostResult;
    
}
/***************  End of  File ************************************/





