 /*******************************************************************************
  USB Audio class definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_audio_local.h

  Summary:
    USB Audio class definitions

  Description:
    This file describes the Audio class specific definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _USB_HOST_AUDIO_LOCAL_H
#define _USB_HOST_AUDIO_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host.h"
#include "usb/src/usb_host_local.h"
#include "usb/usb_host_audio_v1_0.h"
#include "usb/usb_host_client_driver.h"

// *****************************************************************************
/* USB Host Audio v1.0 Client Driver Stream States

  Summary:
    USB Host Audio v1.0 Client Driver Stream States

  Description:
    This enumeration lists all possible states USB HOST Audio 1.0 client driver
    states.  

  Remarks:
    None.
*/
typedef enum
{
    /* Error state */
    USB_HOST_AUDIO_V1_STREAM_STATE_ERROR = -1,

    /* The instance is not ready */
    USB_HOST_AUDIO_V1_STREAM_STATE_NOT_READY = 0,

    /* The instance should set the configuration */
    USB_HOST_AUDIO_V1_STREAM_STATE_SET_CONFIGURATION,

    /* Wait for configuration set */
    USB_HOST_AUDIO_V1_STREAM_STATE_WAIT_FOR_CONFIGURATION_SET,

    /* Wait for interfaces to get ready */
    USB_HOST_AUDIO_V1_STREAM_STATE_WAIT_FOR_INTERFACES,

    /* The instance is ready */
    USB_HOST_AUDIO_V1_STREAM_STATE_READY,
            
    /* Pipe Open or Close is pending on this the stream */ 
    USB_HOST_AUDIO_V1_STREAM_STATE_PIPE_ACTION_PENDING, 
    
    /* Pipe is opened on the stream. Data can be transferred on this stream now */ 
    USB_HOST_AUDIO_V1_STREAM_STATE_PIPE_OPEN_SUCCESS, 
            
    /* Pipe Open failed */ 
    USB_HOST_AUDIO_V1_STREAM_PIPE_OPEN_FAILED,         
            
    /*  Pipe is closed on the stream */        
    USB_HOST_AUDIO_V1_STREAM_STATE_PIPE_CLOSE_SUCCESS,         
    
    /* Pipe Close failed */         
    USB_HOST_AUDIO_V1_STREAM_PIPE_CLOSE_FAILED     
            
} USB_HOST_AUDIO_V1_STREAM_STATE;

// *****************************************************************************
/* USB Host Audio v1.0 Client Driver API Version Flags

  Summary:
    USB Host Audio v1.0 Client Driver API Version Flags

  Description:
    This enumeration lists all possible values of USB Host Audio v1.0 Client 
    Driver API Version Flags. Audio Client driver supports multiple versions of 
    API. These flags are used to identify the API Version during a callback.  

  Remarks:
    None.
*/
typedef enum
{
    USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_INTERFACE_SET = 0, 
    USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_DISABLE = 1, 
    USB_HOST_AUDIO_V1_API_VERSION_FLAG_STREAM_ENABLE = 2,
    USB_HOST_AUDIO_V1_API_VERSION_FLAG_V1 = 3, 
    USB_HOST_AUDIO_V1_API_VERSION_FLAG_V1_0_DEPRECIATED = 4
} USB_HOST_AUDIO_V1_API_VERSION_FLAGS; 

#if defined (USB_HOST_AUDIO_V1_0_INSTANCES_NUMBER) && !defined (USB_HOST_AUDIO_V1_INSTANCES_NUMBER)
 #define USB_HOST_AUDIO_V1_INSTANCES_NUMBER USB_HOST_AUDIO_V1_0_INSTANCES_NUMBER
 #endif 

#if defined (USB_HOST_AUDIO_V1_0_STREAMING_INTERFACES_NUMBER) && !defined (USB_HOST_AUDIO_V1_STREAMING_INTERFACES_NUMBER)
 #define USB_HOST_AUDIO_V1_STREAMING_INTERFACES_NUMBER USB_HOST_AUDIO_V1_0_STREAMING_INTERFACES_NUMBER
 #endif 

#if defined (USB_HOST_AUDIO_V1_0_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER) && !defined (USB_HOST_AUDIO_V1_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER)
 #define USB_HOST_AUDIO_V1_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER USB_HOST_AUDIO_V1_0_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER + 1
 #endif 

#if defined (USB_HOST_AUDIO_V1_0_SAMPLING_FREQUENCIES_NUMBER) && !defined (USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER)
 #define USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER USB_HOST_AUDIO_V1_0_SAMPLING_FREQUENCIES_NUMBER
 #endif

#define USB_HOST_AUDIO_V1_0_INSTANCE  USB_HOST_AUDIO_V1_INSTANCE
#define _USB_HOST_AUDIO_V1_0_ControlRequestCallback _USB_HOST_AUDIO_V1_ControlRequestCallback
// *****************************************************************************
/* USB Host Audio v1.0 Control Transfer Object

  Summary:
    USB Host Audio v1.0Control Transfer Object

  Description:
    This data structure defines a Host Audio v1.0 Control Transfer Object. 
    A Control transfer object tracks a client control transfer request. 

  Remarks:
    None.
*/

typedef struct 
{
    /* True if this object is allocated */
    bool inUse;
    
    /* Transfer context */
    uintptr_t context;

    /* Type of control request */
    USB_HOST_CONTROL_REQUEST_TYPE requestType;

    /* Callback to invoke when control transfer is complete */
    USB_HOST_AUDIO_V1_ENTITY_REQUEST_CALLBACK callback;

} USB_HOST_AUDIO_V1_CONTROL_TRANSFER_OBJ;

typedef struct
{
    uint32_t event;
    
    USB_HOST_DEVICE_INTERFACE_EVENT_SET_INTERFACE_COMPLETE_DATA eventData; 

    uintptr_t context;
} USB_HOST_AUDIO_V1_STREAM_STATE_DATA; 

#define USB_HOST_AUDIO_V1_0_CONTROL_TRANSFER_OBJ USB_HOST_AUDIO_V1_CONTROL_TRANSFER_OBJ
typedef struct
{         
    /* Streaming Interface Number */
    uint8_t asInterfaceId; 

    /* Interface Alternate Setting Number */
    uint8_t interfaceAlternateSetting;

    /* Number of Endpoints */
    uint8_t nEndpoints;

    /* Audio Streaming Interface Class Specific Descriptor */
    uint16_t wFormatTag;

    /* Number of Physical Channels*/
    uint8_t bNrChannels;

    /* The number of bytes occupied by one audio subframe.*/
    uint8_t bSubframeSize;

    /* The number of effectively used bits from the available bits in an audio
     * subframe*/
    uint8_t bBitResolution;

    /* The number of discrete sampling frequencies supported*/
    uint8_t bSamFreqType;
    
    uint8_t bTerminalLink; 
    
    /* Flag indicates if Feedback supported */
    bool isFeebackSupported;
    
     /* Stream Direction */
    USB_HOST_AUDIO_V1_STREAM_DIRECTION direction;
   
    /* Pointer to Sampling frequency */
    uint8_t * tSamFreq; 
    
    USB_INTERFACE_DESCRIPTOR descASInterface; 
    
    USB_AUDIO_CS_AS_INTERFACE_DESCRIPTOR descASClassSepcific;
    
    USB_ENDPOINT_DESCRIPTOR isoDataEndpointDesc; 
    
    USB_ENDPOINT_DESCRIPTOR isoFeedbackaEndpointDesc;
    
     USB_HOST_AUDIO_V1_0_STREAM_EVENT_HANDLER streamEventHandler; 
    
    /* Application defined context */
    uintptr_t context;
    
} USB_HOST_AUDIO_STREAM_SETTING;



// *****************************************************************************

/* USB Host Audio Streaming Interface data Structure

  Summary:
    USB Host Audio Streaming Interface data Structure

  Description:
    This structure has the details about USB Host Audio Streaming interface
    like Interface Number, Interface Type and Alternate setting.

  Remarks:
    None.
 */
typedef struct
{
    
     /* Stream state */
    USB_HOST_AUDIO_V1_STREAM_STATE state; 
    
    USB_HOST_AUDIO_V1_STREAM_STATE_DATA stateData; 
    
    /* Audio Streaming Interface Handle */ 
    USB_HOST_DEVICE_INTERFACE_HANDLE asInterfaceHandle;

    /* Interface Type */
    USB_AUDIO_SUBCLASS_CODE infType;
    
    /* Interface number */
    uint8_t interfaceId;

    /* Number of Interface Alternate Settings */
    uint8_t nInterfaceSetting; 

    /* Interface alternate setting that is active currently */
    uint8_t activeInterfaceSetting;

    /* Temporary storage of requested Interface Alternate Setting */
    uint8_t requestedInterfaceSetting;
    
    /* Audio Stream Info */ 
    USB_HOST_AUDIO_STREAM_SETTING audioStreamSetting[USB_HOST_AUDIO_V1_STREAMING_INTERFACE_ALTERNATE_SETTINGS_NUMBER]; 
    
     /* Pipe status */
    bool isIsoDataPipeSet;
    
    /* Pipe status */
    bool isIsoFeedbackPipeSet;

    /* Pipe Handle for Isochronous Data  Endpoint */
    USB_HOST_PIPE_HANDLE isoDataPipeHandle;

    /* Pipe Handle for Isochronous Feedback Endpoint */
    USB_HOST_PIPE_HANDLE isochronousFeedbackPipeHandle;
    
    USB_SETUP_PACKET setupPacket; 
    
    USB_HOST_AUDIO_V1_CONTROL_TRANSFER_OBJ audioControlObj;
    
    USB_HOST_AUDIO_V1_STREAM_EVENT_HANDLER streamEventHandler; 
    
    /* Application defined context */
    uintptr_t context;

}USB_HOST_AUDIO_STREAMING_INTERFACE;


// *****************************************************************************
/* USB HOST Audio Client Driver Instance Data Structure

  Summary:
    USB HOST Audio Client Driver Instance Data Structure

  Description:
    This structure has the details about USB HOST Audio instance info
    event handler function , states of device , instance , pipes info
    contain all the information about the device driver.

  Remarks:
    None.
 */

typedef struct {

    /* Indicates if the instance is in use or not*/
    bool assigned;
    
    /* Index of this Audio v1.0 Client driver instance */
    uint8_t index; 
    
    /* Device Object Handle */  
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle;

    /* Audio Control Interface Handle */ 
    USB_HOST_DEVICE_INTERFACE_HANDLE acInterfaceHandle;
    
    /* Audio Control Descriptor pointer */ 
    uint8_t * pAudioContorldescriptor; 

    /* Number of Audio Streaming and MIDI Interfaces */
    uint8_t nASInterfaces;
    
    /* Streaming Descriptor count */
    uint8_t countASInterfaces; 
    
    /*Pipe for Audio Control Endpoint  */
    USB_HOST_PIPE_HANDLE controlPipeHandle;

    /* A collection Audio Streaming interfaces */ 
    USB_HOST_AUDIO_STREAMING_INTERFACE streamInf[USB_HOST_AUDIO_V1_STREAMING_INTERFACES_NUMBER];
    
    USB_SETUP_PACKET setupPacket; 
    
    USB_HOST_AUDIO_V1_CONTROL_TRANSFER_OBJ audioControlObj; 
   
    
} USB_HOST_AUDIO_V1_INSTANCE;

/* USB Host Audio Common object */
typedef struct 
{    
    USB_HOST_AUDIO_V1_ATTACH_EVENT_HANDLER attachEventHandler; 
    
    USB_HOST_AUDIO_V1_INSTANCE* prevAudioInstance; 
    
    uintptr_t context; 
    
}  USB_HOST_AUDIO_V1_COMMON_OBJ;


// *****************************************************************************
/*  USB Host Audio Control descriptor header

  Summary:
    USB Host Audio Control descriptor header

  Description:
    USB Host Audio Control descriptor header

  Remarks:
   None.
*/
typedef struct
{
    /* Size of this descriptor */
    uint8_t bLength;

    /* Type of this descriptor */
    uint8_t bDescriptorType;

    /* Subtype of this descriptor */
    uint8_t bDescriptorSubtype; 

} USB_HOST_AUDIO_CONTROL_DESCRIPTOR_HEADER;


// *****************************************************************************
/*  USB Host Audio Control descriptor header

  Summary:
    USB Host Audio Control descriptor header

  Description:
    USB Host Audio Control descriptor header

  Remarks:
   None.
*/
typedef struct
{
    /* Size of this descriptor */
    uint8_t bLength;

    /* Type of this descriptor */
    uint8_t bDescriptorType;

    /* Subtype of this descriptor */
    uint8_t bDescriptorSubtype; 
    
    /* Entity ID */ 
    uint8_t entityID; 

} USB_HOST_AUDIO_CONTROL_ENTITY_DESCRIPTOR_HEADER;

// *****************************************************************************
/*  USB Host Audio Terminal descriptor header

  Summary:
    USB Host Audio Terminal descriptor header

  Description:
    This structure has details about the the common members in the Audio Input
    and Output Terminals. 

  Remarks:
   None.
*/
typedef struct 
{
    /* Size of this descriptor */
    uint8_t bLength;

    /* Type of this descriptor */
    uint8_t bDescriptorType;

    /* Subtype of this descriptor */
    uint8_t bDescriptorSubtype; 
    
    /* Unique ID of the Terminal */ 
    uint8_t bTerminalID; 
    
    /* Characterizes the type of Terminal. See USB Audio Terminal Types. */ 
    uint8_t wTerminalType; 
    
    /* Identifies the Terminal to which this Terminal is associated.*/ 
    uint8_t bAssocTerminal; 
}
USB_HOST_AUDIO_V1_TERMINAL_HEADER_DESCRIPTOR; 

void _USB_HOST_AUDIO_V1_Initialize (void *init); 
void _USB_HOST_AUDIO_V1_Deinitialize(void);
void _USB_HOST_AUDIO_V1_Reinitialize (void * init);
void _USB_HOST_AUDIO_V1_InterfaceAssign
 (
     USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
     USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
     size_t nInterfaces,
     uint8_t * descriptor
 );
void _USB_HOST_AUDIO_V1_InterfaceRelease(USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle);
void _USB_HOST_AUDIO_V1_InterfaceTasks( USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle);
USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_AUDIO_V1_InterfaceEventHandler
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_DEVICE_INTERFACE_EVENT event,
    void * eventData,
    uintptr_t context
);
void _USB_HOST_AUDIO_V1_ReleaseInterface
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
);

void _USB_HOST_AUDIO_V1_InterfaceRelease( USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle ); 


void _USB_HOST_Audio_StreamEnableCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
); 
void _USB_HOST_Audio_StreamDisableCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
); 
bool _USB_HOST_AUDIO_V1_IntrfcHndlToStrmIntrfcPtr
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle, 
    USB_HOST_AUDIO_STREAMING_INTERFACE** strmIntfcPtr, 
    uint8_t *audioIndex, 
    uint8_t *asIntrfcIndex
); 

void _USB_HOST_AUDIO_V1_ControlRequestCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
); 

int _USB_HOST_AUDIO_V1_InterfaceHandleToAudioInstance
( 
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
); 

void _USB_HOST_AUDIO_V1_SetSampleFrequencyCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
); 

USB_HOST_AUDIO_V1_RESULT _USB_HOST_AUDIO_V1_ControlRequest
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_REQUEST_HANDLE * transferHandle,
    USB_SETUP_PACKET *setupPacket,
    void * data
); 

USB_HOST_AUDIO_V1_RESULT _USB_HOST_AUDIO_V1_StreamWrite 
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
    void * source, 
    size_t length, 
    uint8_t flag
); 

USB_HOST_AUDIO_V1_RESULT _USB_HOST_AUDIO_V1_StreamRead 
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
    void * source, 
    size_t length, 
    uint8_t apiVersionFlag    
);

void _USB_HOST_AUDIO_V1_SetSampleRateCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
); 
#endif

 /************ End of file *************************************/
