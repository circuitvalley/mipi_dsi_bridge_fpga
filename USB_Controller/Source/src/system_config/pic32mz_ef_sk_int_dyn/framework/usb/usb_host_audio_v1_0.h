/********************************************************************************
  USB Host Audio v1.0 Class Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_audio_v1_0.h

  Summary:
    USB Host Audio v1_0 Class Driver Interface Header

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface to the USB Host Audio
    v1.0 Class Driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2013-2015 released Microchip Technology Inc.  All rights reserved.

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
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS..
 *******************************************************************************/
#ifndef _USB_HOST_AUDIO_V1_H_
#define _USB_HOST_AUDIO_V1_H_

//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host.h"
#include "usb/usb_host_client_driver.h"
#include "usb/usb_audio_v1_0.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Host Audio v1.0 Object 
 
  Summary: 
    Defines the type of the Audio v1.0 Host client object.

  Description:
    This data type defines the type of the Audio Host client object. This type
    is returned by the client driver attach event handler and is used by the 
    application to open the attached Audio v1.0 Device.  

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_AUDIO_V1_OBJ;

// *****************************************************************************
/* USB Host Audio v1.0 Streaming interface Object 
 
  Summary: 
    Defines the type of the Audio v1.0 Host streaming interface object. 

  Description:
    This data type defines the type of the Audio v1.0 Host streaming interface 
    object. This type is returned by the USB_AUDIO_V1_StreamingInterfaceGetFirst
    and USB_AUDIO_V1_StreamingInterfaceGetNext functions. 

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ;

// *****************************************************************************
/* USB Host Audio v1.0 Streaming Interface Setting Object 
 
  Summary: 
    Defines the type of the Audio v1.0 Host streaming interface setting object. 

  Description:
    This data type defines the type of the Audio v1.0 Host streaming interface
    setting object. This type is returned by the
    USB_AUDIO_V1_StreamingInterfaceSettingGetFirst and
    USB_AUDIO_V1_StreamingInterfaceSettingGetNext functions. 

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ;

// *****************************************************************************
/* USB Host Audio v1.0 Client Driver Request Handle

  Summary:
    USB Host Audio v1.0 Client Driver request handle.

  Description:
    This handle is returned by the Audio v1.0 Host client driver entity control
    functions and audio stream control request functions. Applications should use
    this handle to track a request. 

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_AUDIO_V1_REQUEST_HANDLE;

// *****************************************************************************
/* USB Host Audio v1.0 Client Driver Invalid Request Handle

  Summary:
    USB Host Audio v1.0 Client Driver invalid request handle.

  Description:
    This handle is returned by the Audio v1.0 Client driver command routines when the 
    request could not be scheduled.

  Remarks:
    None.
*/

#define USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID ((USB_HOST_AUDIO_V1_REQUEST_HANDLE)(-1))

// *****************************************************************************
/* USB Host Audio v1.0 Control Entity Object 

  Summary:
    Defines the type of the Audio v1.0 Host control entity object.

  Description:
    This data type defines the type of the object returned by the 
    USB_HOST_AUDIO_V1_ControlEntityGetFirst or 
    USB_HOST_AUDIO_V1_ControlEntityGetNext functions. This application uses
    this object to get more information about that audio control entity. 

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ; 

// *****************************************************************************
/* USB Host Audio stream handle

  Summary:
    Defines the type of the Audio v1.0 Host stream handle.

  Description:
    This data type defines the type of the handle returned by
    USB_HOST_AUDIO_V1_StreamOpen function. The application uses this handle
    to interact with an  Audio Stream. 

  Remarks:
    None.
*/
typedef uintptr_t USB_HOST_AUDIO_V1_STREAM_HANDLE;

// *****************************************************************************
/* USB Host Audio stream Invalid handle

  Summary:
   Defines Audio v1.0 Host stream invalid handle.

  Description:
   This handle is returned by the USB_HOST_AUDIO_V1_StreamOpen function when a stream
   open has failed.  

  Remarks:
    None.
*/

#define USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID ((USB_HOST_AUDIO_V1_STREAM_HANDLE)(-1))

// *****************************************************************************
/* USB Host Audio v1.0 Class Driver Stream Data Transfer Handle

  Summary:
   USB Host Audio v1.0 Class Driver stream data transfer handle.

  Description:
   This handle is returned by the Audio v1.0 Class driver stream data transfer
   functions and should be used by the application to track the transfer,
   especially in cases where transfers are queued.

  Remarks:
   None.
*/

typedef uintptr_t USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE;

// *****************************************************************************
/* USB Host Audio v1.0 Class Driver Invalid Stream Data Transfer Handle Definition
 
  Summary:
   USB Host Audio v1.0 Class Driver invalid stream data transfer handle.

  Description:
   This macro defines a USB Host Audio v1.0 Class Driver invalid stream
   data transfer handle. An invalid transfer handle is returned by the Audio
   v1.0 Class Driver stream data transfer routines when the request was not
   successful. 

  Remarks:
    None.
*/

#define USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE_INVALID ((USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE)(-1))

/*DOM-IGNORE-BEGIN*/
#define USB_HOST_AUDIO_V1_RESULT_MIN -100
/*DOM-IGNORE-END*/

// *****************************************************************************
/* USB Host Audio v1.0 Class Driver Result enumeration.
 
  Summary:
    USB Host Audio v1.0 Class Driver result enumeration.

  Description:
    This enumeration lists the possible USB Host Audio v1.0 Class Driver
    operation results. These values are returned by Audio v1.0 Class Driver
    functions.

  Remarks:
    None.
*/

typedef enum
{
    /* An unknown failure has occurred */
    USB_HOST_AUDIO_V1_RESULT_FAILURE /*DOM-IGNORE-BEGIN*/ = USB_HOST_AUDIO_V1_RESULT_MIN /*DOM-IGNORE-END*/,

    /* The transfer or request could not be scheduled because internal queues
       are full. The request or transfer should be retried */
    USB_HOST_AUDIO_V1_RESULT_BUSY,

    /* The request was stalled */
    USB_HOST_AUDIO_V1_RESULT_REQUEST_STALLED,

    /* A required parameter was invalid */
    USB_HOST_AUDIO_V1_RESULT_INVALID_PARAMETER,

    /* The associated device does not exist in the system. */
    USB_HOST_AUDIO_V1_RESULT_DEVICE_UNKNOWN,

    /* The specified handle is not valid */
    USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID,

    /* The transfer or requested was aborted */
    USB_HOST_AUDIO_V1_RESULT_TRANSFER_ABORTED,

    /* The specified Audio v1.0 object is invalid */ 
    USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID,

    /* No more audio control entity */ 
    USB_HOST_AUDIO_V1_RESULT_END_OF_CONTROL_ENTITY, 
    
    /* No more streaming interface settings present in the audio device */ 
    USB_HOST_AUDIO_V1_RESULT_END_OF_STREAMING_INTERFACE, 

    /* No more interface alternate settings are present in the audio streaming 
       interface */
    USB_HOST_AUDIO_V1_RESULT_END_OF_INTERFACE_SETTINGS,

    /* Indicates that the operation succeeded or the request was accepted and
       will be processed. */ 
    USB_HOST_AUDIO_V1_RESULT_SUCCESS
    
    /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/,
} USB_HOST_AUDIO_V1_RESULT;
   
// *****************************************************************************
/* USB Host Audio v1.0 Stream Event Handler Return Type 
 
  Summary: 
    Returns the type of the USB Host Audio v1.0 stream event handler.

  Description:
    This enumeration lists the possible return values of the USB Host Audio v1.0
    stream event handler.

  Remarks:
    None.
*/

typedef enum
{
    /* This means no response is required */
    USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE_NONE /*DOM-IGNORE-BEGIN*/= 0 /*DOM-IGNORE-END*/
    
} USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE;

// *****************************************************************************
/*  USB Host Audio v1.0 Class Driver Stream Direction
 
  Summary:
    USB Host Audio v1.0 Class Driver stream direction.

  Description:
    This enumeration lists the possible audio stream directions.  

  Remarks:
    None.
*/
typedef enum
{
    /* Stream Direction Host to Device  */ 
    USB_HOST_AUDIO_V1_DIRECTION_OUT  /*DOM-IGNORE-BEGIN*/= 0 /*DOM-IGNORE-END*/,
    
    /* Stream Direction Device to Host  */
    USB_HOST_AUDIO_V1_DIRECTION_IN  /*DOM-IGNORE-BEGIN*/= 1 /*DOM-IGNORE-END*/,
            
} USB_HOST_AUDIO_V1_STREAM_DIRECTION;

// *****************************************************************************
/* Audio v1.0 Class Driver Events

  Summary:
   Identifies the possible events that the Audio v1.0 Class Driver attach event
   handler can generate.
 
 Description:
   This enumeration identifies the possible events that the Audio v1.0 Class
   Driver attach event handler can generate. The application should register an
   event handler using the USB_HOST_AUDIO_V1_AttachEventHandlerSet function
   to receive Audio v1.0 Class Driver Attach events.
*/

typedef enum 
{
    /* This event occurs when the Host layer has detected the Audio v1.0 Class
       Driver instance from a USB Audio v1.0 Device. There is no event data
       associated with this event. */ 
    USB_HOST_AUDIO_V1_EVENT_ATTACH, 

   /* This event occurs when host layer has detached the Audio v1.0 Class
      Driver instance from a USB Audio v1.0 Device. This can happen if the
      device itself was detached or if the device configuration was changed.
      There is no event data associated with this event. */
    USB_HOST_AUDIO_V1_EVENT_DETACH,

} USB_HOST_AUDIO_V1_EVENT;

// *****************************************************************************
/* Audio v1.0 Stream Events

  Summary:
    Identifies the possible events that the Audio v1.0 Stream can generate.

  Description:
    This enumeration identifies the possible events that the Audio v1.0 Stream
    can generate. The application should register an event handler using the
    USB_HOST_AUDIO_V1_StreamEventHandlerSet function to receive Audio v1.0
    stream events.
    
    An event may have data associated with it.  Events that are generated due
    to a transfer of data between the host and device are accompanied by data
    structures that provide the status of the transfer termination. For
    example, the USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE event is
    accompanied by a pointer to a
    USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE_DATA data structure. The
    transferStatus member of this data structure indicates the success or
    failure of the transfer. A transfer may fail due to the device not responding
    on the bus, or if the device stalls any stages of the transfer.  The event
    description provides details on the nature of the event and the data that
    is associated with the event.
*/

typedef enum
{
    /* This event occurs when a Audio v1.0 stream read operation has completed
       (i.e., when the data has been received from the connected Audio v1.0
       stream).  This event is generated after the application calls the
       USB_HOST_AUDIO_V1_StreamRead function. The eventData parameter in the
       event callback function will be of a pointer to a
       USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE_DATA structure. This
       contains details about the transfer handle associated with this read
       request, the amount of data read and the termination status of the read
       request. */
    USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE,

    /* This event occurs when an Audio v1.0 stream write operation has
       completed (i.e., when the data has been written to the connected Audio v1.0
       stream).  This event is generated after the application calls the
       USB_HOST_AUDIO_V1_StreamWrite function. The eventData parameter in the
       event callback function will be of a pointer to a
       USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA structure. This
       contains details about the transfer handle associated with this write
       request, the amount of data written and the termination status of the
       write request. */
    USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE,

   /* This event occurs when an audio streaming set interface request has been 
      completed. This event is generated after the application calls the
      USB_HOST_AUDIO_V1_StreamingInterfaceSet function. The eventData
      parameter in the event callback function will be of a pointer to a
      USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE_DATA. This contains
      details about the request handle associated with the interface set
      request and the termination status of the request.*/
    USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE, 
            
   /* This event occurs when an Audio v1.0 sampling frequency set request has
      been completed. This event is generated after the application calls the
      USB_HOST_AUDIO_V1_StreamSamplingFrequencySet function. The eventData
      parameter in the event callback function will be of a pointer to a
      USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_SET_COMPLETE_DATA. This
      contains details about the request handle associated with this sampling
      frequency set request and the termination status of the request.*/
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_SET_COMPLETE,  

   /* This event occurs when an Audio v1.0 sampling frequency get request has
      been completed. This event is generated after the application calls the
      USB_HOST_AUDIO_V1_StreamSamplingFrequencyGet function. The eventData
      parameter in the event call back function will be of a pointer to a
      USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_GET_COMPLETE_DATA. This
      contains details about the request handle associated with this sampling
      frequency get request and the termination status of the request.*/
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_GET_COMPLETE,

   /* This event occurs when an audio stream is detached from the Host.This 
      can happen if the Audio device itself was detached, or if the Audio
      device configuration was changed. There is no event data associated with
      this event. */
    USB_HOST_AUDIO_V1_STREAM_EVENT_DETACH

} USB_HOST_AUDIO_V1_STREAM_EVENT;

// *****************************************************************************
/*  USB Host Audio v1.0 Class Stream Data Transfer Event Data.
 
  Summary:
    USB Host Audio v1.0 class stream data transfer event data.

  Description:
    This data type defines the data structure returned by the Audio V1.0 stream
    in conjunction with the following events:
    - USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE_DATA
    - USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA

  Remarks:
    None.
*/

typedef struct
{
    /* Transfer handle of this transfer */
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE transferHandle;

    /* Amount of data transferred */
    size_t length;
    
    /* Transfer termination status */
    USB_HOST_AUDIO_V1_RESULT result;

}
USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE_DATA,
USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA;


// *****************************************************************************
/* USB Host Audio v1.0 Class Stream Control Event Data.
 
  Summary:
     USB Host Audio v1.0 class stream control event data.

  Description:
    This data type defines the data structure returned by the Audio V1.0 stream
    in conjunction with the following events:
    - USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE
    - USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_SET_COMPLETE
	- USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_GET_COMPLETE

  Remarks:
    None.
*/

typedef struct
{
    /* Transfer handle of this transfer */
    USB_HOST_AUDIO_V1_REQUEST_HANDLE  requestHandle; 

    /* Transfer termination status */
    USB_HOST_AUDIO_V1_RESULT requestStatus;

} 
USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE_DATA,
USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA,
USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_GET_COMPLETE_DATA;

// *****************************************************************************
/* USB Host Audio v1.0 Client Driver Attach Event Handler Function Pointer Type.

  Summary:
   USB Host Audio v1.0 Client Driver attach event handler function pointer type.

  Description:
   This data type defines the required function signature of the USB Host Audio
   v1.0 Client Driver attach event handling callback function. The application
   must register a pointer to the Audio v1.0 Client Driver attach events handling
   function whose function signature (parameter and return value types) match
   the types specified by this function pointer to receive attach and
   detach events callbacks from the Audio v1.0 Client Driver. The application
   should use the USB_HOST_AUDIO_V1_AttachEventHandlerSet function to register an
   attach event handler. The client driver will call this function with the
   relevant event parameters. The descriptions of the event handler function
   parameters are as follows:
   - audioObj - Audio Device object to which this event is directed
   - event    - Event indicates if it is an Attach or Detach 
   - context  - Value identifying the context of the application that was
                registered with the event handling function

  Remarks:
    None.
*/

typedef void (* USB_HOST_AUDIO_V1_ATTACH_EVENT_HANDLER)
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_EVENT event, 
    uintptr_t context
);

// *****************************************************************************
/* USB Host Audio v1.0 Class Driver Stream Event Handler Function Pointer Type.

  Summary:
   USB Host Audio v1.0 Class Driver stream event handler function pointer type.

  Description:
   This data type defines the required function signature of the USB Host Audio
   v1.0 Class Driver Stream event handling callback function. The application
   must register a pointer to the Audio v1.0 Class Driver stream events handling
   function whose function signature (parameter and return value types) match
   the types specified by this function pointer to receive event callbacks from 
   the Audio v1.0 Class Driver. The application should use the 
   USB_HOST_AUDIO_V1_StreamEventHandlerSet function to register an audio
   stream event handler. The class driver will call this function with the relevant
   event parameters. The descriptions of the stream event handler function
   parameters are as follows:
   - handle           - Handle to the Audio v1.0 stream 
   - event            - Type of event generated
   - eventData        - This parameter should be type casted to an event specific
                        pointer type based on the event that has occurred. Refer 
                        to the USB_HOST_AUDIO_V1_STREAM_EVENT enumeration 
                        description for more information.
   - context          - Value identifying the context of the application that 
                        was registered with the event handling function

  Remarks:
    None.
*/

typedef USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE (* USB_HOST_AUDIO_V1_STREAM_EVENT_HANDLER )
(  
    USB_HOST_AUDIO_V1_STREAM_HANDLE handle,
    USB_HOST_AUDIO_V1_STREAM_EVENT event,
    void * eventData,
    uintptr_t context 
);

//*****************************************************************************
/* USB Host Audio v1.0 Class driver Control Transfer Complete Callback Function
   Pointer type

  Summary:
   USB Host Audio v1.0 class driver control transfer complete callback function
   pointer type.

  Description:
   This data type defines the required function signature of the USB Host Audio
   v1.0 class driver control transfer complete callback function. The client
   must provide a pointer to a control transfer complete callback function
   whose function signature (parameter and return value types) must match the
   types specified by this function pointer to receive notification
   when a control transfer has completed. The application should use the 
   USB_HOST_AUDIO_V1_EntityRequestCallbackSet function to register an entity
   control request callback. The Audio v1.0 client driver will call this
   function with the relevant event parameters. The descriptions of the event
   handler function parameters are as follows:
   - audioObj      - Audio v1.0 client driver object associated with this event
   - requestHandle - Request handle of the control transfer request that caused
                     this event
   - result        - Completion result of the control transfer. This will be
                     USB_HOST_AUDIO_V1_RESULT_SUCCESS if the control transfer
		             completed successfully, USB_HOST_AUDIO_V1_RESULT_FAILURE if 
		             an unknown failure occurred, or 
		             USB_HOST_AUDIO_V1_RESULT_REQUEST_STALLED if the request was 
		             stalled.
   - size          - Size of the data stage that was transferred
   - context       - Value identifying the context of the application that was
                     provided when the USB_HOST_AUDIO_V1_ControlRequest 
		             function was called

  Remarks:
    None.
*/

typedef void (* USB_HOST_AUDIO_V1_ENTITY_REQUEST_CALLBACK)
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
    USB_HOST_AUDIO_V1_RESULT result,
    size_t size,
    uintptr_t context
); 

// ****************************************************************************
// ****************************************************************************
// Section: Client Access Functions
// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_AttachEventHandlerSet
    (
        USB_HOST_AUDIO_V1_ATTACH_EVENT_HANDLER eventHandler,
        uintptr_t context
    );

  Summary:
    Sets an attach/detach event handler.

  Description:
    This function will set an attach event handler. The attach event handler
    will be called when a Audio v1.0 Device has been attached or detached. The 
    context will be returned in the event handler. This function should be 
    called before the bus has been enabled.
    
  Precondition:
    None.

  Parameters:
    eventHandler - Pointer to the attach event handler.
    context      - An application defined context that will be returned in the event
                   handler.

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - If the attach event handler was registered 
	                                     successfully 
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - If the number of registered event handlers 
	                                     has exceeded USB_HOST_AUDIO_V1_ATTACH_LISTENERS_NUMBER

  Remarks:
    This function should be called before the USB_HOST_BusEnable function is called.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_AttachEventHandlerSet
(
    USB_HOST_AUDIO_V1_ATTACH_EVENT_HANDLER eventHandler,
    uintptr_t context
);

// ****************************************************************************
/* Function:
    USB_HOST_DEVICE_OBJ_HANDLE USB_HOST_AUDIO_V1_DeviceObjHandleGet
    (
        USB_HOST_AUDIO_V1_OBJ audioDeviceObj
    );
           
  Summary:
    Returns the device object handle for this Audio v1.0 Device.

  Description:
    This function returns the device object handle for this Audio v1.0 Device.
    This returned handle can be used by the application to perform device-level 
	operations, such as obtaining the string descriptors.

  Precondition:
    None.

  Parameters:
    audioDeviceObj - Audio V1.0 device object handle returned in the 
	                 USB_HOST_AUDIO_V1_ATTACH_EVENT_HANDLER function.

  Returns:
    Will return a valid device object handle if the device is still connected
    to the system. Otherwise, the function will return USB_HOST_DEVICE_OBJ_HANDLE_INVALID.

  Remarks:
    None.                                                                   
*/

USB_HOST_DEVICE_OBJ_HANDLE USB_HOST_AUDIO_V1_DeviceObjHandleGet
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_EntityRequestCallbackSet
    (
        USB_HOST_AUDIO_V1_OBJ audioDeviceObj, 
        USB_HOST_AUDIO_V1_CONTROL_EVENT_HANDLER appAudioEntityRequestCallback,
        uintptr_t context 
    ); 

   Summary:
    Registers an audio entity request callback function with the Audio v1.0
    Client Driver.

   Description:
    This function registers a callback function for the Audio v1.0 control
    entity requests. The Audio v1.0 Host Client Driver will call this
    callback function when an audio entity control request is completed.   
    
   Precondition:
    None.

   Parameters:
    audioDeviceObj                - Audio v1.0 device object.
    appAudioEntityRequestCallback - A pointer to event handler function. If NULL, 
	                                events will not be generated.
    context                       - Application specific context that is 
	                                returned in the event handler.

   Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS     - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - The specified audio object does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE     - An unknown failure occurred

   Remarks:
     None.
*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_EntityRequestCallbackSet
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj, 
    USB_HOST_AUDIO_V1_ENTITY_REQUEST_CALLBACK appAudioEntityRequestCallback,
    uintptr_t context 
); 


// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceGetFirst
    (
        USB_HOST_AUDIO_V1_OBJ audioObj, 
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ* streamingInterfaceObj
    );

   Summary:
    Gets the first streaming interface object from the attached Audio Device.

   Description:
    This function will get the first streaming interface object from the
    attached Audio Device.

   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj              - Audio v1.0 client driver object.
    streamingInterfaceObj - Pointer to an audio streaming interface object. 

   Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request completed successfully
    - USB_HOST_AUDIO_V1_RESULT_END_OF_STREAMING_INTERFACE - No more streaming 
      interfaces are available
    - USB_HOST_AUDIO_V1_RESULT_DEVICE_UNKNOWN - Device is not attached
    - USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - Audio Device object is invalid
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An error has occurred

   Remarks: 
    None.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceGetFirst
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ* streamingInterfaceObj
);


// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceGetNext
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,    
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObjCurrent 
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ* streamingInterfaceObjNext
    );

  Summary:
    Gets the next streaming interface object from the attached Audio Device.

  Description:
    This function will get the next streaming interface object from the
    attached Audio Device. 
    
  Precondition:
    The Audio v1.0 Device should have been attached. 

  Parameters:
    audioObj - Audio Device object.
    streamingInterfaceObjCurrent - Current audio streaming interface object.
    streamingInterfaceObj - Pointer to audio streaming interface object.

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request completed successfully
    - USB_HOST_AUDIO_V1_RESULT_END_OF_STREAMING_INTERFACE - No more streaming 
      interfaces are available 
    - USB_HOST_AUDIO_V1_RESULT_DEVICE_UNKNOWN - Device is not attached
    - USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - Audio Device object is invalid
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An error has occurred

  Remarks:
    None.

*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceGetNext
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObjCurrent, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ* streamingInterfaceObjNext
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetFirst
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *interfaceSettingObj
    );

  Summary:
    Gets the first streaming interface setting object within an audio streaming 
	interface.
    
  Description:
    This function gets the first streaming interface setting object within an 
	audio streaming interface.
    
  Precondition:
    The Audio v1.0 Device should have been attached. 

  Parameters:
    audioObj - Audio device object.
    streamingInterfaceObj - Audio streaming interface object.
    interfaceSettingObj - Pointer to the audio streaming interface setting object. 

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request completed successfully
    - USB_HOST_AUDIO_V1_RESULT_END_OF_INTERFACE_SETTINGS - No more streaming 
      interface settings are available 
    - USB_HOST_AUDIO_V1_RESULT_DEVICE_UNKNOWN - Device is not attached
    - USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - Audio Device object is invalid
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An error has occurred

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetFirst
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *interfaceSettingObj
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetNext 
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObjCurrent, 
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *interfaceSettingObjNext
    );

   Summary:
    Gets the next streaming interface setting object within an audio streaming 
	interface.
    
   Description:
    This function gets the next streaming interface setting object within an 
	audio streaming interface.
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObjCurrent - Current audio streaming interface setting object 
    interfaceSettingObjNext - Pointer to the next audio streaming interface setting object  

   Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request completed successfully
    - USB_HOST_AUDIO_V1_RESULT_END_OF_INTERFACE_SETTINGS - No more streaming 
      interface settings are available 
    - USB_HOST_AUDIO_V1_RESULT_DEVICE_UNKNOWN - Device is not attached
    - USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - Audio Device object is invalid
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An error has occurred

   Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetNext 
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObjCurrent, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *interfaceSettingObjNext
);

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceTerminalLinkGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns the terminal link of the specified streaming interface setting. 
    
   Description:
    This function returns the terminal link of the specified streaming interface
    setting.
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObj - Audio streaming interface setting object 

   Returns:
    The terminal link of the audio streaming interface setting. 

   Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceTerminalLinkGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
);

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceFormatTagGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns the format tag of the specified streaming interface setting. 
    
   Description:
    This function returns the format tag link of the specified streaming interface
    setting.
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObj - Audio streaming interface setting object 

   Returns:
    The format tag of the audio streaming interface setting. 

   Remarks:
    None.

*/
USB_AUDIO_V1_FORMAT_TAG USB_HOST_AUDIO_V1_StreamingInterfaceFormatTagGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
);

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceChannelNumbersGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns the number of channels of the specified streaming interface
    setting. 
    
   Description:
    This function returns the number of channels of the specified streaming 
	interface setting. 
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObj - Audio streaming interface setting object 

   Returns:
    The number of channels present in the audio streaming interface setting. 

   Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceChannelNumbersGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
); 

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceSubFrameSizeGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns the sub-frame size of the specified streaming interface
    setting. 
    
   Description:
    This function returns the sub-frame size of the specified streaming 
	interface setting.
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObj - Audio streaming interface setting object 

   Returns:
    The sub-frame size of the audio streaming interface setting. 

   Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceSubFrameSizeGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
);


// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceBitResolutionGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns the bit resolution of the specified streaming interface
    setting. 
    
   Description:
    This function returns the bit resolution size of the specified streaming 
	interface setting.
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObj - Audio streaming interface setting object 

   Returns:
    The bit resolution size of the audio streaming interface setting. 

   Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceBitResolutionGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
);

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequencyTypeGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns the sampling frequency type of the specified streaming interface
    setting. 
    
   Description:
    This function returns the sampling frequency type of the specified streaming 
    interface setting.
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObj - Audio streaming interface setting object 

   Returns:
     The sampling frequency type of the audio streaming interface setting. 
     - 0        - Continuous Sampling frequency is supported
     - 1 to 255 - The number of discrete sampling frequencies supported by the
                  audio streaming interface

   Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequencyTypeGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
);


// *****************************************************************************
/* Function:
    uint8_t* USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequenciesGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns the sampling frequencies supported by the specified streaming interface
    setting. 
    
   Description:
    This function returns the sampling frequencies supported by the specified streaming 
    interface setting. 
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObj - Audio streaming interface setting object 

   Returns:
    A pointer to the sampling frequencies supported by the audio streaming 
	interface setting. 
     
   Remarks:
    None.

*/
uint8_t* USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequenciesGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_STREAM_DIRECTION USB_HOST_AUDIO_V1_StreamingInterfaceDirectionGet
    (
        USB_HOST_AUDIO_V1_OBJ audioObj, 
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
    );

   Summary:
    Returns the direction of the specified streaming interface setting. 
    
   Description:
    This function returns the direction of the specified streaming 
    interface setting.
    
   Precondition:
    The Audio v1.0 Device should have been attached. 

   Parameters:
    audioObj - Audio Device object
    streamingInterfaceObj - Audio streaming interface object
    interfaceSettingObj - Audio streaming interface setting object 

   Returns:
    - USB_HOST_AUDIO_V1_DIRECTION_OUT - Host to Device
    - USB_HOST_AUDIO_V1_DIRECTION_IN  - Device to Host
     
   Remarks:
    None.

*/
USB_HOST_AUDIO_V1_STREAM_DIRECTION USB_HOST_AUDIO_V1_StreamingInterfaceDirectionGet
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ streamingInterfaceObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
); 
        
// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_REESULT USB_HOST_AUDIO_V1_ControlEntityGetFirst
   (
       USB_HOST_AUDIO_V1_OBJ  audioObj, 
       USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ * pEntityObject
   ); 

  Summary:
    Retrieves the handle to the first audio control entity   

  Description:
    This function retrieves the handle to the first audio control entity. 
    
  Precondition:
    None.

  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 
    pEntityObject - pointer to the Audio control entity handle.

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_END_OF_CONTROL_ENTITY - No more audio control
      entities are available
    - USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred
    
  Remarks:
    None.

*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_ControlEntityGetFirst
(
    USB_HOST_AUDIO_V1_OBJ  audioObj, 
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ * pEntityObject
);
// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_REESULT USB_HOST_AUDIO_V1_ControlEntityGetNext
   (
       USB_HOST_AUDIO_V1_OBJ  audioObj, 
       USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObjectCurrent
       USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ * pEntityObject
   ); 

  Summary:
    Retrieves the handle to the next audio control entity.

  Description:
    This function retrieves the handle to the next audio control entity. 
    
  Precondition:
    None.

  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 
    entityObjectCurrent - Handle to current audio control entity. 
    pEntityObject  -  pointer to audio control entity handle. 

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_END_OF_CONTROL_ENTITY - No more audio control
      entities are available
    - USB_HOST_AUDIO_V1_RESULT_OBJ_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_ControlEntityGetNext
(
    USB_HOST_AUDIO_V1_OBJ  audioObj, 
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObjectCurrent,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ * pEntityObject
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_EntityObjectGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        uint8_t entityId,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* entityObj
    ); 

  Summary:
    Retrieves the entity object for the entity ID.

  Description:
    This function retrieves the entity object for the entity ID.  
    
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object
    entityId     - Entity ID
    entityObject - Audio control entity object 

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
	- USB_HOST_AUDIO_V1_RESULT_FAILURE - The entity Id could not be found or an 
	  unknown failure occurred

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_EntityObjectGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    uint8_t entityId,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* entityObj
);

// *****************************************************************************
/* Function:
    USB_AUDIO_V1_ENTITY_TYPE USB_HOST_AUDIO_V1_EntityTypeGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj, 
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns the entity type of the audio control entity.

  Description:
    This function returns the entity type of the audio control entity. 
	Prior to calling this function the entity object should be obtained by calling 
    USB_HOST_AUDIO_V1_ControlEntityGetFirst, USB_HOST_AUDIO_V1_ControlEntityGetNext, or
    USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object
    entityObject - Audio control entity object 

  Returns:
    USB_AUDIO_V1_ENTITY_TYPE. 

  Remarks:
    None.

*/
USB_AUDIO_V1_ENTITY_TYPE USB_HOST_AUDIO_V1_EntityTypeGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj, 
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
);

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_TerminalIDGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns the terminal ID of the audio control entity.

  Description:
    This function returns the Terminal ID of the Audio Control entity. Prior to
    calling this function the entity object should be obtained by calling  
    USB_HOST_AUDIO_V1_ControlEntityGetFirst, USB_HOST_AUDIO_V1_ControlEntityGetNext, 
	or USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object 
    entityObject - Audio control entity object 

  Returns:
    The terminal ID of the audio control entity object.

  Remarks:
    None.

*/
 
uint8_t USB_HOST_AUDIO_V1_TerminalIDGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
);

// *****************************************************************************
/* Function:
    USB_AUDIO_V1_TERMINAL_TYPE USB_HOST_AUDIO_V1_TerminalTypeGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns the terminal type of the audio control entity.

  Description:
    This function returns the Terminal type of the audio control entity. Prior to
    calling this function Entity Object should be obtained by calling the 
    USB_HOST_AUDIO_V1_ControlEntityGetFirst,
    USB_HOST_AUDIO_V1_ControlEntityGetNext, or
    USB_HOST_AUDIO_V1_EntityObjectGet function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object
    entityObject - Audio control entity Object 

  Returns:
    The terminal type.

  Remarks:
    None.

*/
USB_AUDIO_V1_TERMINAL_TYPE USB_HOST_AUDIO_V1_TerminalTypeGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
);

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_TerminalAssociationGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns the associated terminal ID of the audio control terminal.

  Description:
    This function returns the ID of the associated terminal type of the audio control
    terminal.  Prior to calling this function the entity object should be obtained by
    calling USB_HOST_AUDIO_V1_ControlEntityGetFirst, USB_HOST_AUDIO_V1_ControlEntityGetNext, 
	or USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object
    entityObject - Audio control entity object 

  Returns:
    The ID of the associated terminal.

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_TerminalAssociationGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
);

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_TerminalInputChannelNumbersGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns the number of logical output channels in the terminal's output audio 
    channel cluster.

  Description:
    This function returns the number of logical output channels in the terminal's
    output audio channel cluster. This function is only applicable to an input
    terminal.  Prior to calling this function the entity object should be obtained by
    calling USB_HOST_AUDIO_V1_ControlEntityGetFirst, USB_HOST_AUDIO_V1_ControlEntityGetNext, 
	or USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object.
    entityObject - Audio control entity object 

  Returns:
    The number of logical output channels in the terminal's output audio channel 
    cluster.  

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_TerminalInputChannelNumbersGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
);

// *****************************************************************************
/* Function:
    USB_AUDIO_CHANNEL_CONFIG USB_HOST_AUDIO_V1_TerminalInputChannelConfigGet
	(
		USB_HOST_AUDIO_V1_OBJ  audioObj,
		USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
	); 

  Summary:
    Returns a structure that describes the spatial location of the logical 
	channels of in the terminal's output audio channel cluster.
	
  Description:
    This function returns a structure that describes the spatial location of 
	the logical channels of in the terminal's output audio channel cluster. 
	This function is only applicable to an input terminal.  Prior to calling this
	function the entity object should be obtained by calling 
	USB_HOST_AUDIO_V1_ControlEntityGetFirst, USB_HOST_AUDIO_V1_ControlEntityGetNext, 
	or USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object 
    entityObject - Audio control entity object 

  Returns:
    The structure that describes the spatial location of the logical channels.

  Remarks:
    None.

*/
USB_AUDIO_CHANNEL_CONFIG USB_HOST_AUDIO_V1_TerminalInputChannelConfigGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
); 

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_TerminalSourceIDGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns the ID of the unit or terminal to which this terminal is connected.

  Description:
    This function returns the ID of the unit or terminal to which this terminal is
    connected. This function is only applicable to an output terminal.  Prior to
    calling this function the entity object should be obtained by calling 
    USB_HOST_AUDIO_V1_ControlEntityGetFirst, USB_HOST_AUDIO_V1_ControlEntityGetNext, 
	or USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object 
    entityObject - Audio control entity object 

  Returns:
    The ID of the unit or terminal to which this terminal is connected. 

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_TerminalSourceIDGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
);

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
    This function returns the ID of the D of the Feature Unit.  This function is
    only applicable to Feature Unit. Prior to calling this function Entity
    Object should be obtained by calling the
    USB_HOST_AUDIO_V1_ControlEntityGetFirst,
    USB_HOST_AUDIO_V1_ControlEntityGetNext, or
    USB_HOST_AUDIO_V1_EntityObjectGet function. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 device object. 
    entityObject - Audio control entity Object 

  Returns:
    The ID of the feature unit. 

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_FeatureUnitIDGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
); 
// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_FeatureUnitSourceIDGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    );

  Summary:
    Returns the ID of the unit or terminal to which this feature unit is connected.

  Description:
    This function returns the ID of the Unit or Terminal to which this feature unit
    is connected. This function is only applicable to a feature unit. Prior to
    calling this function the entity object should be obtained by calling  
    USB_HOST_AUDIO_V1_ControlEntityGetFirst,
    USB_HOST_AUDIO_V1_ControlEntityGetNext, or
    USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object 
    entityObject - Audio control entity object 

  Returns:
    The ID of the unit or terminal to which this feature unit is connected. 

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_FeatureUnitSourceIDGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
);
// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_FeatureUnitChannelNumbersGet
    (
        USB_HOST_AUDIO_V1_OBJ  audioObj,
        USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
    ); 

  Summary:
    Returns the number of channels.

  Description:
    This function returns the number of channels. This function is only applicable
    to a feature unit. Prior to calling this function the entity object should be 
    obtained by calling USB_HOST_AUDIO_V1_ControlEntityGetFirst,
    USB_HOST_AUDIO_V1_ControlEntityGetNext, or
    USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object 
    entityObject - Audio control entity object 

  Returns:
    The number of channels. 

  Remarks:
    None.

*/
uint8_t USB_HOST_AUDIO_V1_FeatureUnitChannelNumbersGet
(
    USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject
);

// *****************************************************************************
/* Function:
    bool USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists
    (
         USB_HOST_AUDIO_V1_OBJ  audioObj,
         USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
         uint8_t channel
    );

  Summary:
    Returns "true" if mute control exists for the specified channel of the 
	feature unit. 

  Description:
    This function returns "true" if mute control exists on the specified channel
    of the feature unit. Channel 0 indicates Master mute control.
    This function is only applicable to a feature unit. Prior to calling this
    function the entity object should be obtained by calling  
    USB_HOST_AUDIO_V1_ControlEntityGetFirst,
    USB_HOST_AUDIO_V1_ControlEntityGetNext, or
    USB_HOST_AUDIO_V1_EntityObjectGet. 
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object 
    entityObject - Audio control entity object 
    channel      - Channel number

  Returns:
    - true  - Mute control exists on the specified channel 
    - false - Mute control does not exist on the specified channel 

  Remarks:
    None.

*/
bool USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     uint8_t channel
); 
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
    Schedules a set mute control request to the specified channel.  

  Description:
    This function schedules a set mute control request to the specified
    channel.  Prior to calling this function the user should check if mute control
    exists on the specified channel by calling the 
    USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists function. 
 
    If the request was scheduled successfully, the requestHandle parameter will 
	contain a request handle that uniquely identifies this transfer. If the transfer
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 Client Driver will call
    the callback function that was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet function. The context
    parameter specified here will be returned in the callback.

  Parameters:
    audioObj      - USB Host Audio v1.0 Device object 
    entityObject  - Audio control entity object 
    requestHandle - Output parameter that will contain the handle to this request
    channelNumber - Channel Number 
    muteStatus    - Value of mute control, where 1 mutes the channel and 0 removes unmutes 

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request was scheduled successfully.
      requestHandle will contain a valid request handle.
   -  USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently busy. 
      Retry the request. 
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred. requestHandle will
      contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
    - USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or requestHandle pointer
      is NULL

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelMuteSet
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
     bool *muteStatus
); 
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
    Schedules a get mute control request to the specified channel.  

  Description:
    This function schedules a get mute control request to the specified
    channel. Prior to calling this function the user should check if mute control
    exists on the specified channel by calling the 
    USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists function. 
 
    If the request was scheduled successfully, the requestHandle parameter will contain a
    request handle that uniquely identifies this request. If the transfer
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 Client Driver will call
    the callback function that was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet function. The context
    parameter specified here will be returned in the callback.
     
  Parameters:
    audioObj      - USB Host Audio v1.0 Device object 
    entityObject  - Audio control entity object 
    requestHandle - Output parameter that will contain the handle to this request
    channelNumber - Channel number
    muteStatus    - Output parameter that will contain Current Mute status when
                    the request is completed and a callback is received   

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request was scheduled successfully.
      requestHandle will contain a valid request handle.
    - USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently
      busy.  Retry the request. 
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
      requestHandle will contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
    - USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or
      requestHandle pointer is NULL

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelMuteGet
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
     bool *muteStatus
); 

// *****************************************************************************
/* Function:
    bool USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists
    (
         USB_HOST_AUDIO_V1_OBJ  audioObj,
         USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
         uint8_t channel
    ); 

  Summary:
    Returns "true" if volume control exists for the specified channel of the
    feature unit. 

  Description:
    This function returns "true" if volume control exists on the specified
    channel of the feature unit. Channel 0 indicates master volume control.
    This function is only applicable to a feature unit. Prior to calling this
    function the entity object should be obtained by calling  
    USB_HOST_AUDIO_V1_ControlEntityGetFirst,
    USB_HOST_AUDIO_V1_ControlEntityGetNext, or
    USB_HOST_AUDIO_V1_EntityObjectGet.
     
  Parameters:
    audioObj     - USB Host Audio v1.0 Device object 
    entityObject - Audio control entity object 
    channel      - Channel number 

  Returns:
    - true  - Volume control exists on the specified channel 
    - false - Volume control does not exist on the specified channel 

  Remarks:
    None.

*/

bool USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     uint8_t channel
);

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
    Schedules a set current volume control request to the specified channel.  

  Description:
    This function schedules a set current volume request to the specified
    channel.  Prior to calling this function the user should check if volume
    control exists on the specified channel by calling the 
    USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists function. 
 
    If the request was scheduled successfully, the requestHandle parameter will contain a
    request handle that uniquely identifies this request. If the request
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 Client Driver will call
    the callback function that was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet function. The context
    parameter specified here will be returned in the callback. 
     
  Parameters:
    audioObj      - USB Host Audio v1.0 Device object 
    entityObject  - Audio control entity object 
    requestHandle - Output parameter that will contain the handle to this request
    channelNumber - Channel number to which the volume control is addressed 
    volume        -  Current volume control value that should be set in the Audio Device

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request was scheduled successfully.
      requestHandle will contain a valid request handle.
    - USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently
       busy.  Retry the request. 
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
      requestHandle will contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
    - USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or
      requestHandle pointer is NULL

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
     uint16_t *volume
); 


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
    Schedules a get current volume control request to the specified channel.  

  Description:
    This function schedules a get current volume control request to the
    specified channel. Prior to calling this function the user should check if
    volume control exists on the specified channel by calling the 
    USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists function. 
 
    If the request was scheduled successfully, the requestHandle parameter will 
	contain a request handle that uniquely identifies this request. If the request
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 Client Driver will call
    the callback function that was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet function. The context
    parameter specified here will be returned in the callback. 
     
  Parameters:
    audioObj      - USB Host Audio v1.0 Device object
    entityObject  - Audio control entity object 
    requestHandle - Output parameter that will contain the handle to this request
    channelNumber - Channel number to which the volume control is addressed 
    volume        - Output parameter that will contain the current volume when a 
                    request is completed and a callback is received

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request was scheduled successfully.
      requestHandle will contain a valid request handle.
    - USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently
      busy.  Retry the request. 
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
      requestHandle will contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID
    - USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or
      requestHandle pointer is NULL

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeGet
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
     uint16_t *volume
);


// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSubRangeNumbersGet
	(
		USB_HOST_AUDIO_V1_OBJ  audioObj,
		USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
		USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
		uint8_t channelNumber,
		uint16_t *nSubRanges	 
	);

  Summary:
    Schedules a control request to an Audio Device feature unit to get the number 
	of sub-ranges supported by the volume control on the specified channel. 

  Description:
    This function schedules a control request to the Audio Device feature unit to 
	get the number of sub-ranges supported by the volume control on the specified 
	channel. Prior to calling this function the user should check if volume control 
	exists on the specified channel by calling the 
	USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeExists function. 
 
    If the request was scheduled successfully, the requestHandle parameter will contain a
    request handle that uniquely identifies this request. If the request
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 Client Driver will call
    the callback function that was set using the 
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet function. The context
    parameter specified here will be returned in the callback. 
     
  Parameters:
    audioObj      - USB Host Audio v1.0 Device object
    entityObject  - Audio control entity object 
    requestHandle - Output parameter that will contain the handle to this request
    channelNumber - Channel number to which the volume control is addressed 
    nSubRanges    - Output parameter that will contain the number of sub-ranges 
	                when the request is completed and a callback is received

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request was scheduled successfully.
      requestHandle will contain a valid request handle.
    - USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently
      busy.  Retry the request. 
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
      requestHandle will contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
    - USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or
      requestHandle pointer is NULL

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSubRangeNumbersGet
(
	USB_HOST_AUDIO_V1_OBJ  audioObj,
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
    USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
    uint8_t channelNumber,
	uint16_t *nSubRanges	 
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeRangeGet
	(
		 USB_HOST_AUDIO_V1_OBJ  audioObj,
		 USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
		 USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
		 uint8_t channelNumber,
		 void * data, 
		 size_t size
	);

  Summary:
    Schedules a control request to the Audio Device feature unit to get the range 
	supported by the volume control on the specified channel. 

  Description:
    This function schedules a control request to the Audio Device feature unit to get 
	the range supported by the volume control on the specified channel. 
	
	Prior to calling this function the user should call the 
    USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSubRangeNumbersGet function 
	to know how many sub-ranges are supported.

	Users should calculate the 'size' parameter of this function, as follows: 
	
	<c>size = Size of number of ranges + nSubRanges * (Size (MIN) + Size (MAX) + Size of (RES))</c> 
	
    If the request was scheduled successfully, the requestHandle parameter will contain a
    request handle that uniquely identifies this request. If the request
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID.
 
    When the control request completes, the Audio v1.0 Client Driver will call
    the callback function that was set using the
    USB_HOST_AUDIO_V1_EntityRequestCallbackSet function. The context
    parameter specified here will be returned in the callback. 
     
  Parameters:
    audioObj      - USB Host Audio v1.0 Device object 
    entityObject  - Audio control entity object 
    requestHandle - Output parameter that will contain the handle to this request
    channelNumber - Channel number to which the volume control is addressed 
    nSubRanges    - Output parameter that will contain the number of sub-ranges 
	                when the request is completed and a callback is received

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The request was scheduled successfully.
      requestHandle will contain a valid request handle.
    - USB_HOST_AUDIO_V1_RESULT_BUSY - The control request mechanism is currently
      busy.  Retry the request. 
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred.
      requestHandle will contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
    - USB_HOST_AUDIO_V1_RESULT_PARAMETER_INVALID - The data pointer or
      requestHandle pointer is NULL

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeRangeGet
(
     USB_HOST_AUDIO_V1_OBJ  audioObj,
     USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObject, 
     USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
     uint8_t channelNumber,
	 void * data, 
	 size_t size
);

// ****************************************************************************
// ****************************************************************************
// Section: Audio Stream Access Functions
// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_STREAM_HANDLE USB_HOST_AUDIO_V1_StreamOpen
    ( 
        USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ audiostreamingInterfaceObj  
    );

  Summary:
    Opens the specified audio stream. 

  Description:
    This function will open the specified audio stream. Once opened, the audio
    stream can be accessed via the handle that this function returns. The
    audiostreamingInterfaceObj parameter is the value returned in the
    USB_HOST_AUDIO_V1_StreamingInterfaceGetFirst or
    USB_HOST_AUDIO_V1_StreamingInterfaceGetNext functions.

  Precondition:
    The audio streaming interface object should be valid.

  Input:
    audiostreamingInterfaceObj - Audio streaming interface object 

  Return:
    Will return a valid handle if the audio stream could be opened
    successfully. Otherwise, USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID is returned. 
	The function will return a valid handle if the stream is ready to be opened.

  Remarks:
    None.                                                                   
*/

USB_HOST_AUDIO_V1_STREAM_HANDLE USB_HOST_AUDIO_V1_StreamOpen
( 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ audiostreamingInterfaceObj  
); 

// ****************************************************************************
/* Function:
    void USB_HOST_AUDIO_V1_StreamClose
    ( 
        USB_HOST_AUDIO_V1_STREAM_HANDLE audioSteamHandle
    );
           
  Summary:
    Closes the audio stream. 

  Description:
    This function will close the open audio stream. This closes the association
    between the application entity that opened the audio stream and the audio
    stream. The audio stream handle becomes invalid.

  Precondition:
    None.

  Parameters:
    audioSteamHandle - handle to the audio stream obtained from the
    USB_HOST_AUDIO_V1_StreamOpen function.

  Returns:
    None.

  Remarks:
    The device handle becomes invalid after calling this function.                                                                   
*/

void USB_HOST_AUDIO_V1_StreamClose
( 
    USB_HOST_AUDIO_V1_STREAM_HANDLE audioStreamHandle
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamEventHandlerSet
    (
        USB_HOST_AUDIO_V1_STREAM_HANDLE handle,
        USB_HOST_AUDIO_V1_STREAM_EVENT_HANDLER appAudioHandler,
        uintptr_t context
    ); 

  Summary:
    Registers an event handler with the Audio v1.0 Client Driver stream.

  Description:
    This function registers a client specific Audio v1.0 stream event handler.
    The Audio v1.0 Host Client Driver will call the appAudioHandler function
    specified as the second argument with relevant event and associated event data 
	in response to audio stream data transfers that have been scheduled by the
    client.
    
  Precondition:
    None.

  Parameters:
    handle       - The handle to the Audio v1.0 stream
    eventHandler - A pointer to event handler function. If NULL, events
                   will not be generated.
    context      - The application specific context that is returned in the event handler

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamEventHandlerSet
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE handle,
    USB_HOST_AUDIO_V1_STREAM_EVENT_HANDLER appAudioHandler,
    uintptr_t context
); 

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
    This function schedules an audio stream enable request for the specified
    audio stream. An audio stream must be enable before scheduling any data
    transfer with the stream. A USB_HOST_AUDIO_V1_STREAM_EVENT_ENABLE_COMPLETE 
	event is generated when this request is completed. 
	USB_HOST_AUDIO_V1_STREAM_EVENT_ENABLE_COMPLETE_DATA returns the status and 
	request handle of the request. 
        
       
  Precondition:
    The audio stream should have been opened. Only one audio stream from an audio
    stream group can be enabled at a time. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 stream.
    requestHandle - Handle to the stream enable request. 

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamingInterfaceSet
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle, 
    USB_HOST_AUDIO_V1_REQUEST_HANDLE * requestHandle,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ interfaceSettingObj
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamSamplingFrequencySet
    (
        USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
        USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
        uint32_t *samplingFrequency
    )
  Summary:
    Schedules an audio stream set sampling rate request for the specified 
    audio stream. 

  Description:
    This function schedules an audio stream set sampling rate request for the
    specified audio stream. A USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE 
	event is generated when this request is completed. 
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA returns 
    the status and request handle of the request. 
       
  Precondition:
    The audio stream should have been opened. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 stream
    requestHandle - Handle to the stream set sampling rate request
    samplingRate  - Pointer to the sampling rate

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamSamplingFrequencySet
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_REQUEST_HANDLE *requestHandle,
    uint32_t *samplingFrequency
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamSamplingFrequencyGet
    (
        USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
        USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
        uint32_t *samplingFrequency
    )
  Summary:
    Schedules an audio stream get sampling rate request for the specified 
    audio stream. 

  Description:
    This function schedules an audio stream set sampling rate request for the
    specified audio stream. A USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE 
	event is generated when this request is completed. 
    USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA returns 
    the status and request handle of the request. 
        
       
  Precondition:
    The audio stream should have been opened. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 stream
    requestHandle - Handle to the stream set sampling rate request
    samplingRate  - Pointer to the sampling rate

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamSamplingFrequencyGet
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_REQUEST_HANDLE *requestHandle,
    uint32_t *samplingFrequency
);

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
    Schedules an audio stream write request for the specified audio stream. 

  Description:
    This function schedules an audio stream write request for the specified 
    audio stream. A USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE event is 
	generated when this request is completed. 
    USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA returns 
    the status and request handle of the request.
    
  Precondition:
    The audio stream should have been opened and enabled. The direction of the 
    audio stream should be USB_HOST_AUDIO_V1_DIRECTION_OUT. 

  Parameters:
    streamHandle    - Handle to the Audio v1.0 stream
    transferHandle  - Handle to the stream write transfer request 
    source          - Pointer to the buffer containing data to be written to the 
                      device
    length          - Amount of data to write (in bytes)

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.

*/

USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamWrite 
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
    void * source, 
    size_t length
); 

// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamRead 
   (
       USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
       USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
       void * source, 
       size_t length
   ); 

  Summary:
    Schedules an audio stream read request for the specified audio stream. 

  Description:
    This function schedules an audio stream read request for the specified 
    audio stream. A USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE event is generated 
	when this request is completed. 
    USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE_DATA returns 
    the status and request handle of the request.
    
  Precondition:
    The audio stream should have been opened and enabled. The direction of the 
    audio stream should be USB_HOST_AUDIO_V1_DIRECTION_IN. 

  Parameters:
    streamHandle    - Handle to the Audio v1.0 stream
    transferHandle  - Handle to the stream read transfer request 
    source          - Pointer to the buffer containing data to be read from the 
                      device 
    length          - Amount of data to read (in bytes)

  Returns:
    - USB_HOST_AUDIO_V1_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.

*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_StreamRead 
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
    void * source, 
    size_t length
);

// *****************************************************************************
// Section: Global Data Types. This section is specific to PIC32 implementation
//          of the USB Host Audio V1 Client Driver
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB HOST Audio V1 Client Driver Interface
 
  Summary:
    USB HOST Audio v1.0 Client Driver interface.

  Description:
    This macro should be used by the application in the TPL table while adding
    support for the USB Audio v1.0 Host Client Driver.

  Remarks:
    None.
*/ 
/*DOM-IGNORE-BEGIN*/extern const USB_HOST_CLIENT_DRIVER gUSBHostAudioV1Driver; /*DOM-IGNORE-END*/
#define USB_HOST_AUDIO_V1_INTERFACE  /*DOM-IGNORE-BEGIN*/&gUSBHostAudioV1Driver /*DOM-IGNORE-END*/

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************      
// *****************************************************************************
// Section: Depreciated API - Not recommended for new applications 
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Host Audio v1.0 Object 
 
  Summary: 
    Defines the type of the Audio v1.0 Host client object.

  Description:
    This type defines the type of the Audio Host client object. This type
    is returned by the attach event handler and is used by the application to
    open the attached Audio v1.0 Device.  

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_OBJ USB_HOST_AUDIO_V1_OBJ

// *****************************************************************************
/* USB Host Audio v1.0 Stream Object 
 
  Summary: 
    Defines the type of the Audio v1.0 Host stream object. 

  Description:
    This type defines the type of the Audio v1.0 Host stream object. This type
    is returned by USB_AUDIO_V1_0_StreamGetFirst and USB_AUDIO_V1_0_StreamGetNext
    as part of USB_HOST_AUDIO_V1_0_STREAM_INFO structure.  

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_AUDIO_V1_0_STREAM_OBJ;

// *****************************************************************************
/*  USB Host Audio v1.0 Client Driver Request Handle

  Summary:
    USB Host Audio v1.0 Client Driver request handle.

  Description:
    This is returned by the Audio v1.0 Client Driver command routines and should 
    be used by the application to track the command especially in cases where
    transfers are queued.

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_REQUEST_HANDLE USB_HOST_AUDIO_V1_REQUEST_HANDLE

// *****************************************************************************
/*  USB Host Audio v1.0 Client Driver Invalid Request Handle

  Summary:
    USB Host Audio v1.0 Client Driver invalid request handle.

  Description:
    This is returned by the Audio v1.0 Client Driver command routines when the request
    could not be scheduled.

  Remarks:
    None.
*/

#define USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID ((USB_HOST_AUDIO_V1_0_REQUEST_HANDLE)(-1))

// *****************************************************************************
/* USB HOST Audio Client Driver Interface
 
  Summary:
    USB HOST Audio Client Driver interface.

  Description:
    This macro should be used by the application in the TPL table while adding
    support for the USB Audio Host Client Driver.

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_INTERFACE (void*)USB_HOST_AUDIO_V1_INTERFACE

// *****************************************************************************
/* USB Host Audio stream handle

  Summary:
    Defines the type of the Audio v1.0 Host stream handle.

  Description:
    This type defines the type of the handle returned by the 
    USB_HOST_AUDIO_V1_0_StreamOpen function. This application uses this
    handle to interact with an  audio stream. 

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_STREAM_HANDLE USB_HOST_AUDIO_V1_STREAM_HANDLE

// *****************************************************************************
/* USB Host Audio stream Invalid handle

  Summary:
    Defines the type of the Audio v1.0 Host stream invalid handle.

  Description:
    This is returned by the USB_HOST_AUDIO_V1_0_StreamOpen function when 
    a stream open request has failed.  

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_STREAM_HANDLE_INVALID USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID

// *****************************************************************************
/*  USB Host Audio v1.0 Class Driver Transfer Handle

  Summary:
    USB Host Audio v1.0 Class Driver transfer handle.

  Description:
    This is returned by the Audio v1.0 Class Driver command and data transfer 
    routines and should be used by the application to track the transfer 
    especially in cases where transfers are queued.

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_STREAM_TRANSFER_HANDLE USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE

// *****************************************************************************
/*  USB Host Audio v1.0 Class Driver Invalid Transfer Handle Definition
 
  Summary:
    USB Host Audio v1.0 Class Driver invalid transfer handle definition.

  Description:
    This macro defines a USB Host Audio v1.0 Class Driver invalid transfer
    handle. A invalid transfer handle is returned by the Audio v1.0 Class Driver
    data and command transfer routines when the request was not successful. 

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_STREAM_TRANSFER_HANDLE_INVALID USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE_INVALID

// *****************************************************************************
/* USB Host Audio v1.0 Class Driver Result enumeration.
 
  Summary:
    USB Host Audio v1.0 Class Driver audio result enumeration.

  Description:
    This enumeration lists the possible USB Host Audio v1.0 Class Driver operation
    results. These values are returned by Audio v1.0 Class Driver functions.

  Remarks:
    None.
*/

typedef enum
{
	
	/* The transfer or request could not be scheduled because internal
     * queues are full. The request or transfer should be retried */
    USB_HOST_AUDIO_V1_0_RESULT_BUSY  = /*  DOM-IGNORE-BEGIN */ USB_HOST_RESULT_REQUEST_BUSY, /* DOM-IGNORE-END*/

	/* The transfer or requested was aborted */
    USB_HOST_AUDIO_V1_0_RESULT_TRANSFER_ABORTED,
    
	/* The request was stalled */    
    USB_HOST_AUDIO_V1_0_RESULT_REQUEST_STALLED,

	/* The specified Audio v1.0 Object is Invalid */ 
    USB_HOST_AUDIO_V1_0_RESULT_OBJ_INVALID,

    /* No more audio stream present in the Device */       
    USB_HOST_AUDIO_V1_0_RESULT_END_OF_STREAM_LIST,
    
	/* DOM-IGNORE-BEGIN */
    USB_HOST_AUDIO_V1_0_RESULT_ERROR_INTERFACE_UNKNOWN, 
    /* DOM-IGNORE-END*/
	
	/* A required parameter was invalid */
    USB_HOST_AUDIO_V1_0_RESULT_PARAMETER_INVALID, 
    
	/* DOM-IGNORE-BEGIN */
    USB_HOST_AUDIO_V1_0_RESULT_CONFIGURATION_UNKNOWN, 

    USB_HOST_AUDIO_V1_0_RESULT_BUS_NOT_ENABLED,

    USB_HOST_AUDIO_V1_0_RESULT_BUS_UNKNOWN,
	/* DOM-IGNORE-END*/
	
    /* The specified device does not exist in the system */
    USB_HOST_AUDIO_V1_0_RESULT_DEVICE_UNKNOWN,

    /* An unknown failure has occurred */
    USB_HOST_AUDIO_V1_0_RESULT_FAILURE,

    /* Indicates a false condition */
    USB_HOST_AUDIO_V1_0_RESULT_FALSE = 0,

    /* Indicate a true condition */
    USB_HOST_AUDIO_V1_0_RESULT_TRUE = 1,

    /* Indicates that the operation succeeded or the request was accepted and
       will be processed. */
    USB_HOST_AUDIO_V1_0_RESULT_SUCCESS = USB_HOST_RESULT_TRUE
}
USB_HOST_AUDIO_V1_0_RESULT;

// *****************************************************************************
/* USB Host Audio v1.0 Stream Result enumeration.
 
  Summary:
    USB Host Audio v1.0 stream result enumeration.

  Description:
    This enumeration lists the possible USB Host Audio v1.0 stream operation
    results. These values are returned by Audio v1.0 stream functions.

  Remarks:
    None.
*/
typedef enum 
{
    /* The transfer or request could not be scheduled because internal
     * queues are full. The request or transfer should be retried */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_REQUEST_BUSY =  USB_HOST_RESULT_REQUEST_BUSY,
            
    /* Request was aborted */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_TRANSFER_ABORTED,
            
    /* Request was stalled */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_REQUEST_STALLED,

    /* The specified Stream Handle is not valid */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID,
 
    /* The end of the device list was reached.*/
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_END_OF_DEVICE_LIST,
    
    /* The specified interface is not available */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_INTERFACE_UNKNOWN,
   
    /* A NULL parameter was passed to the function */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_PARAMETER_INVALID,

    /* The specified configuration does not exist on this device.*/
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_CONFIGURATION_UNKNOWN, 

    /* A bus operation was requested but the bus was not operated */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_BUS_NOT_ENABLED,

    /* The specified bus does not exist in the system */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_BUS_UNKNOWN,
 
    /* The specified audio stream does not exist in the system */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_UNKNOWN,

    /* An unknown failure has occurred */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE,

    /* Indicates a false condition */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_FALSE = 0,

    /* Indicate a true condition */
    USB_HOST_AUDIO_V1_0_STREAM_RESULT_TRUE = 1,

    /* Indicates that the operation succeeded or the request was accepted and
       will be processed. */
    USB_HOST_AUDIO_V1_0_STREAM_SUCCESS = USB_HOST_RESULT_TRUE
    
}USB_HOST_AUDIO_V1_0_STREAM_RESULT;

// *****************************************************************************
/* USB Host Audio v1.0 Event Handler Return Type 
 
  Summary: 
    Returns the type of the USB Audio v1.0 Host Client Driver event handler.

  Description:
    This enumeration lists the possible return values of the USB Audio v1.0 Host
    Client Driver event handler.

  Remarks:
    None.
*/      
#define USB_HOST_AUDIO_V1_0_STREAM_EVENT_RESPONSE USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE

// *****************************************************************************
/* USB Host Audio v1.0 Stream Event Handler Return Type 
 
  Summary: 
    Returns the type of the USB Host Audio v1.0 stream event handler.

  Description:
    This enumeration lists the possible return values of the USB Host Audio v1.0 
    stream event handler.

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_STREAM_EVENT_RESPONSE_NONE USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE_NONE

// *****************************************************************************
/* Audio v1.0 Class Driver Events

  Summary:
    Identifies the possible events that the Audio v1.0 Class Driver can generate.
 
 Description:
    This enumeration identifies the possible events that the Audio v1.0 Class Driver
    can generate. The application should register an event handler using the
    USB_HOST_AUDIO_V1_0_AttachEventHandlerSet function to receive Audio v1.0 Class 
    Driver events.
*/
#define USB_HOST_AUDIO_V1_0_EVENT USB_HOST_AUDIO_V1_EVENT
#define USB_HOST_AUDIO_V1_0_EVENT_ATTACH USB_HOST_AUDIO_V1_EVENT_ATTACH
#define USB_HOST_AUDIO_V1_0_EVENT_DETACH USB_HOST_AUDIO_V1_EVENT_DETACH  

// *****************************************************************************
/* Audio v1.0 Stream Events

  Summary:
    Identifies the possible events that the Audio v1.0 stream can generate.

  Description:
    This enumeration identifies the possible events that the Audio v1.0 stream
    can generate. The application should register an event handler using the
    USB_HOST_AUDIO_V1_0_StreamEventHandlerSet function to receive Audio v1.0 
    stream events.
    
    An event may have data associated with it.  Events that are generated due to
    a transfer of data between the Host and Device are accompanied by data
    structures that provide the status of the transfer termination. For example,
    the USB_HOST_AUDIO_V1_0_STREAM_EVENT_READ_COMPLETE event is accompanied by a
    pointer to a USB_HOST_AUDIO_V1_0_STREAM_EVENT_READ_COMPLETE_DATA data
    structure. The transferStatus member of this data structure indicates the
    success or failure of the transfer. A transfer may fail due to the Device not
    responding on the bus if the Device stalls any stages of the transfer or
    due to NAK time-outs. The event description provides details on the nature of
    the event and the data that is associated with the event.
*/

typedef enum
{
    /* This event occurs when a Audio v1.0 stream read operation has completed
       (i.e., when the data has been received from the connected Audio v1.0 stream).
       This event is generated after the application calls the 
       USB_HOST_AUDIO_V1_0_StreamRead function. The eventData parameter in the
       event callback function will be of a pointer to a 
       USB_HOST_AUDIO_V1_0_STREAM_EVENT_READ_COMPLETE_DATA structure. This
       contains details about the transfer handle associated with this read
       request, the amount of data read and the termination status of the read
       request. */
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_READ_COMPLETE,

    /* This event occurs when an Audio v1.0 stream write operation has completed
       (i.e., when the data has been written to the connected Audio v1.0 stream). 
       This event is generated after the application calls the 
       USB_HOST_AUDIO_V1_0_StreamWrte function. The eventData parameter in the 
       event callback function will be of a pointer to a 
       USB_HOST_AUDIO_V1_0_STREAM_EVENT_WRITE_COMPLETE_DATA structure. This
       contains details about the transfer handle associated with this write
       request, the amount of data written and the termination status of the 
       write request. */
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_WRITE_COMPLETE,

   /* This event occurs when an Audio v1.0 stream enable request has been 
      completed. This event is generated after the application calls the 
      USB_HOST_AUDIO_V1_0_StreamEnable function. The eventData parameter in the
      event callback function will be of a pointer to a 
      USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE_DATA. This contains details
      about the request handle associated with this stream enable request and the
      termination status of the Stream Enable request.*/ 
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE, 
    
    /*This event occurs when an Audio v1.0 stream disable request has been 
      completed. This event is generated after the application calls the 
      USB_HOST_AUDIO_V1_0_StreamDisable function. The eventData parameter in the
      event callback function will be of a pointer to a 
      USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE_DATA. This contains details
      about the request handle associated with this stream disable request and the
      termination status of the Stream Disable request.*/
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE,
    
    /*This event occurs when an Audio v1.0 sampling rate set request has been 
      completed. This event is generated after the application calls the 
      USB_HOST_AUDIO_V1_0_StreamSamplingRateSet function. The eventData 
      parameter in the event callback function will be of a pointer to a 
      USB_HOST_AUDIO_V1_0_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA. This 
      contains details about the request handle associated with this Sampling 
      Rate Set request and the termination status of the stream disable request.*/         
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE,       
}   
USB_HOST_AUDIO_V1_0_STREAM_EVENT;

// *****************************************************************************
/*  USB Host Audio v1.0 Class Driver Stream Direction
 
  Summary:
    USB Host Audio v1.0 Class Driver stream direction.

  Description:
    This macro defines the stream direction of the USB Host Audio v1.0 Class Driver.

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_STREAM_DIRECTION USB_HOST_AUDIO_V1_STREAM_DIRECTION
#define USB_HOST_AUDIO_V1_0_DIRECTION_OUT USB_HOST_AUDIO_V1_DIRECTION_OUT
#define USB_HOST_AUDIO_V1_0_DIRECTION_IN USB_HOST_AUDIO_V1_DIRECTION_IN 

// *****************************************************************************
/* USB Host Audio stream Info table structure

  Summary:
    This structure defines USB Host audio stream information structure. 

  Description:
    This structure is an out parameter to the functions 
    USB_HOST_AUDIO_V1_0_StreamGetFirst and USB_HOST_AUDIO_V1_0_StreamGetNext
    functions. This structure contains information about an audio stream in the 
    attached Audio Device. This structure contains the stream object, audio format, etc.      

  Remarks:
    None.
*/
#if defined (USB_HOST_AUDIO_V1_0_SAMPLING_FREQUENCIES_NUMBER) && !defined (USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER)
 #define USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER USB_HOST_AUDIO_V1_0_SAMPLING_FREQUENCIES_NUMBER
 #endif
typedef struct
{
    /* Audio Stream Object. Clients need to pass this object when opening this 
       audio stream using USB_HOST_AUDIO_V1_0_StreamOpen function. */ 
    USB_HOST_AUDIO_V1_0_STREAM_OBJ streamObj;

    /* Audio Format code for this Stream */ 
    USB_AUDIO_FORMAT_CODE format;

    /* Stream direction */ 
    USB_HOST_AUDIO_V1_0_STREAM_DIRECTION streamDirection;

    /* Number of physical channels in the audio stream */ 
    uint8_t nChannels;

    /* Number of bytes occupied by one audio sub-frame */ 
    uint8_t subFrameSize;
    
    /* Number of effectively used bits from the available bits in an audio sub-frame */ 
    uint8_t bitResolution;

    /* Indicates how the sampling frequency can be programmed:
        0: Continuous sampling frequency
        1..255: Number of discrete sampling frequencies supported by Audio stream
                   */ 
    uint8_t nSamplingRates;
    
    /* Supported sampling Frequencies */ 
    uint32_t tSamFreq[USB_HOST_AUDIO_V1_SAMPLING_FREQUENCIES_NUMBER]; 

} USB_HOST_AUDIO_V1_0_STREAM_INFO;
// *****************************************************************************
/*  USB Host Audio v1.0 Class Stream Transfer Event Data.
 
  Summary:
    USB Host Audio v1.0 class stream transfer event data.

  Description:
    This data type defines the data structure returned by the Audio V1.0 Client 
    Driver in conjunction with the following events:
    - USB_HOST_AUDIO_V1_0_STREAM_EVENT_READ_COMPLETE_DATA
    - USB_HOST_AUDIO_V1_0_STREAM_EVENT_WRITE_COMPLETE_DATA

  Remarks:
    None.
*/
#define USB_HOST_AUDIO_V1_0_STREAM_EVENT_WRITE_COMPLETE_DATA USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA
#define USB_HOST_AUDIO_V1_0_STREAM_EVENT_READ_COMPLETE_DATA USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE_DATA

// *****************************************************************************
/* USB Host Audio v1.0 Class Stream Control Event Data.
 
  Summary:
    USB Host Audio v1.0 class stream control event data.

  Description:
    This data type defines the data structure returned by the Audio V1.0 Client 
    Driver in conjunction with the following events:
    - USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE_DATA
    - USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE_DATA

  Remarks:
    None.
*/

typedef struct
{
    /* Transfer handle of this transfer */
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE  requestHandle; 

    /* Transfer termination status */
    USB_HOST_AUDIO_V1_0_RESULT requestStatus;

} 
USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE_DATA,
USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE_DATA;

// *****************************************************************************
/* USB Host Audio v1.0 Client Driver Attach Event Handler Function Pointer Type.

  Summary:
    USB Host Audio v1.0 Client Driver attach event handler function pointer type.

  Description:
    This data type defines the required function signature of the USB Host Audio v1.0
    Client Driver attach event handling callback function. The application must
    register a pointer to a Audio v1.0 Client Driver attach events handling function
    whose function signature (parameter and return value types) match the types
    specified by this function pointer in order to receive attach and detach events call
    backs from the Audio v1.0 Client Driver. The client driver will invoke this
    function with event relevant parameters. The descriptions of the event
    handler function parameters are as follows:
    - audioObj - Handle of the client to which this event is directed
    - event    - Event indicates if it is an attach or detach 
    - context  - Value identifying the context of the application that was
                 registered with the event handling function

  Remarks:
    None.
*/
typedef void (* USB_HOST_AUDIO_V1_0_ATTACH_EVENT_HANDLER)
             (
                 USB_HOST_AUDIO_V1_0_OBJ audioObj, 
                 USB_HOST_AUDIO_V1_0_EVENT event, 
                 uintptr_t context
              );
			  
// *****************************************************************************
/* USB Host Audio v1.0 Class Driver Stream Event Handler Function Pointer Type.

  Summary:
    USB Host Audio v1.0 Class Driver stream event handler function pointer type.

  Description:
    This data type defines the required function signature of the USB Host 
    Audio v1.0 Class Driver stream event handling callback function. The 
    application must register a pointer to a Audio v1.0 Class Driver stream
    events handling function whose function signature (parameter and return
    value types) match the types specified by this function pointer 
    to receive event call backs from the Audio v1.0 Class Driver. The class 
    driver will call this function with relevant event parameters. The 
    descriptions of the event handler function parameters are as follows:
    - handle           - Handle to the Audio v1.0 stream 
    - event            - Type of event generated
    - eventData        - This parameter should be type casted to an event specific
                         pointer type based on the event that has occurred. Refer 
                         to the USB_HOST_AUDIO_V1_0_STREAM_EVENT enumeration 
                         description for more information.
    - context          - Value identifying the context of the application that 
                         was registered with the event handling function

  Remarks:
    None.
*/

typedef USB_HOST_AUDIO_V1_0_STREAM_EVENT_RESPONSE (* USB_HOST_AUDIO_V1_0_STREAM_EVENT_HANDLER )
 (  USB_HOST_AUDIO_V1_0_STREAM_HANDLE handle,
    USB_HOST_AUDIO_V1_0_STREAM_EVENT event,
    void * eventData,
    uintptr_t context );
	
// *****************************************************************************
/* USB Host Audio v1.0 Class driver Control Transfer Complete Callback Function
   Pointer type

  Summary:
   USB Host Audio v1.0 Class Driver control transfer complete callback function
   pointer type.

  Description:
   This data type defines the required function signature of the USB Host 
   Audio v1.0 Class Driver control transfer complete callback function. The 
   client must provide a pointer to a control transfer complete callback 
   function whose function signature (parameter and return value types) must 
   match the types specified by this function pointer to receive 
   notification when a control transfer has completed. The pointer to the 
   callback function must be specified in USB_HOST_AUDIO_V1_0_ControlRequest 
   function. The Audio v1.0 client driver will invoke this function with event
   relevant parameters. The descriptions of the event handler function parameters
   are as follows:
   - audioObj      - Audio v1.0 client driver object associated with this event
   - requestHandle - Request handle of the control transfer request that caused
                     this event
   - result        - Completion result of the control transfer. This will be
                     USB_HOST_AUDIO_V1_0_RESULT_SUCCESS if the control transfer 
					 completed successfully, USB_HOST_AUDIO_V1_0_RESULT_FAILURE 
					 if an unknown failure occurred, or 
					 USB_HOST_AUDIO_V1_0_RESULT_REQUEST_STALLED if the request 
					 was stalled.
   size            - Size of the data stage that was transferred
   context         - Value identifying the context of the application that was
                     provided when the USB_HOST_AUDIO_V1_0_ControlRequest function 
					 was called.

  Remarks:
    None.
*/
typedef void (* USB_HOST_AUDIO_V1_0_CONTROL_CALLBACK)
(
    USB_HOST_AUDIO_V1_0_OBJ audioObj, 
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE requestHandle,
    USB_HOST_AUDIO_V1_0_RESULT result,
    size_t size,
    uintptr_t context
); 

// ****************************************************************************
// ****************************************************************************
// Section: Client Access Functions
// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_AttachEventHandlerSet
    (
        USB_HOST_AUDIO_V1_0_ATTACH_EVENT_HANDLER eventHandler,
        uintptr_t context
    );

  Summary:
    Sets an attach/detach event handler.

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
                   handler

  Returns:
    - USB_HOST_AUDIO_V1_0_RESULT_SUCCESS - if the attach event handler was registered
    successfully 
    - USB_HOST_AUDIO_V1_0_RESULT_FAILURE - if the number of registered event 
    handlers has exceeded USB_HOST_AUDIO_V1_0_ATTACH_LISTENERS_NUMBER

  Remarks:
    This function should be called before the USB_HOST_BusEnable function is called.

 */
 #define USB_HOST_AUDIO_V1_0_AttachEventHandlerSet USB_HOST_AUDIO_V1_AttachEventHandlerSet
 
 // ****************************************************************************
/* Function:
    USB_HOST_DEVICE_OBJ_HANDLE USB_HOST_AUDIO_V1_0_DeviceObjHandleGet
    (
        USB_HOST_AUDIO_V1_0_OBJ audioDeviceObj
    );
           
  Summary:
    Returns the device object handle for this Audio v1.0 Device.

  Description:
    This function returns the device object handle for this Audio v1.0 Device. This
    returned device object handle can be used by the application to perform
    device-level operations such as getting the string descriptors.

  Precondition:
    None.

  Parameters:
    audioDeviceObj - Audio V1.0 device object handle returned in the
    USB_HOST_AUDIO_V1_0_ATTACH_EVENT_HANDLER function.

  Returns:
    This function will return a valid device object handle if the device is still 
	connected to the system. Otherwise, USB_HOST_DEVICE_OBJ_HANDLE_INVALID is 
	returned.

  Remarks:
    None.                                                                   
*/
#define USB_HOST_AUDIO_V1_0_DeviceObjHandleGet USB_HOST_AUDIO_V1_DeviceObjHandleGet

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
    This function schedules an Audio v1.0 control transfer. The audioObj parameter 
	is an object of the Audio v1.0 Class Driver to which the audio control transfer 
	is to be scheduled. The setupPacket parameter points to the SETUP command to be 
	sent in the setup state of the control transfer. The size and the direction of 
	the data stage is indicated by the SETUP packet. For control transfers where
    there is no data stage, data is ignored and can be NULL. In all other instances,
    data should point to the data to data be transferred in the data stage of 
    the control transfer. 
    
    If the transfer was scheduled successfully, requestHandle will contain a
    transfer handle that uniquely identifies this transfer. If the transfer
    could not be scheduled successfully, requestHandle will contain
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.

    When the control transfer completes, the Audio v1.0 Client Driver will call
    the specified callback function. The context parameter specified here will 
    be returned in the callback.

  Precondition:
    The Audio v1.0 Device should be attached. 

  Parameters:
    audioObj      - Audio v1.0 client driver object 
    requestHandle - Output parameter that will contain the handle to this
                    transfer
    setupPacket   - Pointer to the SETUP packet to sent to the device in the SETUP
                    stage of the control transfer
    data          - For control transfer with a data stage, this should point to 
	                data to be sent to the device (for a control write transfer) 
					or point to the buffer that will receive data from the device 
					(for a control read transfer). For control transfers that do not 
					require a data stage, this parameter is ignored and can be NULL.
    callback      - Pointer to the callback function that will be called when the 
	                control transfer completes. If the callback function is NULL, 
					there will be no notification of when the control transfer will 
					complete.
    context       - User-defined context that is returned with the callback function

  Returns:
    - USB_HOST_AUDIO_V1_0_RESULT_SUCCESS - The transfer was scheduled successfully.
      requestHandle will contain a valid transfer handle.
    - USB_HOST_AUDIO_V1_0_RESULT_FAILURE - An unknown failure occurred. requestHandle will
      contain USB_HOST_AUDIO_V1_0_REQUEST_HANDLE_INVALID.
    - USB_HOST_AUDIO_V1_0_RESULT_PARAMETER_INVALID - The data pointer or requestHandle pointer
      is NULL

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_ControlRequest
(
    USB_HOST_AUDIO_V1_0_OBJ OBJ,
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * transferHandle,
    USB_SETUP_PACKET *setupPacket,
    void * data,
    USB_HOST_AUDIO_V1_0_CONTROL_CALLBACK callback, 
    uintptr_t context 
);

// *****************************************************************************
/* Function:
    uint8_t USB_HOST_AUDIO_V1_0_NumberOfStreamGroupsGet
    (
        USB_HOST_AUDIO_V1_0_OBJ audioObj
    );

  Summary:
    Gets the number of stream groups present in the attached  Audio v1.0 Device. 

  Description:
    This function will get number of stream groups present in the attached 
    Audio v1.0 Device. The audio stream within an audio stream cannot be enabled
    at the same time. 
    
  Precondition:
    The Audio v1.0 Device should have been attached. 

  Parameters:
    audioObj - Audio v1.0 Client Driver object 

  Returns:
    A returned uint8_t indicates the number of audio stream groups present in the 
    attached Audio v1.0 Device. 

  Remarks:
    None.

 */
uint8_t USB_HOST_AUDIO_V1_0_NumberOfStreamGroupsGet
(
    USB_HOST_AUDIO_V1_0_OBJ audioObj
);
        
// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_StreamGetFirst 
    (
        USB_HOST_AUDIO_V1_0_OBJ audioDeviceObj, 
        uint8_t streamGroupIndex, 
        USB_HOST_AUDIO_V1_0_STREAM_INFO * streamInfo
    );

  Summary:
    Returns information about first audio stream in the specified audio stream 
    group. 

  Description:
    This function returns information about the first audio stream in the specified
    audio stream group. The stream group index is parameter to this function 
    and it can be any value starting from zero to the number of stream groups minus
    one. Number of stream groups can be obtained by using the  
    USB_HOST_AUDIO_V1_0_NumberOfStreamGroupsGet function. 
    
    The streamInfo object is an out parameter to this function. 
    
  Precondition:
    The Audio v1.0 Device should have been attached to the Host. 

  Parameters:
    audioDeviceObj   - Audio v1.0 Client Driver object
    streamGroupIndex - Stream group index
    streamInfo       -  Pointer to the streamInfo object 

  Returns:
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_0_RESULT_OBJ_INVALID - The specified Audio v1.0 client
      driver object does not exist
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_StreamGetFirst 
(
    USB_HOST_AUDIO_V1_0_OBJ audioDeviceObj, 
    uint8_t streamGroupIndex, 
    USB_HOST_AUDIO_V1_0_STREAM_INFO * streamInfo
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_StreamGetNext 
    (
        USB_HOST_AUDIO_V1_0_STREAM_OBJ audioStreamObj,
        USB_HOST_AUDIO_V1_0_STREAM_INFO * streamInfo
    );

  Summary:
    Returns information about the next audio stream in the specified audio stream 
    group.

  Description:
    This function returns information about next audio stream in the specified
    Audio stream group. The USB_HOST_AUDIO_V1_0_StreamGetFirst function should 
    have been called at least once on the same audio stream group before calling 
    this function. Then, calling this function repeatedly on the stream group 
    will return information about the next audio stream in the stream group. 
    When there are no more audio streams to report, the function returns 
    USB_HOST_AUDIO_V1_0_RESULT_END_OF_STREAM_LIST. 
    
    Calling the USB_HOST_AUDIO_V1_0_StreamGetFirst function on the stream group
    index after the USB_HOST_AUDIO_V1_0_StreamGetNext function has been called 
    will cause the Audio v1.0 Client Driver to reset the audio stream group to point
    to the first stream in the stream group.
    
  Precondition:
    The USB_HOST_AUDIO_V1_0_StreamGetFirst function must have been called
    before calling this function.

  Parameters:
    audioStreamObj   -  Present audio stream object      
    streamInfo       -  Pointer to the streamInfo object 

  Returns:
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_0_RESULT_OBJ_INVALID - The specified Audio v1.0 client
      driver object does not exist
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred
    - USB_HOST_AUDIO_V1_0_RESULT_END_OF_STREAM_LIST - There are no more audio 
      streams in the stream group

  Remarks:
    None.
*/ 
USB_HOST_AUDIO_V1_0_RESULT USB_HOST_AUDIO_V1_0_StreamGetNext 
(
    USB_HOST_AUDIO_V1_0_STREAM_OBJ audioStreamObj,
    USB_HOST_AUDIO_V1_0_STREAM_INFO * streamInfo
); 

// ****************************************************************************
// ****************************************************************************
// Section: Audio Stream Access Functions
// ****************************************************************************
// ****************************************************************************
// ****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE USB_HOST_AUDIO_V1_0_StreamOpen
    ( 
        USB_HOST_AUDIO_V1_0_STREAM_OBJ audioStreamObj  
    );
           
  Summary:
    Opens the specified audio stream. 

  Description:
    This function will open the specified audio stream. Once opened, the audio
    stream can be accessed via the handle which this function returns. The
    audioStreamObj parameter is the value returned in the
    USB_HOST_AUDIO_V1_0_StreamGetFirst or USB_HOST_AUDIO_V1_0_StreamGetNext
    functions.

  Precondition:
    The audio stream object should be valid.

  Parameters:
    audioStreamObj - Audio stream object

  Return:
    This function will return a valid handle if the audio stream could be opened 
	successfully; otherwise, it will return USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID. 
	The function will return a valid handle if the stream is ready to be opened.

  Remarks:
    None.                                                                   
*/
#define USB_HOST_AUDIO_V1_0_StreamOpen USB_HOST_AUDIO_V1_StreamOpen

// ****************************************************************************
/* Function:
    void USB_HOST_AUDIO_V1_0_StreamClose
    ( 
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE audioSteamHandle
    );
           
  Summary:
    Closes the audio stream. 

  Description:
    This function will close the open audio stream. This closes the association
    between the application entity that opened the audio stream and the audio 
    stream. The audio stream handle becomes invalid.

  Precondition:
    None.

  Parameters:
    audioSteamHandle - handle to the audio stream obtained from the
    USB_HOST_AUDIO_V1_0_StreamOpen function.

  Returns:
    None.

  Remarks:
    The device handle becomes invalid after calling this function.                                                                   
*/
#define USB_HOST_AUDIO_V1_0_StreamClose USB_HOST_AUDIO_V1_StreamClose

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamEventHandlerSet
    (
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE handle,
        USB_HOST_AUDIO_V1_0_STREAM_EVENT_HANDLER appAudioHandler,
        uintptr_t context
    ); 

  Summary:
    Registers an event handler with the Audio v1.0 Client Driver stream.

  Description:
    This function registers a client specific Audio v1.0 stream event handler.
    The Audio v1.0 Host Client Driver will call appAudioHandler function 
    specified as 2nd argument with relevant event and associate event data, in
    response to audio stream data transfers that have been scheduled by the 
    client.
    
  Precondition:
    None.

  Parameters:
    handle       - A handle to the Audio v1.0 stream
    eventHandler - A pointer to event handler function. If NULL, events
                   will not be generated.
    context      - The application specific context that is returned in the event handler

  Returns:
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/

USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamEventHandlerSet
(
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE handle,
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_HANDLER appAudioHandler,
    uintptr_t context
); 

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamEnable
    (
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle, 
        USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * requestHandle
    ); 

  Summary:
    Schedules an audio stream enable request for the specified audio stream. 

  Description:
    This function schedules an audio stream enable request for the specified 
    audio stream. An audio stream must be enable before scheduling any data 
    transfer with the stream. A USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE 
	event is generated when this request is completed. 
	USB_HOST_AUDIO_V1_0_STREAM_EVENT_ENABLE_COMPLETE_DATA returns the status and 
	request handle of the request. 
       
  Precondition:
    The audio stream should have been opened. Only one audio stream from an audio
    stream group can be enabled at a time. 

  Parameters:
    streamHandle  - Handle to the audio v1.0 stream
    requestHandle - Handle to the stream enable request 

  Returns:
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamEnable
(
    
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle, 
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * requestHandle
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamDisable
    (
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle, 
        USB_HOST_AUDIO_V1_0_REQUEST_HANDLE * requestHandle
    ); 

  Summary:
    Schedules an audio stream disable request for the specified audio stream. 

  Description:
    This function schedules an audio stream disable request for the specified 
    audio stream. A USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE event is 
	generated when this request is completed. 
	USB_HOST_AUDIO_V1_0_STREAM_EVENT_DISABLE_COMPLETE_DATA 
    returns the status and request handle of the request. 
       
  Precondition:
    The audio stream should have been opened. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 stream
    requestHandle - Handle to the stream disable request 

  Returns:
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamDisable
(
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE *requestHandle
);

// *****************************************************************************
/* Function:
    USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamSamplingRateSet
    (
        USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
        USB_HOST_AUDIO_V1_0_REQUEST_HANDLE requestHandle,
        uint32_t* samplingRate
    ); 

  Summary:
    Schedules an audio stream set sampling rate request for the specified 
    audio stream. 

  Description:
    This function schedules an audio stream set sampling rate request for the
    specified audio stream. A USB_HOST_AUDIO_V1_0_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE 
	event is generated when this request is completed. 
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA returns 
    the status and request handle of the request. 
       
  Precondition:
    The audio stream should have been opened. 

  Parameters:
    streamHandle  - Handle to the Audio v1.0 stream
    requestHandle - Handle to the stream set sampling rate request
    samplingRate  - Pointer to the sampling rate

  Returns:
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
      stream does not exist
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamSamplingRateSet
(
    USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_0_REQUEST_HANDLE *requestHandle,
    uint32_t *samplingRate
);

// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamWrite 
   (
       USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
       USB_HOST_AUDIO_V1_0_STREAM_TRANSFER_HANDLE * transferHandle,
       void * source, 
       size_t length
   ); 

  Summary:
    Schedules an audio stream write request for the specified audio stream. 

  Description:
    This function schedules an audio stream write request for the specified 
    audio stream. A USB_HOST_AUDIO_V1_0_STREAM_EVENT_WRITE_COMPLETE event is 
	generated when this request is completed. 
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_WRITE_COMPLETE_DATA returns 
    the status and request handle of the request.
    
  Precondition:
    The audio stream should have been opened and enabled. The direction of the 
    audio stream should be USB_HOST_AUDIO_V1_0_DIRECTION_OUT. 

  Parameters:
    streamHandle    - Handle to the Audio v1.0 stream
    transferHandle  - Handle to the stream write transfer request 
    source          - Pointer to the buffer containing data to be written to the 
                      device 
    length          - Amount of data to write (in bytes)

  Returns:
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_0_StreamWrite 
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
    void * source, 
    size_t length
);

// *****************************************************************************
/* Function:
   USB_HOST_AUDIO_V1_0_STREAM_RESULT USB_HOST_AUDIO_V1_0_StreamRead 
   (
       USB_HOST_AUDIO_V1_0_STREAM_HANDLE streamHandle,
       USB_HOST_AUDIO_V1_0_STREAM_TRANSFER_HANDLE * transferHandle,
       void * source, 
       size_t length
   ); 

  Summary:
    Schedules an audio stream read request for the specified audio stream. 

  Description:
    This function schedules an audio stream read request for the specified 
    audio stream. A USB_HOST_AUDIO_V1_0_STREAM_EVENT_READ_COMPLETE event is 
	generated when this request is completed. 
    USB_HOST_AUDIO_V1_0_STREAM_EVENT_READ_COMPLETE_DATA returns 
    the status and request handle of the request.
    
  Precondition:
    The audio stream should have been opened and enabled. The direction of the 
    audio stream should be USB_HOST_AUDIO_V1_0_DIRECTION_IN. 

  Parameters:
    streamHandle    - Handle to the Audio v1.0 stream
    transferHandle  - Handle to the stream read transfer request 
    source          - Pointer to the buffer containing data to be read from the 
                      device
    length          - Amount of data to read (in bytes)

  Returns:
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_SUCCESS - The operation was successful
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_HANDLE_INVALID - The specified audio 
    stream does not exist
    - USB_HOST_AUDIO_V1_0_STREAM_RESULT_FAILURE - An unknown failure occurred

  Remarks:
    None.
*/
USB_HOST_AUDIO_V1_RESULT USB_HOST_AUDIO_V1_0_StreamRead 
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE * transferHandle,
    void * source, 
    size_t length
);

// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued)
// *****************************************************************************
// *****************************************************************************

/*  The following included file maps the interface definitions above to appropriate
    static implementations depending on the build mode.
*/
#include "usb/src/usb_host_audio_v1_mapping.h"

#endif 