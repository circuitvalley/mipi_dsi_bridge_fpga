/*******************************************************************************
  USB Host CDC Client Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_cdc.h

  Summary:
    USB Host CDC Client Driver Interface Header

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface to the USB Host CDC
    Client Driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

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
#ifndef _USB_HOST_CDC_H_
#define _USB_HOST_CDC_H_

//DOM-IGNORE-END

// ****************************************************************************
// ****************************************************************************
// Section: Included Files
// ****************************************************************************
// ****************************************************************************

#include "usb/usb_host.h"
#include "usb/usb_host_client_driver.h"
#include "usb/usb_cdc.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// ****************************************************************************
// ****************************************************************************
// Section: Data Types and Constants
// ****************************************************************************
// ****************************************************************************

// *****************************************************************************
/* USB Host CDC Client Driver Handle 
 
  Summary: 
    Defines the type of the CDC Host Client Driver Handle 

  Description:
    This type defines the type of the handle returned by USB_HOST_CDC_Open()
    function. This application uses this handle to specify the instance of the
    CDC client driver being accessed while calling a CDC Client driver function.  

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_CDC_HANDLE;

// *****************************************************************************
/* USB Host CDC Client Driver Invalid Handle 
 
  Summary: 
    Defines an Invalid CDC Client Driver Handle.

  Description:
    This type defines an Invalid CDC Client Driver Handle. The
    USB_HOST_CDC_Open() function returns an invalid handle when it fails to open
    the specified CDC device instance.

  Remarks:
    None.
*/

#define USB_HOST_CDC_HANDLE_INVALID ((USB_HOST_CDC_HANDLE)(-1))

// *****************************************************************************
/* USB HOST CDC Client Driver Interface
 
  Summary:
    USB HOST CDC Client Driver Interface

  Description:
    This macro should be used by the application in TPL table while adding
    support for the USB CDC Host Client Driver.

  Remarks:
    None.
*/

/*DOM-IGNORE-BEGIN*/extern USB_HOST_CLIENT_DRIVER gUSBHostCDCClientDriver; /*DOM-IGNORE-END*/
#define USB_HOST_CDC_INTERFACE  /*DOM-IGNORE-BEGIN*/&gUSBHostCDCClientDriver /*DOM-IGNORE-END*/

// *****************************************************************************
/* USB Host CDC Object 
 
  Summary: 
    Defines the type of the CDC Host Client Object.

  Description:
    This type defines the type of the CDC Host Client Object. This type
    is returned by the Attach Event Handler and is used by the application to
    open the attached CDC Device.  

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_CDC_OBJ;

// *****************************************************************************
/*  USB Host CDC Client Driver Transfer Handle

  Summary:
     USB Host CDC Client Driver Transfer Handle

  Description:
    This is returned by the CDC Client driver data transfer routines and should
    be used by the application to track the transfer especially in cases where
    transfers are queued.

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_CDC_TRANSFER_HANDLE;

// *****************************************************************************
/*  USB Host CDC Client Driver Invalid Transfer Handle Definition
 
  Summary:
     USB Host CDC Client Driver Invalid Transfer Handle Definition.

  Description:
    This definition defines a  USB Host CDC Client Driver Invalid Transfer
    Handle. A Invalid Transfer Handle is returned by the CDC Client Driver data
    transfer routines when the request was not successful. 

  Remarks:
    None.
*/

#define USB_HOST_CDC_TRANSFER_HANDLE_INVALID ((USB_HOST_CDC_TRANSFER_HANDLE)(-1))

// *****************************************************************************
/*  USB Host CDC Client Driver Request Handle

  Summary:
     USB Host CDC Client Driver Request Handle

  Description:
    This is returned by the CDC Client driver command routines and should be
    used by the application to track the command especially in cases where
    transfers are queued.

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_CDC_REQUEST_HANDLE;

// *****************************************************************************
/*  USB Host CDC Client Driver Invalid Request Handle

  Summary:
     USB Host CDC Client Driver Invalid Request Handle

  Description:
    This is returned by the CDC Client driver command routines when the request
    could not be scheduled.

  Remarks:
    None.
*/

#define USB_HOST_CDC_REQUEST_HANDLE_INVALID ((USB_HOST_CDC_REQUEST_HANDLE)(-1))

/*DOM-IGNORE-BEGIN*/#define USB_HOST_CDC_RESULT_MIN -100 /*DOM-IGNORE-END*/

// *****************************************************************************
/* USB Host CDC Client Driver Result.
 
  Summary:
    USB Host CDC Client Driver Result enumeration.

  Description:
    This enumeration lists the possible results the CDC client driver uses. Only
    some results are applicable to some functions and events. Refer to the event
    and function documentation for more details.

  Remarks:
    None.
*/

typedef enum
{
    /* An unknown failure has occurred */
    USB_HOST_CDC_RESULT_FAILURE /*DOM-IGNORE-BEGIN*/ = USB_HOST_CDC_RESULT_MIN /*DOM-IGNORE-END*/,

    /* The transfer or request could not be scheduled because internal
     * queues are full. The request or transfer should be retried */
    USB_HOST_CDC_RESULT_BUSY,

    /* The request was stalled */
    USB_HOST_CDC_RESULT_REQUEST_STALLED,

    /* A required parameter was invalid */
    USB_HOST_CDC_RESULT_INVALID_PARAMETER,

    /* The associated device does not exist in the system. */
    USB_HOST_CDC_RESULT_DEVICE_UNKNOWN,

    /* The transfer or requested was aborted */
    USB_HOST_CDC_RESULT_ABORTED,

    /* The specified handle is not valid */
    USB_HOST_CDC_RESULT_HANDLE_INVALID,

    /* The operation was successful */
    USB_HOST_CDC_RESULT_SUCCESS /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/

} USB_HOST_CDC_RESULT;

// *****************************************************************************
/*  USB Host CDC Client Driver Command Event Data.
 
  Summary:
     USB Host CDC Client Driver Command Event Data.

  Description:
    This data type defines the data structure returned by the driver along with
    the following events:
    USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA,
    USB_HOST_CDC_EVENT_ACM_SEND_BREAK_COMPLETE_DATA,
    USB_HOST_CDC_EVENT_ACM_GET_LINE_CODING_COMPLETE_DATA,
    USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA,

  Remarks:
    None.
*/

typedef struct
{
    /* Request handle of this request */
    USB_HOST_CDC_REQUEST_HANDLE requestHandle;

    /* Termination status */
    USB_HOST_CDC_RESULT result;

    /* Size of the data transferred in the request */
    size_t length;
}
USB_HOST_CDC_EVENT_ACM_GET_LINE_CODING_COMPLETE_DATA,
USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA,
USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA,
USB_HOST_CDC_EVENT_ACM_SEND_BREAK_COMPLETE_DATA;

// *****************************************************************************
/* USB Host CDC Client Driver Event Data.
 
  Summary:
     USB Host CDC Client Driver Event Data.

  Description:
    This data type defines the data structure returned by the driver along with
    the following events:
    USB_HOST_CDC_EVENT_READ_COMPLETE_DATA,
    USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA,

  Remarks:
    None.
*/

typedef struct
{
    /* Transfer handle of this transfer */
    USB_HOST_CDC_TRANSFER_HANDLE transferHandle;

    /* Termination transfer status */
    USB_HOST_CDC_RESULT result;

    /* Size of the data transferred in the request */
    size_t length;
}
USB_HOST_CDC_EVENT_SERIAL_STATE_NOTIFICATION_RECEIVED_DATA,
USB_HOST_CDC_EVENT_READ_COMPLETE_DATA,
USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA;

// *****************************************************************************
/* CDC Class Driver Events

  Summary:
    Identifies the possible events that the CDC Class Driver can generate.

  Description:
    This enumeration identifies the possible events that the CDC Class Driver
    can generate. The application should register an event handler using the
    USB_HOST_CDC_EventHandlerSet function to receive CDC Class Driver events.
    
    An event may have data associated with it.  Events that are generated due to
    a transfer of data between the host and device are accompanied by data
    structures that provide the status of the transfer termination. For example,
    the USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE event is accompanied by a
    pointer to a USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA data
    structure. The transferStatus member of this data structure indicates the
    success or failure of the transfer. A transfer may fail due to device not
    responding on the bus, if the device stalls any stages of the transfer or
    due to NAK timeouts. The event description provides details on the nature of
    the event and the data that is associated with the event.

  Remarks:
    None.
*/

typedef enum
{
    /* This event occurs when a CDC Client Driver Read operation has completed
       i.e when the data has been received from the connected CDC device. This
       event is generated after the application calls the USB_HOST_CDC_Read
       function. The eventData parameter in the event call back function will be
       of a pointer to a USB_HOST_CDC_EVENT_READ_COMPLETE_DATA structure. This
       contains details about the transfer handle associated with this read
       request, the amount of data read and the termination status of the read
       request. */

    USB_HOST_CDC_EVENT_READ_COMPLETE,

    /* This event occurs when a CDC Client Driver Write operation has completed
       i.e when the data has been written to the connected CDC device. This
       event is generated after the application calls the USB_HOST_CDC_Write
       function. The eventData parameter in the event call back function will be
       a pointer to a USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA structure. This
       contains details about the transfer handle associated with this write
       request, the amount of data written and the termination status of the write
       request. */

    USB_HOST_CDC_EVENT_WRITE_COMPLETE,

    /* This event occurs when a CDC Client Driver Send Break 
       request has completed. This event is generated after the application
       calls the USB_HOST_CDC_ACM_BreakSend function and the device has either
       acknowledged or stalled the request. The eventData parameter in the event
       call back function will be of a pointer to a
       USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA structure. This
       contains details about the transfer handle associated with this request,
       the amount of data sent and the termination status of the set request. */

    USB_HOST_CDC_EVENT_ACM_SEND_BREAK_COMPLETE,

    /* This event occurs when a CDC Client Driver Set Control Line State 
       request has completed. This event is generated after the application
       calls the USB_HOST_CDC_ACM_ControlLineStateSet function and the device has
       either acknowledged or stalled the request. The eventData parameter in
       the event call back function will be of a pointer to a
       USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA structure. This
       contains details about the transfer handle associated with this request,
       the amount of data sent and the termination status of the set request. */

    USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE,

    /* This event occurs when a CDC Client Driver Set Line Coding
       request has completed. This event is generated after the application
       calls the USB_HOST_CDC_ACM_LineCodingSet function and the device either
       acknowledged or stalled the request. The eventData parameter in the event
       call back function will be of a pointer to a
       USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA structure. This contains
       details about the transfer handle associated with this request, the
       amount of data sent and the termination status of the set request. */

    USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE,

    /* This event occurs when a CDC Client Driver Get Line Coding
       request has completed.  This event is generated after the application
       calls the USB_HOST_CDC_ACM_LineCodingGet function and the device sends the
       line coding to the host. The eventData parameter in the event call back
       function will be of a pointer to a
       USB_HOST_CDC_EVENT_ACM_GET_LINE_CODING_COMPLETE_DATA structure.  This
       contains details about the transfer handle associated with this request,
       the amount of data received and the termination status of the get
       request. */

    USB_HOST_CDC_EVENT_ACM_GET_LINE_CODING_COMPLETE,

    /* This event occurs when a CDC Client Driver Serial State Notification
       Get operation has completed.  This event is generated after the
       application calls the USB_HOST_CDC_SerialStateNotificationGet and the
       device sends a serial state notification to the host. The eventData
       parameter in the event call back function will be of a pointer to a
       USB_HOST_CDC_EVENT_SERIAL_STATE_NOTIFICATION_RECEIVED_DATA structure.
       This contains details about the transfer handle associated with this
       request, the amount of data received and the termination status of the
       get request. */

    USB_HOST_CDC_EVENT_SERIAL_STATE_NOTIFICATION_RECEIVED,

    /* This event occurs when the device that this client was connected to has
     * been detached. The client should close the CDC instance. There is no
     * event data associated with this event */
    USB_HOST_CDC_EVENT_DEVICE_DETACHED

} USB_HOST_CDC_EVENT;

// *****************************************************************************
/* USB Host CDC Client Driver Attach Event Handler Function Pointer Type.

  Summary:
    USB Host CDC Client Driver Attach Event Handler Function Pointer Type.

  Description:
    This data type defines the required function signature of the USB Host CDC
    Client Driver attach event handling callback function. The application must
    register a pointer to a CDC Client Driver attach events handling function
    whose function signature (parameter and return value types) match the types
    specified by this function pointer in order to receive attach event call
    backs from the CDC Client Driver. The client driver will invoke this
    function with event relevant parameters. The description of the event
    handler function parameters is given here.

    cdcObjHandle - Handle of the client to which this event is directed.
    
    context - Value identifying the context of the application that was
    registered along with  the event handling function.

  Remarks:
    None.
*/

typedef void (* USB_HOST_CDC_ATTACH_EVENT_HANDLER)
(
    USB_HOST_CDC_OBJ cdcObjHandle, 
    uintptr_t context
);

// *****************************************************************************
/* USB Host CDC Event Handler Return Type 
 
  Summary: 
    Return type of the USB CDC Host Client Driver Event Handler.

  Description:
    This enumeration list the possible return values of the USB CDC Host Client
    Driver Event Handler.

  Remarks:
    None.
*/

typedef enum
{
    /* This means no response is required */
    USB_HOST_CDC_EVENT_RESPONE_NONE   /*DOM-IGNORE-BEGIN*/= 0 /*DOM-IGNORE-END*/

} USB_HOST_CDC_EVENT_RESPONSE;

// *****************************************************************************
/* USB Host CDC Client Driver Event Handler Function Pointer Type.

  Summary:
    USB Host CDC Client Driver Event Handler Function Pointer Type.

  Description:
    This data type defines the required function signature of the USB Host CDC
    Client Driver event handling callback function. The application must
    register a pointer to a CDC Client Driver events handling function whose
    function signature (parameter and return value types) match the types
    specified by this function pointer in order to receive event call backs from
    the CDC Client Driver. The class driver will invoke this function with
    event relevant parameters. The description of the event handler function
    parameters is given here.

    handle - Handle of the client to which this event is directed.
    
    event - Type of event generated.
    
    eventData - This parameter should be type casted to a event specific pointer
    type based on the event that has occurred. Refer to the USB_HOST_CDC_EVENT
    enumeration description for more details.
    
    context - Value identifying the context of the application that was
    registered along with  the event handling function.

  Remarks:
    None.
*/

typedef USB_HOST_CDC_EVENT_RESPONSE (* USB_HOST_CDC_EVENT_HANDLER)
(
    USB_HOST_CDC_HANDLE cdcHandle,
    USB_HOST_CDC_EVENT event,
    void * eventData,
    uintptr_t context
);

// ****************************************************************************
// ****************************************************************************
// Section: Client Access Functions
// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_AttachEventHandlerSet
    (
        USB_HOST_CDC_ATTACH_EVENT_HANDLER eventHandler,
        uintptr_t context
    );
           
  Summary:
    This function will set an attach event handler.

  Description:
    This function will set an attach event handler. The attach event handler
    will be called when a CDC device has been attached. The context will be
    returned in the event handler. This function should be called before the bus
    has been enabled.

  Precondition:
    None.

  Input:
    eventHandler - pointer to the attach event handler
    
    context - an application defined context that will be returned in the event
    handler.

  Return:
    USB_HOST_CDC_RESULT_SUCCESS - if the attach event handler was registered
    successfully. 

    USB_HOST_CDC_RESULT_FAILURE - if the number of registered event handlers has
    exceeded USB_HOST_CDC_ATTACH_LISTENERS_NUMBER.

  Example:
    <code>
    </code>

  Remarks:
    Function should be called before USB_HOST_BusEnable() function is called.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_AttachEventHandlerSet
(
    USB_HOST_CDC_ATTACH_EVENT_HANDLER eventHandler,
    uintptr_t context
);

// ****************************************************************************
/* Function:
    USB_HOST_DEVICE_OBJ_HANDLE USB_HOST_CDC_DeviceObjHandleGet
    (
        USB_HOST_CDC_OBJ cdcDeviceObj
    );
           
  Summary:
    This function returns the Device Object Handle for this CDC device.

  Description:
    This function returns the Device Object Handle for this CDC device. This
    returned Device Object Handle can be used by the application to perform
    device level operations such as getting the string descriptors.

  Precondition:
    None.

  Input:
    cdcDeviceObj - CDC device object handle returned in the
    USB_HOST_CDC_ATTACH_EVENT_HANDLER function.

  Return:
    Will return a valid device object handle if the device is still connected to
    the system. Will return an USB_HOST_DEVICE_OBJ_HANDLE_INVALID otherwise.
    
  Example:
    <code>
    </code>

  Remarks:
    None.                                                                   
*/

USB_HOST_DEVICE_OBJ_HANDLE USB_HOST_CDC_DeviceObjHandleGet
(
    USB_HOST_CDC_OBJ cdcDeviceObj
);

// ****************************************************************************
/* Function:
    USB_HOST_CDC_HANDLE USB_HOST_CDC_Open
    (
        USB_HOST_CDC_OBJ cdcDeviceObj
    );
           
  Summary:
    This function opens the specified CDC device.

  Description:
    This function will open the specified CDC device. Once opened, the CDC
    device can be accessed via the handle which this function returns. The
    cdcDeviceObj parameter is the value returned in the
    USB_HOST_CDC_ATTACH_EVENT_HANDLER event handling function.

  Precondition:
    The client handle should be valid.

  Input:
    cdcDeviceObj - CDC device object handle returned in the
    USB_HOST_CDC_ATTACH_EVENT_HANDLER function.

  Return:
    Will return a valid handle if the device could be opened successfully, else
    will return USB_HOST_CDC_HANDLE_INVALID. The function will return a valid
    handle if the device is ready to be opened.
    
  Example:
    <code>
    </code>

  Remarks:
    None.                                                                   
*/

USB_HOST_CDC_HANDLE USB_HOST_CDC_Open
(
    USB_HOST_CDC_OBJ cdcDeviceObj
);

// ****************************************************************************
/* Function:
    void USB_HOST_CDC_Close
    (
        USB_HOST_CDC_HANDLE cdcDeviceHandle
    );
           
  Summary:
    This function closes the CDC device.

  Description:
    This function will close the open CDC device. This closes the association
    between the application entity that opened the device and device. The driver
    handle becomes invalid.

  Precondition:
    None.

  Input:
    cdcDeviceHandle - handle to the CDC device obtained from the
    USB_HOST_CDC_Open() function.

  Return:
    None.
    
  Example:
    <code>
    </code>

  Remarks:
    The device handle becomes invalid after calling this function.                                                                   
*/

void USB_HOST_CDC_Close
(
    USB_HOST_CDC_HANDLE cdcDeviceHandle
);

// *****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_EventHandlerSet
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_EVENT_HANDLER eventHandler,
        uintptr_t context
    );

  Summary:
    Registers an event handler with the CDC Host Client Driver.

  Description:
    This function registers a client specific CDC Host Client Driver event
    handler. The CDC Host Client Driver will call this function with relevant
    event and associated event data, in response to command requests and data
    transfers that have been scheduled by the client.
    
  Precondition:
    None.

  Parameters:
    handle  - handle to the CDC Host Client Driver.

    eventHandler - A pointer to event handler function. If NULL, then events
    will not be generated.
    
    context - Application specific context that is returned in the event handler.

  Returns:
    USB_HOST_CDC_RESULT_SUCCESS - The operation was successful
    USB_HOST_CDC_RESULT_HANDLE_INVALID - The specified instance does 
    not exist.
    USB_HOST_CDC_RESULT_FAILURE - An unknown failure occurred.
    
  Example:
    <code>
    </code>

  Remarks:
    None.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_EventHandlerSet
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_EVENT_HANDLER eventHandler,
    uintptr_t context
);


// ****************************************************************************
// ****************************************************************************
// Section: Data Transfer Functions
// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_Write
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
        void * data,
        size_t size
    );
           
  Summary:
    This function will write data to the attached device.

  Description:
    This function will write data to the attached CDC device. The function will
    write size amount of bytes. If the request was accepted, transferHandle will
    contain a valid transfer handle, else it will contain
    USB_HOST_CDC_TRANSFER_HANDLE_INVALID. The completion of the request will be
    indicated by the USB_HOST_CDC_EVENT_WRITE_COMPLETE event. The transfer
    handle will be returned in the event. 

  Precondition:
    The client handle should be valid.

  Input:
    handle - handle to the CDC device instance to which the request should be
    sent.
    
    transferHandle - Pointer to USB_HOST_CDC_TRANSFER_HANDLE type of a variable.
    This will contain a valid transfer handle if the request was successful.

    data - pointer to the buffer containing the data to be written. The
    contents of the buffer should not be changed till the
    USB_HOST_CDC_EVENT_WRITE_COMPLETE event has occurred.

    size - Number of bytes to write.

  Return:
    USB_HOST_CDC_RESULT_SUCCESS - The operation was successful.

    USB_HOST_CDC_RESULT_DEVICE_UNKNOWN - The device that this request was
    targeted to does not exist in the system.

    USB_HOST_CDC_RESULT_BUSY - The request could not be scheduled at this time.
    The client should try again.

    USB_HOST_CDC_RESULT_INVALID_PARAMETER - An input parameter was NULL.

    USB_HOST_CDC_RESULT_FAILURE - An unknown failure occurred.

    USB_HOST_CDC_RESULT_HANDLE_INVALID - The client handle is not valid.
    
  Example:
    <code>
    </code>

  Remarks:
    None.                                                                   
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_Write
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
    void * data,
    size_t size
);

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_Read
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
        void * data,
        size_t size
    );
           
  Summary:
    This function will read data from the attached device.

  Description:
    This function will read data from the attached CDC device. The function will
    try to read size amount of bytes but will stop reading when the device
    terminates the USB transfer (sends a short packet or a ZLP). If the request
    was accepted, transferHandle will contain a valid transfer handle, else it
    will contain USB_HOST_CDC_TRANSFER_HANDLE_INVALID. The completion of the
    request will be indicated by the USB_HOST_CDC_EVENT_READ_COMPLETE
    event. The transfer handle will be returned in the event. 

  Precondition:
    The client handle should be valid.

  Input:
    handle - handle to the CDC device instance to which the request should be
    sent.
    
    transferHandle - Pointer to USB_HOST_CDC_TRANSFER_HANDLE type of a variable.
    This will contain a valid transfer handle if the request was successful.

    data - pointer to the buffer where the received data will be stored. The
    contents of the buffer will be valid only when the
    USB_HOST_CDC_EVENT_READ_COMPLETE event has occurred.

    size - size of the data buffer. Only these many bytes or less will be read.

  Return:
    USB_HOST_CDC_RESULT_SUCCESS - The operation was successful.

    USB_HOST_CDC_RESULT_DEVICE_UNKNOWN - The device that this request was
    targeted to does not exist in the system.

    USB_HOST_CDC_RESULT_BUSY - The request could not be scheduled at this time.
    The client should try again.

    USB_HOST_CDC_RESULT_INVALID_PARAMETER - An input parameter was NULL.

    USB_HOST_CDC_RESULT_FAILURE - An unknown failure occurred.

    USB_HOST_CDC_RESULT_HANDLE_INVALID - The client handle is not valid.
    
  Example:
    <code>
    </code>

  Remarks:
    None.                                                                   
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_Read
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
    void * data,
    size_t size
);

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_SerialStateNotificationGet
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
        USB_CDC_SERIAL_STATE * serialState
    );
           
  Summary:
    This function will request Serial State Notification from the attached
    device.

  Description:
    This function will request Serial State Notification from the attached
    device. If the request was accepted, transferHandle will contain a valid
    transfer handle, else it will contain USB_HOST_CDC_TRANSFER_HANDLE_INVALID.
    The completion of the request will be indicated by the
    USB_HOST_CDC_EVENT_SERIAL_STATE_NOTIFICATION_RECEIVED event. The transfer
    handle will be returned in the event. 

  Precondition:
    The client handle should be valid.

  Input:
    handle - handle to the CDC device instance to which the request should be
    sent.
    
    transferHandle - Pointer to USB_HOST_CDC_TRANSFER_HANDLE type of a variable.
    This will contain a valid transfer handle if the request was successful.

    serialState - Pointer to the serial state data structure where the received
    serial state will be stored.

  Return:
    USB_HOST_CDC_RESULT_SUCCESS - The operation was successful.

    USB_HOST_CDC_RESULT_DEVICE_UNKNOWN - The device that this request was
    targeted to does not exist in the system.

    USB_HOST_CDC_RESULT_BUSY - The request could not be scheduled at this time.
    The client should try again.

    USB_HOST_CDC_RESULT_INVALID_PARAMETER - An input parameter was NULL.

    USB_HOST_CDC_RESULT_FAILURE - An unknown failure occurred.

    USB_HOST_CDC_RESULT_HANDLE_INVALID - The client handle is not valid.
    
  Example:
    <code>
    </code>

  Remarks:
    None.                                                                   
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_SerialStateNotificationGet
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_TRANSFER_HANDLE * transferHandle,
    USB_CDC_SERIAL_STATE * serialState
);

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_ACM_LineCodingSet
    (
        USB_HOST_CDC_HANDLE handle, 
        USB_HOST_CDC_REQUEST_HANDLE * requestHandle, 
        USB_CDC_LINE_CODING * lineCoding
    );
       
  Summary:
    This function sends a request to the attached device to set its Line Coding.

  Description:
    This function sends a request to the attached device to set its line coding.
    The function schedules a SET LINE CODING control transfer. If successful,
    the requestHandle parameter will contain a valid request handle, else it
    will contain USB_HOST_CDC_REQUEST_HANDLE_INVALID. When completed, the CDC
    client driver will generate a USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE
    event. 

  Remarks:
    Refer to usb_host_cdc_acm.h for usage information.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_ACM_LineCodingSet
(
    USB_HOST_CDC_HANDLE handle, 
    USB_HOST_CDC_REQUEST_HANDLE * requestHandle, 
    USB_CDC_LINE_CODING * lineCoding
);

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_ACM_ControlLineStateSet
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_REQUEST_HANDLE * requestHandle, 
        USB_CDC_CONTROL_LINE_STATE * controlLineState
    );
   
  Summary:
    This function sends a request to the attached device to set its Control Line
    State.

  Description:
    This function sends a request to the attached to set its Control Line State.
    The function schedules a SET CONTROL LINE STATE control transfer. If
    successful, the requestHandle parameter will contain a valid request handle,
    else it will contain USB_HOST_CDC_REQUEST_HANDLE_INVALID. When completed,
    the CDC client driver will generate a
    USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE event. 

  Remarks:
    Refer to usb_host_cdc_acm.h for usage information.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_ACM_ControlLineStateSet
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_REQUEST_HANDLE * requestHandle, 
    USB_CDC_CONTROL_LINE_STATE * controlLineState
);


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif
