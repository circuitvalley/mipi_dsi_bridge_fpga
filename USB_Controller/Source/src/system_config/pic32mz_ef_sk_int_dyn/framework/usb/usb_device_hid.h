/*******************************************************************************
  USB HID Function Driver

  Company:
    Microchip Technology Inc.

  File Name:
    usb_hid_function_driver.h

  Summary:
    USB HID Function Driver 

  Description:
    This file contains the API definitions for the USB Device HID Function
    Driver. The application should include this file if it needs to use the HID
    Function Driver API.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _USB_DEVICE_HID_H_
#define _USB_DEVICE_HID_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.
*/

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "system/common/sys_common.h"
#include "usb_common.h"
#include "usb_chapter_9.h"
#include "usb_device.h"
#include "usb_hid.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Device HID Function Driver Index Constants

  Summary:
    USB Device HID Function Driver Index Constants

  Description:
    This constants can be used by the application to specify HID function
    driver instance indexes.

  Remarks:
    None.
*/

#define USB_DEVICE_HID_INDEX_0 0
#define USB_DEVICE_HID_INDEX_1 1
#define USB_DEVICE_HID_INDEX_2 2
#define USB_DEVICE_HID_INDEX_3 3
#define USB_DEVICE_HID_INDEX_4 4
#define USB_DEVICE_HID_INDEX_5 5
#define USB_DEVICE_HID_INDEX_6 6
#define USB_DEVICE_HID_INDEX_7 7

// *****************************************************************************
/* USB Device HID Driver Index Numbers

  Summary:
    USB device HID Function Driver Index.

  Description:
    This uniquely identifies a HID Function Driver instance.

  Remarks:
    None.
*/

typedef uintptr_t USB_DEVICE_HID_INDEX;

// *****************************************************************************
/* USB Device HID Function Driver Transfer Handle Definition
 
  Summary:
    USB Device HID Function Driver Transfer Handle Definition.

  Description:
    This definition defines a USB Device HID Function Driver Transfer Handle.  A
    Transfer Handle is owned by the application but its value is modified by the
    USB_DEVICE_HID_ReportSend and USB_DEVICE_HID_ReportReceive functions. The
    transfer handle is valid for the life time of the transfer and expires when
    the transfer related event has occurred.

  Remarks:
    None.
*/

typedef uintptr_t USB_DEVICE_HID_TRANSFER_HANDLE;

// *****************************************************************************
/* USB Device HID Function Driver Invalid Transfer Handle Definition
 
  Summary:
    USB Device HID Function Driver Invalid Transfer Handle Definition.

  Description:
    This definition defines a USB Device HID Function Driver Invalid Transfer
    Handle.  A Invalid Transfer Handle is returned by the
    USB_DEVICE_HID_ReportReceive and USB_DEVICE_HID_ReportSend functions
    when the request was not successful.

  Remarks:
    None.
*/

#define USB_DEVICE_HID_TRANSFER_HANDLE_INVALID  /*DOM-IGNORE-BEGIN*/((USB_DEVICE_HID_TRANSFER_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* USB Device HID Function Driver USB Device HID Result enumeration.
 
  Summary:
    USB Device HID Function Driver USB Device HID Result enumeration.

  Description:
    This enumeration lists the possible USB Device HID Function Driver operation
    results. These values USB Device HID Library functions.

  Remarks:
    None.
 */

typedef enum 
{
    /* The operation was successful */
    USB_DEVICE_HID_RESULT_OK 
            /*DOM-IGNORE-BEGIN*/ = USB_ERROR_NONE /*DOM-IGNORE-END*/,

    /* The transfer queue is full. No new transfers can be
     * scheduled */
    USB_DEVICE_HID_RESULT_ERROR_TRANSFER_QUEUE_FULL 
    /*DOM-IGNORE-BEGIN*/ = USB_ERROR_IRP_QUEUE_FULL /*DOM-IGNORE-END*/,

    /* The specified instance is not configured yet */
    USB_DEVICE_HID_RESULT_ERROR_INSTANCE_NOT_CONFIGURED 
            /*DOM-IGNORE-BEGIN*/ = USB_ERROR_ENDPOINT_NOT_CONFIGURED /*DOM-IGNORE-END*/,

    /* The specified instance is not provisioned in the system */
    USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_DEVICE_FUNCTION_INSTANCE_INVALID /* DOM-IGNORE-END */,

    /* One or more parameter/s of the function is invalid */
    USB_DEVICE_HID_RESULT_ERROR_PARAMETER_INVALID = 
        /*DOM-IGNORE-BEGIN*/ USB_ERROR_PARAMETER_INVALID /*DOM-IGNORE-END*/,
    
     /* Transfer terminated because host halted the endpoint */
    USB_DEVICE_HID_RESULT_ERROR_ENDPOINT_HALTED
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_ENDPOINT_HALTED /* DOM-IGNORE-END */,

    /* Transfer terminated by host because of a stall clear */
    USB_DEVICE_HID_RESULT_ERROR_TERMINATED_BY_HOST
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_TRANSFER_TERMINATED_BY_HOST /* DOM-IGNORE-END */,

    /* General Error */
    USB_DEVICE_HID_RESULT_ERROR

} USB_DEVICE_HID_RESULT;
// *****************************************************************************
/* USB Device HID Function Driver Events

  Summary:
    USB Device HID Function Driver Events

  Description:
    These events are specific to the USB Device HID Function Driver instance.
    Each event description contains details about the  parameters passed with
    event. The contents of pData depends on the generated event.
    
    Events that are associated with the HID Function Driver Specific Control
    Transfers require application response. The application should respond to
    these events by using the USB_DEVICE_ControlReceive,
    USB_DEVICE_ControlSend and USB_DEVICE_ControlStatus functions.
    
    Calling the USB_DEVICE_ControlStatus function with a
    USB_DEVICE_CONTROL_STATUS_ERROR will stall the control transfer request.
    The application would do this if the control transfer request is not
    supported. Calling the USB_DEVICE_ControlStatus function with a
    USB_DEVICE_CONTROL_STATUS_OK will complete the status stage of the control
    transfer request. The application would do this if the control transfer
    request is supported 
    
    The following code snippet shows an example of a possible event handling
    scheme.

    <code>
    
    // This code example shows all HID Function Driver events and a possible
    // scheme for handling these events. In this example event responses are not
    // deferred.

    USB_DEVICE_HID_EVENT_RESPONSE USB_AppHIDEventHandler
    (
        USB_DEVICE_HID_INDEX instanceIndex, 
        USB_DEVICE_HID_EVENT event, 
        void * pData,
        uintptr_t userData
    )
    {
        uint8_t currentIdleRate;
        uint8_t someHIDReport[128];
        uint8_t someHIDDescriptor[128];
        USB_DEVICE_HANDLE       usbDeviceHandle;
        USB_HID_PROTOCOL_CODE  currentProtocol;
        USB_DEVICE_HID_EVENT_DATA_GET_REPORT        * getReportEventData;
        USB_DEVICE_HID_EVENT_DATA_SET_IDLE          * setIdleEventData;
        USB_DEVICE_HID_EVENT_DATA_SET_DESCRIPTOR    * setDescriptorEventData;
        USB_DEVICE_HID_EVENT_DATA_SET_REPORT        * setReportEventData;

        switch(event)
        {
            case USB_DEVICE_HID_EVENT_GET_REPORT:

                // In this case, pData should be interpreted as a
                // USB_DEVICE_HID_EVENT_DATA_GET_REPORT pointer. The application
                // must send the requested report by using the
                // USB_DEVICE_ControlSend() function. 

                getReportEventData = (USB_DEVICE_HID_EVENT_DATA_GET_REPORT *)pData;
                USB_DEVICE_ControlSend(usbDeviceHandle, someHIDReport, getReportEventData->reportLength);

                break;

            case USB_DEVICE_HID_EVENT_GET_PROTOCOL:

                // In this case, pData will be NULL. The application
                // must send the current protocol to the host by using
                // the USB_DEVICE_ControlSend() function.

               USB_DEVICE_ControlSend(usbDeviceHandle, &currentProtocol, sizeof(USB_HID_PROTOCOL_CODE)); 
               break;
                
            case USB_DEVICE_HID_EVENT_GET_IDLE:

                // In this case, pData will be a
                // USB_DEVICE_HID_EVENT_DATA_GET_IDLE pointer type containing the
                // ID of the report for which the idle rate is being requested.
                // The application must send the current idle rate to the host
                // by using the USB_DEVICE_ControlSend() function.

               USB_DEVICE_ControlSend(usbDeviceHandle, &currentIdleRate, 1); 
               break;

            case USB_DEVICE_HID_EVENT_SET_REPORT:
                
                // In this case, pData should be interpreted as a
                // USB_DEVICE_HID_EVENT_DATA_SET_REPORT type pointer. The
                // application can analyze the request and then obtain the
                // report by using the USB_DEVICE_ControlReceive() function.

                setReportEventData = (USB_DEVICE_HID_EVENT_DATA_SET_REPORT *)pData;
                USB_DEVICE_ControlReceive(deviceHandle, someHIDReport, setReportEventData->reportLength);
                break;

            case USB_DEVICE_HID_EVENT_SET_PROTOCOL:

                // In this case, pData should be interpreted as a 
                // USB_DEVICE_HID_EVENT_DATA_SET_PROTOCOL type pointer. The application can
                // analyze the data and decide to stall or accept the setting.
                // This shows an example of accepting the protocol setting.

                USB_DEVICE_ControlStatus(deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
                break;
            
            case USB_DEVICE_HID_EVENT_SET_IDLE:
            
                // In this case, pData should be interpreted as a 
                // USB_DEVICE_HID_EVENT_DATA_SET_IDLE type pointer. The
                // application can analyze the data and decide to stall
                // or accept the setting. This shows an example of accepting
                // the protocol setting.

                setIdleEventData = (USB_DEVICE_HID_EVENT_DATA_SET_IDLE *)pData;
                USB_DEVICE_ControlStatus(deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
                break;

            case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

                // In this case, control transfer data was received. The 
                // application can inspect that data and then stall the
                // handshake stage of the control transfer or accept it
                // (as shown here).
                
                USB_DEVICE_ControlStatus(deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
                break;

            case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:
                
                // This means that control transfer data was sent. The
                // application would typically acknowledge the handshake
                // stage of the control transfer.

                break;

            case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_ABORTED:

                // This is an indication only event. The application must
                // reset any HID control transfer related tasks when it receives
                // this event.

            break;

            case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:
                
                // This means a HID report receive request has completed.
                // The pData member should be interpreted as a 
                // USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED pointer type. 
                
                break;

            case USB_DEVICE_HID_EVENT_REPORT_SENT:
                
                // This means a HID report send request has completed.
                // The pData member should be interpreted as a 
                // USB_DEVICE_HID_EVENT_DATA_REPORT_SENT pointer type. 
                
                break;
        }

        return(USB_DEVICE_HID_EVENT_RESPONSE_NONE);
    }
    </code>

  Remarks:
    Some of the events allow the application to defer responses. This allows the
    application some time to obtain the response data rather than having to
    respond to the event immediately. Note that a USB host will typically wait
    for event response for a finite time duration before timing out and
    canceling the event and associated transactions. Even when deferring
    response, the application must respond promptly if such timeouts have to be
    avoided.
*/

typedef enum 
{
    /* This event occurs when the host issues a GET REPORT command. This is a
       HID class specific control transfer related event. The application must
       interpret the pData parameter as USB_DEVICE_HID_EVENT_DATA_GET_REPORT
       pointer type.  If the report request is supported, the application must
       send the report to the host by using the USB_DEVICE_ControlSend
       function either in the event handler or after the event handler routine
       has returned. The application can track the completion of the request by
       using the USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event.  If the
       report request is not supported, the application must stall the request
       by calling the USB_DEVICE_ControlStatus function with a
       USB_DEVICE_CONTROL_STATUS_ERROR flag. */
    
    USB_DEVICE_HID_EVENT_GET_REPORT,

    /* This event occurs when the host issues a GET IDLE command. This is a HID
       class specific control transfer related event. The pData parameter will
       be a USB_DEVICE_HID_EVENT_DATA_GET_IDLE pointer type containing the ID of
       the report for which the idle parameter is requested. If the request is
       supported, the application must send the idle rate to the host by calling
       the USB_DEVICE_ControlSend function. This function can be called either
       in the event handler or after the event handler routine has returned. The
       application can track the completion of the request by using the
       USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event. If the request is
       not supported, the application must stall the request by calling the
       USB_DEVICE_ControlStatus function with a
       USB_DEVICE_CONTROL_STATUS_ERROR flag. */
   
    USB_DEVICE_HID_EVENT_GET_IDLE,

    /* This event occurs when the host issues a GET PROTOCOL command. This is a
       HID class specific control transfer related event. The pData parameter
       will be NULL.  If the  request is supported, the application must send a
       USB_HID_PROTOCOL_CODE data type object, containing the current protocol,
       to the host by calling the USB_DEVICE_ControlSend function. This
       function can be called either in the event handler or after the event
       handler routine has returned. The application can track the completion of
       the request by using the USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT
       event. If the request is not supported, the application must stall the
       request by calling the USB_DEVICE_ControlStatus function with a
       USB_DEVICE_CONTROL_STATUS_ERROR flag. */
   
    USB_DEVICE_HID_EVENT_GET_PROTOCOL,

    /* This event occurs when the host issues a SET REPORT command. This is a
       HID class specific control transfer related event. The application must
       interpret the pData parameter as a USB_DEVICE_HID_EVENT_DATA_SET_REPORT
       pointer type.  If the report request is supported, the application must
       provide a buffer, to receive the report, to the host by calling the
       USB_DEVICE_ControlReceive function either in the event handler or after
       the event handler routine has returned. The application can track the
       completion of the request by using the
       USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_RECEIVED event.  If the report
       request is not supported, the application must stall the request by
       calling the USB_DEVICE_ControlStatus function with a
       USB_DEVICE_CONTROL_STATUS_ERROR flag. */
    
    USB_DEVICE_HID_EVENT_SET_REPORT,

    /* This event occurs when the host issues a SET IDLE command. This is a
       HID class specific control transfer related event. The pData parameter
       will be USB_DEVICE_HID_EVENT_DATA_SET_IDLE pointer type. The application
       can analyze the idle duration and acknowledge or reject the setting by
       calling the USB_DEVICE_ControlStatus function. This function can be
       called in the event handler or after the event handler exits. If
       application can reject the request by calling the
       USB_DEVICE_ControlStatus function with a
       USB_DEVICE_CONTROL_STATUS_ERROR flag. It can accept the request by
       calling this function with USB_DEVICE_CONTROL_STATUS_OK flag. */ 
   
    USB_DEVICE_HID_EVENT_SET_IDLE,

    /* This event occurs when the host issues a SET PROTOCOL command. This is a
       HID class specific control transfer related event. The pData parameter
       will be a pointer to a USB_DEVICE_HID_EVENT_DATA_SET_PROTOCOL data type.
       If the request is supported, the application must acknowledge the request
       by calling the USB_DEVICE_ControlStatus function with a
       USB_DEVICE_CONTROL_STATUS_OK flag. If the request is not supported, the
       application must stall the request by calling the
       USB_DEVICE_ControlStatus function with a
       USB_DEVICE_CONTROL_STATUS_ERROR flag. */

    USB_DEVICE_HID_EVENT_SET_PROTOCOL,

    /* This event indicates that USB_DEVICE_HID_ReportSend function
       completed a report transfer on interrupt endpoint from host to device.
       The pData parameter will be a USB_DEVICE_HID_EVENT_DATA_REPORT_SENT type.
       */
    
    USB_DEVICE_HID_EVENT_REPORT_SENT,

    /* This event indicates that USB_DEVICE_HID_ReportReceive function
       completed a report transfer on interrupt endpoint from device to host.
       The pData parameter will be a USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED
       type */
    
    USB_DEVICE_HID_EVENT_REPORT_RECEIVED,
            
    /* This event occurs when the data stage of a control write transfer has
       completed. This happens after the application uses the
       USB_DEVICE_ControlReceive function to respond to a HID Function Driver
       Control Transfer Event that requires data to be received from the host.
       The pData parameter will be NULL. The application should call the
       USB_DEVICE_ControlStatus function with the USB_DEVICE_CONTROL_STATUS_OK
       flag if the received data is acceptable or should call this function with
       USB_DEVICE_CONTROL_STATUS_ERROR flag if the received data needs to be
       rejected.*/

    USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_RECEIVED 
            /*DOM-IGNORE-BEGIN*/ = USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_RECEIVED /*DOM-IGNORE-END*/,

    /* This event occurs when the data stage of a control read transfer has
       completed. This happens after the application uses the
       USB_DEVICE_ControlSend function to respond to a HID Function Driver
       Control Transfer Event that requires data to be sent to the host. The
       pData parameter will be NULL */
    
    USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT 
            /*DOM-IGNORE-BEGIN*/ = USB_DEVICE_EVENT_CONTROL_TRANSFER_DATA_SENT /*DOM-IGNORE-END*/,

    /* This event occurs when an ongoing control transfer was aborted.
       The application must stop any pending control transfer related
       activities.
       */
    
    USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_ABORTED 
            /*DOM-IGNORE-BEGIN*/ = USB_DEVICE_EVENT_CONTROL_TRANSFER_ABORTED /*DOM-IGNORE-END*/,

    
} USB_DEVICE_HID_EVENT;

// *****************************************************************************
/* USB Device HID Get Idle Event Data

  Summary:
    USB Device HID Get Idle Event Data Type.

  Description:
    This defines the data type of the data generated to the HID event
    handler on a USB_DEVICE_HID_EVENT_GET_IDLE event.

  Remarks:
    None.
*/

typedef struct
{
    /* The protocol code */
    uint8_t reportID;

} USB_DEVICE_HID_EVENT_DATA_GET_IDLE;

// *****************************************************************************
/* USB Device HID Set Protocol Event Data

  Summary:
    USB Device HID Set Protocol Event Data Type.

  Description:
    This defines the data type of the data generated to the HID event
    handler on a USB_DEVICE_HID_EVENT_SET_PROTOCOL event.

  Remarks:
    None.
*/

typedef struct
{
    /* The protocol code */
    USB_HID_PROTOCOL_CODE protocolCode;

} USB_DEVICE_HID_EVENT_DATA_SET_PROTOCOL;


// *****************************************************************************
/* USB Device HID Report Sent Event Data Type.

  Summary:
    USB Device HID Report Sent Event Data Type.

  Description:
    This defines the data type of the data generated to the HID event
    handler on a USB_DEVICE_HID_EVENT_REPORT_SENT event.

  Remarks:
    None.
*/

typedef struct 
{
    /* Transfer handle */
    USB_DEVICE_HID_TRANSFER_HANDLE handle;
    
    /* Report size transmitted */
    size_t length;
    
    /* Completion status of the transfer */
    USB_DEVICE_HID_RESULT status;

} USB_DEVICE_HID_EVENT_DATA_REPORT_SENT;

// *****************************************************************************
/* USB Device HID Report Received Event Data Type.

  Summary:
    USB Device HID Report Received Event Data Type.

  Description:
    This defines the data type of the data generated to the HID event
    handler on a USB_DEVICE_HID_EVENT_REPORT_RECEIVED event.

  Remarks:
    None.
*/

typedef struct 
{
    /* Transfer handle */
    USB_DEVICE_HID_TRANSFER_HANDLE handle;
    
    /* Report size received */
    size_t length;
    
    /* Completion status of the transfer */
    USB_DEVICE_HID_RESULT status;

} USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED;

// *****************************************************************************
/* USB Device HID Set Idle Event Data Type.

  Summary:
    USB Device HID Set Idle Event Data Type.

  Description:
    This defines the data type of the data generated to the HID event
    handler on a USB_DEVICE_HID_EVENT_SET_IDLE event.

  Remarks:
    None.
*/

typedef struct 
{
    /* Idle duration */
    uint8_t duration;

    /* Report ID*/
    uint8_t reportID;

} USB_DEVICE_HID_EVENT_DATA_SET_IDLE;

// *****************************************************************************
/* USB Device HID Get Report Event Data Type.

  Summary:
    USB Device HID Get Report Event Data Type.

  Description:
    This defines the data type of the data generated to the HID event
    handler on a USB_DEVICE_HID_EVENT_GET_REPORT event.

  Remarks:
    None.
*/

typedef struct 
{
    /* Report type */
    uint8_t reportType;

    /* Report ID */
    uint8_t reportID;

    /* Report Length*/
    uint16_t reportLength;

} USB_DEVICE_HID_EVENT_DATA_GET_REPORT;

// *****************************************************************************
/* USB Device HID Set Report Event Data Type.

  Summary:
    USB Device HID Set Report Event Data Type.

  Description:
    This defines the data type of the data generated to the HID event
    handler on a USB_DEVICE_HID_EVENT_SET_REPORT event.

  Remarks:
    None.
*/

typedef struct 
{
    /* Report type */
    uint8_t reportType;

    /* Report ID */
    uint8_t reportID;

    /* Report Length */
    uint16_t reportLength;

} USB_DEVICE_HID_EVENT_DATA_SET_REPORT;

// *****************************************************************************
/* USB Device HID Function Driver Event Handler Response Type

  Summary:
    USB Device HID Function Driver Event Callback Response Type

  Description:
    This is the return type of the HID Function Driver event handler.

  Remarks:
    None.
*/

typedef void USB_DEVICE_HID_EVENT_RESPONSE;

// *****************************************************************************
/* USB Device HID Event Handler Function Pointer Type.

  Summary:
    USB Device HID Event Handler Function Pointer Type.

  Description:
    This data type defines the required function signature of the USB Device HID
    Function Driver event handling callback function. The application must
    register a pointer to a HID Function Driver events handling function whose
    function signature (parameter and return value types) match the types
    specified by this function pointer in order to receive event call backs from
    the HID Function Driver. The function driver will invoke this function with
    event relevant parameters. The description of the event handler function
    parameters is given here.

    instanceIndex           - Instance index of the HID Function Driver that 
                              generated the event.
    
    event                   - Type of event generated.
    
    pData                   - This parameter should be type casted to a event 
                              specific pointer type based on the event that has
                              occurred. Refer to the USB_DEVICE_HID_EVENT 
                              enumeration description for more details.
    
    context                 - Value identifying the context of the application 
                              that registered the event handling function.

  Remarks:
    None.
*/

typedef USB_DEVICE_HID_EVENT_RESPONSE (*USB_DEVICE_HID_EVENT_HANDLER)
(
    USB_DEVICE_HID_INDEX instanceIndex,
    USB_DEVICE_HID_EVENT event,
    void * pData,
    uintptr_t context
);

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Device HID Function Driver Event Handler Response None  

  Summary:
    USB Device HID Function Driver Event Handler Response Type None.

  Description:
    This is the definition of the HID Function Driver Event Handler Response
    Type none.

  Remarks:
    Intentionally defined to be empty.
*/

#define USB_DEVICE_HID_EVENT_RESPONSE_NONE

// *****************************************************************************
// *****************************************************************************
// Section: API definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    USB_DEVICE_HID_RESULT USB_DEVICE_HID_EventHandlerSet
    (
        USB_DEVICE_HID_INDEX instance 
        USB_DEVICE_HID_EVENT_HANDLER eventHandler 
        uintptr_t context
    );

  Summary:
    This function registers a event handler for the specified HID function
    driver instance. 

  Description:
    This function registers a event handler for the specified HID function
    driver instance. This function should be called by the client when it 
    receives a SET CONFIGURATION event from the device layer. A event handler
    must be registered for function driver to respond to function driver 
    specific commands. If the event handler is not registered, the device
    layer will stall function driver specific commands and the USB device
    may not function. 
    
  Precondition:
    This function should be called when the function driver has been initialized
    as a result of a set configuration.

  Parameters:
    instance        - Instance of the HID Function Driver.

    eventHandler    - A pointer to event handler function.
    
    context         - Application specific context that is returned in the 
                      event handler.

  Returns:
    USB_DEVICE_HID_RESULT_OK - The operation was successful

    USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID - The specified instance does 
    not exist.

    USB_DEVICE_HID_RESULT_ERROR_PARAMETER_INVALID - The eventHandler parameter is 
    NULL
    
  Example:
    <code>
    // This code snippet shows an example registering an event handler. Here
    // the application specifies the context parameter as a pointer to an
    // application object (appObject) that should be associated with this 
    // instance of the HID function driver.
    
    USB_DEVICE_HID_RESULT result;
    
    USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler 
    (
        USB_DEVICE_HID_INDEX instanceIndex,
        USB_DEVICE_HID_EVENT event,
        void * pData,
        uintptr_t context 
    )
    {
        // Event Handling comes here

        switch(event) 
        {
            ...
        }

        return(USB_DEVICE_HID_EVENT_RESPONSE_NONE);
    }

    result = USB_DEVICE_HID_EventHandlerSet (0, &APP_EventHandler, (uintptr_t) &appObject);

    if(USB_DEVICE_HID_RESULT_OK != result)
    {
        SYS_ASSERT ( false , "Error while registering event handler" );
    }
  
    </code>

  Remarks:
    None.
*/

USB_DEVICE_HID_RESULT USB_DEVICE_HID_EventHandlerSet 
( 
    USB_DEVICE_HID_INDEX instanceIndex ,
    USB_DEVICE_HID_EVENT_HANDLER eventHandler,
    uintptr_t context
);

//******************************************************************************
/* Function:
    USB_DEVICE_HID_RESULT USB_DEVICE_HID_ReportSend
    (
        USB_DEVICE_HID_INDEX instanceIndex,
        USB_DEVICE_HID_TRANSFER_HANDLE * transferHandle,
        void * buffer, 
        size_t size
    )

  Summary:
    This function submits the buffer to HID function driver library to
    send a report from device to host. 

  Description:
    This function places a request to send a HID report with the USB Device HID
    Function Driver Layer. The function places a requests with driver, the
    request will get serviced when report is requested by the USB Host. A handle
    to the request is returned in the transferHandle parameter. The termination
    of the request is indicated by the USB_DEVICE_HID_EVENT_REPORT_SENT event.
    The amount of data sent, a pointer to the report and the transfer handle
    associated with the request is returned along with the event in the pData
    parameter of the event handler.  The transfer handle expires when event
    handler for the USB_DEVICE_HID_EVENT_REPORT_SENT exits. If the send request
    could not be accepted, the function returns an error code and transferHandle
    will contain the value USB_DEVICE_HID_TRANSFER_HANDLE_INVALID.

  Precondition:
    USB device layer must be initialized.

  Parameters:
    instance  - USB Device HID Function Driver instance.

    transferHandle - Pointer to a USB_DEVICE_HID_TRANSFER_HANDLE type of
                     variable. This variable will contain the transfer handle
                     in case the send request was  successful.
    
    data - pointer to the data buffer containing the report to be sent.
    
    size - Size (in bytes) of the report to be sent.

  Returns:
    USB_DEVICE_HID_RESULT_OK - The send request was successful. transferHandle
    contains a valid transfer handle.
    
    USB_DEVICE_HID_RESULT_ERROR_TRANSFER_QUEUE_FULL - Internal request queue 
    is full. The send request could not be added.

    USB_DEVICE_HID_RESULT_ERROR_INSTANCE_NOT_CONFIGURED - The specified 
    instance is not configured yet.

    USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID - The specified instance
    was not provisioned in the application and is invalid.

  Example:
    <code>
    USB_DEVICE_HID_TRANSFER_HANDLE hidTransferHandle;
    USB_DEVICE_HID_RESULT result;

    // Register APP_HIDEventHandler function
    USB_DEVICE_HID_EventHandlerSet( USB_DEVICE_HID_INDEX_0 ,
                                    APP_HIDEventHandler );

    // Prepare report and request HID to send the report.
    result = USB_DEVICE_HID_ReportSend( USB_DEVICE_HID_INDEX_0,
                               &hidTransferHandle ,
                               &appReport[0], sizeof(appReport));

    if( result != USB_DEVICE_HID_RESULT_OK)
    {
       //Handle error.

    }

    //Implementation of APP_HIDEventHandler

    USB_DEVICE_HIDE_EVENT_RESPONSE APP_HIDEventHandler
    {
        USB_DEVICE_HID_INDEX instanceIndex,
        USB_DEVICE_HID_EVENT event,
        void * pData,
        uintptr_t context
    }
    {
        USB_DEVICE_HID_EVENT_DATA_REPORT_SENT * reportSentEventData;

        // Handle HID events here.
        switch (event)
        {
            case USB_DEVICE_HID_EVENT_REPORT_SENT:
                
                reportSentEventData = (USB_DEVICE_HID_EVENT_REPORT_SENT *)pData;
                if(reportSentEventData->reportSize == sizeof(appReport))
                {
                    // The report was sent completely.
                }
                break;

                ....

        }
        return(USB_DEVICE_HID_EVENT_RESPONSE_NONE);
    }

    </code>

  Remarks:
    While the using the HID Function Driver with the PIC32MZ USB module, the
    report data buffer provided to the USB_DEVICE_HID_ReportSend function should
    be placed in coherent memory and aligned at a 16 byte boundary.  This can be
    done by declaring the buffer using the __attribute__((coherent, aligned(16)))
    attribute.  An example is shown here

    <code>
    uint8_t data[256] __attribute__((coherent, aligned(16)));
    </code>
*/

USB_DEVICE_HID_RESULT USB_DEVICE_HID_ReportSend
(
    USB_DEVICE_HID_INDEX instanceIndex,
    USB_DEVICE_HID_TRANSFER_HANDLE * handle,
    void * buffer, 
    size_t size
);

//******************************************************************************
/* Function:
    USB_DEVICE_HID_RESULT USB_DEVICE_HID_ReportReceive 
    (
        USB_DEVICE_HID_INDEX instanceIndex,
        USB_DEVICE_HID_TRANSFER_HANDLE * transferHandle,
        void * buffer, 
        size_t size
    );

  Summary:
    This function submits the buffer to HID function driver library to
    receive a report from host to device.

  Description:
    This function submits the buffer to HID function driver library to receive a
    report from host to device. On completion of the transfer the library
    generates USB_DEVICE_HID_EVENT_REPORT_RECEIVED event to the application. A
    handle to the request is passed in the transferHandle parameter. The
    transfer handle expires when event handler for the
    USB_DEVICE_HID_EVENT_REPORT_RECEIVED exits. If the receive request could
    not be accepted, the function returns an error code and transferHandle
    will contain the value USB_DEVICE_HID_TRANSFER_HANDLE_INVALID.

  Precondition:
    USB device layer must be initialized.

  Parameters:
    instanceIndex            - HID instance index.

    transferHandle  - HID transfer handle.

    buffer          - Pointer to buffer where the received report has to be 
                      received stored.

    size            - Buffer size.

  Returns:
    USB_DEVICE_HID_RESULT_OK - The receive request was successful. transferHandle
    contains a valid transfer handle.
    
    USB_DEVICE_HID_RESULT_ERROR_TRANSFER_QUEUE_FULL - internal request queue 
    is full. The receive request could not be added.

    USB_DEVICE_HID_RESULT_ERROR_INSTANCE_NOT_CONFIGURED - The specified 
    instance is not configured yet.

    USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID - The specified instance
    was not provisioned in the application and is invalid.

  Example:
    <code>
      USB_DEVICE_HID_TRANSFER_HANDLE hidTransferHandle;
      USB_DEVICE_HID_RESULT result;

      // Register APP_HIDEventHandler function
      USB_DEVICE_HID_EventHandlerSet( USB_DEVICE_HID_INDEX_0 ,
                                      APP_HIDEventHandler );

      // Prepare report and request HID to send the report.
      result = USB_DEVICE_HID_ReportReceive( USB_DEVICE_HID_INDEX_0,
                                 &hidTransferHandle ,
                                 &appReport[0], sizeof(appReport));

      if( result != USB_DEVICE_HID_RESULT_OK)
      {
         //Handle error.

      }

      //Implementation of APP_HIDEventHandler

      USB_DEVICE_HIDE_EVENT_RESPONSE APP_HIDEventHandler
      {
        USB_DEVICE_HID_INDEX instanceIndex,
        USB_DEVICE_HID_EVENT event,
        void * pData,
        uintptr_t context
      }
      {
          USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED reportReceivedEventData;
          // Handle HID events here.
          switch (event)
          {
              case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:
                 if( (reportReceivedEventData->reportSize == sizeof(appReport)
                      && reportReceivedEventData->report == &appReport[0])
                 {
                    // Previous transfer was complete.
                 }
                 break;
              ....
          }
      }
    </code>

  Remarks:
    While the using the HID Function Driver with the PIC32MZ USB module, the
    report data buffer provided to the USB_DEVICE_HID_ReportReceive function should
    be placed in coherent memory and aligned at a 16 byte boundary.  This can be
    done by declaring the buffer using the __attribute__((coherent, aligned(16)))
    attribute.  An example is shown here

    <code>
    uint8_t data[256] __attribute__((coherent, aligned(16)));
    </code>
*/

USB_DEVICE_HID_RESULT USB_DEVICE_HID_ReportReceive 
(
    USB_DEVICE_HID_INDEX instanceIndex,
    USB_DEVICE_HID_TRANSFER_HANDLE * handle,
    void * buffer, 
    size_t size
);

//******************************************************************************
/* Function:
    USB_DEVICE_HID_RESULT USB_DEVICE_HID_TransferCancel
    (
        USB_DEVICE_HID_INDEX instanceIndex,
        USB_DEVICE_HID_TRANSFER_HANDLE transferHandle
    );
 
  Summary:
    This function cancels a scheduled HID Device data transfer.

  Description:
    This function cancels a scheduled HID Device data transfer. The transfer
    could have been scheduled using the USB_DEVICE_HID_ReportReceive,
    USB_DEVICE_HID_ReportSend function. If a transfer is still in the queue
    and its processing has not started, then the transfer is canceled
    completely. A transfer that is in progress may or may not get canceled
    depending on the transaction that is presently in progress. If the last
    transaction of the transfer is in progress, then the transfer will not be
    canceled.  If it is not the last transaction in progress, the in-progress
    will be allowed to complete. Pending transactions will be canceled. The
    first transaction of an in progress transfer cannot be canceled.

  Precondition:
    The USB Device should be in a configured state.

  Parameters:
    instanceIndex  - HID Function Driver instance index.

    transferHandle - Transfer handle of the transfer to be canceled.
    
  Returns:
    USB_DEVICE_HID_RESULT_OK    - The transfer will be canceled completely or 
                              partially.

    USB_DEVICE_HID_RESULT_ERROR_PARAMETER_INVALID - Invalid transfer handle
 
    USB_DEVICE_HID_RESULT_ERROR_INSTANCE_INVALID - Invalid HID instance index
    
    USB_DEVICE_HID_RESULT_ERROR - The transfer could not be canceled because it
                              has either completed, the transfer handle is 
                              invalid or the last transaction is in progress.

  Example:
    <code>

    // The following code snippet cancels a HID transfer.

    USB_DEVICE_HID_TRANSFER_HANDLE transferHandle;
    USB_DEVICE_HID_RESULT result;

    result = USB_DEVICE_HID_TransferCancel(instanceIndex, transferHandle);

    if(USB_DEVICE_HID_RESULT_OK == result)
    {
        // The transfer cancellation was either completely or 
        // partially successful.
    }
    
    </code>

  Remarks:
    The buffer specific to the transfer handle should not be released unless
    the transfer abort event is notified through callback.
*/

USB_DEVICE_HID_RESULT USB_DEVICE_HID_TransferCancel
(
    USB_DEVICE_HID_INDEX usbDeviceHandle,
    USB_DEVICE_HID_TRANSFER_HANDLE transferHandle
);

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and constants specific to PIC32 implementation of the
// USB Device Stack.
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Device HID Function Driver Device Layer callback function pointer group

  Summary:
    This is a pointer to a group of HID Function Driver callback function 
    pointers. 

  Description:
    This is a pointer to a group of HID Function Driver callback function
    pointers.  The application must use this pointer while registering an
    instance of the HID function driver with the Device Layer via the  function
    driver registration table i.e. the driver member of the function driver
    registration object in the device layer function driver registration table
    should be set to this value.

  Remarks:
    None.
*/

/*DOM-IGNORE-BEGIN*/extern const USB_DEVICE_FUNCTION_DRIVER hidFuncDriver; /*DOM-IGNORE-END*/

#define USB_DEVICE_HID_FUNCTION_DRIVER /*DOM-IGNORE-BEGIN*/&hidFuncDriver/*DOM-IGNORE-END*/

// *****************************************************************************
/* USB Device HID Function Driver Initialization Data Structure

  Summary:
    USB Device HID Function Driver Initialization Data Structure

  Description:
    This data structure must be defined for every instance of the HID function
    driver. It is passed to the HID function driver, by the Device Layer, at the
    time of initialization. The funcDriverInit member of the Device Layer
    Function Driver registration table entry must point to this data structure
    for an instance of the HID function driver.

  Remarks:
    None.
*/

typedef struct 
{
    /* Size of the HID report descriptor */
    size_t hidReportDescriptorSize;

    /* Pointer to HID report descriptor */
    void * hidReportDescriptor;

    /* Report send queue size */
    size_t queueSizeReportSend;

    /* Report receive queue size */
    size_t queueSizeReportReceive;
    
} USB_DEVICE_HID_INIT;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif 
/*******************************************************************************
 End of File
*/
