/********************************************************************************
  USB HOST HID Client Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_hid.h

  Summary:
    USB Host HID Client Driver Interface Definition Header

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface between HID client 
    driver and usage driver.
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
//DOM-IGNORE-END

#ifndef _USB_HOST_HID_H_
#define _USB_HOST_HID_H_


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host.h"
#include "usb/usb_hid.h"
#include "usb/usb_host_client_driver.h"
#include "system_config.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
// *****************************************************************************
// Section: USB HID Client Driver Interface (PIC32 specific)
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: USB HID Client Driver Data Structures
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Host HID Result Minimum Constant

  Summary:
    USB Host HID Result Minimum Constant.

  Description:
    Constant identifying the USB Host HID Result Minimum Value. Constant is
    used in the USB_HOST_HID_RESULT enumeration.

  Remarks:
    None.
*/

#define USB_HOST_HID_RESULT_MIN -50

// *****************************************************************************
/* USB HOST HID Client Driver Interface
 
  Summary:
    USB HOST HID Client Driver Interface

  Description:
    This macro should be used by the application in TPL table while adding
    support for the USB HID Host Client Driver.

  Remarks:
    None.
*/

/*DOM-IGNORE-BEGIN*/extern USB_HOST_CLIENT_DRIVER gUSBHostHIDClientDriver; /*DOM-IGNORE-END*/
#define USB_HOST_HID_INTERFACE  /*DOM-IGNORE-BEGIN*/&gUSBHostHIDClientDriver /*DOM-IGNORE-END*/


// *****************************************************************************
/* USB Host HID Result

  Summary:
    USB Host HID client driver Results.

  Description:
    This enumeration defines the possible returns values of USB Host HID client
    driver API. A function may only return some of the values in this 
    enumeration. Refer to function description for details on which values will
    be returned.

  Remarks:
    None.
*/
typedef enum
{   
    /* An unknown failure occurred */
    USB_HOST_HID_RESULT_FAILURE = USB_HOST_HID_RESULT_MIN,
    
    /* Invalid or NULL parameter passed */
    USB_HOST_HID_RESULT_PARAMETER_INVALID,
    
    /* Indicates that HID client driver instance cannot accept any requests
       at this point */
    USB_HOST_HID_RESULT_REQUEST_BUSY,
    
    /* Indicates that the request has been STALLed by the attached device */
    USB_HOST_HID_RESULT_REQUEST_STALLED,
    
    /* Indicates that the operation succeeded or the request was accepted and
       will be processed. */
    USB_HOST_HID_RESULT_SUCCESS = 0

} USB_HOST_HID_RESULT;


// *****************************************************************************
/* USB HOST HID Client Driver instance Handle

  Summary:
    USB HOST HID Client Driver instance Handle

  Description:
    This defines a unique USB Host HID Client Driver Handle.
    This handle is unique to per top level Usage present in
    Report descriptor of a HID interface.

  Remarks:
    None.
*/
typedef uintptr_t USB_HOST_HID_OBJ_HANDLE;

#define USB_HOST_HID_OBJ_HANDLE_INVALID ((USB_HOST_HID_OBJ_HANDLE)(-1))

// *****************************************************************************
/* USB Host HID Request Handle Type

  Summary:
    USB Host HID Request Handle Type

  Description:
    This type defines the USB Host HID Request Handle. This type of handle is
    returned by all HID functions which accept client request. Each
    request will generate a unique handle. This handle will also be returned 
    in the event associated with the completion of the client request.

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_HID_REQUEST_HANDLE;

#define USB_HOST_HID_REQUEST_HANDLE_INVALID ((USB_HOST_HID_REQUEST_HANDLE)(-1))

// *****************************************************************************
/*  USB Host HID Global structure information

  Summary:
    USB Host HID Global structure information

  Description:
    Structure holds the information about the HID parsed global data
  
  Remarks:
    None.
*/
typedef struct _USB_HOST_HID_GLOBAL_ITEM
{
    USB_HID_USAGE_PAGE usagePage; /* Specifies the current usage page */
    int32_t  logicalMinimum; /* Minimum value that will be reported */
    int32_t  logicalMaximum; /* Maximum value that will be reported */
    int32_t  physicalMinimum; /* Minimum value for physical extent */
    int32_t  physicalMaximum; /* Maximum value for physical extent */
    int32_t  unitExponent; /* Unit exponent in base 10 */
    uint32_t unit; /* Unit value */
    uint32_t reportID; /* Report ID */
    uint32_t reportCount; /* Number of data fields for the item */
    uint32_t reportSize; /* Size of the Report fields in bits */
} USB_HOST_HID_GLOBAL_ITEM;

// *****************************************************************************
/*  USB Host HID Local structure information

  Summary:
    USB Host HID Local structure information

  Description:
    Structure holds the information about the HID parsed local items data
  
  Remarks:
    None.
*/
typedef struct _USB_HOST_HID_LOCAL_ITEM
{
    struct
    {
        /* If valid is true, min and max should be used. 
           Else USB_HOST_HID_UsageGet() function needs
           to be called by the usage driver. */
        bool valid;
        uint32_t min;
        uint32_t max;
    } usageMinMax;
    
    struct
    {
        /* If valid is true, min and max should be used. 
           Else USB_HOST_HID_StringIndexGet() function needs
           to be called by the usage driver. */
        bool valid;
        uint32_t min;
        uint32_t max;
    } stringMinMax;
    
    struct
    {
        /* If valid is true, min and max should be used. 
           Else USB_HOST_HID_DesignatorIndexGet() function needs
           to be called by the usage driver. */
        bool valid;
        uint32_t min;
        uint32_t max;
    } designatorMinMax;
    
    uint32_t delimiterBranch;
    uint32_t delimiterCounter;
    
} USB_HOST_HID_LOCAL_ITEM;


// *****************************************************************************
/*  USB Host HID Main structure information

  Summary:
    USB Host HID Main structure information

  Description:
    Structure holds the information about the HID parsed main items data.
    It also contains the current state of global and local items.
    For every Main item(Data\Non-Data) present in the Report Descriptor,
    there will be separate USB_HOST_HID_MAIN_ITEM instance which needs
    to be queried by Usage driver.
  
  Remarks:
    None.
*/
typedef struct _USB_HOST_HID_MAIN_ITEM
{
    /* Main item tag */
    USB_HID_REPORT_TYPE tag;
    /* Describes data/constant, array/variable, etc. Value specific
       to Main item tag*/
    USB_HID_MAIN_ITEM_OPTIONAL_DATA data;
    /* Pointer to global item data */
    USB_HOST_HID_GLOBAL_ITEM * globalItem;
    /* Pointer to local item data */
    USB_HOST_HID_LOCAL_ITEM * localItem;
    
} USB_HOST_HID_MAIN_ITEM ;


// *****************************************************************************
/* USB HOST HID Client Driver Events

  Summary:
    Defines the possible USB HOST HID Client Driver Events

  Description:
    This enumeration lists the possible HID events that HID client driver can
    provide to collection usage driver or the top level application as
    applicable.
    Some of these events have event data associated with them.
  
  Remarks:
    None.
 */
typedef enum
{
    /* HID Attach event - This event occurs when HID client driver has completed
       HID specific enumeration for the attached HID device. At this point of
       time both the host and device are ready for HID transfers. NULL is passed
       as event specific data field */
    USB_HOST_HID_EVENT_ATTACH,

    /* HID Detach event - This event occurs when HID device has been detached from
       the USB BUS. All necessary data structures will be released prior to
       notification of this event. NULL is passed as event specific data field */
    USB_HOST_HID_EVENT_DETACH,

    /* IN Report data available - This event occurs when INTERRUPT IN data has
       come from the HID device. This event should be a trigger for Usage drivers
       to start extraction of HID fields from the HID Report descriptor.
       E.g. - INTERRUPT IN Data sent by Mouse due to change in Mouse parameter
       (button state change, position change, etc.)
       usageDriverEventHandler function callback will notify this event with
       unprocessed INTERRUPT IN data (raw data as sent by device) as event
       specific data. This event data needs to be type caste to uint64_t.
       Report ID, if present, is not extracted from the data. */
    USB_HOST_HID_EVENT_REPORT_RECEIVED,

    /* OUTPUT Report sent - This event occurs when OUTPUT REPORT is sent by
       HID client driver. E.g. when LED Keys are pressed in a keyboard device.
       This event sends USB_HOST_HID_USAGE_DRIVER_REQUEST_RESPONSE_DATA as event
       data */
    USB_HOST_HID_EVENT_REPORT_SENT,
    
    /* IDLE TIME SET - This event occurs when SET IDLE request is sent by
       HID client driver. The event will be notified only when
       USB_HOST_HID_IdleTimeSet() function is called by the application.
       This event sends USB_HOST_HID_USAGE_DRIVER_REQUEST_RESPONSE_DATA as event
       data */
    USB_HOST_HID_EVENT_SET_IDLE_TIME_COMPLETE,
    
    /* IDLE TIME GET - This event occurs when GET IDLE request is sent by
       HID client driver. The event will be notified only when
       USB_HOST_HID_IdleTimeGet() function is called by the application.
       This event sends USB_HOST_HID_USAGE_DRIVER_REQUEST_RESPONSE_DATA as event
       data. The output parameter of USB_HOST_HID_IdleTimeGet() function is
       only valid after this event has been notified to the usage driver. */
    USB_HOST_HID_EVENT_GET_IDLE_TIME_COMPLETE,
    
    /* PROTOCOL SET - This event occurs when SET PROTOCOL request is sent by
       HID client driver. The event will be notified only when
       USB_HOST_HID_ProtocolSet() function is called by the application.
       This event sends USB_HOST_HID_USAGE_DRIVER_REQUEST_RESPONSE_DATA as event
       data */
    USB_HOST_HID_EVENT_SET_PROTOCOL_COMPLETE,
    
    /* PROTOCOL GET - This event occurs when GET PROTOCOL request is sent by
       HID client driver. The event will be notified only when
       USB_HOST_HID_ProtocolGet() function is called by the application.
       This event sends USB_HOST_HID_USAGE_DRIVER_REQUEST_RESPONSE_DATA as event
       data. The output parameter of USB_HOST_HID_ProtocolGet() function is
       only valid after this event has been notified to the usage driver. */
    USB_HOST_HID_EVENT_GET_PROTOCOL_COMPLETE
    
} USB_HOST_HID_EVENT;


// *****************************************************************************
/* USB Host HID Report Sent Event Data, IDLE time Set Event Data,
   PROTOCOL Set Event Data, IDLE time Get and PROTOCOL Get Event Data. 

  Summary:
    Defines the event data that is returned along with the
    USB_HOST_HID_EVENT_REPORT_SENT,
    USB_HOST_HID_EVENT_SET_IDLE_TIME_COMPLETE,
    USB_HOST_HID_EVENT_SET_PROTOCOL_COMPLETE,
    USB_HOST_HID_EVENT_GET_IDLE_TIME_COMPLETE and 
    USB_HOST_HID_EVENT_GET_PROTOCOL_COMPLETE events.

  Description:
    This type defines the event data that is returned with the
    USB_HOST_HID_EVENT_REPORT_SENT,
    USB_HOST_HID_EVENT_SET_IDLE_TIME_COMPLETE,
    USB_HOST_HID_EVENT_SET_PROTOCOL_COMPLETE,
    USB_HOST_HID_EVENT_GET_IDLE_TIME_COMPLETE and 
    USB_HOST_HID_EVENT_GET_PROTOCOL_COMPLETE events.

  Remarks:
    None.
*/

typedef struct
{
    /* The request handle */
    USB_HOST_HID_REQUEST_HANDLE handle;

    /* Result of the request. This will be USB_HOST_HID_RESULT_SUCCESS if the
       request was completed successfully. If the result is
       USB_HOST_HID_RESULT_REQUEST_STALLED, this means that the device stalled the
       request. If the result is USB_HOST_HID_RESULT_FAILURE, this means an unknown
       failure occurred. */
    USB_HOST_HID_RESULT result;

} USB_HOST_HID_USAGE_DRIVER_REQUEST_RESPONSE_DATA;


// *****************************************************************************
/*  USB Host HID Usage driver interface information

  Summary:
    USB Host HID Usage driver interface information

  Description:
    Structure represents the interface functions pointer passed to
    HID driver during initialization. Used as part of
    USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY table.
  
  Remarks:
    None.
*/
typedef struct
{
    /* The initialize function is called when the usage driver initializes.
       Please refer USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY for
       initializeData description */ 
    void (*initialize)(void * initializeData);

    /* The de initialize function is called when the usage driver
       initializes. */
    void (*deinitialize)(void);

    /* HID Client driver calls this function for interface level events. 
       An event may have event data associated with it. The usage driver
       must interpret the event data appropriately, based on the
       generated event */
    void (*usageDriverEventHandler)
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HOST_HID_EVENT event,
        void * eventData
    );
    
    /* HID Client driver calls this function periodically to run the usage
       driver task */
    void (*usageDriverTask)(USB_HOST_HID_OBJ_HANDLE handle);
    
} USB_HOST_HID_USAGE_DRIVER_INTERFACE;


// *****************************************************************************
/*  USB Host HID Usage driver table entry information

  Summary:
    USB Host HID Usage driver table entry information

  Description:
    Structure represents the data structure passed to
    HID driver during initialization. This structure
    is used for initialization of all usage drivers
    registered to work with USB HID client driver.
  
  Remarks:
    None.
*/
typedef struct
{
    /* Associated usage value for this Usage driver */
    uint32_t usage;
    /* Pointer to initialize data. Caller owns the memory */
    void * initializeData;
    /* Usage driver interface functions */
    USB_HOST_HID_USAGE_DRIVER_INTERFACE *interface;
    
} USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY;


// *****************************************************************************
/* USB Host HID Initialization information

  Summary:
    USB Host HID Initialization information

  Description:
    Structure represents the data structure passed to
    HID driver during initialization.
    
  Remarks:
    None.

*/
typedef struct
{
    /* Defines number of entries in usageDriverTable */
    size_t nUsageDriver;
    /* Usage driver initialization table */
    USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY *usageDriverTable;

} USB_HOST_HID_INIT;

// *****************************************************************************
// *****************************************************************************
// Section: HID Interface Function Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_MainItemGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint8_t mainItemIndex,
        USB_HOST_HID_MAIN_ITEM *pMainItemData
    );

  Summary:
    This function facilitates usage driver to get Main item data.

  Description:
    This function facilitates usage driver to get Main item data corresponding
    to the mainItemIndex in the Report Descriptor.
    
  Precondition:
    This function should be called after obtaining
    USB_HOST_HID_EVENT_REPORT_RECEIVED event from HID client driver.
    mainItemIndex should start from 1 and should be incremented
    for each call of this function till applicable
    top level usage has been parsed completely.

  Parameters:
    handle - HID client driver handle
    mainItemIndex - Field index(e.g. 1,2,3..)
    pMainItemData  - Pointer to usage driver provided buffer.

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_FAILURE - On failure to obtain field data
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of obtaining Field information
   
    </code>

  Remarks:
    Return of USB_HOST_HID_RESULT_FAILURE refers that there is no more field
    available in the Report Descriptor for this usage driver.
*/
USB_HOST_HID_RESULT USB_HOST_HID_MainItemGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint8_t mainItemIndex,
    USB_HOST_HID_MAIN_ITEM *pMainItemData
);


// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_ReportSend
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HID_REPORT_TYPE reportType,
        uint8_t reportID,
        uint16_t reportLength,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle,
        const void *report
    );

  Summary:
    This function facilitates usage driver to send REPORT. If Interrupt Out
    endpoint is present in the device, then the request will be sent to
    INTERRUPT OUT endpoint, otherwise to CONTROL endpoint.

  Description:
    This function facilitates usage driver to send REPORT(OUTPUT\FEATURE)
    
  Precondition:
    This function can only be called by Usage driver if
    (OUTPUT\FEATURE) REPORT has been obtained in the field.

  Parameters:
    handle - HID client driver handle
    reportType - Defines the report type(OUTPUT\FEATURE)
    requestHandle - Output parameter. Will contain the handle to the request
    report  - Pointer to OUTPUT\FEATURE REPORT data.

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_REQUEST_BUSY - Last request ongoing
      USB_HOST_HID_RESULT_REQUEST_STALLED - Request stalled
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of sending OUTPUT REPORT
   
    </code>

  Remarks:
    Caller owns the memory for pData.
*/
USB_HOST_HID_RESULT USB_HOST_HID_ReportSend
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HID_REPORT_TYPE reportType,
    uint8_t reportID,
    uint16_t reportLength,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle,
    const void *report
);


// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_ReportGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HID_REPORT_TYPE reportType,
        uint8_t reportID,
        uint16_t reportLength,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle,
        void *report
    );

  Summary:
    This function facilitates usage driver to obtain REPORT through Control
    endpoint.

  Description:
    This function facilitates usage driver to obtain REPORT through Control
    endpoint. It uses HID specific GET REPORT Control request to perform the 
    functionality.
    
  Precondition:
    This function can only be called after USB_HOST_HID_EVENT_ATTACH has been
    received by usage driver.

  Parameters:
    handle        - HID client driver handle
    reportType    - Defines the report type(INPUT\OUTPUT\FEATURE)
    reportID      - Report ID
    reportLength  - Length of the Report in bytes
    requestHandle - Output parameter. Will contain the handle to the request
    report        - Place holder for Report data.

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_REQUEST_BUSY - Last request ongoing
      USB_HOST_HID_RESULT_REQUEST_STALLED - Request stalled
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of sending OUTPUT REPORT
   
    </code>

  Remarks:
    Caller owns the memory for report.
*/
USB_HOST_HID_RESULT USB_HOST_HID_ReportGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HID_REPORT_TYPE reportType,
    uint8_t reportID,
    uint16_t reportLength,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle,
    void *report
);


// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_IdleTimeSet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint8_t idleTime,
        uint8_t reportID,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle
    );

  Summary:
    This function allows usage drivers to set INTERRUPT IN endpoint IDLE TIME
    for specific REPORT ID(s) or all the REPORT IDs generated by the HID
    device.
    
  Description:
    This function facilitates usage drivers to set IDLE TIME for INTERRUPT IN
    endpoint for the specific REPORT ID or all the REPORT IDs generated by the
    HID device. This limits the reporting frequency of an INTERRUPT IN
    endpoint(silence\inhibit time) to the specified time for the specific
    REPORT ID or set of REPORT IDs. 
    Time resolution is from 4 ms (idleTime = 1) to 1020 ms (idleTime = 0xFF).
    idleTime = 0 refers that the endpoint will inhibit reporting forever
    unless change detected in report data.
    Report ID refers to ID of the report whose silencing time will be changed.
    Function internally will initiate a SET IDLE CONTROL request on USB bus.
    
  Precondition:
    This function can only be called after USB_HOST_HID_EVENT_ATTACH has been
    received by usage driver.

  Parameters:
    handle - HID client driver handle
    idleTime - Endpoint IDLE time in milliseconds
    reportID - Report ID of the report whose reporting frequency needs to be
           changed. Value of 0 means all input reports generated by the
           device.
    requestHandle - Output parameter. Will contain the handle to the request

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_REQUEST_BUSY - Last request ongoing
      USB_HOST_HID_RESULT_REQUEST_STALLED - Request stalled
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of setting IDLE time.
   
    </code>

  Remarks:
    As per HID specification it is recommended to keep idle time of 500 ms
    for keyboards and infinity for mice and joystick device.
*/
USB_HOST_HID_RESULT USB_HOST_HID_IdleTimeSet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint8_t idleTime,
    uint8_t reportID,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle
);


// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_ProtocolSet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HID_PROTOCOL_TYPE  protocolType,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle
    );

  Summary:
    This function allows usage drivers to set HID device protocol.
    
  Description:
    This function facilitates usage drivers to set HID device active protocol.
    protoolType cab either BOOT or REPORT PROTOCOL.
    Function internally will initiate a SET PROTOCOL CONTROL request on USB bus.
    
  Precondition:
    This function can only be called after USB_HOST_HID_EVENT_ATTACH has been
    received by usage driver.

  Parameters:
    handle - HID client driver handle
    protoolType - Refers to BOOT or REPORT protocol.
    requestHandle - Output parameter. Will contain the handle to the request

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_REQUEST_BUSY - Last request ongoing
      USB_HOST_HID_RESULT_REQUEST_STALLED - Request stalled
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of setting HID PROTOCOL.
   
    </code>

  Remarks:
    This function is useful only when the underlying interface is a BOOT interface.
    For NON Boot interface, the active mode should always be REPORT protocol.
    Hence for NON Boot interface, changing of protocol may not be a valid
    consideration.
*/
USB_HOST_HID_RESULT USB_HOST_HID_ProtocolSet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HID_PROTOCOL_TYPE  protocolType,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle
);

// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_ProtocolGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle,
        USB_HID_PROTOCOL_TYPE *protocol
    );

  Summary:
    This function allows usage drivers to get active HID device protocol.
    
  Description:
    This function facilitates usage drivers to get HID device active protocol.
    protoolType cab either BOOT or REPORT PROTOCOL.
    Function internally will initiate a GET PROTOCOL CONTROL request on USB bus.
    
  Precondition:
    This function can only be called after USB_HOST_HID_EVENT_ATTACH has been
    received by usage driver.

  Parameters:
    handle - HID client driver handle
    requestHandle - Output parameter. Will contain the handle to the request
    protocol - Pointer to usage driver provided buffer. Protocol value
    in the buffer will be valid only after
    USB_HOST_HID_EVENT_GET_PROTOCOL_COMPLETE event has been notified.

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_REQUEST_BUSY - Last request ongoing
      USB_HOST_HID_RESULT_REQUEST_STALLED - Request stalled
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of getting HID PROTOCOL.
   
    </code>

  Remarks:
    This function is useful only when the underlying interface is a BOOT interface.
    For NON Boot interface, the active mode should always be REPORT protocol.
    Hence for NON Boot interface, obtaining protocol may not be a valid
    consideration.
*/
USB_HOST_HID_RESULT USB_HOST_HID_ProtocolGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle,
    USB_HID_PROTOCOL_TYPE *protocol
);

// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_IdleTimeGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint8_t reportID,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle,
        uint8_t *idleTime
    );

  Summary:
    This function allows usage drivers to get IDLE time for a particular
    Input Report.
    
  Description:
    This function facilitates usage drivers to get IDLE time for a particular
    Input Report. Function internally will initiate a GET IDLE CONTROL request
    on USB bus.
    
  Precondition:
    This function can only be called after USB_HOST_HID_EVENT_ATTACH has been
    received by usage driver.

  Parameters:
    handle - HID client driver handle
    reportID - Report ID of the report whose reporting frequency needs to be
           known. Value of 0 means all input reports generated by the
           device.
    requestHandle - Output parameter. Will contain the handle to the request
    idleTime - Pointer to usage driver provided buffer. Idle time value
    in the buffer will be valid only after
    USB_HOST_HID_EVENT_GET_IDLE_TIME_COMPLETE event has been notified.
    

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_REQUEST_BUSY - Last request ongoing
      USB_HOST_HID_RESULT_REQUEST_STALLED - Request stalled
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of getting IDLE RATE.
   
    </code>

  Remarks:
    None.
*/
USB_HOST_HID_RESULT USB_HOST_HID_IdleTimeGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint8_t reportID,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle,
    uint8_t *idleTime
);

// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_UsageGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint32_t mainItemIndex,
        uint32_t fieldIndex,
        uint32_t *usage
    );

  Summary:
    This function allows usage drivers to obtain usage.
    
  Description:
    This function allows usage drivers to obtain extended usage specific to a
    main Item and the field Index. Function internally will parse the report
    descriptor and will obtain the usage value applicable for the
    specific main item and field index.
    
  Precondition:
    This function can only be called after USB_HOST_HID_EVENT_ATTACH has been
    received by usage driver.

  Parameters:
    handle - HID client driver handle
    mainItemIndex - n'th field that the usage driver is querying for
    fieldIndex - n'th usage that the usage driver is querying for in the field
    usage - Place holder for extended usage. Upper 16 bit contains
                    Usage Page and the lower 16 bit contains Usage ID.

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of getting extended usage.
   
    </code>

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_UsageGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint32_t mainItemIndex,
    uint32_t fieldIndex,
    uint32_t *usage
);

// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_StringIndexGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint32_t mainItemIndex,
        uint32_t fieldIndex,
        uint32_t *stringDescriptorIndex
    );

  Summary:
    This function allows usage drivers to obtain String index.
    
  Description:
    This function allows usage drivers to obtain String index specific to a
    main item and field index. Function internally will parse the report
    descriptor and will obtain the String ID applicable for the
    specific main item and field index.
    
  Precondition:
    This function can only be called after USB_HOST_HID_EVENT_ATTACH has been
    received by usage driver.

  Parameters:
    handle - HID client driver handle
    mainItemIndex - n'th main item that the usage driver is querying for
    fieldIndex - n'th String ID that the usage driver is querying for in this
    main item
    stringDescriptorIndex - Pointer to usage driver provided buffer
    for string index.

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of getting string index.
   
    </code>

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_StringIndexGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint32_t mainItemIndex,
    uint32_t fieldIndex,
    uint32_t *stringDescriptorIndex
);

// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_DesignatorIndexGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint32_t mainItemIndex,
        uint32_t fieldIndex,
        uint32_t *physicalDescriptorDesignatorIndex
    );

  Summary:
    This function allows usage drivers to obtain Physical Descriptor Designator
    index.
    
  Description:
    This function allows usage drivers to obtain Physical Descriptor Designator
    index specific to a main Item and field count. Function internally
    will parse the report descriptor and will obtain the Designator index
    applicable for the specific main Item and field count.
    
  Precondition:
    This function can only be called after USB_HOST_HID_EVENT_ATTACH has been
    received by usage driver.

  Parameters:
    handle - HID client driver handle
    mainItemIndex - n'th main item that the usage driver is querying for
    fieldIndex - n'th designator that the usage driver is querying for in this
    main item
    physicalDescriptorDesignatorIndex - Pointer to usage driver provided buffer
    for designator index.

  Returns:
    USB_HOST_HID_RESULT type data structure is returned.
      USB_HOST_HID_RESULT_PARAMETER_INVALID - Invalid parameter
      USB_HOST_HID_RESULT_FAILURE - On failure
      USB_HOST_HID_RESULT_SUCCESS - Request accepted successfully
    
  Example:
    <code>
    // This code snippet shows an example of getting designator index.
   
    </code>

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_DesignatorIndexGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint32_t mainItemIndex,
    uint32_t fieldIndex,
    uint32_t *physicalDescriptorDesignatorIndex
);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


#endif

/*********** End of file ***************************************/
