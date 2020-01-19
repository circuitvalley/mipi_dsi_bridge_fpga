/*******************************************************************************
  USB Host SCSI Client Driver definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_scsi.h

  Summary:
    USB Host SCSI related definitions.

  Description:
    This file contains USB Host SCSI driver related definitions.
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

#ifndef _USB_HOST_SCSI_H
#define _USB_HOST_SCSI_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/fs/sys_fs_media_manager.h"
#include "usb/usb_host_msd.h"
#include "driver/driver_common.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
/* SCSI Driver Command Handle.

  Summary:
    Handle identifying commands queued in the driver.

  Description:
    A command handle is returned by a call to the Read, Write functions. This
    handle allows the application to track the completion of the operation. This
    command handle is also returned to the client along with the event that has
    occurred with respect to the command.  This allows the application to
    connect the event to a specific command in case where multiple commands are
    queued.

    The command handle associated with the command request expires when the
    client has been notified of the completion of the command (after event
    handler function that notifies the client returns) or after the command has
    been retired by the driver if no event handler callback was set. 

  Remarks:
    None.
*/

typedef SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE USB_HOST_SCSI_COMMAND_HANDLE;

// *****************************************************************************
/* USB Host SCSI Object.
 
  Summary: 
    Defines the type of the SCSI Host Client Object.

  Description:
    This type defines the type of the SCSI Host Client Object. This type
    is returned by the Attach Event Handler and is used by the application to
    open the attached SCSI Device.  

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_SCSI_OBJ;

// *****************************************************************************
/* SCSI Driver Invalid Command Handle.

  Summary:
    This value defines the SCSI Driver's Invalid Command Handle.

  Description:
    This value defines the SCSI Driver Invalid Command Handle. This value is
    returned by read/write routines when the command request was not accepted.

  Remarks:
    None.
*/

#define USB_HOST_SCSI_COMMAND_HANDLE_INVALID SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID

// *****************************************************************************
/* SCSI Result Enumeration.

   Summary
    Identifies the possible value of USB_HOST_SCSI_RESULT.

   Description
    This enumeration identifies the possible values of USB_HOST_SCSI_RESULT. 
    A SCSI Driver function may return one of these values.

   Remarks:
    None.
*/

typedef enum
{
    /* One of the parameters passed to the function was not valid */
    USB_HOST_SCSI_RESULT_PARAMETER_INVALID = -1,
       
    /* The function did not execute successfully. */
    USB_HOST_SCSI_RESULT_FAILURE = 0,
            
    /* The function returned successfully */
    USB_HOST_SCSI_RESULT_SUCCESS = 1
    
} USB_HOST_SCSI_RESULT;

// *****************************************************************************
/* SCSI Driver Events.

   Summary
    Identifies the possible events that can result from a request.

   Description
    This enumeration identifies the possible events that can result from a 
    Write or Erase request caused by the client.

   Remarks:
    One of these values is passed in the "event" parameter of the event handling
    callback function that client registered with the driver by calling the
    USB_HOST_SCSI_EventHandlerSet function when a request is completed.
*/

typedef enum
{
    /* Operation has been completed successfully. */
    USB_HOST_SCSI_EVENT_COMMAND_COMPLETE = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE,

    /* There was an error during the operation */
    USB_HOST_SCSI_EVENT_COMMAND_ERROR = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR,
            
    /* This SCSI device in no longer available */
    USB_HOST_SCSI_EVENT_DETACH = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR + 1

} USB_HOST_SCSI_EVENT;

// *****************************************************************************
/* SCSI Driver Error Codes.

   Summary
    Identifies the different error codes that the SCSI driver can return.

   Description
    This enumeration identifies the different error codes that the SCSI driver
    can return when the Error callback function is enabled. Not all codes
    indicate error conditions. Some of these indicate conditions of the driver
    state machine. 

   Remarks:
    The Error callback function is enabled by defining the
    USB_HOST_SCSI_ERROR_CALLBACK function in system_config.h and setting it to
    true.
*/

typedef enum
{
    /* This error code occurs when the number of SCSI instances provisioned in
     * the system are insufficient. This value is configured by the
     * USB_HOST_MSD_LUN_NUMBERS configuration constant in system_config.h file.
     * The attached LUN will not operate if this error occurs. The value of the
     * object identifier will be the USB_HOST_MSD_LUN_HANDLE value. */ 
    USB_HOST_SCSI_ERROR_CODE_INSUFFICIENT_INSTANCES = 1,

    /* This error code occurs when the driver client has called the
     * USB_HOST_SCSI_SectorWrite function but the media is write protected. The
     * attached LUN will continue to operate after this error occurs. The
     * USB_HOST_SCSI_SectorWrite function related event will occur with block
     * command failure result. The value of the object identifier will be
     * the USB_HOST_SCS_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_MEDIA_WRITE_PROTECTED,

    /* This error code is a condition code that indicates that the driver has
     * detected a period of idle of operation and will start sending the Test
     * Unit Ready command periodically to find out if the media is still
     * connected. The value of the object identifier will be the USB_HOST_SCSI_OBJ
     * value. */
    USB_HOST_SCSI_ERROR_CODE_IDLE_TEST_UNIT_READY,

    /* This error code is a condition code that indicates that the driver has
     * detected that the Test Unit Ready command has failed for
     * USB_HOST_SCSI_DETACH_TIME_OUT seconds and the driver has detached the
     * media. The media will be inaccessible after this error. The value of the
     * object identifier will be the USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_DETACH_TIME_OUT,

    /* This error code occurs when the Mode Sense command has failed and driver
     * is about to issue a Request Sense command to find out why the command
     * failed. The media will be accessible after this error occurs. The
     * value of the object identifier will be the USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_MODE_SENSE_FAILED,

    /* This error code occurs when the Read Capacity command has failed and
     * driver is about to issue a Request Sense command to find out why the
     * command failed. The media will be accessible after this error occurs. The
     * value of the object identifier will be the USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_READ_CAPACITY_FAILED,

    /* This error code is a condition code that occurs when the driver has
     * issued a Request Sense command and the command response is Unit
     * Attention, Not Ready or No Sense. The driver will send the Test Unit
     * Ready command in this case. The media will be accessible after this
     * condition occurs. The value of the object identifier will be the
     * USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_REQUEST_SENSE_TEST_UNIT_READY,

    /* This error code occurs when the driver has issued a Request Sense Command
     * and the sense key is Medium Error. The driver will detach the media. The
     * media will be inaccessible after the condition occurs. The value of the
     * object identifier will be the USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_REQUEST_SENSE_MEDIUM_ERROR,

    /* This error code occurs when the driver has issued a Request Sense Command
     * and the sense key is Medium Error. The media will be inaccessible after
     * the condition occurs. The value of the object identifier will be the
     * USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_REQUEST_SENSE_FAILURE,

    /* This error code is a condition code that occurs when the driver has
     * deferred the processing of the SCSI read write command because the MSD
     * BOT state machine is busy. The media will continue to be accessible after
     * this condition occurs. The value of the object identifier will be the
     * SCSI client handle value. */
    USB_HOST_SCSI_ERROR_CODE_BOT_REQUEST_DEFERRED,

    /* This error code occurs when the USB_HOST_SCSI_SectorWrite or
     * USB_HOST_SCSI_SectorRead function has failed because the SCSI state is
     * busy and not ready for new requests. The media will continue to be
     * accessible after this condition occurs. The value of the object
     * identifier will be the SCSI client handle value. */
    USB_HOST_SCSI_ERROR_CODE_INSTANCE_BUSY,

    /* This error code is a condition that occurs when USB_HOST_SCSI_SectorRead
     * or USB_HOST_SCSI_SectorWrite function has failed and the driver has sent
     * a Request Sense command to find out the response. If the Request Sense
     * response is Sense Not Ready, the driver will send a Test Unit Ready
     * command periodically. The media will continue to be accessible after this
     * condition occurs. The value of the object identifier will be the
     * USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_READ_WRITE_TEST_UNIT_READY,

    /* This error code occurs when the USB_HOST_SCSI_SectorRead or
     * USB_HOST_SCSI_SectorWrite function has failed and the driver has sent a
     * Request Sense command to find out the response. If the Request Sense
     * response is Medium Not Present, the driver will detach the device. The
     * media will not be accessible after this condition occurs. The value of
     * the object identifier will be the USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_READ_WRITE_MEDIUM_NOT_PRESENT,

    /* This error code occurs when the USB_HOST_SCSI_SectorRead or
     * USB_HOST_SCSI_SectorWrite function has failed and the driver has sent a
     * Request Sense command to find out the response. If the Request Sense
     * response is Medium Error, the driver unload the device. The media will
     * not be accessible after this condition occurs. The value of the object
     * identifier will be the USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_READ_WRITE_MEDIUM_ERROR,

    /* This error occurs when the USB_HOST_SCSI_SectorWrite or
     * USB_HOST_SCSI_SectorWrite function has failed and the driver has sent a
     * Request Sense command to find out the response. If the Request Sense
     * response is Unit Not Ready, the driver will send the Test Unit Ready
     * command. If the Test Unit Ready does not pass in
     * USB_HOST_SCSI_COMMAND_FAILURE_TEST_UNIT_READY_NUMBER of times, the read
     * write command will fail. The media will be accessible after this
     * condition occurs. The value of the object identifier will be the
     * USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_READ_WRITE_TEST_UNIT_FAILURE,

    /* This error occurs when the USB_HOST_SCSI_Open function is called and the
     * driver is not ready to be opened. The value of the object identifier will
     * be the USB_HOST_SCSI_OBJ value. */
    USB_HOST_SCSI_ERROR_CODE_OPEN_FAIL_ON_BUSY

} USB_HOST_SCSI_ERROR_CODE;

// *****************************************************************************
/* USB Host SCSI Client Driver Attach Event Handler Function Pointer Type.

  Summary:
    USB Host SCSI Client Driver Attach Event Handler Function Pointer Type.

  Description:
    This data type defines the required function signature of the USB Host SCSI
    Client Driver attach event handling callback function. The application can
    register a pointer to a SCSI Client Driver attach events handling function
    whose function signature (parameter and return value types) match the types
    specified by this function pointer in order to receive attach event call
    backs from the SCSI Client Driver. The client driver will invoke this
    function with event relevant parameters. The description of the event
    handler function parameters is given here.

    scsiObjHandle - Handle to the SCSI Object that the client can use in the 
    USB_HOST_SCSI_Open function.
    
    context - Value identifying the context of the application that was
    registered along with  the event handling function.

  Remarks:
    None.
*/

typedef void (* USB_HOST_SCSI_ATTACH_EVENT_HANDLER)
(
    USB_HOST_SCSI_OBJ scsiObjHandle, 
    uintptr_t context
);

// *****************************************************************************
/* SCSI Driver Event Handler Function Pointer

   Summary
    Pointer to a SCSI Driver Event handler function

   Description
    This data type defines the required function signature for the NVM event
    handling callback function. A client must register a pointer to an event
    handling function whose function signature (parameter and return value 
    types) match the types specified by this function pointer in order to 
    receive event calls back from the driver.
    
    The parameters and return values are described here and a partial example
    implementation is provided.

  Parameters:
    event           - Identifies the type of event
    
    commandHandle   - Handle returned from the Read/Write/Erase requests
    
    context         - Value identifying the context of the application that
                      registered the event handling function

  Returns:
    None.

  Example:
    <code>
    void APP_MySCSIEventHandler
    (
        USB_HOST_SCSI_EVENT event,
        USB_HOST_SCSI_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;
        
        switch(event)
        {
            case USB_HOST_SCSI_EVENT_COMMAND_COMPLETE:

                // Handle the completed buffer. 
                break;
            
            case USB_HOST_SCSI_EVENT_COMMAND_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is USB_HOST_SCSI_EVENT_COMMAND_COMPLETE, it means that the
    write or a read operation was completed successfully. 
    
    If the event is USB_HOST_SCSI_EVENT_COMMAND_ERROR, it means that the
    scheduled operation was not completed successfully.
     
    The context parameter contains the handle to the client context, provided at
    the time the event handling function was  registered using the
    USB_HOST_SCSI_EventHandlerSet function. This context handle value is passed
    back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the read/write/erase
    request.

    The event handler function executes in the driver peripheral's interrupt
    context when the driver is configured for interrupt mode operation. It is
    recommended of the application to not perform process intensive or blocking
    operations within this function.
*/

typedef SYS_FS_MEDIA_EVENT_HANDLER USB_HOST_SCSI_EVENT_HANDLER;

// *****************************************************************************
/*  USB Host SCSI Driver  Handle

  Summary:
    USB Host SCSI Driver Handle.

  Description:
    This type defines the type of the handle that is returned by the
    USB_HOST_SCSI_Open() function.

  Remarks:
    None.
*/

typedef DRV_HANDLE USB_HOST_SCSI_HANDLE;

// *****************************************************************************
/*  USB Host SCSI Driver Invalid Handle

  Summary:
    USB Host SCSI Driver Invalid Handle.

  Description:
    This constant defines an invalid handle. The USB_HOST_SCSI_Open() function
    will return an invalid handle if the open function failed.

  Remarks:
    None.
*/

#define USB_HOST_SCSI_HANDLE_INVALID ((USB_HOST_SCSI_HANDLE)(-1))

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_Initialize (USB_HOST_MSD_LUN_HANDLE lunHandle)

  Summary:
    This function is called by the MSD Host Client Driver for detected LUN.

  Description:
    This function is called by the MSD Host Client Driver for detected LUN. The
    function will then register this LUN as drive with the file system media
    manager, thus allowing the media manager to access this drive.

  Precondition:
    None.

  Parameters:
    lunHandle - handle to the LUN that the SCSI driver should handle.

  Returns:
    None.

  Examples:
  <code>
  </code>

  Remarks:
    The application will never need to call this function. This function is
    always called the USB Host MSD Client Driver.
*/

void USB_HOST_SCSI_Initialize(USB_HOST_MSD_LUN_HANDLE lunHandle);

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_DeInitialize (USB_HOST_MSD_LUN_HANDLE lunHandle)

  Summary:
    This function is called by the MSD Host Client Driver for detached LUN.

  Description:
    This function is called by the MSD Host Client Driver for detached LUN. The
    function will then deregister this LUN from the file system media manager.

  Precondition:
    None.

  Parameters:
    lunHandle - handle to the LUN that was deregistered.

  Returns:
    None.

  Examples:
    <code>
    </code>

  Remarks:
    The application will never need to call this function. This function is
    always called the USB Host MSD Client Driver.
*/

void USB_HOST_SCSI_Deinitialize(USB_HOST_MSD_LUN_HANDLE lunHandle);

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_Tasks (USB_HOST_MSD_LUN_HANDLE lunHandle)

  Summary:
    This function is called by the MSD Host Client Driver to update the driver
    state machine.

  Description:
    This function is called by the MSD Host Client Driver to update the driver
    state machine.

  Precondition:
    None.

  Parameters:
    lunHandle - handle to the LUN for which the state machine needs to be udpated.

  Returns:
    None.

  Examples:
    <code>
    </code>

  Remarks:
    The application will never need to call this function. This function is
    always called the USB Host MSD Client Driver.
*/

void USB_HOST_SCSI_Tasks(USB_HOST_MSD_LUN_HANDLE lunHandle);

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

  Precondition:
    None.

  Parameters:
    index - index of the SCSI driver to open.
    intent - access intent of client (read only, write only, read-write etc.)

  Returns:
    USB_HOST_SCSI_HANDLE_INVALID if the instance is not ready to opened, a valid
    handle otherwise.

  Example:
    <code>
    </code>

  Remarks:
    This function is typically called by the file system media manager.
*/

USB_HOST_SCSI_HANDLE USB_HOST_SCSI_Open 
(
    SYS_MODULE_INDEX index, 
    DRV_IO_INTENT intent
);

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
    handle in scsiHandle will not be valid anymore. The driver must be opened
    once again.

  Precondition:
    None.

  Parameters:
    scsiHandle - handle to the driver to be closed.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    This function is typically called by the file system media manager.
*/

void USB_HOST_SCSI_Close 
(
    USB_HOST_SCSI_HANDLE scsiHandle
);
					
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

  Precondition:
    None.

  Parameters:
    scsiHandle - handle to the driver to be closed.

  Returns:
    true - media is attached and ready to use.
    false - media is detached.

  Example:
    <code>
    </code>

  Remarks:
    This function is typically called by the file system media manager.
*/

bool USB_HOST_SCSI_MediaStatusGet 
(
    USB_HOST_SCSI_HANDLE scsiHandle
);

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
    Peforms a block read operation.

  Description:
    This function will perform a block read operation. The operation will not
    complete when the function returns. Instead a handle to the operation will
    be returned in commandHandle and driver will generate an event when the
    operation has completed. The command handle will be returned along with the
    event. The data will be stored in buffer. Data will be read from the
    blockStart block and nBlock buffers will be read.

  Precondition:
    None.

  Parameters:
    scsiHandle - handle to the driver to be closed.

    commandHandle - pointer to a variable where the command handle will be stored.

    buffer - Destination buffer. The contents of this buffer are valid only when
    data has been received (data has been generated).

    blockStart - block from which data should be read.

    nBlock - number of blocks to read.

  Returns:
    A handle to the command is returned in commandHandle. This will be
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID if the request failed.

  Example:
    <code>
    </code>

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
);

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
    Peforms a block write operation.

  Description:
    This function will perform a block write operation. The operation will not
    complete when the function returns. Instead a handle to the operation will
    be returned in commandHandle and driver will generate an event when the
    operation has completed. The command handle will be returned along with the
    event. The data to be written is specified by buffer. Data will be written
    to the blockStart block and nBlock buffers will be written.

  Precondition:
    None.

  Parameters:
    scsiHandle - handle to the driver to be closed.

    commandHandle - pointer to a variable where the command handle will be stored.

    buffer - Source buffer. The contents of this buffer should not be modified
    untill the write operation has completed (till the event has been
    generated).

    blockStart - block to which data should be written to.

    nBlock - number of blocks to written.

  Returns:
    A handle to the command is returned in commandHandle. This will be
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID if the request failed.

  Example:
    <code>
    </code>

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
);

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

  Precondition:
    None.

  Parameters:
    scsiHandle - handle to the driver to be closed.

    eventHandler - pointer to the event handler.

    context - client specific context that is returned in the event handler.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    This function is typically called by the file system media manager.
*/

void USB_HOST_SCSI_EventHandlerSet 
(
    USB_HOST_SCSI_HANDLE scsiHandle,
    const void * eventHandler,
    const uintptr_t context
);


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
    will be called when a MSD device has been attached. The context will be
    returned in the event handler. This function should be called before the bus
    has been enabled.

  Precondition:
    None.

  Input:
    eventHandler - pointer to the attach event handler
    
    context - an application defined context that will be returned in the event
    handler.

  Return:
    USB_HOST_SCSI_RESULT_SUCCESS - if the attach event handler was registered
    successfully. 

    USB_HOST_SCSI_RESULT_FAILURE - if the number of registered event handlers has
    exceeded USB_HOST_SCSI_ATTACH_LISTENERS_NUMBER.

  Example:
    <code>
    </code>

  Remarks:
    Function should be called before USB_HOST_BusEnable() function is called.
*/

USB_HOST_SCSI_RESULT USB_HOST_SCSI_AttachEventHandlerSet
(
    USB_HOST_SCSI_ATTACH_EVENT_HANDLER eventHandler,
    uintptr_t context
);

USB_HOST_MSD_LUN_HANDLE USB_HOST_SCSI_MSDLUNHandleGet(USB_HOST_SCSI_OBJ scsiObj);

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

  Precondition:
    None.

  Parameters:
    scsiHandle - handle to the driver.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    This function is typically called by the file system media manager.
*/

SYS_FS_MEDIA_GEOMETRY * USB_HOST_SCSI_MediaGeometryGet 
(
    USB_HOST_SCSI_HANDLE scsiHandle
);

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

  Precondition:
    None.

  Parameters:
    lunHandle - handle to the LUN for which the state machine needs to be udpated.

  Returns:
    None.

  Examples:
    <code>
    </code>

  Remarks:

*/

void USB_HOST_SCSI_TransferTasks(USB_HOST_MSD_LUN_HANDLE lunHandle);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif

