 /*******************************************************************************
  USB HOST scsi subclass definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_scsi_local.h

  Summary:
    USB HOST SCSI subclass definitions

  Description:
    This file describes the SCSI sub class specific definitions.
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

#ifndef _USB_HOST_SCSI_LOCAL_H
#define _USB_HOST_SCSI_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "usb/usb_host_scsi.h"
#include "system/tmr/sys_tmr.h"

/* If the USB_HOST_SCSI_FILE_SYSTEM_REGISTER constant is defined and is set to
 * to false, then the any instance of the SCSI driver will not register with 
 * file system */

#if defined(USB_HOST_SCSI_FILE_SYSTEM_REGISTER)
    
    #if (USB_HOST_SCSI_FILE_SYSTEM_REGISTER == false)

        /* The file system register and deregister functions are set to empty */
        #define _USB_HOST_SCSI_FILE_SYSTEM_REGISTER(w,x,y,z) (SYS_FS_MEDIA_HANDLE_INVALID)
        #define _USB_HOST_SCSI_FILE_SYSTEM_DEREGISTER(w) 

    #else

        /* The file system register and deregister functions are set to
         * the SYS_FS_MEDIA_MANAGER_Register and SYS_FS_MEDIA_MANAGER_DeRegister functions. */
        #define _USB_HOST_SCSI_FILE_SYSTEM_REGISTER(w,x,y,z) SYS_FS_MEDIA_MANAGER_Register(w, x, y, z);
        #define _USB_HOST_SCSI_FILE_SYSTEM_DEREGISTER(w) SYS_FS_MEDIA_MANAGER_DeRegister(w)
    #endif

#else

    /* This is the default behavior. The file system register and deregister functions 
     * are set to the SYS_FS_MEDIA_MANAGER_Register and SYS_FS_MEDIA_MANAGER_DeRegister functions. */
    #define _USB_HOST_SCSI_FILE_SYSTEM_REGISTER(w,x,y,z) SYS_FS_MEDIA_MANAGER_Register(w, x, y, z)
    #define _USB_HOST_SCSI_FILE_SYSTEM_DEREGISTER(w) SYS_FS_MEDIA_MANAGER_DeRegister(w)
    
#endif

#if !defined(USB_HOST_SCSI_ATTACH_LISTENERS_NUMBER)

    /* If the USB_HOST_SCSI_ATTACH_LISTENERS_NUMBER is not defined in system_config.h
     * then we know that the project does not want to use SCSI attach listeners */
    #define USB_HOST_SCSI_ATTACH_LISTENERS_NUMBER 0

#endif

/* If the USB_HOST_SCSI_ERROR_CALLBACK constant is defined and is set to true,
 * then the driver will call a USB_HOST_SCSI_ErrorCallback function when an
 * error or other conditions occur. The error and condition codes are defined by
 * USB_HOST_SCSI_ERROR_CODE enum. */

#if defined(USB_HOST_SCSI_ERROR_CALLBACK)
    #if (USB_HOST_SCSI_ERROR_CALLBACK == true)
        #define _USB_HOST_SCSI_ERROR_CALLBACK_DECLARE extern void USB_HOST_SCSI_ErrorCallback(uintptr_t objIdentifier, USB_HOST_SCSI_ERROR_CODE errorCode);
        #define _USB_HOST_SCSI_ERROR_CALLBACK(x, y) USB_HOST_SCSI_ErrorCallback(x, y)
    #else
        #define _USB_HOST_SCSI_ERROR_CALLBACK_DECLARE
        #define _USB_HOST_SCSI_ERROR_CALLBACK(x, y) 
    #endif
#else
    #define _USB_HOST_SCSI_ERROR_CALLBACK_DECLARE
    #define _USB_HOST_SCSI_ERROR_CALLBACK(x, y)
#endif


/* Defines the number of times the SCSI driver will send the Test Unit Ready
 * command when a block command fails due to a unit not being ready. The device
 * must declare ready within these many times before the driver fails the block
 * command. */

#define USB_HOST_SCSI_COMMAND_FAILURE_TEST_UNIT_READY_NUMBER 100

/* Defines the interval (in milliseconds) at which the SCSI driver checks if the
 * media is still attached */

#define USB_HOST_SCSI_DETACH_TEST_UNIT_READY_INTERVAL 100

/* Defines the duration (in milliseconds) during which if all Test Units have
 * failed, the SCSI driver considers that the media is not present. This must be
 * a multiple of USB_HOST_SCSI_DETACH_TEST_UNIT_READY_INTERVAL. */

#define USB_HOST_SCSI_DETACH_TIME_OUT 500

/******************************************
 * At this time coherent attribute is no
 * not available on PIC32C. Using this to
 * to fix the warning that get generated
 ******************************************/

#if defined (__PIC32C__)
#define SCSI_COHERENT_ATTRIBUTE
#else
#define SCSI_COHERENT_ATTRIBUTE __attribute__ ((coherent))
#endif

/*******************************************
 * USB Host CDC Attach Listener Objects
 ******************************************/
typedef struct
{
    /* This object is in use */
    bool inUse;

    /* The attach event handler */
    USB_HOST_SCSI_ATTACH_EVENT_HANDLER eventHandler;

    /* Client context */
    uintptr_t context;

} USB_HOST_SCSI_ATTACH_LISTENER_OBJ;

/*****************************************************************************
*  SCSI Task state
*****************************************************************************/
typedef enum
{
    /* The unit is in an error state */
    USB_HOST_SCSI_STATE_ERROR = -1,

    /* The SCSI object is not initialized */
    USB_HOST_SCSI_STATE_NOT_READY = 0,

    /* Get Inquiry response */
    USB_HOST_SCSI_STATE_INQUIRY_RESPONSE,

    /* Waiting for inquiry response */
    USB_HOST_SCSI_STATE_WAIT_INQUIRY_RESPONSE,

    /* Send read capacity command */
    USB_HOST_SCSI_STATE_READ_CAPACITY,

    /* Wait for read capacity to complete */
    USB_HOST_SCSI_STATE_WAIT_READ_CAPACITY,

    /* Send mode sense command */
    USB_HOST_SCSI_STATE_MODE_SENSE,

    /* Wait for Mode Sense command */
    USB_HOST_SCSI_STATE_WAIT_MODE_SENSE,

    /* Mode sense is done */
    USB_HOST_SCSI_STATE_MODE_SENSE_DONE,
    
    /* Unit is ready general data transfer */
    USB_HOST_SCSI_STATE_READY,
    
    /* Send the request sense command */
    USB_HOST_SCSI_STATE_REQUEST_SENSE,

    /* Wait for request sense to complete */
    USB_HOST_SCSI_STATE_WAIT_REQUEST_SENSE,

    /* Start delay before sending the test unit ready command */
    USB_HOST_SCSI_STATE_TEST_UNIT_READY_DELAY_START,

    /* Wait for the delay to complete */
    USB_HOST_SCSI_STATE_TEST_UNIT_READY_DELAY_WAIT,

    /* Check if the unit is ready */
    USB_HOST_SCSI_STATE_TEST_UNIT_READY,
    
    /* Wait for Test unit ready to complete */
    USB_HOST_SCSI_STATE_WAIT_TEST_UNIT_READY

} USB_HOST_SCSI_STATE;

/******************************************************
 * SCSI Transfer State
 ******************************************************/
typedef enum
{
    /* Nothing to do in the transfer state machine */
    USB_HOST_SCSI_TRANSFER_STATE_IDLE = 0,

    /* Send the Request Sense when a transfer has failed */
    USB_HOST_SCSI_TRANSFER_STATE_REQUEST_SENSE,

    /* Waiting for the request sense to complete */
    USB_HOST_SCSI_TRANSFER_STATE_WAIT_REQUEST_SENSE,

    /* If the request sense reported that the unit is not ready, then the state
     * machine sends a test unit ready command. */
    USB_HOST_SCSI_TRANSFER_STATE_TEST_UNIT_READY,

    /* In this state, the state machine waits for the test unit ready to
     * complete */
    USB_HOST_SCSI_TRANSFER_STATE_WAIT_TEST_UNIT_READY,

    /* In this state, the state machine starts the test unit delay */
    USB_HOST_SCSI_TRANSFER_STATE_TEST_UNIT_READY_DELAY,

    /* In this state, the state machine waits for the test unit delay to
     * complete */
    USB_HOST_SCSI_TRANSFER_STATE_WAIT_TEST_UNIT_READY_DELAY,

    /* In this state, the state machine retries the command */
    USB_HOST_SCSI_TRANSFER_STATE_BLOCK_COMMAND_RETRY

} USB_HOST_SCSI_TRANSFER_STATE;

/******************************************************
 * SCSI Detach Task State
 ******************************************************/
typedef enum
{
    /* Detach task is in an IDLE state */
    USB_HOST_SCSI_DETACH_TASK_STATE_IDLE = 0,

    /* The detach task sends the Test Unit ready */
    USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_SEND,

    /* The detach tasks waits and then checks the Test Unit ready response */
    USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_WAIT,

    /* The detach tasks has processed the test unit ready and is now starting
     * for delay before sending the next test unit ready */
    USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_DELAY,

    /* The detach tasks is waiting for the delay to complete */
    USB_HOST_SCSI_DETACH_TASK_STATE_TEST_UNIT_READY_DELAY_WAIT

} USB_HOST_SCSI_DETACH_TASK_STATE;

/******************************************************
 * SCSI Command object
 ******************************************************/

typedef struct
{
    /* Command object is in use */
    bool inUse;

    /* The CDB buffer for this command object */
    uint8_t cdb[16];

    /* True if the command is completed */
    bool commandCompleted;

    /* The result of the command */
    USB_HOST_MSD_RESULT result;

    /* The processed size */
    size_t size;

    /* The user buffer */
    void * buffer;

    /* The number of sectors to be transferred */
    size_t nSectors;

    /* The direction of the transfer */
    USB_HOST_MSD_TRANSFER_DIRECTION direction;

} USB_HOST_SCSI_COMMAND_OBJ;

/******************************************************
 * SCSI Object
 ******************************************************/
typedef struct 
{
    /* Flag to indicate in use  */
    bool inUse;

    /* Value to return to the file system at time of deinitialize */
    SYS_FS_MEDIA_HANDLE fsHandle;
    
    /* The status of the SCSI object*/
    USB_HOST_SCSI_STATE state;

    /* The transfer state */
    USB_HOST_SCSI_TRANSFER_STATE transferTaskState;

    /* The detach task state */
    USB_HOST_SCSI_DETACH_TASK_STATE detachTaskState;

    /* Tracks the command that failed */
    USB_HOST_SCSI_STATE failedCommandState;
    
    /* The LUN Handle provided by MSD */
    USB_HOST_MSD_LUN_HANDLE lunHandle;

    /* The command object used for block commands */
    USB_HOST_SCSI_COMMAND_OBJ commandObj;

    /* The task command object used by the SCSI task */
    USB_HOST_SCSI_COMMAND_OBJ taskCommandObj;

    /* This command object is used to send the request sense and test unit ready
     * commands when the read10 or write10 command has failed. */
    USB_HOST_SCSI_COMMAND_OBJ transferErrorCommandObj;

    /* Media Geometry object */
    SYS_FS_MEDIA_GEOMETRY mediaGeometry;

    /* Media region table */
    SYS_FS_MEDIA_REGION_GEOMETRY mediaRegionGeometry[3];

    /* Event Handler */
    USB_HOST_SCSI_EVENT_HANDLER eventHandler;

    /* Application specified context */
    uintptr_t context;
    
    /* Internal buffer */
    uint8_t * buffer;

    /* True if media is write protected */
    bool isWriteProtected;

    /* True if media is attached and is ready to use */
    bool isMediaReady;

    /* True if a media error was detected */
    bool isMediaError;

    /* The command delay */
    size_t nCommandFailureTestUnitReadyAttempts;

    /* The command delay system timer object */
    SYS_TMR_HANDLE commandDelayHandle;
    
    /* Flag is set when the timer expires */
    bool timerExpired ;

    /* This get incremented when a test unit ready fails */
    size_t detachTimeOut;

} USB_HOST_SCSI_INSTANCE_OBJ;

/*****************************************************************************
*  SCSI commands
*****************************************************************************/
#define    USB_HOST_SCSI_FORMAT_COMMAND        0x04  // Format
#define    USB_HOST_SCSI_READ6_COMMAND         0x08   // Read
#define    USB_HOST_SCSI_READ10_COMMAND        0x28  // Read
#define    USB_HOST_SCSI_WRITE6_COMMAND        0x0A  // write
#define    USB_HOST_SCSI_WRITE10_COMMAND        0x2A  // write
#define    USB_HOST_SCSI_SEEK6_COMMAND          0x0B  // track access
#define    USB_HOST_SCSI_SEEK10_COMMAND         0x2B  // track access

/*****************************************************************************
*  SCSI Transparent Command Set Sub-class code
*****************************************************************************/
#define USB_SCSI_INQUIRY 				    0x12
#define USB_SCSI_READ_FORMAT_CAPACITY 		0x23
#define USB_SCSI_READ_CAPACITY 				0x25
#define USB_SCSI_READ_10 				    0x28
#define USB_SCSI_WRITE_10 				    0x2a
#define USB_SCSI_REQUEST_SENSE 				0x03
#define USB_SCSI_MODE_SENSE  				0x1a
#define USB_SCSI_TEST_UNIT_READY 			0x00
#define USB_SCSI_VERIFY 					0x2f
#define USB_SCSI_STOP_START 				0x1b

#define USB_SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL   0x1E



/* Error */
#define USB_HOST_DRIVE_NUMBER_INVALID  -1
#define USB_HOST_DRIVE_MAP_INVALID     -2
/* command status */
#define    USB_HOST_SCSI_COMMAND_SUCCESS         0x00
#define    USB_HOST_SCSI_COMMAND_FAIL            0x02  //command fail, send request sense
#define    USB_HOST_SCSI_BUSY                    0x08  // Busy

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_TransferCallback 
    (
        USB_HOST_MSD_LUN_HANDLE lunHandle,
        USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
        USB_HOST_MSD_RESULT result,
        size_t size,
        uintptr_t context
    )

  Summary:
    This function is called when a MSD transfer has completed.

  Description:
    This function is called when a MSD transfer has completed.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_TransferCallback 
(
    USB_HOST_MSD_LUN_HANDLE lunHandle,
    USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
    USB_HOST_MSD_RESULT result,
    size_t size,
    uintptr_t context
);

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_InquiryResponseCommand 
    (
        USB_HOST_MSD_COMMAND * msdCommand
    )

  Summary:
    Sets up the SCSI Inquiry Response command.

  Description:
    This function sets up the SCSI Inquiry Response Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_InquiryResponseCommand (uint8_t * scsiCommand );

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_ReadFormatCapacityCommand
    (
        USB_HOST_MSD_COMMAND * msdCommand
    )

  Summary:
    Sets up the SCSI Read Format Capacity command.

  Description:
    This function sets up the SCSI Read Format Capacity Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_ReadFormatCapacityCommand (uint8_t * scsiCommand );

// ******************************************************************************
/* Function:
    int _USB_HOST_SCSI_LUNHandleToSCSIInstance 
    (
        USB_HOST_MSD_LUN_HANDLE lunHandle
    )

  Summary:
    This function maps a LUN Handle to the SCSI instance.

  Description:
    This function maps a LUN Handle to the SCSI instance. It returns the index
    of the SCSI object. It will return -1 if the object is not found.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_SCSI_LUNHandleToSCSIInstance
(
    USB_HOST_MSD_LUN_HANDLE lunHandle
);

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_ReadCapacityCommand
    (
        USB_HOST_MSD_COMMAND * msdCommand
    )

  Summary:
    Sets up the Read Capacity Command.

  Description:
    This function sets up the Read Capacity Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_ReadCapacityCommand (uint8_t * scsiCommand );

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_RequestSenseCommand 
    (
        USB_HOST_MSD_COMMAND * msdCommand
    )

  Summary:
    Sets up the Request Sense command.

  Description:
    This function sets up the Request Sense Command.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_RequestSenseCommand (uint8_t * scsiCommand );

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_BlockTransferCallback 
    (
        USB_HOST_MSD_LUN_HANDLE lunHandle,
        USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
        USB_HOST_MSD_RESULT result,
        size_t size,
        uintptr_t context
    )

  Summary:
    This function is called when a MSD SCSI Block transfer has completed.

  Description:
    This function is called when a MSD SCSI Block transfer has completed.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_SCSI_BlockTransferCallback 
(
    USB_HOST_MSD_LUN_HANDLE lunHandle,
    USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
    USB_HOST_MSD_RESULT result,
    size_t size,
    uintptr_t context
);

// ******************************************************************************
/* Function:
    void _USB_HOST_SCSI_DetachDetectTasks(int scsiObjIndex);

  Summary:
    This function is called periodically by the USB_HOST_SCSI_Tasks() function
    to check if the media is still attached.

  Description:
    This function is called periodically by the USB_HOST_SCSI_Tasks() function
    to check if the media is still attached. It uses the Test Unit ready command
    to check if the media responding. If the media is not responding the
    function moves the main state machine to initial state.

  Remarks:
    None.
*/

void _USB_HOST_SCSI_DetachDetectTasks(int scsiObjIndex);

// ******************************************************************************
/* Function:
    void USB_HOST_SCSI_Tasks (USB_HOST_MSD_LUN_HANDLE lunHandle)

  Summary:
    This function is called by the MSD Host Client Driver to update the driver
    state machine.

  Description:
    This function is called by the MSD Host Client Driver to update the driver
    state machine.

  Remarks:
    The application will never need to call this function. This function is
    always called the USB Host MSD Client Driver.
*/

void USB_HOST_SCSI_Tasks(USB_HOST_MSD_LUN_HANDLE lunHandle);

// *****************************************************************************
/* Function:
    void * _USB_HOST_SCSI_TimerCallback
    (
       uint32_t context,
       uint32_t currtick
    )

  Summary:
    Function is called when the SYS_TMR_CallbackSingle expires.

  Description:
    Function is called when the SYS_TMR_CallbackSingle expires.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_SCSI_TimerCallback(uintptr_t context, uint32_t currtick);
#endif

