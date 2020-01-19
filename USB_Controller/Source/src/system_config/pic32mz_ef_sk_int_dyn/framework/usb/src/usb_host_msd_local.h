 /*******************************************************************************
  USB HOST MSD class definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_msd_local.h

  Summary:
    USB MSD class definitions

  Description:
    This file describes the MSD class specific definitions.
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

#ifndef _USB_HOST_MSD_LOCAL_H
#define _USB_HOST_MSD_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host.h"
#include "system_config.h"
#include "usb/src/usb_host_local.h"
#include "usb/usb_host_msd.h"
#include "usb/usb_msd.h"

#if defined(USB_HOST_MSD_ERROR_CALLBACK)
    /* Check if error callback has been configured */
    #if (USB_HOST_MSD_ERROR_CALLBACK == true)
        #define _USB_HOST_MSD_ERROR_CALLBACK_DECLARE extern void USB_HOST_MSD_ErrorCallback(uintptr_t objIdentifier, USB_HOST_MSD_ERROR_CODE errorCode);
        #define _USB_HOST_MSD_ERROR_CALLBACK(x, y) USB_HOST_MSD_ErrorCallback(x, y)
    #else
        #define _USB_HOST_MSD_ERROR_CALLBACK_DECLARE
        #define _USB_HOST_MSD_ERROR_CALLBACK(x, y) 
    #endif
#else
     #define _USB_HOST_MSD_ERROR_CALLBACK_DECLARE
    #define _USB_HOST_MSD_ERROR_CALLBACK(x, y)
#endif

/******************************************
 * At this time coherent attribute is no
 * not available on PIC32C. Using this to
 * to fix the warning that get generated
 ******************************************/

#if defined (__PIC32C__)
#define COHERENT_ATTRIBUTE
#else
#define COHERENT_ATTRIBUTE __attribute__ ((coherent))
#endif

/**********************************************
 * Macro to create the LUN handle and get the
 * MSD instance and LUN from the LUN handle.
 **********************************************/

#define USB_HOST_MSD_LUNHandleGet(lun, instance) ((lun << 8) | (instance))
#define USB_HOST_MSD_INDEX(x) (x & 0xFF)
#define USB_HOST_MSD_LUN(x) ((x >> 8) & 0xFF)

/************************************************
 * An MSD Transfer Object that is used by the
 * the MSD object. This tracks the BOT transfers.
 ************************************************/

typedef struct
{
    /* True if this object is assigned */
    bool inUse;

    /* A callback to be called when the request is complete */
    USB_HOST_MSD_TRANSFER_CALLBACK callback;
    
    /* Pointer to the command */
    uint8_t * cdb;

    /* The command length */
    uint8_t cdbLength;
    
    /* Size of the data when the control transfer completed. */
    size_t size;

    /* Transfer direction */
    USB_HOST_MSD_TRANSFER_DIRECTION transferDirection;

    /* The context specified by the caller */
    uintptr_t context;

    /* The LUN Handle */
    USB_HOST_MSD_LUN_HANDLE lunHandle;

    /* Pointer to the buffer */
    void * buffer;


} USB_HOST_MSD_TRANSFER_OBJ;

/********************************************
 * MSD Client Driver Transfer Error States
 *******************************************/

typedef enum
{
    /* The are no transfer related errors */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_NO_ERROR,

    /* A Reset Recovery needs to be performed*/
    USB_HOST_MSD_TRANSFER_ERROR_STATE_RESET_RECOVERY,

    /* Waiting for MSD Reset to complete */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_WAIT_MSD_RESET,

    /* Clearing the IN endpoint stall as a part of Reset Recovery */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_IN,

    /* Waiting for the Reset Recovery IN endpoint stall to clear */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_IN_WAIT,
    
    /* Clearing the OUT endpoint stall as a part of Reset Recovery */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_OUT,
    
    /* Waiting for the Reset Recovery OUT endpoint stall to clear */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_RR_CLEAR_FEATURE_OUT_WAIT,
    
    /* Pipe stalled during data stage */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_IN_PIPE_STALLED,

    /* Waiting for the stall on the IN pipe to clear */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_IN_PIPE_STALLED_CLEAR_WAIT,

    /* Pipe stalled during data stage */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_OUT_PIPE_STALLED,

    /* Waiting for the stall on the OUT pipe to clear */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_OUT_PIPE_STALLED_CLEAR_WAIT,

    /* The CSW was stalled */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_STALLED,

    /* Waiting for CSW stall to clear */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_STALLED_IN_PIPE_CLEAR_WAIT,
    
    /* Indicates that the state machine is now in retrying the CSW */
    USB_HOST_MSD_TRANSFER_ERROR_STATE_CSW_RETRY

} USB_HOST_MSD_TRANSFER_ERROR_STATE;

/********************************************
 * USB HOST MSD Client Driver States
 *******************************************/

typedef enum
{
    /* The device is in an error state */
    USB_HOST_MSD_STATE_ERROR_HOLDING = -2,

    /* The device is in an getting into the error state */
    USB_HOST_MSD_STATE_ERROR = -1,
    
    /* The device is not ready */
    USB_HOST_MSD_STATE_NOT_READY = 0,

    /* The MSD Host will get the Maximum LUN */
    USB_HOST_MSD_STATE_GET_MAX_LUN,

    /* The MSD host is waiting for Get Max LUN to complete */
    USB_HOST_MSD_STATE_WAITING_GET_MAX_LUN,
            
    /* The device is now ready for BOT commands */
    USB_HOST_MSD_STATE_READY,

} USB_HOST_MSD_STATE;

/**************************************************
 * MSD Host Transfer State
 **************************************************/

typedef enum
{
    /* MSD Transfer is in an error state */
    USB_HOST_MSD_TRANSFER_STATE_ERROR = -1,

    /* MSD Host is ready for a transfer */
    USB_HOST_MSD_TRANSFER_STATE_READY = 0,

    /* Waiting for CBW to complete */
    USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CBW,

    /* Waiting for data to complete */
    USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_DATA,
            
    /* Waiting for CSW */
    USB_HOST_MSD_TRANSFER_STATE_WAIT_FOR_CSW

} USB_HOST_MSD_TRANSFER_STATE;


// *****************************************************************************
/* USB HOST MSD Client Driver data structure

  Summary:
    USB HOST MSD Client Driver information

  Description:
    This structre has the details about USB HOST MSD Client Driver info
    event handler function , states of device , instance , pipes info
    contain all the information about the device driver.

  Remarks:
    None.
*/

typedef struct  
{
    /* True if this object is assigned */
    bool assigned;

    /* Pointer to the CBW for this instance */
    USB_MSD_CBW   *msdCBW;

    /* Pointer to the CSW for this instance */
    USB_MSD_CSW   *msdCSW;

    /* CSW phase error has occurred */
    bool cswPhaseError;

    /* Handle to the device object provided by the host */
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle;

    /* The handle to the MSD interface on this device*/
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle;

    /* The state of this object */
    USB_HOST_MSD_STATE msdState;

    /* Logical Unit Number */
    uint8_t logicalUnitNumber;

    /* Control Pipe */
    USB_HOST_PIPE_HANDLE controlPipeHandle;

    /* Bulk in Pipe */
    USB_HOST_PIPE_HANDLE bulkInPipeHandle;

    /* Bulk out Pipe */
    USB_HOST_PIPE_HANDLE bulkOutPipeHandle;

    /* Transfer object */
    USB_HOST_MSD_TRANSFER_OBJ transferObj;

    /* Interface number */
    uint8_t bInterfaceNumber;

    /* Tracks the transfer state */
    USB_HOST_MSD_TRANSFER_STATE transferState;

    /* Tracks the transfer error state */
    USB_HOST_MSD_TRANSFER_ERROR_STATE transferErrorTaskState;

    /* Standard request done flag */
    bool standardRequestDone;

    /* A mutex to control access to the MSD object */
    OSAL_MUTEX_DECLARE(mutexMSDInstanceObject);

    /* Flag is true if control transfer has completed */
    bool controlTransferDone; 
    
    /* Setup packet to use with the control transfer */
    USB_SETUP_PACKET setupPacket;
    
    /* Result of the control transfer */
    USB_HOST_RESULT controlTransferResult;

    /* Control transfer size */
    size_t controlTransferSize;
    
    /* MSD Error Code */
    uintptr_t msdErrorCode;

} USB_HOST_MSD_INSTANCE;


#define USB_HOST_MSD_CBW            0x11
#define USB_HOST_MSD_DATA_IN        0x12
#define USB_HOST_MSD_DATA_OUT       0x13
#define USB_HOST_MSD_CSW            0x14
#define USB_HOST_CLEAR_ENDPOINT     0x15


#define USB_MSD_CSW_STATUS_SUCCESS  0x00
#define USB_MSD_CSW_STATUS_INVALID     -1
#define USB_MSD_CSW_TAG_INVALID        -2
#define  USB_MSD_CSW_SIGNATURE_INVALID -3

/* BOT CSW Status */
#define USB_MSD_CSW_STATUS_GOOD         0x00
#define USB_MSD_CSW_STATUS_FAIL         0x01
#define USB_MSD_CSW_STATUS_PHASE_ERROR  0x02
#define USB_MSD_TRANSFER_SUCESSS 0x00
#define USB_MSD_TRANSFER_FAIL      -1
#define USB_MSD_TRANSPORT_ERROR    -2

// *****************************************************************************
/* USB Host MSD Transfer Flags

  Summary:
    USB Host MSD Function Driver Transfer Flags

  Description:
    These flags are used to indicate status of the pending data while sending
    data to the device by using the USB_HOST_MSD_Transfer() function.

  Remarks:
    The relevance of the specified flag depends on the size of the buffer. Refer
    to the individual flag descriptions for more details.
*/
#define USB_MSD_CBW_FLAG_IN  0x80
#define USB_MSD_CBW_FLAG_OUT  0x00

/* CBW Tag value */
 #define USB_MSD_VALID_CBW_TAG     (uint32_t) 0xDD1331DD

// ****************************************************************************
// ****************************************************************************
// Local Functions
// ****************************************************************************
// ****************************************************************************

// *****************************************************************************
/* Function:
   int_USB_HOST_MSD_InterfaceHandleToMSDInstance
   ( 
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
   )

  Summary:
    This function will return the MSD instance object that is associated with
    this interface.

  Description:
    This function will return the MSD instance object that is associated with
    this interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_MSD_InterfaceHandleToMSDInstance
( 
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
);

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_ReleaseInterface
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    );

  Summary:
    This function will release the device interface. It will close any open
    pipes. It will deinitialize any SCSI instances.

  Description:
    This function will release the device interface. It will close any open
    pipes. It will deinitialize any SCSI instances.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_InterfaceRelease
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
);

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_ControlTransferCallback
    (
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        USB_HOST_REQUEST_HANDLE requestHandle,
        USB_HOST_RESULT result,
        size_t size,
        uintptr_t context
    );

  Summary:
    This function is called when a control transfer completes.

  Description:
    This function is called when a control transfer completes.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_ControlTransferCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
);

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_GetMaxLUNPacketCreate
   ( 
       USB_HOST_MSD_REQUEST * requestObj,
       uint8_t bInterfaceNumber
   )

  Summary:
    This function will create the Get MAX LUN

  Description:
    This function will create the Get MAX LUN

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_GetMaxLUNPacketCreate
(
   USB_SETUP_PACKET * setupPacket,
   uint8_t bInterfaceNumber
);

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_ResetPacketCreate
   ( 
       USB_HOST_MSD_REQUEST * requestObj,
       uint8_t bInterfaceNumber
   )

  Summary:
    This function will create the Reset Packet.

  Description:
    This function will create the Reset Packet.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_ResetPacketCreate
( 
   USB_SETUP_PACKET * setupPacket,
   uint8_t bInterfaceNumber
);

// *****************************************************************************
/* Function:
    void _USB_HOST_MSD_Initialize(void * msdInitData)

  Summary:
    This function is called when the Host Layer is initializing.

  Description:
    This function is called when the Host Layer is initializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_Initialize(void * msdInitData);

// *****************************************************************************
/* Function:
    void _USB_HOST_MSD_Deinitialize(void)

  Summary:
    This function is called when the Host Layer is deinitializing.

  Description:
    This function is called when the Host Layer is deinitializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_Deinitialize(void);

// *****************************************************************************
/* Function:
    void _USB_HOST_MSD_Reinitialize(void)

  Summary:
    This function is called when the Host Layer is reinitializing.

  Description:
    This function is called when the Host Layer is reinitializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_Reinitialize(void * msdInitData);

// *****************************************************************************
/* Function:
    void _USB_HOST_MSD_InterfaceAssign 
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        size_t nInterfaces,
        uint8_t * descriptor
    )

  Summary:
    This function is called when the Host Layer attaches this driver to an
    interface.

  Description:
    This function is called when the Host Layer attaches this driver to an
    interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_InterfaceAssign 
(
    USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    size_t nInterfaces,
    uint8_t * descriptor
);

// *****************************************************************************
/* Function:
    void USB_HOST_MSD_InstanceRelease
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    )

  Summary:
    This function is called when the Host Layer detaches this driver from an
    interface.

  Description:
    This function is called when the Host Layer detaches this driver from an
    interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_InstanceRelease
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
);

// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_MSD_InterfaceEventHandler
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
        USB_HOST_DEVICE_INTERFACE_EVENT event,
        void * eventData,
        uintptr_t context
    )

  Summary:
    This function is called when the Host Layer generates interface level
    events. 

  Description:
    This function is called when the Host Layer generates interface level
    events. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_MSD_InterfaceEventHandler
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_DEVICE_INTERFACE_EVENT event,
    void * eventData,
    uintptr_t context
);

// *****************************************************************************
/* Function:
    void USB_HOST_MSD_InterfaceTasks
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    )

  Summary:
    This function is called by the Host Layer to update the state of this
    driver.

  Description:
    This function is called by the Host Layer to update the state of this
    driver.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_InterfaceTasks
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
);

// *****************************************************************************
/* Function:
   USB_HOST_MSD_RESULT _USB_HOST_MSD_HostResultToMSDResultMap
   (
        USB_HOST_RESULT result
   )

  Summary:
    Maps USB_HOST_RESULT to USB_HOST_MSD_RESULT.

  Description:
    Maps USB_HOST_RESULT to USB_HOST_MSD_RESULT.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_MSD_RESULT _USB_HOST_MSD_HostResultToMSDResultMap
(
    USB_HOST_RESULT result
);

// *****************************************************************************
/* Function:
   void _USB_HOST_MSD_TransferTasks
   (
        uintptr_t msdInstanceIndex,
        USB_HOST_RESULT result,
        size_t size
   );

  Summary:
    This function is called when a transfer event has occurred. This updates the
    transfer state of an ongoing MSD transfer.

  Description:
    This function is called when a transfer event has occurred. This updates the
    transfer state of an ongoing MSD transfer.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_MSD_TransferTasks
(
    uintptr_t msdInstanceIndex,
    USB_HOST_RESULT result,
    size_t size
);

#endif

