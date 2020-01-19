/* ******************************************************************************
  USB Host MSD Class Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_msd.h

  Summary:
    USB Host MSD Class Driver Interface Header

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface to the USB Host MSD
    Class Driver.
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
#ifndef _USB_HOST_MSD_H_
#define _USB_HOST_MSD_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host.h"
#include "usb/usb_host_client_driver.h"
#include "usb/usb_msd.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB HOST MSD LUN Handle

  Summary:
    USB HOST MSD LUN Handle

  Description:
    This type defines a MSD LUN Handle. This handle is used by SCSI driver to 
    identify the LUN.

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_MSD_LUN_HANDLE;

// *****************************************************************************
/* USB HOST MSD LUN Handle Invalid
 
  Summary:
    USB HOST MSD LUN Handle Invalid

  Description:
    This value defines an invalid LUN Handle.

  Remarks:
    None.
*/

#define USB_HOST_MSD_LUN_HANDLE_INVALID /*DOM-IGNORE-BEGIN*/((USB_HOST_MSD_LUN_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* USB HOST MSD Client Driver Interface
 
  Summary:
    USB HOST MSD Client Driver Interface

  Description:
    This macro should be used by the application in TPL table while adding
    support for the USB MSD Host Client Driver.

  Remarks:
    None.
*/

/*DOM-IGNORE-BEGIN*/extern USB_HOST_CLIENT_DRIVER gUSBHostMSDClientDriver; /*DOM-IGNORE-END*/
#define USB_HOST_MSD_INTERFACE  /*DOM-IGNORE-BEGIN*/&gUSBHostMSDClientDriver /*DOM-IGNORE-END*/

// *****************************************************************************
/* USB HOST MSD Transfer Handle

  Summary:
    USB HOST MSD Transfer Handle

  Description:
    This type defines a USB Host MSD Transfer Handle.

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_MSD_TRANSFER_HANDLE;

// *****************************************************************************
/* USB HOST MSD Transfer Handle Invalid
 
  Summary:
    USB HOST MSD Transfer Handle Invalid

  Description:
    This value defines an invalid Transfer Handle.

  Remarks:
    None.
*/

#define USB_HOST_MSD_TRANSFER_HANDLE_INVALID /*DOM-IGNORE-BEGIN*/((USB_HOST_MSD_TRANSFER_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* USB HOST MSD Result

  Summary:
    USB HOST MSD Result

  Description:
    This enumeration defines the possible return values of different USB HOST
    MSD Client driver function call. Refer to the specific function
    documentation for details on the return values.
    
  Remarks:
    None.
*/

typedef enum
{

    /* MSD Command result was success. The command issued to the MSD device
     * passed. */
    USB_HOST_MSD_RESULT_COMMAND_PASSED = 0,

    /* MSD Command failed. The command issued to the MSD device failed. The
     * device BOT state machine is in sync with the host. The data residue
     * length is valid. */
    USB_HOST_MSD_RESULT_COMMAND_FAILED = 1,

    /* MSD Command failed with phase error. The command issued to the MSD device
     * has failed. The failure reason is unknown. The MSD Host Client driver has
     * reset the device BOT state machine. */
    USB_HOST_MSD_RESULT_COMMAND_PHASE_ERROR = 2,

    /* The operation was successful */
    USB_HOST_MSD_RESULT_SUCCESS,
    
    /* An unknown failure has occurred. */
    USB_HOST_MSD_RESULT_FAILURE,

    /* The request cannot be accepted at this time */
    USB_HOST_MSD_RESULT_BUSY,

    /* The specified LUN is not valid */
    USB_HOST_MSD_RESULT_LUN_HANDLE_INVALID,

    /* The MSD request was stalled */
    USB_HOST_MSD_RESULT_COMMAND_STALLED

} USB_HOST_MSD_RESULT;

// *****************************************************************************
/* USB Host MSD Error Codes.

  Summary:
    USB Host MSD Error Codes.

  Description:
    This enumeration defines the codes that the MSD Client Driver returns for
    possible errors that lead to the device being placed in an error state. The
    MSD client driver will not operate on a device which is in an error state.
    The error are returned in the USB_HOST_MSD_ErrorCallback function.
    
  Remarks:
    None.
*/

typedef enum
{
    /* This error occurs when the number of MSD instances defined via
     * USB_HOST_MSD_INSTANCES_NUMBER (in system_config.h) is insufficient. For
     * example, this error would occur if the value of
     * USB_HOST_MSD_INSTANCES_NUMBER is 2, two MSC devices are already connected
     * and third MSC device is connected to the host. The object identifier in
     * this case will be the USB_HOST_DEVICE_OBJ_HANDLE value.  */
    USB_HOST_MSD_ERROR_CODE_INSUFFICIENT_INSTANCES = 1,

    /* This error occurs when the driver descriptor parser could not find a Bulk
     * IN endpoint in the interface descriptor. The object identifier in this
     * case will be the USB_HOST_DEVICE_OBJ_HANDLE value. */
    USB_HOST_MSD_ERROR_CODE_NOT_FOUND_BULK_IN_ENDPOINT,

    /* This error occurs when the driver descriptor parser could not find a Bulk
     * OUT endpoint in the interface descriptor. The object identifier in this
     * case will be USB_HOST_DEVICE_OBJ_HANDLE value. */
    USB_HOST_MSD_ERROR_CODE_NOT_FOUND_BULK_OUT_ENDPOINT,

    /* This error occurs when the driver could not open a Bulk pipe. This
     * typically happens either due to a host layer error or due to insufficient
     * number of pipes (which is configured via USB_HOST_PIPES_NUMBER). The
     * object idenfier in this case will be USB_HOST_DEVICE_OBJ_HANDLE value. */
    USB_HOST_MSD_ERROR_CODE_FAILED_PIPE_OPEN,

    /* This error occurs when the Get Max LUN request issued by the driver fails
     * for any reason. The object identifier in this case will be the MSC device
     * instance index. */
    USB_HOST_MSD_ERROR_CODE_FAILED_GET_MAX_LUN,

    /* This error occurs when any stage of the BOT has failed due to bus error
     * or an unknown failure. The object identifier in this case will be the MSC
     * device instance index. */
    USB_HOST_MSD_ERROR_CODE_FAILED_BOT_TRANSFER,

    /* This error occurs when the MSD Reset Recovery procedure has failed. A MSC
     * device should not fail a MSD Reset Recovery procedure. The object
     * identifier in this case will be the device instance index. */
    USB_HOST_MSD_ERROR_CODE_FAILED_RESET_RECOVERY,

    /* This error code indicates a condtion where the CBW stage of the BOT was
     * stalled and the driver is about to launch MSD reset recovery. The
     * identifier in this case if the MSC Device instance index. This code is
     * generated from an interrupt context. The driver may continue to function
     * normally post this condition. */
    USB_HOST_MSD_ERROR_CODE_CBW_STALL_RESET_RECOVERY,

    /* This error code indicates a condition where the BOT transfer could not be
     * initiated because a transfer is already in progress. The identifier in
     * this case is the MSC Device Instance Index. The driver may continue to
     * function normall post this condition. This condition may occur several
     * times. */
    USB_HOST_MSD_ERROR_CODE_TRANSFER_BUSY,

    /* This error code indicates a condition where the BOT transfer failed due a
     * phase error in the CSW stage of the BOT. The identifier in this case if
     * the MSC Device instance index. This code is generated from an interrupt
     * context. The driver may continue to function normally post this
     * condition. */
    USB_HOST_MSD_ERROR_CODE_CSW_PHASE_ERROR,

    /* This error code indicates that a condition where an unknown error has
     * occured during the CSW stage of the BOT.  The identifier in this case if
     * the MSC Device instance index. This code is generated from an interrupt
     * context. The driver may continue to function normally post this
     * condition. */
    USB_HOST_MSD_ERROR_CODE_CSW_UNKNOWN_ERROR

} USB_HOST_MSD_ERROR_CODE;

// *****************************************************************************
/* USB HOST Transfer Direction

  Summary:
    USB HOST MSD Transfer Direction.

  Description:
    This enumeration specifies the direction of the data stage.
    
  Remarks:
    None.
*/

typedef enum
{
    /* Data moves from host to device */
    USB_HOST_MSD_TRANSFER_DIRECTION_HOST_TO_DEVICE = 0x00,

    /* Data moves from device to host */
    USB_HOST_MSD_TRANSFER_DIRECTION_DEVICE_TO_HOST = 0x80

} USB_HOST_MSD_TRANSFER_DIRECTION;

// *****************************************************************************
/* USB HOST MSD Transfer Complete Callback

  Summary:
    USB HOST MSD Transfer Complete Callback

  Description:
    This type defines the type of the callback function that the application
    must register in the USB_HOST_MSD_Transfer function to receive notification
    when a transfer has completed. The callback function will be called with the
    following parameters.

    lunHandle - The handle to the LUN from where this notification originated.

    transferHandle - the handle to the MSD transfer.

    result - result of the transfer.

    size - of the transfer.

    context - context that specified when this transfer was scheduled.
    
  Remarks:
    None.
*/

typedef void (*USB_HOST_MSD_TRANSFER_CALLBACK)
(
    USB_HOST_MSD_LUN_HANDLE lunHandle,
    USB_HOST_MSD_TRANSFER_HANDLE transferHandle,
    USB_HOST_MSD_RESULT result,
    size_t size,
    uintptr_t context
);

// *****************************************************************************
// *****************************************************************************
// Section: MSD Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
   USB_HOST_MSD_RESULT USB_HOST_MSD_Transfer
   (
       uint8_t * cdb,
       uint8_t cdbLength,
       void * data,
       size_t size,
       USB_HOST_MSD_TRANSFER_CALLBACK callback,
       uintptr_t context
   )

  Summary:
    This function schedules a MSD BOT transfer.

  Description:
    This function schedules a MSD BOT transfer. The command to be executed is
    specified in the cdb. This should be pointer to a 16 byte command descriptor
    block. The actual length of the command is specified by cdbLength. If there
    is data to be transferred, the pointer to the buffer is specified by data.
    The size of the buffer is specified in size. When the transfer completes,
    the callback function will be called. The context will be returned in the
    callback function.

  Preconditions:
    None.

  Parameters:
    cdb - pointer to the command to be executted. Should be a pointer to a 16
    byte array. Unused bytes should be zero-padded.

    cdbLength - Actual size of the command.

    data - pointer to the data buffer if a data stage is involved.

    size - size of the data buffer.

    callback - callback function to called when the transfer has completed.

    transferDirection - specifies the direction of the MSD transfer.

    context - caller defined context that is returned in the callback function.

  Returns:
    USB_HOST_MSD_RESULT_FAILURE - An unknown failure occurred.
    USB_HOST_MSD_RESULT_BUSY - The transfer cannot be scheduled right now. The
    caller should retry.
    USB_HOST_MSD_RESULT_LUN_HANDLE_INVALID - This LUN does not exist in the
    system.
    USB_HOST_MSD_RESULT_SUCCESS - The transfer request was scheduled.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_MSD_RESULT USB_HOST_MSD_Transfer
(
    USB_HOST_MSD_LUN_HANDLE lunHandle,
    uint8_t * cdb,
    uint8_t cdbLength,
    void * data,
    size_t size,
    USB_HOST_MSD_TRANSFER_DIRECTION transferDirection,
    USB_HOST_MSD_TRANSFER_CALLBACK callback,
    uintptr_t context
);

// *****************************************************************************
/* Function:
    void USB_HOST_MSD_TransferErrorTasks
    (
        USB_HOST_MSD_LUN_HANDLE lunHandle,
    );

  Summary:
    This function maintains the MSD transfer error handling state machine.

  Description:
    This function maintains the MSD transfer error handling state machine. This
    function should be called periodically after the USB_HOST_MSD_Transfer
    function has been called to schedule a transfer. The function should be
    called periodically atleast till the transfer completion event has been
    received. Calling this function while a BOT transfer is in progress allows
    the MSD Host Client driver to perform BOT error handling in a non-blocking
    manner.  

    Calling this function when there is no BOT transfer in progress will not
    have any effect. In case of BOT error handling, calling this function will
    eventually result in a BOT transfer event. It is not necessary to call this
    function after this event has occurred (till the next BOT transfer has been
    scheduled).

  Preconditions:
    The lunHandle should be valid.

  Parameters:
    lunHandle - handle to valid LUN.

  Returns:
    None.

  Remarks:
    While running in an RTOS application, this function should be called in the
    same thread that requested the BOT Transfer and operating the logical unit.
*/

void USB_HOST_MSD_TransferErrorTasks
(
    USB_HOST_MSD_LUN_HANDLE lunHandle
);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif

//DOM-IGNORE-END

#endif



