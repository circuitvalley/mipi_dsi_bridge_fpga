/*******************************************************************************
  USB Host CDC Client Driver ACM specific functions Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_cdc.c

  Summary:
    USB Host CDC Client Driver ACM Specific Functions Implementation

  Description:
    This file contains the implementation of the CDC Client Driver ACM specific
    API. It should be included in the application if the CDC Host Client Driver
    ACM functionality is desired.
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

#include "usb/usb_host_cdc.h"
#include "usb/usb_host_client_driver.h"
#include "usb/usb_host.h"
#include "usb/usb_cdc.h"
#include "usb/src/usb_host_cdc_local.h"

// ****************************************************************************
// ****************************************************************************
// Local Functions
// ****************************************************************************
// ****************************************************************************

// *****************************************************************************
/* Function:
   void _USB_HOST_CDC_ControlTransferCallback
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

void _USB_HOST_CDC_ControlTransferCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
)
{
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)(context);
    USB_HOST_CDC_CONTROL_TRANSFER_OBJ * controlTransferObj = &cdcInstance->controlTransferObj;
    USB_HOST_CDC_CONTROL_REQUEST_EVENT_DATA controlRequestEventData;;

    /* The control request is complete. The request in requestType is the same
     * as the event that needs to be sent to the application. */

    controlRequestEventData.result = _USB_HOST_CDC_HostResutlToCDCResultMap(result);
    controlRequestEventData.requestHandle = requestHandle;
    controlRequestEventData.length = size;
    if(cdcInstance->eventHandler != NULL)
    {
        cdcInstance->eventHandler((USB_HOST_CDC_HANDLE)(cdcInstance), controlTransferObj->requestType,
                &controlRequestEventData, cdcInstance->context);
    }

    /* Release the control transfer object */
    controlTransferObj->inUse = false;
}


// ****************************************************************************
// ****************************************************************************
// Public Functions
// ****************************************************************************
// ****************************************************************************

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
)
{
    USB_HOST_CDC_RESULT result = USB_HOST_CDC_RESULT_FAILURE;
    USB_HOST_RESULT hostResult;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_CDC_REQUEST_HANDLE * tempRequestHandle, internalRequestHandle;
    USB_SETUP_PACKET * setupPacket;

    if(requestHandle == NULL)
    {
        /* If the provided request handle is NULL, then use a temporary request
         * handle */
        tempRequestHandle = &internalRequestHandle;
    }
    else
    {
        /* Or use the actual request handle */
        tempRequestHandle = requestHandle;
    }

    cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)(handle);

    if(cdcInstance == NULL)
    {
        /* The handle is not valid */
        result = USB_HOST_CDC_RESULT_HANDLE_INVALID;
    }
    else 
    {
        if(lineCoding == NULL)
        {
            /* The input parameter is not valid */
            result = USB_HOST_CDC_RESULT_INVALID_PARAMETER;
        }
        else
        {
            if(!cdcInstance->inUse)
            {
                /* This device is not valid */
                result = USB_HOST_CDC_RESULT_DEVICE_UNKNOWN;
            }
            else
            {
                if((cdcInstance->controlTransferObj.inUse == true))
                {
                    /* The instance is busy */
                    result = USB_HOST_CDC_RESULT_BUSY;
                }
                else
                {
                    if(cdcInstance->state == USB_HOST_CDC_STATE_READY)
                    {
                        cdcInstance->controlTransferObj.inUse = true;
                        cdcInstance->controlTransferObj.requestType = USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE;

                        /* Create the setup packet */
                        setupPacket = &cdcInstance->setupPacket;
                        setupPacket->bmRequestType = 0x21;
                        setupPacket->bRequest = USB_CDC_REQUEST_SET_LINE_CODING;
                        setupPacket->wValue = 0;
                        setupPacket->wIndex = cdcInstance->commInterfaceNumber;
                        setupPacket->wLength = sizeof(USB_CDC_LINE_CODING);

                        /* Schedule the control transfer */
                        hostResult = USB_HOST_DeviceControlTransfer(cdcInstance->controlPipeHandle, tempRequestHandle,
                                setupPacket, lineCoding, _USB_HOST_CDC_ControlTransferCallback, 
                                (uintptr_t)(cdcInstance ));

                        /* Map the host result to CDC result */
                        result = _USB_HOST_CDC_HostResutlToCDCResultMap(hostResult);
                        if(hostResult != USB_HOST_RESULT_SUCCESS)
                        {
                            /* This means the transfer did not go through. We 
                             * should return the control transfer object so
                             * that control transfers can be re-attempted. */
                            
                            cdcInstance->controlTransferObj.inUse = false;
                        }
                    }
                }
            }
        }
    }

    return(result);
}

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_ACM_LineCodingGet
    (
        USB_HOST_CDC_HANDLE handle, 
        USB_HOST_CDC_REQUEST_HANDLE * requestHandle, 
        USB_CDC_LINE_CODING * lineCoding
    );
   
  Summary:
    This function sends a request to the attached device to get its Line Coding.

  Description:
    This function sends a request to the attached device to get its line coding.
    The function schedules a GET LINE CODING control transfer. If successful,
    the requestHandle parameter will contain a valid request handle, else it
    will contain USB_HOST_CDC_REQUEST_HANDLE_INVALID. When completed, the CDC
    client driver will generate a USB_HOST_CDC_EVENT_ACM_GET_LINE_CODING_COMPLETE
    event. 

  Remarks:
    Refer to usb_host_cdc_acm.h for usage information.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_ACM_LineCodingGet
(
    USB_HOST_CDC_HANDLE handle, 
    USB_HOST_CDC_REQUEST_HANDLE * requestHandle, 
    USB_CDC_LINE_CODING * lineCoding
)
{
    USB_HOST_CDC_RESULT result = USB_HOST_CDC_RESULT_FAILURE;
    USB_HOST_RESULT hostResult;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_CDC_REQUEST_HANDLE * tempRequestHandle, internalRequestHandle;
    USB_SETUP_PACKET * setupPacket;

    if(requestHandle == NULL)
    {
        /* If the provided request handle is NULL, then use a temporary request
         * handle */
        tempRequestHandle = &internalRequestHandle;
    }
    else
    {
        /* Or use the actual request handle */
        tempRequestHandle = requestHandle;
    }

    cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)(handle);

    if(cdcInstance == NULL)
    {
        /* The handle is not valid */
        result = USB_HOST_CDC_RESULT_HANDLE_INVALID;
    }
    else 
    {
        if(lineCoding == NULL)
        {
            /* The input parameter is not valid */
            result = USB_HOST_CDC_RESULT_INVALID_PARAMETER;
        }
        else
        {
            if(!cdcInstance->inUse)
            {
                /* This device is not valid */
                result = USB_HOST_CDC_RESULT_DEVICE_UNKNOWN;
            }
            else
            {
                if((cdcInstance->controlTransferObj.inUse == true))
                {
                    /* The instance is busy */
                    result = USB_HOST_CDC_RESULT_BUSY;
                }
                else
                {
                    if(cdcInstance->state == USB_HOST_CDC_STATE_READY)
                    {
                        cdcInstance->controlTransferObj.inUse = true;
                        cdcInstance->controlTransferObj.requestType = USB_HOST_CDC_EVENT_ACM_GET_LINE_CODING_COMPLETE;

                        /* Create the setup packet */
                        setupPacket = &cdcInstance->setupPacket;
                        setupPacket->bmRequestType = 0xA1;
                        setupPacket->bRequest = USB_CDC_REQUEST_GET_LINE_CODING;
                        setupPacket->wValue = 0;
                        setupPacket->wIndex = cdcInstance->commInterfaceNumber;
                        setupPacket->wLength = sizeof(USB_CDC_LINE_CODING);

                        /* Schedule the control transfer */
                        hostResult = USB_HOST_DeviceControlTransfer(cdcInstance->controlPipeHandle, tempRequestHandle,
                                setupPacket, lineCoding, _USB_HOST_CDC_ControlTransferCallback, 
                                (uintptr_t)(cdcInstance ));

                        /* Map the host result to CDC result */
                        result = _USB_HOST_CDC_HostResutlToCDCResultMap(hostResult);
                        if(hostResult != USB_HOST_RESULT_SUCCESS)
                        {
                            /* This means the transfer did not go through. We 
                             * should return the control transfer object so
                             * that control transfers can be re-attempted. */
                            
                            cdcInstance->controlTransferObj.inUse = false;
                        }
                    }
                }
            }
        }
    }

    return(result);
}

// ****************************************************************************
/* Function:
    USB_HOST_CDC_RESULT USB_HOST_CDC_ACM_BreakSend
    (
        USB_HOST_CDC_HANDLE handle,
        USB_HOST_CDC_REQUEST_HANDLE * requestHandle,
        uint16_t breakDuration
    );
   
  Summary:
    This function sends a request to the attached device to update its break
    duration.

  Description:
    This function sends a request to the attached to update its break
    duration. The function schedules a SEND BREAK control transfer. If
    successful, the transferHandle parameter will contain a valid request
    handle, else it will contain USB_HOST_CDC_REQUEST_HANDLE_INVALID. When
    completed, the CDC client driver will generate a
    USB_HOST_CDC_EVENT_ACM_SEND_BREAK_COMPLETE event. 

  Remarks:
    Refer to usb_host_cdc_acm.h for usage information.
*/

USB_HOST_CDC_RESULT USB_HOST_CDC_ACM_BreakSend
(
    USB_HOST_CDC_HANDLE handle,
    USB_HOST_CDC_REQUEST_HANDLE * requestHandle,
    uint16_t breakDuration
)
{
    USB_HOST_CDC_RESULT result = USB_HOST_CDC_RESULT_FAILURE;
    USB_HOST_RESULT hostResult;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_CDC_REQUEST_HANDLE * tempRequestHandle, internalRequestHandle;
    USB_SETUP_PACKET * setupPacket;

    if(requestHandle == NULL)
    {
        /* If the provided request handle is NULL, then use a temporary request
         * handle */
        tempRequestHandle = &internalRequestHandle;
    }
    else
    {
        /* Or use the actual request handle */
        tempRequestHandle = requestHandle;
    }

    cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)(handle);

    if(cdcInstance == NULL)
    {
        /* The handle is not valid */
        result = USB_HOST_CDC_RESULT_HANDLE_INVALID;
    }
    else 
    {
        if(!cdcInstance->inUse)
        {
            /* This device is not valid */
            result = USB_HOST_CDC_RESULT_DEVICE_UNKNOWN;
        }
        else
        {
            if((cdcInstance->controlTransferObj.inUse == true))
            {
                /* The instance is busy */
                result = USB_HOST_CDC_RESULT_BUSY;
            }
            else
            {
                if(cdcInstance->state == USB_HOST_CDC_STATE_READY)
                {
                    cdcInstance->controlTransferObj.inUse = true;
                    cdcInstance->controlTransferObj.requestType = USB_HOST_CDC_EVENT_ACM_SEND_BREAK_COMPLETE;

                    /* Create the setup packet */
                    setupPacket = &cdcInstance->setupPacket;
                    setupPacket->bmRequestType = 0x21;
                    setupPacket->bRequest = USB_CDC_REQUEST_SEND_BREAK;
                    setupPacket->wValue = breakDuration ;
                    setupPacket->wIndex = cdcInstance->commInterfaceNumber;
                    setupPacket->wLength = 0;

                    /* Schedule the control transfer */
                    hostResult = USB_HOST_DeviceControlTransfer(cdcInstance->controlPipeHandle, tempRequestHandle,
                            setupPacket, NULL, _USB_HOST_CDC_ControlTransferCallback, 
                            (uintptr_t)(cdcInstance ));

                    /* Map the host result to CDC result */
                    result = _USB_HOST_CDC_HostResutlToCDCResultMap(hostResult);
                    if(hostResult != USB_HOST_RESULT_SUCCESS)
                    {
                        /* If the control transfer was not successful, then
                         * release the control transfer object. */
                        cdcInstance->controlTransferObj.inUse = false;
                    }
                }
            }
        }

    }

    return(result);
}

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
)
{
    USB_HOST_CDC_RESULT result = USB_HOST_CDC_RESULT_FAILURE;
    USB_HOST_RESULT hostResult;
    USB_HOST_CDC_INSTANCE_OBJ * cdcInstance;
    USB_HOST_CDC_REQUEST_HANDLE * tempRequestHandle, internalRequestHandle;
    USB_SETUP_PACKET * setupPacket;

    if(requestHandle == NULL)
    {
        /* If the provided request handle is NULL, then use a temporary request
         * handle */
        tempRequestHandle = &internalRequestHandle;
    }
    else
    {
        /* Or use the actual request handle */
        tempRequestHandle = requestHandle;
    }

    cdcInstance = (USB_HOST_CDC_INSTANCE_OBJ *)(handle);

    if(cdcInstance == NULL)
    {
        /* The handle is not valid */
        result = USB_HOST_CDC_RESULT_HANDLE_INVALID;
    }
    else 
    {
        if(controlLineState == NULL)
        {
            result = USB_HOST_CDC_RESULT_INVALID_PARAMETER;
        }
        else
        {
            if(!cdcInstance->inUse)
            {
                /* This device is not valid */
                result = USB_HOST_CDC_RESULT_DEVICE_UNKNOWN;
            }
            else
            {
                if((cdcInstance->controlTransferObj.inUse == true))
                {
                    /* The instance is busy */
                    result = USB_HOST_CDC_RESULT_BUSY;
                }
                else
                {
                    if(cdcInstance->state == USB_HOST_CDC_STATE_READY)
                    {
                        cdcInstance->controlTransferObj.inUse = true;
                        cdcInstance->controlTransferObj.requestType = USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE;

                        /* Create the setup packet */
                        setupPacket = &cdcInstance->setupPacket;
                        setupPacket->bmRequestType = 0x21;
                        setupPacket->bRequest = USB_CDC_REQUEST_SET_CONTROL_LINE_STATE;
                        setupPacket->wValue =  (( *(uint8_t *) controlLineState ) << 8 ); ;
                        setupPacket->wIndex = cdcInstance->commInterfaceNumber;
                        setupPacket->wLength = 0;

                        /* Schedule the control transfer */
                        hostResult = USB_HOST_DeviceControlTransfer(cdcInstance->controlPipeHandle, tempRequestHandle,
                                setupPacket, NULL, _USB_HOST_CDC_ControlTransferCallback, 
                                (uintptr_t)(cdcInstance ));

                        /* Map the host result to CDC result */
                        result = _USB_HOST_CDC_HostResutlToCDCResultMap(hostResult);
                        if(hostResult != USB_HOST_RESULT_SUCCESS)
                        {
                            /* This means the transfer did not go through. We 
                             * should return the control transfer object so
                             * that control transfers can be re-attempted. */
                            
                            cdcInstance->controlTransferObj.inUse = false;
                        }
                    }
                }
            }
        }
    }

    return(result);
}


