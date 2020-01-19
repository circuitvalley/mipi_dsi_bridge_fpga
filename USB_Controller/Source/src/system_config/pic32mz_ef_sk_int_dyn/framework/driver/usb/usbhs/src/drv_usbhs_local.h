/*******************************************************************************
  USB Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhs_local.h

  Summary:
    USB driver local declarations and definitions

  Description:
    This file contains the USB driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_USBHS_LOCAL_H
#define _DRV_USBHS_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "system_config.h"
#include "system/debug/sys_debug.h"
#include "system/tmr/sys_tmr.h"
#include "driver/usb/usbhs/drv_usbhs.h"
#include "driver/usb/usbhs/src/drv_usbhs_variant_mapping.h"
#include "osal/osal.h"


#define INDEX           0xE
#define INTRTXE         0x6
#define INTRTX          0x2
#define INTRRX          0x4
#define INTRRXE         0x8
#define INTRUSBE        0xB
#define INTRUSB         0xA
#define RXTYPE          0x1C
#define TXTYPE          0x1A
#define RXINTERVAL      0x1D
#define TXINTERVAL      0x1B
#define FADDR           0x00
#define POWER           0x01
#define DEVCTL          0x60
#define SOFT_RST        0x7F

#define TXMAXP          0x10
#define CSR0L           0x12
#define CSR0H           0x13
#define TXCSRL          0x12
#define TXCSRH          0x13
#define RXMAXP          0x14
#define RXCSRL          0x16
#define RXCSRH          0x17
#define COUNT0          0x18
#define RXCOUNT         0x18
#define RXFIFOADDR      0x66
#define TXFIFOADDR      0x64
#define TXFIFOSZ        0x62
#define RXFIFOSZ        0x63

#define FIFO0           0x20
#define TXDPKTBUFDIS    0x342
#define RXDPKTBUFDIS    0x340

#define DRV_USBHS_HOST_IRP_PER_FRAME_NUMBER 	5
#define DRV_USBHS_HOST_MAXIMUM_ENDPOINTS_NUMBER           8
#define DRV_USBHS_MAX_DMA_CHANNELS                8

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************


/***************************************************
 * This is an intermediate flag that is set by
 * the driver to indicate that a ZLP should be sent
 ***************************************************/
#define USB_DEVICE_IRP_FLAG_SEND_ZLP 0x80

/***************************************************
 * This object is used by the driver as IRP place
 * holder along with queueing feature.
 ***************************************************/
typedef struct _USB_DEVICE_IRP_LOCAL 
{
    /* Pointer to the data buffer */
    void * data;

    /* Size of the data buffer */
    unsigned int size;

    /* Status of the IRP */
    USB_DEVICE_IRP_STATUS status;

    /* IRP Callback. If this is NULL,
     * then there is no callback generated */
    void (*callback)(struct _USB_DEVICE_IRP * irp);

    /* Request specific flags */
    USB_DEVICE_IRP_FLAG flags;

    /* User data */
    uintptr_t userData;

    /* This points to the next IRP in the queue */
    struct _USB_DEVICE_IRP_LOCAL * next;

    /* This points to the previous IRP in the queue */
    struct _USB_DEVICE_IRP_LOCAL * previous;

    /* Pending bytes in the IRP */
    uint32_t nPendingBytes;
}
USB_DEVICE_IRP_LOCAL;


/************************************************
 * Endpoint state enumeration.
 ************************************************/
typedef enum
{
    DRV_USBHS_DEVICE_ENDPOINT_STATE_ENABLED = 0x1,
    DRV_USBHS_DEVICE_ENDPOINT_STATE_STALLED = 0x2
}
DRV_USBHS_DEVICE_ENDPOINT_STATE;

/************************************************
 * Endpoint data strucutre. This data structure
 * holds the IRP queue and other flags associated
 * with functioning of the endpoint.
 ************************************************/

typedef struct
{
    /* This is the IRP queue for 
     * the endpoint */
    USB_DEVICE_IRP_LOCAL * irpQueue;

    /* Max packet size for the endpoint */
    uint16_t maxPacketSize;

    /* Endpoint type */
    USB_TRANSFER_TYPE endpointType;

    /* Endpoint state bitmap */
    DRV_USBHS_DEVICE_ENDPOINT_STATE endpointState;

}
DRV_USBHS_DEVICE_ENDPOINT_OBJ;

/*********************************************
 * These IRP states are used internally by the
 * HCD to track completion of a host IRP. This
 * is not the same as the public IRP status
 *********************************************/
typedef enum
{   
    DRV_USBHS_HOST_IRP_STATE_SETUP_STAGE,
    DRV_USBHS_HOST_IRP_STATE_SETUP_TOKEN_SENT,
    DRV_USBHS_HOST_IRP_STATE_DATA_STAGE,
    DRV_USBHS_HOST_IRP_STATE_DATA_STAGE_SENT,
    DRV_USBHS_HOST_IRP_STATE_HANDSHAKE,
    DRV_USBHS_HOST_IRP_STATE_HANDSHAKE_SENT,
    DRV_USBHS_HOST_IRP_STATE_COMPLETE,
    DRV_USBHS_HOST_IRP_STATE_ABORTED,
    DRV_USBHS_HOST_IRP_STATE_PROCESSING
}
DRV_USBHS_HOST_IRP_STATE; 

/*********************************************
 * This is the local USB Host IRP object
 ********************************************/
typedef struct _USB_HOST_IRP_LOCAL
{
    /* Points to the 8 byte setup command
     * packet in case this is a IRP is 
     * scheduled on a CONTROL pipe. Should
     * be NULL otherwise */
    void * setup;

    /* Pointer to data buffer */
    void * data;
    
    /* Size of the data buffer */
    unsigned int size;
    
    /* Status of the IRP */ 
    USB_HOST_IRP_STATUS status;

    /* Request specific flags */
    USB_HOST_IRP_FLAG flags;

    /* User data */
    uintptr_t userData;

    /* Pointer to function to be called
     * when IRP is terminated. Can be 
     * NULL, in which case the function
     * will not be called. */
    void (*callback)(struct _USB_HOST_IRP * irp);

    /****************************************
     * These members of the IRP should not be
     * modified by client
     ****************************************/
    DRV_USBHS_HOST_IRP_STATE tempState;
    uint32_t completedBytes;
    struct _USB_HOST_IRP_LOCAL * next;
    struct _USB_HOST_IRP_LOCAL * previous;
    DRV_USBHS_HOST_PIPE_HANDLE  pipe;
}
USB_HOST_IRP_LOCAL;

/************************************************
 * This is the Host Pipe Object.
 ************************************************/
typedef struct _DRV_USBHS_HOST_PIPE_OBJ
{
    /* This pipe object is in use */
    bool inUse;

    /* The device address */
    uint8_t deviceAddress;

    /* Interval in case this
     * is a Isochronous or
     * an interrupt pipe */
    uint8_t bInterval;

    uint8_t noOfSlots;
    unsigned int startingOffset;

    /* Client that owns this pipe */
    DRV_HANDLE hClient;
    
    /* USB endpoint and direction */
    USB_ENDPOINT endpointAndDirection;
    
    /* USB Endpoint type */
    USB_TRANSFER_TYPE pipeType;

    /* The IRP queue on this pipe */
    USB_HOST_IRP_LOCAL * irpQueueHead;

    /* The NAK counter for the IRP
     * being served on the pipe */
    
    uint32_t nakCounter;

    /* Pipe endpoint size*/

    unsigned int endpointSize;

    /* The next and previous pipe */
    struct _DRV_USBHS_HOST_PIPE_OBJ * next;
    struct _DRV_USBHS_HOST_PIPE_OBJ * previous;

    /* Interval counter */

    uint8_t intervalCounter;

    /* Pipe Speed */
    USB_SPEED speed;

    /* Hub Address */
    uint8_t hubAddress;

    /* Hub Port*/
    uint8_t hubPort;

    /* Host endpoint*/
    uint8_t hostEndpoint;
}
DRV_USBHS_HOST_PIPE_OBJ;

/*********************************************
 * Host Transfer Group. This data structures
 * contains all pipes belonging to one tranfer
 * type.
 *********************************************/

typedef struct _DRV_USBHS_HOST_TRANSFER_GROUP
{
    /* The first pipe in this transfer 
     * group */
    DRV_USBHS_HOST_PIPE_OBJ * pipe;
    
    /* The current pipe being serviced
     * in this transfer group */
    DRV_USBHS_HOST_PIPE_OBJ * currentPipe;
    
    /* The current IRP being serviced
     * in the pipe */
    void * currentIRP;

    /* Total number of pipes in this
     * transfer group */
    int nPipes;
}
DRV_USBHS_HOST_TRANSFER_GROUP;

/********************************************
 * This enumeration list the possible status
 * valus of a token once it has completed.
 ********************************************/

typedef enum
{
    USB_TRANSACTION_ACK             = 0x2,
    USB_TRANSACTION_NAK             = 0xA,
    USB_TRANSACTION_STALL           = 0xE,
    USB_TRANSACTION_DATA0           = 0x3,
    USB_TRANSACTION_DATA1           = 0xB,
    USB_TRANSACTION_BUS_TIME_OUT    = 0x0,
    USB_TRANSACTION_DATA_ERROR      = 0xF

}
DRV_USBHS_TRANSACTION_RESULT;

typedef enum
{
    DRV_USBHS_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST,
    DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_IRP_FROM_CLIENT,
    DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_IRP_FROM_CLIENT,
    DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_STAGE_FROM_HOST,
    DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT,
    DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT,
    DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST,
    DRV_USBHS_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS,
    DRV_USBHS_DEVICE_EP0_STATE_RX_DATA_STAGE_IN_PROGRESS,
    DRV_USBHS_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_STAGE_FROM_HOST
}
DRV_USBHS_DEVICE_EP0_STATE;


/**********************************************
 * Host Endpoint Object.
 *********************************************/

typedef struct
{
    /* Indicates this endpoint is in use */
    bool inUse;
    DRV_USBHS_HOST_PIPE_OBJ * pipe;

}_DRV_USBHS_HOST_ENDPOINT;


typedef struct
{
    /* A combination of two structures for
     * each direction. */

    _DRV_USBHS_HOST_ENDPOINT endpoints[2];

}DRV_USBHS_HOST_ENDPOINT_OBJ;


/*********************************************
 * Driver NON ISR Tasks routine states.
 *********************************************/

typedef enum
{
    /* Driver is about to start the initialization delay */
    DRV_USBHS_TASK_STATE_STARTING_DELAY = 1,

    /* Driver has started the delay and is waiting for complete */
    DRV_USBHS_TASK_STATE_WAITING_FOR_DELAY_COMPLETE,

    /* Driver has complete initialization and can be opened */
    DRV_USBHS_TASK_STATE_RUNNING

} DRV_USBHS_TASK_STATE;

typedef struct _DRV_USBHS_DMA_POOL_STRUCT
{
    bool inUse;
    bool endpointDir;
    uint8_t iEndpoint;
    unsigned int count;

} DRV_USBHS_DMA_POOL;

/*********************************************
 * Host Mode Device Attach Detach State
 ********************************************/
typedef enum
{
    /* No device is attached */
    DRV_USBHS_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH = 0,
            
    /* Restarting the host session. */
    DRV_USBHS_HOST_ATTACH_STATE_RESTART_SESSION,

    /* Waiting for debounce delay */
    DRV_USBHS_HOST_ATTACH_STATE_DEBOUNCING,

    /* Debouncing is complete. Device is attached */
    DRV_USBHS_HOST_ATTACH_STATE_READY,

} DRV_USBHS_HOST_ATTACH_STATE;

/*********************************************
 * Host Mode Device Reset state
 *********************************************/
typedef enum
{
    /* No Reset in progress */
    DRV_USBHS_HOST_RESET_STATE_NO_RESET = 0,

    /* Start the reset signalling */
    DRV_USBHS_HOST_RESET_STATE_START,

    /* Check if reset duration is done and de assert reset */
    DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_COMPLETE,
    
    /* Wait for Suspend to be complete post Reset */
    DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_SUSPEND_COMPLETE,
    
    /* Wait for Resume to be complete */
    DRV_USBHS_HOST_RESET_STATE_WAIT_FOR_RESUME_COMPLETE

} DRV_USBHS_HOST_RESET_STATE;

/**************************************
 * Root Hub parameters
 **************************************/

typedef struct
{
    /* Pointer to the port over current detect function */
    DRV_USBHS_ROOT_HUB_PORT_OVER_CURRENT_DETECT portOverCurrentDetect;

    /* Pointer to the port indication function */
    DRV_USBHS_ROOT_HUB_PORT_INDICATION portIndication;

    /* Pointer to the port enable function */
    DRV_USBHS_ROOT_HUB_PORT_POWER_ENABLE portPowerEnable;

    /* Total current available */
    uint32_t rootHubAvailableCurrent;

} DRV_USBHS_HOST_ROOT_HUB_INFO;

/***********************************************
 * Driver object structure. One object per
 * hardware instance
 **********************************************/

typedef struct _DRV_USBHS_OBJ_STRUCT
{
    /* Indicates this object is in use */
    bool    inUse;

    /* Indicates that the client has opened the instance */
    bool isOpened;

    DRV_USBHS_DMA_POOL gDrvUSBDMAPool[DRV_USBHS_MAX_DMA_CHANNELS + 1];

    /* Status of this driver instance */
    SYS_STATUS status;     

    /* Contains the consumed FIFO size */
    unsigned int consumedFIFOSize;

    /* Contains the desired operation speed */
    USB_SPEED operationSpeed;

    /* Contains the EPO state */
    DRV_USBHS_DEVICE_EP0_STATE endpoint0State;

    /* The USB peripheral associated with 
     * the object */
     _DRV_USBHS_FOR_DYNAMIC(USBHS_MODULE_ID, usbID);

    /* Current operating mode of the driver */
    DRV_USBHS_OPMODES operationMode;

    /* Interrupt source for USB module */
    _DRV_USBHS_FOR_DYNAMIC(INT_SOURCE, interruptSource);

    /* Interrupt source for USB DMA module */
    _DRV_USBHS_FOR_DYNAMIC(INT_SOURCE, interruptSourceUSBDma);

    /* Mutex create function place holder*/
    OSAL_MUTEX_DECLARE (mutexID);

    /* Pointer to the endpoint table */
    _DRV_USBHS_FOR_DYNAMIC(DRV_USBHS_DEVICE_ENDPOINT_OBJ, *endpointTable);

    /* Pointer to the endpoint table */
    _DRV_USBHS_FOR_HOST(DRV_USBHS_HOST_ENDPOINT_OBJ, hostEndpointTable[DRV_USBHS_HOST_MAXIMUM_ENDPOINTS_NUMBER]);

    /* The object is current in an interrupt context */
    bool isInInterruptContext;

    /* The object is current in an USB DMA interrupt context */
    bool isInInterruptContextUSBDMA;

    /* Maintains the timer count value for host */
    _DRV_USBHS_FOR_HOST(uint32_t, timerCount);

    /* Root Hub Port 0 attached device speed in host mode
     * In device mode, the speed at which the device attached */
    USB_SPEED deviceSpeed;

    /* Transfer Groups */
    _DRV_USBHS_FOR_HOST(DRV_USBHS_HOST_TRANSFER_GROUP, controlTransferGroup);

    /* Tracks if the current control transfer is a zero length */
    bool isZeroLengthControlTransfer;

    /* Non ISR Task Routine state */
    DRV_USBHS_TASK_STATE state;

    /* System Timer Service delay handle */
    SYS_TMR_HANDLE timerHandle;

    /* Client data that will be returned at callback */
    uintptr_t  hClientArg;

    /* Call back function for this client */
    DRV_USB_EVENT_CALLBACK  pEventCallBack;

    /* Current VBUS level when running in device mode */
    USBHS_VBUS_LEVEL vbusLevel;

    /* Sent session invalid event to the client */
    bool sessionInvalidEventSent;

    /* FIFO allocation table */
    _DRV_USBHS_FOR_HOST(unsigned int, gDrvUSBFifoTable[5]);

    /* FIFO allocation table secondary */
    _DRV_USBHS_FOR_HOST(unsigned int, gDrvUSBFifoTableSecondary[5]);

    /* Attach state of the device */
    DRV_USBHS_HOST_ATTACH_STATE attachState;
    
    /* True if device is attached */
    bool deviceAttached;
    
    /* Flag to understand if timer has expired */
    volatile bool timerExpired;

    /* Device Object handle assigned by the host */
    USB_HOST_DEVICE_OBJ_HANDLE usbHostDeviceInfo;
    
     /* Device object handle of the current attached device. */
    USB_HOST_DEVICE_OBJ_HANDLE attachedDeviceObjHandle;

    /* This is needed to track if the host is generating reset signal */
    bool isResetting;

    /* This counts the reset signal duration */
    DRV_USBHS_HOST_RESET_STATE resetState;

    /* This counts the reset signal duration */
    DRV_USBHS_HOST_ROOT_HUB_INFO rootHubInfo;


} DRV_USBHS_OBJ;



void _DRV_USBHS_HOST_AttachDetachStateMachine (DRV_USBHS_OBJ * hDriver);
void _DRV_USBHS_HOST_ResetStateMachine(DRV_USBHS_OBJ * hDriver);
void _DRV_USBHS_DEVICE_Initialize(DRV_USBHS_OBJ * drvObj, SYS_MODULE_INDEX index);
void _DRV_USBHS_DEVICE_Tasks_ISR(DRV_USBHS_OBJ * hDriver);
void _DRV_USBHS_DEVICE_Tasks_ISR_USBDMA(DRV_USBHS_OBJ * hDriver);
void _DRV_USBHS_HOST_Initialize(DRV_USBHS_OBJ * drvObj, SYS_MODULE_INDEX index);
void _DRV_USBHS_HOST_Tasks_ISR(DRV_USBHS_OBJ * hDriver);
void _DRV_USBHS_HOST_Tasks_ISR_USBDMA(DRV_USBHS_OBJ * hDriver);
uint8_t _DRV_USBHS_DEVICE_Get_FreeDMAChannel
(
    DRV_USBHS_OBJ * hDriver,
    bool endpointDir,
    uint8_t iEndpoint
);
uint8_t _DRV_USBHS_HOST_GetFreeDMAChannel
(
    DRV_USBHS_OBJ * hDriver,
    bool endpointDir,
    uint8_t iEndpoint
);

#endif
