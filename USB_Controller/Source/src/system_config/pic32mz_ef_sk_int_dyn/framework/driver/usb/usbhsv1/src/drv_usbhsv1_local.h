/*******************************************************************************
  USB Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhsv1_local.h

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

#ifndef _DRV_USBHSV1_LOCAL_H
#define _DRV_USBHSV1_LOCAL_H


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
#include "driver/usb/usbhsv1/drv_usbhsv1.h"
#include "driver/usb/usbhsv1/src/drv_usbhsv1_variant_mapping.h"
#include "osal/osal.h"

#define DRV_USBHS_HOST_IRP_PER_FRAME_NUMBER 	            5
#define DRV_USBHSV1_HOST_MAXIMUM_ENDPOINTS_NUMBER           10
#define DRV_USBHSV1_MAX_DMA_CHANNELS                        7

#ifndef USBHSV1_RAM_ADDR
#define USBHSV1_RAM_ADDR                                      0xA0100000u
#endif

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
    DRV_USBHSV1_DEVICE_ENDPOINT_STATE_ENABLED = 0x1,
    DRV_USBHSV1_DEVICE_ENDPOINT_STATE_STALLED = 0x2
}
DRV_USBHSV1_DEVICE_ENDPOINT_STATE;

/************************************************
 * Endpoint data structure. This data structure
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
    DRV_USBHSV1_DEVICE_ENDPOINT_STATE endpointState;

	/* This gives the Endpoint Direction */
	USB_DATA_DIRECTION endpointDirection;

}
DRV_USBHSV1_DEVICE_ENDPOINT_OBJ;

/*********************************************
 * These IRP states are used internally by the
 * HCD to track completion of a host IRP. This
 * is not the same as the public IRP status
 *********************************************/
typedef enum
{   
    DRV_USBHSV1_HOST_IRP_STATE_SETUP_STAGE,
    DRV_USBHSV1_HOST_IRP_STATE_SETUP_TOKEN_SENT,
    DRV_USBHSV1_HOST_IRP_STATE_DATA_STAGE,
    DRV_USBHSV1_HOST_IRP_STATE_DATA_STAGE_SENT,
    DRV_USBHSV1_HOST_IRP_STATE_HANDSHAKE,
    DRV_USBHSV1_HOST_IRP_STATE_HANDSHAKE_SENT,
    DRV_USBHSV1_HOST_IRP_STATE_COMPLETE,
    DRV_USBHSV1_HOST_IRP_STATE_ABORTED,
    DRV_USBHSV1_HOST_IRP_STATE_PROCESSING
}
DRV_USBHSV1_HOST_IRP_STATE; 

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
    DRV_USBHSV1_HOST_IRP_STATE tempState;
    uint32_t completedBytes;
    struct _USB_HOST_IRP_LOCAL * next;
    struct _USB_HOST_IRP_LOCAL * previous;
    DRV_USBHSV1_HOST_PIPE_HANDLE  pipe;
}
USB_HOST_IRP_LOCAL;

/************************************************
 * This is the Host Pipe Object.
 ************************************************/
typedef struct _DRV_USBHSV1_HOST_PIPE_OBJ
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
    struct _DRV_USBHSV1_HOST_PIPE_OBJ * next;
    struct _DRV_USBHSV1_HOST_PIPE_OBJ * previous;

    /* Interval counter */

    uint8_t intervalCounter;

    /* Pipe Speed */
    USB_SPEED speed;

    /* Hub Address */
    uint8_t hubAddress;

    /* Hub Port*/
    uint8_t hubPort;

    /* Host Pipe allocated*/
    uint8_t hostPipeN;
}
DRV_USBHSV1_HOST_PIPE_OBJ;

/*********************************************
 * Host Transfer Group. This data structures
 * contains all pipes belonging to one tranfer
 * type.
 *********************************************/

typedef struct _DRV_USBHSV1_HOST_TRANSFER_GROUP
{
    /* The first pipe in this transfer 
     * group */
    DRV_USBHSV1_HOST_PIPE_OBJ * pipe;
    
    /* The current pipe being serviced
     * in this transfer group */
    DRV_USBHSV1_HOST_PIPE_OBJ * currentPipe;
    
    /* The current IRP being serviced
     * in the pipe */
    void * currentIRP;

    /* Total number of pipes in this
     * transfer group */
    int nPipes;
}
DRV_USBHSV1_HOST_TRANSFER_GROUP;

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
DRV_USBHSV1_TRANSACTION_RESULT;

typedef enum
{
    DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST,
    DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_RX_DATA_STAGE_FROM_HOST,
    DRV_USBHSV1_DEVICE_EP0_STATE_EXPECTING_TX_STATUS_STAGE_FROM_HOST,
    DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_SETUP_IRP_FROM_CLIENT,
    DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_IRP_FROM_CLIENT,
    DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_IRP_FROM_CLIENT,
    DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT,
    DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT,
	DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_COMPLETE,
	DRV_USBHSV1_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_COMPLETE,
    DRV_USBHSV1_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS,
    DRV_USBHSV1_DEVICE_EP0_STATE_RX_DATA_STAGE_IN_PROGRESS,
}
DRV_USBHSV1_DEVICE_EP0_STATE;


/**********************************************
 * Host Endpoint Object.
 *********************************************/

typedef struct
{
    /* Indicates this endpoint is in use */
    bool inUse;
    DRV_USBHSV1_HOST_PIPE_OBJ * pipe;

}_DRV_USBHSV1_HOST_ENDPOINT;


typedef struct
{
    /* A combination of two structures for
     * each direction. */

    _DRV_USBHSV1_HOST_ENDPOINT endpoint;

}DRV_USBHSV1_HOST_ENDPOINT_OBJ;


/*********************************************
 * Driver NON ISR Tasks routine states.
 *********************************************/

typedef enum
{
    /* Driver checks if the UTMI clock is usable */
    DRV_USBHSV1_TASK_STATE_WAIT_FOR_CLOCK_USABLE = 1,

    /* Driver initializes the required operation mode  */
    DRV_USBHSV1_TASK_STATE_INITIALIZE_OPERATION_MODE,

    /* Driver has complete initialization and can be opened */
    DRV_USBHSV1_TASK_STATE_RUNNING

} DRV_USBHSV1_TASK_STATE;

typedef struct _DRV_USBHSV1_DMA_POOL_STRUCT
{
    bool inUse;
    bool endpointDir;
    uint8_t iEndpoint;
    unsigned int count;

} DRV_USBHSV1_DMA_POOL;

/*********************************************
 * Host Mode Device Attach Detach State
 ********************************************/
typedef enum
{
    /* No device is attached */
    DRV_USBHSV1_HOST_ATTACH_STATE_CHECK_FOR_DEVICE_ATTACH = 0,

    /* Waiting for debounce delay */
    DRV_USBHSV1_HOST_ATTACH_STATE_DETECTED,

    /* Debouncing is complete. Device is attached */
    DRV_USBHSV1_HOST_ATTACH_STATE_READY,

} DRV_USBHSV1_HOST_ATTACH_STATE;

/*********************************************
 * Host Mode Device Reset state
 *********************************************/
typedef enum
{
    /* No Reset in progress */
    DRV_USBHSV1_HOST_RESET_STATE_NO_RESET = 0,

    /* Start the reset signalling */
    DRV_USBHSV1_HOST_RESET_STATE_START,

    /* Check if reset duration is done and stop reset */
    DRV_USBHSV1_HOST_RESET_STATE_WAIT_FOR_COMPLETE

} DRV_USBHSV1_HOST_RESET_STATE;

/**************************************
 * Root Hub parameters
 **************************************/

typedef struct
{
    /* Pointer to the port over current detect function */
    DRV_USBHSV1_ROOT_HUB_PORT_OVER_CURRENT_DETECT portOverCurrentDetect;

    /* Pointer to the port indication function */
    DRV_USBHSV1_ROOT_HUB_PORT_INDICATION portIndication;

    /* Pointer to the port enable function */
    DRV_USBHSV1_ROOT_HUB_PORT_POWER_ENABLE portPowerEnable;

    /* Total current available */
    uint32_t rootHubAvailableCurrent;

} DRV_USBHSV1_HOST_ROOT_HUB_INFO;

/***********************************************
 * Driver object structure. One object per
 * hardware instance
 **********************************************/

typedef struct _DRV_USBHSV1_OBJ_STRUCT
{
    /* Indicates this object is in use */
    bool inUse;

    /* Indicates that the client has opened the instance */
    bool isOpened;

    DRV_USBHSV1_DMA_POOL gDrvUSBDMAPool[DRV_USBHSV1_MAX_DMA_CHANNELS + 1];

    /* Status of this driver instance */
    SYS_STATUS status;     

    /* Contains the consumed FIFO size */
    unsigned int consumedFIFOSize;

    /* Contains the desired operation speed */
    DRV_USBHSV1_DEVICE_SPEEDCONF operationSpeed;

    /* Contains the EPO state */
    DRV_USBHSV1_DEVICE_EP0_STATE endpoint0State;

    /* The USB peripheral associated with the object */
    usbhs_registers_t * usbID;

    /* Current operating mode of the driver */
    DRV_USBHSV1_OPMODES operationMode;

    /* Interrupt source for USB module */
    INT_SOURCE interruptSource;

    /* Mutex create function place holder*/
    OSAL_MUTEX_DECLARE (mutexID);

    /* Pointer to the endpoint table */
    DRV_USBHSV1_DEVICE_ENDPOINT_OBJ *endpointTable;

    /* Pointer to the endpoint table */
    DRV_USBHSV1_HOST_ENDPOINT_OBJ hostEndpointTable[DRV_USBHSV1_HOST_MAXIMUM_ENDPOINTS_NUMBER];

    /* The object is current in an interrupt context */
    bool isInInterruptContext;
	
    /* Maintains the timer count value for host */
    uint32_t timerCount;

    /* Root Hub Port 0 attached device speed in host mode
     * In device mode, the speed at which the device attached */
    USB_SPEED deviceSpeed;

    /* Transfer Groups */
    DRV_USBHSV1_HOST_TRANSFER_GROUP controlTransferGroup;

    /* Tracks if the current control transfer is a zero length */
    bool isZeroLengthControlTransfer;

    /* Non ISR Task Routine state */
    DRV_USBHSV1_TASK_STATE state;
	
    /* Client data that will be returned at callback */
    uintptr_t  hClientArg;

    /* Call back function for this client */
    DRV_USB_EVENT_CALLBACK  pEventCallBack;

    /* Current VBUS level when running in device mode */
    DRV_USB_VBUS_LEVEL vbusLevel;

	/* Callback to determine the Vbus level */
	DRV_USBHSV1_VBUS_COMPARATOR vbusComparator;

    /* Sent session invalid event to the client */
    bool sessionInvalidEventSent;

    /* Attach state of the device */
    DRV_USBHSV1_HOST_ATTACH_STATE attachState;
    
    /* True if device is attached */
    bool deviceAttached;

    /* Device Object handle assigned by the host */
    USB_HOST_DEVICE_OBJ_HANDLE usbHostDeviceInfo;
    
     /* Device object handle of the current attached device. */
    USB_HOST_DEVICE_OBJ_HANDLE attachedDeviceObjHandle;

    /* This is needed to track if the host is generating reset signal */
    bool isResetting;

    /* This counts the reset signal duration */
    DRV_USBHSV1_HOST_RESET_STATE resetState;

    /* This counts the reset signal duration */
    DRV_USBHSV1_HOST_ROOT_HUB_INFO rootHubInfo;
	
	/* This is array of device endpoint objects pointers */ 
	DRV_USBHSV1_DEVICE_ENDPOINT_OBJ * deviceEndpointObj[DRV_USBHSV1_ENDPOINTS_NUMBER];
    
    /* True if Root Hub Operation is enabled */
    bool operationEnabled;
	
} DRV_USBHSV1_OBJ;

/****************************************************************************
 * Get 64-, 32-, 16- or 8-bit access to FIFO data register of selected pipe.
 * p:      Target Pipe number
 * scale:  Data scale in bits: 64, 32, 16 or 8
 * return  Volatile 64-, 32-, 16- or 8-bit data pointer to FIFO data register
 *
 * warning It is up to the user of this macro to make sure that all accesses
 * are aligned with their natural boundaries except 64-bit accesses which
 * require only 32-bit alignment.
 *
 * warning It is up to the user of this macro to make sure that used HSB
 * addresses are identical to the DPRAM internal pointer modulo 32 bits.
 ***************************************************************************/
  #define drv_usbhsv1_get_pipe_fifo_access(p) \
  (((volatile uint8_t(*)[0x8000])0xA0100000u)[(p)])

/****************************************************************************
 * Bounds given integer size to allowed range and rounds it up to the nearest
 * available greater size, then applies register format of USBHS controller
 * for pipe size bit-field.
 ****************************************************************************/
#define drv_usbhsv1_format_pipe_size(size) \
(32 - clz(((uint32_t)min(max(size, 8), 1024) << 1) - 1) - 1 - 3)


/*! \brief Tests the bits of a value specified by a given bit-mask.
 *
 * \param value Value of which to test bits.
 * \param mask  Bit-mask indicating bits to test.
 *
 * \return \c 1 if at least one of the tested bits is set, else \c 0.
 */
#define Tst_bits( value, mask)  (Rd_bits(value, mask) != 0)

/*! \brief Clears the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to clear bits.
 * \param mask    Bit-mask indicating bits to clear.
 *
 * \return Resulting value with cleared bits.
 */
#define Clr_bits(lvalue, mask)  ((lvalue) &= ~(mask))

/*! \brief Sets the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue of which to set bits.
 * \param mask    Bit-mask indicating bits to set.
 *
 * \return Resulting value with set bits.
 */
#define Set_bits(lvalue, mask)  ((lvalue) |=  (mask))

/*! \brief Writes the bit-field of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue    C lvalue to write a bit-field to.
 * \param mask      Bit-mask indicating the bit-field to write.
 * \param bitfield  Bit-field to write.
 *
 * \return Resulting value with written bit-field.
 */
#define Wr_bitfield(lvalue, mask, bitfield) (Wr_bits(lvalue, mask, (uint32_t)(bitfield) << ctz(mask)))

/*! \brief Reads the bits of a value specified by a given bit-mask.
 *
 * \param value Value to read bits from.
 * \param mask  Bit-mask indicating bits to read.
 *
 * \return Read bits.
 */
#define Rd_bits( value, mask)        ((value) & (mask))

/*! \brief Writes the bits of a C lvalue specified by a given bit-mask.
 *
 * \param lvalue  C lvalue to write bits to.
 * \param mask    Bit-mask indicating bits to write.
 * \param bits    Bits to write.
 *
 * \return Resulting value with written bits.
 */
#define Wr_bits(lvalue, mask, bits)  ((lvalue) = ((lvalue) & ~(mask)) |\
                                                 ((bits  ) &  (mask)))

/*! \brief Takes the minimal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Minimal value of \a a and \a b.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define min(a, b)   (((a) < (b)) ?  (a) : (b))

/*! \brief Takes the maximal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Maximal value of \a a and \a b.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define max(a, b)   (((a) > (b)) ?  (a) : (b))

/*! \brief Counts the trailing zero bits of the given value considered as a 32-bit integer.
 *
 * \param u Value of which to count the trailing zero bits.
 *
 * \return The count of trailing zero bits in \a u.
 */
#   define ctz(u)              ((u) ? __builtin_ctz(u) : 32)

/*! \brief Counts the leading zero bits of the given value considered as a 32-bit integer.
 *
 * \param u Value of which to count the leading zero bits.
 *
 * \return The count of leading zero bits in \a u.
 */
#   define clz(u)              ((u) ? __builtin_clz(u) : 32)


void _DRV_USBHSV1_HOST_AttachDetachStateMachine (DRV_USBHSV1_OBJ * hDriver);
void _DRV_USBHSV1_HOST_ResetStateMachine(DRV_USBHSV1_OBJ * hDriver);
void _DRV_USBHSV1_DEVICE_Initialize(DRV_USBHSV1_OBJ * drvObj, SYS_MODULE_INDEX index);
void _DRV_USBHSV1_DEVICE_Tasks_ISR(DRV_USBHSV1_OBJ * hDriver);
void _DRV_USBHSV1_DEVICE_Tasks_ISR_USBDMA(DRV_USBHSV1_OBJ * hDriver);
void _DRV_USBHSV1_HOST_Initialize(DRV_USBHSV1_OBJ * drvObj, SYS_MODULE_INDEX index);
void _DRV_USBHSV1_HOST_Tasks_ISR(DRV_USBHSV1_OBJ * hDriver);
uint8_t _DRV_USBHSV1_DEVICE_Get_FreeDMAChannel
(
    DRV_USBHSV1_OBJ * hDriver,
    bool endpointDir,
    uint8_t iEndpoint
);

#endif
