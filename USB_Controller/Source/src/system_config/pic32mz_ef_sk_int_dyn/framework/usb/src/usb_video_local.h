/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _VIDEO_LOCAL_H    /* Guard against multiple inclusion */
#define _VIDEO_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_device.h"
#include "usb/src/usb_device_local.h"


// *****************************************************************************
/* Video flags.

  Summary:
 state of an video streaming interface.

  Description:


  Remarks:
    This structure is internal to the Video function driver.
 */ 
typedef enum
{
    USB_DEVICE_VIDEO_STRMNG_INTFC_NOT_INITIALIZED = 0,
    USB_DEVICE_VIDEO_STRMNG_INTFC_INITIALIZED

}USB_DEVICE_VIDEO_STRMNG_INTFC_STATE;


// *****************************************************************************
/* USB Device Video Transfer direction

  Summary:
    USB Device Video Transfer direction

  Description:
    This enumeration defines the possible USB Device Video device transfer direction.

  Remarks:
    None.
*/
typedef enum
{
    /* Video Read */
    USB_DEVICE_VIDEO_READ = 0,

    /* Video Write */
    USB_DEVICE_VIDEO_WRITE = 1

} USB_DEVICE_VIDEO_TRANSFER_DIRECTION;


// *****************************************************************************
/* Video flags.

  Summary:
   Flags for tracking internal status.

  Description:


  Remarks:
    This structure is internal to the Video function driver.
*/

typedef union _USB_DEVICE_VIDEO_FLAGS
{
    struct
    {
        uint8_t videoControlInterfaceReady:1;
    };
    uint8_t allFlags;
}USB_DEVICE_VIDEO_FLAGS;


// *****************************************************************************
/* USB Device Video Function Driver IRP object.

  Summary:
    USB Device Video Function Driver IRP object.

  Description:
    IRP object used by the Video to service application requests.

  Remarks:
    None.
*/
typedef struct
{

   USB_DEVICE_IRP irp;

   USB_DEVICE_VIDEO_INDEX iVideo;

}USB_DEVICE_VIDEO_IRP_OBJECT;

// *****************************************************************************
/* USB Device Video Endpoint Instance structure .

  Summary:
    USB Device Video Endpoint Instance.

  Description:
    This structure can be used for storing Video endpoint parameters.

  Remarks:
    None.
*/
typedef struct
{
    /* End point address */
    uint8_t epAddr;

    /* End point maximum payload */
    uint16_t epMaxPacketSize;

    /* Indicates if the endpoint is Enabled or not */ 
    bool status;
}USB_DEVICE_VIDEO_EP_INSTANCE;

// *****************************************************************************
/* USB Device Video Streaming Interface Alternate settings.

  Summary:
    USB Device Video Streaming Interface Alternate settings.

  Description:
    This structure can be used for storing Video Streaming Interface alternate settings. 

  Remarks:
    None.
*/
typedef struct
{
    /* Number of Endpoints in this interface. In an Video Streaming interface
     number of Endpoints present can be 0, 1 or 2*/
    uint8_t numEndPoints;

    /* Video Data Endpoint  */
    USB_DEVICE_VIDEO_EP_INSTANCE isoDataEp;

    /* Video Sync Endpoint*/
    USB_DEVICE_VIDEO_EP_INSTANCE isoSyncEp;

}USB_DEVICE_VIDEO_STREAMING_INTERFACE_ALTERNATE_SETTING;

// *****************************************************************************
/* USB Device Video Streaming Interface structure

  Summary:
    USB Device Video Streaming Interface structure

  Description:
    This structure can be used for storing Video Streaming Interface.

  Remarks:
    None.
*/
typedef struct
{

    /* Interface number */
    uint8_t interfaceNum;

    /* Currently active alternate setting */
    uint8_t activeSetting;

    /* Indicates if any endpoint is active on this interface. */ 
    USB_DEVICE_VIDEO_STRMNG_INTFC_STATE state;

    /* Array of Alternate settings for a Streaming Interface*/
    USB_DEVICE_VIDEO_STREAMING_INTERFACE_ALTERNATE_SETTING  alterntSetting[USB_DEVICE_VIDEO_MAX_ALTERNATE_SETTING];

}USB_DEVICE_VIDEO_STREAMING_INTERFACE;


// *****************************************************************************
/* USB Device Video Interface collection

  Summary:
    USB Device Video Interface collection

  Description:
    This structure can be used for storing Video Interface Collection.

  Remarks:
    None.
*/
typedef struct
{
    /* Video Control interface number */
    uint8_t bControlInterfaceNum;

    /* Alternate setting for Video Control Interface*/
    uint8_t bControlAltSetng;
    
    /* Number of streaming interfaces */
    uint8_t numStreamingInf;

    /* Video spec in BCD 0x100 or 0x200 */
    uint16_t bcdADC;

    /* Optional interrupt ep info */
    USB_DEVICE_VIDEO_EP_INSTANCE intEp[1];

    /* Presence or absence of the interrupt EP */
    bool isIntEpExists;

    /* Array Video streaming Interfaces */
    USB_DEVICE_VIDEO_STREAMING_INTERFACE streamInf[USB_DEVICE_VIDEO_MAX_STREAMING_INTERFACES];

}USB_DEVICE_VIDEO_INTERFACE_COLLECTION;



// *****************************************************************************
/* USB Device Video Instance Structure

  Summary:
    USB Device Video Instance Structure

  Description:
    This is Video Instance Structure.

  Remarks:
    None.
*/
typedef struct
{
    /* Device layer instance associated with this function driver instance */
    USB_DEVICE_HANDLE devLayerHandle;

    /* Instance index */
    USB_DEVICE_VIDEO_INDEX videoIndex;

    /* Video Interface Collection*/
    USB_DEVICE_VIDEO_INTERFACE_COLLECTION infCollection;

    /* Application callback */
    USB_DEVICE_VIDEO_EVENT_HANDLER appEventCallBack;

    /* Application user data */
    uintptr_t userData;

    /* Flags for indicating status*/
    USB_DEVICE_VIDEO_FLAGS flags;

    /* Transmit Queue Size*/
    unsigned int queueSizeWrite;

    /* Receive Queue Size */
    unsigned int queueSizeRead;
    
     /* Current Queue Size Write */
    volatile unsigned int currentQSizeWrite;

     /* Current Queue Size Read */
    volatile unsigned int currentQSizeRead;

}USB_DEVICE_VIDEO_INSTANCE;


// *****************************************************************************
/* USB Device Video Instance IRP Data

  Summary:
    USB Device Video Instance IRP Data

  Description:
    This is structure is passed to an IRP for the Video Device layer to retrieve
    the details of the transfer when IRP callback is occurred. 

  Remarks:
    None.
*/
typedef struct
{
    /* Video Instance Number*/
    USB_DEVICE_VIDEO_INDEX iVideo;
    
    /* Video streaming Interface Number*/
    uint8_t interfaceNum;
    
    /* Video Transfer direction */
    USB_DEVICE_VIDEO_TRANSFER_DIRECTION direction;
    
} USB_DEVICE_VIDEO_IRP_DATA;

// *****************************************************************************
/* Video Common data object

  Summary:
    Object used to keep track of data that is common to all instances of the
    Video function driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the Video function driver.

  Remarks:
    None.
*/
typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool isMutexVideoIrpInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexVIDEOIRP);

} USB_DEVICE_VIDEO_COMMON_DATA_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: Extern Data
// *****************************************************************************
// *****************************************************************************
extern USB_DEVICE_IRP gUSBDeviceVideoIRP[USB_DEVICE_VIDEO_QUEUE_DEPTH_COMBINED];
extern USB_DEVICE_VIDEO_IRP_DATA gUSBDeviceVideoIrpData [USB_DEVICE_VIDEO_QUEUE_DEPTH_COMBINED];
extern USB_DEVICE_VIDEO_INSTANCE gUsbDeviceVideoInstance[USB_DEVICE_VIDEO_INSTANCES_NUMBER];
extern USB_DEVICE_VIDEO_COMMON_DATA_OBJ gUSBDeviceVideoCommonDataObj;


// *****************************************************************************
// *****************************************************************************
// Section: Prototypes for Local functions. 
// *****************************************************************************
// *****************************************************************************
USB_DEVICE_VIDEO_RESULT _USB_DEVICE_VIDEO_Transfer
(
    USB_DEVICE_VIDEO_INDEX iVideo,
    USB_DEVICE_VIDEO_TRANSFER_HANDLE *transferHandle,
    uint8_t interfaceNum ,
    void * data ,
    size_t size,
    USB_DEVICE_VIDEO_TRANSFER_DIRECTION direction
);

void _USB_DEVICE_VIDEO_TransferIRPCallBack ( USB_DEVICE_IRP * irp );

void _USB_DEVICE_VIDEO_Initialize
(
    SYS_MODULE_INDEX iVideo,
    DRV_HANDLE usbDeviceHandle,
    void* funcDriverInit,
    uint8_t interfaceNumber,
    uint8_t alternateSetting,
    uint8_t descriptorType, 
    uint8_t * pDescriptor
);

void _USB_DEVICE_VIDEO_ControlTransferHandler
(
    SYS_MODULE_INDEX iVideo,
    USB_DEVICE_EVENT controlEvent,
    USB_SETUP_PACKET* controlEventData
);


void _USB_DEVICE_VIDEO_Deinitialize ( SYS_MODULE_INDEX funcDriverIndex );

void _USB_DEVICE_VIDEO_SetupPacketHandler
(
    USB_DEVICE_VIDEO_INDEX iVideo ,
    USB_SETUP_PACKET* controlEventData
);
void _USB_DEVICE_VIDEO_GlobalInitialize (void);

void _USB_DEVICE_VIDEO_TransferAbortAllow(USB_DEVICE_IRP * irp); 
void _USB_DEVICE_VIDEO_TransferAbortPrevent(USB_DEVICE_IRP * irp); 

#if defined USB_DEVICE_VIDEO_TRANSFER_ABORT_NOTIFY 
    #define _USB_DEVICE_VIDEO_TransferCompleteCallback(x) _USB_DEVICE_VIDEO_TransferAbortAllow(x)
#else
    #define _USB_DEVICE_VIDEO_TransferCompleteCallback(x) _USB_DEVICE_VIDEO_TransferAbortPrevent(x)
#endif
#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
