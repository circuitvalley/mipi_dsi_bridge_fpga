 /*******************************************************************************
  USB Audio class definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_audio_local.h

  Summary:
    USB Audio class definitions

  Description:
    This file describes the Audio class specific definitions.
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

#ifndef _USB_DEVICE_AUDIO_V2_LOCAL_H
#define _USB_DEVICE_AUDIO_V2_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_device.h"
#include "usb/src/usb_device_local.h"
#include "usb/usb_device_audio_v2_0.h"


// *****************************************************************************
/* Audio flags.

  Summary:
 state of an audio streaming interface.

  Description:


  Remarks:
    This structure is internal to the Audio function driver.
 */ 
typedef enum
{
    USB_DEVICE_AUDIO_V2_STRMNG_INTFC_NOT_INITIALIZED = 0,
    USB_DEVICE_AUDIO_V2_STRMNG_INTFC_INITIALIZED

}USB_DEVICE_AUDIO_V2_STRMNG_INTFC_STATE;


// *****************************************************************************
/* USB Device Audio Transfer direction

  Summary:
    USB Device Audio Transfer direction

  Description:
    This enumeration defines the possible USB Device Audio device transfer direction.

  Remarks:
    None.
*/
typedef enum
{
    /* Audio Read */
    USB_DEVICE_AUDIO_V2_READ = 0,

    /* Audio Write */
    USB_DEVICE_AUDIO_V2_WRITE = 1

} USB_DEVICE_AUDIO_V2_TRANSFER_DIRECTION;


// *****************************************************************************
/* Audio flags.

  Summary:
   Flags for tracking internal status.

  Description:


  Remarks:
    This structure is internal to the Audio function driver.
*/

typedef union _USB_DEVICE_AUDIO_V2_FLAGS
{
    struct
    {
        uint8_t audioControlInterfaceReady:1;
    };
    uint8_t allFlags;
}USB_DEVICE_AUDIO_V2_FLAGS;


// *****************************************************************************
/* USB Device Audio Function Driver IRP object.

  Summary:
    USB Device Audio Function Driver IRP object.

  Description:
    IRP object used by the Audio to service application requests.

  Remarks:
    None.
*/
typedef struct
{

   USB_DEVICE_IRP irp;

   USB_DEVICE_AUDIO_V2_INDEX iAudio;

}USB_DEVICE_AUDIO_V2_IRP_OBJECT;

// *****************************************************************************
/* USB Device Audio Endpoint Instance structure .

  Summary:
    USB Device Audio Endpoint Instance.

  Description:
    This structure can be used for storing Audio endpoint parameters.

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
}USB_DEVICE_AUDIO_V2_EP_INSTANCE;

// *****************************************************************************
/* USB Device Audio Streaming Interface Alternate settings.

  Summary:
    USB Device Audio Streaming Interface Alternate settings.

  Description:
    This structure can be used for storing Audio Streaming Interface alternate settings. 

  Remarks:
    None.
*/
typedef struct
{
    /* Number of Endpoints in this interface. In an Audio Streaming interface
     number of Endpoints present can be 0, 1 or 2*/
    uint8_t numEndPoints;

    /* Audio Data Endpoint  */
    USB_DEVICE_AUDIO_V2_EP_INSTANCE isoDataEp;

    /* Audio Sync Endpoint*/
    USB_DEVICE_AUDIO_V2_EP_INSTANCE isoSyncEp;

}USB_DEVICE_AUDIO_V2_STREAMING_INTERFACE_ALTERNATE_SETTING;

// *****************************************************************************
/* USB Device Audio Streaming Interface structure

  Summary:
    USB Device Audio Streaming Interface structure

  Description:
    This structure can be used for storing Audio Streaming Interface.

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
    USB_DEVICE_AUDIO_V2_STRMNG_INTFC_STATE state;

    /* Array of Alternate settings for a Streaming Interface*/
    USB_DEVICE_AUDIO_V2_STREAMING_INTERFACE_ALTERNATE_SETTING  alterntSetting[USB_DEVICE_AUDIO_V2_MAX_ALTERNATE_SETTING];

}USB_DEVICE_AUDIO_V2_STREAMING_INTERFACE;


// *****************************************************************************
/* USB Device Audio Interface collection

  Summary:
    USB Device Audio Interface collection

  Description:
    This structure can be used for storing Audio Interface Collection.

  Remarks:
    None.
*/
typedef struct
{
    /* Audio Control interface number */
    uint8_t bControlInterfaceNum;

    /* Alternate setting for Audio Control Interface*/
    uint8_t bControlAltSetng;
    
    /* Number of streaming interfaces */
    uint8_t numStreamingInf;

    /* Audio spec in BCD 0x100 or 0x200 */
    uint16_t bcdADC;

    /* Optional interrupt ep info */
    USB_DEVICE_AUDIO_V2_EP_INSTANCE intEp[1];

    /* Presence or absence of the interrupt EP */
    bool isIntEpExists;

    /* Array Audio streaming Interfaces */
    USB_DEVICE_AUDIO_V2_STREAMING_INTERFACE streamInf[USB_DEVICE_AUDIO_V2_MAX_STREAMING_INTERFACES];

}USB_DEVICE_AUDIO_V2_INTERFACE_COLLECTION;



// *****************************************************************************
/* USB Device Audio Instance Structure

  Summary:
    USB Device Audio Instance Structure

  Description:
    This is Audio Instance Structure.

  Remarks:
    None.
*/
typedef struct
{
    /* Device layer instance associated with this function driver instance */
    USB_DEVICE_HANDLE devLayerHandle;

    /* Instance index */
    USB_DEVICE_AUDIO_V2_INDEX audioIndex;

    /* Audio Interface Collection*/
    USB_DEVICE_AUDIO_V2_INTERFACE_COLLECTION infCollection;

    /* Application callback */
    USB_DEVICE_AUDIO_V2_EVENT_HANDLER appEventCallBack;

    /* Application user data */
    uintptr_t userData;

    /* Flags for indicating status*/
    USB_DEVICE_AUDIO_V2_FLAGS flags;

    /* Transmit Queue Size*/
    unsigned int queueSizeWrite;

    /* Receive Queue Size */
    unsigned int queueSizeRead;
    
     /* Current Queue Size Write */
    unsigned int currentQSizeWrite;

     /* Current Queue Size Read */
    unsigned int currentQSizeRead;

}USB_DEVICE_AUDIO_V2_INSTANCE;


// *****************************************************************************
/* USB Device Audio Instance IRP Data

  Summary:
    USB Device Audio Instance IRP Data

  Description:
    This is structure is passed to an IRP for the Audio Device layer to retrieve
    the details of the transfer when IRP callback is occurred. 

  Remarks:
    None.
*/
typedef struct
{
    /* Audio Instance Number*/
    USB_DEVICE_AUDIO_V2_INDEX iAudio;
    
    /* Audio streaming Interface Number*/
    uint8_t interfaceNum;
    
    /* Audio Transfer direction */
    USB_DEVICE_AUDIO_V2_TRANSFER_DIRECTION direction;
    
} USB_DEVICE_AUDIO_V2_IRP_DATA;

// *****************************************************************************
/* Audio Common data object

  Summary:
    Object used to keep track of data that is common to all instances of the
    Audio function driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the Audio function driver.

  Remarks:
    None.
*/
typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool isMutexAudioIrpInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexAUDIOIRP);

} USB_DEVICE_AUDIO_V2_COMMON_DATA_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: Extern Data
// *****************************************************************************
// *****************************************************************************
extern USB_DEVICE_IRP gUSBDeviceAudioV2IRP[USB_DEVICE_AUDIO_V2_QUEUE_DEPTH_COMBINED];
extern USB_DEVICE_AUDIO_V2_IRP_DATA gUSBDeviceAudioV2IrpData[USB_DEVICE_AUDIO_V2_QUEUE_DEPTH_COMBINED];
extern USB_DEVICE_AUDIO_V2_INSTANCE gUsbDeviceAudioV2Instance[USB_DEVICE_AUDIO_V2_INSTANCES_NUMBER];
extern USB_DEVICE_AUDIO_V2_COMMON_DATA_OBJ gUSBDeviceAudioV2CommonDataObj;


// *****************************************************************************
// *****************************************************************************
// Section: Prototypes for Local functions. 
// *****************************************************************************
// *****************************************************************************
USB_DEVICE_AUDIO_V2_RESULT _USB_DEVICE_AUDIO_V2_Transfer
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio,
    USB_DEVICE_AUDIO_V2_TRANSFER_HANDLE *transferHandle,
    uint8_t interfaceNum ,
    void * data ,
    size_t size,
    USB_DEVICE_AUDIO_V2_TRANSFER_DIRECTION direction
);

void _USB_DEVICE_AUDIO_V2_TransferIRPCallBack ( USB_DEVICE_IRP * irp );

void _USB_DEVICE_AUDIO_V2_Initialize
(
    SYS_MODULE_INDEX iAudio,
    DRV_HANDLE usbDeviceHandle,
    void* funcDriverInit,
    uint8_t interfaceNumber,
    uint8_t alternateSetting,
    uint8_t descriptorType, 
    uint8_t * pDescriptor
);

void _USB_DEVICE_AUDIO_V2_ControlTransferHandler
(
    SYS_MODULE_INDEX iAudio,
    USB_DEVICE_EVENT controlEvent,
    USB_SETUP_PACKET* controlEventData
);


void _USB_DEVICE_AUDIO_V2_Deinitialize ( SYS_MODULE_INDEX funcDriverIndex );

void _USB_DEVICE_AUDIO_V2_SetupPacketHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_SETUP_PACKET* controlEventData
);
void _USB_DEVICE_AUDIO_V2_GlobalInitialize (void);
#endif

 /************ End of file *************************************/
