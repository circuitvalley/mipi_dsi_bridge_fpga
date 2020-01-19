/*******************************************************************************
 USB Device MSD function driver

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_msd_local.h

  Summary:
    USB Device MSD function driver local header file.

  Description:
    This file contains types and definitions which are local to the MSD
    function driver. The application should not include this file directly.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
 Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

 Microchip licenses to you the right to use, modify, copy and distribute
 Software only when embedded on a Microchip microcontroller or digital signal
 controller that is integrated into your product or third party product
 (pursuant to the sublicense terms in the accompanying license agreement).

 You should refer to the license agreement accompanying this Software for
 additional information regarding your rights and obligations.

 SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END

#ifndef _USB_DEVICE_MSD_LOCAL_H_
#define _USB_DEVICE_MSD_LOCAL_H_

#include "system_config.h"
#include "system/fs/sys_fs_media_manager.h"
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define _DRV_MSD_NUM_SECTORS_BUFFERING (USB_DEVICE_MSD_NUM_SECTOR_BUFFERS)

// *****************************************************************************
// *****************************************************************************
// Section: Local data types.
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB device MSD state enumeration.

  Summary:
    Identifies the different states MSD data transfer state machine.

  Description:
    This enumeration defines values for different MSD data transfer states.

  Remarks:
    None.
 */

typedef enum
{
    USB_DEVICE_MSD_STATE_WAIT_FOR_CBW,        
    USB_DEVICE_MSD_STATE_CBW,        
    USB_DEVICE_MSD_STATE_PROCESS_CBW,        
    USB_DEVICE_MSD_STATE_STALL_IN_OUT,
    USB_DEVICE_MSD_STATE_DATA_IN,
    USB_DEVICE_MSD_STATE_DATA_OUT,
    USB_DEVICE_MSD_STATE_CSW,
    USB_DEVICE_MSD_STATE_SEND_CSW,
    USB_DEVICE_MSD_STATE_IDLE
	
} USB_DEVICE_MSD_STATE;

// *****************************************************************************
/* USB device MSD media operation state.

  Summary:
    Enumeration values for USB device MSD media operation state.

  Description:
    This enumeration defines values for USB device MSD media operation state.

  Remarks:
    None.

 */

typedef enum
{
    USB_DEVICE_MSD_MEDIA_OPERATION_IDLE,
    USB_DEVICE_MSD_MEDIA_OPERATION_COMPLETE,
    USB_DEVICE_MSD_MEDIA_OPERATION_PENDING,
    USB_DEVICE_MSD_MEDIA_OPERATION_ERROR
	
} USB_DEVICE_MSD_MEDIA_OPERATION;  

// *****************************************************************************
/* Structure that carries all media info.

  Summary:
    Structure that carries all media info.

  Description:
    This structure keeps track of the media information that can change.
    
  Remarks:
    Typically one per LUN.
*/

typedef struct
{
    /* True if sense data was sent and sense data should be
     * reset on the next CBW. */
    bool resetSenseData ;

    /* Sense data */
    SCSI_SENSE_DATA *senseData ;

    /* Sector Size of the media */
    uint16_t sectorSize ;

    /* True if the media is attached */
    bool mediaPresent ;

    /* Media driver handle */
    DRV_HANDLE mediaHandle;

    /* Current state of media operation */
    USB_DEVICE_MSD_MEDIA_OPERATION mediaState;
    
    /* Pointer to the media geometry */
    SYS_FS_MEDIA_GEOMETRY * mediaGeometry;

    	
} USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA;

// *****************************************************************************
/* USB MSD device instance structure.

  Summary:
    USB MSD device instance structure

  Description:
    This structure defines data structure on per USB MSD device instance level.

  Remarks:
    This is a private structure of USB MSD device.
 */

typedef struct
{
    /* USB Device Layer Handle */
    USB_DEVICE_HANDLE hUsbDevHandle;                        
    
    /* A CBW structure for this instance */
    USB_MSD_CBW * msdCBW;

    /* Working buffer */
    uint8_t msdBuffer;

    /* State of the main state machine */ 
    USB_DEVICE_MSD_STATE msdMainState;  

    /* A CSW structure for this instance */    
    USB_MSD_CSW * msdCSW;

    /* Receive/Transmit data byte count */
    uint32_t rxTxTotalDataByteCount;                    

    uint8_t bufferOffset;
    uint8_t numUsbSectors;
    uint8_t numSectorsToWrite;
    uint8_t numPendingIrps;

    /* Dynamic media information */
    USB_DEVICE_MSD_MEDIA_DYNAMIC_DATA mediaDynamicData[USB_DEVICE_MSD_LUNS_NUMBER]; 

    /* The IN bulk endpoint */
    USB_ENDPOINT bulkEndpointTx;

    /* The OUT bulk endpoint */
    USB_ENDPOINT bulkEndpointRx;

    /* The OUT bulk endpoint size */
    uint16_t bulkEndpointRxSize;
    
    /* The IN bulk endpoint size */
    uint16_t bulkEndpointTxSize;

    /* The transmit IRP */
    USB_DEVICE_IRP irpTx;

    /* The receive IRP */
    USB_DEVICE_IRP irpRx;

    /* Number of LUNS in this MSD Device */
    uint8_t numberOfLogicalUnits;

    /* Media Initialization table containing
     * data about each media associated with 
     * this MSD device */
    USB_DEVICE_MSD_MEDIA_INIT_DATA * mediaData;

    /* The current alternate setting for this
     * MSD instance */
    uint8_t alternateSetting;

}USB_DEVICE_MSD_INSTANCE;


// *****************************************************************************
/* USB MSD device DWORD structure.

  Summary:
    USB MSD device DWORD structure.

  Description:
    This structure is used by the MSD function driver implementation.

  Remarks:
    This is a private structure of USB MSD device.
 */

typedef union
{
    uint32_t Val;
    uint16_t w[2];
    uint8_t v[4];
    struct
    {
        uint16_t LW;
        uint16_t HW;
    } word;
    struct
    {
        uint8_t LB;
        uint8_t HB;
        uint8_t UB;
        uint8_t MB;
    } byte;

} USB_DEVICE_MSD_DWORD_VAL;

/************************************************
 * The coherent attribute is not available in 
 * ARM GCC compiler for PIC32C. Do this to avoid
 * the compile warning.
 ************************************************/

#if defined (__PIC32C__)
#define MSD_COHERENT_ATTRIBUTE
#define MSD_SPACE_ALIGNED_ATTRIBUTE(VALUE1, VALUE2)
#define MSD_ADDRESS_ATTRIBUTE(VALUE)
#define MSD_SECTION_ATTRIBUTE(VALUE) __attribute__((section(VALUE)))
#else
#define MSD_COHERENT_ATTRIBUTE __attribute__((coherent))
#define MSD_SPACE_ALIGNED_ATTRIBUTE(VALUE1, VALUE2) __attribute__((space(VALUE1),aligned(VALUE2)))
#define MSD_ADDRESS_ATTRIBUTE(VALUE) __attribute__((address(VALUE)))
#define MSD_SECTION_ATTRIBUTE(VALUE)
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Local functions.
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void _USB_DEVICE_MSD_CallBackBulkRxTransfer
    (
        void *  hMsdInstance, 
        DRV_USB_PIPE_HANDLE hPipe,
        DRV_USB_XFER_HANDLE hTransfer,
        unsigned short int  transferByteCount  ,
        DRV_USB_DEVICE_XFER_STATUS  statusTransfer 
    )

  Summary:
    This callback will be called by the controller driver after the bulk out
    transfer.

  Description:
    This callback will be called by the controller driver after the bulk out
    transfer.
  
  Remarks:
    This is a local function and should not be called directly by the
    application.

*/

void _USB_DEVICE_MSD_CallBackBulkRxTransfer( USB_DEVICE_IRP *  hMsdInstance );


// *****************************************************************************
/* Function:
   void _USB_DEVICE_MSD_ResetSenseData
                                ( USB_DEVICE_MSD_SENSE_DATA *  msdSenseDataPtr )

  Summary:
    Reset's sense data that is pointed by "msdSenseDataPtr".

  Description:


  Precondition:
    None.

  Parameters:
    msdSenseDataPtr       -   Pointer to sense data.
   

  Returns:
    none.

  Remarks:

*/

void _USB_DEVICE_MSD_ResetSenseData(SCSI_SENSE_DATA *  msdSenseDataPtr );


// *****************************************************************************
/* Function:
   USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_ProcessCommand(SYS_MODULE_INDEX iMSD,
                                                   uint8_t * commandStatus )

  Summary:
    Process the CBW command.

  Description:
    This function processes the command and returns buffer address, buffer
    size and command status.

  Precondition:
    None.

  Parameters:
    iMSD                -  MSD instance index.
    rxTxBufferAdrs      -  Receive/Transmit buffer address.
    rxTxBufferSize      -  Receive/Transmit buffer size.
    commandStatus       -  Command status.

  Returns:
    MSD state.

  Remarks:

*/

USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_ProcessNonRWCommand
(
    SYS_MODULE_INDEX iMSD,
    uint8_t * commandStatus
);

USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_ProcessRead
(
    SYS_MODULE_INDEX iMSD,
    uint8_t * commandStatus
);
USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_ProcessWrite
(
    SYS_MODULE_INDEX iMSD,
    uint8_t * commandStatus
);

// *****************************************************************************
/* Function:
    USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_VerifyCommand
    (
        SYS_MODULE_INDEX iMSD,
        uint8_t *commandStatus
    )

  Summary:
    Verifies CBW and checks the validity of the command.
 
  Description:

  Precondition:
    None.

  Parameters:
    iMSD                -  MSD instance index.
   
  Returns:
    MSD state.

  Remarks:

*/

USB_DEVICE_MSD_STATE _USB_DEVICE_MSD_VerifyCommand(SYS_MODULE_INDEX iMSD, uint8_t *commandStatus);


// *****************************************************************************
/* Function:
   USB_ERROR_STATUS _USB_DEVICE_MSD_Initialization ( SYS_MODULE_INDEX iMSD,
                                                 SYS_MODULE_INDEX iDriver,
                                                 SYS_MODULE_INDEX iUsbDevice,
                                                 void* funcDriverInit ,
                                                 uint8_t* pConfigDesc )

  Summary:
    Does MSD initialization.

  Description:
    

  Precondition:
    None.

  Parameters:
    iMSD                -  MSD instance index.
    iDriver             -  Receive/Transmit buffer address.
    iUsbDevice          -  Receive/Transmit buffer size.
    funcDriverInit      -  Command status.
    pConfigDesc         -  Pointer to configuration descriptor.

  Returns:
   Success state.

  Remarks:

*/

void _USB_DEVICE_MSD_InitializeByDescriptorType(SYS_MODULE_INDEX iMSD, DRV_HANDLE usbDeviceHandle,
                                            void* funcDriverInit, uint8_t intfNumber, uint8_t altSetting,
                                            uint8_t descriptorType, uint8_t * pDescriptor);


// *****************************************************************************
/* Function:
   USB_ERROR_STATUS _USB_DEVICE_MSD_Deinitialization ( SYS_MODULE_INDEX iMSD,
                                                 SYS_MODULE_INDEX iDriver,
                                                 SYS_MODULE_INDEX iUsbDevice,
                                                 void* funcDriverInit ,
                                                 uint8_t* pConfigDesc )

  Summary:
    Does MSD initialization.

  Description:
    

  Precondition:
    None.

  Parameters:
    iMSD                -  MSD instance index.
    iDriver             -  Receive/Transmit buffer address.
    iUsbDevice          -  Receive/Transmit buffer size.
    funcDriverInit      -  Command status.
    pConfigDesc         -  Pointer to configuration descriptor.

  Returns:
   Success state.

  Remarks:

*/

void _USB_DEVICE_MSD_Deinitialization ( SYS_MODULE_INDEX iMSD );


// *****************************************************************************
/* Function:
   USB_ERROR_STATUS _USB_DEVICE_MSD_CheckInterface ( SYS_MODULE_INDEX funcDriverIndex ,
                                                 uint16_t interfaceNumber )

  Summary:
    This function returns success if the interface number belongs to this instance
    of the function driver.

  Description:


  Precondition:
    None.

  Parameters:
    funcDriverindex - Instance index of MSD function driver.
    interfaceNumber - Interface number.

  Returns:
    USB_ERROR_OK if interface number belongs to this instance of the
    function driver.

  Remarks:

*/

USB_ERROR _USB_DEVICE_MSD_CheckInterface ( SYS_MODULE_INDEX funcDriverIndex ,
                                                 uint16_t interfaceNumber );


// *****************************************************************************
/* Function:
  void _USB_DEVICE_MSD_EndpointNotification(SYS_MODULE_INDEX MSDIndex,
                       USB_ENDPOINT epAddress, uint16_t epSize, uint8_t epType)

  Summary:
    This function is called by the device layer to inform the MSD
    about the endpoints that are opened for this particular instance
    of the MSD.
  Description:


  Precondition:
    None.

  Parameters:
    MSDIndex  - Instance index of MSD function driver.
    epAddress - Endpoint address and direction.
    epSize    - Endpoint size.
    epType    - Endpoint type.

  Returns:
     None.
  Remarks:

*/

void _USB_DEVICE_MSD_DescriptorNotification(SYS_MODULE_INDEX funcDriverIndex,
                                 uint8_t descriptorType, uint8_t * pDescriptor);


// *****************************************************************************
/* Function:
   bool _USB_DEVICE_MSD_ControlTransferHandler ( SYS_MODULE_INDEX iMSD ,
            SETUP_PKT *setupPkt, USB_DEVICE_CONTROL_TRANSFER_DATA_OBJECT * controlDataObject )

  Summary:
    MSD control transfer handler.

  Description:
    MSD control transfer handler. This is the callback the device layer calls
    when there is a setup packet that is targeted to this particular instance
    of MSD.

  Precondition:
    None.

  Parameters:
    iMSD          -  Instance index of MSD function driver.
    setupPkt      -  Pointer to setup packet buffer.
    controlDataObject - Control transfer data object.

  Returns:
    None.

  Remarks:

*/

void _USB_DEVICE_MSD_ControlTransferHandler
(
    SYS_MODULE_INDEX MSDIndex,
    USB_DEVICE_EVENT transferState,
    USB_SETUP_PACKET * setupPkt
);


// *****************************************************************************
/* Function:
   void _USB_DEVICE_MSD_Tasks ( SYS_MODULE_INDEX iMSD )

  Summary:
   MSD function driver tasks.

  Description:


  Precondition:
    None.

  Parameters:
    iMSD          -  Instance index of MSD function driver.
    
  Returns:
    None.

  Remarks:

*/

void _USB_DEVICE_MSD_Tasks ( SYS_MODULE_INDEX iMSD );

/******************************************************************************
  Function:
    void _USB_DEVICE_MSD_CallBackBulkTxTransfer( void *  handle )

  Summary:
    This is a callback function that gets called by controller driver,
    after the completion of bulk-in transfer.

  Description:

  Precondition:
    None.

  Parameters:
    hMsdInstance        : Handle to MSD instance.
    hPipe               : Handle to pipe.
    hTransfer           : Handle to transfers.
    transferByteCount   : Total byte count.
    statusTransfer      : Status of the transfer.

  Returns:
    None.


  Remarks:
    None
*/

void _USB_DEVICE_MSD_CallBackBulkTxTransfer( USB_DEVICE_IRP *  handle );


#endif

