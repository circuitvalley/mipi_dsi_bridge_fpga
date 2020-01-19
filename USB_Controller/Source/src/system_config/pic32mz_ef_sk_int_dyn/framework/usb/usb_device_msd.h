/*******************************************************************************
  USB MSD function driver interface header

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_msd.h

  Summary:
    USB device MSD function driver interface header

  Description:
    USB device MSD function driver interface header. This file should be
    included in the application if USB MSD functionality is required.
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

#ifndef _USB_DEVICE_MSD_H
#define _USB_DEVICE_MSD_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.
*/
#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "system/common/sys_module.h"
#include "system/fs/sys_fs_media_manager.h"
#include "driver/driver_common.h"
#include "usb/usb_common.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"
#include "usb/scsi.h"
#include "usb/usb_msd.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Types. This section is specific to PIC32 implementation
//          of the USB Device MSD Function Driver
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Media Driver Function Pointer Data Structure
  
  Summary:
    Pointer to the media driver functions for media instances to used with the
    the MSD function driver.
  
  Description:
    This structure contains function pointers, pointing to the media driver
    functions. The MSD function driver calls these functions at run time to
    access the media. This data structure should be specified during compilation
    and is a part of the MSD function driver initialization data structure. It
    is processed by the function driver when the function driver is initialized
    by the Device Layer. 

  Remarks:
    None.
*/

typedef struct USB_DEVICE_MSD_MEDIA_FUNCTIONS
{
    /* In case of pluggable media, such as SD Card, this function returns true
       when the media is inserted, initialized and ready to be used. In case of
       non-pluggable media, such as Internal Flash memory, this function can
       return true when the media is ready to be used. The MSD host may not
       detect the media until this function returns true. This function pointer
       cannot be NULL */
    
    bool (*isAttached) (const DRV_HANDLE handle);

    /* The MSD Function Driver calls this function to obtain a handle and gain
       access to functionality of the specified instance of the media driver.
       The MSD function driver will attempt to open the driver with
       DRV_IO_INTENT_READWRITE and DRV_IO_INTENT_NONBLOCKING. The MSD host may
       not detect the media until the MSD function driver obtains a valid driver
       handle. The function driver will use this handle in all other functions
       to communicate with the media driver. This function pointer cannot be
       NULL */
    
    DRV_HANDLE (* open) (const SYS_MODULE_INDEX index, const DRV_IO_INTENT  intent );

    /* The MSD function driver calls this function when the function driver
       gets deinitialized as result of device detach or a change in
       configuration. The MSD function driver will open the media driver again
       to obtain a fresh driver handle when it gets initialized again. This
       function pointer cannot be NULL.
       */
    
    void (* close)(DRV_HANDLE hClient );

    /* The MSD function driver calls this function when the function driver
       needs to know the storage capacity of the media. The MSD function driver
       uses the size of read region and number of read blocks to report the
       media capacity to the MSD host. This function pointer cannot be NULL. */

    SYS_FS_MEDIA_GEOMETRY * (* geometryGet)(DRV_HANDLE hClient);

    /* The MSD function driver calls this function when it needs to read a block
       of data. This function pointer cannot be NULL. */

    void (*blockRead)
    (
        DRV_HANDLE handle,
        uintptr_t * blockOperationHandle,
        void * data,
        uint32_t blockStart,
        uint32_t nBlocks
    );

    /* The MSD function driver calls this function when it needs to write a block
       of data. This function pointer can be NULL if the media is write protected. */

    void (*blockWrite)
    (
        DRV_HANDLE handle,
        uintptr_t * blockOperationHandle,
        void * data,
        uint32_t blockStart,
        uint32_t nBlocks
    );

    /* The MSD function driver calls this function to find out if the media is
       write-protected. This function pointer cannot be NULL. */

    bool  (*isWriteProtected)( DRV_HANDLE drvHandle );

    /* The MSD function driver calls this function to register an block event
       call back function with the media driver. This event call back will be
       called when a block related operation has completed. This function
       pointer should not be NULL. */

    void (*blockEventHandlerSet)
    (
       const DRV_HANDLE drvHandle, 
       const void * eventHandler,
       const uintptr_t context
    );

    /* If not NULL and if the blockStartAddress parameter in the
       USB_DEVICE_MSD_MEDIA_INIT_DATA data structure for this media is not 0,
       then the MSD function driver calls this function immediately after
       opening the media driver. For media such a NVM, where the storage media
       is a part of the program memory flash, this function sets the start of
       the storage area on the media. This function is not required for media
       such as SD Card. */ 

    void (*blockStartAddressSet)
    (
        const DRV_HANDLE drvHandle,
        const void * addressOfStartBlock
    );

} USB_DEVICE_MSD_MEDIA_FUNCTIONS;

// *****************************************************************************
/* USB Device MSD Media Initialization Data Member 
 
  Summary:
    This structure holds media related data of a particular logical unit.
   
  Description:
    It holds pointer to inquiry response, instance index and pointer to a
    structure that contains  all media callback functions.

  Remarks:
    An object of this structure must be configured by the user at compile time.
*/

typedef struct 
{
    /* Instance index of the media driver to opened for this LUN */
    SYS_MODULE_INDEX instanceIndex;

    /* Sector size for this LUN. If 0, means that sector size will be available
       from media geometry. */
    uint32_t sectorSize;

    /* Pointer to a bye buffer whose size if the size of the sector on this
     * media. In case of a PIC32MZ device, this buffer should be coherent and
     * should be aligned on a 16 byte boundary */
    uint8_t * sectorBuffer;

    /* In a case where the sector size of this media is less than the size of
     * the write block, a byte buffer of write block size should be provided to
     * the function driver. For example, the PIC32MZ NVM flash driver has a
     * flash program memory row size of 4096 bytes which is more than the
     * standard 512 byte sector. In such a case the application should set this
     * pointer to 4096 byte buffer */
    uint8_t * blockBuffer; 

    /* Block 0 Start Address on this media. If non zero, then this address will
       be passed to blockStartAddressSet function. This should be set to start
       of the storage address on the media. */
    void * block0StartAddress;

    /* Pointer to SCSI inquiry response for this LUN */
    SCSI_INQUIRY_RESPONSE inquiryResponse;

    /* Function pointers to the media driver functions */
    USB_DEVICE_MSD_MEDIA_FUNCTIONS mediaFunctions;

} USB_DEVICE_MSD_MEDIA_INIT_DATA;

// *****************************************************************************
/* USB MSD init structure.

  Summary:
    This structure contains required parameters for MSD function driver
    initialization.

  Description:
    This structure contains interface number, bulk-IN and bulk-OUT endpoint
    addresses, endpointSize, number of logical units supported and pointer to
    array of structure that contains media initialization.

  Remarks:
    This structure must be configured by the user at compile time.
*/

typedef struct 
{
    /* Number of logical units supported. */
    uint8_t  numberOfLogicalUnits;

    /* Pointer to a Command Block Wrapper structure allocated to this instance
     * of the MSD function driver. In case of PIC32MZ device, this should be
     * placed in non cacheable section of RAM and should be aligned at a 4 byte
     * boundary. */
    USB_MSD_CBW * msdCBW;

    /* Pointer to a Command Status Wrapper structure allocated to this instance
     * of the MSD function driver. In case of PIC32MZ device, this should be
     * placed in non cacheable section of RAM and should be aligned at a 4 byte
     * boundary. */
    USB_MSD_CSW * msdCSW;

    /* Pointer to a table of media initialization data. This should contain an
       entry for every logical unit. */
    USB_DEVICE_MSD_MEDIA_INIT_DATA * mediaInit;

} USB_DEVICE_MSD_INIT;

// *****************************************************************************
/* USB Device MSD Function Driver Function Pointer

  Summary:
    USB Device MSD Function Driver Function pointer

  Description:
    This is the USB Device MSD Function Driver Function pointer. This should
    registered with the device layer in the function driver registration table.

  Remarks:
    None.
*/

/*DOM-IGNORE-BEGIN*/ extern USB_DEVICE_FUNCTION_DRIVER msdFunctionDriver ; /*DOM-IGNORE-END*/
#define USB_DEVICE_MSD_FUNCTION_DRIVER /*DOM-IGNORE-BEGIN*/&msdFunctionDriver/*DOM-IGNORE-END*/

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _USB_DEVICE_MSD_H

/*******************************************************************************
 End of File
*/


