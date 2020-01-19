/*******************************************************************************
  USB Device layer interface names mapping

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_mapping.h

  Summary:
    USB Device Layer Interface names mapping

  Description:
    This header file maps the interface prototypes in "usb_device.h" to static
    variants of these routines appropriate for the selected configuration.
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

#ifndef _USB_DEVICE_MAPPING_H
#define _USB_DEVICE_MAPPING_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  See the bottom of file for implementation header include files.
*/

#include <stdint.h>
#include <stdbool.h>
#include "usb/src/usb_device_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: USB Driver API Mapping to USB device layer API
// *****************************************************************************
// *****************************************************************************
#define USB_CD_HANDLE_mac (((USB_DEVICE_OBJ *)handle)->usbCDHandle)

#ifndef USB_DEVICE_DRIVER_INITIALIZE_EXPLICIT
    #warning USB Device Mode Driver must be initialized explicitly. Support for implicit initialization will be disabled in a future release.
#endif 

#if defined ( USB_DEVICE_DRIVER_INITIALIZE_EXPLICIT)

    /* This mean the device layer must access the driver API through
     * function pointers */

#define USB_DEVICE_DRIVER_INIT()   
#define USB_DEVICE_DRIVER_SELF_INITIALIZE
#define USB_DEVICE_DRIVER_OPEN() usbDeviceThisInstance->driverInterface->open( usbDeviceThisInstance->driverIndex, DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE)
#define DRV_USB_INIT                                    
#define DRV_USB_Tasks(x)
#define DRV_USB_Tasks_ISR(x)                            
#define DRV_USB_Tasks_ISR_DMA(x)
#define DRV_USB_ClientEventCallBackSet(x,y,z)           devClientHandle->driverInterface->eventHandlerSet(x,y,z)
#define DRV_USB_DEVICE_IRPSubmit(x,y,z)                 usbDeviceThisInstance->driverInterface->deviceIRPSubmit(x,y,z)
#define DRV_USB_DEVICE_EndpointStall(x,y)               usbDeviceThisInstance->driverInterface->deviceEndpointStall(x,y)
#define DRV_USB_DEVICE_RemoteWakeupStop(x)              usbDeviceThisInstance->driverInterface->deviceRemoteWakeupStop(x)
#define DRV_USB_DEVICE_RemoteWakeupStart(x)             usbDeviceThisInstance->driverInterface->deviceRemoteWakeupStart(x)
#define DRV_USB_DEVICE_AddressSet(x,y)                  usbDeviceThisInstance->driverInterface->deviceAddressSet(x,y)
#define DRV_USB_DEVICE_TestModeEnter(x,y)               usbDeviceThisInstance->driverInterface->deviceTestModeEnter(x,y)
#define DRV_USB_DEVICE_IRPCancel(x,y)                   usbDeviceThisInstance->driverInterface->deviceIRPCancelAll(x,y)
#define DRV_USB_DEVICE_IRPCancelAll(x,y)                usbDeviceThisInstance->driverInterface->deviceIRPCancelAll(x,y)
#define DRV_USB_DEVICE_EndpointDisableAll(x)            usbDeviceThisInstance->driverInterface->deviceEndpointDisable(x, DRV_USB_DEVICE_ENDPOINT_ALL)
#define DRV_USB_DEVICE_EndpointEnable(w,x,y,z)          usbDeviceThisInstance->driverInterface->deviceEndpointEnable(w,x,y,z)
#define DRV_USB_DEVICE_CurrentSpeedGet(x)               usbDeviceThisInstance->driverInterface->deviceCurrentSpeedGet(x)
#define DRV_USB_DEVICE_SOFNumberGet(x)                  usbDeviceThisInstance->driverInterface->deviceSOFNumberGet(x)
#define DRV_USB_DEVICE_EndpointIsStalled(x,y)           usbDeviceThisInstance->driverInterface->deviceEndpointIsStalled(x,y)
#define DRV_USB_DEVICE_EndpointStallClear(x,y)          usbDeviceThisInstance->driverInterface->deviceEndpointStallClear(x,y)
#define USB_DEVICE_EndpointEnable( v, w, x, y, z )      ((USB_DEVICE_OBJ *)v)->driverInterface->deviceEndpointEnable( (((USB_DEVICE_OBJ *)v)->usbCDHandle), x, y, z )
#define USB_DEVICE_EndpointDisable( x, y )              ((USB_DEVICE_OBJ *)x)->driverInterface->deviceEndpointDisable( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointStall( x, y )                ((USB_DEVICE_OBJ *)x)->driverInterface->deviceEndpointStall((((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointStallClear( x, y )           ((USB_DEVICE_OBJ *)x)->driverInterface->deviceEndpointStallClear( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointIsEnabled( x, y )            ((USB_DEVICE_OBJ *)x)->driverInterface->deviceEndpointIsEnabled( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointIsStalled( x, y )            ((USB_DEVICE_OBJ *)x)->driverInterface->deviceEndpointIsStalled( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_IRPSubmit( x, y, irp )               ((USB_DEVICE_OBJ *)x)->driverInterface->deviceIRPSubmit( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y, irp )
#define USB_DEVICE_IRPCancel( x, y )                    ((USB_DEVICE_OBJ *)x)->driverInterface->deviceIRPCancel( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_IRPCancelAll( x, y )                 ((USB_DEVICE_OBJ *)x)->driverInterface->deviceIRPCancelAll( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_Attach( x )                          ((USB_DEVICE_OBJ *)x)->driverInterface->deviceAttach( ((USB_DEVICE_OBJ *)(x))->usbCDHandle)
#define USB_DEVICE_Detach( x )                          ((USB_DEVICE_OBJ *)x)->driverInterface->deviceDetach( ((USB_DEVICE_OBJ *)x)->usbCDHandle)


#elif defined (__PIC32MX__) || defined (__PIC32MK__) ||defined(__PIC32WK)

    /* Before v1.04, the USB driver API was not driver specific and this would
     * not have allowed us to include 2 different drivers in one application. In
     * v1.04 onwards the driver API will be controller specific. The following
     * mapping will map the driver DRV_USB API to DRV_USBFS */

#include "driver/usb/usbfs/drv_usbfs.h"

#define USB_DEVICE_ENDPOINT_TABLE_SIZE (DRV_USB_ENDPOINTS_NUMBER * DRV_USBFS_ENDPOINT_TABLE_ENTRY_SIZE)
#define USB_DEVICE_DRIVER_SELF_INITIALIZE  DRV_USBFS_INIT drvUsbInit;
#define USB_DEVICE_DRIVER_OPEN() DRV_USB_Open( devLayerObj, DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE)
#define USB_DEVICE_DRIVER_INIT()   \
    drvUsbInit.interruptSource  = deviceInit->interruptSource ;\
    drvUsbInit.moduleInit       = deviceInit->moduleInit ;\
    drvUsbInit.operationMode    = DRV_USB_OPMODE_DEVICE ;\
    drvUsbInit.stopInIdle       = deviceInit->stopInIdle ;\
    drvUsbInit.suspendInSleep   = deviceInit->suspendInSleep ;\
    drvUsbInit.usbID            = deviceInit->usbID ;\
    drvUsbInit.operationSpeed   = deviceInit->deviceSpeed ;\
    drvUsbInit.endpointTable    = deviceInit->endpointTable;\
    usbDeviceThisInstance->usbCDSystemModuleObject = DRV_USB_Initialize(index, (SYS_MODULE_INIT *)&drvUsbInit);\
    if(usbDeviceThisInstance->usbCDSystemModuleObject == SYS_MODULE_OBJ_INVALID)\
    {\
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "USB Device Layer: Could not initialize USBCD");\
        return (SYS_MODULE_OBJ_INVALID);\
    }

#define DRV_USB_Initialize(x,y)                         DRV_USBFS_Initialize(x,y)
#define DRV_USB_Tasks(x)                                DRV_USBFS_Tasks(x)
#define DRV_USB_Open(x,y)                               DRV_USBFS_Open(x,y)
#define DRV_USB_Tasks_ISR(x)                            DRV_USBFS_Tasks_ISR(x)
#define DRV_USB_Tasks_ISR_DMA(x)                        DRV_USBFS_Tasks_ISR_DMA(x)
#define DRV_USB_ClientEventCallBackSet(x,y,z)           DRV_USBFS_ClientEventCallBackSet(x,y,z)
#define DRV_USB_DEVICE_IRPSubmit(x,y,z)                 DRV_USBFS_DEVICE_IRPSubmit(x,y,z)
#define DRV_USB_DEVICE_EndpointStall(x,y)               DRV_USBFS_DEVICE_EndpointStall(x,y)
#define DRV_USB_DEVICE_RemoteWakeupStop(x)              DRV_USBFS_DEVICE_RemoteWakeupStop(x)
#define DRV_USB_DEVICE_RemoteWakeupStart(x)             DRV_USBFS_DEVICE_RemoteWakeupStart(x)
#define DRV_USB_DEVICE_AddressSet(x,y)                  DRV_USBFS_DEVICE_AddressSet(x,y)
#define DRV_USB_DEVICE_IRPCancel(x,y)                   DRV_USBFS_DEVICE_IRPCancel(x,y)
#define DRV_USB_DEVICE_IRPCancelAll(x,y)                DRV_USBFS_DEVICE_IRPCancelAll(x,y)
#define DRV_USB_DEVICE_EndpointDisableAll(x)            DRV_USBFS_DEVICE_EndpointDisable(x, DRV_USBFS_DEVICE_ENDPOINT_ALL)
#define DRV_USB_DEVICE_EndpointEnable(w,x,y,z)          DRV_USBFS_DEVICE_EndpointEnable(w,x,y,z)
#define DRV_USB_DEVICE_CurrentSpeedGet(x)               DRV_USBFS_DEVICE_CurrentSpeedGet(x)
#define DRV_USB_DEVICE_SOFNumberGet(x)                  DRV_USBFS_DEVICE_SOFNumberGet(x)
#define DRV_USB_DEVICE_EndpointIsStalled(x,y)           DRV_USBFS_DEVICE_EndpointIsStalled(x,y)
#define DRV_USB_DEVICE_EndpointStallClear(x,y)          DRV_USBFS_DEVICE_EndpointStallClear(x,y)
#define USB_DEVICE_EndpointEnable( v, w, x, y, z )      DRV_USBFS_DEVICE_EndpointEnable( (((USB_DEVICE_OBJ *)v)->usbCDHandle), x, y, z )
#define USB_DEVICE_EndpointDisable( x, y )              DRV_USBFS_DEVICE_EndpointDisable( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointStall( x, y )                DRV_USBFS_DEVICE_EndpointStall((((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointStallClear( x, y )           DRV_USBFS_DEVICE_EndpointStallClear( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointIsEnabled( x, y )            DRV_USBFS_DEVICE_EndpointIsEnabled( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointIsStalled( x, y )            DRV_USBFS_DEVICE_EndpointIsStalled( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_IRPSubmit( x, y, irp )               DRV_USBFS_DEVICE_IRPSubmit( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y, irp )
#define USB_DEVICE_IRPCancel( x, y )                    DRV_USBFS_DEVICE_IRPCancel( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_IRPCancelAll( x, y )                 DRV_USBFS_DEVICE_IRPCancelAll( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_Attach( x )                          DRV_USBFS_DEVICE_Attach( ((USB_DEVICE_OBJ *)(x))->usbCDHandle)
#define USB_DEVICE_Detach( x )                          DRV_USBFS_DEVICE_Detach( ((USB_DEVICE_OBJ *)x)->usbCDHandle)
#define DRV_USB_DEVICE_TestModeEnter(x,y)      
#define DRV_USB_DEVICE_TestModeExit(x,y)                         

#elif defined (__PIC32MZ__)
    

    /* Before v1.04, the USB driver API was not driver specific and this would
     * not have allowed us to include 2 different drivers in one application. In
     * v1.04 onwards the driver API will be controller specific. The following
     * mapping will map the driver DRV_USB API to DRV_USBHS */

#include "driver/usb/usbhs/drv_usbhs.h"
#define USB_DEVICE_ENDPOINT_TABLE_SIZE (0)
#define USB_DEVICE_DRIVER_OPEN() DRV_USB_Open( devLayerObj, DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE)
#define USB_DEVICE_DRIVER_SELF_INITIALIZE  DRV_USBHS_INIT drvUsbInit;
#define USB_DEVICE_DRIVER_INIT()   \
    drvUsbInit.interruptSource  = deviceInit->interruptSource ;\
    drvUsbInit.moduleInit       = deviceInit->moduleInit ;\
    drvUsbInit.interruptSourceUSBDma = deviceInit->interruptSourceUSBDma;\
    drvUsbInit.operationMode    = DRV_USB_OPMODE_DEVICE ;\
    drvUsbInit.stopInIdle       = deviceInit->stopInIdle ;\
    drvUsbInit.suspendInSleep   = deviceInit->suspendInSleep ;\
    drvUsbInit.usbID            = deviceInit->usbID ;\
    drvUsbInit.operationSpeed   = deviceInit->deviceSpeed ;\
    drvUsbInit.endpointTable    = deviceInit->endpointTable;\
    usbDeviceThisInstance->usbCDSystemModuleObject = DRV_USB_Initialize(index, (SYS_MODULE_INIT *)&drvUsbInit);\
    if(usbDeviceThisInstance->usbCDSystemModuleObject == SYS_MODULE_OBJ_INVALID)\
    {\
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "USB Device Layer: Could not initialize USBCD");\
        return (SYS_MODULE_OBJ_INVALID);\
    }

#define DRV_USB_INIT                                    DRV_USBHS_INIT
#define DRV_USB_Initialize(x,y)                         DRV_USBHS_Initialize(x,y)
#define DRV_USB_Tasks(x)                                DRV_USBHS_Tasks(x)
#define DRV_USB_Open(x,y)                               DRV_USBHS_Open(x,y)
#define DRV_USB_Tasks_ISR(x)                            DRV_USBHS_Tasks_ISR(x)
#define DRV_USB_Tasks_ISR_DMA(x)                        DRV_USBHS_Tasks_ISR_USBDMA(x)
#define DRV_USB_ClientEventCallBackSet(x,y,z)           DRV_USBHS_ClientEventCallBackSet(x,y,z)
#define DRV_USB_DEVICE_IRPSubmit(x,y,z)                 DRV_USBHS_DEVICE_IRPSubmit(x,y,z)
#define DRV_USB_DEVICE_EndpointStall(x,y)               DRV_USBHS_DEVICE_EndpointStall(x,y)
#define DRV_USB_DEVICE_RemoteWakeupStop(x)              DRV_USBHS_DEVICE_RemoteWakeupStop(x)
#define DRV_USB_DEVICE_RemoteWakeupStart(x)             DRV_USBHS_DEVICE_RemoteWakeupStart(x)
#define DRV_USB_DEVICE_AddressSet(x,y)                  DRV_USBHS_DEVICE_AddressSet(x,y)
#define DRV_USB_DEVICE_IRPCancel(x,y)                   DRV_USBHS_DEVICE_IRPCancel(x,y)
#define DRV_USB_DEVICE_IRPCancelAll(x,y)                DRV_USBHS_DEVICE_IRPCancelAll(x,y)
#define DRV_USB_DEVICE_EndpointDisableAll(x)            DRV_USBHS_DEVICE_EndpointDisable(x, DRV_USBHS_DEVICE_ENDPOINT_ALL)
#define DRV_USB_DEVICE_EndpointEnable(w,x,y,z)          DRV_USBHS_DEVICE_EndpointEnable(w,x,y,z)
#define DRV_USB_DEVICE_CurrentSpeedGet(x)               DRV_USBHS_DEVICE_CurrentSpeedGet(x)
#define DRV_USB_DEVICE_SOFNumberGet(x)                  DRV_USBHS_DEVICE_SOFNumberGet(x)
#define DRV_USB_DEVICE_EndpointIsStalled(x,y)           DRV_USBHS_DEVICE_EndpointIsStalled(x,y)
#define DRV_USB_DEVICE_EndpointStallClear(x,y)          DRV_USBHS_DEVICE_EndpointStallClear(x,y)
#define USB_DEVICE_EndpointEnable( v, w, x, y, z )      DRV_USBHS_DEVICE_EndpointEnable( (((USB_DEVICE_OBJ *)v)->usbCDHandle), x, y, z )
#define USB_DEVICE_EndpointDisable( x, y )              DRV_USBHS_DEVICE_EndpointDisable( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointStall( x, y )                DRV_USBHS_DEVICE_EndpointStall((((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointStallClear( x, y )           DRV_USBHS_DEVICE_EndpointStallClear( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointIsEnabled( x, y )            DRV_USBHS_DEVICE_EndpointIsEnabled( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_EndpointIsStalled( x, y )            DRV_USBHS_DEVICE_EndpointIsStalled( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_IRPSubmit( x, y, irp )               DRV_USBHS_DEVICE_IRPSubmit( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y, irp )
#define USB_DEVICE_IRPCancel( x, y )                    DRV_USBHS_DEVICE_IRPCancel( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_IRPCancelAll( x, y )                 DRV_USBHS_DEVICE_IRPCancelAll( (((USB_DEVICE_OBJ *)x)->usbCDHandle), y )
#define USB_DEVICE_Attach( x )                          DRV_USBHS_DEVICE_Attach( ((USB_DEVICE_OBJ *)(x))->usbCDHandle)
#define USB_DEVICE_Detach( x )                          DRV_USBHS_DEVICE_Detach( ((USB_DEVICE_OBJ *)x)->usbCDHandle)
#define DRV_USB_DEVICE_TestModeEnter(x,y)               DRV_USBHS_DEVICE_TestModeEnter(x,y)
#define DRV_USB_DEVICE_TestModeExit(x,y)                DRV_USBHS_DEVICE_TestModeExit(x,y)
#endif

/* The following macros re-map USB Device Layer calls directly to the USB
 * driver. */



#if defined USB_DEVICE_SOF_EVENT_ENABLE
    #define _USB_DEVICE_SOFEventEnable()  eventType
#else
    #define _USB_DEVICE_SOFEventEnable()  0
#endif


#ifdef USB_DEVICE_ENDPOINT_QUEUE_DEPTH_COMBINED
    /* This means that the application has included the endpoint functions and
     * is most probably implementing a custom device. We need the following
     * functions to be included in the code. */
    #define _USB_DEVICE_Initialize_Endpoint_Q(x,y,z) _USB_DEVICE_Initialize_Endpoint_Q_Size(x,y,z)
    #define _USB_DEVICE_EndpointCurrentQueueSizeReset(x)  _USB_DEVICE_EndpointQueueSizeReset(x)
    #define _USB_DEVICE_EndpointMutexCreate(x)       _USB_DEVICE_EndpointMutexCreateFunction(x)
    #define _USB_DEVICE_EndpointMutexDelete(x)       _USB_DEVICE_EndpointMutexDeleteFunction(x)
#else
    /* If the endpoint functions are not called in the code, then the following
     * function are not needed and map to nothing */
    #define _USB_DEVICE_Initialize_Endpoint_Q(x,y,z)
    #define _USB_DEVICE_EndpointMutexCreate(x)
    #define _USB_DEVICE_EndpointCurrentQueueSizeReset(x)
    #define _USB_DEVICE_EndpointMutexDelete(x)
    #define _USB_DEVICE_EndpointDeclareOsalResult(x)
#endif 

#ifdef USB_DEVICE_SET_DESCRIPTOR_EVENT_ENABLE
    #define _USB_DEVICE_Handle_Set_Descriptor_Request(x,y,z) _USB_DEVICE_RedirectControlXfrToClient(x,y,z)
#else
    #define _USB_DEVICE_Handle_Set_Descriptor_Request(x,y,z) sendStatus = true
#endif

#ifdef USB_DEVICE_SYNCH_FRAME_EVENT_ENABLE
    #define _USB_DEVICE_Handle_Synch_Frame_Request(x,y,z) _USB_DEVICE_ForwardControlXfrToFunction(x,y,z)
#else
    #define _USB_DEVICE_Handle_Synch_Frame_Request(x,y,z) USB_DEVICE_ControlStatus((USB_DEVICE_HANDLE)x, USB_DEVICE_CONTROL_STATUS_ERROR )
#endif

#ifdef USB_DEVICE_BOS_DESCRIPTOR_SUPPORT_ENABLE 
    #define _USB_DEVICE_GetBosDescriptorRequest(pBosDesc, data, size)  if (pBosDesc != NULL)\
                                                                                 {\
                                                                                     data = (uint8_t *)(pBosDesc);\
                                                                                     size = ((USB_BOS_DESCRIPTOR *)pBosDesc)->wTotalLength;\
                                                                                 }\

#else
    #define _USB_DEVICE_GetBosDescriptorRequest(x, y, z)
#endif

#ifdef USB_DEVICE_STRING_DESCRIPTOR_TABLE_ADVANCED_ENABLE
    #define _USB_DEVICE_GetStringDescriptorRequest(x,y,z)  _USB_DEVICE_GetStringDescriptorRequestProcessAdvanced(x,y,z)
#else
    #define _USB_DEVICE_GetStringDescriptorRequest(x,y,z) _USB_DEVICE_GetStringDescriptorRequestProcess(x,y,z)
#endif

#ifdef USB_DEVICE_MICROSOFT_OS_DESCRIPTOR_SUPPORT_ENABLE
	#define _USB_DEVICE_VendorInterfaceRequestProcess(x,y,z)  _USB_DEVICE_RedirectControlXfrToClient(x,USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST,z);
#else
	#define _USB_DEVICE_VendorInterfaceRequestProcess(x,y,z)  _USB_DEVICE_ForwardControlXfrToFunction(x,y,z)
#endif 

// *****************************************************************************
// *****************************************************************************
// Section: USB Device Descriptor Macros. 
// *****************************************************************************
// *****************************************************************************

/* The USB_DEVICE_16bitTo8bitArrange() macro is implemented for convenience.
   Since the configuration descriptor array is a uint8_t array, each entry 
   needs to be a uint8_t in LSB format.  The USB_DEVICE_16bitTo8bitArrange()
   macro breaks up a uint16_t into the appropriate uint8_t entries in LSB.
    Typical Usage:
    <code>
        const uint8_t configDescriptor1[]=
        {
            0x09,                           // Size of this descriptor in bytes
            USB_DESCRIPTOR_CONFIGURATION,   // CONFIGURATION descriptor type
            USB_DEVICE_16bitTo8bitArrange(0x0022), // Total length of data for
                                                    // this cfg  
    </code>
*/
#define USB_DEVICE_16bitTo8bitArrange(a) (a&0xFF),((a>>8)&0xFF)

/* The USB_DEVICE_32bitTo8bitArrange() macro is implemented for 
   convenience.  Since the configuration descriptor array is a uint8_t 
   array, each entry needs to be a uint8_t in LSB format.  The 
   USB_DEVICE_32bitTo8bitArrange() macro breaks up a uint32_t into
   the appropriate uint8_t entries in LSB.
*/
#define USB_DEVICE_32bitTo8bitArrange(a) (a&0xFF),((a>>8)&0xFF),((a>>16)&0xFF),((a>>24)&0xFF)

/* The USB_DEVICE_8bitArrange() macro is implemented for convenience.
   The USB_DEVICE_8bitArrange() macro provides a consistent macro for
   use with a byte when generating a configuration descriptor when using either
   the USB_DEVICE_16bitTo8bitArrange() or USB_DEVICE_32bitTo8bitArrange() 
   macros.
*/
#define USB_DEVICE_8bitArrange(a) (a)

#endif // #ifndef _USB_DEVICE_MAPPING_H

/*******************************************************************************
 End of File
*/

