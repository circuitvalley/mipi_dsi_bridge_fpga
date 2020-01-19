/*******************************************************************************
  USB Device Driver Mapping File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usb_mapping.h

  Summary:
    USB Device Driver Mapping File

  Description:
    The USB Device Driver mapping file maps the prototypes in "drv_usb.h" to 
    static variants of these routines appropriate for the selected configuration. 
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip  Technology  Inc.   All  rights  reserved.

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
//DOM-IGNORE-END

#ifndef _DRV_USBFS_MAPPING_H
#define _DRV_USBFS_MAPPING_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  See the bottom of file for implementation header include files.
*/

#include <stdint.h>
#include <stdbool.h>

#include "system_config.h"


// *****************************************************************************
// *****************************************************************************
// Section: USB Driver Static API Name Generation
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Macro: _DRV_USBFS_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static interface name

  Description:
     This macro creates the instance-specific name of the given static interface
     routine by inserting the index number into the name.

     Example DRV_USBFS_Initialize becomes DRV_USBFS1_Initialize for an index of 1.

  Remarks:
*/

#if ((!defined(DRV_USBFS_INSTANCES_NUMBER)) && ((!defined(DRV_USB_INSTANCES_NUMBER))))

    /* If the DRV_USBFS_INSTANCES_NUMBER macro is not defined, then this
     * means that the static mode is desired. A USB driver instance
     * only supports one client. So a static build would always be
     * static single. */

    #define _DRV_USBFS_STATIC_API_SINGLE(index, name)     DRV_USBFS ## index ## _ ## name
    #define _DRV_USBFS_STATIC_API(index, name)            _DRV_USBFS_STATIC_API_SINGLE(index, name)
    #define _DRV_USBFS_MAKE_NAME(name)                    _DRV_USBFS_STATIC_API(DRV_USBFS_INDEX, name)

#else 

    /* This means that a dynamic driver is requested */
    #define _DRV_USBFS_MAKE_NAME(name)                         DRV_USBFS_ ## name

#endif



// *****************************************************************************
// *****************************************************************************
// Section: USB Driver Static API Mapping
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Static Interface Mapping

  Summary:
    Maps the dynamic interface calls to appropriate static interface

  Description:
    These macros map calls to the dynamic interface routines to calls to the 
    appropriate instance-specific static interface routine when a static build
    (DRV_USBFS_INSTANCES_NUMBER is not defined) is configured.
    
    Example:
    
        DRV_USBFS_Status(DRV_USBFS_INDEX_1);
        
    Becomes:
    
        DRV_USBFS_Status();
        
  Remarks:
    Static configuration eliminate the need to pass the object parameter.  
    However, the index is "returned" as the object handle (from the 
    "Initialize" routine) to allow code written for the dynamic API to continue
    to build when using a static configuration.
    
    Example:
    
        object = DRV_USBFS_Initialize(DRV_USBFS_INDEX_1, &initData);
        
    Becomes:
    
        object = ( DRV_USBFS1_Initialize(&initData), DRV_USBFS_INDEX );
        
    Static single-client configurations also eliminate the client handle 
    parameter.  However, the index (the driver-object index) is "returned" from
    the "Open" routine to allow code written for multi-client drivers to build
    when using a static single-open configuration.
    
    Example:
    
        handle = DRV_USBFS_Open(DRV_USBFS_INDEX_1, intent);
        
    Becomes:
    
        handle = ( DRV_USBFS_Open(intent), DRV_USBFS_INDEX );
*/

#if ((!defined(DRV_USBFS_INSTANCES_NUMBER)) && ((!defined(DRV_USB_INSTANCES_NUMBER))))

    #define _DRV_USBFS_DEVICE_ENDPOINT_ALL 17

    #define DRV_USBFS_Initialize(sysID, inData)           _DRV_USBFS_MAKE_NAME(Initialize)(inData)
    #define DRV_USBFS_Deinitialize(sysObj)                _DRV_USBFS_MAKE_NAME(Deinitialize)()
    #define DRV_USBFS_Status(sysObj)                      _DRV_USBFS_MAKE_NAME(Status)()
    #define DRV_USBFS_Tasks(sysObj)                       _DRV_USBFS_MAKE_NAME(Tasks)()
    #define DRV_USBFS_Tasks_ISR(sysObj)                   _DRV_USBFS_MAKE_NAME(Tasks_ISR)()
    #define DRV_USBFS_Tasks_ISR_USBDMA(sysObj)            _DRV_USBFS_MAKE_NAME(Tasks_ISR_USBDMA)()
    #define DRV_USBFS_Open(sysID, intent)                 _DRV_USBFS_MAKE_NAME(Open)(intent)
    #define DRV_USBFS_Close(handle)                       _DRV_USBFS_MAKE_NAME(Close)()
    #define DRV_USBFS_ClientEventCallBackSet(w, x, y)     _DRV_USBFS_MAKE_NAME(ClientEventCallBackSet)(x, y)
    #define DRV_USBFS_HOST_Suspend(handle)                _DRV_USBFS_MAKE_NAME(HOST_Suspend)()
    #define DRV_USBFS_HOST_Resume(handle)                 _DRV_USBFS_MAKE_NAME(HOST_Resume)()
	
	
    #define DRV_USBFS_DEVICE_AddressSet(handle, address)  _DRV_USBFS_MAKE_NAME(DEVICE_AddressSet)(address)
    #define DRV_USBFS_DEVICE_CurrentSpeedGet(handle)      _DRV_USBFS_MAKE_NAME(DEVICE_CurrentSpeedGet)()
    #define DRV_USBFS_DEVICE_Attach(handle)               _DRV_USBFS_MAKE_NAME(DEVICE_Attach)()
    #define DRV_USBFS_DEVICE_Detach(handle)               _DRV_USBFS_MAKE_NAME(DEVICE_Detach)()
	
    #define DRV_USBFS_DEVICE_EndpointEnable(w, x, y, z)   _DRV_USBFS_MAKE_NAME(DEVICE_EndpointEnable)(x, y, z)
    #define DRV_USBFS_DEVICE_EndpointDisable(w, x)        _DRV_USBFS_MAKE_NAME(DEVICE_EndpointDisable)(x)
    #define DRV_USBFS_DEVICE_EndpointIsEnabled(w, x)      _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsEnabled)(x)
    #define DRV_USBFS_DEVICE_EndpointIsStalled(w, x)      _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsStalled)(x)
	
    #define DRV_USBFS_DEVICE_IRPSubmit(w, x, y)           _DRV_USBFS_MAKE_NAME(DEVICE_IRPSubmit)(x, y)
    #define DRV_USBFS_DEVICE_IRPCancelAll(w, x)           _DRV_USBFS_MAKE_NAME(DEVICE_IRPCancelAll)(x)
    #define DRV_USBFS_DEVICE_EndpointStall(w, x)          _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStall)(x)
    #define DRV_USBFS_DEVICE_EndpointStallClear(w, x)     _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStallClear)(x)
    #define DRV_USBFS_DEVICE_EndpointDisableAll(w)        _DRV_USBFS_MAKE_NAME(DEVICE_EndpointDisable)(_DRV_USBFS_DEVICE_ENDPOINT_ALL)
	
    #define DRV_USBFS_DEVICE_RemoteWakeupStart(handle)   _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStart)()
    #define DRV_USBFS_DEVICE_RemoteWakeupStop(handle)    _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStop)()
    #define DRV_USBFS_DEVICE_SOFNumberGet(handle)         _DRV_USBFS_MAKE_NAME(DEVICE_SOFNumberGet)()
	
    

#else // Dynamic Build

    

    // No Change in the Naming convention

#endif



#endif
