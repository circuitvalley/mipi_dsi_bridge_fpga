/*******************************************************************************
  USB Driver Feature Variant Implementations

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhs_variant_mapping.h

  Summary:
    USB Driver Feature Variant Implementations

  Description:
    This file implements the functions which differ based on different parts
    and various implementations of the same feature.
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

#ifndef _DRV_USBHS_VARIANT_MAPPING_H
#define _DRV_USBHS_VARIANT_MAPPING_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"

/**********************************************
 * Macro Mapping
 **********************************************/

/* With v1.04 the USB Driver implementation has been been split such
 * multiple USB Driver can be included in the same application. But to
 * continue support for application developed before v1.04, we should
 * map the DRV_USB configuration macros to DRV_USBHS macros */

#if defined(DRV_USB_INSTANCES_NUMBER)
    #define DRV_USBHS_INSTANCES_NUMBER  DRV_USB_INSTANCES_NUMBER
#endif

#if defined (DRV_USB_INTERRUPT_MODE)
    #define DRV_USBHS_INTERRUPT_MODE DRV_USB_INTERRUPT_MODE
#endif

#if defined(DRV_USB_DEVICE_SUPPORT)
    #define DRV_USBHS_DEVICE_SUPPORT  DRV_USB_DEVICE_SUPPORT
#endif

#if defined(DRV_USB_HOST_SUPPORT)
    #define DRV_USBHS_HOST_SUPPORT  DRV_USB_HOST_SUPPORT
#endif

#if defined(DRV_USB_ENDPOINTS_NUMBER)
    #define DRV_USBHS_ENDPOINTS_NUMBER  DRV_USB_ENDPOINTS_NUMBER
#endif

/************************************************
 * This version of the driver does not support
 * mode.
 ***********************************************/

#if ((!defined(DRV_USBHS_INSTANCES_NUMBER)) && ((!defined(DRV_USB_INSTANCES_NUMBER))))

    #if defined(DRV_USBHS_CLIENTS_NUMBER)

    /* Multi client operation in static is not supported */

    #error "This driver does not support static multi-client operation"
    
    #else
    
        /* Map internal macros and functions to the static 
         * single open variant */
        
        #error "This version of the driver does not support static build"

    #endif

    #define _DRV_USBHS_IS_STATIC

#else 

    /* This means that dynamic operation is requested */

    #define _DRV_USBHS_IS_DYNAMIC

#endif

// *****************************************************************************
// *****************************************************************************
// Section: USB Driver Static Object Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_USBHS_OBJ_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static object name.

  Description:
     This macro creates the instance-specific name of the specified static object
     by inserting the index number into the name.

  Remarks:
    This macro does not affect the dynamic objects.
*/

#define _DRV_STATIC_OBJ_NAME_B(name,id)     name ## id
#define _DRV_STATIC_OBJ_NAME_A(name,id)     _DRV_STATIC_OBJ_NAME_B(name,id)
#define _DRV_USBHS_OBJ_MAKE_NAME(name)        _DRV_STATIC_OBJ_NAME_A(name, DRV_USBHS_INDEX)

/**********************************************
 * These macros allow variables to be compiled
 * based on dynamic or static buil.
 *********************************************/

#ifdef _DRV_USBHS_IS_STATIC

    #define _DRV_USBHS_FOR_DYNAMIC( type, object )
    #define _DRV_USBHS_FOR_STATIC( type, object )     type object

#endif

#ifdef _DRV_USBHS_IS_DYNAMIC

    #define _DRV_USBHS_FOR_DYNAMIC( type, object )    type object
    #define _DRV_USBHS_FOR_STATIC( type, object )

#endif

// *****************************************************************************
/* Interrupt Source Control

  Summary:
    Macros to enable, disable or clear the interrupt source

  Description:
    These macros enable, disable, or clear the interrupt source.

    The macros get mapped to the respective SYS module APIs if the configuration
    option DRV_USBHS_INTERRUPT_MODE is set to true.

  Remarks:
    These macros are mandatory.
*/

#if ((!defined(DRV_USBHS_INTERRUPT_MODE)) && ((!defined(DRV_USB_INTERRUPT_MODE))))
    #error "Interrupt mode must be defined and must be either true or false"
#endif

/* For PIC32MZ EF and DA device, the USB global interrupt in USBCRCON register
 * must be enabled. This is not required for PIC32MZ EC Devices. Additionally
 * the module reset time is 3 milliseconds for the former but 3 seconds for
 * the latter. */

#if (((__PIC32_FEATURE_SET0 == 68) && (__PIC32_FEATURE_SET1 == 65)) || ((__PIC32_FEATURE_SET0 == 69) && (__PIC32_FEATURE_SET1 == 70)))

/* For PIC32MZ DA and EF devices*/
#define _DRV_USBHS_CLOCK_CONTROL_GLOBAL_USB_INT_ENABLE(x) PLIB_USBHS_GlobalInterruptEnable(x)
#define _DRV_USBHS_MODULE_RESET_DURATION 3

#else

/* For PIC32MZ EC devices*/

#define _DRV_USBHS_CLOCK_CONTROL_GLOBAL_USB_INT_ENABLE(x) 
#define _DRV_USBHS_MODULE_RESET_DURATION 3000

#endif

/* On PIC32MZ DA devices, the interrupt are not persistent. The 68 and 65 are
 * ASCII codes for D and A respectively. */
#if ((__PIC32_FEATURE_SET0 == 68) && (__PIC32_FEATURE_SET1 == 65))

    /* This means the code is being compiled for a DA device */
    #define _DRV_USBHS_ID_OVERRIDE_IS_NEEDED              true
    #define _DRV_USBHS_INTERRUPT_PERSISTENT               false
    
    /* The DA device USB module also has a clock control register that needs to
     * be configured depending on whether the module is being setup for device
     * mode or host mode operation. */
    #define _DRV_USBHS_CLOCK_CONTROL_SETUP_DEVICE_MODE  PLIB_USBHS_USBIDOverrideEnable(usbID);\
                                                        PLIB_USBHS_PhyIDMonitoringEnable(usbID);\
                                                        PLIB_USBHS_USBIDOverrideValueSet(usbID, USBHS_USBID_ENABLE);

    #define _DRV_USBHS_CLOCK_CONTROL_SETUP_HOST_MODE  PLIB_USBHS_USBIDOverrideEnable(usbID);\
                                                        PLIB_USBHS_PhyIDMonitoringEnable(usbID);\
                                                        PLIB_USBHS_USBIDOverrideValueSet(usbID, USBHS_USBID_DISABLE);


#else
    #define _DRV_USBHS_ID_OVERRIDE_IS_NEEDED              false
    #define _DRV_USBHS_INTERRUPT_PERSISTENT               true
    #define _DRV_USBHS_CLOCK_CONTROL_SETUP_DEVICE_MODE   
    #define _DRV_USBHS_CLOCK_CONTROL_SETUP_HOST_MODE  
#endif

#if (_DRV_USBHS_INTERRUPT_PERSISTENT == false)

    /* The interrupt flag will be cleared first and then source events will
     * be processed. */
    #define _DRV_USBHS_PersistentInterruptSourceClear(source)
    #define _DRV_USBHS_NonPersistentInterruptSourceClear(source) SYS_INT_SourceStatusClear( source )
#else
    /* The interrupt flag will be cleared after the source events have been
     * processed. */
    #define _DRV_USBHS_PersistentInterruptSourceClear(source)       SYS_INT_SourceStatusClear( source )
    #define _DRV_USBHS_NonPersistentInterruptSourceClear(source)
#endif

#if (DRV_USBHS_INTERRUPT_MODE == true)

    #define _DRV_USBHS_InterruptSourceEnable(source)      SYS_INT_SourceEnable( source )
    #define _DRV_USBHS_InterruptSourceDisable(source)     SYS_INT_SourceDisable( source )
    #define _DRV_USBHS_InterruptSourceClear(source)  SYS_INT_SourceStatusClear( source )
    #define _DRV_USBHS_InterruptSourceStatusGet(source)   SYS_INT_SourceStatusGet( source )
    #define _DRV_USBHS_InterruptSourceStatusSet(source)   SYS_INT_SourceStatusSet( source )
    #define _DRV_USBHS_Tasks_ISR(object)
    #define _DRV_USBHS_Tasks_ISR_USBDMA(object)
 
#endif

#if (DRV_USBHS_INTERRUPT_MODE == false)

    #define _DRV_USBHS_InterruptSourceEnable(source)
    #define _DRV_USBHS_InterruptSourceDisable(source)     false
    #define _DRV_USBHS_InterruptSourceClear(source)       SYS_INT_SourceStatusClear( source )
    #define _DRV_USBHS_InterruptSourceStatusGet(source)   SYS_INT_SourceStatusGet( source )
    #define _DRV_USBHS_Tasks_ISR(object)                  DRV_USBHS_Tasks_ISR(object)
    #define _DRV_USBHS_Tasks_ISR_USBDMA(object)           DRV_USBHS_Tasks_ISR_USBDMA(object)

#endif

/**********************************************
 * Sets up driver mode-specific init routine
 * based on selected support.
 *********************************************/

#ifndef DRV_USBHS_DEVICE_SUPPORT
    #error "DRV_USBHS_DEVICE_SUPPORT must be defined and be either true or false"
#endif

#ifndef DRV_USBHS_HOST_SUPPORT
    #error "DRV_USBHS_HOST_SUPPORT must be defined and be either true or false"
#endif

#if (DRV_USBHS_DEVICE_SUPPORT == true)
    #define _DRV_USBHS_DEVICE_INIT(x, y)      _DRV_USBHS_DEVICE_Initialize(x , y)
    #define _DRV_USBHS_DEVICE_TASKS_ISR(x)    _DRV_USBHS_DEVICE_Tasks_ISR(x)
    #define _DRV_USBHS_DEVICE_TASKS_ISR_USBDMA(x)  _DRV_USBHS_DEVICE_Tasks_ISR_USBDMA(x)
    #define _DRV_USBHS_FOR_DEVICE(x, y)       x y
#elif (DRV_USBHS_DEVICE_SUPPORT == false)
    #define _DRV_USBHS_DEVICE_INIT(x, y)  
    #define _DRV_USBHS_DEVICE_TASKS_ISR(x)
    #define _DRV_USBHS_DEVICE_TASKS_ISR_USBDMA(x)
    #define _DRV_USBHS_FOR_DEVICE(x, y)       
#endif
 
#if (DRV_USBHS_HOST_SUPPORT == true)
    #define _DRV_USBHS_HOST_INIT(x, y)    _DRV_USBHS_HOST_Initialize(x , y)
    #define _DRV_USBHS_HOST_TASKS_ISR(x)  _DRV_USBHS_HOST_Tasks_ISR(x)
    #define _DRV_USBHS_HOST_TASKS_ISR_USBDMA(x) _DRV_USBHS_HOST_Tasks_ISR_USBDMA(x)
    #define _DRV_USBHS_FOR_HOST(x, y)     x y
    #define _DRV_USBHS_HOST_ATTACH_DETACH_STATE_MACHINE(x)  _DRV_USBHS_HOST_AttachDetachStateMachine(x)
    #define _DRV_USBHS_HOST_RESET_STATE_MACINE(x)  _DRV_USBHS_HOST_ResetStateMachine(x)
#elif (DRV_USBHS_HOST_SUPPORT == false)
    #define _DRV_USBHS_HOST_INIT(x, y)  
    #define _DRV_USBHS_HOST_TASKS_ISR(x)
    #define _DRV_USBHS_HOST_TASKS_ISR_USBDMA(x)
    #define _DRV_USBHS_FOR_HOST(x, y) 
    #define _DRV_USBHS_HOST_ATTACH_DETACH_STATE_MACHINE(x)  
    #define _DRV_USBHS_HOST_RESET_STATE_MACINE(x)  
#endif

#endif
