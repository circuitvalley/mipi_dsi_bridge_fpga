/*******************************************************************************
  USB Driver Feature Variant Implementations

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhsv1_variant_mapping.h

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

#ifndef _DRV_USBHSV1_VARIANT_MAPPING_H
#define _DRV_USBHSV1_VARIANT_MAPPING_H

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

#if (!defined(DRV_USBHSV1_INSTANCES_NUMBER))
	#error "DRV_USBHSV1_INSTANCES_NUMBER must be defined"
#endif

#if (!defined(DRV_USBHSV1_INTERRUPT_MODE))




/************************************************
 * This version of the driver does not support
 * mode.
 ***********************************************/



    /* Multi client operation in static is not supported */

    
    
        /* Map internal macros and functions to the static 
         * single open variant */
        




    /* This means that dynamic operation is requested */



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


/**********************************************
 * These macros allow variables to be compiled
 * based on dynamic or static buil.
 *********************************************/







// *****************************************************************************
/* Interrupt Source Control

  Summary:
    Macros to enable, disable or clear the interrupt source

  Description:
    These macros enable, disable, or clear the interrupt source.

    The macros get mapped to the respective SYS module APIs if the configuration
    option DRV_USBHSV1_INTERRUPT_MODE is set to true.

  Remarks:
    These macros are mandatory.
*/

    #error "Interrupt mode must be defined and must be either true or false"
#endif

#if (DRV_USBHSV1_INTERRUPT_MODE == true)

    #define _DRV_USBHSV1_InterruptSourceEnable(source)                  SYS_INT_SourceEnable( source )
    #define _DRV_USBHSV1_InterruptSourceDisable(source)                 SYS_INT_SourceDisable( source )
    #define _DRV_USBHSV1_InterruptSourceClear(source)		            SYS_INT_SourceStatusClear( source )
    #define _DRV_USBHSV1_InterruptSourceStatusGet(source)               SYS_INT_SourceStatusGet( source )
    #define _DRV_USBHSV1_InterruptSourceStatusSet(source)               SYS_INT_SourceStatusSet( source )
    #define _DRV_USBHSV1_InterruptVectorPrioritySet(source, priority)   SYS_INT_VectorPrioritySet(source, priority)
    #define _DRV_USBHSV1_Tasks_ISR(object)
 
#endif

#if (DRV_USBHSV1_INTERRUPT_MODE == false)

    #define _DRV_USBHSV1_InterruptSourceEnable(source)
    #define _DRV_USBHSV1_InterruptSourceDisable(source)     false
    #define _DRV_USBHSV1_InterruptSourceClear(source)       SYS_INT_SourceStatusClear( source )
    #define _DRV_USBHSV1_InterruptSourceStatusGet(source)   SYS_INT_SourceStatusGet( source )
    #define _DRV_USBHSV1_Tasks_ISR(object)                  DRV_USBHSV1_Tasks_ISR(object)

#endif

/**********************************************
 * Sets up driver mode-specific init routine
 * based on selected support.
 *********************************************/

#ifndef DRV_USBHSV1_DEVICE_SUPPORT
    #error "DRV_USBHSV1_DEVICE_SUPPORT must be defined and be either true or false"
#endif

#ifndef DRV_USBHSV1_HOST_SUPPORT
    #error "DRV_USBHSV1_HOST_SUPPORT must be defined and be either true or false"
#endif

#if (DRV_USBHSV1_DEVICE_SUPPORT == true)
    #define _DRV_USBHSV1_DEVICE_INIT(x, y)      _DRV_USBHSV1_DEVICE_Initialize(x , y)
    #define _DRV_USBHSV1_DEVICE_TASKS_ISR(x)    _DRV_USBHSV1_DEVICE_Tasks_ISR(x)
#elif (DRV_USBHSV1_DEVICE_SUPPORT == false)
    #define _DRV_USBHSV1_DEVICE_INIT(x, y)  
    #define _DRV_USBHSV1_DEVICE_TASKS_ISR(x)
#endif
 
#if (DRV_USBHSV1_HOST_SUPPORT == true)
    #define _DRV_USBHSV1_HOST_INIT(x, y)    _DRV_USBHSV1_HOST_Initialize(x , y)
    #define _DRV_USBHSV1_HOST_TASKS_ISR(x)  _DRV_USBHSV1_HOST_Tasks_ISR(x)
    #define _DRV_USBHSV1_HOST_ATTACH_DETACH_STATE_MACHINE(x)  _DRV_USBHSV1_HOST_AttachDetachStateMachine(x)
    #define _DRV_USBHSV1_HOST_RESET_STATE_MACINE(x)  _DRV_USBHSV1_HOST_ResetStateMachine(x)
#elif (DRV_USBHSV1_HOST_SUPPORT == false)
    #define _DRV_USBHSV1_HOST_INIT(x, y)  
    #define _DRV_USBHSV1_HOST_TASKS_ISR(x)
    #define _DRV_USBHSV1_HOST_ATTACH_DETACH_STATE_MACHINE(x)  
    #define _DRV_USBHSV1_HOST_RESET_STATE_MACINE(x)  
#endif

#endif
