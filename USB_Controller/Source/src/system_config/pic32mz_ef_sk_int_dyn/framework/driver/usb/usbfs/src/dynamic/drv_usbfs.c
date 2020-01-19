/*******************************************************************************
  USB Controller Driver Core Routines.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbfs.c

  Summary:
    USB Controller Driver Core Routines intended for Dynamic implementation.

  Description:
    The USB Controller driver provides a simple interface to manage the USB
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines to be used both by the client(USB Host or Device layer)
    and the system for communicating with USB Contoller driver.  While building
    the driver from source, ALWAYS use this file in the build.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/usb/usbfs/src/drv_usbfs_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/******************************************************
 * Hardware instance, endpoint table and client object
 * lumped together as group to save memory.
 ******************************************************/
DRV_USBFS_GROUP gDrvUSBGroup[DRV_USBFS_INSTANCES_NUMBER];

// *****************************************************************************
// *****************************************************************************
// Section: USB Controller Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_USBFS_Initialize
    ( 
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    )

  Summary:
    Dynamic impementation of DRV_USBFS_Initialize system interface function.

  Description:
    This is the dynamic impementation of DRV_USBFS_Initialize system interface
    function. Function performs the following task:
    - Initializes the neccessary USB module as per the instance init data
    - Updates internal data structure for the particular USB instance
    - Returns the USB instance value as a handle to the system
  
  Remarks:
    See drv_usbfs.h for usage information.
*/

SYS_MODULE_OBJ DRV_USBFS_Initialize 
(
    const SYS_MODULE_INDEX  drvIndex,
    const SYS_MODULE_INIT * const init
)
{
    /* Start of local variable */
    DRV_USBFS_OBJ * pUSBDrvObj    = (DRV_USBFS_OBJ *)NULL;
    USB_MODULE_ID  usbID        = USB_NUMBER_OF_MODULES;
    DRV_USBFS_INIT * pusbInit     = (DRV_USBFS_INIT *)NULL;
    SYS_MODULE_OBJ returnValue  = SYS_MODULE_OBJ_INVALID;
    /* End of local variable */

    /* Check if the specified driver index is in valid range */
    if(drvIndex >= DRV_USBFS_INSTANCES_NUMBER)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Invalid driver index");
        returnValue = SYS_MODULE_OBJ_INVALID;
    }

    /* Check if this hardware instance was already initialized */
    else if(gDrvUSBGroup[drvIndex].gDrvUSBObj.inUse == true)
    {
        /* Cannot initialize an object that is
         * already in use. */
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Hardware Instance already in use");
        returnValue = SYS_MODULE_OBJ_INVALID;
    }
    else
    {
        /* Grab the particular USB instance object*/
        gDrvUSBGroup[drvIndex].gDrvUSBObj.inUse = true;

        /* Assign to the local pointer the init data passed */
        pusbInit   = (DRV_USBFS_INIT *) init;
        usbID      = pusbInit->usbID;
        pUSBDrvObj = &gDrvUSBGroup[drvIndex].gDrvUSBObj;

        /* If this being built in an OS application, then create a mutex */
        OSAL_ASSERT(if(OSAL_MUTEX_Create(&pUSBDrvObj->mutexID) == OSAL_RESULT_TRUE), "\r\nUSBFS Driver: Mutex create failed");

        /* Populate the driver instance object with required data */
        pUSBDrvObj->status  = SYS_STATUS_BUSY;
        pUSBDrvObj->usbID   = usbID;
        pUSBDrvObj->operationMode  = pusbInit->operationMode;
        pUSBDrvObj->pBDT    = (DRV_USBFS_BDT_ENTRY *)(pusbInit->endpointTable);
        pUSBDrvObj->isOpened = false;
        pUSBDrvObj->pEventCallBack = NULL;

        /* Assign the endpoint table */
        pUSBDrvObj->endpointTable = &gDrvUSBGroup[drvIndex].gDrvUSBEndpoints[0];

        pUSBDrvObj->interruptSource  = pusbInit->interruptSource;

        /* Enable USB module(U1PWRC<0>). This internally does the following:
         * - Start the USB clock
         * - Allow the USB interrupt to be activated
         * - Select USB as the owner of the necessary I/O pins
         * - Enable the USB transceiver
         * - Enable the USB comparators */

        PLIB_USB_Enable(usbID);

        /* Setup the Hardware */
        if(pusbInit->stopInIdle)
        {
            PLIB_USB_StopInIdleEnable( usbID );
        }
        else
        {
            PLIB_USB_StopInIdleDisable( usbID );
        }
#ifdef PLIB_USB_ExistsAutomaticSuspend
        if(PLIB_USB_ExistsAutomaticSuspend(usbID))
        {
            if(pusbInit->suspendInSleep)
            {
                PLIB_USB_AutoSuspendEnable( usbID );
            }
            else
            {
                PLIB_USB_AutoSuspendDisable( usbID );
            }
        }
#endif
        /* Setup the USB Module as per selected mode */
        switch(pusbInit->operationMode)
        {
            case DRV_USBFS_OPMODE_DEVICE:
                
                /* Initialize USB Controller for Device mode */
                _DRV_USBFS_DEVICE_INIT(pUSBDrvObj, drvIndex);
                break;

            case DRV_USBFS_OPMODE_HOST:
                
                /* Initialize USB Controller for Host mode */
                _DRV_USBFS_HOST_INIT(pUSBDrvObj, drvIndex, pusbInit);
                break;

            case DRV_USBFS_OPMODE_OTG:
                /* Not implemented at this point of time*/
                break;
            default:
                SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: What mode are you trying?");
                break;
        }

        /* Assign the BDT table base address */
        PLIB_USB_BDTBaseAddressSet(usbID , (void *)((uint32_t)KVA_TO_PA(pUSBDrvObj->pBDT)));    

        /* Indicate that the object is ready and in use
         * and return the driver handle */

        pUSBDrvObj->status = SYS_STATUS_READY;
        returnValue = drvIndex;
    }

    return (returnValue);

}/* end of DRV_USBFS_Initialize() */

// *****************************************************************************
/* Function:
    void DRV_USBFS_Deinitialize( const SYS_MODULE_OBJ object )

  Summary:
    Dynamic impementation of DRV_USBFS_Deinitialize system interface function.

  Description:
    This is the dynamic impementation of DRV_USBFS_Deinitialize system interface
    function.

  Remarks:
    See drv_usbfs.h for usage information.
*/

void DRV_USBFS_Deinitialize
( 
    const SYS_MODULE_OBJ  object
)
{
    DRV_USBFS_OBJ * pUSBDrvObj = NULL;
    bool returnValue = false;

    /* Check if USB instance object is valid */
    if((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_USBFS_INSTANCES_NUMBER))
    {
        /* Invalid object */
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Invalid System Module Object");
    }
    else if(gDrvUSBGroup[object].gDrvUSBObj.inUse == false)
    {
        /* Cannot deinitialize an object that is 
         * not in use. */
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Instance not in use");
    }
    else
    {
        pUSBDrvObj = &gDrvUSBGroup[object].gDrvUSBObj;

        /* Release the USB instance object */
        pUSBDrvObj->inUse = false;

        /* Reset the open flag */
        pUSBDrvObj->isOpened = false;

        /* Delete the mutex */
        OSAL_ASSERT(if(OSAL_MUTEX_Delete(&pUSBDrvObj->mutexID) == OSAL_RESULT_TRUE), "\r\nUSBFS Driver: Mutex delete failed");
        
        /* Uninitialize the status*/
        pUSBDrvObj->status = SYS_STATUS_UNINITIALIZED;

        pUSBDrvObj->pEventCallBack = NULL;

        /* Clear and disable the interrupts */
        returnValue = _DRV_USBFS_InterruptSourceDisable(pUSBDrvObj->interruptSource);
        if(returnValue == false)
        {
            SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Interrupt Source Disable failed");
        }
        _DRV_USBFS_InterruptSourceClear(pUSBDrvObj->interruptSource);

        /* Turn off USB module */
        PLIB_USB_Disable(pUSBDrvObj->usbID);

    }

    return;

} /* end of DRV_USBFS_Deinitialize() */

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_USBFS_Status( const SYS_MODULE_OBJ object )

  Summary:
    Dynamic impementation of DRV_USBFS_Status system interface function.

  Description:
    This is the dynamic impementation of DRV_USBFS_Status system interface
    function.

  Remarks:
    See drv_usbfs.h for usage information.
*/

SYS_STATUS DRV_USBFS_Status
(
    const SYS_MODULE_OBJ object
)
{
    /* Start of local variables */
    SYS_STATUS returnValue = SYS_STATUS_UNINITIALIZED;
    /* End of local variables */

    /* Check if USB instance object is valid */
    if((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_USBFS_INSTANCES_NUMBER))
    {
        /* Invalid object */
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Invalid object");
    }
    else
    {
        returnValue = gDrvUSBGroup[object].gDrvUSBObj.status;
    }

    /* Return the status of the driver object */
    return returnValue;
    
}/* end of DRV_USBFS_Status() */

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_USBFS_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT    ioIntent 
    )

  Summary:
    Dynamic impementation of DRV_USBFS_Open client interface function.

  Description:
    This is the dynamic impementation of DRV_USBFS_Open client interface function.

  Remarks:
    See drv_usbfs.h for usage information.
*/

DRV_HANDLE DRV_USBFS_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT    ioIntent 
)
{
    /* Check if the specified driver index is in valid range */
    if(drvIndex >= DRV_USBFS_INSTANCES_NUMBER)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Bad Driver Index");
    }
    /* Check if USB instance object is ready*/
    else if(gDrvUSBGroup[drvIndex].gDrvUSBObj.status != SYS_STATUS_READY)
    {
        /* The USB module should be ready */
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Was the driver initialized?");
    }
    else if(ioIntent != (DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_NONBLOCKING |DRV_IO_INTENT_READWRITE))
    {
        /* The driver only supports this mode */
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: IO intent mode not supported");
    }
    else if(gDrvUSBGroup[drvIndex].gDrvUSBObj.isOpened)
    {
        /* Driver supports exclusive open only */
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Driver already opened once. Cannot open again");
    }
    else
    {
        gDrvUSBGroup[drvIndex].gDrvUSBObj.isOpened = true;
        
        /* Return the client object address */
        return ((DRV_HANDLE)&(gDrvUSBGroup[drvIndex].gDrvUSBObj));
    }

    /* Return invalid handle */
    return DRV_HANDLE_INVALID;

}/* end of DRV_USBFS_Open()*/

// *****************************************************************************
/* Function:
    bool DRV_USBFS_HOST_Resume(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBFS_HOST_Resume
    client interface function.

  Description:
    This is the dynamic implementation of DRV_USBFS_HOST_Resume client interface
    function. Function resumes a suspended BUS.

  Remarks:
    See drv_usbfs.h for usage information.
*/

bool DRV_USBFS_HOST_Resume
(
    DRV_HANDLE handle
)
{
    /* Start of local variable */
    DRV_USBFS_OBJ * pUSBDrvObj = (DRV_USBFS_OBJ *)handle;
    bool returnValue = false;
    /* End of local variable */
    
    /* Check if the handle is valid */
    if((handle == DRV_HANDLE_INVALID) || (!(pUSBDrvObj->isOpened)))
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Bad Client or client closed");
    }
    else
    {
        /* Enable the SOF */
        PLIB_USB_SOFEnable(pUSBDrvObj->usbID);
        PLIB_USB_InterruptEnable(pUSBDrvObj->usbID, USB_INT_SOF);
        returnValue = true;
    }

    return returnValue;

}/* end of DRV_USBFS_HOST_Resume() */

// *****************************************************************************
/* Function:
    bool DRV_USBFS_HOST_Suspend(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBFS_HOST_Suspend
    client interface function.

  Description:
    This is the dynamic implementation of DRV_USBFS_HOST_Suspend client
    interface function. Function suspends USB BUS.

  Remarks:
    See drv_usbfs.h for usage information.
*/

bool DRV_USBFS_HOST_Suspend
(
    DRV_HANDLE handle
)
{
    /* Start of local variable */
    DRV_USBFS_OBJ * pUSBDrvObj = (DRV_USBFS_OBJ *)handle;
    bool returnValue = false;
    /* End of local variable */

    /* Check if the handle is valid */
    if((handle == DRV_HANDLE_INVALID) || (!(pUSBDrvObj->isOpened)))
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Bad Client or client closed");
    }
    else
    {
        /* Disable the SOF */
        PLIB_USB_SOFDisable(pUSBDrvObj->usbID);
        PLIB_USB_InterruptDisable(pUSBDrvObj->usbID, USB_INT_SOF);
        returnValue = true;
    }

    return returnValue;

}/* end of DRV_USBFS_HOST_Suspend() */

// *****************************************************************************
/* Function:
    void DRV_USBFS_Close( DRV_HANDLE client )

  Summary:
    Dynamic impementation of DRV_USBFS_Close client interface function.

  Description:
    This is the dynamic impementation of DRV_USBFS_Close client interface
    function.

  Remarks:
    See drv_usbfs.h for usage information.
*/

void DRV_USBFS_Close
(
    DRV_HANDLE handle
)
{
    /* Start of local variable */
    DRV_USBFS_OBJ * pUSBDrvObj = (DRV_USBFS_OBJ *)NULL;
    /* End of local variable */

    /* Check if the handle is valid */
    if(handle == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Bad Client Handle");
    }
    else
    {
        /* Reset the relevant parameters */
        pUSBDrvObj = (DRV_USBFS_OBJ *)handle;
        if(pUSBDrvObj->isOpened)
        {
            pUSBDrvObj->isOpened = false;
            pUSBDrvObj->pEventCallBack = NULL;
        }
        else
        {
            SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Client Handle already closed");
        }

    }

    return;
    
}/* end of DRV_USBFS_Close() */

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_USBFS_Tasks_ISR( SYS_MODULE_OBJ object )

  Summary:
    Dynamic impementation of DRV_USBFS_Tasks_ISR system interface function.

  Description:
    This is the dynamic impementation of DRV_USBFS_Tasks_ISR system interface
    function.

  Remarks:
    See drv_usbfs.h for usage information.
*/

void DRV_USBFS_Tasks_ISR
(
    SYS_MODULE_OBJ object
)
{
    /* Start of local variable */
    DRV_USBFS_OBJ * pUSBDriver = (DRV_USBFS_OBJ *)NULL;
    /* End of local varibale */

    pUSBDriver = &gDrvUSBGroup[object].gDrvUSBObj;

    /* We are entering an interrupt context */
    pUSBDriver->inInterruptContext = true;

    /* Clear the interrupt */
    _DRV_USBFS_InterruptSourceClear(pUSBDriver->interruptSource);
	
    switch(pUSBDriver->operationMode)
    {
        case DRV_USBFS_OPMODE_DEVICE:
            
            /* Driver is running in Device Mode */
            _DRV_USBFS_DEVICE_TASKS_ISR(pUSBDriver);
            break;
        
        case DRV_USBFS_OPMODE_HOST:

            /* Driver is running in Host Mode */
            _DRV_USBFS_HOST_TASKS_ISR(pUSBDriver);
            break;

        case DRV_USBFS_OPMODE_OTG:
            /* OTG mode is not supported yet */
            break;

        default:
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSBFS Driver: What mode are you trying?");
            break;
    }
  
    pUSBDriver->inInterruptContext = false;
    
}/* end of DRV_USBFS_Tasks_ISR()*/

// *****************************************************************************
/* Function:
    void DRV_USBFS_ClientEventCallBackSet
    (
        DRV_HANDLE   handle,
        uintptr_t    hReferenceData,
        DRV_USBFS_EVENT_CALLBACK eventCallBack
    )

  Summary:
    Dynamic impementation of DRV_USBFS_ClientEventCallBackSet client interface
    function.

  Description:
    This is the dynamic impementation of DRV_USBFS_ClientEventCallBackSet client
    interface function.

  Remarks:
    See drv_usbfs.h for usage information.
*/

void DRV_USBFS_ClientEventCallBackSet
( 
    DRV_HANDLE   handle,
    uintptr_t    hReferenceData,
    DRV_USB_EVENT_CALLBACK eventCallBack 
)
{
    /* Start of local variables */
    DRV_USBFS_OBJ * pUSBDrvObj = (DRV_USBFS_OBJ *)handle;
    /* End of local variables */
    
    /* Check if the handle is valid or opened */
    if((handle == DRV_HANDLE_INVALID) || (!(pUSBDrvObj->isOpened)))
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBFS Driver: Bad Client or client closed");
    }
    else
    {
        /* Assign event call back and reference data */
        pUSBDrvObj->hClientArg = hReferenceData;
        pUSBDrvObj->pEventCallBack = eventCallBack;

        /* If the driver is operating in device mode, this is the time
         * we enable the USB interrupt */

        if(pUSBDrvObj->operationMode == USB_OPMODE_DEVICE)
        {
            /* Enable the session valid interrupt */
            PLIB_USB_OTG_InterruptEnable(pUSBDrvObj->usbID, USB_OTG_INT_SESSION_VALID);
            
            /* Enable the interrupt */
            _DRV_USBFS_InterruptSourceEnable(pUSBDrvObj->interruptSource);

        }
    }
   
    return;
    
} /* end of DRV_USBFS_ClientEventCallBackSet() */

// *****************************************************************************
/* Function:
    void DRV_USBFS_Tasks( SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's state machine when the driver is configured for 
    polled mode.

  Description:
    Maintains the driver's state machine when the driver is configured for 
    polled mode. This function should be called from the system tasks routine.

  Remarks:
    Refer to drv_usbfs.h for usage information.
*/

void DRV_USBFS_Tasks(SYS_MODULE_OBJ object)
{
    /* This driver does not have any non interrupt tasks. When the driver
     * is configured for polled mode operation, the _DRV_USBFS_Tasks_ISR function
     * will map to DRV_USBFS_Tasks_ISR function. In interrupt mode, this function
     * will be mapped to nothing and hence this function will not have any
     * effect. */

    _DRV_USBFS_Tasks_ISR(object);
}

void DRV_USBFS_Tasks_ISR_USBDMA( SYS_MODULE_OBJ object )
{
    /* This function is implemented to only maintain compatibility with the
     * PIC32MZ High Speed USB Driver. This function does not do anything on the
     * PIC32MX USB driver and is not required to be called in a PIC32MX USB
     * applicaiton */
}
