/*******************************************************************************
  USB Device Driver Core Routines

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhsv1.c

  Summary:
    USB Device Driver Dynamic Implementation of Core routines

  Description:
    The USB device driver provides a simple interface to manage the USB
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines for the USB driver. While building the driver from 
    source, ALWAYS use this file in the build.
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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "driver/usb/usbhsv1/src/drv_usbhsv1_local.h"
#include "arch/arm/devices_pic32c.h"
#include "system_definitions.h"


/************************************
 * Driver instance object
 ***********************************/

DRV_USBHSV1_OBJ gDrvUSBObj[DRV_USBHSV1_INSTANCES_NUMBER];

/*********************************
 * Array of endpoint objects. Two 
 * objects per endpoint 
 ********************************/

DRV_USBHSV1_DEVICE_ENDPOINT_OBJ gDrvUSBEndpoints [DRV_USBHSV1_INSTANCES_NUMBER] [DRV_USBHSV1_ENDPOINTS_NUMBER * 2];


// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_USBHSV1_Initialize
    ( 
       const SYS_MODULE_INDEX index,
       const SYS_MODULE_INIT * const init 
    )

  Summary:
    Dynamic implementation of DRV_USBHSV1_Initialize system interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_Initialize system interface
    function. Function performs the following task:
    - Initializes the necessary USB module as per the instance init data
    - Updates internal data structure for the particular USB instance
    - Returns the USB instance value as a handle to the system

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

SYS_MODULE_OBJ DRV_USBHSV1_Initialize 
( 
    const SYS_MODULE_INDEX  drvIndex,
    const SYS_MODULE_INIT * const init
)
{
   
    DRV_USBHSV1_OBJ * drvObj;
    usbhs_registers_t * usbID;
    DRV_USBHSV1_INIT * usbInit;
    SYS_MODULE_OBJ retVal = SYS_MODULE_OBJ_INVALID;
    
    if(drvIndex >= DRV_USBHSV1_INSTANCES_NUMBER)
    {
        /* The driver module index specified does not exist in the system */
        SYS_DEBUG(SYS_ERROR_INFO,"\r\nDRV USB USBHSV1: Invalid Driver Module Index in DRV_USBHSV1_Initialize().");
    }
    else if(gDrvUSBObj[drvIndex].inUse == true)
    {
        /* Cannot initialize an object that is already in use. */
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nDRV USB USBHSV1: Driver is already initialized in DRV_USBHSV1_Initialize().");
    }
    else
    {
        usbInit = (DRV_USBHSV1_INIT *) init;
        usbID = usbInit->usbID;
        drvObj = &gDrvUSBObj[drvIndex];

        /* Try creating the global mutex. This always passes if  */
        OSAL_ASSERT(if(OSAL_MUTEX_Create(&drvObj->mutexID) == OSAL_RESULT_TRUE), "\r\nDRV USB USBHSV1: Mutex create failed");

        /* Populate the driver object with the required data */
        drvObj->inUse = true;
        drvObj->status = SYS_STATUS_BUSY;
        drvObj->usbID = usbID;
        drvObj->operationMode = usbInit->operationMode;
        drvObj->operationSpeed = usbInit->operationSpeed;

        /* Assign the endpoint table */
		drvObj->endpointTable = &gDrvUSBEndpoints[drvIndex][0];
        drvObj->interruptSource = usbInit->interruptSource;

        /* The root hub information is applicable for host mode operation. */    
        drvObj->rootHubInfo.rootHubAvailableCurrent = usbInit->rootHubAvailableCurrent;
        drvObj->rootHubInfo.portIndication = usbInit->portIndication;
        drvObj->rootHubInfo.portOverCurrentDetect = usbInit->portOverCurrentDetect;
        drvObj->rootHubInfo.portPowerEnable = usbInit->portPowerEnable;

        drvObj->isOpened = false;
        drvObj->pEventCallBack = NULL;
        
        /* Set the starting VBUS level. */
        drvObj->vbusLevel = DRV_USB_VBUS_LEVEL_INVALID;
        drvObj->vbusComparator = usbInit->vbusComparator;
        drvObj->sessionInvalidEventSent = false;
    
        if(drvObj->operationMode == DRV_USBHSV1_OPMODE_DEVICE)
        {
 
            if(drvObj->operationSpeed == DRV_USBHSV1_DEVICE_SPEEDCONF_NORMAL)
            {        
                /* Configure for Normal mode - For LS, FS & HS */
                usbID->USBHS_DEVCTRL.SPDCONF = USBHS_DEVCTRL_SPDCONF_NORMAL_Val;
            }
            else
            {
                /* Configure for Low Power mode - For LS & FS */
                usbID->USBHS_DEVCTRL.SPDCONF = USBHS_DEVCTRL_SPDCONF_LOW_POWER_Val;
            }
            /* Configure the device for Device mode configuration */
            /* Doing register access intentionally to clear UID bit. */
            usbID->USBHS_CTRL.w = (USBHS_CTRL_VBUSHWC_Msk | USBHS_CTRL_UIMOD_Msk);
            
            /* Enable the USB hardware */
            usbID->USBHS_CTRL.w |= USBHS_CTRL_USBE_Msk;

            
            if(drvObj->deviceSpeed == USB_SPEED_LOW)
            {
                /* Configure for Low Speed operation */
                usbID->USBHS_DEVCTRL.LS = 1;            
            }
            else
            {
				/* Configure for Full / High Speed operation */
                usbID->USBHS_DEVCTRL.LS = 0;
            }

            /* Unfreeze USB clock */
            usbID->USBHS_CTRL.w &= ~USBHS_CTRL_FRZCLK_Msk;
        
            /* Wait to unfreeze clock */
            while(!(usbID->USBHS_SR.CLKUSABLE));
        
            /* Freeze USB clock */
            usbID->USBHS_CTRL.w |= USBHS_CTRL_FRZCLK_Msk;
        
        }
        else if(drvObj->operationMode == DRV_USBHSV1_OPMODE_HOST)
        {    
            /* Enable the USB hardware for Host Mode*/
            /* Set Host Mode */            
            usbID->USBHS_CTRL.UIMOD = false; 
            /* Enable USB Hardware */            
            usbID->USBHS_CTRL.USBE = true; 
            /* Unfreeze USB clock */
            usbID->USBHS_CTRL.FRZCLK = false; 
        }
        _PMC_REGS->PMC_FSMR.w |= PMC_FSMR_USBAL_Msk;
        

        /* Set the state to indicate that the delay will be started */
        drvObj->state = DRV_USBHSV1_TASK_STATE_WAIT_FOR_CLOCK_USABLE;
        retVal = (SYS_MODULE_OBJ)drvIndex;
    }

    return (retVal); 

} /* end of DRV_USBHSV1_Initialize() */

// *****************************************************************************
/* Function:
    void DRV_USBHSV1_Tasks(SYS_MODULE_OBJ object)

  Summary:
    Dynamic implementation of DRV_USBHSV1_Tasks system interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_Tasks system interface
    function.

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

void DRV_USBHSV1_Tasks(SYS_MODULE_OBJ object)
{
    DRV_USBHSV1_OBJ * hDriver; 
    usbhs_registers_t * usbID;
    DRV_USB_VBUS_LEVEL vbusLevel = DRV_USB_VBUS_LEVEL_INVALID;

     hDriver = &gDrvUSBObj[object];

    if(hDriver->status <= SYS_STATUS_UNINITIALIZED)
    {
        /* Driver is not initialized */
    }
    else
    {        
        usbID = hDriver->usbID;

        /* Check the tasks state and maintain */
        switch(hDriver->state)
        {
            case DRV_USBHSV1_TASK_STATE_WAIT_FOR_CLOCK_USABLE:
                
                if(usbID->USBHS_SR.CLKUSABLE)
                {
                    /* Clock unfreeze successful. 
                     * The operation mode can be initialized */
                    
                    hDriver->state = DRV_USBHSV1_TASK_STATE_INITIALIZE_OPERATION_MODE;
                }
                else
                {
                    /* Continue to check if the clock is usable */
                }
                
                break;

            case DRV_USBHSV1_TASK_STATE_INITIALIZE_OPERATION_MODE:

                /* Setup the USB Module based on the selected
                * mode */

                switch(hDriver->operationMode)
                {
                    case DRV_USBHSV1_OPMODE_DEVICE:
                            
                        /* Device mode specific driver initialization */
                        _DRV_USBHSV1_DEVICE_INIT(hDriver, object);
                    break;
                        
                    case DRV_USBHSV1_OPMODE_HOST:


                        /* Host mode specific driver initialization */
                        _DRV_USBHSV1_HOST_INIT(hDriver, object);
                    break;
                        
                    case DRV_USBHSV1_OPMODE_OTG:
                    
                    break;
                            
                    default:
                    
                        SYS_DEBUG(SYS_ERROR_INFO, "\r\nDRV USB USBHSV1: Unsupported driver operation mode in DRV_USBHSV1_Tasks().");
                    
                    break;
                }
                                        
                /* Clear and enable the interrupts */
                _DRV_USBHSV1_InterruptSourceClear(hDriver->interruptSource);
                _DRV_USBHSV1_InterruptSourceEnable(hDriver->interruptSource);
                
                /* Indicate that the object is ready 
                 * and change the state to running */
                
                hDriver->status = SYS_STATUS_READY;
                hDriver->state = DRV_USBHSV1_TASK_STATE_RUNNING;
    
                break;

            case DRV_USBHSV1_TASK_STATE_RUNNING:
                
                /* The module is in a running state. In the polling mode the 
                 * driver ISR tasks and DMA ISR tasks are called here. We also
                 * check for the VBUS level and generate events if a client 
                 * event handler is registered. */

                if(hDriver->pEventCallBack != NULL && hDriver->operationMode == DRV_USBHSV1_OPMODE_DEVICE)
                {
                    /* We have a valid client call back function. Check if
                     * VBUS level has changed */
    
                    if( hDriver->vbusComparator != NULL)
                    {
                        vbusLevel = hDriver->vbusComparator();
                    }
                    else
                    {
                        vbusLevel = DRV_USB_VBUS_LEVEL_VALID;
                    }
                                    
                    if(hDriver->vbusLevel != vbusLevel)
                    {
                        /* This means there was a change in the level */
                        if(vbusLevel == DRV_USB_VBUS_LEVEL_VALID)
                        {
                            /* We have a valid VBUS level */
                            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBHSV1_EVENT_DEVICE_SESSION_VALID, NULL);
                        
                            /* We should be ready for send session invalid event
                             * to the application when they happen.*/
                            hDriver->sessionInvalidEventSent = false;

                        }
                        else
                        {
                            /* Any thing other than valid is considered invalid.
                             * This event may occur multiple times, but we send
                             * it only once. */
                            if(!hDriver->sessionInvalidEventSent)
                            {
                                hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBHSV1_EVENT_DEVICE_SESSION_INVALID, NULL);
                                hDriver->sessionInvalidEventSent = true;
                            }
                        }

                        hDriver->vbusLevel = vbusLevel;
                    }
                }
                else if(hDriver->operationMode == DRV_USBHSV1_OPMODE_HOST)
                {
                    /* Host mode specific polled 
                     * task routines can be called here */ 
                    
                     _DRV_USBHSV1_HOST_ATTACH_DETACH_STATE_MACHINE(hDriver);            
                }

                /* Polled mode driver tasks routines are really the same as the
                 * the ISR task routines called in the driver task routine */
                _DRV_USBHSV1_Tasks_ISR(object);
                
                break;
                
            default:
                break;
            
        }
    }
    
}/* end of DRV_USBHSV1_Tasks() */

// *****************************************************************************
/* Function:
    void DRV_USBHSV1_Deinitialize( const SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_USBHSV1_Deinitialize system interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_Deinitialize 
    system interface function.

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

void DRV_USBHSV1_Deinitialize 
( 
    const SYS_MODULE_INDEX  object
)
{
    DRV_USBHSV1_OBJ * drvObj;

    if(object == SYS_MODULE_OBJ_INVALID)
    {
        /* Invalid object */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\r\nUSB USBHSV1 Driver: Invalid object in DRV_USBHSV1_Deinitialize()");
    }
    else
    {
        if( object >= DRV_USBHSV1_INSTANCES_NUMBER)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\r\nUSB USBHSV1 Driver: Invalid object in DRV_USBHSV1_Deinitialize()");
        }
        else
        {
            if(gDrvUSBObj[object].inUse == false)
            {
                /* Cannot de-initialize an object that is not already in use. */
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\r\nUSB USBHSV1 Driver: Driver not initialized in DRV_USBHSV1_Deinitialize()");
            }
            else
            {
                drvObj = &gDrvUSBObj[object];

                /* Populate the driver object with the required data */

                drvObj->inUse   = false;
                drvObj->status  = SYS_STATUS_UNINITIALIZED; 

                /* Clear and disable the interrupts. Assigning to a value has
                 * been implemented to remove compiler warning in polling mode.
                   */

                SYS_INT_SourceDisable(drvObj->interruptSource);
                SYS_INT_SourceStatusClear(drvObj->interruptSource);

                drvObj->isOpened = false;
                drvObj->pEventCallBack = NULL;

                /* Delete the mutex */
                if(OSAL_MUTEX_Delete(&drvObj->mutexID) != OSAL_RESULT_TRUE)
                {
                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\r\nUSB USBHSV1 Driver: Could not delete mutex in DRV_USBHSV1_Deinitialize()");
                }
            }
        }
    }

} /* end of DRV_USBHSV1_Deinitialize() */

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_USBHSV1_Status( const SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_USBHSV1_Status system interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_Status system interface
    function.

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

SYS_STATUS DRV_USBHSV1_Status ( SYS_MODULE_OBJ object )
{
    SYS_STATUS retVal = gDrvUSBObj[object].status;;

    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: System Module Object is invalid in DRV_USBHSV1_Status().");
        retVal = SYS_STATUS_ERROR;
    }
       
    /* Return the status of the driver object */
    return (retVal);

}/* end of DRV_USBHSV1_Status() */


DRV_HANDLE DRV_USBHSV1_Open
(
    const SYS_MODULE_INDEX iDriver,
    const DRV_IO_INTENT    ioIntent 
)
{
    DRV_USBHSV1_OBJ * drvObj;
    DRV_HANDLE retVal = DRV_HANDLE_INVALID;

    /* The iDriver value should be valid. It should be less the number of driver
     * object instances.  */

    if(iDriver >= DRV_USBHSV1_INSTANCES_NUMBER)
    {
        SYS_DEBUG(SYS_ERROR_DEBUG, "\r\nUSB USBHSV1 Driver: Bad Driver Index in DRV_USBHSV1_Open().");
    }
    else
    {
        drvObj = &gDrvUSBObj[iDriver];

        if(drvObj->status == SYS_STATUS_READY)
        {
            if(ioIntent != (DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_NONBLOCKING | DRV_IO_INTENT_READWRITE))
            {
                /* The driver only supports this mode */
                SYS_DEBUG(SYS_ERROR_DEBUG, "\r\nUSB USBHSV1 Driver: Unsupported IO Intent in DRV_USBHSV1_Open().");
            }
            else
            {
                if(drvObj->isOpened)
                {
                    /* Driver supports exclusive open only */
                    SYS_DEBUG(SYS_ERROR_DEBUG, "\r\nUSB USBHSV1 Driver: Driver can be opened only once. Multiple calls to DRV_USBHSV1_Open().");
                }
                else
                {
                    /* Clear prior value */
                    drvObj->pEventCallBack = NULL;

                    /* Store the handle in the driver object client table and update
                     * the number of clients*/
                    drvObj->isOpened = true;
                    retVal = ((DRV_HANDLE)drvObj);
                    SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: Driver opened successfully in DRV_USBHSV1_Open().");
                }
            }
        }
    }

    /* Return the client object */

    return (retVal);

}/* end of DRV_USBHSV1_Open() */

// *****************************************************************************
/* Function:
    void DRV_USBHSV1_Close( DRV_HANDLE client )

  Summary:
    Dynamic implementation of DRV_USBHSV1_Close client interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_Close client interface
    function.

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

void DRV_USBHSV1_Close( DRV_HANDLE client )
{
    DRV_USBHSV1_OBJ * hDriver;

    if(client == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: Bad Client Handle in DRV_USBHSV1_Close().");
    }
    else
    {
        hDriver = (DRV_USBHSV1_OBJ *) client;
    
        if(!(hDriver->isOpened))
        {
            SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: Invalid client handle in DRV_USBHSV1_Close().");
        }
        else
        {
            /* Give back the client */
            hDriver->isOpened = false;
            hDriver->pEventCallBack = NULL;
        }
    }

}/* end of DRV_USBHSV1_Close() */


// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_USBHSV1_Tasks_ISR( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_USBHSV1_Tasks_ISR system interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_Tasks_ISR system interface
    function.

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

void DRV_USBHSV1_Tasks_ISR( SYS_MODULE_OBJ object )
{
    DRV_USBHSV1_OBJ * hDriver;

    hDriver = &gDrvUSBObj[object];
    hDriver->isInInterruptContext = true;
    
    switch(hDriver->operationMode)
    {
        case DRV_USBHSV1_OPMODE_DEVICE:
            _DRV_USBHSV1_DEVICE_TASKS_ISR(hDriver);
            break;
        case DRV_USBHSV1_OPMODE_HOST:
            _DRV_USBHSV1_HOST_TASKS_ISR(hDriver);
            break;
        case DRV_USBHSV1_OPMODE_OTG:
            break;
        default:
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: What mode are you trying? in DRV_USBHSV1_Tasks_ISR().");
            break;
    }

    SYS_INT_SourceStatusClear(hDriver->interruptSource);
    hDriver->isInInterruptContext = false;

}/* end of DRV_USBHSV1_Tasks_ISR() */


// *****************************************************************************
/* Function:
    bool DRV_USBHSV1_HOST_Resume(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHSV1_HOST_Resume
    client interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_HOST_Resume client
    interface function. Function resumes a suspended BUS.

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

bool DRV_USBHSV1_HOST_Resume
(
    DRV_HANDLE handle
)
{
    DRV_USBHSV1_OBJ * pusbdrvObj = (DRV_USBHSV1_OBJ *)NULL;
    bool retVal = false;
    
    /* Check if the handle is valid */
    if(handle == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: Bad Client or client closed in DRV_USBHSV1_HOST_Resume().");
    }
    else
    {
        pusbdrvObj = (DRV_USBHSV1_OBJ *)handle;
        pusbdrvObj->usbID->USBHS_HSTCTRL.SOFE = 1;
        retVal = true;
    }

    return (retVal);

}/* end of DRV_USBHSV1_HOST_Resume() */

// *****************************************************************************
/* Function:
    bool DRV_USBHSV1_HOST_Suspend(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHSV1_HOST_Suspend
    client interface function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_HOST_Suspend client
    interface function. Function suspends USB BUS.

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

bool DRV_USBHSV1_HOST_Suspend
(
    DRV_HANDLE handle
)
{
    DRV_USBHSV1_OBJ * pusbdrvObj = (DRV_USBHSV1_OBJ *)NULL;
    bool retVal = false;

    /* Check if the handle is valid */
    if(handle == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: Bad Client or client closed in DRV_USBHSV1_HOST_Suspend().");
    }
    else
    {
        pusbdrvObj = (DRV_USBHSV1_OBJ *)handle;

        /* Suspend the bus */
        pusbdrvObj->usbID->USBHS_HSTCTRL.SOFE = 0;
        retVal = true;
    }

    return (retVal);

}/* end of DRV_USBHSV1_HOST_Suspend() */

// *****************************************************************************
/* Function:
    void DRV_USBHSV1_ClientEventCallBackSet
    (
        DRV_HANDLE   handle,
        uintptr_t    hReferenceData,
        DRV_USBHSV1_EVENT_CALLBACK eventCallBack
    )

  Summary:
    Dynamic implementation of DRV_USBHSV1_ClientEventCallBackSet client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHSV1_ClientEventCallBackSet
    client interface function.

  Remarks:
    See drv_usbhsv1.h for usage information.
*/

void DRV_USBHSV1_ClientEventCallBackSet
( 
    DRV_HANDLE   client,
    uintptr_t    hReferenceData,
    DRV_USB_EVENT_CALLBACK eventCallBack 
)
{
    DRV_USBHSV1_OBJ * pusbDrvObj;
    
    if(client == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: Bad Client Handle in DRV_USBHSV1_ClientEventCallBackSet().");
    }
    else
    {
        pusbDrvObj = (DRV_USBHSV1_OBJ *) client;
    
        if(!pusbDrvObj->isOpened)
        {
            SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSB USBHSV1 Driver: Invalid client handle in DRV_USBHSV1_ClientEventCallBackSet().");
        }
        else
        {
            /* Assign event call back and reference data */
            pusbDrvObj->hClientArg = hReferenceData;
            pusbDrvObj->pEventCallBack = eventCallBack;
        }
    }
        
}/* end of DRV_USBHSV1_ClientEventCallBackSet() */
