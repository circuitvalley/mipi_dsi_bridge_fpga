/*******************************************************************************
  USB Device Driver Core Routines

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbhs.c

  Summary:
    USB Device Driver Dynamic Implementation of Core routines

  Description:
    The USB device driver provides a simple interface to manage the USB
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines for the USB driver.

    While building the driver from source, ALWAYS use this file in the build.
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

#include "driver/usb/usbhs/src/drv_usbhs_local.h"
#include "peripheral/ports/plib_ports.h"

/************************************
 * Driver instance object
 ***********************************/

DRV_USBHS_OBJ gDrvUSBObj[DRV_USBHS_INSTANCES_NUMBER];

/*********************************
 * Array of endpoint objects. Two 
 * objects per endpoint 
 ********************************/

DRV_USBHS_DEVICE_ENDPOINT_OBJ gDrvUSBEndpoints [DRV_USBHS_INSTANCES_NUMBER] [DRV_USBHS_ENDPOINTS_NUMBER * 2];

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_USBHS_Initialize
    ( 
       const SYS_MODULE_INDEX index,
       const SYS_MODULE_INIT * const init 
    )

  Summary:
    Dynamic impementation of DRV_USBHS_Initialize system interface function.

  Description:
    This is the dynamic impementation of DRV_USBHS_Initialize system interface
    function. Function performs the following task:
    - Initializes the neccessary USB module as per the instance init data
    - Updates internal data structure for the particular USB instance
    - Returns the USB instance value as a handle to the system

  Remarks:
    See drv_usbhs.h for usage information.
*/

SYS_MODULE_OBJ DRV_USBHS_Initialize 
( 
    const SYS_MODULE_INDEX  drvIndex,
    const SYS_MODULE_INIT * const init
)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * drvObj;
    USBHS_MODULE_ID  usbID;
    DRV_USBHS_INIT * usbInit;
    
    /* End of local variables */
    if( drvIndex >= DRV_USBHS_INSTANCES_NUMBER)
    {
        SYS_DEBUG(SYS_ERROR_INFO,"Increase the value of DRV_USBHS_INSTANCES_NUMBER");
        return SYS_MODULE_OBJ_INVALID;
    }
    
    if(gDrvUSBObj[drvIndex].inUse == true)
    {
        /* Cannot initialize an object that is already in use. */

        SYS_DEBUG(SYS_ERROR_INFO, "Instance already in use");
        return SYS_MODULE_OBJ_INVALID;
    }


    usbInit     = (DRV_USBHS_INIT *) init;
	
	/* User needs to assign the peripheral ID of the USB HS module to the usbID
       member of the DRV_USBHS_INIT structure. Peripheral ID assigned should be 
	   one of the member of USBHS_MODULE_ID enumeration. The following code is 
	   to provide backward compatibility with the applications where they have 
	   specified usbID as 0. */ 

	if (usbInit->usbID == 0)
	{
        /* For the optimized PLIBs, USBHS_ID_X is a pointer to the USB module
         * base address. */

		usbID = USBHS_ID_0; 
	}
    else
    {
        /* No change required */
        usbID = usbInit->usbID;
    }

    drvObj = &gDrvUSBObj[drvIndex];

    OSAL_ASSERT(if(OSAL_MUTEX_Create(&drvObj->mutexID) == OSAL_RESULT_TRUE), "Mutex create failed");

    /* Populate the driver object with the required data */

    drvObj->inUse   = true;
    drvObj->status  = SYS_STATUS_BUSY; 
    drvObj->usbID   = usbID;            
    drvObj->operationMode  = usbInit->operationMode; 
    drvObj->operationSpeed = usbInit->operationSpeed;

    /* Assign the endpoint table */
    drvObj->endpointTable = &gDrvUSBEndpoints[drvIndex][0];
    drvObj->interruptSource  = usbInit->interruptSource;
    
    drvObj->rootHubInfo.rootHubAvailableCurrent = usbInit->rootHubAvailableCurrent;
    drvObj->rootHubInfo.portIndication = usbInit->portIndication;
    drvObj->rootHubInfo.portOverCurrentDetect = usbInit->portOverCurrentDetect;
    drvObj->rootHubInfo.portPowerEnable = usbInit->portPowerEnable;

    drvObj->isOpened = false;
    drvObj->pEventCallBack = NULL;

    drvObj->interruptSourceUSBDma = usbInit->interruptSourceUSBDma;

    if(drvObj->operationMode == DRV_USBHS_OPMODE_HOST)
    {
        /* For Host the ID pin needs to be pulle down */
        PLIB_PORTS_ChangeNoticePullDownPerPortEnable( PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3 );
    }
    
    /* Set the starting VBUS level. */
    drvObj->vbusLevel = USBHS_VBUS_SESSION_END;
    drvObj->sessionInvalidEventSent = false;

    /* Set the state to indicate that the delay will be started */
    drvObj->state = DRV_USBHS_TASK_STATE_STARTING_DELAY;
    
    return ((SYS_MODULE_OBJ)drvIndex); 

} /* end of DRV_USBHS_Initialize() */

// *****************************************************************************
/* Function:
    void DRV_USBHS_Tasks(SYS_MODULE_OBJ object)

  Summary:
    Dynamic impementation of DRV_USBHS_Tasks system interface function.

  Description:
    This is the dynamic impementation of DRV_USBHS_Tasks system interface
    function.

  Remarks:
    See drv_usbhs.h for usage information.
*/

void DRV_USBHS_Tasks(SYS_MODULE_OBJ object)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver; 
    hDriver = &gDrvUSBObj[object];
    USBHS_MODULE_ID usbID = hDriver->usbID;
    /* End of local variables */

    if(hDriver->status <= SYS_STATUS_UNINITIALIZED)
    {
        /* Driver is not initialized */
        return;
    }

    /* Check the tasks state and maintain */
    switch(hDriver->state)
    {
        case DRV_USBHS_TASK_STATE_STARTING_DELAY:

            /* On PI32MZ DA and EF devices, enable the global USB interrupt in 
             * the USBCRCON register. */
            _DRV_USBHS_CLOCK_CONTROL_GLOBAL_USB_INT_ENABLE(usbID);

            /* Start the delay here - 3 sec */
            hDriver->timerHandle = SYS_TMR_DelayMS(_DRV_USBHS_MODULE_RESET_DURATION);
 
            if(hDriver->timerHandle != SYS_TMR_HANDLE_INVALID)
            {
                /* Reset the PHY. This is a workaround
                 * for an errata */
                PLIB_USBHS_SoftResetEnable(usbID);

                /* Delay has started. Move to the next state */
                hDriver->state = DRV_USBHS_TASK_STATE_WAITING_FOR_DELAY_COMPLETE;
            }

            break;

        case DRV_USBHS_TASK_STATE_WAITING_FOR_DELAY_COMPLETE:

            /* Check if the delay is complete */
            if(SYS_TMR_DelayStatusGet(hDriver->timerHandle)) 
            {
                /* This means the delay is complete. Clear the Soft Reset  */
                PLIB_USBHS_SoftResetDisable(usbID);

                /* Setup the USB Module based on the selected
                 * mode */

                switch(hDriver->operationMode)
                {
                    case DRV_USBHS_OPMODE_DEVICE:
                        
                        /* Configure the driver object for device mode operation.
                         * In the PIC32MZ DA devices, the USBCRCON register needs
                         * to configured */
                        
                        _DRV_USBHS_CLOCK_CONTROL_SETUP_DEVICE_MODE
                        _DRV_USBHS_DEVICE_INIT(hDriver, object);
                        break;
                        
                    case DRV_USBHS_OPMODE_HOST:
                       
                        /* Configure the driver object for host mode operation.
                         * In the PICMZ DA devices, the USBCRCON register needs
                         * to configured. */
                        _DRV_USBHS_HOST_INIT(hDriver, object);
                        _DRV_USBHS_CLOCK_CONTROL_SETUP_HOST_MODE
                                       
                        break;
                        
                    case DRV_USBHS_OPMODE_OTG:
                        break;
                    default:
                        SYS_DEBUG(SYS_ERROR_INFO, "What mode are you trying?");
                        break;
                }

                /* Clear and enable the interrupts */
                _DRV_USBHS_InterruptSourceClear(hDriver->interruptSource);
                _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSource);
                
                _DRV_USBHS_InterruptSourceClear(hDriver->interruptSourceUSBDma);
                _DRV_USBHS_InterruptSourceEnable(hDriver->interruptSourceUSBDma);


                /* Indicate that the object is ready and change the state to running
                 * */

                hDriver->status = SYS_STATUS_READY;
                hDriver->state = DRV_USBHS_TASK_STATE_RUNNING;
            }

            break;

        case DRV_USBHS_TASK_STATE_RUNNING:
            /* The module is in a running state. In the polling mode the 
             * driver ISR tasks and DMA ISR tasks are called here. We also
             * check for the VBUS level and generate events if a client 
             * event handler is registered. */

            if(hDriver->pEventCallBack != NULL && hDriver->operationMode == DRV_USBHS_OPMODE_DEVICE)
            {
                /* We have a valid client call back function. Check if
                 * VBUS level has changed */

                USBHS_VBUS_LEVEL vbusLevel = PLIB_USBHS_VBUSLevelGet(hDriver->usbID);
                if(hDriver->vbusLevel != vbusLevel)
                {
                    /* This means there was a change in the level */
                    if((vbusLevel >= USBHS_VBUS_BELOW_VBUSVALID))
                    {
                        if(((vbusLevel == USBHS_VBUS_VALID) && (hDriver->vbusLevel == USBHS_VBUS_BELOW_VBUSVALID))
                                || ((vbusLevel == USBHS_VBUS_BELOW_VBUSVALID) && (hDriver->vbusLevel == USBHS_VBUS_VALID)))
                        {
                            /* Do not do anything here. If the code flow comes
                             * here, it means that VBUS transition from
                             * USBHS_VBUS_BELOW_VBUSVALID to USBHS_VBUS_VALID or
                             * vice versa. The callback call and necessary data
                             * structures have already been updated. */
                        }
                        else
                        {
                            /* We have a valid VBUS level */
                            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBHS_EVENT_DEVICE_SESSION_VALID, NULL);

                            /* We should be ready for send session invalid event
                             * to the application when they happen.*/
                            hDriver->sessionInvalidEventSent = false;
                        }

                    }
                    else
                    {
                        /* Any thing other than valid is considered invalid.
                         * This event may occur multiple times, but we send
                         * it only once. */
                        if(!hDriver->sessionInvalidEventSent)
                        {
                            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBHS_EVENT_DEVICE_SESSION_INVALID, NULL);
                            hDriver->sessionInvalidEventSent = true;
                        }
                    }

                    hDriver->vbusLevel = vbusLevel;
                }
            }
            else if(hDriver->operationMode == DRV_USBHS_OPMODE_HOST)
            {
                _DRV_USBHS_HOST_ATTACH_DETACH_STATE_MACHINE(hDriver);
                _DRV_USBHS_HOST_RESET_STATE_MACINE(hDriver);
            }

            _DRV_USBHS_Tasks_ISR(object);
            _DRV_USBHS_Tasks_ISR_USBDMA(object);
            break;
    }
}/* end of DRV_USBHS_Tasks() */

// *****************************************************************************
/* Function:
    void DRV_USBHS_Deinitialize( const SYS_MODULE_OBJ object )

  Summary:
    Dynamic impementation of DRV_USBHS_Deinitialize system interface function.

  Description:
    This is the dynamic impementation of DRV_USBHS_Deinitialize system interface
    function.

  Remarks:
    See drv_usbhs.h for usage information.
*/

void DRV_USBHS_Deinitialize 
( 
    const SYS_MODULE_INDEX  object
)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * drvObj;
    bool returnValue = false;
    /* End of local variables */

    if(object == SYS_MODULE_OBJ_INVALID)
    {
        /* Invalid object */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\r\nUSBHS Driver: Invalid object in DRV_USBHS_Deinitialize()");
    }
    else
    {
        if( object >= DRV_USBHS_INSTANCES_NUMBER)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\r\nUSBHS Driver: Invalid object in DRV_USBHS_Deinitialize()");
        }
        else
        {
            if(gDrvUSBObj[object].inUse == false)
            {
                /* Cannot de-initialize an object that is not already in use. */
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\r\nUSBHS Driver: Driver not initialized in DRV_USBHS_Deinitialize()");
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

                returnValue = _DRV_USBHS_InterruptSourceDisable(drvObj->interruptSource);
                returnValue = returnValue;
                _DRV_USBHS_InterruptSourceClear(drvObj->interruptSource);

                drvObj->isOpened = false;
                drvObj->pEventCallBack = NULL;

                /* Delete the mutex */
                if(OSAL_MUTEX_Delete(&drvObj->mutexID) != OSAL_RESULT_TRUE)
                {
                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\r\nUSBHS Driver: Could not delete mutex in DRV_USBHS_Deinitialize()");
                }
            }
        }
    }

    return;

} /* end of DRV_USBHS_Deinitialize() */

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_USBHS_Status( const SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_USBHS_Status system interface function.

  Description:
    This is the dynamic implementation of DRV_USBHS_Status system interface
    function.

  Remarks:
    See drv_usbhs.h for usage information.
*/

SYS_STATUS DRV_USBHS_Status ( SYS_MODULE_OBJ object )
{
    SYS_STATUS result = SYS_STATUS_ERROR;

    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSBHS Driver: System Module Object is invalid in DRV_USBHS_Status().");
    }
    else
    {
        result = gDrvUSBObj[object].status;
    }
       
    /* Return the status of the driver object */
    return(result);

}/* end of DRV_USBHS_Status() */


DRV_HANDLE DRV_USBHS_Open
(
    const SYS_MODULE_INDEX iDriver,
    const DRV_IO_INTENT    ioIntent 
)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * drvObj;
    DRV_HANDLE handle = DRV_HANDLE_INVALID;
    /* End of local variables */

    /* The iDriver value should be valid. It should be less the number of driver
     * object instances.  */

    if(iDriver >= DRV_USBHS_INSTANCES_NUMBER)
    {
        SYS_DEBUG(SYS_ERROR_DEBUG, "\r\nUSBHS Driver: Bad Driver Index in DRV_USBHS_Open().");
    }
    else
    {
        drvObj = &gDrvUSBObj[iDriver];

        if(drvObj->status == SYS_STATUS_READY)
        {
            if(ioIntent != (DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_NONBLOCKING |DRV_IO_INTENT_READWRITE))
            {
                /* The driver only supports this mode */
                SYS_DEBUG(SYS_ERROR_DEBUG, "\r\nUSBHS Driver: Unsupported IO Intent in DRV_USBHS_Open().");
            }
            else
            {
                if(drvObj->isOpened)
                {
                    /* Driver supports exclusive open only */
                    SYS_DEBUG(SYS_ERROR_DEBUG, "\r\nUSBHS Driver: Driver can be opened only once. Multiple calls to DRV_USBHS_Open().");
                }
                else
                {
                    /* Clear prior value */
                    drvObj->pEventCallBack = NULL;

                    /* Store the handle in the driver object client table and update
                     * the number of clients*/
                    drvObj->isOpened = true;
                    handle = ((DRV_HANDLE)drvObj);
                    SYS_DEBUG(SYS_ERROR_INFO, "\r\nUSBHS Driver: Driver opened successfully in DRV_USBHS_Open().");
                }
            }
        }
    }

    /* Return the client object */

    return (handle);

}/* end of DRV_USBHS_Open() */

// *****************************************************************************
/* Function:
    void DRV_USBHS_Close( DRV_HANDLE client )

  Summary:
    Dynamic implementation of DRV_USBHS_Close client interface function.

  Description:
    This is the dynamic implementation of DRV_USBHS_Close client interface
    function.

  Remarks:
    See drv_usbhs.h for usage information.
*/

void DRV_USBHS_Close( DRV_HANDLE client )
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    /* end of local variables */

    if(client == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "Bad Client Handle");
        return;
    }

    hDriver = (DRV_USBHS_OBJ *) client;
    
    if(!(hDriver->isOpened))
    {
        SYS_DEBUG(SYS_ERROR_INFO, "Invalid client handle");
        return;
    }
    /* Give back the client */
    hDriver->isOpened = false;
    hDriver->pEventCallBack = NULL;
}/* end of DRV_USBHS_Close() */

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_USBHS_Tasks_ISR( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_USBHS_Tasks_ISR system interface function.

  Description:
    This is the dynamic implementation of DRV_USBHS_Tasks_ISR system interface
    function.

  Remarks:
    See drv_usbhs.h for usage information.
*/

void DRV_USBHS_Tasks_ISR( SYS_MODULE_OBJ object )
{
    /* Start of local variables */
    DRV_USBHS_OBJ * 	hDriver;
    /* end of local variables */

    hDriver = &gDrvUSBObj[object];
    hDriver->isInInterruptContext = true;

	switch(hDriver->operationMode)
	{
        case DRV_USBHS_OPMODE_DEVICE:
            _DRV_USBHS_DEVICE_TASKS_ISR(hDriver);
            break;
        case DRV_USBHS_OPMODE_HOST:
            _DRV_USBHS_HOST_TASKS_ISR(hDriver);
            break;
        case DRV_USBHS_OPMODE_OTG:
            break;
        default:
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "\r\nUSBHS Driver: What mode are you trying?");
            break;
	}	
  
    /* Clear the interrupt */
    _DRV_USBHS_PersistentInterruptSourceClear(hDriver->interruptSource);
    hDriver->isInInterruptContext = false;
}/* end of DRV_USBHS_Tasks_ISR() */

void DRV_USBHS_Tasks_ISR_USBDMA
(
    SYS_MODULE_OBJ object
)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * 	hDriver;
    /* end of local variables */

    hDriver = &gDrvUSBObj[object];
    hDriver->isInInterruptContextUSBDMA = true;

	switch(hDriver->operationMode)
	{
        case DRV_USBHS_OPMODE_DEVICE:
            _DRV_USBHS_DEVICE_TASKS_ISR_USBDMA(hDriver);
            break;
        case DRV_USBHS_OPMODE_HOST:
            _DRV_USBHS_HOST_TASKS_ISR_USBDMA(hDriver);
            break;
        case DRV_USBHS_OPMODE_OTG:
            break;
        default:
            SYS_DEBUG(SYS_ERROR_INFO, "What mode are you trying?");
            break;
	}
        
    /* Clear the interrupt */
    hDriver->isInInterruptContextUSBDMA = false;
    _DRV_USBHS_InterruptSourceClear(hDriver->interruptSourceUSBDma);

}/* end of DRV_USBHS_Tasks_ISR()*/


void DRV_USBHS_ResumeControl(DRV_HANDLE hClient, bool control)
{
    /* Start of local variables */
    DRV_USBHS_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
    /* end of local variables */

    if((hClient == DRV_HANDLE_INVALID))
    {
        SYS_DEBUG(SYS_ERROR_INFO, "Invalid client");
    }

    hDriver = (DRV_USBHS_OBJ *)hClient;
    usbID = hDriver->usbID;

    if(control)
    {
        PLIB_USBHS_ResumeEnable(usbID);
    }
    else
    {
        PLIB_USBHS_ResumeDisable(usbID);
    }
}/* end of DRV_USBHS_ResumeControl() */

// *****************************************************************************
/* Function:
    bool DRV_USBHS_HOST_Resume(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHS_HOST_Resume
    client interface function.

  Description:
    This is the dynamic implementation of DRV_USBHS_HOST_Resume client
    interface function. Function resumes a suspended BUS.

  Remarks:
    See drv_usbhs.h for usage information.
*/

bool DRV_USBHS_HOST_Resume
(
    DRV_HANDLE handle
)
{
    /* Start of local variable */
    DRV_USBHS_OBJ * pusbdrvObj = (DRV_USBHS_OBJ *)NULL;
    bool returnValue = false;
    /* End of local variable */
    
    /* Check if the handle is valid */
    if((handle == DRV_HANDLE_INVALID))
    {
        SYS_DEBUG(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        pusbdrvObj = (DRV_USBHS_OBJ *)handle;

        PLIB_USBHS_ResumeEnable(pusbdrvObj->usbID);
        returnValue = true;
    }
    return returnValue;

}/* end of DRV_USBHS_HOST_Resume() */

// *****************************************************************************
/* Function:
    bool DRV_USBHS_HOST_Suspend(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USBHS_HOST_Suspend
    client interface function.

  Description:
    This is the dynamic implementation of DRV_USBHS_HOST_Suspend client
    interface function. Function suspends USB BUS.

  Remarks:
    See drv_usbhs.h for usage information.
*/

bool DRV_USBHS_HOST_Suspend
(
    DRV_HANDLE handle
)
{	
    /* Start of local variable */
    DRV_USBHS_OBJ * pusbdrvObj = (DRV_USBHS_OBJ *)NULL;
    bool returnValue = false;
    /* End of local variable */

    /* Check if the handle is valid */
    if((handle == DRV_HANDLE_INVALID))
    {
        SYS_DEBUG(SYS_ERROR_INFO, "Bad Client or client closed");
    }
    else
    {
        pusbdrvObj = (DRV_USBHS_OBJ *)handle;

        /* Suspend the bus */
        PLIB_USBHS_SuspendEnable(pusbdrvObj->usbID);
        returnValue = true;
    }
    return returnValue;

}/* end of DRV_USBHS_HOST_Suspend() */

// *****************************************************************************
/* Function:
    void DRV_USBHS_ClientEventCallBackSet
    (
        DRV_HANDLE   handle,
        uintptr_t    hReferenceData,
        DRV_USBHS_EVENT_CALLBACK eventCallBack
    )

  Summary:
    Dynamic implementation of DRV_USBHS_ClientEventCallBackSet client interface
    function.

  Description:
    This is the dynamic implementation of DRV_USBHS_ClientEventCallBackSet
    client interface function.

  Remarks:
    See drv_usbhs.h for usage information.
*/

void DRV_USBHS_ClientEventCallBackSet
( 
    DRV_HANDLE   client          ,
    uintptr_t    hReferenceData ,
    DRV_USB_EVENT_CALLBACK eventCallBack 
)
{
    /* Start of local variable */
    DRV_USBHS_OBJ * pusbDrvObj;
    /* end of local variable */
    
    if(client == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "Bad Client Handle");
        return;
    }

    pusbDrvObj = (DRV_USBHS_OBJ *) client;
    
    if(!pusbDrvObj->isOpened)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "Invalid client handle");
        return;
    }

    /* Assign event call back and reference data */
    pusbDrvObj->hClientArg = hReferenceData;
    pusbDrvObj->pEventCallBack = eventCallBack;
   
    return;
    
}/* end of DRV_USBHS_ClientEventCallBackSet() */
