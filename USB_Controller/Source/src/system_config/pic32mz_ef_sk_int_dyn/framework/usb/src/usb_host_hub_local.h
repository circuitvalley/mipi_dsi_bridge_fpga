/*******************************************************************************
  USB HOST HUB Local definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_hub_local.h

  Summary:
    USB HOST HUB Local definitions

  Description:
    This file contains local declarations and defintions required by the Hub
    Driver. This file or its contents should not be directly accessed by the
    application.
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

#ifndef _USB_HOST_HUB_LOCAL_H
#define _USB_HOST_HUB_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system/tmr/sys_tmr.h"

/****************************************
 * This is needed by the implementation
 ****************************************/

#define USB_HOST_HUB_PORT_REQUEST    0x00
#define USB_HOST_HUB_REQUEST         0x01

/************************************************
 * Defines the the masks needed for creating the
 * control transfer contexts.
 *************************************************/

#define PORT_INDEX_MASK 0xFFFF0000
#define HUB_INDEX_MASK  0x0000FFFF

#define   USB_HUB_CLASS_STANDARD_HUB_REQUEST        0xA0
#define   USB_HUB_CLASS_STANDARD_PORT_REQUEST       0xA3

/*************************************************
 * These are the possible commands that Host Layer
 * can issue to the port.
 **************************************************/

typedef enum
{
    /* A reset command was requested on the port */
    USB_HOST_PORT_COMMAND_RESET = 1 ,

    /* A suspend command was requested on the port */
    USB_HOST_PORT_COMMAND_SUSPEND = 2,

    /* A resume command was requested on the port */
    USB_HOST_PORT_COMMAND_RESUME = 4,
   
} USB_HOST_HUB_PORT_COMMAND ;

/****************************************************
 * Defines the possible states the Hub Device Task
 * Routine.
 ****************************************************/

typedef enum
{
    /* Hub Device Tasks gets the device status */
    USB_HOST_HUB_STATE_DEVICE_STATUS_GET,

    /* Hub Device Tasks waits for device status get */
    USB_HOST_HUB_STATE_WAIT_FOR_DEVICE_STATUS_GET,

    /* Hub Device Task gets the configuration descriptor */
    USB_HOST_HUB_STATE_GET_CONFIGURATION ,

    /* Hub Device Task is waiting for get configuration to complete */
    USB_HOST_HUB_STATE_WAIT_FOR_CONFIGURATION_DESCRIPTOR_GET,

    /* Hub Device Task sets configuration */
    USB_HOST_HUB_STATE_CONFIGURATION_SET,

    /* Hub Device Task is waiting for set configuration to complete */
    USB_HOST_HUB_STATE_WAIT_FOR_CONFIGURATION_SET,

    /* Hub Device Task is waiting for interface to be configured */
    USB_HOST_HUB_STATE_WAIT_FOR_INTERFACE_READY,

    /* Hub Device Task gets the Hub descriptor */
    USB_HOST_HUB_STATE_HUB_DESCRIPTOR_GET,

    /* Hub Device Tasks waits for Hub Descriptor Get to complete */
    USB_HOST_HUB_STATE_WAIT_FOR_HUB_DESCRIPTOR_GET,

    /* Hub is in a running state */
    USB_HOST_HUB_STATE_RUNNING,

    /* Hub is in an error state */
    USB_HOST_HUB_STATE_ERROR,
    
} USB_HOST_HUB_STATE;

/********************************************************
 * Defines the possible states of the Hub Task Routine.
 ********************************************************/

typedef enum
{
    USB_HOST_HUB_TASK_STATE_HUB_STATUS_GET ,
    USB_HOST_HUB_TASK_STATE_WAIT_FOR_HUB_STATUS_GET,
    USB_HOST_HUB_TASK_STATE_LPS_CHANGE_CLEAR,
    USB_HOST_HUB_TASK_STATE_WAIT_FOR_LPS_CHANGE_CLEAR,
    USB_HOST_HUB_TASK_STATE_CHECK_OC_STATUS,
    USB_HOST_HUB_TASK_STATE_OC_CHANGE_CLEAR_SEND,
    USB_HOST_HUB_TASK_STATE_WAIT_FOR_OC_CHANGE_CLEAR,
    USB_HOST_HUB_TASK_STATE_OC_CLEAR_SEND,
    
} USB_HOST_HUB_TASK_STATE;

/********************************************************
 * Defines the possible states of the Port Task Routines
 ********************************************************/

typedef enum
{
    /* Enable the power to the port, provide the power on delay 
     * and wait for the power on delay to complete */
    USB_HOST_HUB_PORT_TASK_STATE_POWER_ENABLE,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_POWER_ENABLE,
    USB_HOST_HUB_PORT_TASK_STATE_PWRON2PWRGOOD_DELAY,
    USB_HOST_HUB_PORT_TASK_STATE_PWRON2PWRGOOD_DELAY_WAIT,

    /* Check if there is any change. If so then get the port status */
    USB_HOST_HUB_PORT_TASK_STATE_CHECK_CHANGE_STATUS,
    USB_HOST_HUB_PORT_TASK_STATE_PORT_STATUS_GET,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_STATUS_GET,

    /* If a device was connected to the port, then process this and clear the
     * port connect change status bit. */
    USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_CONNECT,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_CLEAR_FEATURE_PORT_CONNECT,
    USB_HOST_HUB_PORT_TASK_STATE_POST_PORT_CONNECT_STATUS_GET,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_POST_PORT_CONNECT_STATUS_GET,

    /* Check if the device was disable, this would set the port enable change
     * bit */
    USB_HOST_HUB_PORT_TASK_STATE_CHECK_PORT_ENABLE_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_ENABLE_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_ENABLE_CHANGE,
    
    /* Check if the resume operation has completed */
    USB_HOST_HUB_PORT_TASK_STATE_CHECK_SUSPEND_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_SUSPEND_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_SUSPEND_CHANGE,

    /* Check if overcurrent condition has occurred */
    USB_HOST_HUB_PORT_TASK_STATE_CHECK_OC_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_OC_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_OC_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_CHECK_OVERCURRENT_STATUS,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_OVERCURRENT_STATUS,

    /* Check if reset has completed */
    USB_HOST_HUB_PORT_TASK_STATE_CHECK_PORT_RESET_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_CLEAR_FEATURE_PORT_RESET_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_FEATURE_PORT_RESET_CHANGE,
    USB_HOST_HUB_PORT_TASK_STATE_POST_RESET_COMPLETE_STATUS_GET,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_POST_RESET_COMPLETE_STATUS_GET,

    /* Command execution states */
    USB_HOST_HUB_PORT_TASK_STATE_CHECK_COMMAND,
    USB_HOST_HUB_PORT_TASK_STATE_PORT_RESET,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_RESET_COMMAND,
    USB_HOST_HUB_PORT_TASK_STATE_CLEAR_TT_BUFFER,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_CLEAR_TT_BUFFER,
    USB_HOST_HUB_PORT_TASK_STATE_PORT_SUSPEND,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_SUSPEND_COMMAND,
    USB_HOST_HUB_PORT_TASK_STATE_PORT_RESUME,
    USB_HOST_HUB_PORT_TASK_STATE_WAIT_FOR_PORT_RESUME_COMMAND,
    USB_HOST_HUB_PORT_TASK_STATE_OVERCURRENT

} USB_HOST_HUB_PORT_TASK_STATE;

/*********************************************
 * Defines the Hub Port Info Object
 *********************************************/

typedef struct 
{
    /* Tracks status of the port task routine */
    USB_HOST_HUB_PORT_TASK_STATE portTaskState;

    /* This is port status that is received from the hub */
    USB_HOST_PORT_STATUS portStatus;

    /* Is set to true if a port related control request has completed */
    bool controlRequestDone;

    /* Stores the result of the completed control result */
    USB_HOST_RESULT controlRequestResult;

    /* Device Object handle of the device that is connected to this port */
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle;

    /* This is bit map of the currently requested port command */
    USB_HOST_HUB_PORT_COMMAND portCommand;

    /* This setup packet stores the setup command to be sent to the port */
    USB_SETUP_PACKET setupPacket;

    /* Timer handle required to track delays */
    SYS_TMR_HANDLE timerHandle;

    /* Set to true if the port is enabled */
    bool isEnabled;

    /* Tracks the speed of the port */
    USB_SPEED portSpeed;

    /* Is true if resume has completed */
    bool isResumed;

    /* Is true if port is suspended */
    bool isSuspended;

    /* Is true is port is facing an overcurrent condition */
    bool overCurrent;
    
    /* Is true if port is reset */
    bool isResetComplete;

    /* Is true if the port is powered off because it in an overcurrent state or
     * because it is an affected port */
    bool isOCPoweredOff;
   
} USB_HOST_HUB_PORT_INFO;

/**************************************************
 * Defines the Hub Instance Object. Maintained
 * for every connected Hub.
 **************************************************/

typedef struct _USB_HOST_HUB_INSTANCE_OBJ_
{
    /* True if this object is allocated */
    bool inUse;

    /* Setup packet that will be used by this hub instance */
    USB_SETUP_PACKET setupPacket;
    
    /* The device status */
    USB_DEVICE_GET_STATUS_RESPONSE deviceStatus;
    
    /* Interface handle provided by the host when a hub interface is found */
    USB_HOST_DEVICE_INTERFACE_HANDLE hubInterfaceHandle;
    
    /* Device Object Handle assigned by the host */
    USB_HOST_DEVICE_OBJ_HANDLE hubObjHandle;
    
    /* Control Pipe Handle */
    USB_HOST_CONTROL_PIPE_HANDLE controlPipeHandle;

    /* Interrupt Pipe Handle */
    USB_HOST_PIPE_HANDLE interruptInPipeHandle;
    
    /* Hub Descriptor obtained from the Hub */
    USB_HUB_DESCRIPTOR hubDescriptor;

    /* State of the hub device task */
    USB_HOST_HUB_STATE hubInstanceState;
    
    /* State of the hub task */
    USB_HOST_HUB_TASK_STATE hubTaskState;

    /* Device client handle provided by the host */
    USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceClientHandle;
    
    /* Port database */
    USB_HOST_HUB_PORT_INFO portInfo[ USB_HOST_HUB_PORTS_NUMBER ];
    
    /* Hub status that is obtained from the hub */
    USB_HUB_STATUS hubStatus;

    /* Interrupt Transfer Handle */
    USB_HOST_TRANSFER_HANDLE interruptTransferHandle;

    /* Control Transfer Handle */
    USB_HOST_TRANSFER_HANDLE controlTransferHandle;
    
    /* Change status obtained from the hub over the interrupt pipe */
    uint32_t changeStatus;

    /* True if the control request is done */
    bool controlRequestDone;

    /* True if interrupt transfer is done */
    bool interruptTransferDone;

    /* Tracks the result of the control transfer */
    USB_HOST_RESULT controlRequestResult;

    /* True if the hub is bus powered */
    bool isBusPowered;
   
    /* True is the the hub is in an over current state */ 
    bool isOverCurrent;
        
} USB_HOST_HUB_INSTANCE_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_Initialize(void * hubInitData)

  Summary:
    This function is called when the Host Layer is initializing.

  Description:
    This function is called when the Host Layer is initializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_Initialize(void * hubInitData );

//*****************************************************************************
/* Function:
    void _USB_HOST_HUB_Deinitialize(void)

  Summary:
    This function is called when the Host Layer is deinitializing.

  Description:
    This function is called when the Host Layer is deinitializing .

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_Deinitialize( void );

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_Reinitialize(void)

  Summary:
    This function is called when the Host Layer is reinitializing.

  Description:
    This function is called when the Host Layer is reinitializing.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_Reinitialize(void * hubInitData);

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_DeviceAssign 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,,
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        USB_DEVICE_DESCRIPTOR * deviceDescriptor ,
    )

  Summary:
    This function is called when the Host Layer attaches this driver to an
    device.

  Description:
    This function is called when the Host Layer attaches this driver to an
    device level.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_DeviceAssign 
( 
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_DEVICE_DESCRIPTOR * deviceDescriptor 
);

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_DeviceRelease 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceHandle
    )
 
  Summary: 
    This function is called when the device is detached. 

  Description:
    This function is called when the device is detached. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_DeviceRelease
(
    USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceHandle
);

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_DeviceTasks 
    (
        USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceHandle
    )
 
  Summary: 
    This function is called when the host layer want the device to update its
    state. 

  Description:
    This function is called when the host layer want the device to update its
    state. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_DeviceTasks
(
    USB_HOST_DEVICE_CLIENT_HANDLE hubDeviceHandle
);

// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HUB_DeviceEventHandler
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
        USB_HOST_DEVICE_EVENT event,
        void * eventData,
        uintptr_t context
    )

  Summary:
    This function is called when the Host Layer generates device level
    events. 

  Description:
    This function is called when the Host Layer generates device level
    events. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_DEVICE_EVENT_RESPONSE _USB_HOST_HUB_DeviceEventHandler
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceHandle,
    USB_HOST_DEVICE_EVENT event,
    void * eventData,
    uintptr_t context
);

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_InterfaceAssign 
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        size_t nInterfaces,
        uint8_t * descriptor 
    );

  Summary:
    This function is called when the Host Layer attaches this driver to an
    interface.

  Description:
    This function is called when the Host Layer attaches this driver to an
    interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_InterfaceAssign 
(
    USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    size_t nInterfaces,
    uint8_t * descriptor 
);
  
// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_InterfaceRelease 
   (
        USB_HOST_DEVICE_INTERFACE_HANDLE interface 
   )

  Summary:
    This function is called when the Host Layer detaches this driver from an
    interface.

  Description:
    This function is called when the Host Layer detaches this driver from an
    interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_InterfaceRelease (USB_HOST_DEVICE_INTERFACE_HANDLE interface);
  
// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HUB_InterfaceEventHandler
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
        USB_HOST_DEVICE_INTERFACE_EVENT event,
        void * eventData,
        uintptr_t context
    )

  Summary:
    This function is called when the Host Layer generates interface level
    events. 

  Description:
    This function is called when the Host Layer generates interface level
    events. 

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HUB_InterfaceEventHandler
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_DEVICE_INTERFACE_EVENT event,
    void * eventData,
    uintptr_t context
);
  
// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_InterfaceTasks 
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    )
 
  Summary:
    This function is called by the Host Layer to update the state of this
    driver.

  Description:
    This function is called by the Host Layer to update the state of this
    driver.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_InterfaceTasks (USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle);
  
// *****************************************************************************
/* Function:
    int _USB_HOST_HUB_DeviceHandleToInstance
    (
        USB_HOST_DEVICE_CLIENT_HANDLE deviceClientHandle
    );

  Summary:
    This function will return the index of the HUB object that own this device.

  Description:
    This function will return the index of the HUB object that own this device.
    If an instance is not found, the function will return -1.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int _USB_HOST_HUB_DeviceHandleToInstance
(
    USB_HOST_DEVICE_CLIENT_HANDLE deviceClientHandle
);

// ****************************************************************************
/* Function:
    USB_SPEED  USB_HOST_HUB_PortSpeedGet
    (
        uintptr_t deviceHandle, 
        uint8_t port
    )

  Summary:
    Returns the speed at which the bus to which this hub is connected is
    operating.

  Description:
    This function returns the speed at which the bus to which this port is
    connected is operating.

 Remarks:
    None.
*/

USB_SPEED  USB_HOST_HUB_PortSpeedGet
(
    uintptr_t deviceHandle, 
    uint8_t portNumber
);

// *****************************************************************************
/* Function:
    void  _USB_HOST_HUB_ControlTransferComplete
    (
      USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
      USB_HOST_REQUEST_HANDLE requestHandle,
      USB_HOST_RESULT result,
      uint32_t size,
      uintptr_t context
    )

  Summary:
    This function is called when a control transfer initiated by the hub driver
    completes.

  Description:
    This function is called when a control transfer initiated by the hub driver
    completes.
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void  _USB_HOST_HUB_ControlTransferComplete
(
   USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
   USB_HOST_REQUEST_HANDLE requestHandle,
   USB_HOST_RESULT result,
   size_t size,
   uintptr_t context
);

// ****************************************************************************
/* Function:
    USB_ERROR USB_HOST_HUB_PortResume 
    ( 
        uintptr_t deviceHandle, 
        uint8_t portNumber
    )

  Summary:
    Request the hub driver to resume the port.

  Description:
    This function requests the hub driver to resume the port. The actual resume
    operation is completed in the _USB_HOST_HUB_PortTasks() function.
    
  Remarks:
    This function is called by the host layer.
*/

USB_ERROR USB_HOST_HUB_PortResume 
( 
    uintptr_t deviceHandle, 
    uint8_t portNumber
);

// ****************************************************************************
/* Function:
    USB_ERROR USB_HOST_HUB_PortSuspend 
    ( 
        uintptr_t deviceHandle, 
        uint8_t portNumber
    )

  Summary:
    Request the hub driver to suspend the port.

  Description:
    This function requests the hub driver to suspend the port. The actual suspend
    operation is completed in the _USB_HOST_HUB_PortTasks() function.
    
  Remarks:
    This function is called by the host layer.
*/

USB_ERROR USB_HOST_HUB_PortSuspend 
(
    uintptr_t deviceHandle, 
    uint8_t portNumber
);

// ****************************************************************************
/* Function:
    bool USB_HOST_HUB_PortResetComplete 
    (
        uintptr_t deviceHandle,
        uint8_t portNumber
    );

  Summary:
    Returns true if the root hub has completed the port reset operation.

  Description:
    This function returns true if the port reset operation has completed. It
    should be called after the USB_HOST_HUB_PortReset() function to
    check if the reset operation has completed.

  Remarks:
    This function is called by the host layer.
*/

bool USB_HOST_HUB_PortResetComplete 
(
   uintptr_t deviceHandle, 
   uint8_t portNumber 
);

// ****************************************************************************
/* Function:
    bool USB_HOST_HUB_PortResetComplete 
    (
        uintptr_t deviceHandle,
        uint8_t portNumber
    );

  Summary:
    Returns true if the root hub has completed the port reset operation.

  Description:
    This function returns true if the port reset operation has completed. It
    should be called after the USB_HOST_HUB_PortReset() function to
    check if the reset operation has completed.

  Remarks:
    This function is typically called by the host layer.
*/

USB_ERROR USB_HOST_HUB_PortReset (uintptr_t deviceHandle, uint8_t portNumber);

// *****************************************************************************
/* Function:
    USB_HOST_RESULT _USB_HOST_HUB_HubDescriptorGet (int hubInstanceIndex )

  Summary:
    Launches control transfers to get the Hub Descriptor. 

  Description:
    Launches control transfers to get the Hub Descriptor. 
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_RESULT _USB_HOST_HUB_HubDescriptorGet (int hubInstanceIndex );

// *****************************************************************************
/* Function:
    USB_HOST_RESULT  _USB_HOST_HUB_featureRequest 
    (
        USB_HOST_HUB_DRIVER_INFO *hubDriverInstanceInfo,
        uint32_t portNumber ,
        uint8_t Feature, 
        uint8_t hubCommand ,
        uint8_t requestType
    )
 
  Summary:
    Launches control transfers targetted at the hub or port. 

  Description:
    This function is called by the Hub Driver to launch set or clear feature
    control transfers targetted to the hub or port.
  
  Remarks:
    This is local function and should not be called directly by the application.    
*/

USB_HOST_RESULT  _USB_HOST_HUB_FeatureRequest 
(
    uint32_t hubInstanceIndex,
    uint32_t portNumber ,
    uint8_t feature, 
    uint8_t hubCommand ,
    uint8_t requestType
);

// *****************************************************************************
/* Function:
    USB_HOST_RESULT USB_HOST_HUB_StatusRequest 
    ( 
        USB_HOST_HUB_DRIVER_INFO *hubDriverInstnceInfo, 
        USB_HOST_HUB_STATUS  *hubStatus ,
        uint8_t portNumber , 
        uint8_t requestType 
    )
 
  Summary:
    Launches control transfers to read hub or port status.

  Description:
    Launches control transfers to read hub or port status.
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
    
*/

USB_HOST_RESULT _USB_HOST_HUB_StatusRequest 
( 
    int hubInstanceIndex,
    void * status ,
    uint8_t portNumber , 
    uint8_t requestType 
);

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_HubTasks( uint32_t hubInstanceIndex )
 
  Summary: 
    This function maintains the hub level status.

  Description:
    This function maintains the hub level status. It is called periodically when
    the hub state is USB_HOST_HUB_STATE_RUNNING. It is called from the
    _USB_HOST_HUB_DeviceTasks() function. This function checks if the hub power
    status has changed or if an over current has occurred. It clear the hub
    change bits if required.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_HubTasks( uint32_t hubInstanceIndex );

// *****************************************************************************
/* Function:
    void _USB_HOST_HUB_PortTasks(uint32_t hubInstanceIndex, uint32_t portNumber)
 
  Summary: 
    This function maintains the port level status.

  Description:
    This function maintains the port level status. It is called periodically
    when the hub state is USB_HOST_HUB_STATE_RUNNING. It is called from the
    _USB_HOST_HUB_DeviceTasks() function. This function checks the state of all
    the change bits in wPortChange and responds to each change source.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HUB_PortTasks( uint32_t hubInstanceIndex , uint32_t portNumber );

// *****************************************************************************
/* Function:
    USB_HOST_RESULT _USB_HOST_HUB_PortTTBufferClear
    ( 
        int hubInstanceIndex,
        uint8_t deviceAddress,
        uint8_t endpoint,
        USB_TRANSFER_TYPE transferType, 
        uint8_t ttPortNumber, 
    )

  Summary:
    Launches control transfer to clear the hub port TT buffer

  Description:
    Launches control transfer to clear the hub port TT buffer
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
    
*/

USB_HOST_RESULT _USB_HOST_HUB_PortTTBufferClear 
( 
    int hubInstanceIndex,
    uint8_t deviceAddress,
    uint8_t endpoint,
    USB_TRANSFER_TYPE transferType, 
    uint8_t ttPortNumber, 
    uint8_t portNumber
);

// *****************************************************************************
/* Function:
    USB_HOST_RESULT _USB_HOST_HUB_HubDeviceStatusGet (int hubInstanceIndex )

  Summary:
    Launches control transfers to get the Hub Device Status. 

  Description:
    Launches control transfers to get the Hub Device Status. 
  
  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_RESULT _USB_HOST_HUB_HubDeviceStatusGet (int hubInstanceIndex );
 
#endif

