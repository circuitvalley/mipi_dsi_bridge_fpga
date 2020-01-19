/*******************************************************************************
  USB HOST Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_local.h

  Summary:
    USB HOST layer Local Data Structures

  Description:
    Host layer Local Data Structures
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _USB_HOST_LOCAL_H
#define _USB_HOST_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "usb/usb_common.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_host.h"
#include "usb/usb_host_client_driver.h"
#include "driver/usb/drv_usb.h"
#include "system/tmr/sys_tmr.h"
#include "osal/osal.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************


/* Enumeration retry count  */
#define  USB_HOST_ENUMERATION_RETRY_COUNT      3

/* Root Hub address is always this */
#define USB_HOST_ROOT_HUB_ADDRESS    1

/* Default address of a device attached to the bus */
#define USB_HOST_DEFAULT_ADDRESS     0

/* Invalid configuration value */
#define USB_HOST_CONFIGURATION_NUMBER_INVALID 0xFF

/* Definition of USB_HOST_DEVICE_OBJ_HANDLE 
 * Bits 31-16: Unique PNP identifier
 * Bits 15-8: Bus number
 * Bits 7-0 : Device Array Index */

#define _USB_HOST_DeviceObjHandleGet( pnpIdentifier, busNumber, deviceIndex) ( (uintptr_t)(((pnpIdentifier) << 16) | ((busNumber) << 8) | deviceIndex))

/* Definition of USB_HOST_DEVICE_CLIENT_HANDLE is the same as that of the
 * USB_HOST_DEVICE_OBJ_HANDLE */

/* Defintion of USB_HOST_CONTROL_PIPE_HANDLE is the same as that of the
 * USB_HOST_DEVICE_OBJ_HANDLE */

/* The USB_HOST_PIPE_HANDLE is a pointer to the USB_HOST_PIPE_OBJ */

/* The USB_HOST_TRANSFER_HANDLE is a pointer to the USB_HOST_TRANSFER_OBJ */

/* The defintion of USB_HOST_DEVICE_INTERFACE_HANDLE 
 * Bits 31-16: Unique PNP identifier
 * Bits 15-8: Interface number
 * Bits 7-0 : Device Array Index */

#define _USB_HOST_DeviceInterfaceHandleGet( pnpIdentifier, interfaceHandle, deviceIndex) ( (uintptr_t)(((pnpIdentifier) << 16) | ((interfaceHandle) << 8) | deviceIndex))

/* This macro creates a handle that is used as the user data for control
 * transfer IRPs */

#define _USB_HOST_ControlTransferIRPUserData( pnpIdentifier, transferObjIndex, deviceIndex) ( (uintptr_t)(((pnpIdentifier) << 16) | ((transferObjIndex) << 8) | deviceIndex))

/* Macros to obtain the bus number and device array index from the 
 * from the USB_HOST_DEVICE_OBJ_HANDLE */
#define USB_HOST_DEVICE_INDEX( X )      ((X) & 0x000000FF)
#define USB_HOST_BUS_NUMBER( X )        (( (X) & 0x0000FF00) >> 8 )
#define USB_HOST_INTERFACE_INDEX( X )   (( (X) & 0x0000FF00) >> 8)
#define USB_HOST_PNP_IDENTIFIER( X )    (( (X) & 0xFFFF0000) >> 16)

#define USB_HOST_QUERY_FLAG_MASK        0xFF

#define USB_HOST_DeviceIsRootHub(deviceObjHandle)  ((gUSBHostDeviceList[USB_HOST_DEVICE_INDEX(deviceObjHandle)].deviceAddress == USB_HOST_ROOT_HUB_ADDRESS) ? true : false)

 bool   _IS_PARENT_ROOT_HUB( USB_HOST_DEVICE_OBJ_HANDLE  parentDeviceIdentifier );

/* Call back should be function pointer */
typedef void *  USB_HOST_IRP_CALLBACK;

/* Standard USB config descriptor size */
#define    USB_HOST_CONFIG_DESCRPTR_SIZE        9

/* This values based on USB standards It will change based on
   controller driver */
#define  USB_HOST_FULL_SPEED_BANDWIDTH   900
#define  USB_HOST_HIGH_SPEED_BANDWIDTH   800
#define  USB_HOST_ENDPOINT_NUMBER      DRV_USB_HOST_PIPES_NUMBER

#ifndef USB_HOST_MALLOC
/* Dynamic allocation of memory */
#define USB_HOST_MALLOC(size)       malloc(size)
#endif

#ifndef USB_HOST_FREE
/* Free memory */
#define USB_HOST_FREE(ptr)          free(ptr)
#endif

// *****************************************************************************
/*  USB Host Layer Device State Enumeration

  Summary:
    USB Host Layer Device State Enumeration.

  Description:
    These are the possible states of the device attached to the host.

  Remarks:
    None.
*/

typedef enum
{
    USB_HOST_DEVICE_STATE_ERROR_HOLDING = -2 ,
    /* Device is in an error state */
    USB_HOST_DEVICE_STATE_ERROR = -1,

    /* Device is attached and is waiting for enumeration */
    USB_HOST_DEVICE_STATE_WAITING_FOR_ENUMERATION = 0,

    /* Device has been reset and is waiting for reset to complete */
    USB_HOST_DEVICE_STATE_WAITING_FOR_RESET_COMPLETE ,

    /* Start the post reset settling delay */
    USB_HOST_DEVICE_STATE_START_RESET_SETTLING_DELAY,

    /* Device is waiting for reset settling delay to complete */
    USB_HOST_DEVICE_STATE_WAITING_FOR_RESET_SETTLING_DELAY_COMPLETE,

    /* Device is waiting for Get short device descriptor to complete */
    USB_HOST_DEVICE_STATE_WAITING_FOR_GET_DEVICE_DESCRIPTOR_SHORT,

    /* The address of the device is set in this state */
    USB_HOST_DEVICE_STATE_SET_ADDRESS,

    /* Device is waiting for Set Address to complete */
    USB_HOST_DEVICE_STATE_WATING_FOR_SET_ADDRESS_COMPLETE,
            
    /* Provide some delay after the address has been set */
    USB_HOST_DEVICE_STATE_POST_SET_ADDRESS_DELAY,

    /* Wait for the delay to complete */
    USB_HOST_DEVICE_STATE_WAITING_POST_SET_ADDRESS_DELAY,

    /* The full device descriptor is obtained. */
    USB_HOST_DEVICE_STATE_GET_DEVICE_DESCRIPTOR_FULL,

    /* Host is waiting for Get Full Device Descriptor to complete */
    USB_HOST_DEVICE_STATE_WAITING_FOR_GET_DEVICE_DESCRIPTOR_FULL,

    /* Host wants to the get the short configuration descriptor */
    USB_HOST_DEVICE_STATE_GET_CONFIGURATION_DESCRIPTOR_SHORT,

    /* Host is waiting for Get Short Device Descriptor to complete */
    USB_HOST_DEVICE_STATE_WAITING_FOR_GET_CONFIGURATION_DESCRIPTOR_SHORT,

    /* Host wants to the get the full configuration descriptor */
    USB_HOST_DEVICE_STATE_GET_CONFIGURATION_DESCRIPTOR_FULL,

    /* Host is waiting for Get Configuration Descriptor Full */
    USB_HOST_DEVICE_STATE_WAITING_FOR_GET_CONFIGURATION_DESCRIPTOR_FULL,

    /* Device is ready to be used by the rest of the system */
    USB_HOST_DEVICE_STATE_READY

} USB_HOST_DEVICE_STATE;

// *****************************************************************************
/*  USB Host Layer Device State Configuration

  Summary:
    USB Host Layer Device State Configuration.

  Description:
    These are the possible states of the device as it is getting configured.

  Remarks:
    None.
*/

typedef enum
{
    /* Device is ready for configuration */
    USB_HOST_DEVICE_CONFIG_STATE_READY_FOR_CONFIG = 0x0,

    /* Device configuration has start */
    USB_HOST_DEVICE_CONFIG_STATE_START,

    /* Device configuration state get configuration descriptor header */
    USB_HOST_DEVICE_CONFIG_STATE_CONFIG_DESCRIPTOR_HEADER_GET,

    /* Device configuration state waiting for configuration header get */
    USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIG_DESCRIPTOR_HEADER_GET,

    /* Device configurtion state get full configuration descriptor */
    USB_HOST_DEVICE_CONFIG_STATE_CONFIG_DESCRIPTOR_GET,
    
    /* Device configuration state waiting for configuration descriptor get */
    USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIG_DESCRIPTOR_GET,

    /* Device configuration will be set */
    USB_HOST_DEVICE_CONFIG_STATE_CONFIGURATION_SET,

    /* Host is waiting for set configuration to complete */
    USB_HOST_DEVICE_CONFIG_STATE_WAIT_FOR_CONFIGURATION_SET

} USB_HOST_DEVICE_CONFIG_STATE;

// *****************************************************************************
/* USB Host Control Request Type

  Summary:
    USB Host Control Request Type.
  
  Description:
    This enumeration defines the different types of control requests.

  Remarks:
    None.
*/

typedef enum
{
    /* This is a client driver specific reuqest */
    USB_HOST_CONTROL_REQUEST_TYPE_CLIENT_DRIVER_SPECIFIC,

    /* Control request to get the configuration */
    USB_HOST_CONTROL_REQUEST_TYPE_PIPE_HALT_CLEAR,

    /* Control request to Set Interface */
    USB_HOST_CONTROL_REQUEST_TYPE_INTERFACE_SET,

    /* String descriptor request */
    USB_HOST_CONTROL_REQUEST_TYPE_STRING_DESCRIPTOR,

    /* Configuration descriptor Get request */
    USB_HOST_CONTROL_REQUEST_TYPE_CONFIGURATION_DESCRIPTOR_GET

} USB_HOST_CONTROL_REQUEST_TYPE;

// *****************************************************************************
/* USB Host Pipe Object

  Summary:
    USB Host Pipe Object

  Description:
    This data structre defines a Host Pipe Object.

  Remarks:
    None.
*/

typedef struct _USB_HOST_PIPE_OBJ_
{
    /* True if the pipe object is allocated */
    bool  inUse;

    /* Handle to the interface that this pipe belongs to */
    USB_HOST_DEVICE_INTERFACE_HANDLE  interfaceHandle;

    /* The driver pipe handle for this pipe */
    DRV_USB_HOST_PIPE_HANDLE pipeHandle;
    
    /* Device endpoint that this pipe connects to */
    USB_ENDPOINT_ADDRESS  endpointAddress;

} USB_HOST_PIPE_OBJ;

// *****************************************************************************
/* USB Host Control Transfer Object

  Summary:
    USB Host Control Transfer Object

  Description:
    This data structre defines a Host Control Transfer Object. A Control
    transfer object tracks a client control transfer request. 

  Remarks:
    None.
*/

typedef struct 
{
    /* True if this object is allocated */
    bool inUse;
    
    /* Transfer context */
    uintptr_t context;

    /* IRP that is used by this transfer object */
    USB_HOST_IRP controlIRP;

    /* Type of control request */
    USB_HOST_CONTROL_REQUEST_TYPE requestType;

    /* Callback to call when control transfer is complete */
    void * callback;

} USB_HOST_CONTROL_TRANSFER_OBJ;

// *****************************************************************************
/*  USB Host Interface Descriptor Information.

  Summary:
    USB Host Interface Descriptor Information.
 
  Description:
    This USB Host Layer object contains information about about the attached
    device interface. 

  Remarks:
    None.
*/

typedef struct _USB_HOST_INTERFACE_DESC_INFO_
{
    /* Interface descriptor for this interface */
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor;

    /* IAD associated with this interface */
    USB_INTERFACE_ASSOCIATION_DESCRIPTOR * interfaceAssociationDescriptor;

    /* Driver for this interface */
    USB_HOST_CLIENT_DRIVER * interfaceDriver;

    /* Interface handle */
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle;
    
    /* True if this interface was tried with a device level driver */
    bool wasTriedWithDeviceDriver;

    /* The TPL index that matched */
    int tplEntryMatched;

    /* Current alternate setting */
    uint8_t currentAlternateSetting;

    /* In case of an IAD this will point to the next interface that belongs to
     * this IAD group. */
    struct _USB_HOST_INTERFACE_DESC_INFO_ * nextInterface;

} USB_HOST_INTERFACE_DESC_INFO;

// *****************************************************************************
/* USB Host Transfer Object

  Summary:
    USB Host Transfer Object

  Description:
    This data structre defines a Host Transfer Object. A transfer object tracks
    a client transfer request. 

  Remarks:
    None.
*/

typedef struct 
{
    /* True if this object is allocated */
    bool inUse;
    
    /* Transfer context */
    uintptr_t context;

    /* IRP that is used by this transfer object */
    USB_HOST_IRP irp;

    /* Handle to the interface to which this IRP was submitted */
    USB_HOST_INTERFACE_DESC_INFO * interfaceInfoObj;

} USB_HOST_TRANSFER_OBJ;

// *****************************************************************************
/*  USB Host Device Configuration Information. 

  Summary:
    USB Host Device Configuration Information.

  Description:
    This USB Host Layer object contains information about the configuration of
    the attached device.

  Remarks:
    None.
*/

typedef struct 
{
    /* Standard USB Configuration Descriptor */
    USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor;
    
    /* Local interface info */
    USB_HOST_INTERFACE_DESC_INFO interfaceInfo[USB_HOST_DEVICE_INTERFACES_NUMBER];

    /* Bandwidth for each configuration */
    uint8_t load;
    
    /* Configuration number */
    uint8_t configurationNumber ;

} USB_HOST_CONFIGURATION_INFO;

// *****************************************************************************
/*  USB Host Device Information.

  Summary:
    USB Host Device Information.

  Description:
    This USB Host Layer object contains information about the attached device.

  Remarks:
    None.
*/

typedef struct  _USB_HOST_DEVICE_OBJ_
{
    /* True if this object is allocated */
    bool inUse;

    /* Control transfer object */
    USB_HOST_CONTROL_TRANSFER_OBJ controlTransferObj;

    /* Device Identifier */
    USB_HOST_DEVICE_OBJ_HANDLE deviceIdentifier;

    /* Device client handle */
    USB_HOST_DEVICE_CLIENT_HANDLE deviceClientHandle;
    
    /* Parent device information for Hub support */
    USB_HOST_DEVICE_OBJ_HANDLE parentDeviceIdentifier;

    /* Control pipe Handle */
    DRV_USB_HOST_PIPE_HANDLE controlPipeHandle;

     /* Control pipe Handle */
    DRV_USB_HOST_PIPE_HANDLE defaultPipeHandle;

    /* Device address */
    uint8_t deviceAddress;

    /* Requested alternate setting */
    uint8_t requestedAlternateSetting;

    /* HCD handle */
    DRV_HANDLE hcdHandle;

    /* Handle to the hub to which this device is connected */
    uintptr_t hubHandle;

    /* HCD function pointers */
    DRV_USB_HOST_INTERFACE * hcdInterface;

    /* Hub Function pointers */
    USB_HUB_INTERFACE * hubInterface;

    /* Address of the hub that this device is connected to */
    uint8_t hubAddress;
    
    /* Device attached  port Number */
    uint8_t devicePort;

    /* Speed of the device */
    USB_SPEED  speed;

    /* Device descriptor */
    USB_DEVICE_DESCRIPTOR deviceDescriptor;

    /* Dynamically allocated buffer for client drivers to use */
    USB_CONFIGURATION_DESCRIPTOR * holdingConfigurationDescriptor;
    
    /* Useful buffer */
    uint8_t  buffer[64];
    
    /* Number of configurations available in device */
    uint8_t nConfiguration;

    /* Number of interfaces available in this device for the set configuration.
     * */
    uint8_t  nInterfaces;

    /* setup packet */
    USB_SETUP_PACKET setupPacket;

    /* Device level driver */
    USB_HOST_CLIENT_DRIVER  * deviceClientDriver;

    /* Index of the TPL entry that matched last */
    int tplEntryTried;

    /* Index of the TPL entry that matched device level class subclass protocol
     * */
    int deviceClScPTried;

    /* Configuration Descriptor size */
    uint16_t configurationSize;

    /* Device state for state machine */
    USB_HOST_DEVICE_STATE deviceState;

    /* Enumeration Fail count */
    uint8_t enumerationFailCount;

    /* Configuration check count */
    uint8_t configurationCheckCount;

    /* Size of the configuration descriptor*/
    uint32_t configDescSize;

    /* Requested configuration number */
    uint8_t requestedConfigurationNumber;

    /* Information about the configuration*/
    USB_HOST_CONFIGURATION_INFO  configDescriptorInfo;

    /* Next device attached on the same bus*/
    struct _USB_HOST_DEVICE_OBJ_ * nextDeviceObj;

    /* Device configuration state */
    USB_HOST_DEVICE_CONFIG_STATE configurationState;

} USB_HOST_DEVICE_OBJ;

// *****************************************************************************
/*  USB Host Root Hub Information

  Summary:
    USB Host Root Hub Information.

  Description:
    This USB Host Layer object stores information about the root hub.

  Remarks:
    None.
*/

typedef struct 
{
    /* Number of ports this root hub has */
    uint8_t    ports;

    /* Speed of the bus */
    USB_SPEED  speed;

    /* The total current this root hub can supply */
    uint32_t   power;

    /* Pointer to the root hub interface */
    USB_HUB_INTERFACE rootHubPortInterface;

} USB_HOST_ROOT_HUB_INFO;

// *****************************************************************************
/*  USB Host Bus states .

  Summary:
    USB Host Bus states.

  Description:
    This enumeration lists the possible states of the bus.

  Remarks:
    This data type is local and should not be used directly by the application.
*/

typedef enum
{
    /* The default state of the bus is disabled */
    USB_HOST_BUS_STATE_DISABLED = 0x0 ,

    /* The bus is in the process of being disabled */
    USB_HOST_BUS_STATE_DISABLING = 0x1,

    /* The bus is in the process of being enabled */
    USB_HOST_BUS_STATE_ENABLING = 0x2 ,

    /* The bus is waiting for enable to complete */
    USB_HOST_BUS_STATE_WAIT_FOR_ENABLE_COMPLETE = 0x3,

    /* The bus is enabled */
    USB_HOST_BUS_STATE_ENABLED  = 0x4 ,

    /* The bus is enabled but is being suspended */
    USB_HOST_BUS_STATE_SUSPENDING = 0x5,

    /* The bus is enabled but suspended */
    USB_HOST_BUS_STATE_SUSPENDED  = 0x6

} USB_HOST_BUS_STATE;

// *****************************************************************************
/*  USB Host Bus Object

  Summary:
    USB Host Bus Object

  Description:
    This USB Host Layer object stores information about a bus.

  Remarks:
    None.
*/

typedef struct  
{
    /* Handle to the Host controller driver */
    DRV_HANDLE hcdHandle;

    /* Host Controller index */
    SYS_MODULE_INDEX hcdIndex;

    /* Bus state */
    USB_HOST_BUS_STATE state;

    /* USB Host Controller Driver function pointers */
    DRV_USB_HOST_INTERFACE * hcdInterface;

    /* Attached device list in the bus */
    USB_HOST_DEVICE_OBJ * busDeviceList;

    /* Based on bits position the device address assigned and free*/
    uint8_t addressBits[(USB_HOST_DEVICES_NUMBER/8) + 1];

    /* Only one device should enumerate at a time */
    bool deviceIsEnumerating;

    /* Total bandwidth available in a bus */
    uint32_t totalBandwidth;

    /* Available Bandwidth */
    uint32_t availableBandwidth;

    /* Root Hub information */
    USB_HOST_ROOT_HUB_INFO  rootHubInfo;

    /* Newly connected device */
    USB_HOST_DEVICE_OBJ_HANDLE  * enumeratingDeviceInfo;

    /* System timer handle */
    SYS_TMR_HANDLE  busOperationsTimerHandle;

    /* Unique identifier that get incremented for every device attach */
    uint16_t pnpIdentifier ;
    
    /* Bus Interrupt Enable Disable State */
    bool eventsStatusRestore;

    /* Flag is set when the timer expires */
    bool timerExpired;
    
    /* Identifier of the device that is enumerating */
    USB_HOST_DEVICE_OBJ_HANDLE enumeratingDeviceIdentifier;

}USB_HOST_BUS_OBJ;

// *****************************************************************************
/*  USB Host Layer Object.

  Summary:
    USB Host Layer Object.

  Description:
    This USB Host Layer Object contains information about the Host Layer.
    
  Remarks:
    None.
*/

typedef struct  
{
    /* Event Handler  */
    USB_HOST_EVENT_HANDLER hostEventHandler;

    /* Application will give this number */
    uintptr_t context;

    /* System is required the Host status */
    SYS_STATUS status;

    /* Supported Target peripheral list */
    uint8_t  nTPLEntries;

    /* Pointer to the entry of TPL*/
    USB_HOST_TPL_ENTRY * tpl;
  
    /* Control transfer Object mutex */
    OSAL_MUTEX_DECLARE(mutexControlTransferObj);
    
    /* Control transfer Object mutex */
    OSAL_MUTEX_DECLARE(mutexTransferObj);
    
    /* Pipe Object mutex */
    OSAL_MUTEX_DECLARE(mutexPipeObj);

    /* True if the host layer is presently in an interrupt context */
    volatile bool isInInterruptContext;
    
} USB_HOST_OBJ;

// ****************************************************************************
// ****************************************************************************
// USB Host Private Functions
// ****************************************************************************
// ****************************************************************************

// *****************************************************************************
/* Function:
    bool _USB_HOST_NoInterfacesOwned
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
    );

  Summary:
    This function will return true if no interface have been owned and search
    has reach end of TPL.

  Description:
    This function will return true if no interface have been owned and search
    has reach end of TPL. It will return false if atleast one interface is
    claimed or if all the interfaces are empty.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

bool _USB_HOST_NoInterfacesOwned
(
    USB_HOST_DEVICE_OBJ * deviceObj
);

// *****************************************************************************
/* Function:
    void _USB_HOST_UpdateInterfaceStatus
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
        int busIndex
    );

  Summary:
    This function will update status of the interfaces.

  Description:
    This function will update the status of the interfaces. If a interface is
    not assigned it is either assigned to the device level driver or it is
    assigned to an interface driver. The function will call the tasks routines
    of the interface driver. It checks if the all device is not owned at all
    then it will move the device to an error state.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_UpdateInterfaceStatus
(
    USB_HOST_DEVICE_OBJ * deviceObj,
    int busIndex
);

// *****************************************************************************
/* Function:
    void _USB_HOST_ReleaseInterfaceDrivers
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
    );

  Summary:
    This function will release all the loaded interface drivers.

  Description:
    This function will release all the loaded interface drivers. The matching
    driver index for each interface will be updated to indicate that the
    matching should start at the top of the TPL table.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_ReleaseInterfaceDrivers
(
    USB_HOST_DEVICE_OBJ * deviceObj
);

// *****************************************************************************
/* Function:
    void _USB_HOST_ConfigurationDescriptorParse
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
    );

  Summary:
    This function will parse the configuration descriptor contained in the
    configurationDescriptor of the configDescriptorInfo structure in deviceObj
    and will populate the interface tables. If the configuration descriptor
    contains IADs, it will then link the interfaces as defined by the IAD.

  Description:
    This function will parse the configuration descriptor contained in the
    configurationDescriptor of the configDescriptorInfo structure in deviceObj
    and will populate the interface tables. If the configuration descriptor
    contains IADs, it will then link the interfaces as defined by the IAD.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

bool _USB_HOST_ConfigurationDescriptorParse
(
    USB_HOST_DEVICE_OBJ * deviceObj
);

// *****************************************************************************
/* Function:
    int _USB_HOST_FindClassSubClassProtocolDriver
    (
        uint8_t bDeviceClass,
        uint8_t bDeviceSubClass,
        uint8_t bDeviceProtocol,
        int startPoint
    );

  Summary:
    This function will search for matching class subclass protocol driver in the
    TPL table.

  Description:
    This function will search for matching class subclass protocol driver in the
    TPL table. If a driver was not found, the function will return the last
    index of the TPL table + 1. The function will start searching from (and
    including) startPoint.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

int _USB_HOST_FindClassSubClassProtocolDriver
(
    uint8_t bDeviceClass,
    uint8_t bDeviceSubClass,
    uint8_t bDeviceProtocol,
    int startPoint
);

// *****************************************************************************
/* Function:
    void _USB_HOST_UpdateConfigurationState
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
        USB_HOST_BUS_OBJ * busObj
    );

  Summary:
    This function will update the configuration state of the device.

  Description:
    This function will check if the device configuation needs to be changed. If
    so then it gets the configuration, parses the configuration, sets up the
    interface tables and then sets the configuration.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_UpdateConfigurationState
(
    USB_HOST_DEVICE_OBJ * deviceObj,
    int busIndex
);

// *****************************************************************************
/* Function:
    void _USB_HOST_UpdateDeviceOwnership
    (
        USB_HOST_DEVICE_OBJ * deviceObj,
        USB_HOST_BUS_OBJ * busObj
    );

  Summary:
    This function will find a device level owner client driver.

  Description:
    This function will find a device level client driver owner. If a VID PID
    level driver is not found then a device level class subclass protocol driver
    needs to be found. If device was released, then a new owner needs to be
    found. If the end of the TPL table is reached, then the stop searching and
    hand over ownership of the device to the host. If a driver is attached, the
    function will call the tasks routine of this driver.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_UpdateDeviceOwnership
(
    USB_HOST_DEVICE_OBJ * deviceObj,
    int busIndex
);

// *****************************************************************************
/* Function:
    bool _USB_HOST_DeviceConfigurationDescriptorErrorCheck
    (
        USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor
    );

  Summary:
    This function checks the configuration descriptor for errors.

  Description:
    This function checks the configuration descriptor for errors. The following
    errors are checked. The sizes reported by each descriptor headers are added
    up to check if this sum is equal to total configuration descriptor size
    reported in the configuration descriptor header, The number of endpoint
    descriptors in an interface match the endpoints specified in the interface
    descriptor and the number of interfaces mentioned in the configuration
    header match the number of descriptors found in the configuration
    descriptor.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

bool _USB_HOST_DeviceConfigurationDescriptorErrorCheck
(
    USB_CONFIGURATION_DESCRIPTOR * configurationDescriptor
);

// *****************************************************************************
/* Function:
    uint8_t _USB_HOST_GetNewAddress( USB_HOST_BUS_OBJ *busObj )

  Summary:
    Searches and allocates a new device address.

  Description:
    This function searches and allocates a new device address.

  Remarks:
    This is a local function and should not be called by the application
    directly.
*/

uint8_t _USB_HOST_GetNewAddress(USB_HOST_BUS_OBJ *busObj);

// *****************************************************************************
/* Function:
    void _USB_HOST_FillSetupPacket
    (
        USB_SETUP_PACKET *setupPacket ,
        uint8_t requestType,
        uint8_t request ,
        uint16_t value,
        uint16_t index,
        uint16_t length
    )

  Summary:
    Helper function to create setup packet.

  Description:
    Helper function to create setup packet

  Remarks:
    This is a local function and should not be called by the application
    directly.
*/

void _USB_HOST_FillSetupPacket
(
    USB_SETUP_PACKET *setupPacket ,
    uint8_t requestType,
    uint8_t request ,
    uint16_t value,
    uint16_t index,
    uint16_t length
);

// *****************************************************************************
/* Function:
    void _USB_HOST_MakeDeviceReady
    ( 
        USB_HOST_DEVICE_OBJ * deviceObj, 
        USB_HOST_BUS_OBJ * busObj 
    )

  Summary:
    Maintains the state of the device at a device level.

  Description:
    Maintains the state of the device at a device level. Moves the state of the
    device from attached to ready. It opens the control transfer pipe and checks
    configuration descriptors for errors.

  Remarks:
    This is a local function and should not be called by the application
    directly.
*/

void _USB_HOST_MakeDeviceReady
(
    USB_HOST_DEVICE_OBJ * deviceObj, 
    int busIndex
);

// *****************************************************************************
/* Function:
    void _USB_HOST_UpdateDeviceTask(int busIndex)

  Summary:
    This function maintains the state of each device on the bus.

  Description:
    This function maintains the state of each device on the bus.

  Remarks:
    This is a local function and should not be called directly by the 
    application.
*/

void _USB_HOST_UpdateDeviceTask(int busIndex);

// *****************************************************************************
/* Function:
    void _USB_HOST_FreeAddress ( USB_HOST_DEVICE_OBJ_HANDLE deviceIdentifier)

  Summary:
    Frees up the address bit assigned to this device hence making the address 
    available.

  Description:
    This function frees up the address bit assigned to this device hence making 
    the address available.

  Remarks:
    This is a local function and should not be called directly by the 
    application.
*/

void _USB_HOST_FreeAddress (USB_HOST_DEVICE_OBJ_HANDLE deviceIdentifier);

// *****************************************************************************
/* Function:
    void _USB_HOST_DataTransferIRPCallback( USB_HOST_IRP * irp )
 
  Summary:
    This is the callback for IRPs submitted through the
    USB_HOST_DeviceTransfer() function.

  Description:
    This is the callback function for IRPs submitted through the
    USB_HOST_DeviceTransfer() function. The function will get the
    USB_HOST_TRANSFER_OBJ object associated with this IRP, find out the
    interface on which the transfer took place and then call the
    interfaceEventHandler function of the client driver that owns this
    interface.

  Remarks
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_DataTransferIRPCallback( USB_HOST_IRP * irp );

// *****************************************************************************
/* Function:
    USB_HOST_RESULT _USB_HOST_IRPResultToHostResult( USB_HOST_IRP * irp )
 
  Summary:
    This function maps the IRP completion result to a USB_HOST_RESULT type.

  Description:
    This function maps the IRP completion result to a USB_HOST_RESULT type.

  Remarks
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_RESULT _USB_HOST_IRPResultToHostResult(USB_HOST_IRP * irp);

// *****************************************************************************
/* Function:
    void _USB_HOST_DeviceControlTransferCallback( USB_HOST_IRP * irp )
 
  Summary:
    This is the callback for control IRPs submitted through the
    USB_HOST_DeviceControlTransfer() function.

  Description:
    This is the callback function for control IRPs submitted through the
    USB_HOST_DeviceControlTransfer() function. The function will get the
    USB_HOST_CONTROL_TRANSFER_OBJ object associated with this IRP, find out the
    type of control request call the callback function associated with the
    control transfer.

  Remarks
    This is a local function and should not be called directly by the
    application.
*/

void  _USB_HOST_DeviceControlTransferCallback( USB_HOST_IRP * irp );

// *****************************************************************************
/* Function:
    void * _USB_HOST_FindEndOfDescriptor(void * descriptor) 

  Summary:
    Function finds the end of descritor marker and returns the pointer to where
    the marker has started.

  Description:
    Function finds the end of descritor marker and returns the pointer to where
    the marker has started.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void * _USB_HOST_FindEndOfDescriptor(void * descriptor);

// *****************************************************************************
/* Function:
    void * _USB_HOST_TimerCallback
    (
       uint32_t context,
       uint32_t currtick
    )

  Summary:
    Function is called when the SYS_TMR_CallbackSingle expires.

  Description:
    Function is called when the SYS_TMR_CallbackSingle expires.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/    

void _USB_HOST_TimerCallback(uintptr_t context, uint32_t currtick);
#endif
