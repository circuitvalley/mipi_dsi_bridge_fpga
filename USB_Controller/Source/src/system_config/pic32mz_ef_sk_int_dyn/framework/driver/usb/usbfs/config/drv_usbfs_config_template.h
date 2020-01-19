/*******************************************************************************
  USB Full Speed Driver Configuration Template.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbfs_config_template.h

  Summary:
    USB Full Speed (USBFS) Driver Configuration Template.

  Description:
    This file lists all the configurations constants that affect the operation
    of the USBFS Driver. 
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_USBFS_CONFIG_TEMPLATE_H
#define _DRV_USBFS_CONFIG_TEMPLATE_H

#error "This is a configuration template file.  Do not include it directly."

// *****************************************************************************
/* USB Full Speed Driver Device Mode Support. 

  Summary
    Determines if the USB Device Functionality should be enabled.

  Description
    This constant should be set to true if USB device support is required in the
    application. It should be set to false if device support is not required.

  Remarks:
    This constant should always be defined. 
*/

#define DRV_USBFS_DEVICE_SUPPORT      true

// *****************************************************************************
/* USB Full Speed Driver Host Mode Support. 

  Summary
    Determines if the USB Host Functionality should be enabled.

  Description
    This constant should be set to true if USB Host mode support is required in
    the application. It should be set to false if host support is not
    required.

  Remarks:
    This constant should always be defined. 
*/

#define DRV_USBFS_HOST_SUPPORT      false

// *****************************************************************************
/* USB Full Speed Driver Instances Number. 

  Summary
    Specifies the number of driver instances to be enabled in the application.

  Description
    This constant defines the number of driver instances to be enabled in the
    application. This will be typically be the number of USB controllers to be
    used in the application. On PIC32MX microcontrollers that have one USB
    controller, this value will always be 1. On PIC32MX microcontrollers which
    have 2 USB controllers, this value could 1 or 2, depending on whether 1 or 2
    USB segments are required. To conserve data memory, this constant should be
    set to exactly the number of USB controller that are required in the system.

  Remarks:
    This constant should always be defined. 
*/

#define DRV_USBFS_INSTANCES_NUMBER   1

// *****************************************************************************
/* USB Full Speed Driver Interrupt Mode. 

  Summary
    Configures the driver for interrupt or polling mode operation.

  Description:
    This constant configures the driver for interrupt or polling operation. If
    this flag is set to true, the driver will operate in interrupt mode. If the
    flag is set to false, the driver will operate in polled mode. In polled, the
    driver interrupt state machine gets updated in the SYS_Tasks(). If the
    driver is configured interrupt mode, the driver interrupt state machine gets
    updated in the driver interrupt service routine. It is always recommended
    for the driver to operate in interrupt mode.  

  Remarks:
    This constant should always be defined. 
*/

#define DRV_USBFS_INTERRUPT_MODE  true

// *****************************************************************************
/* USB Full Speed Driver Endpoint Numbers. 

  Summary
    Configures the number of endpoints to be provisioned in the driver.

  Description:
    This constant configures the number of endpoints that the driver needs to
    manage. When DRV_USBFS_DEVICE_SUPPORT is enabled, this constant should be
    set to the total number of endpoints to be enabled in the device. When
    enabled, a endpoint can be used for communication. Using any direction of an
    endpoint will require that entire endpoint to be enabled.   

    Consider the case of a composite USB Device that containing a CDC and MSD
    function. The CDC function will require 1 Bulk endpoint (OUT and IN
    directions) and 1 Interrupt endpoint (IN direction). The MSD function will
    require 1 Bulk endpoint (IN and OUT directions). This design can be
    implemented by using 4 endpoints. Endpoint 0 is used for the mandatory
    control interface. Endpoint 1 is used for CDC Bulk interface. Endpoint 2 is
    used for CDC interrupt interface and endpoint 3 is used for MSD Bulk
    Interface. The constant should then be set to 4.   

    For Host mode operation, this constant should be set to 1. Setting this to
    greater than 1 will result in unused data memory allocation.

  Remarks:
    This constant should always be defined. 
*/

#define DRV_USBFS_ENDPOINTS_NUMBER    3

// *****************************************************************************
/* USB Full Speed Driver Host Mode Control Transfers NAK Limit. 

  Summary
    Configures the NAK Limit for Host Mode Control Transfers.

  Description:
    This constant configures the number of NAKs that the driver can accept from
    the device in the data stage of a control transfer before aborting the
    control transfer with a USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT. Setting this
    constant to 0 will disable NAK limit checking. This constant should be
    adjusted to enable USB host compatibility with USB Devices which require more
    time to process control transfers.

  Remarks:
    This constant should always be defined when DRV_USBFS_HOST_SUPPORT is set to
    true. 
*/

#define DRV_USBFS_HOST_NAK_LIMIT      2000

// *****************************************************************************
/* USB Full Speed Driver Host Mode Pipes Number. 

  Summary
    Configures the maximum number of pipes that are can be opened when the
    driver is operating in Host mode. 

  Description:
    This constant configures the maximum number of pipes that can be opened when
    the driver is operating in Host mode. Calling the DRV_USBFS_HOST_PipeSetup
    function will cause a pipe to be opened. Calling this function when
    DRV_USBFS_HOST_PIPES_NUMBER number of pipes have already been opened will
    cause the function to return an Invalid Pipe Handle. This constant should be
    configured to account for the maximum number of devices and the device types
    to be supported by the host application. 
    
    For example if the USB Host application must support 2 USB Mass Storage
    devices and 1 CDC device, it must set this constant 9 ( 4 bulk pipes for 2
    Mass Storage devices + 2 bulk pipes and 1 interrupt pipe for 1 CDC device
    and 2 control pipes for 2 devices). Allocating pipes consumes data memory. 

  Remarks:
    This constant should always be defined when DRV_USBFS_HOST_SUPPORT is set to
    true. 
*/

#define DRV_USBFS_HOST_PIPES_NUMBER    10

// *****************************************************************************
/* USB Full Speed Driver Host Mode Attach Debounce Duration. 

  Summary
    Configures the time duration (in milliseconds) that the driver will wait to
    re-confirm a device attach. 

  Description:
    This constant configures the time duration (in milliseconds) that driver
    will wait to re-confirm a device attach. When the driver first detects
    device attach, it start, it will start a timer for the duration specified by
    the constant. When the timer expires, the driver will check if the device is
    still attached. If so, the driver will then signal attach to the host stack.
    The duration allows for device attach to become electro-mechanically stable. 

  Remarks:
    This constant should always be defined when DRV_USBFS_HOST_SUPPORT is set to
    true. 
*/

#define DRV_USBFS_HOST_ATTACH_DEBOUNCE_DURATION 500

// *****************************************************************************
/* USB Full Speed Driver Host Mode Reset Duration.

  Summary
    Configures the time duration (in milliseconds) of the Reset Signal.

  Description:
    This constant configures the duration of the reset signal. The driver
    generates reset signal when the USB Host stack requests for root hub port
    reset. The driver will generate the reset signal for the duration specified
    by this constant and will then stop generating the reset signal. 

  Remarks:
    This constant should always be defined when DRV_USBFS_HOST_SUPPORT is set to
    true. 
*/

#define DRV_USBFS_HOST_RESET_DURATION 100


#
