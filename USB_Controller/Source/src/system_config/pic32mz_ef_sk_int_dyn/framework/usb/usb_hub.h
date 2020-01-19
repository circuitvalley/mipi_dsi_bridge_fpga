/*******************************************************************************
  USB HOST HUB driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    usb_hub.h

  Summary:
    USB HUB Layer Interface Header

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface to the USB HOST HUB
    driver
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

#ifndef _USB_HOST_HUB_H
#define _USB_HOST_HUB_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system/common/sys_common.h"
#include "driver/driver_common.h"
#include "usb/usb_host.h"
#include "usb/usb_host_client_driver.h"

// *****************************************************************************
// *****************************************************************************
// Section: USB HOST HUB CLASS DRIVER Constants
// *****************************************************************************
// *****************************************************************************

/* HUB Class codes as defined by USB */
#define USB_HUB_CLASS_CODE  0x09

/* bDescriptorType for Hub Descriptors */
#define USB_HUB_DESCRIPTOR_TYPE 0x29     

// *****************************************************************************
/* USB Hub Port Indicator Colors

  Summary:
    Enumerates the color of the Port Indicators.

  Description:
    This enumeration lists the possible Hub Port Indicator Colors.

  Remarks:
    None.
*/

typedef enum
{
    /* Port Indicator Color Green */ 
    USB_HUB_PORT_INDICATOR_COLOR_GREEN,

    /* Port Indicator Color Amber */
    USB_HUB_PORT_INDICATOR_COLOR_AMBER

} USB_HUB_PORT_INDICATOR_COLOR;

// *****************************************************************************
/* USB Hub Port Indicator State

  Summary:
    Enumerates the state of the Port Indicators.

  Description:
    This enumeration lists the possible Hub Port Indicator states.

  Remarks:
    None.
*/

typedef enum
{
    /* Port Indicator should be off */
    USB_HUB_PORT_INDICATOR_STATE_OFF,

    /* Port Indicator should be on */
    USB_HUB_PORT_INDICATOR_STATE_ON,

    /* Port Indicator should be blinking */
    USB_HUB_PORT_INDICATOR_STATE_BLINKING,

} USB_HUB_PORT_INDICATOR_STATE;

// *****************************************************************************
/* USB Hub Port Status

  Summary:
    This is the Hub port status.

  Description:
    This data type defines the Hub Port Status. This is the type of data that is
    received from the Hub in reponse to a Hub Port Status Control Request.

  Remarks:
    None.
*/

typedef struct __attribute__ ((packed))
{
    union 
    {
         struct __attribute__ ((packed))
         {
            unsigned currentConnectStatus :1 ; 
            unsigned portEnabledDisabled :1 ;
            unsigned suspend :1 ;
            unsigned overCurrent :1 ;
            unsigned reset :1 ;
            unsigned reserved :3 ;
            unsigned portPower :1 ;
            unsigned lowSpeedDeviceAttached :1 ;
            unsigned highSpeedDeviceAttached :1 ;
            unsigned portTestMode :1 ;
            unsigned portIndicatorControl :1 ;
            unsigned reserved1:3;
         };
         uint16_t    wPortStatus;
    };
    
    union 
    {
         struct __attribute__ ((packed))
         {
            unsigned connectStatusChange:1 ; 
            unsigned portEnableDisableChange:1 ;
            unsigned suspendChange :1 ;
            unsigned overCurrentIndicatorChange:1 ;
            unsigned resetChange:1 ;
      
         };
         uint16_t    wPortChange;
    };
      
} USB_HOST_PORT_STATUS;

// *****************************************************************************
/* USB Hub Status

  Summary:
    This is the Hub status data type.

  Description:
    This data type defines the Hub Status. This is the type of data that is
    received from the Hub in reponse to a Hub Status Control Request.

  Remarks:
    None.
*/

typedef struct __attribute__ ((packed))
{
    union 
    {
         struct __attribute__ ((packed))
         {
            unsigned localPowerSource :1 ; 
            unsigned overCurrent :1 ;
         };
         uint16_t    wHubStatus;
    };
    
    union 
    {
         struct __attribute__ ((packed))
         {
             unsigned localPowerStatusChange:1 ; 
             unsigned overCurrentChange:1 ;
         };
         uint16_t    wHubChange;
    };

} USB_HUB_STATUS;

// *****************************************************************************
/* USB Hub Class Request Codes

  Summary:
    This is an enumeration of different Hub Class Request Codes.

  Description:
    This enumeration defines the possible Hub Class Request Codes. This is
    defined in table 11-16 of the USB 2.0 specification.

  Remarks:
    None.
*/

typedef enum
{
    USB_HUB_CLASS_REQUEST_GET_STATUS     = 0x0,
    USB_HUB_CLASS_REQUEST_CLEAR_FEATURE  = 0x1,
    USB_HUB_CLASS_REQUEST_SET_FEATURE    = 0x3,
    USB_HUB_CLASS_REQUEST_GET_DESCRIPTOR = 0x6,
    USB_HUB_CLASS_REQUEST_SET_DESCRIPTOR = 0x7,
    USB_HUB_CLASS_REQUEST_CLEAR_TT_BUFFER = 0x8

} USB_HOST_CLASS_REQUEST;

// *****************************************************************************
/* USB Hub Class Feature Selectors

  Summary:
    This is an enumeration of different Hub Class Feature Selectors.

  Description:
    This enumeration defines the possible Hub Class Feature Selectors. This is
    defined in table 11-17 of the USB 2.0 specification.

  Remarks:
    None.
*/

typedef enum
{
    USB_HUB_CLASS_FEATURE_C_HUB_LOCAL_POWER = 0,
    USB_HUB_CLASS_FEATURE_C_HUB_OVER_CURRENT = 1,
    USB_HUB_CLASS_FEATURE_PORT_CONNECTION = 0,
    USB_HUB_CLASS_FEATURE_PORT_ENABLE = 1,
    USB_HUB_CLASS_FEATURE_PORT_SUSPEND = 2,
    USB_HUB_CLASS_FEATURE_PORT_OVER_CURRENT = 3,
    USB_HUB_CLASS_FEATURE_PORT_RESET = 4,
    USB_HUB_CLASS_FEATURE_PORT_POWER = 8,
    USB_HUB_CLASS_FEATURE_PORT_LOW_SPEED = 9,
    USB_HUB_CLASS_FEATURE_C_PORT_CONNECTION = 16,
    USB_HUB_CLASS_FEATURE_C_PORT_ENABLE = 17,
    USB_HUB_CLASS_FEATURE_C_PORT_SUSPEND = 18,
    USB_HUB_CLASS_FEATURE_C_PORT_OVER_CURRENT = 19,
    USB_HUB_CLASS_FEATURE_C_PORT_RESET = 20,
    USB_HUB_CLASS_FEATURE_PORT_TEST = 21,
    USB_HUB_CLASS_FEATURE_PORT_INDICATOR = 22

} USB_HUB_CLASS_FEATURE;
 
//*****************************************************************************
/* USB Host Hub Descriptor

  Summary:
    Defines the USB Hub Descriptor.

  Description:
    This structure defines USB Hub Descriptor. This as per Table 11-13 of the
    USB 2.0 specification.

  Remarks:
    None.
*/

typedef struct __attribute__((packed)) 
{
    uint8_t  bDescLength;        
    uint8_t  bDescriptorType;    
    uint8_t  bNbrPorts;          
    uint16_t wHubCharacteristics;    
    uint8_t  bPwrOn2PwrGood;     
    uint8_t  bHubContrCurrent;   
    uint8_t  DeviceRemovable;    
    uint8_t  PortPwrCtrlMask;    
    
} USB_HUB_DESCRIPTOR;

#endif

