/*******************************************************************************
  USB Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_usb.h

  Summary:
    USB Peripheral Library Interface Header for common definitions

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the USB
    Peripheral Library.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2013 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _PLIB_USB_H
#define _PLIB_USB_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued at end of file)
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.  However,
    please see the bottom of the file for additional implementation header files
    that are also included
*/

#include <stdint.h>
#include <stdbool.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Maximum number of endpoints

  Summary:
    Maximum number of endpoints supported (not including EP0).

  Description:
    This constant defines the maximum number of endpoints supported (not
    including EP0).  It is used in dimensioning the Buffer Descriptor Table (BDT)
    array.
*/

#define USB_MAX_EP_NUMBER 15


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Operating Modes Enumeration

  Summary:
    Provides enumeration of operating modes supported by USB.

  Description:
    This data type provides enumeration of the operating modes supported by the 
    USB module.

  Remarks:
    None.
*/

typedef enum
{
    // None
    USB_OPMODE_NONE   /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/ ,
    // Device
    USB_OPMODE_DEVICE /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/ ,
    // Host
    USB_OPMODE_HOST   /*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/ ,
    // OTG
    USB_OPMODE_OTG    /*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/

} USB_OPMODES;


// *****************************************************************************
/* USB OTG Interrupts Enumeration

  Summary:
    Provides enumeration of interrupts related to the USB On-The-Go (OTG) module.

  Description:
    This data type provides enumeration of interrupts related to the USB OTG module.

  Remarks:
    Not applicable if the USB OTG module is not enabled.
*/

typedef enum
{
    // State of (VBUS > Va_vbus_vld) on the A device has changed
    USB_OTG_INT_ADEVICE_VBUS_VALID   /*DOM-IGNORE-BEGIN*/ =  1<<0 /*DOM-IGNORE-END*/ ,
    // Reserved. Don't use.
    USB_OTG_INT_OTG_RESERVED         /*DOM-IGNORE-BEGIN*/ =  1<<1 /*DOM-IGNORE-END*/ ,
    // State of (VBUS < Vb_sess_end) on the B device has changed
    USB_OTG_INT_BDEVICE_SESSION_END  /*DOM-IGNORE-BEGIN*/ =  1<<2 /*DOM-IGNORE-END*/ ,
    // State of (VBUS > Va_sess_vld) on the A or B devices has changed
    USB_OTG_INT_SESSION_VALID        /*DOM-IGNORE-BEGIN*/ =  1<<3 /*DOM-IGNORE-END*/ ,
    // Activity detected on the D+, D-, ID, or VBUS lines
    USB_OTG_INT_ACTIVITY_DETECT      /*DOM-IGNORE-BEGIN*/ =  1<<4 /*DOM-IGNORE-END*/ ,
    // USB line state has been stable for 1 ms, but different from last time
    USB_OTG_INT_STABLE_LINE_STATE    /*DOM-IGNORE-BEGIN*/ =  1<<5 /*DOM-IGNORE-END*/ ,
    // One millisecond timer has expired
    USB_OTG_INT_ONE_MS_TIMEOUT       /*DOM-IGNORE-BEGIN*/ =  1<<6 /*DOM-IGNORE-END*/ ,
    // Change in state of ID pin detected.
    USB_OTG_INT_ID_STATE_CHANGE      /*DOM-IGNORE-BEGIN*/ =  1<<7 /*DOM-IGNORE-END*/ ,
    // All or Any of the above
    USB_OTG_INT_ANY                  /*DOM-IGNORE-BEGIN*/ =  0xFD /*DOM-IGNORE-END*/ ,
    // All or Any of the above
    USB_OTG_INT_ALL                  /*DOM-IGNORE-BEGIN*/ =  0xFD /*DOM-IGNORE-END*/

} USB_OTG_INTERRUPTS;


// *****************************************************************************
/* Enumeration of USB Ping-Pong Modes

  Summary:
    Supports the four modes of ping-pong buffering.

  Description:
    This data type supports the four modes of ping-pong buffering.

  Remarks:
    None.
*/

typedef enum
{
    // Ping-Pong buffering on all endpoints except Endpoint Zero
    USB_PING_PONG_ALL_BUT_EP0    /*DOM-IGNORE-BEGIN*/ = 0x03 /*DOM-IGNORE-END*/ , 
    // Ping-Pong buffering on all endpoints
    USB_PING_PONG_FULL_PING_PONG /*DOM-IGNORE-BEGIN*/ = 0x02 /*DOM-IGNORE-END*/ , 
    // Ping-Pong buffering on just Endpoint Zero transmit
    USB_PING_PONG_EP0_OUT_ONLY   /*DOM-IGNORE-BEGIN*/ = 0x01 /*DOM-IGNORE-END*/ , 
    // No ping-pong buffering
    USB_PING_PONG_NO_PING_PONG   /*DOM-IGNORE-BEGIN*/ = 0x00 /*DOM-IGNORE-END*/   

} USB_PING_PONG_MODE;


// *****************************************************************************
/* Enumeration of USB Ping-Pong Indicator

  Summary:
    Decodes which buffer (Even vs. Odd) was used for the last transaction.

  Description:
    This data type decodes which buffer (Even vs. Odd) was used for the last 
    transaction.

  Remarks:
    None.
*/

typedef enum
{
    // Last transaction on Even Buffer
    USB_PING_PONG_EVEN /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/ ,
    // Last transaction on Odd  Buffer
    USB_PING_PONG_ODD  /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/   

} USB_PING_PONG_STATE;


// *****************************************************************************
/* Enumeration of USB Buffer Ping-Pong

  Summary:
   Enumerates the ping-pong buffer (Even vs. Odd).

  Description:
   This data type enumerates the ping-pong buffer (Even vs. Odd).

  Remarks:
    None.
*/

typedef enum
{
    USB_BUFFER_EVEN /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/ , // Even Buffer
    USB_BUFFER_ODD  /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/   // Odd  Buffer

} USB_BUFFER_PING_PONG;


// *****************************************************************************
/* Enumeration of Legal Packet IDs (PIDs)

  Summary:
    Legal PID values.

  Description:
    This data type enumerates the valid (i.e., legal) PID values.  While the PID
    field is four bits long, only these values are legal and should be used.  
    The use of any other values may cause unpredictable results.

*/

typedef enum
{
    USB_PID_SETUP /*DOM-IGNORE-BEGIN*/ = 0x0D /*DOM-IGNORE-END*/ , // Setup token
    USB_PID_IN    /*DOM-IGNORE-BEGIN*/ = 0x09 /*DOM-IGNORE-END*/ , // IN token
    USB_PID_OUT   /*DOM-IGNORE-BEGIN*/ = 0x01 /*DOM-IGNORE-END*/   // OUT token

} USB_PID;


// *****************************************************************************
/* Enumeration of Pull-Up and Pull-Down Resistors for OTG

  Summary:
    USB OTG pull-Up and pull-Down resistors for D+ and D- .

  Description:
    This data type enumerates the OTG Pull-Up and Pull-Down resistors for D+ and D- .

  Remarks:
    None.
*/

typedef enum
{
    USB_OTG_DPLUS_PULLUP   /*DOM-IGNORE-BEGIN*/ = 1<<7 /*DOM-IGNORE-END*/ , // D+ Pull-Up
    USB_OTG_DMINUS_PULLUP  /*DOM-IGNORE-BEGIN*/ = 1<<6 /*DOM-IGNORE-END*/ , // D- Pull-Up
    USB_OTG_DPLUS_PULLDN   /*DOM-IGNORE-BEGIN*/ = 1<<5 /*DOM-IGNORE-END*/ , // D+ Pull-Down
    USB_OTG_DMINUS_PULLDN  /*DOM-IGNORE-BEGIN*/ = 1<<4 /*DOM-IGNORE-END*/   // D- Pull-Down

} USB_OTG_PULL_UP_PULL_DOWN;


// *****************************************************************************
/* USB Token Speeds Enumeration

  Summary:
    Provides enumeration of available token speeds.

  Description:
    This data type provides enumeration of available token speeds.

  Remarks:
    For Host mode only.
*/

typedef enum
{
    USB_LOWSPEED_TOKENS, // Low Speed Tokens
    USB_FULLSPEED_TOKENS // Full Speed Tokens

} USB_TOKEN_SPEED;


// *****************************************************************************
/* Enumeration of USB Endpoint Transmit/Receive Setup

  Summary:
    Provides enumeration transmit/receive setup for an endpoint.

  Description:
    This data type provides enumeration transmit/receive setup for an endpoint.

  Remarks:
    None.
*/

typedef enum
{
    USB_EP_NOTXRX,  // Nothing              enabled for endpoint
    USB_EP_RX,      // Receive              enabled for endpoint
    USB_EP_TX,      // Transmit             enabled for endpoint
    USB_EP_TX_RX    // Transmit and Receive enabled for endpoint

} USB_EP_TXRX;


// *****************************************************************************
/* USB Endpoint Buffer Direction Enumeration

  Summary:
    Provides enumeration transmit/receive direction for a buffer.

  Description:
    This data type provides enumeration transmit/receive direction for a buffer.

  Remarks:
    None.
*/

typedef enum
{
    USB_BUFFER_RX  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/, // Receive
    USB_BUFFER_TX  /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/  // Transmit

} USB_BUFFER_DIRECTION;


// *****************************************************************************
/* USB Endpoint Buffer Data Toggle Enumeration

  Summary:
    Provides enumeration data toggle for a buffer.

  Description:
    This data type provides enumeration data toggle for a buffer.

  Remarks:
    None.
*/

typedef enum
{
    USB_BUFFER_DATA0, // DATA0/1 = 0
    USB_BUFFER_DATA1  // DATA0/1 = 1

} USB_BUFFER_DATA01;


// *****************************************************************************
/* USB Endpoint Buffer Data Toggle Enumeration for Buffer Schedulint

  Summary:
    Provides enumeration data toggle for a buffer.

  Description:
    This data type provides enumeration data toggle for a buffer.

  Remarks:
    None.
*/

typedef enum
{
    USB_BUFFER_DONTCHANGE  /*DOM-IGNORE-BEGIN*/ = -1 /*DOM-IGNORE-END*/, // Don't Change DATA0/1
    USB_BUFFER_SET_DATA0   /*DOM-IGNORE-BEGIN*/ =  0 /*DOM-IGNORE-END*/, // DATA0/1 = 0
    USB_BUFFER_SET_DATA1   /*DOM-IGNORE-BEGIN*/ =  1 /*DOM-IGNORE-END*/  // DATA0/1 = 1

} USB_BUFFER_SCHEDULE_DATA01;


// ****************************************************************************
// ****************************************************************************
// Section: Processor Include Files
// ****************************************************************************
// ****************************************************************************

/*DOM-IGNORE-BEGIN*/

#include "peripheral/usb/processor/usb_processor.h"

/*DOM-IGNORE-END*/


// *****************************************************************************
// *****************************************************************************
// Section: Helper Macros
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: USB Setup
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    uint8_t PLIB_USB_DeviceAddressGet ( USB_MODULE_ID index )

  Summary:
    Returns the address of the USB module in Device mode.

  Description:
    This function returns the address of the USB module in Device mode.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    Device Address

  Example:
  <code>
    myUSBAddress = PLIB_USB_DeviceAddressGet(MY_USB_INSTANCE);
  </code>

  Remarks:
    Note: This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsDeviceAddressin your 
	application to determine whether this feature is available.
*/

uint8_t PLIB_USB_DeviceAddressGet( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_DeviceAddressSet ( USB_MODULE_ID index, uint8_t address )

  Summary:
    Sets the USB Device's address.

  Description:
    This function sets the USB Device's address as part of enumeration.

  Precondition:
    USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest
    address        - USB address

  Returns:
    None.

  Example:
  <code>
    uint8_t myUSBAddress = ....;
    PLIB_USB_DeviceAddressSet(MY_USB_INSTANCE, myUSBAddress);
  </code>

  Remarks:
    Note: This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsDeviceAddressin your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_DeviceAddressSet ( USB_MODULE_ID index, uint8_t address );


// *****************************************************************************
/* Function:
    void PLIB_USB_Disable ( USB_MODULE_ID index )

  Summary:
    Disables (powers down) the USB module.

  Description:
    This function disables (powers down) the USB module.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
#if defined(__PIC32MX__)
    // Disable Host, Device, or OTG before powering down
    PLIB_USB_OperatingModeSelect( MY_USB_INSTANCE, USB_OPMODE_NONE );
    // Turn off USB
    PLIB_USB_Disable(MY_USB_INSTANCE);
    // For PIC32, wait until module is no longer busy before trying to
    // access any USB module registers.
    while ( PLIB_USB_ModuleIsBusy (MY_USB_INSTANCE) )
    {
        //wait
    }
#endif
    // Can now read or modify USB module status
  </code>

  Remarks:
    For PIC32 devices, the USB module must be in Device mode
    before the USB module is powered down.

    For PIC32 devices, all reads or writes to module registers after powering down
    the module will be invalid until PLIB_USB_ModuleIsBusy (MY_USB_INSTANCE) == false.
	
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsModulePower in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_Disable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
      void PLIB_USB_Enable ( USB_MODULE_ID index )

  Summary:
    Enables (powers up) the USB module.

  Description:
    This function enables (powers up) the USB module.

  Conditions:
    None.

  Input:
    index -  Identifier for the device instance of interest

  Return:
    None.

  Example:
    <code>
      // Complete Needed setup for the module
      PLIB_USB_Enable(MY_USB_INSTANCE);
    </code>

  Remarks:
    See also PLIB_USB_ModuleIsBusy.
	
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsModulePoweryour 
	application to determine whether this feature is available.	
*/

void PLIB_USB_Enable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_FullSpeedDisable ( USB_MODULE_ID index )

  Summary:
    Forces the USB module to operate at low speed.

  Description:
    This function forces the USB module to operate at low speed.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_FullSpeedDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    For PIC32 devices: Host mode only.
	
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsSpeedControl in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_FullSpeedDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_FullSpeedEnable ( USB_MODULE_ID index )

  Summary:
    Enables the USB to operate at full speed.

  Description:
    This function enables the USB to operate at full speed.

  Precondition:
    Use only before the USB module is enabled by calling PLIB_USB_Enable.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_FullSpeedEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    For PIC32 devices: Host mode only.
	
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsSpeedControl in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_FullSpeedEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    bool PLIB_USB_ModuleIsBusy ( USB_MODULE_ID index )

  Summary:
    Indicates if the USB module is not ready to be enabled.

  Description:
    This function indicates if the USB module is not ready to be enabled.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    - true  - USB module is active or disabled, but not ready to be enabled
    - false - USB module is not active and is ready to be enabled

  Example:
  <code>
#ifdef (__PIC32MX__)
    while ( PLIB_USB_ModuleIsBusy(MY_USB_INSTANCE) )
    {
        // wait
    }
#endif
    PLIB_USB_Disable(MY_USB_INSTANCE);
  </code>

  Remarks:
    If PLIB_USB_ModuleIsBusy(MY_USB_INSTANCE) == true and the USB module is
    disabled, and all status returned for the module, all enables/disables for
    the module will produce undefined results.
	
	This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsModuleBusyin your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_ModuleIsBusy ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OnChipPullUpDisable ( USB_MODULE_ID index )

  Summary:
    Disables on-chip pull-ups.

  Description:
    This function disables on-chip pull-ups.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OnChipPullUpDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOnChipPullup in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_OnChipPullUpDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OnChipPullUpEnable ( USB_MODULE_ID index )

  Summary:
    Enables on-chip pull-ups.

  Description:
    This function enables on-chip pull-ups. Pull-up on D+ in Full-Speed mode.
    Pull-up on D- in Low-Speed mode.

  Precondition:
    Use only before the USB module is enabled by calling PLIB_USB_Enable.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OnChipPullUpEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOnChipPullup in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_OnChipPullUpEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OperatingModeSelect( USB_MODULE_ID index, USB_OPMODES opMode )

  Summary:
    Selects the operating mode of the USB module.

  Description:
    This function selects the operating mode of the USB module, either Host, 
    Device, or OTG.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    opMode    - Selected operating mode: USB_OPMODE_DEVICE, USB_OPMODE_HOST,
                or USB_OPMODE_OTG

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OperatingModeSelect( MY_USB_INSTANCE, USB_OPMODE_DEVICE );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOpModeSelect in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_OperatingModeSelect( USB_MODULE_ID index, USB_OPMODES opMode );


// *****************************************************************************
/* Function:
    void PLIB_USB_PingPongFreeze ( USB_MODULE_ID index )

  Summary:
    Resets all Ping-Pong buffer pointers to even buffers.

  Description:
    This function resets all Ping-Pong buffer pointers to even buffers.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    // Reset all ping-pong buffers to "Even"
    PLIB_USB_PingPongFreeze(MY_USB_INSTANCE);
    PLIB_USB_PingPongUnfreeze(MY_USB_INSTANCE);
  </code>

  Remarks:
    Buffers remain "frozen" at "Even" until they are unfrozen using
    PLIB_USB_PingPongUnfreeze.
	
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBufferFreeze in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_PingPongFreeze ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_PingPongUnfreeze ( USB_MODULE_ID index )

  Summary:
    Enables Ping-Pong buffering.

  Description:
    This function enables Ping-Pong buffering.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    // Reset all Ping-Pong buffers to "Even"
    PLIB_USB_PingPongFreeze(MY_USB_INSTANCE);
    PLIB_USB_PingPongUnfreeze(MY_USB_INSTANCE);
  </code>

  Remarks:
    See PLIB_USB_PingPongFreeze.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBufferFreeze in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_PingPongUnfreeze ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    USB_PING_PONG_MODE PLIB_USB_PingPongModeGet ( USB_MODULE_ID index )

  Summary:
    Returns the Ping-Pong Configuration setting.

  Description:
    This function returns the Ping-Pong Configuration setting.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    Ping-Pong Mode - One of USB_PING_PONG__ALL_BUT_EP0,  USB_PING_PONG__FULL_PING_PONG,
                            USB_PING_PONG__EP0_OUT_ONLY, USB_PING_PONG__NO_PING_PONG

  Example:
  <code>
    ppConfig = PLIB_USB_PingPongModeGet(MY_USB_INSTANCE);
  </code>

  Remarks:
    None.
	
	This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsPingPongMode in your 
	application to determine whether this feature is available.	
*/

USB_PING_PONG_MODE PLIB_USB_PingPongModeGet ( USB_MODULE_ID index ) ;


// *****************************************************************************
/* Function:
    void PLIB_USB_PingPongModeSelect ( USB_MODULE_ID index, USB_PING_PONG_MODE ppConfig)

  Summary:
    Selects the Ping-Pong Configuration setting.

  Description:
    This function selects the Ping-Pong Configuration setting.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    ppConfig       - Ping-Pong configuration selection. One of
                     USB_PING_PONG__ALL_BUT_EP0,  USB_PING_PONG__FULL_PING_PONG,
                     USB_PING_PONG__EP0_OUT_ONLY, USB_PING_PONG__NO_PING_PONG

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_PingPongModeSelect(MY_USB_INSTANCE,USB_PING_PONG__ALL_BUT_EP0);
  </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsPingPongMode in your 
	application to determine whether this feature is available.		

*/

void PLIB_USB_PingPongModeSelect ( USB_MODULE_ID index, USB_PING_PONG_MODE ppConfig );


// *****************************************************************************
/* Function:
    void PLIB_USB_SleepGuardDisable ( USB_MODULE_ID index )

  Summary.
    Disables Sleep Guard. Entry into Sleep mode is immediate.

  Description:
    This function disables Sleep Guard. Entry into Sleep mode is immediate.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_SleepGuardDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Not available on all PIC32 devices.  Refer to the specific device data sheet 
    for details.
	
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsSleepEntryGuard in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_SleepGuardDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_SleepGuardEnable ( USB_MODULE_ID index )

  Summary:
    Entry into Sleep mode is blocked if bus activity is detected or if an 
    interrupt is pending.

  Description:
    This function block entry into Sleep mode if bus activity is detected or if 
    an interrupt is pending.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_SleepGuardEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Not available on all PIC32 devices.  Refer to the specific device data sheet 
    for details.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsSleepEntryGuard in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_SleepGuardEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_StopInIdleDisable ( USB_MODULE_ID index )

  Summary:
    Allows the USB module to continue operation when the device enters Idle mode.

  Description:
    This function allows the USB module to continue operation when the device 
    enters Idle mode.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_StopInIdleDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsStopInIdle in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_StopInIdleDisable ( USB_MODULE_ID index );


// *****************************************************************************
/*  Function:
    void PLIB_USB_StopInIdleEnable ( USB_MODULE_ID index )

  Summary:
    Enables USB module operation to stop when the device enters Idle mode.

  Description:
    This function enables USB module operation to stop when the device enters 
    Idle mode.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_StopInIdleEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsStopInIdle in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_StopInIdleEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_SuspendDisable ( USB_MODULE_ID index )

  Summary:
    Disables USB OTG Suspend mode.

  Description:
    This function disables USB OTG Suspend mode.  The USB OTG module will operate 
    normally.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_SuspendDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsSuspend in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_SuspendDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_SuspendEnable ( USB_MODULE_ID index )

  Summary:
    Enables USB Suspend mode.

  Description:
    This function enables USB Suspend mode. The 48 MHz USB clock will be gated off. 
    The transceiver is placed in a low-power state.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_SuspendEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsSuspend in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_SuspendEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_AutoSuspendDisable ( USB_MODULE_ID index )

  Summary:
    Disables USB OTG Auto-suspend mode.

  Description:
    This function disables USB OTG Auto-suspend mode. The USB OTG module will
    operate normally and does not automatically suspend upon entry to Sleep mode. 
    Software must use PLIB_USB_SuspendEnable to suspend the module, including the 
    USB 48 MHz clock

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_AutoSuspendDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsAutomaticSuspend in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_AutoSuspendDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_AutoSuspendEnable ( USB_MODULE_ID index )

  Summary:
    Enables USB Auto-suspend mode.

  Description:
    This function enables USB Auto-suspend mode. The USB module automatically 
    suspends upon entry to Sleep mode.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_AutoSuspendEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsAutomaticSuspend in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_AutoSuspendEnable ( USB_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: Buffer Descriptor Table
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void * PLIB_USB_BDTBaseAddressGet ( USB_MODULE_ID index )

  Summary:
    Returns the base address of the Buffer Descriptor Table.

  Description:
    This function returns the base address of the Buffer Descriptor Table.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    void * pMyBDT;
    pMyBDT = PLIB_USB_BDTBaseAddressGet(MY_USB_INSTANCE);
  </code>

  Remarks:
    Must be set for PIC32 devices using PLIB_USB_BDTBaseAddressSet.
	
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTBaseAddress in your 
	application to determine whether this feature is available.		
*/

void* PLIB_USB_BDTBaseAddressGet ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_BDTBaseAddressSet ( USB_MODULE_ID index, void * address )

  Summary:
    Sets the base address for the Buffer Descriptor Table for PIC32 
    devices.

  Description:
    This function sets the base address for the Buffer Descriptor Table. This 
    function is only available on PIC32 devices.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    address        - Physical memory address in RAM of Buffer Descriptor Table

  Returns:
    None.

  Example:
  <code>
#if defined(__PIC32MX__)
    // For PIC32
    PLIB_USB_BDTBaseAddressSet(MY_USB_INSTANCE, (void *)((uint32_t)KVA_TO_PA(&myBDT)) );
#else
    // Everybody else
    PLIB_USB_BDTBaseAddressSet(MY_USB_INSTANCE, (void *)(&myBDT) );
#endif
  </code>

  Remarks:
    The address of the Buffer Descriptor Table must be 512 byte-aligned.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTBaseAddress in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_BDTBaseAddressSet ( USB_MODULE_ID index, void* address );


// *****************************************************************************
/* Function:
    uint8_t  PLIB_USB_BufferIndexGet ( USB_MODULE_ID index,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Gets the Buffer Descriptor Table index for a buffer.

  Description:
    This function gets the Buffer Descriptor Table index for a buffer.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    Buffer index into the Buffer Descriptor Table.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

uint8_t  PLIB_USB_BufferIndexGet ( USB_MODULE_ID index,
                                   USB_PING_PONG_MODE ppMode,
                                   uint8_t epValue,
                                   USB_BUFFER_DIRECTION bufferDirection,
                                   USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    void * PLIB_USB_BufferAddressGet ( USB_MODULE_ID index,
                                       void * pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Gets the memory address of an endpoint buffer.

  Description:
    This function gets the memory address of an endpoint buffer.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint Value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    Buffer address in memory.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

void* PLIB_USB_BufferAddressGet ( USB_MODULE_ID index,
                                  void* pBDT,
                                  USB_PING_PONG_MODE ppMode,
                                  uint8_t epValue,
                                  USB_BUFFER_DIRECTION bufferDirection,
                                  USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferAddressSet ( USB_MODULE_ID index,
                                     void * pBDT,
                                     USB_PING_PONG_MODE ppMode,
                                     uint8_t epValue,
                                     USB_BUFFER_DIRECTION bufferDirection,
                                     USB_BUFFER_PING_PONG  bufferPingPong,
                                     void * bufferAddress )
  Summary:
    Sets the endpoint buffer address.

  Description:
    This function sets the endpoint buffer address.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD
    bufferAddress - address in memory of endpoint transmit or receive buffer

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_BufferAddressSet ( USB_MODULE_ID index,
                                 void* pBDT,
                                 USB_PING_PONG_MODE ppMode,
                                 uint8_t epValue,
                                 USB_BUFFER_DIRECTION bufferDirection,
                                 USB_BUFFER_PING_PONG  bufferPingPong,
                                 void* bufferAddress );


// *****************************************************************************
/* Function:
    uint16_t PLIB_USB_BufferByteCountGet ( USB_MODULE_ID index,
                                           void * pBDT,
                                           USB_PING_PONG_MODE ppMode,
                                           uint8_t epValue,
                                           USB_BUFFER_DIRECTION bufferDirection,
                                           USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Returns the endpoint buffer byte count.

  Description:
    This function returns the endpoint buffer byte count, the actual number of 
    bytes transmitted or received.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    Endpoint buffer byte count.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

uint16_t PLIB_USB_BufferByteCountGet ( USB_MODULE_ID index,
                                       void* pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    PLIB_USB_BufferByteCountSet ( USB_MODULE_ID index,
                                  void * pBDT,
                                  USB_PING_PONG_MODE ppMode,
                                  uint8_t epValue,
                                  USB_BUFFER_DIRECTION bufferDirection,
                                  USB_BUFFER_PING_PONG  bufferPingPong,
                                  uint16_t bufferByteCount )

  Summary:
    Sets the buffer byte count.

  Description:
    This function sets the number of bytes to be transmitted or the maximum number 
    of bytes to be received.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD
    bufferByteCount - number of bytes to be transmitted or the maximum number of bytes to be received

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

void     PLIB_USB_BufferByteCountSet ( USB_MODULE_ID index,
                                       void* pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong,
                                       uint16_t bufferByteCount );

// *****************************************************************************
/* Function:
    USB_BUFFER_DATA01
        PLIB_USB_BufferDataToggleGet ( USB_MODULE_ID index,
                                       void * pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Returns data synchronization (DATA0 or DATA1) for the endpoint buffer.

  Description:
    This function returns data synchronization (DATA0 or DATA1) for the endpoint 
    buffer.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    Data Toggle value, USB_BUFFER_DATA0 or USB_BUFFER_DATA1, for the buffer

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

USB_BUFFER_DATA01 PLIB_USB_BufferDataToggleGet ( USB_MODULE_ID index,
                                                 void* pBDT,
                                                 USB_PING_PONG_MODE ppMode,
                                                 uint8_t epValue,
                                                 USB_BUFFER_DIRECTION bufferDirection,
                                                 USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferDataToggleSelect ( USB_MODULE_ID index,
                                           void * pBDT,
                                           USB_PING_PONG_MODE ppMode,
                                           uint8_t epValue,
                                           USB_BUFFER_DIRECTION bufferDirection,
                                           USB_BUFFER_PING_PONG  bufferPingPong,
                                           USB_BUFFER_DATA01 bufferData01 )

  Summary:
    Sets the endpoint buffer to DATA0 or DATA1.

  Description:
    This function sets the endpoint buffer to DATA0 or DATA1.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD
    bufferData01  - USB_BUFFER_DATA0 or USB_BUFFER_DATA1

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    See PLIB_USB_BufferDataToggleGet to determine the received data toggle setting.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_BufferDataToggleSelect ( USB_MODULE_ID index,
                                       void* pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong,
                                       USB_BUFFER_DATA01 bufferData01 );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferDataToggleSyncDisable ( USB_MODULE_ID index,
                                                void * pBDT,
                                                USB_PING_PONG_MODE ppMode,
                                                uint8_t epValue,
                                                USB_BUFFER_DIRECTION bufferDirection,
                                                USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Disables DATA0/DATA1 synchronization between the device and host.

  Description:
    This function disables DATA0/DATA1 synchronization between the device and host.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_BufferDataToggleSyncDisable ( USB_MODULE_ID index,
                                            void* pBDT,
                                            USB_PING_PONG_MODE ppMode,
                                            uint8_t epValue,
                                            USB_BUFFER_DIRECTION bufferDirection,
                                            USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferDataToggleSyncEnable  ( USB_MODULE_ID index,
                                                void * pBDT,
                                                USB_PING_PONG_MODE ppMode,
                                                uint8_t epValue,
                                                USB_BUFFER_DIRECTION bufferDirection,
                                                USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Enables DATA0/DATA1 synchronization between the device and host.

  Description:
    This function enables DATA0/DATA1 synchronization between the device and host.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_BufferDataToggleSyncEnable  ( USB_MODULE_ID index,
                                            void* pBDT,
                                            USB_PING_PONG_MODE ppMode,
                                            uint8_t epValue,
                                            USB_BUFFER_DIRECTION bufferDirection,
                                            USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    uint8_t PLIB_USB_BufferPIDGet ( USB_MODULE_ID index,
                                    void * pBDT,
                                    USB_PING_PONG_MODE ppMode,
                                    uint8_t epValue,
                                    USB_BUFFER_DIRECTION bufferDirection,
                                    USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Returns the token packet ID (PID) from the endpoint buffer status.

  Description:
    This function returns the token packet ID (PID) from the endpoint buffer status.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    Endpoint buffer packet ID (PID).

  Example:
  <code>
  </code>

  Remarks:
    There is no equivalent "Set" routine, since this field is read-only in the
    buffer status register within the Buffer Descriptor Table.  It is set
    when the buffer has been transmitted or received by the USB module and the
    usbOwnsBuffer field has been cleared by the USB module, releasing the buffer
    for software access.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.		
*/

uint8_t PLIB_USB_BufferPIDGet ( USB_MODULE_ID index,
                                void* pBDT,
                                USB_PING_PONG_MODE ppMode,
                                uint8_t epValue,
                                USB_BUFFER_DIRECTION bufferDirection,
                                USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    bool PLIB_USB_BufferReleasedToSW ( USB_MODULE_ID index,
                                       void * pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Returns the boolean flag value of 'true' when the buffer has been released by 
    the USB module.

  Description:
    This function returns the boolean flag value of 'true' when the buffer has 
    been released by the USB module.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    - true  - The buffer has been released by hardware
    - false - The buffer is still controlled by hardware

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_BufferReleasedToSW ( USB_MODULE_ID index,
                                   void* pBDT,
                                   USB_PING_PONG_MODE ppMode,
                                   uint8_t epValue,
                                   USB_BUFFER_DIRECTION bufferDirection,
                                   USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferReleaseToUSB ( USB_MODULE_ID index,
                                       void * pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Releases the endpoint buffer by software, allowing the USB module access to 
    the buffer.

  Description:
    This function releases the endpoint buffer by software, allowing the USB module 
    access to buffer.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_BufferReleaseToUSB ( USB_MODULE_ID index,
                                   void* pBDT,
                                   USB_PING_PONG_MODE ppMode,
                                   uint8_t epValue,
                                   USB_BUFFER_DIRECTION bufferDirection,
                                   USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
  void PLIB_USB_BufferCancelReleaseToUSB ( USB_MODULE_ID index,
                                           void * pBDT,
                                           USB_PING_PONG_MODE ppMode,
                                           uint8_t epValue,
                                           USB_BUFFER_DIRECTION bufferDirection,
                                           USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Cancels release of the endpoint buffer by software, allowing software to again 
    access the buffer.

  Description:
    This function cancels the release of the endpoint buffer by software, allowing 
    software to again access the buffer.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_BufferCancelReleaseToUSB ( USB_MODULE_ID index,
                                         void* pBDT,
                                         USB_PING_PONG_MODE ppMode,
                                         uint8_t epValue,
                                         USB_BUFFER_DIRECTION bufferDirection,
                                         USB_BUFFER_PING_PONG  bufferPingPong );


//******************************************************************************
/* Function :
    void PLIB_USB_BufferSchedule( USB_MODULE_ID  index ,
                                  void*  pBDT ,
                                  USB_PING_PONG_MODE  ppMode ,
                                  uint8_t  epValue ,
                                  USB_BUFFER_DIRECTION  bufferDirection ,
                                  USB_BUFFER_PING_PONG  bufferPingPong ,
                                  void *  bufferAddress ,
                                  int16_t  bufferByteCount ,
                                  USB_BUFFER_SCHEDULE_DATA01 bufferData01 )

  Summary:
    Hands over a buffer to the USB module along with the buffer address and byte count.

  Description:
    This function sets the endpoint descriptor buffer address, sets the send/receive
    byte count, and then hands over the buffer to the USB module.

  Preconditions:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong  - USB_BUFFER_EVEN or USB_BUFFER_ODD
    bufferAddress   - Address of the application buffer
    bufferByteCount - Send or expected receive byte count
    bufferData01    - USB_BUFFER_SET_DATA0, USB_BUFFER_SET_DATA1, or USB_BUFFER_DONTCHANGE
                      (The last choice leaves the existing DATA0/1 value of the buffer alone.)

  Returns:
    None.

  Remarks:
    This function does the work of three other functions:
        PLIB_USB_BufferAddressSet,
        PLIB_USB_BufferByteCountSet,
        PLIB_USB_BufferReleaseToUSB
		
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.			

*/

void PLIB_USB_BufferSchedule( USB_MODULE_ID  index ,
                              void*  pBDT ,
                              USB_PING_PONG_MODE  ppMode ,
                              uint8_t  epValue ,
                              USB_BUFFER_DIRECTION  bufferDirection ,
                              USB_BUFFER_PING_PONG  bufferPingPong ,
                              void *  bufferAddress,
                              int16_t  bufferByteCount ,
                              USB_BUFFER_SCHEDULE_DATA01 bufferData01 );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferStallDisable ( USB_MODULE_ID index,
                                       void * pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Disables STALL handshaking for the associated endpoint buffer.

  Description:
    This function disables STALL handshaking for the associated endpoint buffer.

  Precondition:
    The associated buffer must have been released by the USB module (i.e., 
    PLIB_USB_BufferReleasedToSW returns 'true').

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    Release of a STALL handshake for the buffer is done by hardware when the 
    host sends a SETUP token to the associated endpoint or resets the USB module
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.		.
*/

void PLIB_USB_BufferStallDisable ( USB_MODULE_ID index,
                                   void* pBDT,
                                   USB_PING_PONG_MODE ppMode,
                                   uint8_t epValue,
                                   USB_BUFFER_DIRECTION bufferDirection,
                                   USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferStallEnable  ( USB_MODULE_ID index,
                                       void * pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Enables STALL handshaking for the associated endpoint buffer.

  Description:
    This function enables STALL handshaking for the associated endpoint buffer.

  Precondition:
    The associated buffer must have been released by the USB module (i.e. 
    PLIB_USB_BufferReleasedToSW returns 'true').

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    Release of a STALL handshake for the buffer is done by hardware when the host
    sends a SETUP token to the associated endpoint or resets the USB module.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_BufferStallEnable  ( USB_MODULE_ID index,
                                   void* pBDT,
                                   USB_PING_PONG_MODE ppMode,
                                   uint8_t epValue,
                                   USB_BUFFER_DIRECTION bufferDirection,
                                   USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    bool PLIB_USB_BufferStallGet ( USB_MODULE_ID index,
                                   void * pBDT,
                                   USB_PING_PONG_MODE ppMode,
                                   uint8_t epValue,
                                   USB_BUFFER_DIRECTION bufferDirection,
                                   USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Returns the buffer stall status for an endpoint/direction/ping-pong.

  Description:
    This function returns the buffer stall status for an endpoint/direction/ping-pong.

  Precondition:
    the associated buffer must have been released by the USB module (i.e., 
    PLIB_USB_BufferReleasedToSW returns 'true').

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    - true  - Buffer stall is enabled
    - false - Buffer stall is not enabled

  Example:
  <code>
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

bool  PLIB_USB_BufferStallGet ( USB_MODULE_ID index,
                                void* pBDT,
                                USB_PING_PONG_MODE ppMode,
                                uint8_t epValue,
                                USB_BUFFER_DIRECTION bufferDirection,
                                USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferPIDBitsClear ( USB_MODULE_ID index,
                                       void * pBDT,
                                       USB_PING_PONG_MODE ppMode,
                                       uint8_t epValue,
                                       USB_BUFFER_DIRECTION bufferDirection,
                                       USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Clears the Buffer Status bits in the Buffer Descriptor Table.

  Description:
    This function clears the Buffer Status bits in the Buffer Descriptor Table.

  Precondition:
    The associated buffer must have been released by the USB module (i.e., 
    PLIB_USB_BufferReleasedToSW returns 'true'.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    Call PLIB_USB_BufferPIDBitsClear before setting buffer control bits.
    This is equivalent to:
        PLIB_USB_BufferCancelReleaseToUSB(...)
        PLIB_USB_BufferDataToggleSelect( ...,USB_BUFFER_DATA0)
        PLIB_USB_BufferDataToggleSyncDisable(...)
        PLIB_USB_BufferStallDisable(...)
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.			
*/

void PLIB_USB_BufferPIDBitsClear ( USB_MODULE_ID index,
                                   void* pBDT,
                                   USB_PING_PONG_MODE ppMode,
                                   uint8_t epValue,
                                   USB_BUFFER_DIRECTION bufferDirection,
                                   USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
/* Function:
    void PLIB_USB_BufferClearAll( USB_MODULE_ID index,
                                  void * pBDT,
                                  USB_PING_PONG_MODE ppMode,
                                  uint8_t epValue,
                                  USB_BUFFER_DIRECTION bufferDirection,
                                  USB_BUFFER_PING_PONG  bufferPingPong )

  Summary:
    Clears (zeros out) entries in the Buffer Descriptor Table.

  Description:
    This function clears (zeros out) the entries in the Buffer Descriptor Table.

  Precondition:
    None.

  Parameters:
    index    - Dummy argument, identifier for the device instance of interest
    pBDT     - Pointer to start of Buffer Descriptor Table
    ppMode   - Ping-Pong buffering mode
    epValue  - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    bufferDirection - USB_BUFFER_RX or USB_BUFFER_TX
    bufferPingPong - USB_BUFFER_EVEN or USB_BUFFER_ODD

  Returns:
    None.

  Example:
  <code>
  </code>
  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	  
*/
void PLIB_USB_BufferClearAll( USB_MODULE_ID index,
                              void * pBDT,
                              USB_PING_PONG_MODE ppMode,
                              uint8_t epValue,
                              USB_BUFFER_DIRECTION bufferDirection,
                              USB_BUFFER_PING_PONG  bufferPingPong );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: Endpoints
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USB_EP0LSDirectConnectDisable ( USB_MODULE_ID index )

  Summary:
    Disables direct connection to a low-speed device for Endpoint 0.

  Description:
    This function disables direct connection to a low-speed device for Endpoint 0.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EP0LSDirectConnectDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Host mode and U1EP0 only.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEP0LowSpeedConnect in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_EP0LSDirectConnectDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_EP0LSDirectConnectEnable ( USB_MODULE_ID index )

  Summary:
     Enables direct connection to a low-speed device for Endpoint 0.

  Description:
     This function enables direct connection to a low-speed device for Endpoint 0.

  Precondition:
    USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EP0LSDirectConnectEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Host Mode and U1EP0 only.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEP0LowSpeedConnect in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_EP0LSDirectConnectEnable ( USB_MODULE_ID index );


// *****************************************************************************
/*  Function:
    void PLIB_USB_EP0NakRetryDisable ( USB_MODULE_ID index )

  Summary:
    Disables retrying of NAKed transactions.

  Description:
    This function disables retrying of NAKed transactions.

  Precondition:
    The USB module must be in Host or OTG modes.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EP0NakRetryDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Host/OTG only.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEP0NAKRetry in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_EP0NakRetryDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_EP0NakRetryEnable ( USB_MODULE_ID index )

  Summary:
    Enables retrying NAK'd transactions for Endpoint 0.

  Description:
    This function enables retrying NAK'd transactions for Endpoint 0.

  Precondition:
    The USB module must be in Host or OTG modes.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EP0NakRetryEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Host/OTG only.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEP0NAKRetry in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_EP0NakRetryEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_EPnControlTransferDisable ( USB_MODULE_ID index, uint8_t epValue )

  Summary:
    Disables endpoint control transfers.

  Description:
    This function disables endpoint control transfers when endpoint transmit and 
    endpoint receive are both enabled.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EPnControlTransferDisable(MY_USB_INSTANCE, someEP);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnControlTransferDisable ( USB_MODULE_ID index, uint8_t epValue );

// *****************************************************************************
/* Function:
    void PLIB_USB_EPnAttributesSet ( USB_MODULE_ID index, 
            uint8_t epValue, int direction, bool isControl, bool handshake)

  Summary:
    Configures attributes of the endpoint such as direction, handshake capability
    and direction.

  Description:
    Configures attributes of the endpoint such as direction, handshake capability
    and direction. If the isControl flag is true, then the direction and handshake
    parameters are ignored and the endpoint is configured for control transfers. 

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    direction      - Endpoint direction, if 1 then RX and if 0 then TX.
    isControl      - If true endpoint is configured for control transfers.
    handshake      - If true, then handshake is enabled on the endpoint else
                     it is disabled.

  Returns:
    None.

  Example:
  <code>
    // This enables endpoint 0 for control transfers. 
    PLIB_USB_EPnAttributesSet(MY_USB_INSTANCE, 0, 0, true, true);
    // This enables endpoint 2 for non control transfer, direction
    // is RX and handshake enable. 
    PLIB_USB_EPnAttributesSet(MY_USB_INSTANCE, 2, 1, false, true);
  </code>

  Remarks:
    None.
*/

void PLIB_USB_EPnAttributesSet ( USB_MODULE_ID index, 
        uint8_t epValue, int direction, bool isControl, bool handshake);

// *****************************************************************************
/* Function:
    void PLIB_USB_EPnAttributesClear ( USB_MODULE_ID index, uint8_t epValue)

  Summary:
    Clears the set attributes of the specified endpoint.

  Description:
    Clears the set attributes of the specified endpoint. The endpoint transmit
    receive, handshake and setup packet handling capability is disabled.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint

  Returns:
    None.

  Example:
  <code>
    // This clears up the endpoint 0 attributes and thus disables
    // the endpoints
    PLIB_USB_EPnAttributesClear(MY_USB_INSTANCE, 0);
  </code>

  Remarks:
    None.
*/

void PLIB_USB_EPnAttributesClear ( USB_MODULE_ID index, uint8_t epValue);

// *****************************************************************************
/* Function:
    void PLIB_USB_EPnDirectionDisable ( USB_MODULE_ID index, 
                                            uint8_t epValue, int direction)

  Summary:
    Disables the specified endpoint direction.

  Description:
    Disables the specified endpoint direction.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    direction      - If 1, then TX direction is disabled. If 0 RX direction is
                     disabled.

  Returns:
    None.

  Example:
  <code>
    // This function disables the TX direction of endpoint 1
    PLIB_USB_EPnDirectionDisable(MY_USB_INSTANCE, 1, 1);
  </code>

  Remarks:
    None.
*/

void PLIB_USB_EPnDirectionDisable ( USB_MODULE_ID index, 
                                uint8_t epValue, int direction);

// *****************************************************************************
/* Function:
    void PLIB_USB_EPnControlTransferEnable ( USB_MODULE_ID index, uint8_t epValue )

  Summary:
    Enables endpoint control transfers.

  Description:
    This function enables endpoint control transfers when endpoint transmit and 
    endpoint receive are both enabled.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EPnControlTransferEnable(MY_USB_INSTANCE, someEP);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnControlTransferEnable ( USB_MODULE_ID index, uint8_t epValue );


// *****************************************************************************
/* Function:
    void PLIB_USB_EPnHandshakeDisable ( USB_MODULE_ID index, uint8_t epValue )

  Summary:
    Disables endpoint handshaking.

  Description:
    This function disables endpoint handshaking.  Typically used for Isochronous endpoints.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EPnHandshakeDisable(MY_USB_INSTANCE, someEP);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnHandshakeDisable ( USB_MODULE_ID index, uint8_t epValue );


// *****************************************************************************
/* Function:
    void PLIB_USB_EPnHandshakeEnable ( USB_MODULE_ID index, uint8_t epValue )

  Summary:
    Enables endpoint handshaking.

  Description:
    This function enables endpoint handshaking.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EPnHandshakeEnable(MY_USB_INSTANCE, someEP);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnHandshakeEnable ( USB_MODULE_ID index, uint8_t epValue );

// *****************************************************************************
/* Function:
    bool PLIB_USB_EPnIsStalled ( USB_MODULE_ID index, uint8_t epValue  )

  Summary:
    Tests whether the endpoint epValue is stalled.

  Description:
    This function tests whether the endpoint epValue is stalled.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint

  Returns:
    - true  - The endpoint is stalled
    - false - The endpoint is not stalled

  Example:
  <code>
    if( PLIB_USB_EPnIsStalled(MY_USB_INSTANCE, someEP) )
    {
        // Handle the stall
    }
  </code>

  Remarks:
    Not valid before an endpoint is enabled.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_EPnIsStalled ( USB_MODULE_ID index, uint8_t epValue );


// *****************************************************************************
/* Function:
    void PLIB_USB_EPnStallClear ( USB_MODULE_ID index, uint8_t epValue )

  Summary:
    Clears an endpoint's stalled flag.

  Description:
    This function clears an endpoint's stalled flag.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint

  Returns:
    None.

  Example:
  <code>
    if( PLIB_USB_EPnIsStalled(MY_USB_INSTANCE, someEP) )
    {
        // Handle the stall
        PLIB_USB_EPnStallClear(MY_USB_INSTANCE, someEP);
    }
  </code>

  Remarks:
    Not valid before an endpoint is enabled.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_EPnStallClear ( USB_MODULE_ID index, uint8_t epValue  );


// *****************************************************************************
/* Function:
    void PLIB_USB_EPnRxSelect ( USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx )

  Summary:
    Selects receive capabilities of an endpoint.

  Description:
    This function selects receive capabilities of an endpoint.

  Precondition:
    None.

  Parameters:
    index   - Identifier for the device instance of interest
    epValue - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    epTxRx  - Transmit/Receive setting for endpoint: USB_EP_RX, USB_EP_NOTXRX

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EPnRxSelect(MY_USB_INSTANCE, someEP, USB_EP_RX);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnRxSelect ( USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx );


// *****************************************************************************
/* Function:
    void PLIB_USB_EPnTxSelect ( USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx )

  Summary:
    Selects transmit capabilities of an endpoint.

  Description:
    This function selects transmit capabilities of an endpoint.

  Precondition:
    None.

  Parameters:
    index   - Identifier for the device instance of interest
    epValue - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    epTxRx  - Transmit/Receive setting for endpoint: USB_EP_TX, USB_EP_NOTXRX

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EPnTxSelect(MY_USB_INSTANCE, someEP, USB_EP_TX);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnTxSelect ( USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx );


// *****************************************************************************
/* Function:
    void PLIB_USB_EPnTxRxSelect ( USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx )

  Summary:
    Selects transmit and/or receive capabilities of an endpoint.

  Description:
    This function selects transmit and/or receive capabilities of an endpoint.

  Precondition:
    None.

  Parameters:
    index   - Identifier for the device instance of interest
    epValue - Endpoint value, 0 <= epValue <= Module Maximum Endpoint
    epTxRx  - Transmit/Receive setting for endpoint: USB_EP_TX, USB_EP_RX, USB_EP_TX_RX, USB_EP_NOTXRX

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EPnTxRxSelect(MY_USB_INSTANCE, someEP, USB_EP_TXRX);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnTxRxSelect ( USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: Interrupts
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USB_InterruptDisable( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag )

  Summary:
    Disables a general interrupt for the USB module.

  Description:
    This function disables a general interrupt source for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_InterruptDisable( MY_USB_INSTANCE, USB_INT_ERROR );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_Interrupt in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_InterruptDisable( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag );


// *****************************************************************************
/* Function:
    void PLIB_USB_InterruptEnable( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag )

  Summary:
    Enables a general interrupt for the USB module.

  Description:
    This function enables general interrupt sources of the USB module to trigger 
    a USB interrupt.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_InterruptEnable( MY_USB_INSTANCE, USB_INT_ERROR );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_Interrupt in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_InterruptEnable( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag );

// *****************************************************************************
/* Function:
    USB_INTERRUPTS PLIB_USB_InterruptEnableGet( USB_MODULE_ID index)

  Summary:
    Returns the enable/disable status of general USB module interrupts

  Description:
    Returns the enable/disable status of general USB module interrupts

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    A bit map containing status of enabled interrupts.

  Example:
  <code>
    USB_INTERRUPTS enabledInterrupts;
    enabledInterrupts = PLIB_USB_InterruptEnableGet( MY_USB_INSTANCE);
    if(enabledInterrupts|USB_INT_ATTACH)
    {
        // This means Attach interrupt is enabled.
    }

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_Interrupt in your 
	application to determine whether this feature is available.	
*/
USB_INTERRUPTS PLIB_USB_InterruptEnableGet(USB_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USB_TokenSend(USB_MODULE_ID index, USB_PID pidValue, 
            uint8_t endpoint, uint8_t deviceAddress, bool isLowSpeed);

  Summary:
    Sends token to the specified address.

  Description:
    This function sends the specified token to the specified endpoint and
    address. The token is placed on the bus at the next available time. The
    token can be executed at low speed. 

  Precondition:
    None.

  Parameters:
    index       - Identifier for the device instance of interest
    pidValue    - PID of the token to be placed on the bus.
    endpoint    - Device endpoint to which the token should be sent.
    isLowSpeed  - Is true if the token should be executed at low speed. 

  Returns:
    None.

  Example:
  <code>
    // Send an OUT token to endpoint 1 device address 2 at full speed
    PLIB_USB_SendToken(MY_USB_INSTANCE, USB_PID_OUT, 1, 2, false);

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_Interrupt in your 
	application to determine whether this feature is available.	
*/
void PLIB_USB_TokenSend(USB_MODULE_ID index, USB_PID pidValue, uint8_t endpoint, uint8_t deviceAddress, bool isLowSpeed);

// *****************************************************************************
/* Function:
    void PLIB_USB_EP0HostSetup(USB_MODULE_ID index);

  Summary:
    Sends token to the specified address.

  Description:
    This function configures endpoint 0 for typical host operation. Control transfers
    are enable. Transmit and Receive is enabled. Handshaking is enabled. Low Speed
    connection is disabled. NAK retry is disabled.

  Precondition:
    None.

  Parameters:
    index       - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    // Configure endpoint 0 for host operation

    PLBIB_USB_EP0HostSetup(USB_MODULE_ID index);

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_Interrupt in your 
	application to determine whether this feature is available.	
*/
void PLIB_USB_EP0HostSetup(USB_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USB_InterruptFlagClear( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag )

  Summary:
    Clears a general interrupt flag for the USB module.

  Description:
    This function clears a general interrupt source flag for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_InterruptFlagClear( MY_USB_INSTANCE, USB_INT_ERROR );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_InterruptFlagClear( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag );


// *****************************************************************************
/* Function:
    bool PLIB_USB_InterruptFlagGet( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag )

  Summary:
    Tests a general interrupt flag for the USB module.

  Description:
    This function tests a general interrupt source flag for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    if ( PLIB_USB_InterruptFlagGet(MY_USB_INSTANCE,USB_INT_ANY) )
        if ( PLIB_USB_InterruptFlagGet(MY_USB_INSTANCE,USB_INT_ERROR) )
        {
            PLIB_USB_InterruptFlagClear(MY_USB_INSTANCE,USB_INT_ERROR);
            // Error clean up
        }
        if ( PLIB_USB_InterruptFlagGet(MY_USB_INSTANCE,USB_INT_HOST_DETACH) )
        {
            PLIB_USB_InterruptFlagClear(MY_USB_INSTANCE,USB_INT_HOST_DETACH);
            // Device detached clean up
        }
        .
        .
        .
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_InterruptFlagGet( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag );


// *****************************************************************************
/* Function:
    void PLIB_USB_InterruptFlagSet( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag )

  Summary:
    Sets a general interrupt flag for the USB module.

  Description:
    This function sets a general interrupt source flag for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_InterruptFlagSet( MY_USB_INSTANCE, USB_INT_ERROR );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_InterruptFlagSet( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag );


// *****************************************************************************
/* Function:
    bool PLIB_USB_InterruptIsEnabled( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag )

  Summary:
    Returns true if interrupts are enabled.

  Description:
    This function returns true if interrupts are enabled.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    - true  - Interrupts are enabled
    - false - Interrupts are not enabled

  Example:
  <code>
    if ( PLIB_USB_InterruptIsEnabled( MY_USB_INSTANCE, USB_INT_ERROR ) )
    {

    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_Interrupt in your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_InterruptIsEnabled( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: Error Interrupts
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USB_ErrorInterruptDisable( USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag )

  Summary:
    Disables an error interrupt for the USB module.

  Description:
    This function disables an error interrupt source for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_ErrorInterruptDisable( MY_USB_INSTANCE, USB_ERR_INT_BAD_CRC16 );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsERR_Interrupt in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_ErrorInterruptDisable( USB_MODULE_ID index, USB_ERROR_INTERRUPTS   interruptFlag );


// *****************************************************************************
/* Function:
    void PLIB_USB_ErrorInterruptEnable( USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag )

  Summary:
    Enables an error interrupt for the USB module.

  Description:
    This function enables error interrupt sources of the USB module to trigger 
    a USB interrupt.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_ErrorInterruptEnable( MY_USB_INSTANCE, USB_ERR_INT_BAD_CRC16 );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsERR_Interrupt in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_ErrorInterruptEnable( USB_MODULE_ID index, USB_ERROR_INTERRUPTS   interruptFlag );


// *****************************************************************************
/* Function:
    void PLIB_USB_ErrorInterruptFlagClear( USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag )

  Summary:
    Clears an error interrupt flag for the USB module.

  Description:
    This function clears an error interrupt source flag for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_ErrorInterruptFlagClear( MY_USB_INSTANCE, USB_ERR_INT_BAD_CRC16 );
  </code>

  Remarks:
    None.
*/

void PLIB_USB_ErrorInterruptFlagClear( USB_MODULE_ID index, USB_ERROR_INTERRUPTS   interruptFlag );


// *****************************************************************************
/* Function:
    bool PLIB_USB_ErrorInterruptFlagGet( USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag )

  Summary:
    Tests an error interrupt flag for the USB module.

  Description:
    This function tests an error interrupt source flag for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    if ( PLIB_USB_ErrorInterruptFlagGet(MY_USB_INSTANCE,USB_ERR_INT_ANY) )
        if ( PLIB_USB_ErrorInterruptFlagGet(MY_USB_INSTANCE,USB_ERR_INT_PID_CHECK_FAILURE) )
        {
            PLIB_USB_ErrorInterruptFlagClear(MY_USB_INSTANCE,USB_ERR_INT_PID_CHECK_FAILURE);
            // PID Error Check failure cleanup
        }
        if ( PLIB_USB_ErrorInterruptFlagGet(MY_USB_INSTANCE,USB_ERR_INT_DEVICE_EOF_ERROR) )
        {
            PLIB_USB_ErrorInterruptFlagClear(MY_USB_INSTANCE,USB_ERR_INT_DEVICE_EOF_ERROR);
            // EOF error cleanup
        }
        .
        .
        .
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsERR_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_ErrorInterruptFlagGet( USB_MODULE_ID index, USB_ERROR_INTERRUPTS   interruptFlag );


// *****************************************************************************
/* Function:
    void PLIB_USB_ErrorInterruptFlagSet( USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag )

  Summary:
    Sets an error interrupt flag for the USB module.

  Description:
    This function sets an error interrupt source flag for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_ErrorInterruptFlagSet( MY_USB_INSTANCE, USB_ERR_INT_BAD_CRC16 );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsERR_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_ErrorInterruptFlagSet( USB_MODULE_ID index, USB_ERROR_INTERRUPTS   interruptFlag );


// *****************************************************************************
/* Function:
    bool PLIB_USB_ErrorInterruptIsEnabled( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag )

  Summary:
    Returns true if interrupts are enabled.

  Description:
    This function determines whether interrupts are enabled.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    - true  - Interrupts are enabled
    - false - Interrupts are not enabled

  Example:
  <code>
    if ( PLIB_USB_ErrorInterruptIsEnabled( MY_USB_INSTANCE, USB_ERR_INT_BAD_CRC16 ) )
    {

    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsERR_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_ErrorInterruptIsEnabled( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: Last Transaction Status
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    USB_BUFFER_DIRECTION PLIB_USB_LastTransactionDirectionGet ( USB_MODULE_ID index )

  Summary:
    Indicates the direction of the last transaction.

  Description:
    This function indicates the direction of the last transaction, either transmit 
    or receive.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    USB_LastDirection: USB_RECEIVE_TRANSFER or USB_TRANSMIT_TRANSFER

  Example:
    See PLIB_USB_LastTransactionEndPtGet.

  Remarks:
    None.
*/

USB_BUFFER_DIRECTION PLIB_USB_LastTransactionDirectionGet ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    uint8_t PLIB_USB_LastTransactionEndPtGet ( USB_MODULE_ID index )

  Summary:
    Returns the endpoint number of the last USB transfer.

  Description:
    This function returns the endpoint number of the last USB transfer, which is 
    actually the index into the Buffer Descriptor Table of the last USB transfer.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    endPoint - Endpoint of last completed USB transfer

  Example:
  <code>
    while ( !PLIB_USB_INT_FlagGet(MY_USB_INSTANCE,USB_INT_GEN,TOKEN_DONE) )
    {
        // Do nothing, wait until completion of next transaction
    }

    // Retrieve information relating to the last completed transaction
    endpoint  = PLIB_USB_LastTransactionEndPtGet(MY_USB_INSTANCE);
    direction = PLIB_USB_LastTransactionDirectionGet(MY_USB_INSTANCE);
    pingPongState = PLIB_USB_LastTransactionPingPongStateGet(MY_USB_INSTANCE);

    // Clearing the Token Processing Done flag advances the status FIFO to
    // oldest transaction in the FIFO.  Wait for completion of next transaction
    // before using PLIB_USB_Last*Get functions again to read status.
    PLIB_USB_INT_FlagClear(MY_USB_INSTANCE,USB_INT_GEN,TOKEN_DONE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsLastDirection in your 
	application to determine whether this feature is available.	
*/

uint8_t PLIB_USB_LastTransactionEndPtGet ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    USB_PING_PONG_STATE PLIB_USB_LastTransactionPingPongStateGet ( USB_MODULE_ID index )

  Summary:
    Indicates whether the last transaction was to an EVEN buffer or an ODD buffer.

  Description:
    This function indicates whether the last transaction was to an Even buffer or 
    an Odd buffer.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    USB_PING_PONG_STATE.

  Example:
    See PLIB_USB_LastTransactionEndPtGet.

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsLastPingPong in your 
	application to determine whether this feature is available.	
*/

USB_PING_PONG_STATE PLIB_USB_LastTransactionPingPongStateGet ( USB_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: Host
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    bool PLIB_USB_IsBusyWithToken ( USB_MODULE_ID index )

  Summary:
    Indicates whether there is a token being executed by the USB module as Host.

  Description:
    This function indicates whether there is a token being executed by the USB module 
    as Host. Software should check that the previous token is finished before issuing 
    a new token.

  Precondition:
    USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    while( PLIB_USB_IsBusyWithToken(MY_USB_INSTANCE) )
    {
        // do nothing
    }
    // Issue new token
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsHostBusyWithToken in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_IsBusyWithToken ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_SOFDisable ( USB_MODULE_ID index )

  Summary:
    Disables the automatic generation of the SOF token.

  Description:
    This function disables the automatic generation of the SOF token.

  Precondition:
    USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_SOFDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsStartOfFrames in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_SOFDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_SOFEnable ( USB_MODULE_ID index )

  Summary:
    Enables the automatic generation of the SOF token every 1 ms.

  Description:
    This function enables the automatic generation of the SOF token every 1 ms.

  Precondition:
    USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_SOFEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsStartOfFrames in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_SOFEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    uint8_t PLIB_USB_SOFThresholdGet ( USB_MODULE_ID index )

  Summary:
    Returns the Start-of-Frame (SOF) Count bits.

  Description:
    This function returns the Start-of-Frame (SOF) Count bits. (Value represents 
    10 + (packet size of n bytes);).

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    SOF threshold value.

  Example:
  <code>
    thresholdSOF = PLIB_USB_SOFThresholdGet(MY_USB_INSTANCE);
  </code>

  Remarks:
    Host mode only.
<pre><c>
    SOF Threshold Value = packet byte count + 10
                        = 0b0100_1010 = 0x4A = 74 for 64-byte packet
                        = 0b0010_1010 = 0x2A = 42 for 32-byte packet
                        = 0b0001_1010 = 0x1A = 26 for 16-byte packet
                        = 0x0001_0010 = 0x12 = 18 for  8-byte packet
 </c></pre>
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsSOFThreshold in your 
	application to determine whether this feature is available.	 
*/

uint8_t PLIB_USB_SOFThresholdGet ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_SOFThresholdSet ( USB_MODULE_ID index, uint8_t threshold )

  Summary:
    Sets the Start-of-Frame (SOF) threshold value.

  Description:
    This function sets the Start-of-Frame (SOF) threshold value.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest
    threshold      - SOF threshold

  Returns:
    None.

  Example:
  <code>
    // Set SOF threshold for 64-byte packets
    PLIB_USB_SOFThresholdSet(MY_USB_INSTANCE,64+10);
  </code>

  Remarks:
    Host mode only.
<pre><c>
    SOF Threshold Value = packet byte count + 10
                        = 0b0100_1010 = 0x4A = 74 for 64-byte packet
                        = 0b0010_1010 = 0x2A = 42 for 32-byte packet
                        = 0b0001_1010 = 0x1A = 26 for 16-byte packet
                        = 0x0001_0010 = 0x12 = 18 for  8-byte packet
 </c></pre>
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsSOFThreshold in your 
	application to determine whether this feature is available.	 
*/

void PLIB_USB_SOFThresholdSet ( USB_MODULE_ID index, uint8_t threshold );


// *****************************************************************************
/* Function:
    uint8_t PLIB_USB_TokenEPGet ( USB_MODULE_ID index )

  Summary:
    Returns the specified Endpoint address.

  Description: 
    This function returns the address of the specified Endpoint.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    Endpoint value - 0 <= epValue <= Module Maximum Endpoint.

  Example:
  <code>
    someEP = PLIB_USB_TokenEPGet(MY_USB_INSTANCE);
  </code>

  Remarks:
    Host mode only.
	
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsTokenEP in your 
	application to determine whether this feature is available.		
*/

uint8_t PLIB_USB_TokenEPGet ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_TokenEPSet ( USB_MODULE_ID index, uint8_t epValue )

  Summary:
    Sets the Endpoint address for a host transaction.

  Description:
    This function sets the Endpoint address for a host transaction.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest
    epValue        - Endpoint value, 0 <= epValue <= Module Maximum Endpoint

  Returns:
    None.

  Example:
  <code>
    uint8_t someEP = 0x03;
    PLIB_USB_TokenEPSet(MY_USB_INSTANCE, someEP);
  </code>

  Remarks:
    Host mode only.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsTokenEP in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_TokenEPSet ( USB_MODULE_ID index, uint8_t epValue );


// *****************************************************************************
/* Function:
    USB_PID PLIB_USB_TokenPIDGet ( USB_MODULE_ID index )

  Summary:
    Returns the token transaction type.

  Description:
    This function returns the token transaction type.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    Packet ID of token, USB_PID_SETUP, USB_PID_IN, or USB_PID_OUT

  Example:
  <code>
    somePID = PLIB_USB_TokenPIDGet(MY_USB_INSTANCE);
  </code>

  Remarks:
    Host mode only.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsTokenPID in your 
	application to determine whether this feature is available.		
*/

USB_PID PLIB_USB_TokenPIDGet ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_TokenPIDSet ( USB_MODULE_ID index, USB_PID pidValue)

  Summary:
    Sets the token transaction type to pidValue.

  Description:
    This function sets the token transaction type to pidValue.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest
    pidValue       - USB_PID_SETUP, USB_PID_IN, or USB_PID_OUT

  Returns:
    None.

  Example:
  <code>
    somePID = USB_PID_SETUP;
    PLIB_USB_TokenPIDSet (MY_USB_INSTANCE, somePID );
  </code>

  Remarks:
    Host mode only.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsTokenPID in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_TokenPIDSet ( USB_MODULE_ID index, USB_PID pidValue );


// *****************************************************************************
/* Function:
    void PLIB_USB_TokenSpeedSelect ( USB_MODULE_ID index, USB_TOKEN_SPEED tokenSpeed )

  Summary:
    Selects low speed or full speed for subsequent token executions.

  Description:
    This function selects low speed or full speed for subsequent token executions.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest
    tokenSpeed     - Speed for next token execution: USB_LOWSPEED_TOKENS or USB_FULLSPEED_TOKENS

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_TokenSpeedSet(MY_USB_INSTANCE,USB_LOWSPEED_TOKENS);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsNextTokenSpeed in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_TokenSpeedSelect ( USB_MODULE_ID index, USB_TOKEN_SPEED tokenSpeed );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: Bus Signaling
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USB_ResetSignalDisable ( USB_MODULE_ID index )

  Summary:
    Disables reset signaling on the USB bus.

  Description:
    This function disables reset signaling on the USB bus.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
    See PLIB_USB_ResetSignalEnable.

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsHostGeneratesReset in your 
	application to determine whether this feature is available.		

*/

void PLIB_USB_ResetSignalDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_ResetSignalEnable ( USB_MODULE_ID index )

  Summary:
    Enables reset signaling on the USB bus.

  Description:
    This function enables reset signaling on the USB bus.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    // Snippet to perform a software reset:
    PLIB_USB_ResetSignalEnable(MY_USB_INSTANCE);
    // ... delay 50ms
    PLIB_USB_ResetSignalDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsHostGeneratesReset in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_ResetSignalEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_ResumeSignalingDisable ( USB_MODULE_ID index )

  Summary:
    Disables resume signaling.

  Description:
    This function disables resume signaling.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
    See PLIB_USB_ResumeSignalingEnable.

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsResumeSignaling in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_ResumeSignalingDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_ResumeSignalingEnable ( USB_MODULE_ID index )

  Summary:
    Enables resume signaling.

  Description:
    This function enables resume signaling. Resume allows the peripheral to 
    perform a remote wake-up by executing resume signaling.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    // Perform resume signaling:
    PLIB_USB_ResumeSignalingEnable(MY_USB_INSTANCE);
    // Delay 10ms
    PLIB_USB_ResumeSignalingDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Software must enable resume signaling for 10 ms if the device is in Device mode,
    or for 25 ms if the device is in Host mode, and then disable resume signaling 
    to enable remote wake-up. In Host mode, the USB module will append a low-speed 
    EOP to the end resume signaling when it is disabled.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsResumeSignaling in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_ResumeSignalingEnable ( USB_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: On-The-Go (OTG)
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    bool PLIB_USB_OTG_BSessionHasEnded ( USB_MODULE_ID index )

  Summary:
    Returns the status of the B-Session End Indicator bit.

  Description:
    This function returns the status of the B-Session End Indicator bit.

  Precondition:
    The USB module must be in OTG mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    - true  - The VBUS Voltage is below VB_SESS_END on the B-device
    - false - The VBUS voltage is above VB_SESS_END on the B-device

  Example:
  <code>
    if ( !PLIB_USB_OTG_BSessionHasEnded(MY_USB_INSTANCE) )
    {
        // B session valid
    }
    else
    {
        // B session not valid
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_BSessionEnd in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_OTG_BSessionHasEnded ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    bool PLIB_USB_OTG_IDPinStateIsTypeA ( USB_MODULE_ID index )

  Summary:
    Returns the ID Pin state.

  Description:
    This function returns ID Pin state.

  Precondition:
    The USB module must be in OTG mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    - true  - Type A Cable attached,
    - false - No cable is attached or a Type B cable is attached

  Example:
  <code>
    if ( PLIB_USB_OTG_IDPinStateIsTypeA(MY_USB_INSTANCE) )
    {
        // Type A cable attached
    }
    else
    {
        // No cable or Type B cable attached
    };
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_IDPinState in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_OTG_IDPinStateIsTypeA ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    bool PLIB_USB_OTG_LineStateIsStable ( USB_MODULE_ID index )

  Summary:
    Returns the status of the Line Stable Indicator bit.

  Description:
    This function returns the status of the Line Stable Indicator bit.

  Precondition:
    The USB module must be in OTG mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    - true  - The USB line state has been stable for the previous 1 ms
    - false - The USB line state has not been stable for the previous 1 ms

  Example:
  <code>
    if( PLIB_USB_OTG_LineStateIsStable(MY_USB_INSTANCE) ) {
        // Line has been stable
        // ... rest of code ...
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_LineState in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_OTG_LineStateIsStable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_PullUpPullDownSetup( USB_MODULE_ID index,
                                           USB_OTG_PULL_UP_PULL_DOWN resistor,
                                           bool enableResistor )

  Summary:
    Enables or disables pull-up and pull-down resistors.

  Description:
    This function enables or disables pull-up and pull-down resistors.

  Precondition:
    USB On-The-Go (OTG) must be enabled.

  Parameters:
    index          - Identifier for the device instance of interest
    resistor       - USB_OTG_DPLUS_PULLUP,    USB_OTG_DMINUS_PULLUP,
                     USB_OTG_DPLUS_PULLDN, or USB_OTG_DMINUS_PULLDN
    enableResistor  - true to enable resistor, false to disable it

  Returns:
    None.

  Example:
  <code>
    // Enable pull-up resistor for D+
    PLIB_USB_OTG_PullUpPullDownSetup(MY_USB_INSTANCE,USB_OTG_DPLUS_PULLUP,true);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_PullUpPullDown in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_OTG_PullUpPullDownSetup( USB_MODULE_ID index,
                                       USB_OTG_PULL_UP_PULL_DOWN resistor,
                                       bool enableResistor );


// *****************************************************************************
/* Function:
    bool PLIB_USB_OTG_SessionValid ( USB_MODULE_ID index )

  Summary:
    Returns the status of the Session Valid Indicator bit.

  Description:
    This function returns the status of the Session Valid Indicator bit.

  Precondition:
    The USB module must be in OTG mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    - true  - The VBUS voltage is above Session Valid on the A or B device
    - false - The VBUS voltage is below Session Valid on the A or B device

  Example:
  <code>
    if ( PLIB_USB_OTG_SessionValid(MY_USB_INSTANCE) )
    {
        // Session valid
    }
    else
    {
        // Session not valid
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_SessionValid in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_OTG_SessionValid ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_VBusChargeDisable ( USB_MODULE_ID index )

  Summary:
    Disables VBUS line charge.

  Description:
    This function disables VBUS line charge.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_VBusChargeDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Not available on PIC32 devices.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_VbusCharge in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_OTG_VBusChargeDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_VBusDischargeEnable ( USB_MODULE_ID index )

  Summary:
    Enables VBUS line to be discharged through a resistor.

  Description:
    This function enables the VBUS line to be discharged through a resistor.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_VBusDischargeEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Not available on PIC32 devices.  This feature may not be available
    on all devices. Please refer to the specific device data sheet to determine
    availability or use PLIB_USB_ExistsOTG_VbusCharge in your application to
    determine whether this feature is available.
*/

void PLIB_USB_OTG_VBusDischargeEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_VBusChargeTo3V( USB_MODULE_ID index )

  Summary:
    Sets the VBUS line to charge to 3.3V.

  Description:
    This function sets the VBUS line to charge to 3.3V.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_VBusChargeTo3V(MY_USB_INSTANCE);
  </code>

  Remarks:
    Not available on PIC32 devices.
*/

void PLIB_USB_OTG_VBusChargeTo3V( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_VBusChargeTo5V ( USB_MODULE_ID index )

  Summary:
    Sets the VBUS line to charge to 5V.

  Description: 
    This function sets the VBUS line to charge to 5V.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_VBusChargeTo5V (MY_USB_INSTANCE);
  </code>

  Remarks:
    Not available on PIC32 devices.
*/

void PLIB_USB_OTG_VBusChargeTo5V( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_VBusDischargeDisable ( USB_MODULE_ID index )

  Summary:
    Disables VBUS line discharge.

  Description:
    This function disables VBUS line discharge.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_VBusDischargeDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Not available on PIC32 devices.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_VbusDischarge in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_OTG_VBusDischargeDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_VBusChargeEnable ( USB_MODULE_ID index )

  Summary:
    Enables the VBUS line to be charged through a pull-up resistor.

  Description:
    This function enables the VBUS line to be charged through a pull-up resistor.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_VBusChargeEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    Not available on PIC32 devices.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_VbusDischarge in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_OTG_VBusChargeEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_VBusPowerOff ( USB_MODULE_ID index )

  Summary:
    Turns off power on the VBUS Line.

  Description:
    This function turns off power on the VBUS Line.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_VBusPowerOff(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_VbusPowerOnOff in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_OTG_VBusPowerOff ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_VBusPowerOn ( USB_MODULE_ID index )

  Summary:
    Turns on power for the VBUS line.

  Description:
    This function turns on power for the VBUS line.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_VBusPowerOn(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_VbusPowerOnOff in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_OTG_VBusPowerOn ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    bool PLIB_USB_OTG_VBusValid ( USB_MODULE_ID index )

  Summary:
    Returns the status of the A-VBUS valid indicator.

  Description:
    This function returns the status of the A-VBUS valid indicator.

  Precondition:
    The USB module must be in OTG mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    - true  - The VBUS voltage is above VA_VBUS_VLD on the A-device,
    - false - The VBUS voltage is below VA_VBUS_VLD on the A-device

  Example:
  <code>
    if ( PLIB_USB_OTG_VBusValid(MY_USB_INSTANCE) )
    {
        // VBUS voltage above session valid for A device
    }
    else
    {
        // VBUS voltage below session valid for A device
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_ASessionValid in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_OTG_VBusValid ( USB_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: OTG Interrupts
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_InterruptDisable( USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag )

  Summary:
    Disables a USB On-The-Go (OTG) Interrupt for the USB module.

  Description:
    This function disables a USB On-The-Go (OTG) interrupt source for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_InterruptDisable( MY_USB_INSTANCE, USB_OTG_INT_ID_STATE_CHANGE );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_Interrupt in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_OTG_InterruptDisable( USB_MODULE_ID index, USB_OTG_INTERRUPTS     interruptFlag );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_InterruptEnable( USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag )

  Summary:
    Enables a USB On-The-Go (OTG) Interrupt for the USB module.

  Description:
    This function enables USB On-The-Go (OTG) interrupt sources of the USB module 
    to trigger a USB interrupt.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_InterruptEnable( MY_USB_INSTANCE, USB_OTG_INT_ID_STATE_CHANGE );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_Interrupt in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_OTG_InterruptEnable( USB_MODULE_ID index, USB_OTG_INTERRUPTS     interruptFlag );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_InterruptFlagClear( USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag )

  Summary:
    Clears a USB On-The-Go (OTG) Interrupt flag for the USB module.

  Description:
    This function clears a USB On-The-Go (OTG) interrupt source flag for the 
    USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_InterruptFlagClear( MY_USB_INSTANCE, USB_OTG_INT_ID_STATE_CHANGE );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_OTG_InterruptFlagClear( USB_MODULE_ID index, USB_OTG_INTERRUPTS     interruptFlag );


// *****************************************************************************
/* Function:
    bool PLIB_USB_OTG_InterruptFlagGet( USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag )

  Summary:
    Tests a USB On-The-Go (OTG) Interrupt flag for the USB module.

  Description:
    This function tests a USB On-The-Go (OTG) interrupt source flag for the 
    USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    if ( PLIB_USB_OTG_InterruptFlagGet(MY_USB_INSTANCE,USB_OTG_INT_ANY) )
        if ( PLIB_USB_OTG_InterruptFlagGet(MY_USB_INSTANCE,USB_OTG_INT_BDEVICE_SESSION_END) )
        {
            PLIB_USB_OTG_InterruptFlagClear(MY_USB_INSTANCE,USB_OTG_INT_ADEVICE_VBUS_VALID );
            // Device A VBUS Valid Change
        }
        if ( PLIB_USB_OTG_InterruptFlagGet(MY_USB_INSTANCE,USB_OTG_INT_BDEVICE_SESSION_END) )
        {
            PLIB_USB_OTG_InterruptFlagClear(MY_USB_INSTANCE,USB_OTG_INT_BDEVICE_SESSION_END);
            // Device B VBUS Valid Change
        }
        .
        .
        .
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_OTG_InterruptFlagGet( USB_MODULE_ID index, USB_OTG_INTERRUPTS     interruptFlag );


// *****************************************************************************
/* Function:
    void PLIB_USB_OTG_InterruptFlagSet( USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag )

  Summary:
    Sets a USB On-The-Go (OTG) Interrupt flag for the USB module.

  Description:
    This function sets a USB On-The-Go (OTG) interrupt source flag for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_OTG_InterruptFlagSet( MY_USB_INSTANCE, USB_OTG_INT_ID_STATE_CHANGE );
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_OTG_InterruptFlagSet( USB_MODULE_ID index, USB_OTG_INTERRUPTS     interruptFlag );


// *****************************************************************************
/* Function:
    bool PLIB_USB_OTG_InterruptIsEnabled( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag )

  Summary:
    Returns whether or not interrupts are enabled.

  Description:
    This function returns whether or not interrupts are enabled.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    - true  - Interrupts are enabled
    - false - Interrupts are not enabled

  Example:
  <code>
    if ( PLIB_USB_OTG_InterruptIsEnabled( MY_USB_INSTANCE, USB_OTG_INT_ID_STATE_CHANGE  ) )
    {

    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOTG_Interrupt in your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_OTG_InterruptIsEnabled( USB_MODULE_ID index, USB_INTERRUPTS interruptFlag );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: USB Activity
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    bool PLIB_USB_ActivityPending ( USB_MODULE_ID index )

  Summary:
    Returns whether or not USB activity is pending.

  Description:
    This function returns whether or not USB bus activity has been detected, an 
    interrupt is pending, or an interrupt is yet to be generated.

  Preconditions:
    None.

  Input:
    index -  Identifier for the device instance of interest

  Return:
    - true  - The USB module should not be suspended
    - false - No interrupts are pending or module may be suspended or powered down

  Example:
  <code>
    while ( PLIB_USB_ActivityPending(MY_USB_INSTANCE) )
    {
        // Wait
    }
    // Suspend USB module.
  </code>
  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsActivityPending in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_ActivityPending ( USB_MODULE_ID index );

// *****************************************************************************
/* Function:
    uint16_t PLIB_USB_FrameNumberGet ( USB_MODULE_ID index )

  Summary:
    Returns the USB frame number.

  Description:
    This function returns the USB frame number in the lower 11 bits.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    Current frame number in lower 11 bits.

  Example:
  <code>
    frameNumber = PLIB_USB_FrameNumberGet(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsFrameNumber in your 
	application to determine whether this feature is available.	
*/

uint16_t PLIB_USB_FrameNumberGet ( USB_MODULE_ID index );

// *****************************************************************************
/* Function:
    bool PLIB_USB_JStateIsActive ( USB_MODULE_ID index )

  Summary:
    Live differential receiver J State flag.

  Description:
    This function indicates the live JState (differential '0' in low speed, 
    differential '1' in full speed) on the bus.

  Precondition:
    The USB module must be in Host mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    // Enable Host Mode
    PLIB_USB_OperatingModeSelect(MY_USB_INSTANCE,USB_OPMODE_HOST);

    // Enable D+ and D- pull-down resistors
    PLIB_USB_OTG_PullUpPullDownSetup(MY_USB_INSTANCE,USB_OTG_DPLUS_PULLDN, true);
    PLIB_USB_OTG_PullUpPullDownSetup(MY_USB_INSTANCE,USB_OTG_DMINUS_PULLDN,true);

    // Disable D+ and D- pull-up resistors
    PLIB_USB_OTG_PullUpPullDownSetup(MY_USB_INSTANCE,USB_OTG_DPLUS_PULLUP, false);
    PLIB_USB_OTG_PullUpPullDownSetup(MY_USB_INSTANCE,USB_OTG_DMINUS_PULLUP,false);

    // Enable SOF Packet generation
    PLIB_USB_SOFEnable(MY_USB_INSTANCE);

    // Enable the device attach interrupt
    PLIB_USB_INT_Enable(MY_USB_INSTANCE,USB_INT_OTG,ACTIVITY_DETECT);

    // Wait for the Attach interrupt.
    while(!PLIB_USB_INT_FlagGet(MY_USB_INSTANCE,USB_INT_OTG,ACTIVITY_DETECT) )
    {
        //Do nothing
    }

    // Check JState
    if( PLIB_USB_JStateIsActive(MY_USB_INSTANCE) )
    {
        // Full Speed
        PLIB_USB_FullSpeedEnable(MY_USB_INSTANCE);
    }
    else
    {
        // Low Speed
        PLIB_USB_FullSpeedDisable(MY_USB_INSTANCE);
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsLiveJState in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_JStateIsActive ( USB_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_USB_PacketTransferEnable ( USB_MODULE_ID index )

  Summary:
    Re-enables the Serial Interface Engine (SIE), allowing token and packet
    processing.

  Description:
    This function re-enables the Serial Interface Engine (SIE), allowing token and 
    packet processing.

  Precondition:
    USB module must be in device mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  <code>
    if( PLIB_USB_PacketTransferIsDisabled(MY_USB_INSTANCE) )
    {
        // SETUP token received, do the needful operations
        .
        .
        .
        // SETUP handling completed, enable Setup token and packet processing:
        PLIB_USB_PacketTransferEnable(MY_USB_INSTANCE);
    }
  </code>

  Remarks:
    Not valid when the USB module is in Host mode.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsPacketTransfer in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_PacketTransferEnable ( USB_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_USB_PacketTransferDisable ( USB_MODULE_ID index )

  Summary:
    Disables the Serial Interface Engine (SIE).

  Description:
    This function disables the Serial Interface Engine (SIE).

  Precondition:
    USB module must be in device mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  <code>
    if( PLIB_USB_PacketTransferIsDisabled(MY_USB_INSTANCE) )
    {
        // SETUP token received, do the needful operations
        .
        .
        .
        // SETUP handling completed, enable Setup token and packet processing:
        PLIB_USB_PacketTransferDisable(MY_USB_INSTANCE);
    }
  </code>

  Remarks:
    Not valid when the USB module is in Host mode.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsPacketTransfer in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_PacketTransferDisable ( USB_MODULE_ID index );

// *****************************************************************************
/* Function:
    bool PLIB_USB_PacketTransferIsDisabled ( USB_MODULE_ID index )

  Summary:
    Indicates that a setup token has been received from the Host
    and that token/packet processing is disabled.

  Description:
    This function indicates that a setup token has been received from the Host
    and that the Serial Interface Engine (SIE) has been turned off, disabling 
    token and packet processing.

  Precondition:
    USB module must be in device mode.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    if( PLIB_USB_PacketTransferIsDisabled(MY_USB_INSTANCE) )
    {
        // SETUP token received, do the needful operations
        .
        .
        .
        // SETUP handling completed, enable Setup token and packet processing:
        PLIB_USB_PacketTransferEnable(MY_USB_INSTANCE);
    }
  </code>

  Remarks:
    Not valid when USB is Host.
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsPacketTransfer in your 
	application to determine whether this feature is available.		
*/

bool PLIB_USB_PacketTransferIsDisabled ( USB_MODULE_ID index );

// *****************************************************************************
/* Function:
    bool PLIB_USB_SE0InProgress( USB_MODULE_ID index )

  Summary:
    Returns whether a single-ended zero event is in progress.

  Description:
    This function returns whether a single-ended zero event is in progress.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    - true  - A single ended zero event (SE0) is occurring
    - false - A single-ended zero event (SE0) is not occurring

  Example:
  <code>
    if( PLIB_USB_SE0InProgress(MY_USB_INSTANCE) )
    {
        // handle the SE0 event
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsLiveSingleEndedZero in your 
	application to determine whether this feature is available.	
*/

bool PLIB_USB_SE0InProgress( USB_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library I/F Routines: External Transceiver Support
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USB_I2CInterfaceForExtModuleDisable ( USB_MODULE_ID index )

  Summary:
    Specifies external module(s) are controlled via dedicated pins.

  Description:
    Specifies that external module(s) are controlled via dedicated pins.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_I2CInterfaceForExtModuleDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_I2CInterfaceForExtModuleDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_I2CInterfaceForExtModuleEnable ( USB_MODULE_ID index )

  Summary:
    Specifies external module(s) are controlled via the I2C interface.

  Description:
    This function specifies that external module(s) are controlled via the I2C
    interface.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_I2CInterfaceForExtModuleEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_I2CInterfaceForExtModuleEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_TransceiverDisable ( USB_MODULE_ID index )

  Summary:
    Disables the on-chip transceiver

  Description:
    This function disables the on-chip transceiver and enables the interface to 
    the off-chip transceiver.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_TransceiverDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOnChipTransceiver in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_TransceiverDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_TransceiverEnable ( USB_MODULE_ID index )

  Summary:
    Enables the on-chip transceiver.

  Description:
    This function enables the on-chip transceiver. The interface to the off-chip 
    transceiver is disabled.

  Precondition:
    Use only before the USB module is enabled by calling PLIB_USB_Enable.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_TransceiverEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsOnChipTransceiver in your 
	application to determine whether this feature is available.		
*/

void PLIB_USB_TransceiverEnable ( USB_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: VBUS Support
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Function:
    void PLIB_USB_ExternalComparatorMode2Pin ( USB_MODULE_ID index )

  Summary: 
    Sets the 2-pin input configuration for VBUS comparators.

  Description:
    This function sets the 2-pin input configuration for VBUS Comparators.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_ExternalComparatorMode2Pin(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_ExternalComparatorMode2Pin ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_ExternalComparatorMode3Pin ( USB_MODULE_ID index )

  Summary: 
    Sets the 3-pin input configuration for VBUS Comparators.

  Description:
    This function sets the 3-pin input configuration for VBUS comparators.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_ExternalComparatorMode3Pin(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_ExternalComparatorMode3Pin ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_PWMCounterDisable( USB_MODULE_ID index );

  Summary:
    Disables the PWM counter used to generate the VBUS for the USB module.

  Description:
    This function disables the PWM counter used to generate the VBUS for the 
    USB module.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_PWMDisable(MY_USB_INSTANCE);
    PLIB_USB_PWMCounterDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_PWMCounterDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_PWMCounterEnable( USB_MODULE_ID index )

  Summary:
    Enables the PWM counter used to generate the VBUS for the USB module.

  Description:
    This function enables the PWM counter used to generate the VBUS for the 
    USB module.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_PWMEnable(MY_USB_INSTANCE);
    PLIB_USB_PWMCounterEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_PWMCounterEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_PWMDisable ( USB_MODULE_ID index )

  Summary:
    Disables the PWM Generator.

  Description:
    This function disables the PWM Generator. PWM output held in a reset state 
    defined by PLIB_USB_PWMPolarityActiveHigh or PLIB_USB_PWMPolarityActiveLow.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_PWMDisable (MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_PWMDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_PWMEnable ( USB_MODULE_ID index )

  Summary:
    Enables the PWM Generator.

  Description:
    This function enables the PWM Generator.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_PWMEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_PWMEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_PWMPolarityActiveHigh ( USB_MODULE_ID index )

  Summary:
    Sets the PWM output to active-low and resets high.

  Description:
    This function sets the PWM output to active-low and resets high.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_PWMPolaritiy (MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_PWMPolarityActiveHigh ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_PWMPolaritiyActiveLow ( USB_MODULE_ID index )

  Summary:
    Sets the PWM output to active-high and resets low.

  Description:
    This function sets the PWM output to active-high and resets low.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_PWMPolaritiyActiveLow ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_VBoostDisable ( USB_MODULE_ID index )

  Summary:
    Disables the On-Chip 5V Boost Regulator Circuit Disabled bit.

  Description:
    This function disables the On-Chip 5V Boost Regulator Circuit Disabled bit.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_VBoostDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_VBoostDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_VBoostEnable ( USB_MODULE_ID index )

  Summary: 
    Enables the On-Chip 5V Boost Regulator Circuit Enabled bit.

  Description:
    This function enables the On-Chip 5V Boost Regulator Circuit Enabled bit.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_VBoostEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_VBoostEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_VBUSComparatorDisable ( USB_MODULE_ID index )

  Summary:
    Disables the on-chip VBUS Comparator.

  Description: 
    This function disables the on-chip VBUS Comparator.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_VBUSComparatorDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_VBUSComparatorDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_VBUSComparatorEnable ( USB_MODULE_ID index )

  Summary:
    Enables the on-chip VBUS Comparator.

  Description:
    This function enables the on-chip VBUS Comparator.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_VBUSComparatorEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_VBUSComparatorEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_VBUSPullUpDisable ( USB_MODULE_ID index )

  Summary:
    Disables the pull-up on the VBUS pin.

  Description:
    This function disables the pull-up on the VBUS pin.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_VBUSPullUpDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_VBUSPullUpDisable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_VBUSPullUpEnable ( USB_MODULE_ID index )

  Summary: 
    Enables the pull-up on the VBUS pin.

  Description: 
    This function enables the pull-up on the VBUS pin.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_VBUSPullUpEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet to determine whether this feature is available on your device.
*/

void PLIB_USB_VBUSPullUpEnable ( USB_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Functions: Test Support
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USB_EyePatternEnable ( USB_MODULE_ID index )

  Summary:
    Enables USB eye pattern test.

  Description:
    This function enables the USB eye pattern test.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EyePatternEnable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEyePattern in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EyePatternEnable ( USB_MODULE_ID index );


// *****************************************************************************
/* Function:
    void PLIB_USB_EyePatternDisable ( USB_MODULE_ID index )

  Summary:
    Disables the USB eye pattern test.

  Description:
    This function disables the USB eye pattern test.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_EyePatternDisable(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEyePattern in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EyePatternDisable ( USB_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_USB_PingPongReset ( USB_MODULE_ID index )

  Summary:
    Resets the USB peripheral internal Ping-Pong indicator to point to even 
    buffers.

  Description:
    This function resets the USB peripheral internal Ping-Pong indicator to point 
    to Even buffers.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USB_PingPongReset(MY_USB_INSTANCE);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBufferFreeze in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_PingPongReset (USB_MODULE_ID index); 

// *****************************************************************************
/* Function:
    void PLIB_USB_AllInterruptEnable(USB_MODULE_ID index, USB_INTERRUPTS usbInterruptsFlag, 
        USB_ERROR_INTERRUPTS usbErrorInterruptsFlag, USB_OTG_INTERRUPTS otgInterruptFlag);  

  Summary:
    Configures the USB peripheral general interrupts, error interrupts and
    OTG interrupts.

  Description:
    This function configures the USB peripheral general interrupts, error 
    interrupts and OTG interrupts.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    usbInterruptsFlag - General interrupts to be configured
    usbErrorInterruptsFlag - USB Error interrupts to be configured
    otgInterruptFlag - OTG interrupts to be configured

  Returns:
    None.

  Example:
  <code>
    // This code snippet disables all OTG interrupts, disables
    // the SOF interrupt and enables all error interrupts.
    USB_OTG_INTERRUPTS otgInterruptEnables = ~USB_OTG_INT_ALL ;
    USB_INTERRUPTS generalInterruptEnables = USB_INT_ALL & ~USB_INT_SOF ;
    USB_ERROR_INTERRUPTS errorInterruptEnables = USB_ERR_INT_ALL ; 
   
    PLIB_USB_AllInterruptEnable(USB_MODULE_ID index, USB_INTERRUPTS usbInterruptsFlag, 
        USB_ERROR_INTERRUPTS usbErrorInterruptsFlag,
        USB_OTG_INTERRUPTS otgInterruptFlag);  
  
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsALL_Interrupt in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_AllInterruptEnable(USB_MODULE_ID index, USB_INTERRUPTS usbInterruptsFlag, 
        USB_ERROR_INTERRUPTS usbErrorInterruptsFlag,
        USB_OTG_INTERRUPTS otgInterruptFlag);  

// *****************************************************************************
/* Function:
    USB_ERROR_INTERRUPTS PLIB_USB_ErrorInterruptFlagAllGet(USB_MODULE_ID index);

  Summary:
    Returns a logically ORed bit map of active error interrupt flags.

  Description:
    This function returns a logically ORed bit map of active error interrupt 
    flags.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    Returns a logically ORed bit map of active error interrupt flags.

  Example:
  <code>
    USB_ERROR_INTERRUPTS errorInterruptEnables; 

    errorInterruptEnables = PLIB_USB_ErrorInterruptFlagAllGet(MY_USB_INSTANCE);
    if(errorInterruptEnables | USB_ERR_INT_DEVICE_EOF_ERROR)
    {
        // End of frame error occurred.
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsERR_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

USB_ERROR_INTERRUPTS PLIB_USB_ErrorInterruptFlagAllGet(USB_MODULE_ID index);

// *****************************************************************************
/* Function:
    USB_INTERRUPTS PLIB_USB_InterruptFlagAllGet(USB_MODULE_ID index);

  Summary:
    Returns a logically ORed bit map of active general USB interrupt flags.

  Description:
    This function returns a logically ORed bit map of active general USB interrupt
    flags.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    Returns a logically ORed bit map of active general USB interrupt flags.

  Example:
  <code>
    USB_INTERRUPTS interruptEnables; 

    interruptEnables = PLIB_USB_InterruptFlagAllGet(MY_USB_INSTANCE);
    if(interruptEnables | USB_INT_DEVICE_RESET)
    {
        // Device received reset signaling
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsGEN_InterruptStatus in your 
	application to determine whether this feature is available.	
*/

USB_INTERRUPTS PLIB_USB_InterruptFlagAllGet(USB_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USB_LastTransactionDetailsGet(USB_MODULE_ID index, 
        USB_BUFFER_DIRECTION * direction, 
        USB_PING_PONG_STATE * pingpong,
        uint8_t * endpoint);

  Summary:
    Returns the details of the last completed transaction.

  Description:
    This function returns the details of the last completed transaction.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    direction      - Return value contains direction of the last transfer
    pingpong       - Return value contains Ping-Pong indication of the
                     the last transfer
    endpoint       - Return value contains the endpoint which processed the
                     last transfer

  Returns:
    None.

  Example:
  <code>
    USB_BUFFER_DIRECTION direction; 
    USB_PING_PONG_STATE pingpong;
    uint8_t endpoint;

    interruptEnables = PLIB_USB_InterruptFlagAllGet(MY_USB_INSTANCE);
    if(interruptEnables | USB_INT_TOKEN_DONE)
    {
        // Find out details of the token
        PLIB_USB_LastTransactionDetailsGet(MY_USB_INSTANCE, &direction, 
                                        &pingpong, &endpoint);
    }
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsLastTransactionDetails in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_LastTransactionDetailsGet(USB_MODULE_ID index, 
        USB_BUFFER_DIRECTION * direction, 
        USB_PING_PONG_STATE * pingpong,
        uint8_t * endpoint);

// *****************************************************************************
/* Function:
    void PLIB_USB_BufferAllCancelReleaseToUSB(USB_MODULE_ID index, 
        void * pBDT,USB_PING_PONG_MODE ppMode, int nEndpoints);
  
  Summary:
    Cancels all endpoint buffer releases to the USB module and hands over the 
    buffer to the CPU.

  Description:
    This function cancels all endpoint buffer releases to the USB module and hands 
    over the buffer to the CPU.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    pBDT           - Pointer to the Buffer Descriptor Table
    ppMode         - Buffer Descriptor Table Ping-Pong mode
    nEndpoints     - Number of endpoints in the Buffer-Descriptor table

  Returns:
    None.

  Example:
  <code>
    
    //Cancel all buffer releases to USB.
    //BDT has 3 Endpoints.

    PLIB_USB_BufferAllCancelReleaseToUSB(MY_USB_INSTANCE, pBDT, 
                                    USB_PING_PONG_NO_PING_PONG, 3);
  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/
void PLIB_USB_BufferAllCancelReleaseToUSB(USB_MODULE_ID index, 
        void * pBDT,USB_PING_PONG_MODE ppMode, int nEndpoints);

// *****************************************************************************
/* Function:
    void PLIB_USB_EPnRxEnable(USB_MODULE_ID index, uint8_t endpoint);
  
  Summary:
    Enables an endpoint to process IN tokens.

  Description:
    This function enables an endpoint to process IN tokens.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    endpoint       - Endpoint to be affected

  Returns:
    None.

  Example:
  <code>
    
    //Provision endpoint 3 to process IN Token 
    PLIB_USB_EPnRxEnable(MY_USB_INSTANCE, 3);

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/
void PLIB_USB_EPnRxEnable(USB_MODULE_ID index, uint8_t endpoint);

// *****************************************************************************
/* Function:
    void PLIB_USB_EPnRxDisable(USB_MODULE_ID index, uint8_t endpoint);
  
  Summary:
    Disables an endpoint's ability to process IN tokens.

  Description:
    This function disables an endpoint's ability to process IN tokens.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    endpoint       - Endpoint to be affected

  Returns:
    None.

  Example:
  <code>
    
    //De-provision endpoint 3 to process IN Token 
    PLIB_USB_EPnRxDisable(MY_USB_INSTANCE, 3);

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnRxDisable(USB_MODULE_ID index, uint8_t endpoint);

// *****************************************************************************
/* Function:
    void PLIB_USB_EPnTxEnable(USB_MODULE_ID index, uint8_t endpoint);
  
  Summary:
    Enables an endpoint to process OUT tokens.

  Description:
    This function enables an endpoint to process OUT tokens.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    endpoint       - Endpoint to be affected

  Returns:
    None.

  Example:
  <code>
    
    //Provision endpoint 3 to process OUT Token 
    PLIB_USB_EPnTxEnable(MY_USB_INSTANCE, 3);

  </code>

  Remarks:
    None.
*/
void PLIB_USB_EPnTxEnable(USB_MODULE_ID index, uint8_t endpoint);

// *****************************************************************************
/* Function:
    void PLIB_USB_EPnTxDisable(USB_MODULE_ID index, uint8_t endpoint);
  
  Summary:
    Disables an endpoint's ability to process OUT tokens.

  Description:
    This function disables an endpoint's ability to process OUT tokens.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    endpoint       - Endpoint to be affected

  Returns:
    None.

  Example:
  <code>
    
    //De-provision endpoint 3 to process OUT Token 
    PLIB_USB_EPnTxDisable(MY_USB_INSTANCE, 3);

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsEPnRxEnable in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_EPnTxDisable(USB_MODULE_ID index, uint8_t endpoint);

// *****************************************************************************
/* Function:
    void PLIB_USB_BufferEP0RxStatusInitialize ( USB_MODULE_ID index,
                                            void* pBDT,
                                            USB_PING_PONG_MODE ppMode,
                                            USB_BUFFER_PING_PONG pingpong,
                                            uint16_t bufferByteCount );

  Summary:
    Initializes the Endpoint 0 RX endpoint buffer descriptors.

  Description:
    This function initializes the Endpoint 0 RX endpoint buffer descriptors. This 
    function will clear the Endpoint 0 RX Buffer Descriptor status field, load the 
    endpoint size and release the buffer to the USB module.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    pBDT           - Pointer to Buffer Descriptor Table
    pingpong       - Ping-Pong mode
    bufferByteCount - size of the EP0 RX buffer in bytes


  Returns:
    None.

  Example:
  <code>
    
    //Initialize EP0 RX even buffer descriptor and release back to 
    //USB. Buffer size is 64

    PLIB_USB_BufferEP0RxStatusInitialize ( MY_USB_INSTANCE, pBDT,
        USB_PING_PONG_NO_PING_PONG, USB_BUFFER_EVEN, 64);

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsBDTFunctions in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_BufferEP0RxStatusInitialize ( USB_MODULE_ID index,
                                            void* pBDT,
                                            USB_PING_PONG_MODE ppMode,
                                            USB_BUFFER_PING_PONG pingpong,
                                            uint16_t bufferByteCount );

// *****************************************************************************
/* Function:
    void PLIB_USB_BufferClearAllDTSEnable( USB_MODULE_ID index,
                                           void * pBDT,
                                           USB_PING_PONG_MODE ppMode,
                                           uint8_t epValue,
                                           USB_BUFFER_DIRECTION bufferDirection);

  Summary:
    Clears the endpoint descriptor entry and enables data toggle synchronization. 

  Description:
    This function clears the endpoint descriptor entry and enables data toggle 
    synchronization.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest
    pBDT           - pointer to Buffer Descriptor Table
    pingpong       - Ping-Pong mode.
    epvalue        - Endpoint to be be affected
    bufferDirection - Endpoint direction

  Returns:
    None.

  Example:
  <code>
    
    //Clear endpoint 6 buffer descriptor transmit entry and
    //enable data toggle synchronization. 

    PLIB_USB_BufferClearAllDTSEnable ( MY_USB_INSTANCE, pBDT,
        USB_PING_PONG_NO_PING_PONG, 6, USB_BUFFER_TX);

  </code>

  Remarks:
    None.
*/

void PLIB_USB_BufferClearAllDTSEnable( USB_MODULE_ID index,
                                           void * pBDT,
                                           USB_PING_PONG_MODE ppMode,
                                           uint8_t epValue,
                                           USB_BUFFER_DIRECTION bufferDirection);

// *****************************************************************************
/* Function:
    void PLIB_USB_UOEMonitorEnable( USB_MODULE_ID index );
  
  Summary:
    Enables the OE signal output.

  Description:
    This function enables the OE signal output.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    
    //Enable the OE output.
    PLIB_USB_UOEMonitorEnable(MY_USB_INSTANCE);

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsUOEMonitor in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_UOEMonitorEnable( USB_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_USB_UOEMonitorDisable( USB_MODULE_ID index );
  
  Summary:
    Disables the OE signal output.

  Description:
    This function disables the OE signal output.

  Precondition:
    None.

  Parameters:
    index          - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    
    //Disable the OE output.
    PLIB_USB_UOEMonitorDisable(MY_USB_INSTANCE);

  </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific 
	device data sheet to determine availability or use PLIB_USB_ExistsUOEMonitor in your 
	application to determine whether this feature is available.	
*/

void PLIB_USB_UOEMonitorDisable( USB_MODULE_ID index );

// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Exists API Functions
// *****************************************************************************
// *****************************************************************************
/* The following functions indicate the existence of the features on the device.
*/

//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_InterruptStatus( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_InterruptStatus feature exists on the USB module.

  Description:
    This function identifies whether the OTG_InterruptStatus feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_OTG_InterruptFlagSet
    - PLIB_USB_OTG_InterruptFlagClear
    - PLIB_USB_OTG_InterruptFlagGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_InterruptStatus feature is supported on the device
    - false  - The OTG_InterruptStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_InterruptStatus( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_Interrupt( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_Interrupt feature exists on the USB module.

  Description:
    This function identifies whether the OTG_Interrupt feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_OTG_InterruptEnable
    - PLIB_USB_OTG_InterruptDisable
    - PLIB_USB_OTG_InterruptIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_Interrupt feature is supported on the device
    - false  - The OTG_Interrupt feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_Interrupt( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_IDPinState( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_IDPinState feature exists on the USB module.

  Description:
    This function identifies whether the OTG_IDPinState feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_OTG_IDPinStateIsTypeA

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_IDPinState feature is supported on the device
    - false  - The OTG_IDPinState feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_IDPinState( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_LineState( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_LineState feature exists on the USB module.

  Description:
    This function identifies whether the OTG_LineState feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_OTG_LineStateIsStable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_LineState feature is supported on the device
    - false  - The OTG_LineState feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_LineState( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_SessionValid( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_SessionValid feature exists on the USB module.

  Description:
    This function identifies whether the OTG_SessionValid feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_OTG_SessionValid

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_SessionValid feature is supported on the device
    - false  - The OTG_SessionValid feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_SessionValid( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_BSessionEnd( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_BSessionEnd feature exists on the USB module.

  Description:
    This function identifies whether the OTG_BSessionEnd feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_OTG_BSessionHasEnded

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_BSessionEnd feature is supported on the device
    - false  - The OTG_BSessionEnd feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_BSessionEnd( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_ASessionValid( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_ASessionValid feature exists on the USB module.

  Description:
    This function identifies whether the OTG_ASessionValid feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_OTG_VBusValid

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_ASessionValid feature is supported on the device
    - false  - The OTG_ASessionValid feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_ASessionValid( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_PullUpPullDown( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_PullUpPullDown feature exists on the USB module.

  Description:
    This function identifies whether the OTG_PullUpPullDown feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_OTG_PullUpPullDownSetup

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_PullUpPullDown feature is supported on the device
    - false  - The OTG_PullUpPullDown feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_PullUpPullDown( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_VbusPowerOnOff( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_VbusPowerOnOff feature exists on the USB module.

  Description:
    This function identifies whether the OTG_VbusPowerOnOff feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_OTG_VBusPowerOff
    - PLIB_USB_OTG_VBusPowerOn

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_VbusPowerOnOff feature is supported on the device
    - false  - The OTG_VbusPowerOnOff feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_VbusPowerOnOff( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_VbusCharge( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_VbusCharge feature exists on the USB module.

  Description:
    This function identifies whether the OTG_VbusCharge feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_OTG_VBusChargeEnable
    - PLIB_USB_OTG_VBusChargeDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_VbusCharge feature is supported on the device
    - false  - The OTG_VbusCharge feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_VbusCharge( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOTG_VbusDischarge( USB_MODULE_ID index )

  Summary:
    Identifies whether the OTG_VbusDischarge feature exists on the USB module.

  Description:
    This function identifies whether the OTG_VbusDischarge feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_OTG_VBusDischargeEnable
    - PLIB_USB_OTG_VBusDischargeDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OTG_VbusDischarge feature is supported on the device
    - false  - The OTG_VbusDischarge feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOTG_VbusDischarge( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsActivityPending( USB_MODULE_ID index )

  Summary:
    Identifies whether the ActivityPending feature exists on the USB module.

  Description:
    This function identifies whether the ActivityPending feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_ActivityPending

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ActivityPending feature is supported on the device
    - false  - The ActivityPending feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsActivityPending( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsSleepEntryGuard( USB_MODULE_ID index )

  Summary:
    Identifies whether the SleepEntryGuard feature exists on the USB module.

  Description:
    This function identifies whether the SleepEntryGuard feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_SleepGuardEnable
    - PLIB_USB_SleepGuardDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The SleepEntryGuard feature is supported on the device
    - false  - The SleepEntryGuard feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsSleepEntryGuard( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsModuleBusy( USB_MODULE_ID index )

  Summary:
    Identifies whether the ModuleBusy feature exists on the USB module.

  Description:
    This function identifies whether the ModuleBusy feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_ModuleIsBusy

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ModuleBusy feature is supported on the device
    - false  - The ModuleBusy feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsModuleBusy( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsSuspend( USB_MODULE_ID index )

  Summary:
    Identifies whether the Suspend feature exists on the USB module.

  Description:
    This function identifies whether the Suspend feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_SuspendEnable
    - PLIB_USB_SuspendDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Suspend feature is supported on the device
    - false  - The Suspend feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsSuspend( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsModulePower( USB_MODULE_ID index )

  Summary:
    Identifies whether the ModulePower feature exists on the USB module.

  Description:
    This function identifies whether the ModulePower feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_Enable
    - PLIB_USB_Disable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ModulePower feature is supported on the device
    - false  - The ModulePower feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsModulePower( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsGEN_InterruptStatus( USB_MODULE_ID index )

  Summary:
    Identifies whether the GEN_InterruptStatus feature exists on the USB module.

  Description:
    This function identifies whether the GEN_InterruptStatus feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_InterruptFlagSet
    - PLIB_USB_InterruptFlagClear
    - PLIB_USB_InterruptFlagGet
    - PLIB_USB_InterruptFlagAllGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GEN_InterruptStatus feature is supported on the device
    - false  - The GEN_InterruptStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsGEN_InterruptStatus( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsGEN_Interrupt( USB_MODULE_ID index )

  Summary:
    Identifies whether the GEN_Interrupt feature exists on the USB module.

  Description:
    This function identifies whether the GEN_Interrupt feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_InterruptEnable
    - PLIB_USB_InterruptDisable
    - PLIB_USB_InterruptIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GEN_Interrupt feature is supported on the device
    - false  - The GEN_Interrupt feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsGEN_Interrupt( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsALL_Interrupt( USB_MODULE_ID index )

  Summary:
    Identifies whether the ALL_Interrupt feature exists on the USB module.

  Description:
    This function identifies whether the ALL_Interrupt feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_AllInterruptEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ALL_Interrupt feature is supported on the device
    - false  - The ALL_Interrupt feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsALL_Interrupt( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsERR_InterruptStatus( USB_MODULE_ID index )

  Summary:
    Identifies whether the ERR_InterruptStatus feature exists on the USB module.

  Description:
    This function identifies whether the ERR_InterruptStatus feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_ErrorInterruptFlagSet
    - PLIB_USB_ErrorInterruptFlagClear
    - PLIB_USB_ErrorInterruptFlagGet
    - PLIB_USB_ErrorInterruptFlagAllGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ERR_InterruptStatus feature is supported on the device
    - false  - The ERR_InterruptStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsERR_InterruptStatus( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsERR_Interrupt( USB_MODULE_ID index )

  Summary:
    Identifies whether the ERR_Interrupt feature exists on the USB module.

  Description:
    This function identifies whether the ERR_Interrupt feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_ErrorInterruptEnable
    - PLIB_USB_ErrorInterruptDisable
    - PLIB_USB_ErrorInterruptIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ERR_Interrupt feature is supported on the device
    - false  - The ERR_Interrupt feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsERR_Interrupt( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsLastEndpoint( USB_MODULE_ID index )

  Summary:
    Identifies whether the LastEndpoint feature exists on the USB module.

  Description:
    This function identifies whether the LastEndpoint feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_LastTransactionEndPtGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LastEndpoint feature is supported on the device
    - false  - The LastEndpoint feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsLastEndpoint( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsLastDirection( USB_MODULE_ID index )

  Summary:
    Identifies whether the LastDirection feature exists on the USB module.

  Description:
    This function identifies whether the LastDirection feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_LastTransactionDirectionGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LastDirection feature is supported on the device
    - false  - The LastDirection feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsLastDirection( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsLastPingPong( USB_MODULE_ID index )

  Summary:
    Identifies whether the LastPingPong feature exists on the USB module.

  Description:
    This function identifies whether the LastPingPong feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_LastTransactionPingPongStateGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LastPingPong feature is supported on the device
    - false  - The LastPingPong feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsLastPingPong( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsLastTransactionDetails( USB_MODULE_ID index )

  Summary:
    Identifies whether the LastTransactionDetails feature exists on the USB module.

  Description:
    This function identifies whether the LastTransactionDetails feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_LastTransactionDetailsGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LastTransactionDetails feature is supported on the device
    - false  - The LastTransactionDetails feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsLastTransactionDetails( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsLiveJState( USB_MODULE_ID index )

  Summary:
    Identifies whether the LiveJState feature exists on the USB module.

  Description:
    This function identifies whether the LiveJState feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_JStateIsActive

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LiveJState feature is supported on the device
    - false  - The LiveJState feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsLiveJState( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsLiveSingleEndedZero( USB_MODULE_ID index )

  Summary:
    Identifies whether the LiveSingleEndedZero feature exists on the USB module.

  Description:
    This function identifies whether the LiveSingleEndedZero feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_SE0InProgress

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The LiveSingleEndedZero feature is supported on the device
    - false  - The LiveSingleEndedZero feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsLiveSingleEndedZero( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsPacketTransfer( USB_MODULE_ID index )

  Summary:
    Identifies whether the PacketTransfer feature exists on the USB module.

  Description:
    This function identifies whether the PacketTransfer feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_PacketTransferIsDisabled
    - PLIB_USB_PacketTransferEnable
    - PLIB_USB_PacketTransferDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PacketTransfer feature is supported on the device
    - false  - The PacketTransfer feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsPacketTransfer( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsHostBusyWithToken( USB_MODULE_ID index )

  Summary:
    Identifies whether the HostBusyWithToken feature exists on the USB module.

  Description:
    This function identifies whether the HostBusyWithToken feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_IsBusyWithToken

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HostBusyWithToken feature is supported on the device
    - false  - The HostBusyWithToken feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsHostBusyWithToken( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsHostGeneratesReset( USB_MODULE_ID index )

  Summary:
    Identifies whether the HostGeneratesReset feature exists on the USB module.

  Description:
    This function identifies whether the HostGeneratesReset feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_ResetSignalEnable
    - PLIB_USB_ResetSignalDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The HostGeneratesReset feature is supported on the device
    - false  - The HostGeneratesReset feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsHostGeneratesReset( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOpModeSelect( USB_MODULE_ID index )

  Summary:
    Identifies whether the OpModeSelect feature exists on the USB module.

  Description:
    This function identifies whether the OpModeSelect feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_OperatingModeSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OpModeSelect feature is supported on the device
    - false  - The OpModeSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOpModeSelect( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsResumeSignaling( USB_MODULE_ID index )

  Summary:
    Identifies whether the ResumeSignaling feature exists on the USB module.

  Description:
    This function identifies whether the ResumeSignaling feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_ResumeSignalingEnable
    - PLIB_USB_ResumeSignalingDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ResumeSignaling feature is supported on the device
    - false  - The ResumeSignaling feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsResumeSignaling( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsBufferFreeze( USB_MODULE_ID index )

  Summary:
    Identifies whether the BufferFreeze feature exists on the USB module.

  Description:
    This function identifies whether the BufferFreeze feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_PingPongFreeze
    - PLIB_USB_PingPongUnfreeze
    - PLIB_USB_PingPongReset

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The BufferFreeze feature is supported on the device
    - false  - The BufferFreeze feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsBufferFreeze( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsStartOfFrames( USB_MODULE_ID index )

  Summary:
    Identifies whether the StartOfFrames feature exists on the USB module.

  Description:
    This function identifies whether the StartOfFrames feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_SOFEnable
    - PLIB_USB_SOFDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The StartOfFrames feature is supported on the device
    - false  - The StartOfFrames feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsStartOfFrames( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsNextTokenSpeed( USB_MODULE_ID index )

  Summary:
    Identifies whether the NextTokenSpeed feature exists on the USB module.

  Description:
    This function identifies whether the NextTokenSpeed feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_TokenSpeedSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The NextTokenSpeed feature is supported on the device
    - false  - The NextTokenSpeed feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsNextTokenSpeed( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsDeviceAddress( USB_MODULE_ID index )

  Summary:
    Identifies whether the DeviceAddress feature exists on the USB module.

  Description:
    This function identifies whether the DeviceAddress feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_DeviceAddressSet
    - PLIB_USB_DeviceAddressGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DeviceAddress feature is supported on the device
    - false  - The DeviceAddress feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsDeviceAddress( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsFrameNumber( USB_MODULE_ID index )

  Summary:
    Identifies whether the FrameNumber feature exists on the USB module.

  Description:
    This function identifies whether the FrameNumber feature is available
    on the USB module. When this function returns true, this function is
    supported on the device:
    - PLIB_USB_FrameNumberGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The FrameNumber feature is supported on the device
    - false  - The FrameNumber feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsFrameNumber( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsTokenPID( USB_MODULE_ID index )

  Summary:
    Identifies whether the TokenPID feature exists on the USB module.

  Description:
    This function identifies whether the TokenPID feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_TokenPIDGet
    - PLIB_USB_TokenPIDSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TokenPID feature is supported on the device
    - false  - The TokenPID feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsTokenPID( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsTokenEP( USB_MODULE_ID index )

  Summary:
    Identifies whether the TokenEP feature exists on the USB module.

  Description:
    This function identifies whether the TokenEP feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_TokenEPGet
    - PLIB_USB_TokenEPSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TokenEP feature is supported on the device
    - false  - The TokenEP feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsTokenEP( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsSOFThreshold( USB_MODULE_ID index )

  Summary:
    Identifies whether the SOFThreshold feature exists on the USB module.

  Description:
    This function identifies whether the SOFThreshold feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_SOFThresholdGet
    - PLIB_USB_SOFThresholdSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The SOFThreshold feature is supported on the device
    - false  - The SOFThreshold feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsSOFThreshold( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsBDTBaseAddress( USB_MODULE_ID index )

  Summary:
    Identifies whether the BDTBaseAddress feature exists on the USB module.

  Description:
    This function identifies whether the BDTBaseAddress feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_BDTBaseAddressGet
    - PLIB_USB_BDTBaseAddressSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The BDTBaseAddress feature is supported on the device
    - false  - The BDTBaseAddress feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsBDTBaseAddress( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsEyePattern( USB_MODULE_ID index )

  Summary:
    Identifies whether the EyePattern feature exists on the USB module.

  Description:
    This function identifies whether the EyePattern feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_EyePatternDisable
    - PLIB_USB_EyePatternEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EyePattern feature is supported on the device
    - false  - The EyePattern feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsEyePattern( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsStopInIdle( USB_MODULE_ID index )

  Summary:
    Identifies whether the StopInIdle feature exists on the USB module.

  Description:
    This function identifies whether the StopInIdle feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_StopInIdleEnable
    - PLIB_USB_StopInIdleDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The StopInIdle feature is supported on the device
    - false  - The StopInIdle feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsStopInIdle( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsAutomaticSuspend( USB_MODULE_ID index )

  Summary:
    Identifies whether the AutomaticSuspend feature exists on the USB module.

  Description:
    This function identifies whether the AutomaticSuspend feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_AutoSuspendDisable
    - PLIB_USB_AutoSuspendEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AutomaticSuspend feature is supported on the device
    - false  - The AutomaticSuspend feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsAutomaticSuspend( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsPingPongMode( USB_MODULE_ID index )

  Summary:
    Identifies whether the PingPongMode feature exists on the USB module.

  Description:
    This function identifies whether the PingPongMode feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_PingPongModeSelect
    - PLIB_USB_PingPongModeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PingPongMode feature is supported on the device
    - false  - The PingPongMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsPingPongMode( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsUOEMonitor( USB_MODULE_ID index )

  Summary:
    Identifies whether the UOEMonitor feature exists on the USB module.

  Description:
    This function identifies whether the UOEMonitor feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_UOEMonitorEnable
    - PLIB_USB_UOEMonitorDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The UOEMonitor feature is supported on the device
    - false  - The UOEMonitor feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsUOEMonitor( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOnChipPullup( USB_MODULE_ID index )

  Summary:
    Identifies whether the OnChipPullup feature exists on the USB module.

  Description:
    This function identifies whether the OnChipPullup feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_OnChipPullUpDisable
    - PLIB_USB_OnChipPullUpEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OnChipPullup feature is supported on the device
    - false  - The OnChipPullup feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOnChipPullup( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsOnChipTransceiver( USB_MODULE_ID index )

  Summary:
    Identifies whether the OnChipTransceiver feature exists on the USB module.

  Description:
    This function identifies whether the OnChipTransceiver feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_TransceiverEnable
    - PLIB_USB_TransceiverDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OnChipTransceiver feature is supported on the device
    - false  - The OnChipTransceiver feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsOnChipTransceiver( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsSpeedControl( USB_MODULE_ID index )

  Summary:
    Identifies whether the SpeedControl feature exists on the USB module.

  Description:
    This function identifies whether the SpeedControl feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_FullSpeedEnable
    - PLIB_USB_FullSpeedDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The SpeedControl feature is supported on the device
    - false  - The SpeedControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsSpeedControl( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsEP0LowSpeedConnect( USB_MODULE_ID index )

  Summary:
    Identifies whether the EP0LowSpeedConnect feature exists on the USB module.

  Description:
    This function identifies whether the EP0LowSpeedConnect feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_EP0LSDirectConnectEnable
    - PLIB_USB_EP0LSDirectConnectDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EP0LowSpeedConnect feature is supported on the device
    - false  - The EP0LowSpeedConnect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsEP0LowSpeedConnect( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsEP0NAKRetry( USB_MODULE_ID index )

  Summary:
    Identifies whether the EP0NAKRetry feature exists on the USB module.

  Description:
    This function identifies whether the EP0NAKRetry feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_EP0NakRetryEnable
    - PLIB_USB_EP0NakRetryDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EP0NAKRetry feature is supported on the device
    - false  - The EP0NAKRetry feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsEP0NAKRetry( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsEPnTxRx( USB_MODULE_ID index )

  Summary:
    Identifies whether the EPnTxRx feature exists on the USB module.

  Description:
    This function identifies whether the EPnTxRx feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_EPnTxSelect
    - PLIB_USB_EPnRxSelect
    - PLIB_USB_EPnTxRxSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EPnTxRx feature is supported on the device
    - false  - The EPnTxRx feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsEPnTxRx( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsEPnRxEnable( USB_MODULE_ID index )

  Summary:
    Identifies whether the EPnRxEnableEnhanced feature exists on the USB module.

  Description:
    This function identifies whether the EPnRxEnableEnhanced feature is available
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_EPnRxEnable
    - PLIB_USB_EPnRxDisable
    - PLIB_USB_EPnTxEnable
    - PLIB_USB_EPnTxDisable
    - PLIB_USB_EPnHandshakeEnable
    - PLIB_USB_EPnHandshakeDisable
    - PLIB_USB_EPnControlTransferEnable
    - PLIB_USB_EPnControlTransferDisable
    - PLIB_USB_EPnIsStalled
    - PLIB_USB_EPnStallClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EPnRxEnableEnhanced feature is supported on the device
    - false  - The EPnRxEnableEnhanced feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsEPnRxEnable( USB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_USB_ExistsBDTFunctions( USB_MODULE_ID index )

  Summary:
    Identifies whether the BDTFunctions feature exists on the USB module.

  Description:
    This function identifies whether the BDTFunctions feature is available 
    on the USB module. When this function returns true, these functions are 
    supported on the device:
    - PLIB_USB_BufferAddressGet
    - PLIB_USB_BufferAddressSet
    - PLIB_USB_BufferByteCountGet
    - PLIB_USB_BufferByteCountSet
    - PLIB_USB_BufferCancelReleaseToUSB
    - PLIB_USB_BufferAllCancelReleaseToUSB
    - PLIB_USB_BufferClearAll
    - PLIB_USB_BufferDataToggleGet
    - PLIB_USB_BufferDataToggleSelect
    - PLIB_USB_BufferDataToggleSyncEnable
    - PLIB_USB_BufferDataToggleSyncDisable
    - PLIB_USB_BufferIndexGet
    - PLIB_USB_BufferPIDBitsClear
    - PLIB_USB_BufferPIDGet
    - PLIB_USB_BufferReleasedToSW
    - PLIB_USB_BufferReleaseToUSB
    - PLIB_USB_BufferSchedule
    - PLIB_USB_BufferStallDisable
    - PLIB_USB_BufferStallEnable
    - PLIB_USB_BufferStallGet
    - PLIB_USB_BufferEP0RxStatusInitialize
    - PLIB_USB_BufferClearAllDTSEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The BDTFunctions feature is supported on the device
    - false  - The BDTFunctions feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_USB_ExistsBDTFunctions( USB_MODULE_ID index );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

// ****************************************************************************
// ****************************************************************************
// Section: Included Files (continued)
// ****************************************************************************
// ****************************************************************************
/*  The file included below  maps the interface definitions above to appropriate
    implementations defined in the implementation (imp) file(s).
*/

// DOM-IGNORE-BEGIN
//#include other_stuff.h
// DOM-IGNORE-END

// Note: usb_processor.h is not included here.  It is included above to
// provide processor-specific definitions of the Buffer Descriptor Table.

#endif//ndef _PLIB_USB_H

/*******************************************************************************
 End of File
*/
