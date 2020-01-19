/********************************************************************************
  USB HOST HID Keyboard Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_hid_keyboard.h

  Summary:
    USB Host HID Keyboard Driver Definition Header

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface between HID Keyboard
    driver and top level application.
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
#ifndef _USB_HOST_HID_KEYBOARD_H_
#define _USB_HOST_HID_KEYBOARD_H_


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host_hid.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
// *****************************************************************************
// Section: USB HID Keyboard Driver Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Host HID Keyboard Driver Result Minimum Constant

  Summary:
    USB Host HID Keyboard Driver Result Minimum Constant.

  Description:
    Constant identifying the USB Host HID Keyboard Driver Result Minimum Value.
    Constant is used in the USB_HOST_HID_KEYBOARD_RESULT enumeration.

  Remarks:
    None.
*/

#define USB_HOST_HID_KEYBOARD_RESULT_MIN -50

// *****************************************************************************
/* USB HOST HID Keyboard Driver Events

  Summary:
    Defines the possible USB HOST HID Keyboard Driver Events

  Description:
    This enumeration lists the possible Keyboard events that Keyboard driver can
    provide to application.
    Some of these events have event data associated with them.

  Remarks:
    None.
 */

typedef enum
{
    /* Keyboard has been attached */
    USB_HOST_HID_KEYBOARD_EVENT_ATTACH = 0,
    /* Keyboard has been detached */
    USB_HOST_HID_KEYBOARD_EVENT_DETACH,
    /* Keyboard IN Report data available */
    USB_HOST_HID_KEYBOARD_EVENT_REPORT_RECEIVED

} USB_HOST_HID_KEYBOARD_EVENT;

// *****************************************************************************
/* USB Host HID KEYBOARD Result

  Summary:
    USB Host HID Keyboard driver Results.

  Description:
    This enumeration defines the possible returns values of USB Host HID Keyboard
    driver API. A function may only return some of the values in this 
    enumeration. Refer to function description for details on which values will
    be returned.

  Remarks:
    None.
*/

typedef enum
{   
    /* An unknown failure occurred */
    USB_HOST_HID_KEYBOARD_RESULT_FAILURE = USB_HOST_HID_KEYBOARD_RESULT_MIN,
    
    /* Invalid or NULL parameter passed */
    USB_HOST_HID_KEYBOARD_RESULT_INVALID_PARAMETER,
    
    /* USB HOST Keyboard busy to accept this request */
    USB_HOST_HID_KEYBOARD_RESULT_REQUEST_BUSY,
    
    /* Indicates that the operation succeeded or the request was accepted and
       will be processed. */
    USB_HOST_HID_KEYBOARD_RESULT_SUCCESS = 0

} USB_HOST_HID_KEYBOARD_RESULT;

// *****************************************************************************
/* USB HOST HID Keyboard Driver instance Handle

  Summary:
    USB HOST HID Keyboard Driver instance Handle

  Description:
    This defines a USB Host HID Keyboard Driver Handle.

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_HID_KEYBOARD_HANDLE;

#define USB_HOST_HID_KEYBOARD_HANDLE_INVALID ((USB_HOST_HID_KEYBOARD_HANDLE)(-1))

// *****************************************************************************
/* USB Host HID Keyboard Non Modifier Keys Data Object

  Summary:
    Defines the USB Host HID Keyboard Non Modifier Keys Data Object.

  Description:
    Defines the USB Host HID Keyboard Non Modifier Keys Data Object.

  Remarks:
    None.
*/

typedef struct
{
    /* USB_HID_KEY_EVENT will determine if the key has been pressed or released.
     * On key press, event will be USB_HID_KEY_PRESS. On key release
     * USB_HID_KEY_RELEASE will be notified as event value. */
    USB_HID_KEY_EVENT  event;
    /* The usage value of the key for which this event is notified of. */
    USB_HID_KEYBOARD_KEYPAD keyCode;
    
    uint64_t sysCount;
    
} USB_HOST_HID_KEYBOARD_NON_MODIFIER_KEYS_DATA;


typedef struct
{
    uint8_t leftControl :1;
	uint8_t leftShift :1;
	uint8_t leftAlt :1;
	uint8_t leftGui :1;
	uint8_t rightControl :1;
	uint8_t rightShift :1;
	uint8_t rightAlt :1;
	uint8_t rightGui :1;
    
} USB_HID_KEYBOARD_MODIFIER_KEYS_DATA;

// *****************************************************************************
/* USB Host HID Keyboard Data Object

  Summary:
    Defines the USB Host HID Keyboard Data Object.

  Description:
    Defines the USB Host HID Keyboard Data Object.

  Remarks:
    None.
*/

typedef struct
{
    /* This holds the key state for all modifier Keys Data. On key press,
	 * event will be USB_HID_KEY_PRESS. On key release USB_HID_KEY_RELEASE
	 * will be notified as event value. */
	USB_HID_KEYBOARD_MODIFIER_KEYS_DATA modifierKeysData;
	/* This value determines how many instances of nonModifierKeysData[]
	 * is valid. */
    size_t nNonModifierKeysData;
	/* This holds all the non modifier keys state that has been changed
     * from the last time USB_HOST_HID_KEYBOARD_EVENT_REPORT_RECEIVED event
	 * was notified. If a non modifier key has been kept pressed
	 * continuously, that key will also be reported here. */
	USB_HOST_HID_KEYBOARD_NON_MODIFIER_KEYS_DATA nonModifierKeysData[6];

} USB_HOST_HID_KEYBOARD_DATA;

// *****************************************************************************
/* USB HOST Keyboard Driver Event Handler Function Pointer Type.

  Summary:
    USB HOST Keyboard Driver Event Handler Function Pointer Type.

  Description:
    This defines the USB HOST HID Keyboard Driver event handler function
    pointer type. Application must register a function of this type to
    receive HID mouse events. Registration should happen before USB BUS is
    enabled by the application.

  Remarks:
    None.
*/

typedef void (*USB_HOST_HID_KEYBOARD_EVENT_HANDLER)
(
    /* Unique Handle passed to the caller. This will be unique
       for each Keyboard interface attached
     */
    USB_HOST_HID_KEYBOARD_HANDLE handle,
    /* Associated event type */
    USB_HOST_HID_KEYBOARD_EVENT event,
	/* Associated event data. For USB_HOST_HID_KEYBOARD_EVENT_ATTACH
       and USB_HOST_HID_KEYBOARD_EVENT_DETACH this will be NULL.
       For USB_HOST_HID_KEYBOARD_EVENT_REPORT_RECEIVED event, this
	   will be USB_HOST_HID_KEYBOARD_DATA data type
	 */
	void * data
);

// *****************************************************************************
// *****************************************************************************
// Section: KEYBOARD Driver Function Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************


void _USB_HOST_HID_KEYBOARD_Task(USB_HOST_HID_OBJ_HANDLE handle);
void _USB_HOST_HID_KEYBOARD_EventHandler
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HOST_HID_EVENT event,
    void * eventData
);


// *****************************************************************************
/* Function:
    USB_HOST_HID_KEYBOARD_RESULT USB_HOST_HID_KEYBOARD_EventHandlerSet
    (
        USB_HOST_HID_KEYBOARD_EVENT_HANDLER appKeyboardEventHandler
    );

  Summary:
    This function registers application callback function with Keyboard driver

  Description:
    This function registers application callback function with Keyboard driver.
    Any subsequent Keyboard events is passed to the application by calling the
    registered application function.
    Function prototype should be of USB_HOST_HID_KEYBOARD_EVENT_HANDLER type.
    
  Precondition:
    This function should be called before USB BUS is enabled.

  Parameters:
    appKeyboardEventHandler  - Function pointer to the application function.

  Returns:
    Returns data structure of USB_HOST_HID_KEYBOARD_RESULT type.
      USB_HOST_HID_KEYBOARD_RESULT_INVALID_PARAMETER: Invalid Parameter
      USB_HOST_HID_KEYBOARD_RESULT_FAILURE: On failure
      USB_HOST_HID_KEYBOARD_RESULT_SUCCESS: On success
    
  Example:
    <code>
    // This code snippet shows an example of registering event handler
   
    </code>

  Remarks:
    This function should be called before USB BUS is enabled
*/

USB_HOST_HID_KEYBOARD_RESULT USB_HOST_HID_KEYBOARD_EventHandlerSet
(
    USB_HOST_HID_KEYBOARD_EVENT_HANDLER appKeyboardEventHandler
);


// *****************************************************************************
/* Function:
    USB_HOST_HID_KEYBOARD_RESULT USB_HOST_HID_KEYBOARD_ReportSend
    (
        USB_HOST_HID_KEYBOARD_HANDLE handle
        uint8_t outputReport
    );

  Summary:
    This function facilitates Keyboard application to send OUTPUT Report

  Description:
    This function facilitates Keyboard application to send OUTPUT Report.
    The state of the LEDs will be maintained by the application. 
    
  Precondition:
    This function should be called after
    USB_HOST_HID_KEYBOARD_EVENT_REPORT_RECEIVED and OUTPUT report needs to be
    send

  Parameters:
    handle  - Keyboard driver handle to application.
    outputReport - Output Report data

  Returns:
    Returns data structure of USB_HOST_HID_KEYBOARD_RESULT type.
      USB_HOST_HID_KEYBOARD_RESULT_INVALID_PARAMETER: Invalid Parameter
      USB_HOST_HID_RESULT_REQUEST_BUSY - Cannot accept request now
      USB_HOST_HID_KEYBOARD_RESULT_FAILURE: On failure
      USB_HOST_HID_KEYBOARD_RESULT_SUCCESS: On success
    
  Example:
    <code>
    // This code snippet shows an example of sending Report
   
    </code>

  Remarks:
    None
*/

USB_HOST_HID_KEYBOARD_RESULT USB_HOST_HID_KEYBOARD_ReportSend
(
    USB_HOST_HID_KEYBOARD_HANDLE handle,
    uint8_t outputReport
);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif

//DOM-IGNORE-END

#endif

/*********** End of file ***************************************/
