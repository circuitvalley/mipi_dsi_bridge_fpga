 /*******************************************************************************
  USB HOST HID Keyboard local definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_hid_keyboard_local.h

  Summary:
    USB HOST HID Keyboard local definitions

  Description:
    This file describes the local HID Keyboard usage definitions.
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

#ifndef _USB_HOST_HID_KEYBOARD_LOCAL_H
#define _USB_HOST_HID_KEYBOARD_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "usb/usb_host_hid_keyboard.h"

#define _USB_HOST_HID_KEYBOARD_BUFFER_QUEUE_SIZE 15

// *****************************************************************************
/* USB HOST HID Keyboard Driver State

  Summary:
    USB HOST HID Keyboard Driver State.

  Description:
    This enumeration defines the possible task state of USB HOST HID Keyboard
    driver.

  Remarks:
    None.
*/

typedef enum
{
    USB_HOST_HID_KEYBOARD_DETACHED = 0,
    USB_HOST_HID_KEYBOARD_ATTACHED,
    USB_HOST_HID_KEYBOARD_REPORT_PROCESS
    
}USB_HOST_HID_KEYBOARD_STATE;

// *****************************************************************************
/* USB HOST HID Keyboard Driver buffer data structure

  Summary:
    USB HOST HID Keyboard Driver buffer information

  Description:
    This structure holds the keyboard buffer data structure. Structure gets
    updated on every INTERRUPT IN data from HID keyboard. 

  Remarks:
    None.
 */

typedef struct
{
    bool tobeDone;
    int8_t data[64];
    
} USB_HOST_HID_KEYBOARD_DATA_BUFFER;

// *****************************************************************************
/* USB HOST HID Keyboard Driver data structure

  Summary:
    USB HOST HID Keyboard Driver information

  Description:
    This structure holds all information on per HID Keyboard driver instance level.
    Contains information like driver state, buffer information, ping pong states.
    Structure is used as part of parsing once report is received from keyboard.

  Remarks:
    None.
 */

typedef struct
{
    bool inUse;
    uint8_t index;
    uint8_t counter;
    uint8_t outputReportID;
    USB_HOST_HID_KEYBOARD_DATA_BUFFER buffer[_USB_HOST_HID_KEYBOARD_BUFFER_QUEUE_SIZE];
    USB_HID_KEYBOARD_KEYPAD lastKeyCode[6];
    USB_HOST_HID_KEYBOARD_STATE state;
    USB_HOST_HID_OBJ_HANDLE handle;
    USB_HOST_HID_KEYBOARD_DATA appData;
    uint8_t outputReport;
    
} USB_HOST_HID_KEYBOARD_DATA_OBJ;


#endif

