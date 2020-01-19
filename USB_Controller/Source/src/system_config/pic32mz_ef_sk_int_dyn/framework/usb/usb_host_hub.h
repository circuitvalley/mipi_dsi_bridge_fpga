/********************************************************************************
  USB Host Hub Client Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_hub.h

  Summary:
    USB Host Hub Client Driver Interface Header

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface to the USB HOST Hub
    Client Driver.
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
#ifndef _USB_HOST_HUB_H_
#define _USB_HOST_HUB_H_
//DOM-IGNORE-END

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// ****************************************************************************
// ****************************************************************************
// Section: Data Types and Constants
// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
/* USB Hub Host Client Driver Interface Pointer
 
  Summary: 
    USB Hub Host Client Driver Interface Pointer.

  Description:
    This constant is a pointer to a table of function pointers that define the
    interface between the Hub Host Client Driver and the USB Host Layer. This
    constant should be used while adding support for the Hub Driver in TPL
    table.

  Remarks:
    None.
*/

/*DOM-IGNORE-BEGIN*/extern USB_HOST_CLIENT_DRIVER gUSBHostHUBClientDriver; /*DOM-IGNORE-END*/
#define USB_HOST_HUB_INTERFACE  /*DOM-IGNORE-BEGIN*/&gUSBHostHUBClientDriver /*DOM-IGNORE-END*/

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

// ****************************************************************************
// ****************************************************************************
// Section: Hub Host Client Driver Interface
// ****************************************************************************
// ****************************************************************************

/* The Hub Host Client Driver does not contain any application callable API. */

#endif
