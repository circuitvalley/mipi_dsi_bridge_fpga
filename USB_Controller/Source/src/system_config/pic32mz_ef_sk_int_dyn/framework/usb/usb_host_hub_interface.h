/******************************************************************************
  PIC32 USB Host Hub Interface Header File

  Company:
    Microchip Technology Inc.
    
  File Name:
    drv_usb.h
	
  Summary:
    PIC32 USB Host Hub Interface Header File
	
  Description:
    This file contains defintions and structure that implement the interface
    between the USB Host Layer and an external Hub.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _USB_HOST_HUB_INTERFACE_H_
#define _USB_HOST_HUB_INTERFACE_H_


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
/* USB Hub Port API Interface

  Summary:
    Group of function pointers to the USB Hub Port Functions.

  Description:
    This structure is a group of function pointers pointing to the USB Hub Port
    API routines. The USB Hub or USB root hub should export this group of
    functions so that the Host and device layer can access the port
    functionality.

  Remarks:
    None.
*/

typedef struct
{
    /* This is a pointer to the port reset function */
    USB_ERROR (*hubPortReset)(uintptr_t hubAddress, uint8_t port);

    /* This is pointer to the port reset completion status inquiry function */
    bool (*hubPortResetIsComplete)(uintptr_t hubAddress, uint8_t port);

    /* This is pointer to the port suspend function */
    USB_ERROR(*hubPortSuspend)(uintptr_t hubAddress, uint8_t port);

    /* This is a pointer to the port resume function */
    USB_ERROR(*hubPortResume)(uintptr_t hubAddress, uint8_t port);

    /* This is a pointer to the port speed get function */
    USB_SPEED(*hubPortSpeedGet)(uintptr_t hubAddress, uint8_t port);

} USB_HUB_INTERFACE;


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif

