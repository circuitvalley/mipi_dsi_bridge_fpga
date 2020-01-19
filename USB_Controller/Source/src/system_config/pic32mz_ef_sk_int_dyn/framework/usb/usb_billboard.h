		
/*******************************************************************************
  USB Billboard class definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_billboard.h

  Summary:
    USB Billboard class definitions

  Description:
    This file describes the Billboard class specific definitions. File needs to
    be included by the application for USB Billboard host/device functionality.
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

#ifndef _USB_BILLBOARD_H
#define _USB_BILLBOARD_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define USB_BILLBOARD_CLASS 0x11
#define USB_BILLBOARD_SUBCLASS 0x00
#define USB_BILLBOARD_PROTOCOL 0x00
#define USB_BILLBOARD_CAPABILITY_DESCRIPTOR_TYPE 0x0D

		

	
// *****************************************************************************
/* USB Billboard Capability Descriptor

  Summary:
    Identifies USB Billboard Capability Descriptor

  Description:
    This type identifies USB Billboard Capability Descriptor. This structure is
    as per Table 3-6: Billboard Capability Descriptor of USB Device Class Definition 
    for Billboard Devices protocol Revision 1.0 .

  Remarks:
    Needs to packed always.
*/
typedef struct __attribute__ ((packed))
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDevCapabilityType;
    uint8_t iAdditionalInfoURL;
    uint8_t bNumberOfAlternateModes;
    uint8_t bPreferredAlternateMode;
    uint16_t vconnPower; 
    uint8_t bmConfigured[32]; 
    uint32_t bReserved;
} USB_BILLBOARD_CAPABILITY_DESCRIPTOR_HEADER;
 
 
// *****************************************************************************
/* USB Billboard Alternate mode

  Summary:
    Identifies USB Billboard Alternate mode

  Description:
    This type identifies USB Billboard Alternate mode. This structure is
    as per Table 3-6: Billboard Capability Descriptor of USB Device Class Definition 
    for Billboard Devices protocol Revision 1.0 .

  Remarks:
    Needs to packed always.
*/ 
typedef struct __attribute__ ((packed))
{
    uint16_t wSVID;
    uint8_t bAlternateMode; 
    uint8_t iAlternateModeString;
} USB_BILLBOARD_ALTERNATE_MODE;
       
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END
 
        
#endif
       
