/*******************************************************************************
  WDT Peripheral Library Template Implementation

  File Name:
    wdt_WindowEnable_Default.h

  Summary:
    WDT PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : WindowEnable
    and its Variant : Default
    For following APIs :
        PLIB_WDT_ExistsWindowEnable
        PLIB_WDT_WindowEnable
        PLIB_WDT_WindowDisable

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

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _WDT_WINDOWENABLE_DEFAULT_H
#define _WDT_WINDOWENABLE_DEFAULT_H

//******************************************************************************
/* Function :  WDT_ExistsWindowEnable_Default

  Summary:
    Implements Default variant of PLIB_WDT_ExistsWindowEnable

  Description:
    This template implements the Default variant of the PLIB_WDT_ExistsWindowEnable function.
*/

#define PLIB_WDT_ExistsWindowEnable PLIB_WDT_ExistsWindowEnable
PLIB_TEMPLATE bool WDT_ExistsWindowEnable_Default( WDT_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  WDT_WindowEnable_Default

  Summary:
    Implements Default variant of PLIB_WDT_WindowEnable

  Description:
    This template implements the Default variant of the PLIB_WDT_WindowEnable function.
*/

PLIB_TEMPLATE void WDT_WindowEnable_Default( WDT_MODULE_ID index )
{
    WDTCONSET = _WDTCON_WDTWINEN_MASK;
}


//******************************************************************************
/* Function :  WDT_WindowDisable_Default

  Summary:
    Implements Default variant of PLIB_WDT_WindowDisable

  Description:
    This template implements the Default variant of the PLIB_WDT_WindowDisable function.
*/

PLIB_TEMPLATE void WDT_WindowDisable_Default( WDT_MODULE_ID index )
{
    WDTCONCLR = _WDTCON_WDTWINEN_MASK;
}


#endif /*_WDT_WINDOWENABLE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

