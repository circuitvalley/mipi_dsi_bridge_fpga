/*******************************************************************************
  IC Peripheral Library Template Implementation

  File Name:
    ic_AlternateClock_WK.h

  Summary:
    IC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : AlternateClock
    and its Variant : WK
    For following APIs :
        PLIB_IC_AlternateClockEnable
        PLIB_IC_AlternateClockDisable
        PLIB_IC_ExistsAlternateClock

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _IC_ALTERNATECLOCK_WK_H
#define _IC_ALTERNATECLOCK_WK_H


//******************************************************************************
/* Function :  IC_AlternateClockEnable_WK

  Summary:
    Implements WK variant of PLIB_IC_AlternateClockEnable 

  Description:
    This template implements the WK variant of the PLIB_IC_AlternateClockEnable function.
*/

PLIB_TEMPLATE void IC_AlternateClockEnable_WK( IC_MODULE_ID index )
{
    CFGCON0bits.ICACLK = 1;
}


//******************************************************************************
/* Function :  IC_AlternateClockDisable_WK

  Summary:
    Implements WK variant of PLIB_IC_AlternateClockDisable 

  Description:
    This template implements the WK variant of the PLIB_IC_AlternateClockDisable function.
*/

PLIB_TEMPLATE void IC_AlternateClockDisable_WK( IC_MODULE_ID index )
{
    CFGCON0bits.ICACLK = 0;
}


//******************************************************************************
/* Function :  IC_ExistsAlternateClock_WK

  Summary:
    Implements WK variant of PLIB_IC_ExistsAlternateClock

  Description:
    This template implements the WK variant of the PLIB_IC_ExistsAlternateClock function.
*/

#define PLIB_IC_ExistsAlternateClock PLIB_IC_ExistsAlternateClock
PLIB_TEMPLATE bool IC_ExistsAlternateClock_WK( IC_MODULE_ID index )
{
    return true;
}


#endif /*_IC_ALTERNATECLOCK_WK_H*/

/******************************************************************************
 End of File
*/

