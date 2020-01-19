/*******************************************************************************
  DMT Peripheral Library Template Implementation

  File Name:
    dmt_PostscalerValue_Default.h

  Summary:
    DMT PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PostscalerValue
    and its Variant : Default
    For following APIs :
        PLIB_DMT_PostscalerValueGet
        PLIB_DMT_ExistsPostscalerValue

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

#ifndef _DMT_POSTscalerVALUE_DEFAULT_H
#define _DMT_POSTscalerVALUE_DEFAULT_H

//******************************************************************************
/* Function :  DMT_PostscalerValueGet_Default

  Summary:
    Implements Default variant of PLIB_DMT_PostscalerValueGet 

  Description:
    This template implements the Default variant of the PLIB_DMT_PostscalerValueGet function.
*/

PLIB_TEMPLATE uint32_t DMT_PostscalerValueGet_Default( DMT_MODULE_ID index )
{
    return (uint32_t)DMTPSCNT;
}


//******************************************************************************
/* Function :  DMT_ExistsPostscalerValue_Default

  Summary:
    Implements Default variant of PLIB_DMT_ExistsPostscalerValue

  Description:
    This template implements the Default variant of the PLIB_DMT_ExistsPostscalerValue function.
*/

#define PLIB_DMT_ExistsPostscalerValue PLIB_DMT_ExistsPostscalerValue
PLIB_TEMPLATE bool DMT_ExistsPostscalerValue_Default( DMT_MODULE_ID index )
{
    return true;
}


#endif /*_DMT_POSTscalerVALUE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

