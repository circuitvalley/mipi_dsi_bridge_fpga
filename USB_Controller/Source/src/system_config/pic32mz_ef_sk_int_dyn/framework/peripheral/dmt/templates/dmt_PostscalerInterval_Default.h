/*******************************************************************************
  DMT Peripheral Library Template Implementation

  File Name:
    dmt_PostscalerInterval_Default.h

  Summary:
    DMT PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PostscalerInterval
    and its Variant : Default
    For following APIs :
        PLIB_DMT_PostscalerIntervalGet
        PLIB_DMT_ExistsPostscalerInterval

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

#ifndef _DMT_POSTscalerINTERVAL_DEFAULT_H
#define _DMT_POSTscalerINTERVAL_DEFAULT_H

//******************************************************************************
/* Function :  DMT_PostscalerIntervalGet_Default

  Summary:
    Implements Default variant of PLIB_DMT_PostscalerIntervalGet 

  Description:
    This template implements the Default variant of the PLIB_DMT_PostscalerIntervalGet function.
*/

PLIB_TEMPLATE uint32_t DMT_PostscalerIntervalGet_Default( DMT_MODULE_ID index )
{
    return (uint32_t)DMTPSINTV;
}


//******************************************************************************
/* Function :  DMT_ExistsPostscalerInterval_Default

  Summary:
    Implements Default variant of PLIB_DMT_ExistsPostscalerInterval

  Description:
    This template implements the Default variant of the PLIB_DMT_ExistsPostscalerInterval function.
*/

#define PLIB_DMT_ExistsPostscalerInterval PLIB_DMT_ExistsPostscalerInterval
PLIB_TEMPLATE bool DMT_ExistsPostscalerInterval_Default( DMT_MODULE_ID index )
{
    return true;
}


#endif /*_DMT_POSTscalerINTERVAL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

