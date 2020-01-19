/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_InitPermGrp_Default.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : InitPermGrp
    and its Variant : Default
    For following APIs :
        PLIB_SB_ExistsInitPermGrp
        PLIB_SB_InitPermGrpSet

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

#ifndef _SB_INITPERMGRP_DEFAULT_H
#define _SB_INITPERMGRP_DEFAULT_H

//******************************************************************************
/* Function :  SB_ExistsInitPermGrp_Default

  Summary:
    Implements Default variant of PLIB_SB_ExistsInitPermGrp

  Description:
    This template implements the Default variant of the PLIB_SB_ExistsInitPermGrp function.
*/

#define PLIB_SB_ExistsInitPermGrp PLIB_SB_ExistsInitPermGrp
PLIB_TEMPLATE bool SB_ExistsInitPermGrp_Default( SB_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  SB_InitPermGrpSet_Default

  Summary:
    Implements Default variant of PLIB_SB_InitPermGrpSet 

  Description:
    This template implements the Default variant of the PLIB_SB_InitPermGrpSet function.
    Operation is not atomic.
*/
PLIB_TEMPLATE void SB_InitPermGrpSet_Default( SB_MODULE_ID index , PLIB_SB_PG_INITIATOR initiator , PLIB_SB_INIT_PG pg )
{
    uint32_t val = CFGPG;
    val &= ~(0x03u << initiator);
    val |= (pg << initiator);
    CFGPG = val;
}


#endif /*_SB_INITPERMGRP_DEFAULT_H*/

/******************************************************************************
 End of File
*/

