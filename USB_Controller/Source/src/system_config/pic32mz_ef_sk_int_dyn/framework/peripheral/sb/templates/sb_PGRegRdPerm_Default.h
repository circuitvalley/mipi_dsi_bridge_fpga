/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGRegRdPerm_Default.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGRegRdPerm
    and its Variant : Default
    For following APIs :
        PLIB_SB_PGRegionReadPermSet
        PLIB_SB_PGRegionReadPermClear
        PLIB_SB_ExistsPGRegRdPerm

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

#ifndef _SB_PGREGRDPERM_DEFAULT_H
#define _SB_PGREGRDPERM_DEFAULT_H

//******************************************************************************
/* Function :  SB_PGRegionReadPermSet_Default

  Summary:
    Implements Default variant of PLIB_SB_PGRegionReadPermSet 

  Description:
    This template implements the Default variant of the PLIB_SB_PGRegionReadPermSet function.
    This operation is not atomic.
*/
PLIB_TEMPLATE void SB_PGRegionReadPermSet_Default( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , PLIB_SB_REGION_PG readPerm )
{
    volatile uint32_t *SBTxRDy = (volatile uint32_t *)(&SBT0RD0 + (0x400u * (region >> 8u) + 0x20u * (region & 0xFF)) / 4u);

    *SBTxRDy |= readPerm; /* readPerm is a bitmask */
}


//******************************************************************************
/* Function :  SB_PGRegionReadPermClear_Default

  Summary:
    Implements Default variant of PLIB_SB_PGRegionReadPermClear 

  Description:
    This template implements the Default variant of the PLIB_SB_PGRegionReadPermClear function.
    This operation is not atomic.
*/
PLIB_TEMPLATE void SB_PGRegionReadPermClear_Default( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , PLIB_SB_REGION_PG readPerm )
{
    volatile uint32_t *SBTxRDy = (volatile uint32_t *)(&SBT0RD0 + (0x400u * (region >> 8u) + 0x20u * (region & 0xFF)) / 4u);

    *SBTxRDy &= ~readPerm;
}


//******************************************************************************
/* Function :  SB_ExistsPGRegRdPerm_Default

  Summary:
    Implements Default variant of PLIB_SB_ExistsPGRegRdPerm

  Description:
    This template implements the Default variant of the PLIB_SB_ExistsPGRegRdPerm function.
*/

#define PLIB_SB_ExistsPGRegRdPerm PLIB_SB_ExistsPGRegRdPerm
PLIB_TEMPLATE bool SB_ExistsPGRegRdPerm_Default( SB_MODULE_ID index )
{
    return true;
}


#endif /*_SB_PGREGRDPERM_DEFAULT_H*/

/******************************************************************************
 End of File
*/

