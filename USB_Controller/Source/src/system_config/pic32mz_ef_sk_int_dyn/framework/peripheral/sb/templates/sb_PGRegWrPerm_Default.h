/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGRegWrPerm_Default.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGRegWrPerm
    and its Variant : Default
    For following APIs :
        PLIB_SB_PGRegionWritePermSet
        PLIB_SB_PGRegionWritePermClear
        PLIB_SB_ExistsPGRegWrPerm

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

#ifndef _SB_PGREGWRPERM_DEFAULT_H
#define _SB_PGREGWRPERM_DEFAULT_H

//******************************************************************************
/* Function :  SB_PGRegionWritePermSet_Default

  Summary:
    Implements Default variant of PLIB_SB_PGRegionWritePermSet 

  Description:
    This template implements the Default variant of the PLIB_SB_PGRegionWritePermSet function.
    This operation is not atomic.
*/
PLIB_TEMPLATE void SB_PGRegionWritePermSet_Default( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , PLIB_SB_REGION_PG writePerm )
{
    volatile uint32_t *SBTxWRy = (volatile uint32_t *)(&SBT0WR0 +  (0x400u * (region >> 8u) + 0x20u * (region & 0xFF)) / 4u);

    *SBTxWRy |= writePerm; /* writePerm is bitmask */
}

//******************************************************************************
/* Function :  SB_PGRegionWritePermClear_Default

  Summary:
    Implements Default variant of PLIB_SB_PGRegionWritePermClear 

  Description:
    This template implements the Default variant of the PLIB_SB_PGRegionWritePermClear function.
    This operation is not atomic.
*/
PLIB_TEMPLATE void SB_PGRegionWritePermClear_Default( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , PLIB_SB_REGION_PG writePerm )
{
    volatile uint32_t *SBTxWRy = (volatile uint32_t *)(&SBT0WR0 +  (0x400u * (region >> 8u) + 0x20u * (region & 0xFF)) / 4u);

    *SBTxWRy &= ~writePerm;
}

//******************************************************************************
/* Function :  SB_ExistsPGRegWrPerm_Default

  Summary:
    Implements Default variant of PLIB_SB_ExistsPGRegWrPerm

  Description:
    This template implements the Default variant of the PLIB_SB_ExistsPGRegWrPerm function.
*/
#define PLIB_SB_ExistsPGRegWrPerm PLIB_SB_ExistsPGRegWrPerm
PLIB_TEMPLATE bool SB_ExistsPGRegWrPerm_Default( SB_MODULE_ID index )
{
    return true;
}

#endif /*_SB_PGREGWRPERM_DEFAULT_H*/

/******************************************************************************
 End of File
*/

