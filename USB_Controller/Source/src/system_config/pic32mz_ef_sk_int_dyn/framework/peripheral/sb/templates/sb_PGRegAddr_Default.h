/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGRegAddr_Default.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGRegAddr
    and its Variant : Default
    For following APIs :
        PLIB_SB_PGRegionAddrSet
        PLIB_SB_PGRegionAddrGet
        PLIB_SB_ExistsPGRegAddr

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

#ifndef _SB_PGREGADDR_DEFAULT_H
#define _SB_PGREGADDR_DEFAULT_H

//******************************************************************************
/* Function :  SB_PGRegionAddrSet_Default

  Summary:
    Implements Default variant of PLIB_SB_PGRegionAddrSet 

  Description:
    This template implements the Default variant of the PLIB_SB_PGRegionAddrSet function.
*/
PLIB_TEMPLATE void SB_PGRegionAddrSet_Default( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , uint32_t phys_addr )
{
    volatile __SBT0REG0bits_t *SBTxREGy = (volatile __SBT0REG0bits_t *)(&SBT0REG0 +  ((0x400u * (region >> 8u) + 0x20u * (region & 0xFF)) / 4u));

    SBTxREGy->BASE = phys_addr >> 10u; /* legacy */
}

//******************************************************************************
/* Function :  SB_PGRegionAddrGet_Default

  Summary:
    Implements Default variant of PLIB_SB_PGRegionAddrGet 

  Description:
    This template implements the Default variant of the PLIB_SB_PGRegionAddrGet function.
*/
PLIB_TEMPLATE uint32_t SB_PGRegionAddrGet_Default( SB_MODULE_ID index , PLIB_SB_TGT_REGION region )
{
    volatile __SBT0REG0bits_t *SBTxREGy = (volatile __SBT0REG0bits_t *)(&SBT0REG0 +  ((0x400u * (region >> 8u) + 0x20u * (region & 0xFF)) / 4u));

    return (uint32_t)SBTxREGy->BASE << 10u; /* legacy */
}

//******************************************************************************
/* Function :  SB_ExistsPGRegAddr_Default

  Summary:
    Implements Default variant of PLIB_SB_ExistsPGRegAddr

  Description:
    This template implements the Default variant of the PLIB_SB_ExistsPGRegAddr function.
*/
#define PLIB_SB_ExistsPGRegAddr PLIB_SB_ExistsPGRegAddr
PLIB_TEMPLATE bool SB_ExistsPGRegAddr_Default( SB_MODULE_ID index )
{
    return true;
}

#endif /*_SB_PGREGADDR_DEFAULT_H*/

/******************************************************************************
 End of File
*/

