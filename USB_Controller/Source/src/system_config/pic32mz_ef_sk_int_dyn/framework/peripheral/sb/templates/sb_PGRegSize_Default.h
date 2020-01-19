/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGRegSize_Default.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGRegSize
    and its Variant : Default
    For following APIs :
        PLIB_SB_PGRegionSizeSet
        PLIB_SB_PGRegionSizeGet
        PLIB_SB_ExistsPGRegSize

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

#ifndef _SB_PGREGSIZE_DEFAULT_H
#define _SB_PGREGSIZE_DEFAULT_H

//******************************************************************************
/* Function :  SB_PGRegionSizeSet_Default

  Summary:
    Implements Default variant of PLIB_SB_PGRegionSizeSet 

  Description:
    This template implements the Default variant of the PLIB_SB_PGRegionSizeSet function.
    This operation is not atomic.
*/
PLIB_TEMPLATE void SB_PGRegionSizeSet_Default( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , uint32_t size )
{
    volatile __SBT0REG0bits_t *SBTxREGy = (volatile __SBT0REG0bits_t *)(&SBT0REG0 + (0x400u * (region >> 8u) + 0x20u * (region & 0xFF)) / 4u);

    SBTxREGy->SIZE = size;
}

//******************************************************************************
/* Function :  SB_PGRegionSizeGet_Default

  Summary:
    Implements Default variant of PLIB_SB_PGRegionSizeGet 

  Description:
    This template implements the Default variant of the PLIB_SB_PGRegionSizeGet function.
    This operation is atomic.
*/
PLIB_TEMPLATE uint32_t SB_PGRegionSizeGet_Default( SB_MODULE_ID index , PLIB_SB_TGT_REGION region )
{
    volatile __SBT0REG0bits_t *SBTxREGy = (volatile __SBT0REG0bits_t *)(&SBT0REG0 + (0x400u * (region >> 8u) + 0x20u * (region & 0xFF)) / 4u);

    return (uint32_t)SBTxREGy->SIZE;
}

//******************************************************************************
/* Function :  SB_ExistsPGRegSize_Default

  Summary:
    Implements Default variant of PLIB_SB_ExistsPGRegSize

  Description:
    This template implements the Default variant of the PLIB_SB_ExistsPGRegSize function.
*/
#define PLIB_SB_ExistsPGRegSize PLIB_SB_ExistsPGRegSize
PLIB_TEMPLATE bool SB_ExistsPGRegSize_Default( SB_MODULE_ID index )
{
    return true;
}

#endif /*_SB_PGREGSIZE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

