/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGRegSize_Unsupported.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGRegSize
    and its Variant : Unsupported
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

#ifndef _SB_PGREGSIZE_UNSUPPORTED_H
#define _SB_PGREGSIZE_UNSUPPORTED_H

//******************************************************************************
/* Function :  SB_PGRegionSizeSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGRegionSizeSet 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGRegionSizeSet function.
*/

PLIB_TEMPLATE void SB_PGRegionSizeSet_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , uint32_t size )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGRegionSizeSet");
}


//******************************************************************************
/* Function :  SB_PGRegionSizeGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGRegionSizeGet 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGRegionSizeGet function.
*/

PLIB_TEMPLATE uint32_t SB_PGRegionSizeGet_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_REGION region )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGRegionSizeGet");

    return 0;
}


//******************************************************************************
/* Function :  SB_ExistsPGRegSize_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_ExistsPGRegSize

  Description:
    This template implements the Unsupported variant of the PLIB_SB_ExistsPGRegSize function.
*/

PLIB_TEMPLATE bool SB_ExistsPGRegSize_Unsupported( SB_MODULE_ID index )
{
    return false;
}


#endif /*_SB_PGREGSIZE_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

