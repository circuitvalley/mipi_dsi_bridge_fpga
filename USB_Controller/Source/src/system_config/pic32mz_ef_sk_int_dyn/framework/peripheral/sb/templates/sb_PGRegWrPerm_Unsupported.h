/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGRegWrPerm_Unsupported.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGRegWrPerm
    and its Variant : Unsupported
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

#ifndef _SB_PGREGWRPERM_UNSUPPORTED_H
#define _SB_PGREGWRPERM_UNSUPPORTED_H

//******************************************************************************
/* Function :  SB_PGRegionWritePermSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGRegionWritePermSet 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGRegionWritePermSet function.
*/

PLIB_TEMPLATE void SB_PGRegionWritePermSet_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , PLIB_SB_REGION_PG writePerm )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGRegionWritePermSet");
}


//******************************************************************************
/* Function :  SB_PGRegionWritePermClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGRegionWritePermClear 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGRegionWritePermClear function.
*/

PLIB_TEMPLATE void SB_PGRegionWritePermClear_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , PLIB_SB_REGION_PG writePerm )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGRegionWritePermClear");
}


//******************************************************************************
/* Function :  SB_ExistsPGRegWrPerm_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_ExistsPGRegWrPerm

  Description:
    This template implements the Unsupported variant of the PLIB_SB_ExistsPGRegWrPerm function.
*/

PLIB_TEMPLATE bool SB_ExistsPGRegWrPerm_Unsupported( SB_MODULE_ID index )
{
    return false;
}


#endif /*_SB_PGREGWRPERM_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

