/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGRegAddr_Unsupported.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGRegAddr
    and its Variant : Unsupported
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

#ifndef _SB_PGREGADDR_UNSUPPORTED_H
#define _SB_PGREGADDR_UNSUPPORTED_H

//******************************************************************************
/* Function :  SB_PGRegionAddrSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGRegionAddrSet 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGRegionAddrSet function.
*/

PLIB_TEMPLATE void SB_PGRegionAddrSet_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_REGION region , uint32_t phys_addr )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGRegionAddrSet");
}


//******************************************************************************
/* Function :  SB_PGRegionAddrGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGRegionAddrGet 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGRegionAddrGet function.
*/

PLIB_TEMPLATE uint32_t SB_PGRegionAddrGet_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_REGION region )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGRegionAddrGet");

    return 0;
}


//******************************************************************************
/* Function :  SB_ExistsPGRegAddr_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_ExistsPGRegAddr

  Description:
    This template implements the Unsupported variant of the PLIB_SB_ExistsPGRegAddr function.
*/

PLIB_TEMPLATE bool SB_ExistsPGRegAddr_Unsupported( SB_MODULE_ID index )
{
    return false;
}


#endif /*_SB_PGREGADDR_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

