/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGVErrClear_Unsupported.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGVErrClear
    and its Variant : Unsupported
    For following APIs :
        PLIB_SB_ExistsPGVErrClear
        PLIB_SB_PGVErrorMulti
        PLIB_SB_PGVErrorCode
        PLIB_SB_PGVErrorLogClearSingle
        PLIB_SB_PGVErrorLogClearMulti

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

#ifndef _SB_PGVERRCLEAR_UNSUPPORTED_H
#define _SB_PGVERRCLEAR_UNSUPPORTED_H

//******************************************************************************
/* Function :  SB_ExistsPGVErrClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_ExistsPGVErrClear

  Description:
    This template implements the Unsupported variant of the PLIB_SB_ExistsPGVErrClear function.
*/

PLIB_TEMPLATE bool SB_ExistsPGVErrClear_Unsupported( SB_MODULE_ID index )
{
    return false;
}


//******************************************************************************
/* Function :  SB_PGVErrorMulti_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGVErrorMulti 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGVErrorMulti function.
*/

PLIB_TEMPLATE bool SB_PGVErrorMulti_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_ID target )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGVErrorMulti");

    return false;
}


//******************************************************************************
/* Function :  SB_PGVErrorCode_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGVErrorCode 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGVErrorCode function.
*/

PLIB_TEMPLATE PLIB_SB_ERROR SB_PGVErrorCode_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_ID target )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGVErrorCode");

    return (PLIB_SB_ERROR) 0;
}


//******************************************************************************
/* Function :  SB_PGVErrorLogClearSingle_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGVErrorLogClearSingle 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGVErrorLogClearSingle function.
*/

PLIB_TEMPLATE void SB_PGVErrorLogClearSingle_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_ID target )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGVErrorLogClearSingle");
}


//******************************************************************************
/* Function :  SB_PGVErrorLogClearMulti_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGVErrorLogClearMulti 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGVErrorLogClearMulti function.
*/

PLIB_TEMPLATE void SB_PGVErrorLogClearMulti_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_ID target )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGVErrorLogClearMulti");
}


#endif /*_SB_PGVERRCLEAR_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

