/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGVErrCmdCode_Unsupported.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGVErrCmdCode
    and its Variant : Unsupported
    For following APIs :
        PLIB_SB_ExistsPGVErrCmdCode
        PLIB_SB_PGVErrorCommandCode

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

#ifndef _SB_PGVERRCMDCODE_UNSUPPORTED_H
#define _SB_PGVERRCMDCODE_UNSUPPORTED_H

//******************************************************************************
/* Function :  SB_ExistsPGVErrCmdCode_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_ExistsPGVErrCmdCode

  Description:
    This template implements the Unsupported variant of the PLIB_SB_ExistsPGVErrCmdCode function.
*/

PLIB_TEMPLATE bool SB_ExistsPGVErrCmdCode_Unsupported( SB_MODULE_ID index )
{
    return false;
}


//******************************************************************************
/* Function :  SB_PGVErrorCommandCode_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_SB_PGVErrorCommandCode 

  Description:
    This template implements the Unsupported variant of the PLIB_SB_PGVErrorCommandCode function.
*/

PLIB_TEMPLATE PLIB_SB_OCP_CMD_CODE SB_PGVErrorCommandCode_Unsupported( SB_MODULE_ID index , PLIB_SB_TGT_ID target )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_SB_PGVErrorCommandCode");

    return (PLIB_SB_OCP_CMD_CODE)0;
}


#endif /*_SB_PGVERRCMDCODE_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

