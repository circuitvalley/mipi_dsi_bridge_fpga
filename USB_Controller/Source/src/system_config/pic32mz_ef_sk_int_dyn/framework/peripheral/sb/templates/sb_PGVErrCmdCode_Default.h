/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGVErrCmdCode_Default.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGVErrCmdCode
    and its Variant : Default
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

#ifndef _SB_PGVERRCMDCODE_DEFAULT_H
#define _SB_PGVERRCMDCODE_DEFAULT_H

//******************************************************************************
/* Function :  SB_ExistsPGVErrCmdCode_Default

  Summary:
    Implements Default variant of PLIB_SB_ExistsPGVErrCmdCode

  Description:
    This template implements the Default variant of the PLIB_SB_ExistsPGVErrCmdCode function.
*/
#define PLIB_SB_ExistsPGVErrCmdCode PLIB_SB_ExistsPGVErrCmdCode
PLIB_TEMPLATE bool SB_ExistsPGVErrCmdCode_Default( SB_MODULE_ID index )
{
    return true;
}

//******************************************************************************
/* Function :  SB_PGVErrorCommandCode_Default

  Summary:
    Implements Default variant of PLIB_SB_PGVErrorCommandCode 

  Description:
    This template implements the Default variant of the PLIB_SB_PGVErrorCommandCode function.
    This operation is atomic.
*/
PLIB_TEMPLATE PLIB_SB_OCP_CMD_CODE SB_PGVErrorCommandCode_Default( SB_MODULE_ID index , PLIB_SB_TGT_ID target )
{
    volatile __SBT0ELOG1bits_t *SBTxELOG1 = (volatile __SBT0ELOG1bits_t *)(&SBT0ELOG1 + (0x400u * target) / 4u);

    return (PLIB_SB_OCP_CMD_CODE)SBTxELOG1->CMD;
}

#endif /*_SB_PGVERRCMDCODE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

