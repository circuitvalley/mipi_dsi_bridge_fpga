/*******************************************************************************
  SB Peripheral Library Template Implementation

  File Name:
    sb_PGVErrGroupStatus_Default.h

  Summary:
    SB PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PGVErrGroupStatus
    and its Variant : Default
    For following APIs :
        PLIB_SB_ExistsPGVErrGroupStatus
        PLIB_SB_PGVErrGroupStatus

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _SB_PGVERRGROUPSTATUS_DEFAULT_H
#define _SB_PGVERRGROUPSTATUS_DEFAULT_H

//******************************************************************************
/* Function :  SB_ExistsPGVErrGroupStatus_Default

  Summary:
    Implements Default variant of PLIB_SB_ExistsPGVErrGroupStatus

  Description:
    This template implements the Default variant of the PLIB_SB_ExistsPGVErrGroupStatus function.
*/
#define PLIB_SB_ExistsPGVErrGroupStatus PLIB_SB_ExistsPGVErrGroupStatus
PLIB_TEMPLATE bool SB_ExistsPGVErrGroupStatus_Default( SB_MODULE_ID index )
{
    return true;
}

//******************************************************************************
/* Function :  SB_PGVErrGroupStatus_Default

  Summary:
    Implements Default variant of PLIB_SB_PGVErrGroupStatus 

  Description:
    This template implements the Default variant of the PLIB_SB_PGVErrGroupStatus function.
*/
PLIB_TEMPLATE bool SB_PGVErrGroupStatus_Default( SB_MODULE_ID index , PLIB_SB_PGV_GROUP_ID groupId )
{
    switch (groupId)
    {
        case PLIB_SB_PGV_GROUP0:
	    return SBFLAG0bits.T0PGV0;

        case PLIB_SB_PGV_GROUP1:
	    return SBFLAG1bits.T0PGV1;

        case PLIB_SB_PGV_GROUP2:
	    return SBFLAG2bits.T0PGV2;

        case PLIB_SB_PGV_GROUP3:
        default:
	    return SBFLAG3bits.T0PGV3;
    }
}

#endif /*_SB_PGVERRGROUPSTATUS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

