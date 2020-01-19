/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTRInterrupt_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTRInterrupt
    and its Variant : Default
    For following APIs :
        PLIB_CTR_ExistsInterrupt
        PLIB_CTR_IntModeLatchSelect
        PLIB_CTR_IntModeLatchGet
        PLIB_CTR_IntLatchSelect
        PLIB_CTR_IntLatchGet

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

#ifndef _CTR_CTRINTERRUPT_DEFAULT_H
#define _CTR_CTRINTERRUPT_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsInterrupt_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsInterrupt

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsInterrupt function.
*/

#define PLIB_CTR_ExistsInterrupt PLIB_CTR_ExistsInterrupt
PLIB_TEMPLATE bool CTR_ExistsInterrupt_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_IntModeLatchSelect_Default

  Summary:
    Implements Default variant of PLIB_CTR_IntModeLatchSelect 

  Description:
    This template implements the Default variant of the PLIB_CTR_IntModeLatchSelect function.
*/

PLIB_TEMPLATE void CTR_IntModeLatchSelect_Default( CTR_MODULE_ID index , CTR_LATCH_INT_MODE intMode , CTR_LATCH_UNIT_SELECT latNum )
{
	CTR_INT = (CTR_INT & ~(_CTR_INT_INT_MODE_0_MASK << (latNum * 2))) | (intMode << (_CTR_INT_INT_MODE_0_POSITION + (latNum * 2))); 
}


//******************************************************************************
/* Function :  CTR_IntModeLatchGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_IntModeLatchGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_IntModeLatchGet function.
*/

PLIB_TEMPLATE CTR_LATCH_INT_MODE CTR_IntModeLatchGet_Default( CTR_MODULE_ID index , CTR_LATCH_UNIT_SELECT latNum )
{
	return (CTR_LATCH_INT_MODE) ((CTR_INT & (_CTR_INT_INT_MODE_0_MASK << (latNum * 2))) >> (_CTR_INT_INT_MODE_0_POSITION + (latNum * 2)));
}


//******************************************************************************
/* Function :  CTR_IntLatchSelect_Default

  Summary:
    Implements Default variant of PLIB_CTR_IntLatchSelect 

  Description:
    This template implements the Default variant of the PLIB_CTR_IntLatchSelect function.
*/

PLIB_TEMPLATE void CTR_IntLatchSelect_Default( CTR_MODULE_ID index , CTR_ENABLE_LATCH_INT_GEN                               enableLatchVal )
{
	CTR_INT = (CTR_INT & ~_CTR_INT_INT_SEL_MASK) | enableLatchVal;
}


//******************************************************************************
/* Function :  CTR_IntLatchGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_IntLatchGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_IntLatchGet function.
*/

PLIB_TEMPLATE CTR_ENABLE_LATCH_INT_GEN CTR_IntLatchGet_Default( CTR_MODULE_ID index )
{
    return CTR_INT & _CTR_INT_INT_SEL_MASK;
}


#endif /*_CTR_CTRINTERRUPT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

