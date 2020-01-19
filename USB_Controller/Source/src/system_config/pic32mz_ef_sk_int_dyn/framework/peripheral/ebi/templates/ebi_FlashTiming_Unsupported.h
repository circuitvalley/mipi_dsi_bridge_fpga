/*******************************************************************************
  EBI Peripheral Library Template Implementation

  File Name:
    ebi_FlashTiming_Unsupported.h

  Summary:
    EBI PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : FlashTiming
    and its Variant : Unsupported
    For following APIs :
        PLIB_EBI_FlashTimingSet
        PLIB_EBI_FlashTimingGet
        PLIB_EBI_ExistsFlashTiming

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _EBI_FLASHTIMING_UNSUPPORTED_H
#define _EBI_FLASHTIMING_UNSUPPORTED_H


//******************************************************************************
/* Function :  EBI_FlashTimingSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_EBI_FlashTimingSet 

  Description:
    This template implements the Unsupported variant of the PLIB_EBI_FlashTimingSet function.
*/

PLIB_TEMPLATE void EBI_FlashTimingSet_Unsupported( EBI_MODULE_ID index , int FlashTiming )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_EBI_FlashTimingSet");
}


//******************************************************************************
/* Function :  EBI_FlashTimingGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_EBI_FlashTimingGet 

  Description:
    This template implements the Unsupported variant of the PLIB_EBI_FlashTimingGet function.
*/

PLIB_TEMPLATE int EBI_FlashTimingGet_Unsupported( EBI_MODULE_ID index , int ChipSelectNumber )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_EBI_FlashTimingGet");

    return 0;
}


//******************************************************************************
/* Function :  EBI_ExistsFlashTiming_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_EBI_ExistsFlashTiming

  Description:
    This template implements the Unsupported variant of the PLIB_EBI_ExistsFlashTiming function.
*/

PLIB_TEMPLATE bool EBI_ExistsFlashTiming_Unsupported( EBI_MODULE_ID index )
{
    return false;
}


#endif /*_EBI_FLASHTIMING_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

