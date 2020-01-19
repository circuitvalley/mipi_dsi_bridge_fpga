/*******************************************************************************
  CTR Peripheral Library Template Implementation

  File Name:
    ctr_CTRTestBusSelect_Default.h

  Summary:
    CTR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CTRTestBusSelect
    and its Variant : Default
    For following APIs :
        PLIB_CTR_ExistsTestBusSelect
        PLIB_CTR_TestBusSelect
        PLIB_CTR_TestBusGet

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

#ifndef _CTR_CTRTESTBUSSELECT_DEFAULT_H
#define _CTR_CTRTESTBUSSELECT_DEFAULT_H

//******************************************************************************
/* Function :  CTR_ExistsTestBusSelect_Default

  Summary:
    Implements Default variant of PLIB_CTR_ExistsTestBusSelect

  Description:
    This template implements the Default variant of the PLIB_CTR_ExistsTestBusSelect function.
*/

#define PLIB_CTR_ExistsTestBusSelect PLIB_CTR_ExistsTestBusSelect
PLIB_TEMPLATE bool CTR_ExistsTestBusSelect_Default( CTR_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  CTR_TestBusSelect_Default

  Summary:
    Implements Default variant of PLIB_CTR_TestBusSelect 

  Description:
    This template implements the Default variant of the PLIB_CTR_TestBusSelect function.
*/

PLIB_TEMPLATE void CTR_TestBusSelect_Default( CTR_MODULE_ID index , CTR_TEST_BUS_SELECT testBusVal )
{
	CTR_TEST_BUS_SEL = testBusVal;
}


//******************************************************************************
/* Function :  CTR_TestBusGet_Default

  Summary:
    Implements Default variant of PLIB_CTR_TestBusGet 

  Description:
    This template implements the Default variant of the PLIB_CTR_TestBusGet function.
*/

PLIB_TEMPLATE uint32_t CTR_TestBusGet_Default( CTR_MODULE_ID index )
{
    return CTR_TEST_BUS_SEL;
}


#endif /*_CTR_CTRTESTBUSSELECT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

