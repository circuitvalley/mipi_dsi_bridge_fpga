/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_PrimaryTimerSetup_Unsupported.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PrimaryTimerSetup
    and its Variant : Unsupported
    For following APIs :
        PLIB_MCPWM_PrimaryTimerSetup
        PLIB_MCPWM_PrimaryTimerCountRead
        PLIB_MCPWM_ExistsPrimaryTimerSetup

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

#ifndef _MCPWM_PRIMARYTIMERSETUP_UNSUPPORTED_H
#define _MCPWM_PRIMARYTIMERSETUP_UNSUPPORTED_H

//******************************************************************************
/* Function :  MCPWM_PrimaryTimerSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_PrimaryTimerSetup 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_PrimaryTimerSetup function.
*/

PLIB_TEMPLATE void MCPWM_PrimaryTimerSetup_Unsupported( MCPWM_MODULE_ID index , MCPWM_CLOCK_DIVIDER clock_div , uint16_t period_value )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_PrimaryTimerSetup");
}


//******************************************************************************
/* Function :  MCPWM_PrimaryTimerCountRead_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_PrimaryTimerCountRead 

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_PrimaryTimerCountRead function.
*/

PLIB_TEMPLATE void MCPWM_PrimaryTimerCountRead_Unsupported( MCPWM_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_MCPWM_PrimaryTimerCountRead");
}


//******************************************************************************
/* Function :  MCPWM_ExistsPrimaryTimerSetup_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_MCPWM_ExistsPrimaryTimerSetup

  Description:
    This template implements the Unsupported variant of the PLIB_MCPWM_ExistsPrimaryTimerSetup function.
*/

PLIB_TEMPLATE bool MCPWM_ExistsPrimaryTimerSetup_Unsupported( MCPWM_MODULE_ID index )
{
    return false;
}


#endif /*_MCPWM_PRIMARYTIMERSETUP_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

