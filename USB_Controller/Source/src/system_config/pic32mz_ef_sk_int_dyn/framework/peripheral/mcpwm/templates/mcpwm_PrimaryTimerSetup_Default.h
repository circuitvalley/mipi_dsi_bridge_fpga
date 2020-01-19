/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_PrimaryTimerSetup_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PrimaryTimerSetup
    and its Variant : Default
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

#ifndef _MCPWM_PRIMARYTIMERSETUP_DEFAULT_H
#define _MCPWM_PRIMARYTIMERSETUP_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_PrimaryTimerSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_PrimaryTimerSetup 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_PrimaryTimerSetup function.
*/

PLIB_TEMPLATE void MCPWM_PrimaryTimerSetup_Default( MCPWM_MODULE_ID index , MCPWM_CLOCK_DIVIDER clock_div , uint16_t period_value )
{
    volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
	module_regs->PTCON.PCLKDIV = clock_div;
	module_regs->PTPER = period_value;
}


//******************************************************************************
/* Function :  MCPWM_PrimaryTimerCountRead_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_PrimaryTimerCountRead 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_PrimaryTimerCountRead function.
*/

PLIB_TEMPLATE uint16_t MCPWM_PrimaryTimerCountRead_Default( MCPWM_MODULE_ID index )
{
    volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
	uint16_t primary_tmr = module_regs->PMTMR;
	return primary_tmr;
}


//******************************************************************************
/* Function :  MCPWM_ExistsPrimaryTimerSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsPrimaryTimerSetup

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsPrimaryTimerSetup function.
*/

#define PLIB_MCPWM_ExistsPrimaryTimerSetup PLIB_MCPWM_ExistsPrimaryTimerSetup
PLIB_TEMPLATE bool MCPWM_ExistsPrimaryTimerSetup_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_PRIMARYTIMERSETUP_DEFAULT_H*/

/******************************************************************************
 End of File
*/

