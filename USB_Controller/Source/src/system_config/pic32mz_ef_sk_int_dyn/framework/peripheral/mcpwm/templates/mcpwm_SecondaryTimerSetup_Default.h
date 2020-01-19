/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_SecondaryTimerSetup_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : SecondaryTimerSetup
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_SecondaryTimerSetup
        PLIB_MCPWM_SecondaryTimerCountRead
        PLIB_MCPWM_ExistsSecondaryTimerSetup

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

#ifndef _MCPWM_SECONDARYTIMERSETUP_DEFAULT_H
#define _MCPWM_SECONDARYTIMERSETUP_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_SecondaryTimerSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_SecondaryTimerSetup 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_SecondaryTimerSetup function.
*/

PLIB_TEMPLATE void MCPWM_SecondaryTimerSetup_Default( MCPWM_MODULE_ID index , MCPWM_CLOCK_DIVIDER clock_div , uint16_t period_value )
{
    volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
	module_regs->STCON.SCLKDIV = clock_div;
	module_regs->STPER = period_value;
}


//******************************************************************************
/* Function :  MCPWM_SecondaryTimerCountRead_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_SecondaryTimerCountRead 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_SecondaryTimerCountRead function.
*/

PLIB_TEMPLATE uint16_t MCPWM_SecondaryTimerCountRead_Default( MCPWM_MODULE_ID index )
{
    volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
	uint16_t secondary_tmr = module_regs->SMTMR;
	return secondary_tmr;
}


//******************************************************************************
/* Function :  MCPWM_ExistsSecondaryTimerSetup_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsSecondaryTimerSetup

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsSecondaryTimerSetup function.
*/

#define PLIB_MCPWM_ExistsSecondaryTimerSetup PLIB_MCPWM_ExistsSecondaryTimerSetup
PLIB_TEMPLATE bool MCPWM_ExistsSecondaryTimerSetup_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_SECONDARYTIMERSETUP_DEFAULT_H*/

/******************************************************************************
 End of File
*/

