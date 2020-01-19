/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ModuleControl_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ModuleControl
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_Enable
        PLIB_MCPWM_Disable
        PLIB_MCPWM_StopInIdleEnable
        PLIB_MCPWM_StopInIdleDisable
        PLIB_MCPWM_ModuleIsReady
        PLIB_MCPWM_ExistsModuleControl

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

#ifndef _MCPWM_MODULECONTROL_DEFAULT_H
#define _MCPWM_MODULECONTROL_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_Enable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_Enable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_Enable function.
*/

PLIB_TEMPLATE void MCPWM_Enable_Default( MCPWM_MODULE_ID index )
{
    volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;

    module_regs->PTCONSET = _PTCON_PTEN_MASK;
}


//******************************************************************************
/* Function :  MCPWM_Disable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_Disable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_Disable function.
*/

PLIB_TEMPLATE void MCPWM_Disable_Default( MCPWM_MODULE_ID index )
{
       volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;

    module_regs->PTCONCLR = _PTCON_PTEN_MASK;
}


//******************************************************************************
/* Function :  MCPWM_StopInIdleEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_StopInIdleEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_StopInIdleEnable function.
*/

PLIB_TEMPLATE void MCPWM_StopInIdleEnable_Default( MCPWM_MODULE_ID index )
{
   volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;

    module_regs->PTCONSET = _PTCON_PTSIDL_MASK;
}


//******************************************************************************
/* Function :  MCPWM_StopInIdleDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_StopInIdleDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_StopInIdleDisable function.
*/

PLIB_TEMPLATE void MCPWM_StopInIdleDisable_Default( MCPWM_MODULE_ID index )
{
    volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;

    module_regs->PTCONCLR = _PTCON_PTSIDL_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ModuleIsReady_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ModuleIsReady 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ModuleIsReady function.
*/

PLIB_TEMPLATE bool MCPWM_ModuleIsReady_Default( MCPWM_MODULE_ID index )
{
    volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
	if(module_regs->PTCON.PWMRDY == 0b1)
		return true;
	else
		return false;
}


//******************************************************************************
/* Function :  MCPWM_ExistsModuleControl_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsModuleControl

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsModuleControl function.
*/

#define PLIB_MCPWM_ExistsModuleControl PLIB_MCPWM_ExistsModuleControl
PLIB_TEMPLATE bool MCPWM_ExistsModuleControl_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_MODULECONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

