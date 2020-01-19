/*******************************************************************************
  MCPWM Peripheral Library Template Implementation

  File Name:
    mcpwm_ChannelGeneralFunctions_Default.h

  Summary:
    MCPWM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelGeneralFunctions
    and its Variant : Default
    For following APIs :
        PLIB_MCPWM_ChannelLocalPWMTimerCountRead
        PLIB_MCPWM_IOCONxUnlock
        PLIB_MCPWM_ChannelTimerDirectionGet
        PLIB_MCPWM_ChannelPWMxHEnable
        PLIB_MCPWM_ChannelPWMxHDisable
        PLIB_MCPWM_ChannelPWMxLEnable
        PLIB_MCPWM_ChannelPWMxLDisable
        PLIB_MCPWM_ChannelSwapHighLowEnable
        PLIB_MCPWM_ChannelSwapHighLowDisable
        PLIB_MCPWM_ExistsChannelGeneralFunctions

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

#ifndef _MCPWM_CHANNELGENERALFUNCTIONS_DEFAULT_H
#define _MCPWM_CHANNELGENERALFUNCTIONS_DEFAULT_H
#include "mcpwm_registers.h"

//******************************************************************************
/* Function :  MCPWM_ChannelLocalPWMTimerCountRead_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelLocalPWMTimerCountRead 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelLocalPWMTimerCountRead function.
*/

PLIB_TEMPLATE uint16_t MCPWM_ChannelLocalPWMTimerCountRead_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
   volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	uint16_t local_tmr = channel_regs->PTMRx;
	return local_tmr;
}


//******************************************************************************
/* Function :  MCPWM_IOCONxUnlock_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_IOCONxUnlock 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_IOCONxUnlock function.
*/

PLIB_TEMPLATE void MCPWM_IOCONxUnlock_Default( MCPWM_MODULE_ID index )
{
     volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
	module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
}


//******************************************************************************
/* Function :  MCPWM_ChannelTimerDirectionGet_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelTimerDirectionGet 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelTimerDirectionGet function.
*/

PLIB_TEMPLATE MCPWM_TIMER_DIRECTION MCPWM_ChannelTimerDirectionGet_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
     volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	if(channel_regs->PWMCONx.PTDIR == 0b1)
		return MCPWM_TIMER_DECREMENTING;
	else
		return MCPWM_TIMER_INCREMENTING;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxHEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPWMxHEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPWMxHEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxHEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
	module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
	channel_regs->IOCONxSET = IOCONx_PENH_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxHDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPWMxHDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPWMxHDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxHDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
    module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
	channel_regs->IOCONxCLR = IOCONx_PENH_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxLEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPWMxLEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPWMxLEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxLEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
    module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
	channel_regs->IOCONxSET = IOCONx_PENL_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelPWMxLDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelPWMxLDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelPWMxLDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelPWMxLDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	volatile mcpwm_module_registers_t *module_regs = (mcpwm_module_registers_t *)index;
    module_regs->PWMKEY = 0xABCD;
	module_regs->PWMKEY = 0x4321;
	channel_regs->IOCONxCLR = IOCONx_PENL_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelSwapHighLowEnable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelSwapHighLowEnable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelSwapHighLowEnable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelSwapHighLowEnable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
    volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->IOCONxSET = IOCONx_SWAP_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ChannelSwapHighLowDisable_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ChannelSwapHighLowDisable 

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ChannelSwapHighLowDisable function.
*/

PLIB_TEMPLATE void MCPWM_ChannelSwapHighLowDisable_Default( MCPWM_MODULE_ID index , MCPWM_CHANNEL_ID channel )
{
volatile mcpwm_channel_registers_t *channel_regs = (mcpwm_channel_registers_t *)channel;
	channel_regs->IOCONxCLR = IOCONx_SWAP_MASK;
}


//******************************************************************************
/* Function :  MCPWM_ExistsChannelGeneralFunctions_Default

  Summary:
    Implements Default variant of PLIB_MCPWM_ExistsChannelGeneralFunctions

  Description:
    This template implements the Default variant of the PLIB_MCPWM_ExistsChannelGeneralFunctions function.
*/

#define PLIB_MCPWM_ExistsChannelGeneralFunctions PLIB_MCPWM_ExistsChannelGeneralFunctions
PLIB_TEMPLATE bool MCPWM_ExistsChannelGeneralFunctions_Default( MCPWM_MODULE_ID index )
{
    return true;
}


#endif /*_MCPWM_CHANNELGENERALFUNCTIONS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

