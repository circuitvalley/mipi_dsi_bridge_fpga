/*******************************************************************************
  SYS CLK Functions for Clock System Service

  Company:
    Microchip Technology Inc.

  File Name:
    sys_clk.c

  Summary:
    SYS CLK function implementations for the Clock System Service.

  Description:
    The Clock System Service provides a simple interface to manage the oscillators
    on Microchip microcontrollers. This file defines the implementation for the 
    Clock System Service.
    
  Remarks:
    The functions incorporate all system clock configuration settings as
    determined by the user via the Microchip Harmony Configurator GUI.  
	
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "peripheral/osc/plib_osc.h"
#include "system/devcon/sys_devcon.h"
#include "framework/system/clk/src/sys_clk_local.h"

SYS_CLK_OBJECT clkObject;

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void SYS_CLK_Initialize ( const SYS_CLK_INIT * clkInit  )

  Summary:
    Initializes hardware and internal data structure of the System Clock.

  Description:
    This function initializes the hardware and internal data structure of System
    Clock Service.

  Remarks:
    None. 
*/

void SYS_CLK_Initialize( const SYS_CLK_INIT * clkInit )
{
    SYS_DEVCON_SystemUnlock ( );

    PLIB_OSC_FRCDivisorSelect( OSC_ID_0, OSC_FRC_DIV_1);


    /* Enable Peripheral Bus 1 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_1, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_1 );

    /* Enable Peripheral Bus 2 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_2, 2 );  //USART I2C PMP SPI on PBCLK2
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_2 );
    /* Enable Peripheral Bus 3 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_3, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_3 );
    /* Enable Peripheral Bus 4 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_4, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_4 );
    /* Enable Peripheral Bus 5 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_5, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_5 );
    /* Enable Peripheral Bus 7 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_7, 1 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_7 );
    /* Enable Peripheral Bus 8 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_8, 1 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_8 );
  
 

       /* Disable REFCLKO1*/
    PLIB_OSC_ReferenceOscDisable ( OSC_ID_0, OSC_REFERENCE_1 );
    /* Disable REFCLK1_OE*/
    PLIB_OSC_ReferenceOutputDisable ( OSC_ID_0, OSC_REFERENCE_1 );
    /* Disable REFCLKO2*/
    PLIB_OSC_ReferenceOscDisable ( OSC_ID_0, OSC_REFERENCE_2 );
    /* Disable REFCLK2_OE*/
    PLIB_OSC_ReferenceOutputDisable ( OSC_ID_0, OSC_REFERENCE_2 );
    /* Disable REFCLKO3*/
    PLIB_OSC_ReferenceOscDisable ( OSC_ID_0, OSC_REFERENCE_3 );
    /* Disable REFCLK3_OE*/
    PLIB_OSC_ReferenceOutputDisable ( OSC_ID_0, OSC_REFERENCE_3 );
    /* Disable REFCLKO4*/
    PLIB_OSC_ReferenceOscDisable ( OSC_ID_0, OSC_REFERENCE_4 );
    /* Disable REFCLK4_OE*/
    PLIB_OSC_ReferenceOutputDisable ( OSC_ID_0, OSC_REFERENCE_4 );

    
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, 0, 1 );
    PLIB_OSC_ReferenceOscDisable( OSC_ID_0, OSC_REFERENCE_3 );
    
    PLIB_OSC_ReferenceOscDivisorValueSet(OSC_ID_0, OSC_REFERENCE_3, 12); // 192M/2x(12+(0/512)) --> 8Mhz // 192M/2x(4+(409/512)) --> ~20Mhz
    PLIB_OSC_ReferenceOscTrimSet(OSC_ID_0, OSC_REFERENCE_3, 0);
    PLIB_OSC_ReferenceOscBaseClockSelect(OSC_ID_0, OSC_REFERENCE_3, OSC_REF_BASECLOCK_SYSPLLOUT);
    PLIB_OSC_ReferenceOscStopInIdleIsEnabled(OSC_ID_0, OSC_REFERENCE_3);
 
    /* Enable REFCLKO3*/
    PLIB_OSC_ReferenceOscEnable( OSC_ID_0, OSC_REFERENCE_3 );
    /* Enable REFCLK1_OE*/
    PLIB_OSC_ReferenceOutputEnable( OSC_ID_0, OSC_REFERENCE_3 );
    
    
    PLIB_OSC_ReferenceOscDisable( OSC_ID_0, OSC_REFERENCE_1 );
    
    PLIB_OSC_ReferenceOscDivisorValueSet(OSC_ID_0, OSC_REFERENCE_1, 12); // 192M/2x(12+(0/512)) --> 8Mhz // 192M/2x(4+(409/512)) --> ~20Mhz
    PLIB_OSC_ReferenceOscTrimSet(OSC_ID_0, OSC_REFERENCE_1, 0);
    PLIB_OSC_ReferenceOscBaseClockSelect(OSC_ID_0, OSC_REFERENCE_1, OSC_REF_BASECLOCK_SYSPLLOUT);
    PLIB_OSC_ReferenceOscStopInIdleIsEnabled(OSC_ID_0, OSC_REFERENCE_1);
 
    /* Enable REFCLKO3*/
    PLIB_OSC_ReferenceOscEnable( OSC_ID_0, OSC_REFERENCE_1 );
    /* Enable REFCLK1_OE*/
    PLIB_OSC_ReferenceOutputEnable( OSC_ID_0, OSC_REFERENCE_1 );
    

    SYS_DEVCON_SystemLock ( );
	
	clkObject.systemClock = _SYS_CLK_SystemClockRead ();
	clkObject.callback = NULL;
}

/******************************************************************************
  Function:
    void SYS_CLK_ClockFailureCallbackRegister ( SYS_CLK_ERROR_HANDLER callback )

  Summary:
    Registers the call back function that will be triggered on a clock failure.

  Description:
    This function registers the call back function that will be triggered on a 
    clock failure.

  Remarks:
    None.
*/

void SYS_CLK_ClockFailureCallbackRegister ( SYS_CLK_ERROR_HANDLER callback )
{
    clkObject.callback = callback;
}

/******************************************************************************
  Function:
    uint32_t SYS_CLK_SystemFrequencySet ( CLK_SOURCES_SYSTEM systemSource,
				uint32_t systemClockHz, bool waitUntilComplete )

  Summary:
    Sets the system clock of the device to the value specified.
    
  Description:
    This function configures the clock multipliers and divisors to achieve requested
    System clock frequency. Initially it checks the difference between the requested
    value and possible value. If it is not within 'SYS_CLK_CONFIG_FREQ_ERROR_LIMIT',
    the registers values will not be changed and a value '0' will be returned to let
    user know that the operation was unsuccessful. If the value requested is acceptable,
    then it writes to the oscillator registers and return with the newly set frequency.
    If the operation is a failure, SYS_CLK_SystemClosestFrequencyGet function will give
    the closest possible frequency. If the closest possible value is acceptable, user
    can reconfigure with that value.

  Remarks:
    None.
*/

uint32_t SYS_CLK_SystemFrequencySet ( CLK_SOURCES_SYSTEM systemSource,
				uint32_t systemClockHz, bool waitUntilComplete )
{
    uint32_t clockClosest = 0;

    if ( _SYS_CLK_SystemClockSet ( systemSource, systemClockHz,
                waitUntilComplete, &clockClosest ) )
    {
        return clkObject.systemClock;
    }
    else 
    {
        /* Update the closest in any case */
        clkObject.systemClosestClock = clockClosest;

        return 0;
    }
}

/******************************************************************************
  Function:
    uint32_t SYS_CLK_SystemFrequencyGet ( void )

  Summary:
    Gets the system clock frequency in Hertz.

  Description:
    This function gets the System clock frequency in Hertz.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    System clock frequency in Hertz.

  Example:
    <code>
    uint32_t sysClockHz;

    sysClockHz = SYS_CLK_SystemFrequencyGet ( );
    </code>

  Remarks:
 */

uint32_t SYS_CLK_SystemFrequencyGet ( void )
{
    return clkObject.systemClock;
}

/******************************************************************************
  Function:
    uint32_t SYS_CLK_PeripheralFrequencySet ( CLK_BUSES_PERIPHERAL peripheralBus,
                CLK_SOURCES_PERIPHERAL peripheralSource, uint32_t peripheralClockHz,
                bool waitUntilComplete )

  Summary:
    Configures the Peripheral clock of the device to the value specified.

  Description:
    This function configures the clock multipliers and divisors to achieve requested
    Peripheral clock frequency. Initially it checks the difference between the requested
    value and possible value. If it is not within 'SYS_CLK_CONFIG_FREQ_ERROR_LIMIT',
    the registers values will not be changed and a value '0' will be returned to 
    let user know that the operation was unsuccessful. If the value requested is
    acceptable, then it writes to the oscillator registers and return with the newly
    set frequency. If the operation is a failure, SYS_CLK_PeripheralClosestFrequencyGet
    function will give the closest possible frequency. If the closest possible value
    is acceptable, user can reconfigure with that value.

  Remarks:
    None.
*/

uint32_t SYS_CLK_PeripheralFrequencySet ( CLK_BUSES_PERIPHERAL peripheralBus,
            CLK_SOURCES_PERIPHERAL peripheralSource, uint32_t peripheralClockHz,
            bool waitUntilComplete )
{
    uint32_t clockClosest = 0;

    if ( _SYS_CLK_PeripheralFrequencySet ( peripheralBus, peripheralClockHz, &clockClosest ) )
    {
        /* System clock update is successful. Update the data structures */
        clkObject.peripheralClock[peripheralBus] = clockClosest;

        /* Update the closest in any case */
        clkObject.peripheralClosestClock[peripheralBus] = clockClosest;

    }
    else
    {
        /* Update the closest in any case */
        clkObject.peripheralClosestClock[peripheralBus] = clockClosest;

        return 0;
    }
    return clockClosest;
}

/******************************************************************************
  Function:
    uint32_t SYS_CLK_PeripheralFrequencyGet ( CLK_BUSES_PERIPHERAL peripheralBus )

  Summary:
    Gets the selected clock peripheral bus frequency in Hertz.

  Description:
    This function gets the selected peripheral bus clock frequency in Hertz.

  Precondition:
    None.

  Parameters:
	peripheralBus - Peripheral clock bus selection. One of the possible value from
				CLK_BUSES_PERIPHERAL enum. For devices that do not have multiple
				clock channels for peripheral clock, CLK_BUS_PERIPHERAL_1 should be
				the selection.

  Returns:
    Clock frequency in Hertz.

  Example:
    <code>
    unsigned long peripheralClockHz;

    peripheralClockHz = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_5 );
    </code>

  Remarks:
	Most of the devices doesn't have multiple Peripheral clock buses. In that case, 
	pass CLK_USB_PERIPHERAL_1 as the bus number.
 */

uint32_t SYS_CLK_PeripheralFrequencyGet ( CLK_BUSES_PERIPHERAL peripheralBus )
{
	return _SYS_CLK_PeripheralClockRead (peripheralBus, clkObject.systemClock );
}

/******************************************************************************
  Function:
    uint32_t SYS_CLK_ReferenceFrequencySet ( CLK_BUSES_USB usbBus, CLK_SOURCES_USB usbSource,
					uint32_t usbClockHz, bool waitUntilComplete )

  Summary:
    Configures the Reference clock of the device to the value specified.

  Description:
    This function configures the clock multipliers and divisors to achieve requested
    Reference clock frequency. Initially it checks the difference between the requested
    value and possible value. If it is not within 'SYS_CLK_CONFIG_FREQ_ERROR_LIMIT',
    the registers values will not be changed and a value '0' will be returned to let
    user know that the operation was unsuccessful. If the value requested is acceptable,
    then it writes to the oscillator registers and return with the newly set frequency.
    If the operation is a failure, SYS_CLK_USBClosestFrequencyGet function will give
    the closest possible frequency. If the closest possible value is acceptable, user
    can reconfigure with that value.

  Remarks:
    None.
*/

uint32_t SYS_CLK_ReferenceFrequencySet ( CLK_BUSES_REFERENCE referenceBus,
        CLK_SOURCES_REFERENCE referenceSource, uint32_t referenceClockHz,
        bool waitUntilComplete )
{
    uint32_t clockClosest = 0;

    if ( _SYS_CLK_ReferenceFrequencySet ( referenceBus, referenceSource, referenceClockHz,
                waitUntilComplete, &clockClosest ) )
    {
        /* System clock update is successful. Update the data structures */
        clkObject.referenceClock[referenceBus] = clockClosest;

        /* Update the closest with the calculated value */
        clkObject.referenceClosestClock[referenceBus] = clockClosest;

    }
    else
    {
        /* Update the closest in any case */
        clkObject.referenceClosestClock[referenceBus] = clockClosest;

        return 0;
    }
    return clockClosest;
}

/******************************************************************************
  Function:
    uint32_t SYS_CLK_ReferenceFrequencyGet ( CLK_BUSES_REFERENCE referenceBus )

  Summary:
    Gets the selected Reference clock bus frequency in Hertz.

  Description:
    This function gets frequency of the selected Reference clock bus in Hertz.

  Precondition:
    None.

  Parameters:
	referenceBus - Reference clock bus selection. One of the possible value from
				CLK_BUSES_REFERENCE enum. For devices that do not have multiple
				clock channels for Reference clock, CLK_BUS_REFERENCE_1 should be
				the selection.

  Returns:
    Clock frequency in Hz.

  Example:
    <code>
    unsigned long sysClockOutputHz;

    sysClockOutputHz = SYS_CLK_ReferenceFrequencyGet ( CLK_BUS_REFERENCE_3 );
    </code>

  Remarks:
    None.
 */

uint32_t SYS_CLK_ReferenceFrequencyGet ( CLK_BUSES_REFERENCE referenceBus )
{
	return clkObject.referenceClock[referenceBus];
}

/******************************************************************************
  Function:
    void SYS_CLK_TaskError ( void )

  Summary:
    Informs the user on a clock failure by invoking the registered call back
    function.

  Description:
    This function informs the user on a clock failure by invoking the registered
    call back function. This must be called from the Fail Safe Clock Monitor (FSCM)
    interrupt service routine.
    
  Remarks:
    None.
*/

void SYS_CLK_TaskError ( void )
{
    if (clkObject.callback != NULL)
    {
        clkObject.callback ( clkObject.systemClockSource, clkObject.systemClock );
    } 
}

/******************************************************************************
  Function:
    void SYS_CLK_ReferenceClockSetup ( CLK_BUSES_REFERENCE referenceBus,
                                        SYS_CLK_REFERENCE_SETUP *refSetup )

  Summary:
    Initializes the Reference clock module. 

  Description:
    This function initializes the Reference clock module.
 *
  Remarks:
    None.
*/

void SYS_CLK_ReferenceClockSetup ( CLK_BUSES_REFERENCE referenceBus,
                                        SYS_CLK_REFERENCE_SETUP *refSetup )
{
    _SYS_CLK_ReferenceClockSetup(referenceBus, refSetup);
}

/******************************************************************************
  Function:
    void SYS_CLK_FRCTune ( SYS_CLK_FRC_TUNING_TYPE tuningData )

  Summary:
    Triggers the direct value-based FRC oscillator tuning.

  Description:
    This function tunes the FRC as per the given value. FRC
    tuning functionality has been provided to help customers compensate for
    temperature effects on the FRC frequency over a wide range of temperatures.

  Remarks:
    None.
*/

void SYS_CLK_FRCTune ( SYS_CLK_FRC_TUNING_TYPE tuningData )
{
    PLIB_OSC_FRCTuningSelect(OSC_ID_0, tuningData);
}

/******************************************************************************
  Function:
    void SYS_CLK_SecondaryOscillatorEnable ( void )

  Summary:
    Enables the secondary oscillator.

  Description:
    This function enables the secondary oscillator.

  Remarks:
    For more details refer sys_clk.h.
*/

void SYS_CLK_SecondaryOscillatorEnable ( void )
{
    /* Check for secondary oscillator status */
    if (!PLIB_OSC_SecondaryIsEnabled(OSC_ID_0))
    {    
        /* Unlock and enable secondary oscillator */
        SYS_DEVCON_SystemUnlock();
        
        PLIB_OSC_SecondaryEnable(OSC_ID_0);
        
        SYS_DEVCON_SystemLock();
    }
}

/******************************************************************************
  Function:
    void SYS_CLK_SecondaryOscillatorDisable ( void )

  Summary:
    Disables the secondary oscillator.

  Description:
    This function disables the secondary oscillator.

  Remarks:
    For more details refer sys_clk.h.
*/

void SYS_CLK_SecondaryOscillatorDisable ( void )
{
    /* Check for secondary oscillator status */
    if (PLIB_OSC_SecondaryIsEnabled(OSC_ID_0))
    {    
        /* Unlock and disable secondary oscillator*/
        SYS_DEVCON_SystemUnlock();
        
        PLIB_OSC_SecondaryDisable(OSC_ID_0);
        
        SYS_DEVCON_SystemLock();
    }
}

/******************************************************************************
  Function:
    bool SYS_CLK_SecondaryOscillatorIsEnabled ( void )

  Summary:
    Identifies whether secondary oscillator is enabled or disabled.

  Description:
    This function identifies whether the secondary oscillator is enabled or 
    disabled.
    
  Remarks:
    For more details refer sys_clk.h.
*/

bool SYS_CLK_SecondaryOscillatorIsEnabled ( void )
{
    return (PLIB_OSC_SecondaryIsEnabled(OSC_ID_0));
}
