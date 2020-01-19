/*******************************************************************************
  Device Control System Service Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    sys_devcon.c

  Summary:
    Device Control System Service implementation.

  Description:
    The DEVCON system service provides a simple interface to manage the Device 
    Control module on Microchip microcontrollers. This file Implements the core
    interface routines for the Device Control system service. While building 
    the system service from source, ALWAYS include this file in the build.
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
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "sys_devcon_local.h"
#include "peripheral/int/plib_int.h"
#include "peripheral/osc/plib_osc.h"
#include "peripheral/pcache/plib_pcache.h"
#include "peripheral/bmx/plib_bmx.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Variable Definitions
// *****************************************************************************
// *****************************************************************************
//
#ifndef PLIB_PCACHE_PREFETCH_ENABLE_ALL
#define PLIB_PCACHE_PREFETCH_ENABLE_ALL 3
#endif

// *****************************************************************************
/* Function:
    void SYS_DEVCON_PerformanceConfig( void )

  Summary:
    Configures the PFM wait states and prefetch (cache) module for maximum 
    performance.

  Description:
    This function configures the PFM wait states and prefetch (cache) module 
    for maximum performance.

  Remarks:
    None.
*/

void __attribute__((nomips16)) SYS_DEVCON_PerformanceConfig( unsigned int sysclk )
{
    bool int_flag = false;
    register unsigned long tmp = 0;

    /* Set kseg0 coherency algorithm to "cacheable, non-coherent, write-back, 
     * write-allocate. This is needed for the prefetch buffer */
    asm("mfc0 %0,$16,0" :  "=r"(tmp));
    tmp = (tmp & ~7) | 3;
    asm("mtc0 %0,$16,0" :: "r" (tmp));

    /* Set the PFM wait states based on the system clock */
    #if defined(PLIB_PCACHE_ExistsWaitState)
    if (PLIB_PCACHE_ExistsWaitState(PCACHE_ID_0))
    {
        int ws; /* number of wait states */
        if (sysclk <= 30000000)
            ws = 0;
        else if (sysclk <= 60000000)
            ws = 1;
        else if (sysclk <= 80000000)
            ws = 2;
        else
            ws = 3;
        /* Interrupts must be disabled when changing wait states */
        int_flag = (bool)(PLIB_INT_GetStateAndDisable( INT_ID_0 ) & 0x01);

        PLIB_PCACHE_WaitStateSet(PCACHE_ID_0, ws);

        if (int_flag)
        {
            PLIB_INT_Enable(INT_ID_0);
            int_flag = false;
        }
    }
    #endif // defined(PLIB_PCACHE_ExistsWaitState)

    /* Interrupts must be disabled when enabling the Prefetch Cache Module */
    int_flag = (bool)(PLIB_INT_GetStateAndDisable( INT_ID_0 ) & 0x01);

    /* Enable Prefetch Cache Module */
    #if defined(PLIB_PCACHE_ExistsPrefetchEnable)
    if (PLIB_PCACHE_ExistsPrefetchEnable(PCACHE_ID_0))
    {
        PLIB_PCACHE_PrefetchEnableSet(PCACHE_ID_0, PLIB_PCACHE_PREFETCH_ENABLE_ALL);
    }
    #endif

    /* Set the SRAM wait states to zero */
    #if defined (PLIB_BMX_ExistsDataRamWaitState)
    if (PLIB_BMX_ExistsDataRamWaitState(BMX_ID_0))
    {
        PLIB_BMX_DataRamWaitStateSet(BMX_ID_0, PLIB_BMX_DATA_RAM_WAIT_ZERO);
    }            
    #endif
    if (int_flag)
    {
        PLIB_INT_Enable(INT_ID_0);
    }
}

/*******************************************************************************
 End of File
*/

