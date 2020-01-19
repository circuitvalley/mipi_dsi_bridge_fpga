/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_HighSpeedSupport_Default.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : HighSpeedSupport
    and its Variant : Default
    For following APIs :
        PLIB_USBHS_HighSpeedEnable
        PLIB_USBHS_HighSpeedDisable
        PLIB_USBHS_HighSpeedIsConnected
        PLIB_USBHS_FullOrHighSpeedIsConnected
        PLIB_USBHS_ExistsHighSpeedSupport
        PLIB_USBHS_DMAErrorGet
        PLIB_USBHS_DMAInterruptGet
        PLIB_USBHS_DMAOperationEnable
        PLIB_USBHS_LoadEPInIndex
        PLIB_USBHS_GetEP0FIFOAddress
        PLIB_USBHS_GetEP0CSRAddress
        PLIB_USBHS_GetReceiveDataCount
        PLIB_USBHS_TestModeEnter
        PLIB_USBHS_TestModeExit

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

#ifndef _USBHS_HIGHSPEEDSUPPORT_DEFAULT_H
#define _USBHS_HIGHSPEEDSUPPORT_DEFAULT_H

#include <sys/kmem.h>
#include "usbhs_registers.h"

/* This table maps the Test Mode values defined by the USB 2.0 specification
 * to values that can be written directly to the Test mode register. So for
 * example, if the test mode is Test_K, the specification value for this is 
 * 0x02 and the bit position in Testmode register is D2. Hence 
 * PLIB_USBHS_TestModeMapping[2] = 0x4 (bit D2 is set). */


//******************************************************************************
/* Function :  USBHS_HighSpeedEnable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_HighSpeedEnable 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_HighSpeedEnable function.
*/

PLIB_TEMPLATE void USBHS_HighSpeedEnable_Default( USBHS_MODULE_ID index )
{
    /* This function enables high speed support */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->POWERbits.HSEN = 1;
}

//******************************************************************************
/* Function :  USBHS_HighSpeedDisable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_HighSpeedDisable 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_HighSpeedDisable function.
*/

PLIB_TEMPLATE void USBHS_HighSpeedDisable_Default( USBHS_MODULE_ID index )
{
    /* This function disables high speed support */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->POWERbits.HSEN = 0;
}

//******************************************************************************
/* Function :  USBHS_HighSpeedIsConnected_Default

  Summary:
    Implements Default variant of PLIB_USBHS_HighSpeedIsConnected 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_HighSpeedIsConnected function.
*/

PLIB_TEMPLATE bool USBHS_HighSpeedIsConnected_Default( USBHS_MODULE_ID index )
{
    /* Returns true if the connected device is high speed
     * else returns false. */

    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    return((bool)(usbhs->POWERbits.HSMODE));
}

//******************************************************************************
/* Function :  USBHS_DMAErrorGet_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DMAErrorGet 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DMAErrorGet function.
*/

PLIB_TEMPLATE bool USBHS_DMAErrorGet_Default
( 
    USBHS_MODULE_ID index, 
    uint8_t dmaChannel 
)
{
    /* Returns true the specified DMA channel is in an error condition
     * and then clears the error flag. */
    
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    bool result = false;
    
    if( usbhs->DMA_CHANNEL[dmaChannel - 1].DMACNTLbits.DMAERR == 1)
    {
        /* Clear the error flag*/
        result = true;
        usbhs->DMA_CHANNEL[dmaChannel - 1].DMACNTLbits.DMAERR = 0;
        
    }
  
    return result;
}

//******************************************************************************
/* Function :  USBHS_DMAInterruptGet_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DMAInterruptGet 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DMAInterruptGet function.
*/

PLIB_TEMPLATE uint8_t USBHS_DMAInterruptGet_Default( USBHS_MODULE_ID index)
{
    /* Returns the status the DMA channel interrupts. Calling this function
     * also clears the interrupt flags. */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    volatile uint8_t * dmaInterruptRegAddress = (uint8_t *)(&usbhs->DMA_INTR);
    return (*dmaInterruptRegAddress);
}

//******************************************************************************
/* Function :  USBHS_DMAOperationEnable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DMAOperationEnable

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DMAOperationEnable function.
*/

PLIB_TEMPLATE void USBHS_DMAOperationEnable_Default
(
    USBHS_MODULE_ID index,
    uint8_t endpoint,
    uint8_t dmaChannel,
    void * address,
    uint32_t count,
    bool direction
)
{
    /* Configures the DMA channel for specified endpoint and transfer 
     * direction */
    
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
   
    /* DMA ADDR register program */
    usbhs->DMA_CHANNEL[dmaChannel - 1].DMAADDR = KVA_TO_PA(address);
   
    /* DMA COUNT register program */
    usbhs->DMA_CHANNEL[dmaChannel - 1].DMACOUNT = (uint32_t)(count);
    
    /* Set up the rest of the DMA channel */
    usbhs->DMA_CHANNEL[dmaChannel - 1].DMACNTLbits.DMABRSTM = 3;
    usbhs->DMA_CHANNEL[dmaChannel - 1].DMACNTLbits.DMAIE = 1;
    usbhs->DMA_CHANNEL[dmaChannel - 1].DMACNTLbits.DMAMODE = 0;
	usbhs->DMA_CHANNEL[dmaChannel - 1].DMACNTLbits.DMAEP = endpoint;
    
    /* The input direction bit is a complement of the DMA channel direction
     * bit. */
    usbhs->DMA_CHANNEL[dmaChannel - 1].DMACNTLbits.DMADIR = !direction;
    usbhs->DMA_CHANNEL[dmaChannel - 1].DMACNTLbits.DMAEN = 1;

}

//******************************************************************************
/* Function :  USBHS_LoadEPInIndex_Default

  Summary:
    Implements Default variant of PLIB_USBHS_LoadEPInIndex 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_LoadEPInIndex function.
*/

PLIB_TEMPLATE void USBHS_LoadEPInIndex_Default
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
)
{
    /* Load the index register. This defines which endpoint control status
     * register set appears in the EPCSR window. */
    
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->INDEXbits.ENDPOINT = endpoint; 
}

//******************************************************************************
/* Function :  USBHS_GetEP0FIFOAddress_Default

  Summary:
    Implements Default variant of PLIB_USBHS_GetEP0FIFOAddress 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_GetEP0FIFOAddress function.
*/

PLIB_TEMPLATE uint8_t * USBHS_GetEP0FIFOAddress_Default(USBHS_MODULE_ID index)
{
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    return ((uint8_t *)(&usbhs->FIFO[0]));
}

//******************************************************************************
/* Function :  USBHS_GetEP0CSRAddress_Default

  Summary:
    Implements Default variant of PLIB_USBHS_GetEP0CSRAddress 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_GetEP0CSRAddress function.
*/

PLIB_TEMPLATE uint8_t * USBHS_GetEP0CSRAddress_Default(USBHS_MODULE_ID index)
{
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    return ((uint8_t *)(&usbhs->INDEXED_EPCSR.CSR0L_DEVICEbits));
}

//******************************************************************************
/* Function :  USBHS_GetReceiveDataCount_Default

  Summary:
    Implements Default variant of PLIB_USBHS_GetReceiveDataCount 

  Description:
    This template implements the Default variant of the PLIB_USBHS_GetReceiveDataCount function.
*/

PLIB_TEMPLATE uint32_t USBHS_GetReceiveDataCount_Default
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
)
{
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    uint32_t count;
    
    count = usbhs->EPCSR[endpoint].RXCOUNTbits.w;
    return(count);
}


//******************************************************************************
/* Function :  USBHS_FullOrHighSpeedIsConnected_Default

  Summary:
    Implements Default variant of PLIB_USBHS_FullOrHighSpeedIsConnected 

  Description:
    This template implements the Default variant of the PLIB_USBHS_FullOrHighSpeedIsConnected function.
*/

PLIB_TEMPLATE bool USBHS_FullOrHighSpeedIsConnected_Default( USBHS_MODULE_ID index )
{
    /* Function returns true is high or full speed device is connected.
     * Return false if low speed or no device is connected */

    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    bool result = false;
    
    if(usbhs->DEVCTLbits.FSDEV == 1)
    {
        result = true;
    }
    
    if(usbhs->DEVCTLbits.LSDEV == 1)
    {
        result = false;
    }
    
    return result;
}

//******************************************************************************
/* Function :  USBHS_ExistsHighSpeedSupport_Default

  Summary:
    Implements Default variant of PLIB_USBHS_ExistsHighSpeedSupport

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_ExistsHighSpeedSupport function.
*/

#define PLIB_USBHS_ExistsHighSpeedSupport PLIB_USBHS_ExistsHighSpeedSupport
PLIB_TEMPLATE bool USBHS_ExistsHighSpeedSupport_Default( USBHS_MODULE_ID index )
{
    return true;
}

//******************************************************************************
/* Function :  USBHS_TestModeEnter_Default

  Summary:
    Implements Default variant of PLIB_USBHS_TestModeEnter 

  Description:
    This template implements the Default variant of the PLIB_USBHS_TestModeEnter function.
*/

PLIB_TEMPLATE bool USBHS_TestModeEnter_Default
( 
    USBHS_MODULE_ID index, 
    uint8_t testMode 
)
{
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    bool result = false;
	
	uint8_t PLIB_USBHS_TestModeMapping[5] = 
    {
        0x0, /* Not used */ 
        0x2, /* Test_J */ 
        0x4, /* Test_K */
        0x1, /* Test_SE0_NAK */
        0x8  /* Test Packet */
    };
    
    if((usbhs->TESTMODEbits.w & 0x7F) == 0x0)
    {
        /* We can proceed only if bits D6-D0 are are all 0. Now check if 
         * testMode is not zero and is not greater than 5. This is not a valid test 
         * mode */
        
        if((testMode != 0) && (testMode < 5))
        {
            usbhs->TESTMODEbits.w |= PLIB_USBHS_TestModeMapping[testMode];
            result = true;
        }
    }
  
	return result;
}


//******************************************************************************
/* Function :  USBHS_TestModeExit_Default

  Summary:
    Implements Default variant of PLIB_USBHS_TestModeExit 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_TestModeExit function.
*/

PLIB_TEMPLATE bool USBHS_TestModeExit_Default( USBHS_MODULE_ID index , uint8_t testMode )
{
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    bool result = false;
	
	uint8_t PLIB_USBHS_TestModeMapping[5] = 
    {
        0x0, /* Not used */ 
        0x2, /* Test_J */ 
        0x4, /* Test_K */
        0x1, /* Test_SE0_NAK */
        0x8  /* Test Packet */
    };
    
    /* We can proceed only if bits D6-D0 are are all 0. Now check if 
     * testMode is not zero and is not greater than 5. This is not a valid test 
     * mode */

    if((testMode != 0) && (testMode < 5))
    {
        usbhs->TESTMODEbits.w &= (~(PLIB_USBHS_TestModeMapping[testMode]));
        result = true;
    }
   
	return result;
}


#endif /*_USBHS_HIGHSPEEDSUPPORT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

