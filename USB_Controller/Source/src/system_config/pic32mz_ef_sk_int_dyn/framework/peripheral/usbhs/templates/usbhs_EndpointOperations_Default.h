/*******************************************************************************
  USBHS Peripheral Library Template Implementation

  File Name:
    usbhs_EndpointOperations_Default.h

  Summary:
    USBHS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : EndpointOperations
    and its Variant : Default
    For following APIs :
        PLIB_USBHS_EndpointRxRequestEnable
        PLIB_USBHS_EndpointRxRequestClear
        PLIB_USBHS_HostRxEndpointConfigure
        PLIB_USBHS_HostTxEndpointConfigure
        PLIB_USBHS_HostTxEndpointDataToggleClear
        PLIB_USBHS_HostRxEndpointDataToggleClear
        PLIB_USBHS_DeviceRxEndpointConfigure
        PLIB_USBHS_DeviceTxEndpointConfigure
        PLIB_USBHS_DeviceRxEndpointStallEnable
        PLIB_USBHS_DeviceTxEndpointStallEnable
        PLIB_USBHS_DeviceRxEndpointStallDisable
        PLIB_USBHS_DeviceTxEndpointStallDisable
        PLIB_USBHS_DeviceTxEndpointPacketReady
        PLIB_USBHS_ExistsEndpointOperations

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

#ifndef _USBHS_ENDPOINTOPERATIONS_DEFAULT_H
#define _USBHS_ENDPOINTOPERATIONS_DEFAULT_H

#include "usbhs_registers.h"

//******************************************************************************
/* Function :  USBHS_EndpointRxRequestEnable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_EndpointRxRequestEnable 

  Description:
    This template implements the Default variant of the
    PLIB_USBHS_EndpointRxRequestEnable function.
*/

PLIB_TEMPLATE void USBHS_EndpointRxRequestEnable_Default
( 
    USBHS_MODULE_ID index, 
    uint8_t endpoint 
)
{
    /* Sets the Receive Packet Request bit causing an IN endpoint to send an IN
     * token. This function is to be called in the host mode operation. */

    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[endpoint].RXCSRL_HOSTbits.REQPKT = 1;
}

//******************************************************************************
/* Function :  USBHS_EndpointRxRequestClear_Default

  Summary:
    Implements Default variant of PLIB_USBHS_EndpointRxRequestClear 

  Description:
    This template implements the Default variant of the
    PLIB_USBHS_EndpointRxRequestClear function.
*/

PLIB_TEMPLATE void USBHS_EndpointRxRequestClear_Default
( 
    USBHS_MODULE_ID index, 
    uint8_t endpoint 
)
{
    /* This function clear the IN Request Packet bit. This function should be
     * called in host mode only. */

    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[endpoint].RXCSRL_HOSTbits.RXPKTRDY = 0;
}

//******************************************************************************
/* Function :  USBHS_HostRxEndpointConfigure_Default

  Summary:
    Implements Default variant of PLIB_USBHS_HostRxEndpointConfigure 

  Description:
    This template implements the Default variant of the
    PLIB_USBHS_HostRxEndpointConfigure function.
*/

PLIB_TEMPLATE void USBHS_HostRxEndpointConfigure_Default
( 
    USBHS_MODULE_ID index, 
    uint8_t hostEndpoint, 
    uint32_t speed, 
    uint32_t pipeType, 
    uint16_t endpointSize, 
    uint16_t receiveFIFOAddress, 
    uint8_t fifoSize, 
    uint8_t  targetEndpoint, 
    uint8_t  targetDevice, 
    uint8_t  targetHub, 
    uint8_t  targetHubPort, 
    uint8_t nakInterval 
)
{
    /* This function sets up the endpoint size, receive FIFO address, receive
     * FIFO size, pipe type, speed, target endpoint, target device target hub
     * and target hub port for host receive endpoint. This function is called by
     * the driver pipe setup function. This function also clears up the receive
     * fifo if there is any stale data, nak interval and then finally enables
     * the interrupt */
    
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    uint8_t indexBackup;

    /* Backup the index register and set it to the host endpoint that is to be
     * configured. The indexedEPControlRegisters will now point to the endpoint
     * to be configured */

    indexBackup = usbhs->INDEXbits.ENDPOINT;
    usbhs->INDEXbits.ENDPOINT = hostEndpoint;

    /* Setup the endpoint size */
    usbhs->INDEXED_EPCSR.RXMAXPbits.RXMAXP = endpointSize;

    /* Setup the receive fifo address */
    usbhs->RXFIFOADDbits.RXFIFOAD = receiveFIFOAddress;

    /* Setup the receive FIFO size */
    usbhs->RXFIFOSZbits.RXFIFOSZ = fifoSize;

    /* Setup the pipe type, speed and target endpoint */
     usbhs->INDEXED_EPCSR.RXTYPEbits.w = ((speed << 6) | (pipeType << 4) | (targetEndpoint));

    /* Setup device address */
    usbhs->TADDR[hostEndpoint].RXFUNCADDRbits.RXFADDR = targetDevice;

    /* Setup target Hub address */
    usbhs->TADDR[hostEndpoint].RXHUBADDRbits.RXHUBADDR = targetHub;

    /* Setup target Hub port */
    usbhs->TADDR[hostEndpoint].RXHUBPORTbits.RXHUBPRT = targetHubPort;

    /* Clear the data toggle */
    usbhs->INDEXED_EPCSR.RXCSRL_HOSTbits.CLRDT = 1;

    /* If there is stale data in the fifo, then flush the fifo */
    if(usbhs->INDEXED_EPCSR.RXCSRL_HOSTbits.RXPKTRDY == 1)
    {
        /* Flush the fifo */
        usbhs->INDEXED_EPCSR.RXCSRL_HOSTbits.FLUSH = 1; 
    }

    /* For bulk pipes setup up the NAK interval and
     * for interrupt pipes set up the transaction 
     * interval */

    usbhs->INDEXED_EPCSR.RXINTERVALbits.RXINTERV = nakInterval;

    /* Enable the RX endpoint interrupt */
    usbhs->INTRRXEbits.w |= (1 << hostEndpoint);

    /* Restore the index register before exiting */
    usbhs->INDEXbits.ENDPOINT = indexBackup;

}

//******************************************************************************
/* Function :  USBHS_HostTxEndpointConfigure_Default

  Summary:
    Implements Default variant of PLIB_USBHS_HostTxEndpointConfigure 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_HostTxEndpointConfigure function.
*/

PLIB_TEMPLATE void USBHS_HostTxEndpointConfigure_Default
(
    USBHS_MODULE_ID index , 
    uint8_t hostEndpoint , 
    uint32_t speed , 
    uint32_t pipeType , 
    uint16_t endpointSize , 
    uint16_t fifoAddress , 
    uint8_t  fifoSize , 
    uint8_t  targetEndpoint , 
    uint8_t  targetDevice , 
    uint8_t  targetHub , 
    uint8_t  targetHubPort,
    uint8_t  nakInterval
)
{
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    uint8_t indexBackup;

    /* Backup the index register and set it to the
     * host endpoint that is to be configured */

    indexBackup = usbhs->INDEXbits.ENDPOINT;
    
    usbhs->INDEXbits.ENDPOINT = hostEndpoint;

    /* Setup the endpoint size */
    usbhs->INDEXED_EPCSR.TXMAXPbits.TXMAXP = endpointSize;

    /* Setup the transmit fifo address */
    usbhs->TXFIFOADDbits.TXFIFOAD = fifoAddress;

    /* Setup the transmit FIFO size */
    usbhs->TXFIFOSZbits.TXFIFOSZ = fifoSize;
 
    /* Setup the pipe type, speed and target endpoint */
    usbhs->INDEXED_EPCSR.TXTYPEbits.w = (( speed << 6) | (pipeType << 4) | (targetEndpoint));

    /* Setup device address */
    usbhs->TADDR[hostEndpoint].TXFUNCADDRbits.TXFADDR = targetDevice;

    /* Setup target Hub address */
    usbhs->TADDR[hostEndpoint].TXHUBADDRbits.TXHUBADDR = targetHub;

    /* Setup target Hub port */
    usbhs->TADDR[hostEndpoint].TXHUBPORTbits.TXHUBPRT = targetHubPort;

    /* Clear the data toggle if it is set */
    usbhs->INDEXED_EPCSR.TXCSRL_HOSTbits.CLRDT = 1;

    /* If there is stale data in the fifo, then flush the fifo */
    if(usbhs->INDEXED_EPCSR.TXCSRL_HOSTbits.FIFONE == 1)
    {
        /* Flush the fifo */
        usbhs->INDEXED_EPCSR.TXCSRL_HOSTbits.FLUSH = 1;
    }

    /* For bulk pipes setup up the NAK interval and
     * for interrupt pipes set up the transaction 
     * interval */

    usbhs->INDEXED_EPCSR.TXINTERVALbits.TXINTERV = nakInterval;

    /* Enable the TX endpoint interrupt */
    usbhs->INTRTXEbits.w |= (1 << hostEndpoint);

    /* Restore the index register before exiting */
    usbhs->INDEXbits.ENDPOINT = indexBackup;

}

//******************************************************************************
/* Function :  USBHS_HostTxEndpointDataToggleClear_Default

  Summary:
    Implements Default variant of PLIB_USBHS_HostTxEndpointDataToggleClear 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_HostTxEndpointDataToggleClear function.
*/

PLIB_TEMPLATE void USBHS_HostTxEndpointDataToggleClear_Default
( 
    USBHS_MODULE_ID index , 
    uint8_t hostEndpoint 
)
{
    /* Clear the Data toggle on the TX endpoint in host mode. */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[hostEndpoint].TXCSRL_HOSTbits.CLRDT = 1;
}

//******************************************************************************
/* Function :  USBHS_HostRxEndpointDataToggleClear_Default

  Summary:
    Implements Default variant of PLIB_USBHS_HostRxEndpointDataToggleClear 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_HostRxEndpointDataToggleClear function.
*/

PLIB_TEMPLATE void USBHS_HostRxEndpointDataToggleClear_Default
( 
    USBHS_MODULE_ID index , 
    uint8_t hostEndpoint 
)
{
    /* Clear the Data toggle on the RX endpoint in host mode. Writing
     * a one to this bit will clear the data toggle. */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[hostEndpoint].RXCSRL_HOSTbits.CLRDT = 1;
}

//******************************************************************************
/* Function :  USBHS_DeviceTxEndpointConfigure_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DeviceTxEndpointConfigure 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DeviceTxEndpointConfigure function.
*/

PLIB_TEMPLATE void USBHS_DeviceTxEndpointConfigure_Default
(
    USBHS_MODULE_ID index,
    uint8_t endpoint,
    uint16_t endpointSize,
    uint16_t fifoAddress,
    uint8_t fifoSize,
    uint32_t transferType
)
{
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    
    uint8_t indexBackup;

    /* This function configures the TX endpoint for device mode
     * operation. Start by setting up the index register */

    indexBackup = usbhs->INDEXbits.ENDPOINT;
        
    usbhs->INDEXbits.ENDPOINT = endpoint;
    
    /* Configure the Endpoint size */
    usbhs->INDEXED_EPCSR.TXMAXPbits.TXMAXP = endpointSize;
    
    /* Set up the fifo address */
    usbhs->TXFIFOADDbits.TXFIFOAD = fifoAddress;
    
    /* Clear the data toggle */
    usbhs->INDEXED_EPCSR.TXCSRL_DEVICEbits.CLRDT = 1;
   
    /* Set up the FIFO size */
    usbhs->TXFIFOSZbits.TXFIFOSZ = fifoSize;
    
    if(1 == transferType)
    {
        /* Enable ISOC operation */
        usbhs->INDEXED_EPCSR.TXCSRH_DEVICEbits.ISO = 1;
    }
    else
    {
        /* Enable handshake */
        usbhs->INDEXED_EPCSR.TXCSRH_DEVICEbits.ISO = 0;
       
    }

    /* Restore the index register */
    usbhs->INDEXbits.ENDPOINT = indexBackup;
  
    /* Enable the interrupt */
    usbhs->INTRTXEbits.w |=  (1 << endpoint);
    
}

//******************************************************************************
/* Function :  USBHS_DeviceRxEndpointConfigure_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DeviceTxEndpointConfigure 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DeviceTxEndpointConfigure function.
*/

PLIB_TEMPLATE void USBHS_DeviceRxEndpointConfigure_Default
(
    USBHS_MODULE_ID index,
    uint8_t endpoint,
    uint16_t endpointSize,
    uint16_t fifoAddress,
    uint8_t fifoSize,
    uint32_t transferType
)
{
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    uint8_t indexBackup;

    /* This function configures the RX endpoint for device mode
     * operation. Start by setting up the index register */

    indexBackup = usbhs->INDEXbits.ENDPOINT;
     
    usbhs->INDEXbits.ENDPOINT = endpoint;
 
    /* Configure the Endpoint size */
    usbhs->INDEXED_EPCSR.RXMAXPbits.RXMAXP = endpointSize;

    /* Set up the fifo address */
    usbhs->RXFIFOADDbits.RXFIFOAD = fifoAddress;
  
    /* Clear the data toggle */
    usbhs->INDEXED_EPCSR.RXCSRL_DEVICEbits.CLRDT = 1;

    /* Set up the FIFO size */
    usbhs->RXFIFOSZbits.RXFIFOSZ = fifoSize;
    
    if(transferType == 1)
    {
        /* For Isochronous endpoints, handshaking must
         * be disabled*/

        usbhs->INDEXED_EPCSR.RXCSRH_DEVICEbits.ISO = 1;

    }
    else if(transferType == 3)
    {
        /* For interrupt endpoints, handshaking must
         * be enabled by NYET should be disabled */

        usbhs->INDEXED_EPCSR.RXCSRH_DEVICEbits.DISNYET = 1;
        usbhs->INDEXED_EPCSR.RXCSRH_DEVICEbits.ISO = 0;
    }
    else
    {
        /* For bulk endpoints, handshaking must
         * enabled and NYET should be enabled */

        usbhs->INDEXED_EPCSR.RXCSRH_DEVICEbits.DISNYET = 0;
        usbhs->INDEXED_EPCSR.RXCSRH_DEVICEbits.ISO = 0;
    }

    /* Restore the index register */
    usbhs->INDEXbits.ENDPOINT = indexBackup;
   
    /* Enable the endpoint interrupt */
    usbhs->INTRRXEbits.w |= (1 << endpoint);
    
}

//******************************************************************************
/* Function :  USBHS_DeviceRxEndpointStallEnable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DeviceTxEndpointStallEnable 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DeviceTxEndpointStallEnable function.
*/

PLIB_TEMPLATE void USBHS_DeviceRxEndpointStallEnable_Default
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
)
{
    /* Stalls the RX direction on specified endpoint. */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[endpoint].RXCSRL_DEVICEbits.SENDSTALL = 1;
}

//******************************************************************************
/* Function :  USBHS_DeviceTxEndpointStallEnable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DeviceTxEndpointStallEnable 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DeviceTxEndpointStallEnable function.
*/

PLIB_TEMPLATE void USBHS_DeviceTxEndpointStallEnable_Default
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
)
{
    /* Stalls the TX direction on specified endpoint. */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[endpoint].TXCSRL_DEVICEbits.SENDSTALL = 1;
}

//******************************************************************************
/* Function :  USBHS_DeviceRxEndpointStallDisable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DeviceTxEndpointStallDisable 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DeviceTxEndpointStallDisable function.
*/

PLIB_TEMPLATE void USBHS_DeviceRxEndpointStallDisable_Default
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
)
{
    /* Disable the stall and reset the data toggle */
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[endpoint].RXCSRL_DEVICEbits.w &= (~(USBHS_EP_DEVICE_RX_SENT_STALL|USBHS_EP_DEVICE_RX_SEND_STALL));
    usbhs->EPCSR[endpoint].RXCSRL_DEVICEbits.CLRDT = 1;
}

//******************************************************************************
/* Function :  USBHS_DeviceTxEndpointStallDisable_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DeviceTxEndpointStallDisable 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DeviceTxEndpointStallDisable function.
*/

PLIB_TEMPLATE void USBHS_DeviceTxEndpointStallDisable_Default
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
)
{
    /* Disable the stall and reset the data toggle */
         
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[endpoint].TXCSRL_DEVICEbits.w &= (~(USBHS_EP_DEVICE_TX_SENT_STALL|USBHS_EP_DEVICE_TX_SEND_STALL));
    usbhs->EPCSR[endpoint].TXCSRL_DEVICEbits.CLRDT = 1;
}

//******************************************************************************
/* Function :  USBHS_DeviceTxEndpointPacketReady_Default

  Summary:
    Implements Default variant of PLIB_USBHS_DeviceTxEndpointPacketReady 

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_DeviceTxEndpointPacketReady function.
*/

PLIB_TEMPLATE void USBHS_DeviceTxEndpointPacketReady_Default
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
)
{
    /* Set the TX Packet Ready bit. */
    
    volatile usbhs_registers_t * usbhs = (usbhs_registers_t *)(index);
    usbhs->EPCSR[endpoint].TXCSRL_DEVICEbits.TXPKTRDY = 1;
}


//******************************************************************************
/* Function :  USBHS_ExistsEndpointOperations_Default

  Summary:
    Implements Default variant of PLIB_USBHS_ExistsEndpointOperations

  Description:
    This template implements the Default variant of the 
    PLIB_USBHS_ExistsEndpointOperations function.
*/

#define PLIB_USBHS_ExistsEndpointOperations PLIB_USBHS_ExistsEndpointOperations
PLIB_TEMPLATE bool USBHS_ExistsEndpointOperations_Default( USBHS_MODULE_ID index )
{
    return true;
}


#endif /*_USBHS_ENDPOINTOPERATIONS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

