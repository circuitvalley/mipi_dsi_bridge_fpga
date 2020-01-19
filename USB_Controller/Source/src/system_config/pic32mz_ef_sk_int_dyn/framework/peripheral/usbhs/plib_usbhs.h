/*******************************************************************************
  Hi-Speed USB (HS USB) Module Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_usbhs.h

  Summary:
    HS USB PLIB Interface Header for definitions common to the Hi-Speed USB
    module.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the USB
    PLIB for families of Microchip microcontrollers that feature the Hi-Speed
    USB module.

  Notes:
    Required to access the High Speed USB Module registers.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2013-2015 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _PLIB_USBHS_H
#define _PLIB_USBHS_H



#include <stdint.h>
#include <stdbool.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued at end of file)
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.  However,
    please see the end of the file for additional implementation header files
    that are also included
*/
#include "peripheral/usbhs/processor/usbhs_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Maximum number of endpoints

  Summary:
    Maximum number of endpoints supported (not including EP0).

  Description:
    This constant defines the maximum number of endpoints supported (not
    including EP0).

  Remarks:
    None.
*/

#define USBHS_MAX_EP_NUMBER 7

// *****************************************************************************
/* Number of DMA Channels

  Summary:
    Number of available DMA Channels.

  Description:
    This constant defines the number of available DMA Channels.

  Remarks:
    None.
*/

#define USBHS_MAX_DMA_CHANNEL_NUMBER 8

// *****************************************************************************
/* USBHS DMA Interrupts Enumeration

  Summary:
    Provides enumeration of interrupts for DMA channels 0-7.

  Description:
    This data type provides the enumeration of interrupts for DMA channels 0-7.

  Remarks:
    None.
*/

typedef enum
{
    USBHS_DMAINT_1   /*DOM-IGNORE-BEGIN*/ =  0x01 /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_2   /*DOM-IGNORE-BEGIN*/ =  0x02 /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_3   /*DOM-IGNORE-BEGIN*/ =  0x04 /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_4   /*DOM-IGNORE-BEGIN*/ =  0x08 /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_5   /*DOM-IGNORE-BEGIN*/ =  0x10 /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_6   /*DOM-IGNORE-BEGIN*/ =  0x20 /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_7   /*DOM-IGNORE-BEGIN*/ =  0x40 /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_8   /*DOM-IGNORE-BEGIN*/ =  0x80 /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_ANY /*DOM-IGNORE-BEGIN*/ =  0xFF /*DOM-IGNORE-END*/ ,
    USBHS_DMAINT_ALL /*DOM-IGNORE-BEGIN*/ =  0xFF /*DOM-IGNORE-END*/

} USBHS_DMA_INTERRUPT;

// *****************************************************************************
/* USBHS Link Power Management (LPM) Interrupts Enumeration

  Summary:
    Provides an enumeration of LPM interrupt sources.

  Description:
    This data type provides an enumeration of LPM interrupt sources.

  Remarks:
    None.
*/

typedef enum
{
    // Host: Device responded with STALL.  Peripheral/Device: Sent a STALL.
    USBHS_LPMINT_STALL        /*DOM-IGNORE-BEGIN*/ =  0x01 /*DOM-IGNORE-END*/ ,

    // Host: Device responded with NYET.  Peripheral/Device: Sent a NYET.
    USBHS_LPMINT_NYET         /*DOM-IGNORE-BEGIN*/ =  0x02 /*DOM-IGNORE-END*/ ,

    // Host: Device responded with ACK.  Peripheral/Device: Sent an ACK.
    USBHS_LPMINT_ACK          /*DOM-IGNORE-BEGIN*/ =  0x04 /*DOM-IGNORE-END*/ ,

    // Host: LPM transaction failed to complete.  Peripheral/Device:  NYET sent because data is pending in RX FIFOs.
    USBHS_LPMINT_NOTCOMPLETE  /*DOM-IGNORE-BEGIN*/ =  0x08 /*DOM-IGNORE-END*/ ,

    // USB Module has resumed operation.
    USBHS_LPMINT_RESUMED      /*DOM-IGNORE-BEGIN*/ =  0x10 /*DOM-IGNORE-END*/ ,

    // Host: Received Bit Stuff or PID error.  Peripheral/Device: Unsupported LinkState field received.
    USBHS_LPMINT_ERROR        /*DOM-IGNORE-BEGIN*/ =  0x20 /*DOM-IGNORE-END*/

} USBHS_LPM_INTERRUPT;

// *****************************************************************************
/* USBHS Endpoint Direction Enumeration

  Summary:
    Used as an argument to identify an endpoint direction.

  Description:
    This data type is used as an argument to identify an endpoint direction.
    Valid only for Endpoints 1-7.

  Remarks:
    None.
*/

typedef enum
{
    USBHS_ENDPOINT_RX, // RX endpoint
    USBHS_ENDPOINT_TX  // TX endpoint

} USBHS_ENDPOINT_DIRECTION;

// *****************************************************************************
/* USBHS DMA Request Mode Enumeration

  Summary:
    Used as an argument to set DMA request mode.

  Description:
    THis data type is used as an argument to set DMA request mode.
    Valid only for Endpoints 1-7.

  Remarks:
    Used by PLIB_USBHS_EPnDMARequestModeSet.
*/

typedef enum
{
    USBHS_DMA_MODE0   /*DOM-IGNORE-BEGIN*/ =  0 /*DOM-IGNORE-END*/ ,
    USBHS_DMA_MODE1   /*DOM-IGNORE-BEGIN*/ =  1 /*DOM-IGNORE-END*/

} USBHS_DMA_REQUEST_MODE;

// *****************************************************************************
/* USBHS Test Speeds Enumeration

  Summary:
    Used as an argument for in setting module speeds in Test mode.

  Description:
    This data type is used as an argument for setting module speeds in Test mode.

  Remarks:
    For use with PLIB_USBHS_TestSpeedForce.
*/

typedef enum
{
    USBHS_TEST_LOW_SPEED   /*DOM-IGNORE-BEGIN*/ =  0x00 /*DOM-IGNORE-END*/ ,
    USBHS_TEST_FULL_SPEED  /*DOM-IGNORE-BEGIN*/ =  0x02 /*DOM-IGNORE-END*/ ,
    USBHS_TEST_HIGH_SPEED  /*DOM-IGNORE-BEGIN*/ =  0x01 /*DOM-IGNORE-END*/

} USBHS_TEST_SPEED;

// *****************************************************************************
/* USBHS Endpoint Data Toggle Enumeration

  Summary:
    Provides an enumeration data toggle for a packet.

  Description:
    This data type provides an enumeration data toggle for a packet.

  Remarks:
    None.
*/

typedef enum
{
    USBHS_DATA0 /*DOM-IGNORE-BEGIN*/ =  0 /*DOM-IGNORE-END*/, // DATA0/1 = 0
    USBHS_DATA1 /*DOM-IGNORE-BEGIN*/ =  1 /*DOM-IGNORE-END*/  // DATA0/1 = 1

} USBHS_DATA01;

// *****************************************************************************
/* USBHS Endpoint Speed Enumeration

  Summary:
    Provides enumeration Host endpoint speeds.

  Description:
    This data type provides enumeration Host endpoint speeds.

  Remarks:
    Used by PLIB_USBHS_EPnSpeedSet.
*/

typedef enum
{
    USBHS_LOW_SPEED  /*DOM-IGNORE-BEGIN*/ =  0x03 /*DOM-IGNORE-END*/,
    USBHS_FULL_SPEED /*DOM-IGNORE-BEGIN*/ =  0x02 /*DOM-IGNORE-END*/,
    USBHS_HIGH_SPEED /*DOM-IGNORE-BEGIN*/ =  0x01 /*DOM-IGNORE-END*/

} USBHS_SPEED;

// *****************************************************************************
/* USB Operating Modes Enumeration

  Summary:
    Provides enumeration of operating modes supported by USB.

  Description:
    This data type provides enumeration of the operating modes supported by the 
    USB module.

  Remarks:
    None.
*/

typedef enum
{
    // None
    USBHS_OPMODE_NONE   /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/ ,
    // Device
    USBHS_OPMODE_DEVICE /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/ ,
    // Host
    USBHS_OPMODE_HOST   /*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/ ,
    // OTG
    USBHS_OPMODE_OTG    /*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/

} USBHS_OPMODES;

// *****************************************************************************
/* USBHS Endpoint Transaction Type Enumeration

  Summary:
    Provides an enumeration of transaction types.

  Description:
    This data type provides an enumeration of transaction types.

  Remarks:
    Used by PLIB_USBHS_EPnTransactionTypeSet
*/

typedef enum
{
    USBHS_TRANSACTION_CONTROL   /*DOM-IGNORE-BEGIN*/ =  0x00 /*DOM-IGNORE-END*/,
    USBHS_TRANSACTION_ISO       /*DOM-IGNORE-BEGIN*/ =  0x01 /*DOM-IGNORE-END*/,
    USBHS_TRANSACTION_BULK      /*DOM-IGNORE-BEGIN*/ =  0x02 /*DOM-IGNORE-END*/,
    USBHS_TRANSACTION_INTERRUPT /*DOM-IGNORE-BEGIN*/ =  0x03 /*DOM-IGNORE-END*/

} USBHS_TRANSACTION_TYPE;

// *****************************************************************************
/* USBHS Hardware Configuration Bits Enumeration

  Summary:
    Provides the enumeration Configuration bits.

  Description:
    This data type provides the enumeration configuration bits returned by
    PLIB_USBHS_ConfigurationBitsGet.

  Remarks:
    See also PLIB_USBHS_ConfigurationBitsGet.
*/

typedef enum
{
    // Soft connect/disconnect supported
    USBHS_CONFIG_SOFTCONN    /*DOM-IGNORE-BEGIN*/ =  0x02 /*DOM-IGNORE-END*/,

    // Dynamic sizing of FIFO buffers supported
    USBHS_CONFIG_DYNFIFOSIZE /*DOM-IGNORE-BEGIN*/ =  0x04 /*DOM-IGNORE-END*/,

    // High bandwidth TX isochronous transfers supported
    USBHS_CONFIG_HBWTXISO    /*DOM-IGNORE-BEGIN*/ =  0x08 /*DOM-IGNORE-END*/,

    // High bandwidth RX isochronous transfers supported
    USBHS_CONFIG_HBWRXISO    /*DOM-IGNORE-BEGIN*/ =  0x10 /*DOM-IGNORE-END*/,

    // Big Endian byte ordering supported instead of Little Endian
    USBHS_CONFIG_BIGENDIAN   /*DOM-IGNORE-BEGIN*/ =  0x20 /*DOM-IGNORE-END*/,

    // Automatic splitting of bulk packets supported
    USBHS_CONFIG_AUTOSPLIT   /*DOM-IGNORE-BEGIN*/ =  0x40 /*DOM-IGNORE-END*/,

    // Automatic combining of bulk packets supported
    USBHS_CONFIG_AUTOJOIN    /*DOM-IGNORE-BEGIN*/ =  0x80 /*DOM-IGNORE-END*/

} USBHS_CONFIGURATION;

// *****************************************************************************
/* USBHS FIFO State Enumeration

  Summary:
    Provides enumeration of receive and transmit FIFO states, as reported by
    status bits.

  Description:
    This data type provides enumeration of receive and transmit FIFO states, as
    reported by status bits.

  Remarks:
    None.
*/

typedef enum
{
     USBHS_TX_FIFO_EMPTY,     // TX FIFO Empty
     USBHS_TX_FIFO_NOT_EMPTY, // TX FIFO NOT Empty
     USBHS_RX_FIFO_FULL,      // RX FIFO Full
     USBHS_RX_FIFO_NOTFULL    // RX FIFO NOT Full

} USBHS_TXRX_FIFO_STATE;

// *****************************************************************************
/* USBHS Iso Packets Per Microframe Enumeration

  Summary:
    Provides an enumeration of the allowable isochronous packets per microframe
    when operating in High-Speed mode.

  Description:
    This data type provides an enumeration of the allowable isochronous packets
    per microframe when operating in High-Speed mode.

  Remarks:
    Used by PLIB_USBHS_EPnPacketsInMicroFrameSet.
*/

typedef enum
{
    USBHS_ONE_PKTS    /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
    USBHS_TWO_PKTS    /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/,
    USBHS_THREE_PKTS  /*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/,
    USBHS_RSVD_PKTS   /*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/,

} USBHS_PKTS_PER_MICROFRAME;

// *****************************************************************************
/* USBHS Dynamic FIFO Size Enumeration

  Summary:
    Provides enumeration of dynamic FIFO sizes.

  Description:
    This data type provides enumeration of dynamic FIFO sizes.

  Remarks:
    FIFO size = 2^(FIFO_Size<3:0> + 3)
*/

typedef enum
{
    USBHS_FIFO_SIZE_8BYTES     /*DOM-IGNORE-BEGIN*/ =  0x0 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_16BYTES    /*DOM-IGNORE-BEGIN*/ =  0x1 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_32BYTES    /*DOM-IGNORE-BEGIN*/ =  0x2 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_64BYTES    /*DOM-IGNORE-BEGIN*/ =  0x3 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_128BYTES   /*DOM-IGNORE-BEGIN*/ =  0x4 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_256BYTES   /*DOM-IGNORE-BEGIN*/ =  0x5 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_512BYTES   /*DOM-IGNORE-BEGIN*/ =  0x6 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_1024BYTES  /*DOM-IGNORE-BEGIN*/ =  0x7 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_2048BYTES  /*DOM-IGNORE-BEGIN*/ =  0x8 /*DOM-IGNORE-END*/,
    USBHS_FIFO_SIZE_4096BYTES  /*DOM-IGNORE-BEGIN*/ =  0x9 /*DOM-IGNORE-END*/

} USBHS_DYN_FIFO_SIZE;

// *****************************************************************************
/* USBHS Dynamic FIFO Double Packet versus Single Packet Buffering

  Summary:
    Provides enumeration of dynamic FIFO double-packet versus single-packet
    buffering.

  Description:
    This data type provides enumeration of dynamic FIFO double-packet versus
    single-packet buffering.

  Remarks:
    None.
*/

typedef enum
{
    USBHS_FIFO_SINGLEPKT   /*DOM-IGNORE-BEGIN*/ =  0x0 /*DOM-IGNORE-END*/,
    USBHS_FIFO_DOUBLEPKT   /*DOM-IGNORE-BEGIN*/ =  0x1 /*DOM-IGNORE-END*/

} USBHS_DYN_FIFO_PACKET_BUFFERING;

// *****************************************************************************
/* USBHS DMA Assertion Timing Enumeration

  Summary:
    Provides enumeration DMA assertion timing (early versus late).

  Description:
    This data type provides enumeration DMA assertion timing (early versus late).

  Remarks:
    None.
*/

typedef enum
{
    // Assert DMA 8 bytes before filling FIFO
    USBHS_DMA_ASSERT_EARLY  /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/ ,

    // Assert DMA when FIFO is full
    USBHS_DMA_ASSERT_LATE   /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/ ,

} USBHS_DMA_ASSERT_TIMING;

// *****************************************************************************
/* USBHS DMA Burst Modes Enumeration

  Summary:
    Provides enumeration of all DMA burst modes.

  Description:
    This data type provides enumeration of all DMA burst modes.

  Remarks:
    None.
*/

typedef enum
{
    // Burst Mode 0: Bursts of unspecified length
    USBHS_DMA_BURST_MODE0  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,

    // Burst Mode 1: INCR4 or unspecified length
    USBHS_DMA_BURST_MODE1  /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/,

    // Burst Mode 2: INCR8, INCR4 or unspecified length
    USBHS_DMA_BURST_MODE2  /*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/,

    // Burst Mode 3: INCR16, INCR8, INCR4 or unspecified length
    USBHS_DMA_BURST_MODE3  /*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/

} USBHS_DMA_BURST_MODE;

// *****************************************************************************
/* USBHS DMA Transfer Modes Enumeration

  Summary:
    Provides enumeration of all DMA transfer modes.

  Description:
    This data type provides enumeration of all DMA Transfer modes.

  Remarks:
    None.
*/

typedef enum
{
    USBHS_DMA_TRANSFER_MODE0 /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,
    USBHS_DMA_TRANSFER_MODE1 /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/

} USBHS_DMA_TRANSFER_MODE;

// *****************************************************************************
/* USBHS Link Power Management (LPM) Mode Enumeration

  Summary:
    Provides enumeration of Link Power Management (LPM) modes.

  Description:
    This data type provides enumeration of Link Power Management (LPM) modes.

  Remarks:
    Used by PLIB_USBHS_LPMModeSet.
*/

typedef enum
{
    // LPM and Extended transactions not supported and will time-out
    USBHS_LPM_DISABLED       /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,

    // Extended transactions supported but LPM transactions not supported and produce STALLs
    USBHS_LPM_EXTENDEDNOLPM  /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/,

    // LPM and Extended transactions not supported and will timeout
    USBHS_LPM_DISABLED2      /*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/,

    // LPM and Extended transactions are supported
    USBHS_LPM_LPMEXTENDED    /*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/

} USBHS_LPM_MODE;

// *****************************************************************************
/* USBHS Link Power Management Requested State Enumeration

  Summary:
    Provides enumeration requested device state after accepting an LPM
    transaction.

  Description:
    This data type provides enumeration requested device state after accepting
    an LPM transaction.

  Remarks:
    Used by PLIB_USBHS_LPMRequestedLinkStateGet and
    PLIB_USBHS_LPMRequestedLinkStateSet.
*/

typedef enum
{
    USBHS_LPM_RESERVED_0 /*DOM-IGNORE-BEGIN*/ = 0x0 /*DOM-IGNORE-END*/,
    USBHS_LPM_L1_STATE   /*DOM-IGNORE-BEGIN*/ = 0x1 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_2 /*DOM-IGNORE-BEGIN*/ = 0x2 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_3 /*DOM-IGNORE-BEGIN*/ = 0x3 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_4 /*DOM-IGNORE-BEGIN*/ = 0x4 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_5 /*DOM-IGNORE-BEGIN*/ = 0x5 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_6 /*DOM-IGNORE-BEGIN*/ = 0x6 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_7 /*DOM-IGNORE-BEGIN*/ = 0x7 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_8 /*DOM-IGNORE-BEGIN*/ = 0x8 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_9 /*DOM-IGNORE-BEGIN*/ = 0x9 /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_A /*DOM-IGNORE-BEGIN*/ = 0xA /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_B /*DOM-IGNORE-BEGIN*/ = 0xB /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_C /*DOM-IGNORE-BEGIN*/ = 0xC /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_D /*DOM-IGNORE-BEGIN*/ = 0xD /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_E /*DOM-IGNORE-BEGIN*/ = 0xE /*DOM-IGNORE-END*/,
    USBHS_LPM_RESERVED_F /*DOM-IGNORE-BEGIN*/ = 0xF /*DOM-IGNORE-END*/,

} USBHS_LPM_LINK_STATE;

// ****************************************************************************
// ****************************************************************************
// Section: Processor Include Files
// ****************************************************************************
// ****************************************************************************
/*DOM-IGNORE-BEGIN*/
#include "./processor/usbhs_processor.h"
/*DOM-IGNORE-END*/

// *****************************************************************************
// *****************************************************************************
// Section: Helper Macros
// *****************************************************************************
// *****************************************************************************

// None

// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Routines: USB Setup
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceAddressSet (USBHS_MODULE_ID index, uint8_t address)

  Summary:
    Sets the device address.

  Description:
    This function sets the device address. This function should be called with
    the address received from the host in the SET_ADDRESS request.

  Precondition:
    The USB module must be configured for Device mode operation.

  Parameters:
    index  - Identifier for the device instance to be configured
    address - 7-bit USB Device address that should be assigned to this device

  Returns:
    None.

  Example:
    <code>
    // This code example assigns a USB device address.

    uint8_t address;
    PLIB_USBHS_DeviceAddressSet(USBHS_ID_0, address);
    </code>

  Remarks:
    None.
*/

void PLIB_USBHS_DeviceAddressSet ( USBHS_MODULE_ID index, uint8_t address );

// *****************************************************************************
/* Function:
    uint8_t PLIB_USBHS_DeviceAddressGet (USBHS_MODULE_ID index)

  Summary:
    Returns the current USB device address.

  Description:
    This function returns the current USB device address.

  Precondition:
    The USB module should have been configured for Device mode operation.

  Parameters:
    index  - Identifier for the device instance to be configured

  Returns:
    7-bit unsigned integer value indicating the current USB device address.

  Example:
    <code>
    // This code example reads the current assigned USB device address.

    uint8_t address;
    address = PLIB_USBHS_DeviceAddressGet(USBHS_ID_0);
    </code>

  Remarks:
    None.
*/

uint8_t PLIB_USBHS_DeviceAddressGet (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceConnect(USBHS_MODULE_ID index)

  Summary:
    Tri-states the USB D+ and D- lines.

  Description:
    This function enables the USB D+ and D- lines, which connects the
    device from the host.

  Precondition:
    The USB module must be configured for Device mode operation.

  Parameters:
    index - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Disconnect the device from the host

    PLIB_USBHS_DeviceConnect(USBHS_ID_0);

    </code>

  Remarks:
    Only valid for Device mode.
*/

void PLIB_USBHS_DeviceConnect(USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceDisconnect(USBHS_MODULE_ID index)

  Summary:
    Tri-states the USB D+ and D- lines.

  Description:
    This function tri-states the USB D+ and D- lines, which disconnects the
    device from the host.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Disconnect the device from the host

    PLIB_USBHS_DeviceDisconnect(USBHS_ID_0);

    </code>

  Remarks:
    Only valid for Device mode.
*/

void PLIB_USBHS_DeviceDisconnect(USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_EndpointFIFOLoad
    (
        USBHS_MODULE_ID index, 
        uint8_t endpoint, 
        void * source, 
        size_t nBytes
    )

  Summary:
    Loads the endpoint FIFO with provided data and then enables
    the endpoint transmit.

  Description:
    This function loads the endpoint FIFO with provided data. This operation
    would typically be performed for a endpoint transmit operation. Once the
    FIFO is loaded the function will enable the enable the endpoint for
    transmit.  

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance to be configured
    endpoint - endpoint of which FIFO needs to be loaded
    source   - data that should be loaded
    nBytes   - number of bytes to load.

  Returns:
    None.

  Example:
    <code>
    // Load endpoint 1 FIFO

    PLIB_USBHS_EndpointFIFOLoad(USBHS_ID_0, 1, data, 10);

    </code>

  Remarks:
    Valid in both Peripheral and Host mode.
*/

void PLIB_USBHS_EndpointFIFOLoad
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint, 
    void * source, 
    size_t nBytes
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_Endpoint0SetupPacketLoad
    (
        USBHS_MODULE_ID index,  
        void * setupPacket, 
        uint8_t deviceAddress, 
        uint8_t hubAddress, 
        uint8_t hubPortAddress,
        uint32_t speed
    )

  Summary:
    Loads the endpoint 0 FIFO with provided setup packet and then
    enables the endpoint transmit.

  Description:
    This function loads the endpoint 0 FIFO with provided setup packet. This
    operation would typically be performed at the start of a endpoint 0 control
    transfer. Once the FIFO is loaded the function will load the target device
    address, hub and hub port address and then transmit the packet.  

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    setupPacket     - setup packet to be sent to the device
    deviceAddress   - USB address of the device
    hubAddress      - USB address of the hub if the device is connected to a 
                      hub.
                      Should be zero otherwise.
    hubPortAddress  - Port number of the hub port to which the device is
                      connected if it is connected to a hub. Should be zero 
                      otherwise.
    speed           - Speed of the device to which the packet should be sent.

  Returns:
    None.

  Example:
    <code>
    // Send a setup packet to device 7 control endpoint
    // connected directly to module.

    PLIB_USBHS_Endpoint0SetupPacketLoad
    (
        USBHS_ID_0, 
        setupPacket, 
        7, 0, 0, USB_SPEED_HIGH
    );

    </code>

  Remarks:
    Valid in Host mode only. Setup packet size should be 8 bytes. Packet will
    always be targeted to endpoint 0 of the target device.
*/

void PLIB_USBHS_Endpoint0SetupPacketLoad
(
    USBHS_MODULE_ID index,  
    void * setupPacket, 
    uint8_t deviceAddress, 
    uint8_t hubAddress, 
    uint8_t hubPortAddress,
    uint32_t speed
);

// *****************************************************************************
/* Function:
    int PLIB_USBHS_EndpointFIFOUnload
    (
        USBHS_MODULE_ID index, 
        uint8_t endpoint, 
        void * destination
    )

  Summary:
    Unloads the endpoint FIFO.

  Description:
    This function unloads the endpoint FIFO. This operation would typically be
    performed for a endpoint receive operation. The unloaded data is stored in
    provided buffer.

  Precondition:
    The USB module must be configured for Device mode operation.

  Parameters:
    index           - Identifier for the device instance to be configured
    endpoint        - endpoint of which FIFO to be unloaded.
    dest            - target address where unloaded data should be stored.

  Returns:
    Returns the number of functions that were read.

  Example:
    <code>
    // Load endpoint 1 FIFO
    int count;

    count = PLIB_USBHS_EndpointFIFOUnload(USBHS_ID_0, 1, dest, 10);

    </code>

  Remarks:
    Valid in both Peripheral and Host mode.
*/

int PLIB_USBHS_EndpointFIFOUnload
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint, 
    void * dest
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_SoftResetEnable(USBHS_MODULE_ID index);

  Summary:
    Enables soft reset.

  Description:
    This function enables soft reset.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>

    PLIB_USBHS_SoftResetEnable(USBHS_ID_0);

    </code>

  Remarks:
    Valid in both Peripheral and Host mode.
*/

void PLIB_USBHS_SoftResetEnable(USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_SoftResetDisable(USBHS_MODULE_ID index);

  Summary:
    Disables soft reset.

  Description:
    This function disables soft reset.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>

    PLIB_USBHS_SoftResetDisable(USBHS_ID_0);

    </code>

  Remarks:
    Valid in both Peripheral and Host mode.
*/

void PLIB_USBHS_SoftResetDisable(USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_SoftResetNRSTXEnable(USBHS_MODULE_ID index);

  Summary:
    Enables the module NRSTX reset output.

  Description:
    Enables the module NRSTX reset output.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>

    PLIB_USBHS_SoftResetNRSTXEnable(USBHS_ID_0);

    </code>

  Remarks:
    Valid in both Peripheral and Host mode.
*/

void PLIB_USBHS_SoftResetNRSTXEnable(USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_ModuleSpeedSet(USBHS_MODULE_ID index, uint32_t speed)

  Summary:
    Sets the operation speed of the USB Module.

  Description:
    This function sets the operation speed of the USB module.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance to be configured
    speed  - The operation speed of the USB module

  Returns:
    None.

  Example:
    <code>
    // Enable the USB module for high speed support.

    PLIB_USBHS_HighSpeedEnable(USBHS_ID_0, USB_SPEED_HIGH);

    </code>

  Remarks:
    None.
*/

void PLIB_USBHS_HighSpeedEnable(USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    USB_SPEED PLIB_USBHS_ModuleSpeedGet( USBHS_MODULE_ID index )

  Summary:
    Returns the speed at which the module is operating.

  Description:
    This function returns the speed at which the module is operating. In case of
    device mode operation, this function returns the speed at which the device
    attached to the host.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    Current module operation speed on the USB.

  Example:
  <code>
    USB_SPEED speed;
    speed = PLIB_USBHS_ModuleSpeedGet(USBHS_ID_0);
  </code>

  Remarks:
    None.
*/

bool PLIB_USBHS_ModuleSpeedGet( USBHS_MODULE_ID index );

// *****************************************************************************
/* Function:
    USBHS_VBUS_DETECT_LEVEL PLIB_USBHS_VbusLevelGet( USBHS_MODULE_ID index )

  Summary:
    Returns the current VBUS level encode using the USBHS_VBUS_DETECT_LEVEL
    enumeration.

  Description:
    Returns the current VBUS level encode using the USBHS_VBUS_DETECT_LEVEL
    enumeration.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    Detected VBUS level, see USBHS_VBUS_DETECT_LEVEL enumeration.

  Example:
  <code>
  USBHS_VBUS_DETECT_LEVEL vbusLevel;
  vbusLevel = PLIB_USBHS_VbusLevelGet(USBHS_ID_0);
  </code>

  Remarks:
    None.
*/

USBHS_VBUS_LEVEL PLIB_USBHS_VbusLevelGet( USBHS_MODULE_ID index );

// *****************************************************************************
// *****************************************************************************
// Section: USB Peripheral Library Interface Routines: Multi-Point Support
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: USBHS Peripheral Library Interface Routines: Interrupts
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PLIB_USBHS_GenInterruptDisable
    (
        USBHS_MODULE_ID index,
        USBHS_GEN_INTERRUPT interruptFlag 
    )

  Summary:
    Disables a general interrupt for the USB module.

  Description:
    This function disables a general interrupt source for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_GenInterruptDisable( MY_USB_INSTANCE, USBHS_GENINT_VBUSERR );
  </code>

  Remarks:
    See also PLIB_USBHS_GenInterruptEnable.
*/

void PLIB_USBHS_GenInterruptDisable
( 
    USBHS_MODULE_ID index,
    USBHS_GEN_INTERRUPT interruptFlag 
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_GenInterruptEnable
    ( 
        USBHS_MODULE_ID index,
        USBHS_GEN_INTERRUPT interruptFlag 
    )

  Summary:
    Enables a general interrupt for the USB module.

  Description:
    This function enables the general interrupt sources of the USB module that 
	are used to trigger a USB interrupt.

  Precondition:
    None.

  Parameters:
    index         - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_GenInterruptEnable( MY_USB_INSTANCE, USBHS_GENINT_VBUSERR );
  </code>

  Remarks:
    See also PLIB_USBHS_GenInterruptDisable.
*/

void PLIB_USBHS_GenInterruptEnable
( 
    USBHS_MODULE_ID index,
    USBHS_GEN_INTERRUPT interruptFlag 
);

// *****************************************************************************
/* Function:
    USBHS_GEN_INTERRUPT PLIB_USBHS_GenInterruptFlagsGet( USBHS_MODULE_ID index )

  Summary:
    Gets general interrupt flags.

  Description:
    This function gets the general interrupt flags. Calling this function also
    clears the flags.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    USBHS_GEN_INTERRUPT interruptFlags;
    interruptFlags = PLIB_USBHS_GenInterruptFlagsGet( MY_USB_INSTANCE );
    // All interrupt flags cleared on read.

    if ( interruptFlags > 0 )
    {
        if ( interruptFlags & USBHS_GENINT_SUSPEND )
        {
           // Device has detected suspend signaling on the bus.
        }
        if ( interruptFlags & USBHS_GENINT_RESUME )
        {
            //
        }
        .
        .
        .
    }
  </code>

  Remarks:
    None.
*/

USBHS_GEN_INTERRUPT PLIB_USBHS_GenInterruptFlagsGet( USBHS_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_USBHS_TxInterruptDisable
    ( 
        USBHS_MODULE_ID index,
        USBHS_EPTXRX_INTERRUPT interruptFlag 
    )

  Summary:
    Disables a TX endpoint interrupt source for the USB module.

  Description:
    This function disables a TX endpoint interrupt source for the USB module.

  Precondition:
    None.

  Parameters:
    index         - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_TxInterruptDisable( MY_USB_INSTANCE, USBHS_TXRXINT_EP1 );
  </code>

  Remarks:
    See also PLIB_USBHS_TxInterruptEnable.
*/

void PLIB_USBHS_TxInterruptDisable
(
    USBHS_MODULE_ID index,
    USBHS_EPTXRX_INTERRUPT interruptFlag 
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_TxInterruptEnable
    ( 
        USBHS_MODULE_ID index,
        USBHS_EPTXRX_INTERRUPT interruptFlag 
    )

  Summary:
    Enables a TX endpoint Interrupt for the USB module.

  Description:
    This function enables the TX endpoint interrupt sources of the USB module that 
	are used to trigger a USB interrupt.

  Precondition:
    None.

  Parameters:
    index         - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_TxInterruptEnable( MY_USB_INSTANCE, USBHS_TXRXINT_EP1 );
  </code>

  Remarks:
    See also PLIB_USBHS_TxInterruptDisable.
*/

void PLIB_USBHS_TxInterruptEnable
(
    USBHS_MODULE_ID index,
    USBHS_EPTXRX_INTERRUPT interruptFlag 
);

// *****************************************************************************
/* Function:
    USBHS_EPTXRX_INTERRUPT PLIB_USBHS_TxInterruptFlagsGet
    ( 
        USBHS_MODULE_ID index 
    )

  Summary:
    Gets the TX endpoint interrupt flags.

  Description:
    This function gets the TX endpoint interrupt flags. Calling this function
    will also clear the interrupt flags.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    USBHS_EPTXRX_INTERRUPT interruptFlags;
    interruptFlags = PLIB_USBHS_TxInterruptFlagsGet( MY_USB_INSTANCE );
    // All interrupt flags cleared on read.

    if ( interruptFlags > 0 )
    {
        if ( interruptFlags & USBHS_TXRXINT_EP0 )
        {
           // Endpoint Zero
        }
        if ( interruptFlags & USBHS_TXRXINT_EP1 )
        {
            // Endpoint One
        }
        .
        .
        .
    }
  </code>

  Remarks:
    TX interrupts must first be enabled by calling PLIB_USBHS_TxInterruptEnable.
*/

USBHS_EPTXRX_INTERRUPT PLIB_USBHS_TxInterruptFlagsGet( USBHS_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_USBHS_RxInterruptDisable
    ( 
        USBHS_MODULE_ID index,
        USBHS_EPTXRX_INTERRUPT interruptFlag 
    )

  Summary:
    Disables a RX endpoint interrupt for the USB module.

  Description:
    This function disables a RX endpoint interrupt source for the USB module.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_RxInterruptDisable( MY_USB_INSTANCE, USBHS_TXRXINT_EP1 );
  </code>

  Remarks:
    See also PLIB_USBHS_RxInterruptEnable.
    USBHS_TXRXINT_EP0 is not a valid argument. For endpoint zero, use
    PLIB_USBHS_TxInterruptDisable.
*/

void PLIB_USBHS_RxInterruptDisable
(
    USBHS_MODULE_ID index,
    USBHS_EPTXRX_INTERRUPT interruptFlag 
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_RxInterruptEnable
    ( 
        USBHS_MODULE_ID index,
        USBHS_EPTXRX_INTERRUPT interruptFlag 
    )

  Summary:
    Enables a RX endpoint interrupt for the USB module.

  Description:
    This function enables RX endpoint interrupt sources of the USB module to
    trigger a USB interrupt.

  Precondition:
    None.

  Parameters:
    index         - Identifier for the device instance of interest
    interruptFlag - Interrupt

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_RxInterruptEnable( MY_USB_INSTANCE, USBHS_TXRXINT_EP1 );
  </code>

  Remarks:
    See also PLIB_USBHS_RxInterruptDisable.
    USBHS_TXRXINT_EP0 is not a valid argument.
    For endpoint zero use PLIB_USBHS_TxInterruptEnable.
*/

void PLIB_USBHS_RxInterruptEnable
( 
    USBHS_MODULE_ID index,
    USBHS_EPTXRX_INTERRUPT interruptFlag 
);

// *****************************************************************************
/* Function:
    USBHS_EPTXRX_INTERRUPT PLIB_USBHS_RxInterruptFlagsGet
    ( 
        USBHS_MODULE_ID index 
    )

  Summary:
    Gets RX endpoint interrupt flags.

  Description:
    This function gets the RX endpoint interrupt flags. Calling this function
    will also clear all the flags.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    USBHS_EPTXRX_INTERRUPT interruptFlags;
    interruptFlags = PLIB_USBHS_RxInterruptFlagsGet( MY_USB_INSTANCE );
    // All interrupt flags cleared on read.

    {if ( interruptFlags > 0 )
    {
        if ( interruptFlags & USBHS_TXRXINT_EP1 )
        {
           // Endpoint Zero
        }
        if ( interruptFlags & USBHS_TXRXINT_EP2 )
        {
            // Endpoint One
        }
        .
        .
        .
    }
  </code>

  Remarks:
    RX interrupts must first be enabled by calling PLIB_USBHS_RxInterruptEnable.
    USBHS_TXRXINT_EP0 is not a valid argument. For endpoint zero use
    PLIB_USBHS_TxInterruptFlagsGet.
*/

USBHS_EPTXRX_INTERRUPT PLIB_USBHS_RxInterruptFlagsGet( USBHS_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DMAInterruptDisable
    ( 
        USBHS_MODULE_ID index,
        USBHS_DMA_INTERRUPT dmaChannelInterrupt 
    )

  Summary:
    Disables DMA channel interrupts.

  Description:
    This function disables DMA Channel interrupts.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest
    dmaChannelInterrupt - DMA channel Interrupt to disable

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_DMAInterruptDisable (USBHS_ID_0, USBHS_DMAINT_5);
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_DMAInterruptDisable
( 
    USBHS_MODULE_ID index,
    USBHS_DMA_INTERRUPT nChannelNumber 
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_InterruptEnableSet
    ( 
        USBHS_MODULE_ID index, 
        USBHS_GEN_INTERRUPT generalInterrupts,
        USBHS_EPTXRX_INTERRUPT transmitInterrupts,
        USBHS_EPTXRX_INTERRUPT receiveInterrupts
    ); 

  Summary:
    Enables USB module event interrupts.

  Description:
    This function enables USB module event interrupts. Combines the
    functionality of the PLIB_USBHS_RxInterruptEnable,
    PLIB_USBHS_TxInterruptEnable, and PLIB_USBHS_GenInterruptEnable functions.

  Precondition:
    None.

  Parameters:
    index               - Identifier for the device instance of interest
    generalInterrupts   - General Module interrupts.
    transmitInterrupts  - Transmit Endpoint Interrupts to be enabled.
    receiveInterrupts   - Receive Endpoint Interrupts to be enabled.

  Returns:
    None.

  Example:
  <code>
    // Enable the reset interrupt, endpoint 1 transmit interrupt
    // and endpoint 2 receive interrupts

    PLIB_USBHS_InterruptEnableSet (USBHS_ID_0, USBHS_GENINT_RESET, 
        USBHS_TXRXINT_EP1, USBHS_TXRXINT_EP2 );
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_InterruptEnableSet
( 
    USBHS_MODULE_ID index, 
    USBHS_GEN_INTERRUPT generalInterrupts,
    USBHS_EPTXRX_INTERRUPT transmitInterrupts,
    USBHS_EPTXRX_INTERRUPT receiveInterrupts
); 

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DMAInterruptEnable
    ( 
        USBHS_MODULE_ID index,
        USBHS_DMA_INTERRUPT dmaChannelInterrupt 
    )

  Summary:
    Enables DMA channel interrupts.

  Description:
    This function enables DMA Channel interrupts.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest
    dmaChannelInterrupt - DMA channel Interrupt to enable

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_DMAInterruptEnable (USBHS_ID_0, USBHS_DMAINT_5);
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_DMAInterruptEnable
( 
    USBHS_MODULE_ID index,
    USBHS_DMA_INTERRUPT nChannelNumber 
);

// *****************************************************************************
/* Function:
    USBHS_DMA_INTERRUPT PLIB_USBHS_DMAInterruptFlagsGet( USBHS_MODULE_ID index )

  Summary:
    Gets the DMA channel interrupt flags.

  Description:
    This function gets the DMA channel interrupt flags. Calling this function
    will also clear the flags.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    USBHS_DMA_INTERRUPT interruptFlags;
    interruptFlags = PLIB_USBHS_DMAInterruptFlagsGet( MY_USB_INSTANCE );
    // All interrupt flags cleared on read.

    if ( interruptFlags > 0 )
    {
        if ( interruptFlags & USBHS_DMAINT_1 )
        {
            // DMA Channel 1
        }
        if (interruptFlags & USBHS_DMAINT_2)
        {
            // DMA Channel 2
        }
        .
        .
        .
    }
  </code>

  Remarks:
    None.
*/

USBHS_DMA_INTERRUPT PLIB_USBHS_DMAInterruptFlagsGet( USBHS_MODULE_ID index );

// *****************************************************************************
/* Function:
    void PLIB_USBHS_HighSpeedEnable (USBHS_MODULE_ID index);

  Summary:
    Enables module high speed operation.

  Description:
    This function enables module high speed operation. If the module is
    operating as a device, the device will attempt to attach at high speed. If
    the module is operating as host, it will respond to device high speed
    handshake.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_HighSpeedEnable(USBHS_ID_0);
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_HighSpeedEnable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_HighSpeedDisable (USBHS_MODULE_ID index);

  Summary:
    Disables high speed operation.

  Description:
    This function disables module high speed operation. If operating as device,
    the module will always connect to the host at Full Speed. If operating as a
    host, the module will only connect to a device at full speed or low speed.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_HighSpeedDisable(USBHS_ID_0);
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_HighSpeedDisable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    bool PLIB_USBHS_HighSpeedIsConnected  (USBHS_MODULE_ID index);

  Summary:
    Returns true if the module is operating in High Speed mode.

  Description:
    This function returns true if the module is operating in high speed mode. In
    device mode, this means the module connected to the host at high speed. In
    host mode, this means that a device connected to the module at high speed
    mode.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    true - module is operating in high speed mode.
    false - module is not operating in high speed mode.

  Example:
  <code>
    if(PLIB_USBHS_HighSpeedIsConnected(USBHS_ID_0)
    {
        //This means module is operating in high speed mode.
    }
  </code>

  Remarks:
    None.
*/

bool PLIB_USBHS_HighSpeedIsConnected (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    bool PLIB_USBHS_HostModeIsEnabled (USBHS_MODULE_ID index);

  Summary:
    Returns true if the module is operating as a USB host.

  Description:
    This function returns true if the module is operating as a USB Host.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    true - module is operating as a USB host.
    false - module is operating as a USB device.

  Example:
  <code>
    if(PLIB_USBHS_HostModeIsEnabled(USBHS_ID_0))
    {
        // Module is operating as a USB Host.
    }
    else
    {
        // Module is operating as a USB Device.
    }
  </code>

  Remarks:
    None.
*/

bool PLIB_USBHS_HostModeIsEnabled (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    bool PLIB_USBHS_IsBDevice (USBHS_MODULE_ID index);

  Summary:
    Returns true if the module is operating as OTG B Device.

  Description:
    This function returns true if the module is operating as a OTG B Device.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest

  Returns:
    true - module is operating as a OTG B Device.
    false - modeule is operating as a OTG A Device.

  Example:
  <code>
    if(PLIB_USBHS_IsBDevice(USBHS_ID_0))
    {
        // Module is operating as a OTG B device
    }
    else
    {
        // Module is operating as a OTG A Device
    }
  </code>

  Remarks:
    None.
*/

bool PLIB_USBHS_IsBDevice (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_ResumeEnable (USBHS_MODULE_ID index);

  Summary:
    Enables resume signalling on the bus.

  Description:
    This function enables resume signalling on the bus.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_ResumeEnable(USBHS_ID_0);
  </code>

  Remarks:
    This function can be called in both Device and Host mode.
*/

void PLIB_USBHS_ResumeEnable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_ResumeDisable (USBHS_MODULE_ID index);

  Summary:
    Disables resume signalling on the bus.

  Description:
    This function disables resume signalling on the bus.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_ResumeDisable(USBHS_ID_0);
  </code>

  Remarks:
    This function can be called in both Device and Host mode.
*/

void PLIB_USBHS_ResumeDisable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_SuspendEnable (USBHS_MODULE_ID index);

  Summary:
    Suspends the USB.

  Description:
    This function suspends the USB. This will cause the module to stop
    generating SOFs in host mode.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_SuspendEnable(USBHS_ID_0);
  </code>

  Remarks:
    This fucntion should be called in host mode only.
*/

void PLIB_USBHS_SuspendEnable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_SuspendDisable (USBHS_MODULE_ID index);

  Summary:
    Brings the USB out of suspend.

  Description:
    This function will bring the USB out of suspend. This will cause the moduel
    to start generating SOFs in host mode.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_SuspendDisable(USBHS_ID_0);
  </code>

  Remarks:
    This fucntion should be called in host mode only.
*/

void PLIB_USBHS_SuspendDisable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_SessionEnable (USBHS_MODULE_ID index);

  Summary:
    Starts a host session when module is in OTG A mode.

  Description:
    This function will enable the session when the module is operating in OTG A
    mode.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_SessionEnable(USBHS_ID_0);
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_SessionEnable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_SessionDisable (USBHS_MODULE_ID index);

  Summary:
    Ends a host session when module is in OTG A mode.

  Description:
    This function will end the session when the module is operating in OTG A
    mode.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_SessionDisable(USBHS_ID_0);
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_SessionDisable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_ResetEnable (USBHS_MODULE_ID index);

  Summary:
    Enables reset signalling on the USB.

  Description:
    This function enables reset signalling on the USB.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_ResetEnable(USBHS_ID_0);
  </code>

  Remarks:
    This function should be called in host mode operation only.
*/

void PLIB_USBHS_ResetEnable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_ResetDisable (USBHS_MODULE_ID index);

  Summary:
    Disables reset signalling on the USB.

  Description:
    This function disables reset signalling on the USB.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_ResetDisables(USBHS_ID_0);
  </code>

  Remarks:
    This function should be called in host mode operation only.
*/

void PLIB_USBHS_ResetDisable (USBHS_MODULE_ID index);
void PLIB_USBHS_DeviceEPFIFOLoad            (USBHS_MODULE_ID index, uint8_t endpoint, void * source, size_t nBytes);
int  PLIB_USBHS_DeviceEPFIFOUnload          (USBHS_MODULE_ID index, uint8_t endpoint, void * dest);
void PLIB_USBHS_EndpointRxRequestEnable     (USBHS_MODULE_ID index, uint8_t endpoint);
bool PLIB_USBHS_FullOrHighSpeedIsConnected  (USBHS_MODULE_ID index);
bool PLIB_USBHS_DMAErrorGet (USBHS_MODULE_ID index, uint8_t dmaChannel);
uint8_t PLIB_USBHS_DMAInterruptGet(USBHS_MODULE_ID index);
void PLIB_USBHS_DMAOperationEnable(USBHS_MODULE_ID index, uint8_t endpoint, uint8_t dmaChannel, void * address, uint32_t count, bool direction);
void PLIB_USBHS_LoadEPInIndex(USBHS_MODULE_ID index, uint8_t endpoint);
uint8_t * PLIB_USBHS_GetEP0FIFOAddress(USBHS_MODULE_ID index);
uint8_t * PLIB_USBHS_GetEP0CSRAddress(USBHS_MODULE_ID index);
uint32_t PLIB_USBHS_GetReceiveDataCount(USBHS_MODULE_ID index, uint8_t endpoint);
bool PLIB_USBHS_TestModeEnter(USBHS_MODULE_ID index, uint8_t testMode);
bool PLIB_USBHS_TestModeExit(USBHS_MODULE_ID index, uint8_t testMode);




USBHS_VBUS_LEVEL PLIB_USBHS_VBUSLevelGet(USBHS_MODULE_ID index);

void PLIB_USBHS_DeviceTxEndpointConfigure
(
    USBHS_MODULE_ID index,
    uint8_t endpoint,
    uint16_t endpointSize,
    uint16_t fifoAddress,
    uint8_t fifoSize,
    uint32_t transferType
);

void PLIB_USBHS_DeviceRxEndpointConfigure
(
    USBHS_MODULE_ID index,
    uint8_t endpoint,
    uint16_t endpointSize,
    uint16_t fifoAddress,
    uint8_t fifoSize,
    uint32_t transferType
);

void PLIB_USBHS_HostRxEndpointConfigure
(
    USBHS_MODULE_ID index, 
    uint8_t hostEndpoint, 
    uint32_t speed,
    uint32_t pipeType,
    uint16_t endpointSize,
    uint16_t receiveFIFOAddress,
    uint16_t fifoSize,
    uint8_t  targetEndpoint,
    uint8_t  targetDevice,
    uint8_t  targetHub,
    uint8_t  targetHubPort,
    uint8_t  nakInterval
);
 
void PLIB_USBHS_HostTxEndpointConfigure
(
    USBHS_MODULE_ID index, 
    uint8_t hostEndpoint, 
    uint32_t speed,
    uint32_t pipeType,
    uint16_t endpointSize,
    uint16_t receiveFIFOAddress,
    uint16_t fifoSize,
    uint8_t  targetEndpoint,
    uint8_t  targetDevice,
    uint8_t  targetHub,
    uint8_t  targetHubPort,
    uint8_t  nakInterval
);

void PLIB_USBHS_HostTxEndpointDataToggleClear(USBHS_MODULE_ID index, uint8_t hostEndpoint);
void PLIB_USBHS_HostRxEndpointDataToggleClear(USBHS_MODULE_ID index, uint8_t hostEndpoint);


uint8_t PLIB_USBHS_EP0StatusGet                 (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0StatusClear               (USBHS_MODULE_ID index, USBHS_EP0_ERROR error);
void    PLIB_USBHS_EP0INHandshakeSend           (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0INTokenSend               (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0OUTHandshakeSend          (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0INHandshakeClear          (USBHS_MODULE_ID index);
void    PLIB_USBHS_Endpoint0FIFOFlush           (USBHS_MODULE_ID index);

uint8_t PLIB_USBHS_TxEPStatusGet                (USBHS_MODULE_ID index, uint8_t endpoint);
void    PLIB_USBHS_TxEPStatusClear              (USBHS_MODULE_ID index, uint8_t endpoint, USBHS_TXEP_ERROR error);
void    PLIB_USBHS_EndpointTxFIFOFlush          (USBHS_MODULE_ID index, uint8_t endpoint);
    
uint8_t PLIB_USBHS_RxEPStatusGet                (USBHS_MODULE_ID index, uint8_t endpoint);
void    PLIB_USBHS_RxEPStatusClear              (USBHS_MODULE_ID index, uint8_t endpoint, USBHS_RXEP_ERROR error);
void    PLIB_USBHS_EndpointRxFIFOFlush          (USBHS_MODULE_ID index, uint8_t endpoint);
void    PLIB_USBHS_RxEPINTokenSend              (USBHS_MODULE_ID index, uint8_t endpoint);

// *****************************************************************************
/* Function:
    void  PLIB_USBHS_DeviceAddressSet (USBHS_MODULE_ID index, uint8_t address);

  Summary:
    Set the device USB address.

  Description:
    This function sets the Device address when the module is operating as a USB
    Device. 

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest
    address - address received from the host in SET ADDRESS control request.

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_DeviceAddressSet(USBHS_ID_0, 2);
  </code>

  Remarks:
    This function can be called in device mode only.
*/

void    PLIB_USBHS_DeviceAddressSet (USBHS_MODULE_ID index, uint8_t address);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceAttach (USBHS_MODULE_ID index, uint32_t speed);

  Summary:
    Attaches the device at the specified speed.

  Description:
    This function attaches the device at the specified speed. The attach
    signaling resistors on the data lines are activated.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest
    speed - Speed at which the device should attach.

  Returns:
    None.

  Example:
  <code>
    // Attach at full speed
    PLIB_USBHS_DeviceAttach(USBHS_ID_0, USB_SPEED_FULL);
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_DeviceAttach (USBHS_MODULE_ID index, uint32_t speed);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceDetach (USBHS_MODULE_ID index);

  Summary:
    Detaches the device from the USB.

  Description:
    This function will detach the device from the USB.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_DeviceDetach(USBHS_ID_0);
  </code>

  Remarks:
    None.
*/

void PLIB_USBHS_DeviceDetach (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_EP0StallEnable (USBHS_MODULE_ID index);

  Summary:
    Enables stall on EP0.

  Description:
    This function enables the stall condtion on endpoint 0. Any transaction
    issued by the host will be stalled.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_EP0StallEnable (USBHS_ID_0);
  </code>

  Remarks:
    This function should be called in device mode operation only.
*/

void PLIB_USBHS_EP0StallEnable (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_EP0StallDisable (USBHS_MODULE_ID index);

  Summary:
    Disables stall on EP0.

  Description:
    This function disables the stall condtion on endpoint 0. This function is
    typically called after the PLIB_USBHS_EP0StallEnable function was called to
    enable the stall on the endpoint. 

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance of interest

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_EP0StallDisable (USBHS_ID_0);
  </code>

  Remarks:
    This function should be called in device mode operation only.
*/

void    PLIB_USBHS_EP0StallDisable              (USBHS_MODULE_ID index);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceRxEndpointStallEnable 
    (
        USBHS_MODULE_ID index, 
        uint8_t endpoint
    );

  Summary:
    Enables stall on the specified receive endpoint.

  Description:
    This function enables stall condtion on the specified recieve endpoint.
    Calling this function will cause the OUT transaction on this endpoint to be
    stalled.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest
    endpoint - endpoint to be stalled.

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_DeviceRxEndpointStallEnable (USBHS_ID_0, 02);
  </code>

  Remarks:
    This function should be called in device mode operation only. This function
    should not be called to stall endpoint 0.
*/

void PLIB_USBHS_DeviceRxEndpointStallEnable 
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceTxEndpointStallEnable 
    (
        USBHS_MODULE_ID index, 
        uint8_t endpoint
    );

  Summary:
    Enables stall on the specified transmit endpoint.

  Description:
    This function enables stall condtion on the specified transmit endpoint.
    Calling this function will cause the IN transaction on this endpoint to be
    stalled.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest
    endpoint - endpoint to be stalled.

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_DeviceTxEndpointStallEnable (USBHS_ID_0, 02);
  </code>

  Remarks:
    This function should be called in device mode operation only. This function
    should not be called to stall endpoint 0.
*/

void PLIB_USBHS_DeviceTxEndpointStallEnable 
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceRxEndpointStallDisable 
    (
        USBHS_MODULE_ID index, 
        uint8_t endpoint
    );

  Summary:
    Disables stall on the specified receive endpoint.

  Description:
    This function disables stall condtion on the specified recieve endpoint.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest
    endpoint - endpoint on which stall must be cleared.

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_DeviceRxEndpointStallDisable (USBHS_ID_0, 02);
  </code>

  Remarks:
    This function should be called in device mode operation only. This function
    should not be called to stall endpoint 0.
*/

void PLIB_USBHS_DeviceRxEndpointStallDisable
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
);

// *****************************************************************************
/* Function:
    void PLIB_USBHS_DeviceTxEndpointStallDisable 
    (
        USBHS_MODULE_ID index, 
        uint8_t endpoint
    );

  Summary:
    Disables stall on the specified transmit endpoint.

  Description:
    This function disables stall condtion on the specified transmit endpoint.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance of interest
    endpoint - endpoint on which stall must be cleared.

  Returns:
    None.

  Example:
  <code>
    PLIB_USBHS_DeviceTxEndpointStallDisable (USBHS_ID_0, 02);
  </code>

  Remarks:
    This function should be called in device mode operation only. This function
    should not be called to stall endpoint 0.
*/

void PLIB_USBHS_DeviceTxEndpointStallDisable 
(
    USBHS_MODULE_ID index, 
    uint8_t endpoint
);
void    PLIB_USBHS_DeviceTxEndpointPacketReady  (USBHS_MODULE_ID index, uint8_t endpoint);
void    PLIB_USBHS_EP0SentStallClear            (USBHS_MODULE_ID index);
void    PLIB_USBHS_Endpoint0SetupPacketUnload   (USBHS_MODULE_ID index, void * dest);
void    PLIB_USBHS_EP0SetupEndServiced          (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0RxPktRdyServiced          (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0RxPktRdyServicedDataEnd   (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0TxPktRdyDataEnd           (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0TxPktRdy                  (USBHS_MODULE_ID index);
void    PLIB_USBHS_EP0DataEndSet                (USBHS_MODULE_ID index);
void    PLIB_USBHS_EndpointRxRequestClear       (USBHS_MODULE_ID index, uint8_t endpoint);

void    PLIB_USBHS_GlobalInterruptEnable        (USBHS_MODULE_ID index);
void    PLIB_USBHS_GlobalInterruptDisable       (USBHS_MODULE_ID index);

void    PLIB_USBHS_USBIDOverrideEnable       (USBHS_MODULE_ID index);
void    PLIB_USBHS_USBIDOverrideDisable      (USBHS_MODULE_ID index);
void    PLIB_USBHS_PhyIDMonitoringEnable     (USBHS_MODULE_ID index);
void    PLIB_USBHS_PhyIDMonitoringDisable    (USBHS_MODULE_ID index);
void    PLIB_USBHS_USBIDOverrideValueSet     (USBHS_MODULE_ID index, USBHS_USBID_OVERRIDE_VALUE id);

// *****************************************************************************
// *****************************************************************************
// Section: USBHS Peripheral Library Exists Functions
// *****************************************************************************
// *****************************************************************************
/* The interfaces below indicate the existence of the features on the device.
*/

//******************************************************************************
/* Function:  
    PLIB_USBHS_ExistsEndpointFIFO( USBHS_MODULE_ID index )

  Summary:
    Identifies that the Endpoint FIFO feature exists on the Hi-Speed USB module.

  Description:
    This interface identifies that the Endpoint FIFO feature is available on the 
	Hi-Speed USB module.  When this interface returns true, these functions are
    supported on the device:
    - PLIB_USBHS_EndpointFIFOLoad
    - PLIB_USBHS_EndpointFIFOUnload

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true    - The feature is supported
    - false   - The feature is not supported


  Remarks:
    None.
*/

bool PLIB_USBHS_ExistsEndpointFIFO( USBHS_MODULE_ID index );

bool PLIB_USBHS_ExistsSoftReset( USBHS_MODULE_ID index );

bool PLIB_USBHS_ExistsInterrupts( USBHS_MODULE_ID index );

bool PLIB_USBHS_ExistsHighSpeedSupport( USBHS_MODULE_ID index );

bool PLIB_USBHS_ExistsEndpointOperations(USBHS_MODULE_ID index);

bool PLIB_USBHS_ExistsEP0Status(USBHS_MODULE_ID index);

bool PLIB_USBHS_ExistsTxEPStatus(USBHS_MODULE_ID index);

bool PLIB_USBHS_ExistsRxEPStatus(USBHS_MODULE_ID index);

bool PLIB_USBHS_ExistsModuleControl(USBHS_MODULE_ID index);

bool PLIB_USBHS_ExistsClockResetControl(USBHS_MODULE_ID index);

bool PLIB_USBHS_ExistsUSBIDControl(USBHS_MODULE_ID index);

// ****************************************************************************
// ****************************************************************************
// Section: Included Files (continued)
// ****************************************************************************
// ****************************************************************************
/*  The file included below  maps the interface definitions above to appropriate
    implementations defined in the implementation (imp) file(s).
*/

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif//ndef _PLIB_USBHS_H

/*******************************************************************************
 End of File
*/
