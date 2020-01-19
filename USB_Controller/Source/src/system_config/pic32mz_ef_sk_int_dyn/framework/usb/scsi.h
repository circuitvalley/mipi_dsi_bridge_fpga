/*******************************************************************************
  SCSI commands and related definitions

  Company:
    Microchip Technology Inc.

  File Name:
    scsi.h

  Summary:
    SCSI commands and related definitions

  Description:
    This file describes contains constants and definitions that are required
    while implementing or operating MSD device or host based on SCSI commands.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _SCSI_H_
#define _SCSI_H_

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
/* Supported SCSI Primary Commands

  Summary:
    Identifies the supported SCSI Primary Commands

  Description:
    Identifies the supported SCSI Primary Commands

  Remarks:
    None.
*/

typedef enum
{
    SCSI_INQUIRY                        = 0x12,
    SCSI_REQUEST_SENSE                  = 0x03,
    SCSI_MODE_SENSE                     = 0x1A,
    SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL   = 0x1E,
    SCSI_TEST_UNIT_READY                = 0x00

} SCSI_PRIMARY_COMMAND;

// *****************************************************************************
/* Supported SCSI Block Commands

  Summary:
    Identifies the supported SCSI Block Commands

  Description:
    Identifies the supported SCSI Block Commands

  Remarks:
    None.
*/

typedef enum
{
    SCSI_READ_CAPACITY  = 0x25,
    SCSI_READ_10        = 0x28,
    SCSI_WRITE_10       = 0x2A,
    SCSI_STOP_START     = 0x1B,
    SCSI_VERIFY         = 0x2F

} SCSI_BLOCK_COMMAND;

// *****************************************************************************
/* Supported SCSI Multimedia Commands

  Summary:
    Identifies the supported SCSI Multimedia Commands

  Description:
    Identifies the supported SCSI Multimedia Commands

  Remarks:
    None.
*/

typedef enum
{
   SCSI_READ_FORMAT_CAPACITY = 0x23

} SCSI_MULTIMEDIA_COMMAND;

// *****************************************************************************
/* SCSI Sense Data Additional Sense Code (ASC)

  Summary:
    Identifies the Sense Data Additional Sense Codes 

  Description:
    Identifies the Sense Data Additional Sense Codes 

  Remarks:
    None.
*/

typedef enum
{
    SCSI_ASC_NO_ADDITIONAL_SENSE_INFO              = 0x00, 
    SCSI_ASC_INVALID_COMMAND_OPCODE                = 0x20,
    SCSI_ASC_LUN_NOT_SUPPORTED                     = 0x25,
    SCSI_ASC_LUN_DOES_NOT_RESPOND                  = 0x05,
    SCSI_ASC_NOT_READY_TO_READY_CHANGE             = 0x28,
    SCSI_ASC_MEDIUM_NOT_PRESENT                    = 0x3A,
    SCSI_ASC_LUN_NOT_READY_CAUSE_NOT_REPORTABLE    = 0x04,
    SCSI_ASC_LUN_IN_PROCESS                        = 0x04,
    SCSI_ASC_LUN_NOT_READY_INIT_REQD               = 0x04,
    SCSI_ASC_LUN_NOT_READY_INTERVENTION_REQD       = 0x04,
    SCSI_ASC_LUN_NOT_READY_FORMATTING              = 0x04,
    SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE    = 0x21,
    SCSI_ASC_WRITE_PROTECTED                       = 0x27

} SCSI_ASC;

// *****************************************************************************
/* SCSI Sense Data Additional Sense Code Qualifier(ASCQ)

  Summary:
    Identifies the Sense Data Additional Sense Code Qualifier

  Description:
    Identifies the Sense Data Additional Sense Code Qualifier 

  Remarks:
    None.
*/

typedef enum 
{
    SCSI_ASCQ_NO_ADDITIONAL_SENSE_INFO             = 0x00,
    SCSI_ASCQ_INVALID_COMMAND_OPCODE               = 0x00,
    SCSI_ASCQ_LUN_NOT_SUPPORTED                    = 0x00,
    SCSI_ASCQ_LUN_DOES_NOT_RESPOND                 = 0x00,
    SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED              = 0x00,
    SCSI_ASCQ_MEDIUM_NOT_PRESENT                   = 0x00,
    SCSI_ASCQ_LUN_NOT_READY_CAUSE_NOT_REPORTABLE   = 0x00,
    SCSI_ASCQ_LUN_IN_PROCESS                       = 0x01,
    SCSI_ASCQ_LUN_NOT_READY_INIT_REQD              = 0x02,
    SCSI_ASCQ_LUN_NOT_READY_INTERVENTION_REQD      = 0x03,
    SCSI_ASCQ_LUN_NOT_READY_FORMATTING             = 0x04,
    SCSI_ASCQ_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE   = 0x00,
    SCSI_ASCQ_WRITE_PROTECTED                      = 0x00

} SCSI_ASCQ;

// *****************************************************************************
/* SCSI Sense Data Response Code

  Summary:
    Identifies the Sense Data Response Code

  Description:
    Identifies the Sense Data Response Code

  Remarks:
    None.
*/

typedef enum 
{
    SCSI_SENSE_NO_SENSE           = 0x00,
    SCSI_SENSE_RECOVERED_ERROR    = 0x01,
    SCSI_SENSE_NOT_READY          = 0x02,
    SCSI_SENSE_MEDIUM_ERROR       = 0x03,
    SCSI_SENSE_HARDWARE_ERROR     = 0X04,
    SCSI_SENSE_ILLEGAL_REQUEST    = 0x05,
    SCSI_SENSE_UNIT_ATTENTION     = 0x06,
    SCSI_SENSE_DATA_PROTECT       = 0x07,
    SCSI_SENSE_BLANK_CHECK        = 0x08,
    SCSI_SENSE_VENDOR_SPECIFIC    = 0x09,
    SCSI_SENSE_COPY_ABORTED       = 0x0A,
    SCSI_SENSE_ABORTED_COMMAND    = 0x0B,
    SCSI_SENSE_OBSOLETE           = 0x0C,
    SCSI_SENSE_VOLUME_OVERFLOW    = 0x0D,
    SCSI_SENSE_MISCOMPARE         = 0x0E,
    SCSI_SENSE_CURRENT            = 0x70,
    SCSI_SENSE_DEFERRED           = 0x71

} SCSI_SENSE_RESPONSE_CODE;

// *****************************************************************************
/* SCSI Inquiry Response Structure 
  
  Summary:
    SCSI Inquiry Response structure.

  Description:
    SCSI Inquiry Response structure as defined in SPC - 4.
 
  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Peripheral_Qualifier:3; Peripheral_DevType:5; */
    uint8_t peripheral;

    /* Removable medium bit7 = 0 means non removable, rest reserved */
    uint8_t removable;

    /* SPC Version */
    uint8_t version;

    /* b7,b6 Obsolete, b5 Access control coordinator, b4 hierarchical addressing support */
    /* b3:0 response data format 2 indicates response is in format defined by spec */
    uint8_t responseDataFormat;

    /* length in bytes of remaining in standard inquiry data */
    uint8_t additionalLength;

    /* b7 SCCS, b6 ACC, b5-b4 TGPS, b3 3PC, b2-b1 Reserved, b0 Protected */
    uint8_t sccstp;

    /* b7 bque, b6- EncServ, b5-VS, b4-MultiP, b3-MChngr, b2-b1 Obsolete, b0-Addr16 */
    uint8_t bqueetc;

    /* b7-b6 Obsolete, b5-WBUS, b4-Sync, b3-Linked, b2 Obsolete,b1 Cmdque, b0-VS */
    uint8_t cmdQue;

    /* Vendor ID */
    uint8_t vendorID[8];

    /* Product ID */
    uint8_t productID[16];

    /* Product Revision */
    uint8_t productRev[4];
    
} SCSI_INQUIRY_RESPONSE;

// *****************************************************************************
/* SCSI Sense Data structure.

  Summary:
    SCSI Sense Data structure.

  Description:
    SCSI Sense Data structure. This structure represents the header only. The
    application must used this structure as header along with the device
    specific data to provide Sense Data to the host.

  Remarks:
    Always needs to be packed.
*/

typedef union __attribute__((packed)) 
{
    struct
    {
        uint8_t _byte[18];
    };
    struct __attribute__((packed))
    {
        /* b6-b0 is Response Code Fixed or descriptor format */
        unsigned ResponseCode:7;            
        
        /* Set to 1 to indicate information field is a valid value */
        unsigned VALID:1;                    

        uint8_t Obsolete;

        /* Refer SPC-3 Section 4.5.6 */
        unsigned SenseKey:4;                
        
        unsigned Resv:1;

        /* Incorrect Length Indicator */
        unsigned ILI:1;                        
        
        /* End of Medium */
        unsigned EOM:1;                        
        
        /* for READ and SPACE commands */
        unsigned FILEMARK:1;                 
        
        /* Device type or command specific (SPC-33.1.18) */
        uint8_t InformationB0;                
        uint8_t InformationB1;                
        uint8_t InformationB2;                
        uint8_t InformationB3;                
        
        /* Number of additional sense bytes that follow <=244 */
        uint8_t AddSenseLen;                

        /* Depends on command on which exception occurred */
        uint32_t CmdSpecificInfo;            
        
        /* Additional sense code */
        uint8_t ASC;            

        /* Additional sense code qualifier Section 4.5.2.1 SPC-3 */
        uint8_t ASCQ;                        
        
        /* Field Replaceable Unit Code 4.5.2.5 SPC-3 */
        uint8_t FRUC;                        
        
        /* msb is SKSV sense-key specific valid field set=> valid SKS */
        uint8_t SenseKeySpecific[3];        
        
    };
    
    // 18-n additional sense bytes can be defined later
    // 18 Bytes Request Sense Fixed Format

} SCSI_SENSE_DATA;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif
