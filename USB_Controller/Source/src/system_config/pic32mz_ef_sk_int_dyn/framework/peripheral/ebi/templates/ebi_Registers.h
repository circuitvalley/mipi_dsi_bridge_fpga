//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef __EBI_REGISTERS_H_
#define __EBI_REGISTERS_H_

/* EBI Registers */
typedef struct ebi_regs {
    __EBICS0bits_t EBICSx[4];
    uint32_t DONTUSE1[12];
    __EBIMSK0bits_t EBIMSKx[4];
    uint32_t DONTUSE2[12];
    __EBISMT0bits_t EBISMTx[3];
    __EBIFTRPDbits_t EBIFTRPD;
    __EBISMCONbits_t EBISMCON;
} ebi_register_t;

/* EBI Configuration Registers */
typedef struct ebi_cfg_regs {
    __CFGEBIAbits_t CFGEBIA;
    uint32_t DONT_USE[3];
    __CFGEBICbits_t CFGEBIC;
    uint32_t DONT_USE1[3];
} cfgebi_register_t;

#endif /* __EBI_REGISTERS_H_ */

/******************************************************************************
 End of File
*/
