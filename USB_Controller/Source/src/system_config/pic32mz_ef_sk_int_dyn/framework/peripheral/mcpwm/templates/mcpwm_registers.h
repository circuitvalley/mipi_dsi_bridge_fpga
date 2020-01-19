/*******************************************************************************
  MCPWM Peripheral Library structure implementation
 
 File Name:
    mcpwm_registers.h

  Summary:
    MCPWM PLIB base structure implementation

  Description:
    This header file contains instance structure for mcpwm plib module.
    

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
#include <xc.h>

#ifndef _MCPWM_STRUCTURE_H
#define _MCPWM_STRUCTURE_H

/* This is the register set structure of MCPWM module */


/* This is the register set structure of PWM Module Registers */
typedef struct __attribute__((packed , aligned(4)))
{
    __PTCONbits_t           PTCON;
    volatile unsigned int   PTCONCLR;
    volatile unsigned int   PTCONSET;
    volatile unsigned int   PTCONINV;
    
	volatile unsigned int   PTPER;
    volatile unsigned int   PTPERCLR;
    volatile unsigned int   PTPERSET;
    volatile unsigned int   PTPERINV;
	
	volatile unsigned int   SEVTCMP;
    volatile unsigned int   SEVTCMPCLR;
    volatile unsigned int   SEVTCMPSET;
    volatile unsigned int   SEVTCMPINV;
	
	volatile unsigned int   PMTMR;
    volatile unsigned int   PMTMRCLR;
    volatile unsigned int   PMTMRSET;
    volatile unsigned int   PMTMRINV;
	
	__STCONbits_t         	STCON;
    volatile unsigned int   STCONCLR;
    volatile unsigned int   STCONSET;
    volatile unsigned int   STCONINV;
	
	volatile unsigned int   STPER;
    volatile unsigned int   STPERCLR;
    volatile unsigned int   STPERSET;
    volatile unsigned int   STPERINV;
	
	volatile unsigned int   SSEVTCMP;
    volatile unsigned int   SSEVTCMPCLR;
    volatile unsigned int   SSEVTCMPSET;
    volatile unsigned int   SSEVTCMPINV;
        
	volatile unsigned int   SMTMR;
    volatile unsigned int   SMTMRCLR;
    volatile unsigned int   SMTMRSET;
    volatile unsigned int   SMTMRINV;
	
	__CHOPbits_t   CHOP;
    volatile unsigned int   CHOPCLR;
    volatile unsigned int   CHOPSET;
    volatile unsigned int   CHOPINV;
	
	
	volatile unsigned int   PWMKEY;
    volatile unsigned int   PWMKEYCLR;
    volatile unsigned int   PWMKEYSET;
    volatile unsigned int   PWMKEYINV;
 
} mcpwm_module_registers_t;


/* This is the register set structure of PWM Channel Registers */
typedef struct __attribute__((packed , aligned(4)))
{
    __PWMCON1bits_t        PWMCONx;
    volatile unsigned int   PWMCONxCLR;
    volatile unsigned int   PWMCONxSET;
    volatile unsigned int   PWMCONxINV;
	
	__IOCON1bits_t         	IOCONx;
    volatile unsigned int   IOCONxCLR;
    volatile unsigned int   IOCONxSET;
    volatile unsigned int   IOCONxINV;
	
	volatile unsigned int   PDCx;
    volatile unsigned int   PDCxCLR;
    volatile unsigned int   PDCxSET;
    volatile unsigned int   PDCxINV;
	
	volatile unsigned int   SDCx;
    volatile unsigned int   SDCxCLR;
    volatile unsigned int   SDCxSET;
    volatile unsigned int   SDCxINV;
	
	volatile unsigned int   PHASEx;
    volatile unsigned int   PHASExCLR;
    volatile unsigned int   PHASExSET;
    volatile unsigned int   PHASExINV;
	
	volatile unsigned int   DTRx;
    volatile unsigned int   DTRxCLR;
    volatile unsigned int   DTRxSET;
    volatile unsigned int   DTRxINV;
	
	volatile unsigned int   ALTDTRx;
    volatile unsigned int   ALTDTRxCLR;
    volatile unsigned int   ALTDTRxSET;
    volatile unsigned int   ALTDTRxINV;
	
	volatile unsigned int   DTCOMPx;
    volatile unsigned int   DTCOMPxCLR;
    volatile unsigned int   DTCOMPxSET;
    volatile unsigned int   DTCOMPxINV;
	
	volatile unsigned int   TRIGx;
    volatile unsigned int   TRIGxCLR;
    volatile unsigned int   TRIGxSET;
    volatile unsigned int   TRIGxINV;
	
	__TRGCON1bits_t         TRGCONx;
    volatile unsigned int   TRGCONxCLR;
    volatile unsigned int   TRGCONxSET;
    volatile unsigned int   TRGCONxINV;
	
	volatile unsigned int   STRIGx;
    volatile unsigned int   STRIGxCLR;
    volatile unsigned int   STRIGxSET;
    volatile unsigned int   STRIGxINV;
	
	volatile unsigned int   CAPx;
    volatile unsigned int   CAPxCLR;
    volatile unsigned int   CAPxSET;
    volatile unsigned int   CAPxINV;
	
	__LEBCON1bits_t         LEBCONx;
    volatile unsigned int   LEBCONxCLR;
    volatile unsigned int   LEBCONxSET;
    volatile unsigned int   LEBCONxINV;
	
	volatile unsigned int   LEBDLYx;
    volatile unsigned int   LEBDLYxCLR;
    volatile unsigned int   LEBDLYxSET;
    volatile unsigned int   LEBDLYxINV;
	
	__AUXCON1bits_t         AUXCONx;
    volatile unsigned int   AUXCONxCLR;
    volatile unsigned int   AUXCONxSET;
    volatile unsigned int   AUXCONxINV;
		
	volatile unsigned int   PTMRx;
    volatile unsigned int   PTMRxCLR;
    volatile unsigned int   PTMRxSET;
    volatile unsigned int   PTMRxINV;
	
} mcpwm_channel_registers_t;

#define PWMCONx_TRGIEN_MASK _PWMCON1_TRGIEN_MASK
#define PWMCONx_TRGIF_MASK _PWMCON1_TRGIF_MASK
#define PWMCONx_CLIF_MASK _PWMCON1_CLIF_MASK
#define PWMCONx_CLIEN_MASK _PWMCON1_CLIEN_MASK
#define PWMCONx_XPRES_MASK _PWMCON1_XPRES_MASK
#define PWMCONx_ITB_MASK _PWMCON1_ITB_MASK
#define IOCONx_CLMOD_MASK _IOCON1_CLMOD_MASK
#define PWMCONx_FLTIF_MASK _PWMCON1_FLTIF_MASK
#define PWMCONx_FLTIEN_MASK _PWMCON1_FLTIEN_MASK
#define PWMCONx_PWMLIF_MASK _PWMCON1_PWMLIF_MASK
#define PWMCONx_PWMLIEN_MASK _PWMCON1_PWMLIEN_MASK
#define PWMCONx_PWMLIEN_MASK _PWMCON1_PWMLIEN_MASK
#define PWMCONx_PWMHIF_MASK _PWMCON1_PWMHIF_MASK
#define PWMCONx_PWMHIEN_MASK _PWMCON1_PWMHIEN_MASK
#define IOCONx_OVRENH_MASK _IOCON1_OVRENH_MASK
#define IOCONx_OVRENL_MASK _IOCON1_OVRENL_MASK
#define IOCONx_OSYNC_MASK _IOCON1_OSYNC_MASK
#define IOCONx_PENH_MASK _IOCON1_PENH_MASK
#define IOCONx_PENL_MASK _IOCON1_PENL_MASK
#define IOCONx_SWAP_MASK _IOCON1_SWAP_MASK
#define LEBCONx_PHR_MASK _LEBCON1_PHR_MASK
#define LEBCONx_PHF_MASK _LEBCON1_PHF_MASK
#define LEBCONx_PLR_MASK _LEBCON1_PLR_MASK
#define LEBCONx_PLF_MASK _LEBCON1_PLF_MASK


#endif /*_MCPWM_STRUCTURE_H*/