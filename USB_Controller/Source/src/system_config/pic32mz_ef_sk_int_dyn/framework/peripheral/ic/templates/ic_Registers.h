
#ifndef __FRAMEWORK_PERIPHERAL_IC_REGISTERS__
#define __FRAMEWORK_PERIPHERAL_IC_REGISTERS__

typedef struct ic_regs {
	__IC1CONbits_t ICxCON;
	uint32_t ICxCONCLR;
	uint32_t ICxCONSET;
	uint32_t ICxCONINV;
	uint32_t ICxBUF;
	uint32_t DONTUSE[3];
} ic_register_t;

#define ICxCON_ICTMR_MASK	_IC1CON_ICTMR_MASK

#endif /* __FRAMEWORK_PERIPHERAL_IC_REGISTERS__ */
