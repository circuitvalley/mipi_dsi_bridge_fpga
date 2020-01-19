
#ifndef _OC_REGISTERS_H_
#define _OC_REGISTERS_H_

typedef struct oc_regs {
    __OC1CONbits_t OCxCON;
    uint32_t OCxCONCLR;
    uint32_t OCxCONSET;
    uint32_t OCxCONINV;
    uint32_t OCxR;
    uint32_t Resvd1[3];
    uint32_t OCxRS;
    uint32_t Resvd2[3];
} oc_register_t;

#define OCxCON_OCM_MASK		_OC1CON_OCM_MASK
#define OCxCON_SIDL_MASK    _OC1CON_OCSIDL_MASK
#define OCxCON_OCFLT_MASK	_OC1CON_OCFLT_MASK
#define OCxCON_ON_MASK		_OC1CON_ON_MASK
#define OCxCON_OCTSEL_MASK	_OC1CON_OCTSEL_MASK

#endif /* _OC_REGISTERS_H_ */
