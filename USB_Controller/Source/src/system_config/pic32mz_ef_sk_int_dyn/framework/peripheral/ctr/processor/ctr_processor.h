#ifndef _CTR_PROCESSOR_H
#define _CTR_PROCESSOR_H

#if defined(__PIC32MX__)
    #include "ctr_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "ctr_p32xxxx.h"
	
#elif defined(__PIC32MK__)
    #include "ctr_p32xxxx.h"

#elif defined(__PIC32WK__)
    #include "ctr_p32xxxx.h"
	
#else
    #error "Can't find header"

#endif

#endif//B_CTR_PROCESSOR_H
