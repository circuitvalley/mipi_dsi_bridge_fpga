#ifndef _PLIB_PTG_PROCESSOR_H
#define _PLIB_PTG_PROCESSOR_H

#if defined(__PIC32MX__)
    #include "ptg_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "ptg_p32xxxx.h"

#elif defined(__PIC32MK__)
    #include "ptg_p32xxxx.h"
	
#elif defined(__PIC32WK__)
	#include "ptg_p32xxxx.h"
	
#else
    #error "Can't find header"

#endif

#endif//_PLIB_PTG_PROCESSOR_H
