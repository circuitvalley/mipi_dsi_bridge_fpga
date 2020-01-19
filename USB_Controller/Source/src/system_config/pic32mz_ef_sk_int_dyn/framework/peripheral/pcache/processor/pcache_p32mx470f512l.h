/* Created by plibgen $Revision: 1.31 $ */

#ifndef _PCACHE_P32MX470F512L_H
#define _PCACHE_P32MX470F512L_H

/* Section 1 - Enumerate instances, define constants, VREGs */

#include <xc.h>
#include <stdbool.h>

#include "peripheral/peripheral_common_32bit.h"

/* Default definition used for all API dispatch functions */
#ifndef PLIB_INLINE_API
    #define PLIB_INLINE_API extern inline
#endif

/* Default definition used for all other functions */
#ifndef PLIB_INLINE
    #define PLIB_INLINE extern inline
#endif

typedef enum {

    PCACHE_ID_0 = _PCACHE_BASE_ADDRESS,
    PCACHE_NUMBER_OF_MODULES = 1

} PCACHE_MODULE_ID;

typedef enum {

    PLIB_PCACHE_PREFETCH_DISABLE = 0x00,
    PLIB_PCACHE_PREFETCH_ENABLE_CACHED_REGIONS = 0x01,
    PLIB_PCACHE_PREFETCH_ENABLE_NONCACHED_REGIONS = 0x02,
    PLIB_PCACHE_PREFETCH_ENABLE_ALL = 0x03

} PLIB_PCACHE_PREFETCH_ENABLE;

typedef enum {

    PLIB_PCACHE_DATA_DISABLE = 0x00,
    PLIB_PCACHE_DATA_1LINE = 0x01,
    PLIB_PCACHE_DATA_2LINE = 0x02,
    PLIB_PCACHE_DATA_4LINE = 0x03

} PLIB_PCACHE_DATA_ENABLE;

typedef enum {

    PCACHE_CACHE_TYPE_NONE

} PCACHE_CACHE_TYPE;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/pcache_WaitState_Default.h"
#include "../templates/pcache_PFMAddressWaitStateEnable_Unsupported.h"
#include "../templates/pcache_CachePerformanceCountersEnable_Unsupported.h"
#include "../templates/pcache_PrefetchEnable_Default.h"
#include "../templates/pcache_DataCacheEnable_Default.h"
#include "../templates/pcache_CacheEnable_Unsupported.h"
#include "../templates/pcache_FlashSECInt_Unsupported.h"
#include "../templates/pcache_FlashDEDStatus_Unsupported.h"
#include "../templates/pcache_FlashSECStatus_Unsupported.h"
#include "../templates/pcache_FlashSECCount_Unsupported.h"
#include "../templates/pcache_InvalidateOnPFMProgram_Default.h"
#include "../templates/pcache_InvalidateCache_Unsupported.h"
#include "../templates/pcache_CacheLineSelect_Default.h"
#include "../templates/pcache_CacheLineType_Default.h"
#include "../templates/pcache_CacheLineLock_Default.h"
#include "../templates/pcache_CacheLineValid_Default.h"
#include "../templates/pcache_CacheLineAddr_Default.h"
#include "../templates/pcache_CacheLineFlashType_Default.h"
#include "../templates/pcache_CacheLineMask_Default.h"
#include "../templates/pcache_Word_Default.h"
#include "../templates/pcache_LeastRecentlyUsedState_Default.h"
#include "../templates/pcache_CacheHit_Default.h"
#include "../templates/pcache_CacheMiss_Default.h"
#include "../templates/pcache_PrefetchAbort_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_PCACHE_ExistsWaitState(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsWaitState_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_WaitStateSet(PCACHE_MODULE_ID index, uint32_t clocks)
{
     PCACHE_WaitStateSet_Default(index, clocks);
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_WaitStateGet(PCACHE_MODULE_ID index)
{
     return PCACHE_WaitStateGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsPFMAddressWaitStateEnable(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsPFMAddressWaitStateEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_PFMAddressWaitStateEnable(PCACHE_MODULE_ID index)
{
     PCACHE_PFMAddressWaitStateEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_PFMAddressWaitStateDisable(PCACHE_MODULE_ID index)
{
     PCACHE_PFMAddressWaitStateDisable_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_PFMAddressWaitStateIsEnabled(PCACHE_MODULE_ID index)
{
     return PCACHE_PFMAddressWaitStateIsEnabled_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCachePerformanceCountersEnable(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCachePerformanceCountersEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CachePerformanceCountersEnable(PCACHE_MODULE_ID index)
{
     PCACHE_CachePerformanceCountersEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CachePerformanceCountersDisable(PCACHE_MODULE_ID index)
{
     PCACHE_CachePerformanceCountersDisable_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_CachePerformanceCountersIsEnabled(PCACHE_MODULE_ID index)
{
     return PCACHE_CachePerformanceCountersIsEnabled_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsPrefetchEnable(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsPrefetchEnable_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_PrefetchEnableSet(PCACHE_MODULE_ID index, PLIB_PCACHE_PREFETCH_ENABLE region)
{
     PCACHE_PrefetchEnableSet_Default(index, region);
}

PLIB_INLINE_API PLIB_PCACHE_PREFETCH_ENABLE PLIB_PCACHE_PrefetchEnableGet(PCACHE_MODULE_ID index)
{
     return PCACHE_PrefetchEnableGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsDataCacheEnable(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsDataCacheEnable_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_DataCacheEnableSet(PCACHE_MODULE_ID index, PLIB_PCACHE_DATA_ENABLE dcache_en)
{
     PCACHE_DataCacheEnableSet_Default(index, dcache_en);
}

PLIB_INLINE_API PLIB_PCACHE_DATA_ENABLE PLIB_PCACHE_DataCacheEnableGet(PCACHE_MODULE_ID index)
{
     return PCACHE_DataCacheEnableGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheEnable(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheEnable(PCACHE_MODULE_ID index, PCACHE_CACHE_TYPE cache_type)
{
     PCACHE_CacheEnable_Unsupported(index, cache_type);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheDisable(PCACHE_MODULE_ID index, PCACHE_CACHE_TYPE cache_type)
{
     PCACHE_CacheDisable_Unsupported(index, cache_type);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_CacheIsEnabled(PCACHE_MODULE_ID index, PCACHE_CACHE_TYPE cache_type)
{
     return PCACHE_CacheIsEnabled_Unsupported(index, cache_type);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsFlashSECInt(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsFlashSECInt_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_FlashSECIntEnable(PCACHE_MODULE_ID index)
{
     PCACHE_FlashSECIntEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_FlashSECIntDisable(PCACHE_MODULE_ID index)
{
     PCACHE_FlashSECIntDisable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsFlashDEDStatus(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsFlashDEDStatus_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_FlashDEDStatusGet(PCACHE_MODULE_ID index)
{
     return PCACHE_FlashDEDStatusGet_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_FlashDEDStatusClear(PCACHE_MODULE_ID index)
{
     PCACHE_FlashDEDStatusClear_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsFlashSECStatus(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsFlashSECStatus_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_FlashSECStatusGet(PCACHE_MODULE_ID index)
{
     return PCACHE_FlashSECStatusGet_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_FlashSECStatusSet(PCACHE_MODULE_ID index)
{
     PCACHE_FlashSECStatusSet_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_FlashSECStatusClear(PCACHE_MODULE_ID index)
{
     PCACHE_FlashSECStatusClear_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsFlashSECCount(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsFlashSECCount_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_FlashSECCountSet(PCACHE_MODULE_ID index, uint8_t count)
{
     PCACHE_FlashSECCountSet_Unsupported(index, count);
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_PCACHE_FlashSECCountGet(PCACHE_MODULE_ID index)
{
     return PCACHE_FlashSECCountGet_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsInvalidateOnPFMProgram(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsInvalidateOnPFMProgram_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_InvalidateOnPFMProgramAll(PCACHE_MODULE_ID index)
{
     PCACHE_InvalidateOnPFMProgramAll_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_InvalidateOnPFMProgramUnlocked(PCACHE_MODULE_ID index)
{
     PCACHE_InvalidateOnPFMProgramUnlocked_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsInvalidateCache(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsInvalidateCache_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_ForceInvalidateCacheEnable(PCACHE_MODULE_ID index, PCACHE_CACHE_TYPE cache_type)
{
     PCACHE_ForceInvalidateCacheEnable_Unsupported(index, cache_type);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_ForceInvalidateCacheIsEnabled(PCACHE_MODULE_ID index, PCACHE_CACHE_TYPE cache_type)
{
     return PCACHE_ForceInvalidateCacheIsEnabled_Unsupported(index, cache_type);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_AutoInvalidateCacheEnable(PCACHE_MODULE_ID index, PCACHE_CACHE_TYPE cache_type)
{
     PCACHE_AutoInvalidateCacheEnable_Unsupported(index, cache_type);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_AutoInvalidateCacheDisable(PCACHE_MODULE_ID index, PCACHE_CACHE_TYPE cache_type)
{
     PCACHE_AutoInvalidateCacheDisable_Unsupported(index, cache_type);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_AutoInvalidateCacheIsEnabled(PCACHE_MODULE_ID index, PCACHE_CACHE_TYPE cache_type)
{
     return PCACHE_AutoInvalidateCacheIsEnabled_Unsupported(index, cache_type);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLine(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheLine_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineSelect(PCACHE_MODULE_ID index, uint32_t cache_line)
{
     PCACHE_CacheLineSelect_Default(index, cache_line);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineDeselect(PCACHE_MODULE_ID index, uint32_t cache_line)
{
     PCACHE_CacheLineDeselect_Default(index, cache_line);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineData(PCACHE_MODULE_ID index)
{
     PCACHE_CacheLineData_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineInst(PCACHE_MODULE_ID index)
{
     PCACHE_CacheLineInst_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_CacheLineIsInst(PCACHE_MODULE_ID index)
{
     return PCACHE_CacheLineIsInst_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineType(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheLineType_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineLock(PCACHE_MODULE_ID index)
{
     PCACHE_CacheLineLock_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineUnlock(PCACHE_MODULE_ID index)
{
     PCACHE_CacheLineUnlock_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_CacheLineIsLocked(PCACHE_MODULE_ID index)
{
     return PCACHE_CacheLineIsLocked_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineLock(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheLineLock_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineValid(PCACHE_MODULE_ID index)
{
     PCACHE_CacheLineValid_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineInvalid(PCACHE_MODULE_ID index)
{
     PCACHE_CacheLineInvalid_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_CacheLineIsValid(PCACHE_MODULE_ID index)
{
     return PCACHE_CacheLineIsValid_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineValid(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheLineValid_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineAddrSet(PCACHE_MODULE_ID index, uint32_t addr)
{
     PCACHE_CacheLineAddrSet_Default(index, addr);
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_CacheLineAddrGet(PCACHE_MODULE_ID index)
{
     return PCACHE_CacheLineAddrGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineAddr(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheLineAddr_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineFlashTypeBoot(PCACHE_MODULE_ID index)
{
     PCACHE_CacheLineFlashTypeBoot_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineFlashTypeInst(PCACHE_MODULE_ID index)
{
     PCACHE_CacheLineFlashTypeInst_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_CacheLineFlashTypeIsInst(PCACHE_MODULE_ID index)
{
     return PCACHE_CacheLineFlashTypeIsInst_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineFlashType(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheLineFlashType_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheLineMaskSet(PCACHE_MODULE_ID index, uint32_t mask)
{
     PCACHE_CacheLineMaskSet_Default(index, mask);
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_CacheLineMaskGet(PCACHE_MODULE_ID index)
{
     return PCACHE_CacheLineMaskGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineMask(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheLineMask_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_WordRead(PCACHE_MODULE_ID index, uint32_t word)
{
     return PCACHE_WordRead_Default(index, word);
}

PLIB_INLINE_API void PLIB_PCACHE_WordWrite(PCACHE_MODULE_ID index, uint32_t word, uint32_t data)
{
     PCACHE_WordWrite_Default(index, word, data);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsWord(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsWord_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_LeastRecentlyUsedStateRead(PCACHE_MODULE_ID index)
{
     return PCACHE_LeastRecentlyUsedStateRead_Default(index);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsLeastRecentlyUsedState(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsLeastRecentlyUsedState_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_CacheHitRead(PCACHE_MODULE_ID index)
{
     return PCACHE_CacheHitRead_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheHitWrite(PCACHE_MODULE_ID index, uint32_t data)
{
     PCACHE_CacheHitWrite_Default(index, data);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheHit(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheHit_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_CacheMissRead(PCACHE_MODULE_ID index)
{
     return PCACHE_CacheMissRead_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_CacheMissWrite(PCACHE_MODULE_ID index, uint32_t data)
{
     PCACHE_CacheMissWrite_Default(index, data);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheMiss(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsCacheMiss_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_PrefetchAbortRead(PCACHE_MODULE_ID index)
{
     return PCACHE_PrefetchAbortRead_Default(index);
}

PLIB_INLINE_API void PLIB_PCACHE_PrefetchAbortWrite(PCACHE_MODULE_ID index, uint32_t data)
{
     PCACHE_PrefetchAbortWrite_Default(index, data);
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsPrefetchAbort(PCACHE_MODULE_ID index)
{
     return PCACHE_ExistsPrefetchAbort_Default(index);
}

#endif
