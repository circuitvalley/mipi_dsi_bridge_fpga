/* Created by plibgen $Revision: 1.31 $ */

#ifndef _ETH_P32MX470F512L_H
#define _ETH_P32MX470F512L_H

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

    ETH_NUMBER_OF_MODULES = 0

} ETH_MODULE_ID;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PauseTimerSet(ETH_MODULE_ID index, uint16_t PauseTimerValue)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_PauseTimerGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsPauseTimer(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_Enable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_Disable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_IsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsEnable(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_StopInIdleEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_StopInIdleDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_StopInIdleIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsStopInIdle(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TxRTSEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TxRTSDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_TxRTSIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsTransmitRTS(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_RxIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRxEnable(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_AutoFlowControlEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_AutoFlowControlDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_AutoFlowControlIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsAutoFlowControl(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ManualFlowControlEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ManualFlowControlDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_ManualFlowControlIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsManualFlowControl(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxBufferCountDecrement(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRxBufferCountDecrement(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_ReceiveBufferSizeGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ReceiveBufferSizeSet(ETH_MODULE_ID index, uint8_t ReceiveBufferSize)
{
     
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_ETH_RxSetBufferSize(ETH_MODULE_ID index, int rxBuffSize)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsReceiveBufferSize(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TxPacketDescAddrSet(ETH_MODULE_ID index, uint8_t* txPacketDescStartAddr)
{
     
}

PLIB_INLINE_API uint8_t* _PLIB_UNSUPPORTED PLIB_ETH_TxPacketDescAddrGet(ETH_MODULE_ID index)
{
     return (uint8_t*)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsTxPacketDescriptorAddress(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxPacketDescAddrSet(ETH_MODULE_ID index, uint8_t* rxPacketDescStartAddr)
{
     
}

PLIB_INLINE_API uint8_t* _PLIB_UNSUPPORTED PLIB_ETH_RxPacketDescAddrGet(ETH_MODULE_ID index)
{
     return (uint8_t*)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRxPacketDescriptorAddress(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_HashTableSet(ETH_MODULE_ID index, uint64_t hashTableValue)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_ETH_HashTableGet(ETH_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFiltersHTSet(ETH_MODULE_ID index, uint64_t htable)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsHashTable(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PatternMatchSet(ETH_MODULE_ID index, uint64_t patternMatchMaskValue)
{
     
}

PLIB_INLINE_API uint64_t _PLIB_UNSUPPORTED PLIB_ETH_PatternMatchGet(ETH_MODULE_ID index)
{
     return (uint64_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PatternMatchChecksumSet(ETH_MODULE_ID index, uint16_t PatternMatchChecksumValue)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_PatternMatchChecksumGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PatternMatchOffsetSet(ETH_MODULE_ID index, uint16_t PatternMatchOffsetValue)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_PatternMatchOffsetGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PatternMatchModeSet(ETH_MODULE_ID index, ETH_PATTERN_MATCH_MODE modeSel)
{
     
}

PLIB_INLINE_API ETH_PATTERN_MATCH_MODE _PLIB_UNSUPPORTED PLIB_ETH_PatternMatchModeGet(ETH_MODULE_ID index)
{
     return (ETH_PATTERN_MATCH_MODE)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFiltersPMClr(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsPatternMatch(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFiltersPMSet(ETH_MODULE_ID index, ETH_PMATCH_MODE mode, uint64_t matchMask, uint32_t matchOffs, uint32_t matchChecksum)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRxFiltersPMSet(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ReceiveFilterEnable(ETH_MODULE_ID index, ETH_RECEIVE_FILTER filter)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ReceiveFilterDisable(ETH_MODULE_ID index, ETH_RECEIVE_FILTER filter)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_ReceiveFilterIsEnable(ETH_MODULE_ID index, ETH_RECEIVE_FILTER filter)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFiltersSet(ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFiltersClr(ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFiltersWrite(ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsReceiveFilters(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFullWmarkSet(ETH_MODULE_ID index, uint8_t watermarkValue)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_RxFullWmarkGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxEmptyWmarkSet(ETH_MODULE_ID index, uint8_t watermarkValue)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_RxEmptyWmarkGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsReceiveWmarks(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_InterruptSourceEnable(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_InterruptSourceDisable(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_InterruptSourceIsEnabled(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
     return (bool)0;
}

PLIB_INLINE_API ETH_INTERRUPT_SOURCES _PLIB_UNSUPPORTED PLIB_ETH_InterruptSourcesGet(ETH_MODULE_ID index)
{
     return (ETH_INTERRUPT_SOURCES)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_InterruptSet(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_InterruptClear(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
     
}

PLIB_INLINE_API ETH_INTERRUPT_SOURCES _PLIB_UNSUPPORTED PLIB_ETH_InterruptsGet(ETH_MODULE_ID index)
{
     return (ETH_INTERRUPT_SOURCES)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_InterruptStatusGet(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_EventsEnableSet(ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_EventsEnableClr(ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_EventsEnableWrite(ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents)
{
     
}

PLIB_INLINE_API PLIB_ETH_EVENTS _PLIB_UNSUPPORTED PLIB_ETH_EventsEnableGet(ETH_MODULE_ID index)
{
     return (PLIB_ETH_EVENTS)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_EventsClr(ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents)
{
     
}

PLIB_INLINE_API PLIB_ETH_EVENTS _PLIB_UNSUPPORTED PLIB_ETH_EventsGet(ETH_MODULE_ID index)
{
     return (PLIB_ETH_EVENTS)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsInterrupt(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_RxPacketCountGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_EthernetIsBusy(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_TransmitIsBusy(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_ReceiveIsBusy(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsEthernetControllerStatus(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxOverflowCountClear(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_RxOverflowCountGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsReceiveOverflowCount(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_FramesTxdOkCountClear(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_FramesTxdOkCountGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsFramesTransmittedOK(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_SingleCollisionCountClear(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_SingleCollisionCountGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MultipleCollisionCountClear(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_MultipleCollisionCountGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsCollisionCounts(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_FramesRxdOkCountClear(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_FramesRxdOkCountGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsFramexReceivedOK(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_FCSErrorCountClear(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_FCSErrorCountGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsFCSErrorCount(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_AlignErrorCountClear(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_AlignErrorCountGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsAlignmentErrorCount(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIResetEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIResetDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MIIResetIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_SimResetEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_SimResetDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_SimResetIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MCSRxResetEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MCSRxResetDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MCSRxResetIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFuncResetEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxFuncResetDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_RxFuncResetIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MCSTxResetEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MCSTxResetDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MCSTxResetIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TxFuncResetEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TxFuncResetDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_TxFuncResetIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMAC_Resets(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_LoopbackEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_LoopbackDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_LoopbackIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TxPauseEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TxPauseDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_TxPauseIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxPauseEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RxPauseDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_RxPauseIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PassAllEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PassAllDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_PassAllIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ReceiveEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ReceiveDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_ReceiveIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ExcessDeferEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ExcessDeferDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_ExcessDeferIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_BackPresNoBackoffEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_BackPresNoBackoffDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_BackPresNoBackoffIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_NoBackoffEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_NoBackoffDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_NoBackoffIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_LongPreambleEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_LongPreambleDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_LongPreambleIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PurePreambleEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PurePreambleDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_PurePreambleIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API ETH_AUTOPAD_OPTION _PLIB_UNSUPPORTED PLIB_ETH_AutoDetectPadGet(ETH_MODULE_ID index)
{
     return (ETH_AUTOPAD_OPTION)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_AutoDetectPadSet(ETH_MODULE_ID index, ETH_AUTOPAD_OPTION option)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_AutoDetectPadClear(ETH_MODULE_ID index, ETH_AUTOPAD_OPTION option)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_CRCEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_CRCDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_CRCIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_DelayedCRCEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_DelayedCRCDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_DelayedCRCIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_HugeFrameEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_HugeFrameDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_HugeFrameIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_FrameLengthCheckEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_FrameLengthCheckDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_FrameLengthCheckIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_FullDuplexEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_FullDuplexDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_FullDuplexIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMAC_Configuration(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_BackToBackIPGGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_BackToBackIPGSet(ETH_MODULE_ID index, uint8_t backToBackIPGValue)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_NonBackToBackIPG1Get(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_NonBackToBackIPG1Set(ETH_MODULE_ID index, uint8_t nonBackToBackIPGValue)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_NonBackToBackIPG2Get(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_NonBackToBackIPG2Set(ETH_MODULE_ID index, uint8_t nonBackToBackIPGValue)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsInterPacketGaps(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_CollisionWindowGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_CollisionWindowSet(ETH_MODULE_ID index, uint8_t collisionWindowValue)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsCollisionWindow(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_ReTxMaxGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ReTxMaxSet(ETH_MODULE_ID index, uint16_t retransmitMax)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRetransmissionMaximum(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_MaxFrameLengthGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MaxFrameLengthSet(ETH_MODULE_ID index, uint16_t MaxFrameLength)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MACSetMaxFrame(ETH_MODULE_ID index, uint16_t maxFrmSz)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMaxFrameLength(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RMIIResetEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RMIIResetDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_RMIIResetIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API ETH_RMII_SPEED _PLIB_UNSUPPORTED PLIB_ETH_RMIISpeedGet(ETH_MODULE_ID index)
{
     return (ETH_RMII_SPEED)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RMIISpeedSet(ETH_MODULE_ID index, ETH_RMII_SPEED RMIISpeed)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRMII_Support(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TestBackPressEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TestBackPressDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_TestBackPressIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TestPauseEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_TestPauseDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_TestPauseIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ShortcutQuantaEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ShortcutQuantaDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_ShortcutQuantaIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMAC_Testing(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMResetEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMResetDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MIIMResetIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API ETH_MIIM_CLK _PLIB_UNSUPPORTED PLIB_ETH_MIIMClockGet(ETH_MODULE_ID index)
{
     return (ETH_MIIM_CLK)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMClockSet(ETH_MODULE_ID index, ETH_MIIM_CLK MIIMClock)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMNoPreEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMNoPreDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MIIMNoPreIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMScanIncrEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMScanIncrDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MIIMScanIncrIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIM_Config(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMScanModeEnable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMScanModeDisable(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MIIMScanModeIsEnabled(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIMScanMode(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMReadStart(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMWriteStart(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIMReadWrite(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_PHYAddressGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_PHYAddressSet(ETH_MODULE_ID index, uint8_t phyAddr)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_RegisterAddressGet(ETH_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_RegisterAddressSet(ETH_MODULE_ID index, uint8_t regAddr)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIMAddresses(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MIIMWriteDataSet(ETH_MODULE_ID index, uint16_t writeData)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_ETH_MIIMReadDataGet(ETH_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIWriteReadData(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_LinkHasFailed(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_DataNotValid(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_ClearDataNotValid(ETH_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MIIMIsScanning(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_ETH_MIIMIsBusy(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIM_Indicators(ETH_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_ETH_StationAddressGet(ETH_MODULE_ID index, uint8_t which)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_StationAddressSet(ETH_MODULE_ID index, uint8_t which, uint8_t stationAddress)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MACSetAddress(ETH_MODULE_ID index, uint8_t* bAddress)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_ETH_MACGetAddress(ETH_MODULE_ID index, uint8_t* bAddress)
{
     
}

PLIB_INLINE_API bool PLIB_ETH_ExistsStationAddress(ETH_MODULE_ID index)
{
     return (bool)0;
}

#endif
