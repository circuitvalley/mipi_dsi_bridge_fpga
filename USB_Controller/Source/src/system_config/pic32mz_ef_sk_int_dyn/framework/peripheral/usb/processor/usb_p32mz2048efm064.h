/* Created by plibgen $Revision: 1.31 $ */

#ifndef _USB_P32MZ2048EFM064_H
#define _USB_P32MZ2048EFM064_H

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

    USB_NUMBER_OF_MODULES = 0

} USB_MODULE_ID;

typedef enum {

    USB_INTERRUPTS_NONE

} USB_INTERRUPTS;

typedef enum {

    USB_ERROR_INTERRUPTS_NONE

} USB_ERROR_INTERRUPTS;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_InterruptFlagSet(USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_InterruptFlagClear(USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_OTG_InterruptFlagGet(USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_InterruptStatus(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_InterruptEnable(USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_InterruptDisable(USB_MODULE_ID index, USB_OTG_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_OTG_InterruptIsEnabled(USB_MODULE_ID index, USB_INTERRUPTS interruptFlag)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_Interrupt(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_OTG_IDPinStateIsTypeA(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_IDPinState(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_OTG_LineStateIsStable(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_LineState(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_OTG_SessionValid(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_SessionValid(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_OTG_BSessionHasEnded(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_BSessionEnd(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_OTG_VBusValid(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_ASessionValid(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_PullUpPullDownSetup(USB_MODULE_ID index, USB_OTG_PULL_UP_PULL_DOWN resistor, bool enableResistor)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_PullUpPullDown(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_VBusPowerOff(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_VBusPowerOn(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_VbusPowerOnOff(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_VBusChargeEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_VBusChargeDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_VbusCharge(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_VBusDischargeEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OTG_VBusDischargeDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsOTG_VbusDischarge(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_ActivityPending(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsActivityPending(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_SleepGuardEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_SleepGuardDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsSleepEntryGuard(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_ModuleIsBusy(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsModuleBusy(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_SuspendEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_SuspendDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsSuspend(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_Enable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_Disable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsModulePower(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_InterruptFlagSet(USB_MODULE_ID index, USB_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_InterruptFlagClear(USB_MODULE_ID index, USB_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_InterruptFlagGet(USB_MODULE_ID index, USB_INTERRUPTS interruptFlag)
{
     return (bool)0;
}

PLIB_INLINE_API USB_INTERRUPTS _PLIB_UNSUPPORTED PLIB_USB_InterruptFlagAllGet(USB_MODULE_ID index)
{
     return (USB_INTERRUPTS)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsGEN_InterruptStatus(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_InterruptEnable(USB_MODULE_ID index, USB_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_InterruptDisable(USB_MODULE_ID index, USB_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_InterruptIsEnabled(USB_MODULE_ID index, USB_INTERRUPTS interruptFlag)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsGEN_Interrupt(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_AllInterruptEnable(USB_MODULE_ID index, USB_INTERRUPTS usbInterruptsFlag, USB_ERROR_INTERRUPTS usbErrorInterruptsFlag, USB_OTG_INTERRUPTS otgInterruptFlag)
{
     
}

PLIB_INLINE_API USB_INTERRUPTS _PLIB_UNSUPPORTED PLIB_USB_InterruptEnableGet(USB_MODULE_ID index)
{
     return (USB_INTERRUPTS)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsALL_Interrupt(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_ErrorInterruptFlagSet(USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_ErrorInterruptFlagClear(USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_ErrorInterruptFlagGet(USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag)
{
     return (bool)0;
}

PLIB_INLINE_API USB_ERROR_INTERRUPTS _PLIB_UNSUPPORTED PLIB_USB_ErrorInterruptFlagAllGet(USB_MODULE_ID index)
{
     return (USB_ERROR_INTERRUPTS)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsERR_InterruptStatus(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_ErrorInterruptEnable(USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_ErrorInterruptDisable(USB_MODULE_ID index, USB_ERROR_INTERRUPTS interruptFlag)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_ErrorInterruptIsEnabled(USB_MODULE_ID index, USB_INTERRUPTS interruptFlag)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsERR_Interrupt(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USB_LastTransactionEndPtGet(USB_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsLastEndpoint(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API USB_BUFFER_DIRECTION _PLIB_UNSUPPORTED PLIB_USB_LastTransactionDirectionGet(USB_MODULE_ID index)
{
     return (USB_BUFFER_DIRECTION)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsLastDirection(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API USB_PING_PONG_STATE _PLIB_UNSUPPORTED PLIB_USB_LastTransactionPingPongStateGet(USB_MODULE_ID index)
{
     return (USB_PING_PONG_STATE)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsLastPingPong(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_LastTransactionDetailsGet(USB_MODULE_ID index, USB_BUFFER_DIRECTION* direction, USB_PING_PONG_STATE* pingpong, uint8_t* endpoint)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsLastTransactionDetails(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_JStateIsActive(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsLiveJState(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_SE0InProgress(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsLiveSingleEndedZero(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_PacketTransferIsDisabled(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_PacketTransferEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_PacketTransferDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsPacketTransfer(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_IsBusyWithToken(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsHostBusyWithToken(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_ResetSignalEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_ResetSignalDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsHostGeneratesReset(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OperatingModeSelect(USB_MODULE_ID index, USB_OPMODES opMode)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsOpModeSelect(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_ResumeSignalingEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_ResumeSignalingDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsResumeSignaling(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_PingPongFreeze(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_PingPongUnfreeze(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_PingPongReset(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsBufferFreeze(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_SOFEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_SOFDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsStartOfFrames(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_TokenSpeedSelect(USB_MODULE_ID index, USB_TOKEN_SPEED tokenSpeed)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsNextTokenSpeed(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_DeviceAddressSet(USB_MODULE_ID index, uint8_t address)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USB_DeviceAddressGet(USB_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsDeviceAddress(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_USB_FrameNumberGet(USB_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsFrameNumber(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API USB_PID _PLIB_UNSUPPORTED PLIB_USB_TokenPIDGet(USB_MODULE_ID index)
{
     return (USB_PID)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_TokenPIDSet(USB_MODULE_ID index, USB_PID pidValue)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_TokenSend(USB_MODULE_ID index, USB_PID pidValue, uint8_t endpoint, uint8_t deviceAddress, bool isLowSpeed)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsTokenPID(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USB_TokenEPGet(USB_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_TokenEPSet(USB_MODULE_ID index, uint8_t epValue)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsTokenEP(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USB_SOFThresholdGet(USB_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_SOFThresholdSet(USB_MODULE_ID index, uint8_t threshold)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsSOFThreshold(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void* _PLIB_UNSUPPORTED PLIB_USB_BDTBaseAddressGet(USB_MODULE_ID index)
{
     return (void*)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BDTBaseAddressSet(USB_MODULE_ID index, void* address)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsBDTBaseAddress(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EyePatternDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EyePatternEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsEyePattern(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_StopInIdleEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_StopInIdleDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsStopInIdle(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_AutoSuspendDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_AutoSuspendEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsAutomaticSuspend(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_PingPongModeSelect(USB_MODULE_ID index, USB_PING_PONG_MODE ppConfig)
{
     
}

PLIB_INLINE_API USB_PING_PONG_MODE _PLIB_UNSUPPORTED PLIB_USB_PingPongModeGet(USB_MODULE_ID index)
{
     return (USB_PING_PONG_MODE)0;
}

PLIB_INLINE_API bool PLIB_USB_ExistsPingPongMode(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_UOEMonitorEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_UOEMonitorDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsUOEMonitor(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OnChipPullUpEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_OnChipPullUpDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsOnChipPullup(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_TransceiverEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_TransceiverDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsOnChipTransceiver(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_FullSpeedEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_FullSpeedDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsSpeedControl(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EP0LSDirectConnectEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EP0LSDirectConnectDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsEP0LowSpeedConnect(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EP0NakRetryEnable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EP0NakRetryDisable(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsEP0NAKRetry(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnTxSelect(USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnRxSelect(USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnTxRxSelect(USB_MODULE_ID index, uint8_t epValue, USB_EP_TXRX epTxRx)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsEPnTxRx(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnRxEnable(USB_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnRxDisable(USB_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnTxEnable(USB_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnTxDisable(USB_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnHandshakeEnable(USB_MODULE_ID index, uint8_t epValue)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnHandshakeDisable(USB_MODULE_ID index, uint8_t epValue)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnControlTransferEnable(USB_MODULE_ID index, uint8_t epValue)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnControlTransferDisable(USB_MODULE_ID index, uint8_t epValue)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnAttributesSet(USB_MODULE_ID index, uint8_t epValue, int direction, bool isControl, bool handshake)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnDirectionDisable(USB_MODULE_ID index, uint8_t epValue, int direction)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnAttributesClear(USB_MODULE_ID index, uint8_t epValue)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_EPnIsStalled(USB_MODULE_ID index, uint8_t epValue)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EPnStallClear(USB_MODULE_ID index, uint8_t epValue)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_EP0HostSetup(USB_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsEPnRxEnable(USB_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void* _PLIB_UNSUPPORTED PLIB_USB_BufferAddressGet(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     return (void*)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferAddressSet(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong, void* bufferAddress)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_USB_BufferByteCountGet(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     return (uint16_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferByteCountSet(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong, uint16_t bufferByteCount)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferCancelReleaseToUSB(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferAllCancelReleaseToUSB(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, int nEndpoints)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferClearAll(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     
}

PLIB_INLINE_API USB_BUFFER_DATA01 _PLIB_UNSUPPORTED PLIB_USB_BufferDataToggleGet(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     return (USB_BUFFER_DATA01)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferDataToggleSelect(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong, USB_BUFFER_DATA01 bufferData01)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferDataToggleSyncEnable(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferDataToggleSyncDisable(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USB_BufferIndexGet(USB_MODULE_ID index, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferPIDBitsClear(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USB_BufferPIDGet(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_BufferReleasedToSW(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferReleaseToUSB(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferSchedule(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong, void* bufferAddress, int16_t bufferByteCount, USB_BUFFER_SCHEDULE_DATA01 bufferData01)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferStallDisable(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferStallEnable(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USB_BufferStallGet(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection, USB_BUFFER_PING_PONG bufferPingPong)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferEP0RxStatusInitialize(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, USB_BUFFER_PING_PONG pingpong, uint16_t bufferByteCount)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USB_BufferClearAllDTSEnable(USB_MODULE_ID index, void* pBDT, USB_PING_PONG_MODE ppMode, uint8_t epValue, USB_BUFFER_DIRECTION bufferDirection)
{
     
}

PLIB_INLINE_API bool PLIB_USB_ExistsBDTFunctions(USB_MODULE_ID index)
{
     return (bool)0;
}

#endif
