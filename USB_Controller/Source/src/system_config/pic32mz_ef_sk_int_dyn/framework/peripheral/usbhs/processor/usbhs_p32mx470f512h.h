/* Created by plibgen $Revision: 1.31 $ */

#ifndef _USBHS_P32MX470F512H_H
#define _USBHS_P32MX470F512H_H

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

    USBHS_NUMBER_OF_MODULES = 0

} USBHS_MODULE_ID;

typedef enum {

    USBHS_EPTXRX_INTERRUPT_NONE

} USBHS_EPTXRX_INTERRUPT;

typedef enum {

    USBHS_GEN_INTERRUPT_NONE

} USBHS_GEN_INTERRUPT;

typedef enum {

    USBHS_EP0_ERROR_NONE

} USBHS_EP0_ERROR;

typedef enum {

    USBHS_TXEP_ERROR_NONE

} USBHS_TXEP_ERROR;

typedef enum {

    USBHS_RXEP_ERROR_NONE

} USBHS_RXEP_ERROR;

typedef enum {

    USBHS_VBUS_LEVEL_NONE

} USBHS_VBUS_LEVEL;

typedef enum {

    USBHS_DEVICE_EP0_STATUS_NONE

} USBHS_DEVICE_EP0_STATUS;

typedef enum {

    USBHS_DEVICE_TXEP_STATUS_NONE

} USBHS_DEVICE_TXEP_STATUS;

typedef enum {

    USBHS_DEVICE_RXEP_STATUS_NONE

} USBHS_DEVICE_RXEP_STATUS;

typedef enum {

    USBHS_USBID_OVERRIDE_VALUE_NONE

} USBHS_USBID_OVERRIDE_VALUE;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EndpointFIFOLoad(USBHS_MODULE_ID index, uint8_t endpoint, void* source, size_t nBytes)
{
     
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_USBHS_EndpointFIFOUnload(USBHS_MODULE_ID index, uint8_t endpoint, void* dest)
{
     return (int)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_Endpoint0FIFOFlush(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EndpointTxFIFOFlush(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EndpointRxFIFOFlush(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_Endpoint0SetupPacketLoad(USBHS_MODULE_ID index, void* setupPacket, uint8_t deviceAddress, uint8_t hubAddress, uint8_t hubPortAddress, uint32_t speed)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_Endpoint0SetupPacketUnload(USBHS_MODULE_ID index, void* dest)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceEPFIFOLoad(USBHS_MODULE_ID index, uint8_t endpoint, void* source, size_t nBytes)
{
     
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_USBHS_DeviceEPFIFOUnload(USBHS_MODULE_ID index, uint8_t endpoint, void* dest)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsEndpointFIFO(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_SoftResetEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_SoftResetNRSTXEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_SoftResetDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsSoftReset(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_InterruptEnableSet(USBHS_MODULE_ID index, USBHS_GEN_INTERRUPT generalInterrupts, USBHS_EPTXRX_INTERRUPT transmitInterrupts, USBHS_EPTXRX_INTERRUPT receiveInterrupts)
{
     
}

PLIB_INLINE_API USBHS_EPTXRX_INTERRUPT _PLIB_UNSUPPORTED PLIB_USBHS_TxInterruptFlagsGet(USBHS_MODULE_ID index)
{
     return (USBHS_EPTXRX_INTERRUPT)0;
}

PLIB_INLINE_API USBHS_EPTXRX_INTERRUPT _PLIB_UNSUPPORTED PLIB_USBHS_RxInterruptFlagsGet(USBHS_MODULE_ID index)
{
     return (USBHS_EPTXRX_INTERRUPT)0;
}

PLIB_INLINE_API USBHS_GEN_INTERRUPT _PLIB_UNSUPPORTED PLIB_USBHS_GenInterruptFlagsGet(USBHS_MODULE_ID index)
{
     return (USBHS_GEN_INTERRUPT)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_TxInterruptEnable(USBHS_MODULE_ID index, USBHS_EPTXRX_INTERRUPT interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_RxInterruptEnable(USBHS_MODULE_ID index, USBHS_EPTXRX_INTERRUPT interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_TxInterruptDisable(USBHS_MODULE_ID index, USBHS_EPTXRX_INTERRUPT interruptFlag)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_RxInterruptDisable(USBHS_MODULE_ID index, USBHS_EPTXRX_INTERRUPT interruptFlag)
{
     
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsInterrupts(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_HighSpeedEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_HighSpeedDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USBHS_HighSpeedIsConnected(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USBHS_FullOrHighSpeedIsConnected(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsHighSpeedSupport(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USBHS_DMAErrorGet(USBHS_MODULE_ID index, uint8_t dmaChannel)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USBHS_DMAInterruptGet(USBHS_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DMAOperationEnable(USBHS_MODULE_ID index, uint8_t endpoint, uint8_t dmaChannel, void* address, uint32_t count, bool direction)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_LoadEPInIndex(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API uint8_t* _PLIB_UNSUPPORTED PLIB_USBHS_GetEP0FIFOAddress(USBHS_MODULE_ID index)
{
     return (uint8_t*)0;
}

PLIB_INLINE_API uint8_t* _PLIB_UNSUPPORTED PLIB_USBHS_GetEP0CSRAddress(USBHS_MODULE_ID index)
{
     return (uint8_t*)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_USBHS_GetReceiveDataCount(USBHS_MODULE_ID index, uint8_t endpoint)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USBHS_TestModeEnter(USBHS_MODULE_ID index, uint8_t testMode)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USBHS_TestModeExit(USBHS_MODULE_ID index, uint8_t testMode)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsClockResetControl(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_GlobalInterruptEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_GlobalInterruptDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsUSBIDControl(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_USBIDOverrideEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_USBIDOverrideDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_USBIDOverrideValueSet(USBHS_MODULE_ID index, USBHS_USBID_OVERRIDE_VALUE id)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_PhyIDMonitoringEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_PhyIDMonitoringDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_ResumeEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_ResumeDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_SuspendEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_SuspendDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_ResetEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_ResetDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API USBHS_VBUS_LEVEL _PLIB_UNSUPPORTED PLIB_USBHS_VBUSLevelGet(USBHS_MODULE_ID index)
{
     return (USBHS_VBUS_LEVEL)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USBHS_HostModeIsEnabled(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USBHS_IsBDevice(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_SessionEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_SessionDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceAddressSet(USBHS_MODULE_ID index, uint8_t address)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceAttach(USBHS_MODULE_ID index, uint32_t speed)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceDetach(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsModuleControl(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USBHS_EP0StatusGet(USBHS_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0StatusClear(USBHS_MODULE_ID index, USBHS_EP0_ERROR error)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0SentStallClear(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0INHandshakeSend(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0INTokenSend(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0OUTHandshakeSend(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0INHandshakeClear(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0StallEnable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0StallDisable(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0SetupEndServiced(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0RxPktRdyServiced(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0RxPktRdyServicedDataEnd(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0TxPktRdyDataEnd(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0TxPktRdy(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EP0DataEndSet(USBHS_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsEP0Status(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USBHS_TxEPStatusGet(USBHS_MODULE_ID index, uint8_t endpoint)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_TxEPStatusClear(USBHS_MODULE_ID index, uint8_t endpoint, USBHS_TXEP_ERROR error)
{
     
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsTxEPStatus(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USBHS_RxEPStatusGet(USBHS_MODULE_ID index, uint8_t endpoint)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_RxEPStatusClear(USBHS_MODULE_ID index, uint8_t endpoint, USBHS_RXEP_ERROR error)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_RxEPINTokenSend(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsRxEPStatus(USBHS_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EndpointRxRequestEnable(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_EndpointRxRequestClear(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_HostRxEndpointConfigure(USBHS_MODULE_ID index, uint8_t hostEndpoint, uint32_t speed, uint32_t pipeType, uint16_t endpointSize, uint16_t receiveFIFOAddress, uint16_t fifoSize, uint8_t targetEndpoint, uint8_t targetDevice, uint8_t targetHub, uint8_t targetHubPort, uint8_t nakInterval)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_HostTxEndpointConfigure(USBHS_MODULE_ID index, uint8_t hostEndpoint, uint32_t speed, uint32_t pipeType, uint16_t endpointSize, uint16_t receiveFIFOAddress, uint16_t fifoSize, uint8_t targetEndpoint, uint8_t targetDevice, uint8_t targetHub, uint8_t targetHubPort, uint8_t nakInterval)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_HostTxEndpointDataToggleClear(USBHS_MODULE_ID index, uint8_t hostEndpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_HostRxEndpointDataToggleClear(USBHS_MODULE_ID index, uint8_t hostEndpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceRxEndpointConfigure(USBHS_MODULE_ID index, uint8_t endpoint, uint16_t endpointSize, uint16_t fifoAddress, uint8_t fifoSize, uint32_t transferType)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceTxEndpointConfigure(USBHS_MODULE_ID index, uint8_t endpoint, uint16_t endpointSize, uint16_t fifoAddress, uint8_t fifoSize, uint32_t transferType)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceRxEndpointStallEnable(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceTxEndpointStallEnable(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceRxEndpointStallDisable(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceTxEndpointStallDisable(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USBHS_DeviceTxEndpointPacketReady(USBHS_MODULE_ID index, uint8_t endpoint)
{
     
}

PLIB_INLINE_API bool PLIB_USBHS_ExistsEndpointOperations(USBHS_MODULE_ID index)
{
     return (bool)0;
}

#endif
