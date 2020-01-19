/* Created by plibgen $Revision: 1.31 $ */

#ifndef _EBI_P32MZ2048EFM064_H
#define _EBI_P32MZ2048EFM064_H

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

    EBI_NUMBER_OF_MODULES = 0

} EBI_MODULE_ID;

typedef enum {

    EBI_CS_TIMING_NONE

} EBI_CS_TIMING;

typedef enum {

    EBI_MEMORY_TYPE_NONE

} EBI_MEMORY_TYPE;

typedef enum {

    EBI_MEMORY_SIZE_NONE

} EBI_MEMORY_SIZE;

typedef enum {

    EBI_PAGE_SIZE_NONE

} EBI_PAGE_SIZE;

typedef enum {

    EBI_STATIC_MEMORY_WIDTH_NONE

} EBI_STATIC_MEMORY_WIDTH;

typedef enum {

    EBI_ADDRESS_PORT_NONE

} EBI_ADDRESS_PORT;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_BaseAddressSet(EBI_MODULE_ID index, int ChipSelectNumber, uint32_t BaseAddress)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_EBI_BaseAddressGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsBaseAddress(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_MemoryCharacteristicsSet(EBI_MODULE_ID index, int ChipSelectNumber, EBI_MEMORY_TYPE MemoryType, EBI_MEMORY_SIZE MemorySize, EBI_CS_TIMING TimingReg)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsMemoryCharacteristics(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_MemoryTimingConfigSet(EBI_MODULE_ID index, int CS_Timing_Reg, int PageReadTime, int DataTurnAroundTime, int WritePulseWidth, int AddressHoldTime, int AddressSetupTime, int ReadCycleTime)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsMemoryTimingConfig(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ReadyModeSet(EBI_MODULE_ID index, bool ReadyPin0, bool ReadyPin1, bool ReadyPin2)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_EBI_ReadyModeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyMode(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_MemoryPagingSet(EBI_MODULE_ID index, int ChipSelectNumber, bool PageMode, EBI_PAGE_SIZE MemoryPageSize)
{
     
}

PLIB_INLINE_API EBI_PAGE_SIZE _PLIB_UNSUPPORTED PLIB_EBI_MemoryPageSizeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (EBI_PAGE_SIZE)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsMemoryPaging(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_EBI_PageModeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsPageMode(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_EBI_PageReadCycleTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsPageReadTime(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_EBI_DataTurnAroundTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsDataTurnAroundTime(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_EBI_WritePulseWidthGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsWritePulseWidth(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_EBI_AddressHoldTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsAddressHoldTime(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_EBI_AddressSetupTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsAddressSetupTime(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_EBI_ReadCycleTimeGet(EBI_MODULE_ID index, int ChipSelectNumber)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadCycleTime(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_FlashTimingSet(EBI_MODULE_ID index, int FlashTiming)
{
     
}

PLIB_INLINE_API int _PLIB_UNSUPPORTED PLIB_EBI_FlashTimingGet(EBI_MODULE_ID index)
{
     return (int)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsFlashTiming(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_MODULE_ID index, int RegisterNumber, EBI_STATIC_MEMORY_WIDTH StaticMemoryWidthRegister)
{
     
}

PLIB_INLINE_API EBI_STATIC_MEMORY_WIDTH _PLIB_UNSUPPORTED PLIB_EBI_StaticMemoryWidthRegisterGet(EBI_MODULE_ID index, int RegisterNumber)
{
     return (EBI_STATIC_MEMORY_WIDTH)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsStaticMemoryWidthRegister(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_FlashPowerDownModeSet(EBI_MODULE_ID index, bool FlashPowerDownMode)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_EBI_FlashPowerDownModeGet(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsFlashPowerDownMode(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ControlEnableSet(EBI_MODULE_ID index, bool EnableBit)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_EBI_ControlEnableGet(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsControlEnable(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_AddressPinEnableBitsSet(EBI_MODULE_ID index, EBI_ADDRESS_PORT AddressPort)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsAddressPinEnableBits(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ReadyPin3ConfigSet(EBI_MODULE_ID index, bool ReadyPin3Enable, bool ReadyPin3Invert)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyPin3Config(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ReadyPin2ConfigSet(EBI_MODULE_ID index, bool ReadyPin2Enable, bool ReadyPin2Invert)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyPin2Config(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ReadyPin1ConfigSet(EBI_MODULE_ID index, bool ReadyPin1Enable, bool ReadyPin1Invert)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyPin1Config(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ReadyPinSensSet(EBI_MODULE_ID index, bool SensitivityControl)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsReadyPinSens(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_FlashResetPinSet(EBI_MODULE_ID index, bool FlashResetPin)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_EBI_FlashResetPinGet(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_EBI_ExistsFlashResetPin(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_WriteOutputControlSet(EBI_MODULE_ID index, bool WriteEnable, bool OutputEnable)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsWriteOutputControl(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ByteSelectPinSet(EBI_MODULE_ID index, bool ByteSelect0, bool ByteSelect1)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsByteSelectPin(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_ChipSelectEnableSet(EBI_MODULE_ID index, bool ChipSelect0, bool ChipSelect1, bool ChipSelect2, bool ChipSelect3)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsChipSelectEnable(EBI_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_EBI_DataEnableSet(EBI_MODULE_ID index, bool DataUpperByte, bool DataLowerByte)
{
     
}

PLIB_INLINE_API bool PLIB_EBI_ExistsDataEnable(EBI_MODULE_ID index)
{
     return (bool)0;
}

#endif
