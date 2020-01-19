/* Created by plibgen $Revision: 1.31 $ */

#ifndef _INT_P32MX470F512L_H
#define _INT_P32MX470F512L_H

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

    INT_ID_0 = 0,
    INT_NUMBER_OF_MODULES = 1

} INT_MODULE_ID;

typedef enum {

    INT_EXTERNAL_INT_SOURCE0 = 0x01,
    INT_EXTERNAL_INT_SOURCE1 = 0x02,
    INT_EXTERNAL_INT_SOURCE2 = 0x04,
    INT_EXTERNAL_INT_SOURCE3 = 0x08,
    INT_EXTERNAL_INT_SOURCE4 = 0x10

} INT_EXTERNAL_SOURCES;

typedef enum {

    INT_DISABLE_INTERRUPT = 0,
    INT_PRIORITY_LEVEL1 = 1,
    INT_PRIORITY_LEVEL2 = 2,
    INT_PRIORITY_LEVEL3 = 3,
    INT_PRIORITY_LEVEL4 = 4,
    INT_PRIORITY_LEVEL5 = 5,
    INT_PRIORITY_LEVEL6 = 6,
    INT_PRIORITY_LEVEL7 = 7

} INT_PRIORITY_LEVEL;

typedef enum {

    INT_SUBPRIORITY_LEVEL0 = 0x00,
    INT_SUBPRIORITY_LEVEL1 = 0x01,
    INT_SUBPRIORITY_LEVEL2 = 0x02,
    INT_SUBPRIORITY_LEVEL3 = 0x03

} INT_SUBPRIORITY_LEVEL;

typedef enum {

    INT_SOURCE_TIMER_CORE = 0x00,
    INT_SOURCE_SOFTWARE_0 = 0x01,
    INT_SOURCE_SOFTWARE_1 = 0x02,
    INT_SOURCE_EXTERNAL_0 = 0x03,
    INT_SOURCE_TIMER_1 = 0x04,
    INT_SOURCE_INPUT_CAPTURE_1_ERROR = 0x05,
    INT_SOURCE_INPUT_CAPTURE_1 = 0x06,
    INT_SOURCE_OUTPUT_COMPARE_1 = 0x07,
    INT_SOURCE_EXTERNAL_1 = 0x08,
    INT_SOURCE_TIMER_2 = 0x09,
    INT_SOURCE_INPUT_CAPTURE_2_ERROR = 0x0A,
    INT_SOURCE_INPUT_CAPTURE_2 = 0x0B,
    INT_SOURCE_OUTPUT_COMPARE_2 = 0x0C,
    INT_SOURCE_EXTERNAL_2 = 0x0D,
    INT_SOURCE_TIMER_3 = 0x0E,
    INT_SOURCE_INPUT_CAPTURE_3_ERROR = 0x0F,
    INT_SOURCE_INPUT_CAPTURE_3 = 0x10,
    INT_SOURCE_OUTPUT_COMPARE_3 = 0x11,
    INT_SOURCE_EXTERNAL_3 = 0x12,
    INT_SOURCE_TIMER_4 = 0x13,
    INT_SOURCE_INPUT_CAPTURE_4_ERROR = 0x14,
    INT_SOURCE_INPUT_CAPTURE_4 = 0x15,
    INT_SOURCE_OUTPUT_COMPARE_4 = 0x16,
    INT_SOURCE_EXTERNAL_4 = 0x17,
    INT_SOURCE_TIMER_5 = 0x18,
    INT_SOURCE_INPUT_CAPTURE_5_ERROR = 0x19,
    INT_SOURCE_INPUT_CAPTURE_5 = 0x1A,
    INT_SOURCE_OUTPUT_COMPARE_5 = 0x1B,
    INT_SOURCE_ADC_1 = 0x1C,
    INT_SOURCE_SPI_1_ERROR = 0x23,
    INT_SOURCE_SPI_1_RECEIVE = 0x24,
    INT_SOURCE_SPI_1_TRANSMIT = 0x25,
    INT_SOURCE_USART_1_ERROR = 0x26,
    INT_SOURCE_USART_1_RECEIVE = 0x27,
    INT_SOURCE_USART_1_TRANSMIT = 0x28,
    INT_SOURCE_I2C_1_BUS = 0x29,
    INT_SOURCE_I2C_1_ERROR = 0x29,
    INT_SOURCE_I2C_1_SLAVE = 0x2A,
    INT_SOURCE_I2C_1_MASTER = 0x2B,
    INT_SOURCE_CHANGE_NOTICE_A = 0x2C,
    INT_SOURCE_CHANGE_NOTICE_B = 0x2D,
    INT_SOURCE_CHANGE_NOTICE_C = 0x2E,
    INT_SOURCE_CHANGE_NOTICE_D = 0x2F,
    INT_SOURCE_CHANGE_NOTICE_E = 0x30,
    INT_SOURCE_CHANGE_NOTICE_F = 0x31,
    INT_SOURCE_CHANGE_NOTICE_G = 0x32,
    INT_SOURCE_PARALLEL_PORT = 0x33,
    INT_SOURCE_PARALLEL_PORT_ERROR = 0x34,
    INT_SOURCE_COMPARATOR_1 = 0x20,
    INT_SOURCE_COMPARATOR_2 = 0x21,
    INT_SOURCE_USB_1 = 0x22,
    INT_SOURCE_DMA_0 = 0x48,
    INT_SOURCE_DMA_1 = 0x49,
    INT_SOURCE_DMA_2 = 0x4A,
    INT_SOURCE_DMA_3 = 0x4B,
    INT_SOURCE_SPI_2_ERROR = 0x35,
    INT_SOURCE_SPI_2_RECEIVE = 0x36,
    INT_SOURCE_SPI_2_TRANSMIT = 0x37,
    INT_SOURCE_USART_2_ERROR = 0x38,
    INT_SOURCE_USART_2_RECEIVE = 0x39,
    INT_SOURCE_USART_2_TRANSMIT = 0x3A,
    INT_SOURCE_I2C_2_BUS = 0x3B,
    INT_SOURCE_I2C_2_ERROR = 0x3B,
    INT_SOURCE_I2C_2_SLAVE = 0x3C,
    INT_SOURCE_I2C_2_MASTER = 0x3D,
    INT_SOURCE_USART_3_ERROR = 0x3E,
    INT_SOURCE_USART_3_RECEIVE = 0x3F,
    INT_SOURCE_USART_3_TRANSMIT = 0x40,
    INT_SOURCE_USART_4_ERROR = 0x41,
    INT_SOURCE_USART_4_RECEIVE = 0x42,
    INT_SOURCE_USART_4_TRANSMIT = 0x43,
    INT_SOURCE_USART_5_ERROR = 0x44,
    INT_SOURCE_USART_5_RECEIVE = 0x45,
    INT_SOURCE_USART_5_TRANSMIT = 0x46,
    INT_SOURCE_CLOCK_MONITOR = 0x1D,
    INT_SOURCE_RTCC = 0x1E,
    INT_SOURCE_FLASH_CONTROL = 0x1F,
    INT_SOURCE_CTMU = 0x47

} INT_SOURCE;

typedef enum {

    INT_VECTOR_CT = _CORE_TIMER_VECTOR,
    INT_VECTOR_CS0 = _CORE_SOFTWARE_0_VECTOR,
    INT_VECTOR_CS1 = _CORE_SOFTWARE_1_VECTOR,
    INT_VECTOR_INT0 = _EXTERNAL_0_VECTOR,
    INT_VECTOR_T1 = _TIMER_1_VECTOR,
    INT_VECTOR_IC1 = _INPUT_CAPTURE_1_VECTOR,
    INT_VECTOR_IC1_ERROR = _INPUT_CAPTURE_1_VECTOR,
    INT_VECTOR_OC1 = _OUTPUT_COMPARE_1_VECTOR,
    INT_VECTOR_INT1 = _EXTERNAL_1_VECTOR,
    INT_VECTOR_T2 = _TIMER_2_VECTOR,
    INT_VECTOR_IC2 = _INPUT_CAPTURE_2_VECTOR,
    INT_VECTOR_IC2_ERROR = _INPUT_CAPTURE_2_VECTOR,
    INT_VECTOR_OC2 = _OUTPUT_COMPARE_2_VECTOR,
    INT_VECTOR_INT2 = _EXTERNAL_2_VECTOR,
    INT_VECTOR_T3 = _TIMER_3_VECTOR,
    INT_VECTOR_IC3 = _INPUT_CAPTURE_3_VECTOR,
    INT_VECTOR_IC3_ERROR = _INPUT_CAPTURE_3_VECTOR,
    INT_VECTOR_OC3 = _OUTPUT_COMPARE_3_VECTOR,
    INT_VECTOR_INT3 = _EXTERNAL_3_VECTOR,
    INT_VECTOR_T4 = _TIMER_4_VECTOR,
    INT_VECTOR_IC4 = _INPUT_CAPTURE_4_VECTOR,
    INT_VECTOR_IC4_ERROR = _INPUT_CAPTURE_4_VECTOR,
    INT_VECTOR_OC4 = _OUTPUT_COMPARE_4_VECTOR,
    INT_VECTOR_INT4 = _EXTERNAL_4_VECTOR,
    INT_VECTOR_T5 = _TIMER_5_VECTOR,
    INT_VECTOR_IC5 = _INPUT_CAPTURE_5_VECTOR,
    INT_VECTOR_IC5_ERROR = _INPUT_CAPTURE_5_VECTOR,
    INT_VECTOR_OC5 = _OUTPUT_COMPARE_5_VECTOR,
    INT_VECTOR_SPI1_FAULT = _SPI_1_VECTOR,
    INT_VECTOR_SPI1_RX = _SPI_1_VECTOR,
    INT_VECTOR_SPI1_TX = _SPI_1_VECTOR,
    INT_VECTOR_UART1_FAULT = _UART_1_VECTOR,
    INT_VECTOR_UART1_RX = _UART_1_VECTOR,
    INT_VECTOR_UART1_TX = _UART_1_VECTOR,
    INT_VECTOR_I2C1_BUS = _I2C_1_VECTOR,
    INT_VECTOR_I2C1_SLAVE = _I2C_1_VECTOR,
    INT_VECTOR_I2C1_MASTER = _I2C_1_VECTOR,
    INT_VECTOR_SPI2_FAULT = _SPI_2_VECTOR,
    INT_VECTOR_SPI2_RX = _SPI_2_VECTOR,
    INT_VECTOR_SPI2_TX = _SPI_2_VECTOR,
    INT_VECTOR_UART2_FAULT = _UART_2_VECTOR,
    INT_VECTOR_UART2_RX = _UART_2_VECTOR,
    INT_VECTOR_UART2_TX = _UART_2_VECTOR,
    INT_VECTOR_I2C2_BUS = _I2C_2_VECTOR,
    INT_VECTOR_I2C2_SLAVE = _I2C_2_VECTOR,
    INT_VECTOR_I2C2_MASTER = _I2C_2_VECTOR,
    INT_VECTOR_UART3_FAULT = _UART_3_VECTOR,
    INT_VECTOR_UART3_RX = _UART_3_VECTOR,
    INT_VECTOR_UART3_TX = _UART_3_VECTOR,
    INT_VECTOR_UART4_FAULT = _UART_4_VECTOR,
    INT_VECTOR_UART4_RX = _UART_4_VECTOR,
    INT_VECTOR_UART4_TX = _UART_4_VECTOR,
    INT_VECTOR_UART5_FAULT = _UART_5_VECTOR,
    INT_VECTOR_UART5_RX = _UART_5_VECTOR,
    INT_VECTOR_UART5_TX = _UART_5_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_A = _CHANGE_NOTICE_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_B = _CHANGE_NOTICE_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_C = _CHANGE_NOTICE_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_D = _CHANGE_NOTICE_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_E = _CHANGE_NOTICE_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_F = _CHANGE_NOTICE_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_G = _CHANGE_NOTICE_VECTOR,
    INT_VECTOR_PMP = _PMP_VECTOR,
    INT_VECTOR_PMP_ERROR = _PMP_VECTOR,
    INT_VECTOR_USB1 = _USB_1_VECTOR,
    INT_VECTOR_RTCC = _RTCC_VECTOR,
    INT_VECTOR_FLASH = _FCE_VECTOR,
    INT_VECTOR_SPI1 = _SPI_1_VECTOR,
    INT_VECTOR_UART1 = _UART_1_VECTOR,
    INT_VECTOR_I2C1 = _I2C_1_VECTOR,
    INT_VECTOR_CN = _CHANGE_NOTICE_VECTOR,
    INT_VECTOR_AD1 = _ADC_VECTOR,
    INT_VECTOR_CMP1 = _COMPARATOR_1_VECTOR,
    INT_VECTOR_CMP2 = _COMPARATOR_2_VECTOR,
    INT_VECTOR_UART3 = _UART_3_VECTOR,
    INT_VECTOR_SPI2 = _SPI_2_VECTOR,
    INT_VECTOR_UART2 = _UART_2_VECTOR,
    INT_VECTOR_I2C2 = _I2C_2_VECTOR,
    INT_VECTOR_FSCM = _FAIL_SAFE_MONITOR_VECTOR,
    INT_VECTOR_CTMU = _CTMU_VECTOR,
    INT_VECTOR_DMA0 = _DMA_0_VECTOR,
    INT_VECTOR_DMA1 = _DMA_1_VECTOR,
    INT_VECTOR_DMA2 = _DMA_2_VECTOR,
    INT_VECTOR_DMA3 = _DMA_3_VECTOR,
    INT_VECTOR_FCE = _FCE_VECTOR,
    INT_VECTOR_UART4 = _UART_4_VECTOR,
    INT_VECTOR_UART5 = _UART_5_VECTOR

} INT_VECTOR;

typedef enum {

    INT_VECTOR_SPACING_0 = 0x00,
    INT_VECTOR_SPACING_8 = 0x01,
    INT_VECTOR_SPACING_16 = 0x02,
    INT_VECTOR_SPACING_32 = 0x04,
    INT_VECTOR_SPACING_64 = 0x08,
    INT_VECTOR_SPACING_128 = 0x10,
    INT_VECTOR_SPACING_256 = 0x20,
    INT_VECTOR_SPACING_512 = 0x40

} INT_VECTOR_SPACING;

typedef enum {

    INT_SHADOW_REGISTER_NONE

} INT_SHADOW_REGISTER;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/int_SingleVectorShadowSet_MX.h"
#include "../templates/int_VectorSelect_Default.h"
#include "../templates/int_ProximityTimerEnable_Default.h"
#include "../templates/int_ProximityTimerControl_Default.h"
#include "../templates/int_ExternalINTEdgeSelect_Default.h"
#include "../templates/int_INTCPUPriority_Default.h"
#include "../templates/int_INTCPUVector_MX.h"
#include "../templates/int_SourceFlag_Default.h"
#include "../templates/int_SourceControl_Default.h"
#include "../templates/int_VectorPriority_Default.h"
#include "../templates/int_CPUCurrentPriorityLevel_Default.h"
#include "../templates/int_EnableControl_PIC32.h"
#include "../templates/int_ShadowRegisterAssign_Unsupported.h"
#include "../templates/int_VariableOffset_Unsupported.h"
#include "../templates/int_SoftwareNMI_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_INT_ExistsSingleVectorShadowSet(INT_MODULE_ID index)
{
     return INT_ExistsSingleVectorShadowSet_MX(index);
}

PLIB_INLINE_API void PLIB_INT_SingleVectorShadowSetDisable(INT_MODULE_ID index)
{
     INT_SingleVectorShadowSetDisable_MX(index);
}

PLIB_INLINE_API void PLIB_INT_SingleVectorShadowSetEnable(INT_MODULE_ID index)
{
     INT_SingleVectorShadowSetEnable_MX(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsVectorSelect(INT_MODULE_ID index)
{
     return INT_ExistsVectorSelect_Default(index);
}

PLIB_INLINE_API void PLIB_INT_MultiVectorSelect(INT_MODULE_ID index)
{
     INT_MultiVectorSelect_Default(index);
}

PLIB_INLINE_API void PLIB_INT_SingleVectorSelect(INT_MODULE_ID index)
{
     INT_SingleVectorSelect_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsProximityTimerEnable(INT_MODULE_ID index)
{
     return INT_ExistsProximityTimerEnable_Default(index);
}

PLIB_INLINE_API void PLIB_INT_ProximityTimerEnable(INT_MODULE_ID index, INT_PRIORITY_LEVEL priority)
{
     INT_ProximityTimerEnable_Default(index, priority);
}

PLIB_INLINE_API void PLIB_INT_ProximityTimerDisable(INT_MODULE_ID index)
{
     INT_ProximityTimerDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsProximityTimerControl(INT_MODULE_ID index)
{
     return INT_ExistsProximityTimerControl_Default(index);
}

PLIB_INLINE_API void PLIB_INT_ProximityTimerSet(INT_MODULE_ID index, uint32_t timerreloadvalue)
{
     INT_ProximityTimerSet_Default(index, timerreloadvalue);
}

PLIB_INLINE_API uint32_t PLIB_INT_ProximityTimerGet(INT_MODULE_ID index)
{
     return INT_ProximityTimerGet_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsExternalINTEdgeSelect(INT_MODULE_ID index)
{
     return INT_ExistsExternalINTEdgeSelect_Default(index);
}

PLIB_INLINE_API void PLIB_INT_ExternalRisingEdgeSelect(INT_MODULE_ID index, INT_EXTERNAL_SOURCES source)
{
     INT_ExternalRisingEdgeSelect_Default(index, source);
}

PLIB_INLINE_API void PLIB_INT_ExternalFallingEdgeSelect(INT_MODULE_ID index, INT_EXTERNAL_SOURCES source)
{
     INT_ExternalFallingEdgeSelect_Default(index, source);
}

PLIB_INLINE_API bool PLIB_INT_ExistsINTCPUPriority(INT_MODULE_ID index)
{
     return INT_ExistsINTCPUPriority_Default(index);
}

PLIB_INLINE_API INT_PRIORITY_LEVEL PLIB_INT_PriorityGet(INT_MODULE_ID index)
{
     return INT_PriorityGet_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsINTCPUVector(INT_MODULE_ID index)
{
     return INT_ExistsINTCPUVector_MX(index);
}

PLIB_INLINE_API INT_VECTOR PLIB_INT_VectorGet(INT_MODULE_ID index)
{
     return INT_VectorGet_MX(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsSourceFlag(INT_MODULE_ID index)
{
     return INT_ExistsSourceFlag_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_SourceFlagGet(INT_MODULE_ID index, INT_SOURCE source)
{
     return INT_SourceFlagGet_Default(index, source);
}

PLIB_INLINE_API void PLIB_INT_SourceFlagSet(INT_MODULE_ID index, INT_SOURCE source)
{
     INT_SourceFlagSet_Default(index, source);
}

PLIB_INLINE_API void PLIB_INT_SourceFlagClear(INT_MODULE_ID index, INT_SOURCE source)
{
     INT_SourceFlagClear_Default(index, source);
}

PLIB_INLINE_API bool PLIB_INT_ExistsSourceControl(INT_MODULE_ID index)
{
     return INT_ExistsSourceControl_Default(index);
}

PLIB_INLINE_API void PLIB_INT_SourceEnable(INT_MODULE_ID index, INT_SOURCE source)
{
     INT_SourceEnable_Default(index, source);
}

PLIB_INLINE_API void PLIB_INT_SourceDisable(INT_MODULE_ID index, INT_SOURCE source)
{
     INT_SourceDisable_Default(index, source);
}

PLIB_INLINE_API bool PLIB_INT_SourceIsEnabled(INT_MODULE_ID index, INT_SOURCE source)
{
     return INT_SourceIsEnabled_Default(index, source);
}

PLIB_INLINE_API bool PLIB_INT_ExistsVectorPriority(INT_MODULE_ID index)
{
     return INT_ExistsVectorPriority_Default(index);
}

PLIB_INLINE_API void PLIB_INT_VectorPrioritySet(INT_MODULE_ID index, INT_VECTOR vector, INT_PRIORITY_LEVEL priority)
{
     INT_VectorPrioritySet_Default(index, vector, priority);
}

PLIB_INLINE_API INT_PRIORITY_LEVEL PLIB_INT_VectorPriorityGet(INT_MODULE_ID index, INT_VECTOR vector)
{
     return INT_VectorPriorityGet_Default(index, vector);
}

PLIB_INLINE_API void PLIB_INT_VectorSubPrioritySet(INT_MODULE_ID index, INT_VECTOR vector, INT_SUBPRIORITY_LEVEL subPriority)
{
     INT_VectorSubPrioritySet_Default(index, vector, subPriority);
}

PLIB_INLINE_API INT_SUBPRIORITY_LEVEL PLIB_INT_VectorSubPriorityGet(INT_MODULE_ID index, INT_VECTOR vector)
{
     return INT_VectorSubPriorityGet_Default(index, vector);
}

PLIB_INLINE_API bool PLIB_INT_ExistsCPUCurrentPriorityLevel(INT_MODULE_ID index)
{
     return INT_ExistsCPUCurrentPriorityLevel_Default(index);
}

PLIB_INLINE_API INT_PRIORITY_LEVEL PLIB_INT_CPUCurrentPriorityLevelGet(INT_MODULE_ID index)
{
     return INT_CPUCurrentPriorityLevelGet_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsEnableControl(INT_MODULE_ID index)
{
     return INT_ExistsEnableControl_PIC32(index);
}

PLIB_INLINE_API void PLIB_INT_Enable(INT_MODULE_ID index)
{
     INT_Enable_PIC32(index);
}

PLIB_INLINE_API void PLIB_INT_Disable(INT_MODULE_ID index)
{
     INT_Disable_PIC32(index);
}

PLIB_INLINE_API bool PLIB_INT_IsEnabled(INT_MODULE_ID index)
{
     return INT_IsEnabled_PIC32(index);
}

PLIB_INLINE_API void PLIB_INT_SetState(INT_MODULE_ID index, INT_STATE_GLOBAL interrupt_state)
{
     INT_SetState_PIC32(index, interrupt_state);
}

PLIB_INLINE_API INT_STATE_GLOBAL PLIB_INT_GetStateAndDisable(INT_MODULE_ID index)
{
     return INT_GetStateAndDisable_PIC32(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsShadowRegisterAssign(INT_MODULE_ID index)
{
     return INT_ExistsShadowRegisterAssign_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_INT_ShadowRegisterAssign(INT_MODULE_ID index, INT_PRIORITY_LEVEL priority, INT_SHADOW_REGISTER shadowRegister)
{
     INT_ShadowRegisterAssign_Unsupported(index, priority, shadowRegister);
}

PLIB_INLINE_API INT_SHADOW_REGISTER _PLIB_UNSUPPORTED PLIB_INT_ShadowRegisterGet(INT_MODULE_ID index, INT_PRIORITY_LEVEL priority)
{
     return INT_ShadowRegisterGet_Unsupported(index, priority);
}

PLIB_INLINE_API bool PLIB_INT_ExistsVariableOffset(INT_MODULE_ID index)
{
     return INT_ExistsVariableOffset_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_INT_VariableVectorOffsetSet(INT_MODULE_ID index, INT_VECTOR vector, uint32_t offset)
{
     INT_VariableVectorOffsetSet_Unsupported(index, vector, offset);
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_INT_VariableVectorOffsetGet(INT_MODULE_ID index, INT_VECTOR vector)
{
     return INT_VariableVectorOffsetGet_Unsupported(index, vector);
}

PLIB_INLINE_API bool PLIB_INT_ExistsSoftwareNMI(INT_MODULE_ID index)
{
     return INT_ExistsSoftwareNMI_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_INT_SoftwareNMITrigger(INT_MODULE_ID index)
{
     INT_SoftwareNMITrigger_Unsupported(index);
}

#endif
