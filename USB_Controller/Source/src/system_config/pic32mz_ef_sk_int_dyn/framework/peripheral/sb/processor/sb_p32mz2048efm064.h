/* Created by plibgen $Revision: 1.31 $ */

#ifndef _SB_P32MZ2048EFM064_H
#define _SB_P32MZ2048EFM064_H

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

    SB_ID_0 = 0,
    SB_NUMBER_OF_MODULES = 1

} SB_MODULE_ID;

typedef enum {

    PLIB_SB_INIT_ID_CPU_LRS = 0x1,
    PLIB_SB_INIT_ID_CPU_HI = 0x02,
    PLIB_SB_INIT_ID_DMA1_RD_LRS = 0x03,
    PLIB_SB_INIT_ID_DMA1_RD_HI = 0x04,
    PLIB_SB_INIT_ID_DMA1_WR_LRS = 0x05,
    PLIB_SB_INIT_ID_DMA1_WR_HI = 0x06,
    PLIB_SB_INIT_ID_USB1 = 0x07,
    PLIB_SB_INIT_ID_ETH1_RD = 0x08,
    PLIB_SB_INIT_ID_ETH1_WR = 0x09,
    PLIB_SB_INIT_ID_CAN1 = 0x0A,
    PLIB_SB_INIT_ID_CAN2 = 0x0B,
    PLIB_SB_INIT_ID_SQI1 = 0x0C,
    PLIB_SB_INIT_ID_FLASH_CTL = 0x0D,
    PLIB_SB_INIT_ID_CRYPTO = 0x0E

} PLIB_SB_INIT_ID;

typedef enum {

    PLIB_SB_PG_INITIATOR_CPU = 0x0,
    PLIB_SB_PG_INITIATOR_DMA1 = 0x04,
    PLIB_SB_PG_INITIATOR_USB1 = 0x08,
    PLIB_SB_PG_INITIATOR_CAN1 = 0x0C,
    PLIB_SB_PG_INITIATOR_CAN2 = 0x0E,
    PLIB_SB_PG_INITIATOR_ETH1 = 0x10,
    PLIB_SB_PG_INITIATOR_SQI1 = 0x14,
    PLIB_SB_PG_INITIATOR_FLASH_CTL = 0x16,
    PLIB_SB_PG_INITIATOR_CRYPTO = 0x18

} PLIB_SB_PG_INITIATOR;

typedef enum {

    PLIB_SB_TGT_ID_T0 = 0x0,
    PLIB_SB_TGT_ID_T1 = 0x01,
    PLIB_SB_TGT_ID_T2 = 0x02,
    PLIB_SB_TGT_ID_T3 = 0x03,
    PLIB_SB_TGT_ID_T4 = 0x04,
    PLIB_SB_TGT_ID_T5 = 0x05,
    PLIB_SB_TGT_ID_T6 = 0x06,
    PLIB_SB_TGT_ID_T7 = 0x07,
    PLIB_SB_TGT_ID_T8 = 0x08,
    PLIB_SB_TGT_ID_T9 = 0x09,
    PLIB_SB_TGT_ID_T10 = 0x0A,
    PLIB_SB_TGT_ID_T11 = 0x0B,
    PLIB_SB_TGT_ID_T12 = 0x0C,
    PLIB_SB_TGT_ID_T13 = 0x0D

} PLIB_SB_TGT_ID;

typedef enum {

    PLIB_SB_T0_REGION_0 = 0x0000,
    PLIB_SB_T0_REGION_1 = 0x0001,
    PLIB_SB_T1_REGION_0 = 0x0100,
    PLIB_SB_T1_REGION_2 = 0x0102,
    PLIB_SB_T1_REGION_3 = 0x0103,
    PLIB_SB_T1_REGION_4 = 0x0104,
    PLIB_SB_T1_REGION_5 = 0x0105,
    PLIB_SB_T1_REGION_6 = 0x0106,
    PLIB_SB_T1_REGION_7 = 0x0107,
    PLIB_SB_T1_REGION_8 = 0x0108,
    PLIB_SB_T2_REGION_0 = 0x0200,
    PLIB_SB_T2_REGION_1 = 0x0201,
    PLIB_SB_T2_REGION_2 = 0x0202,
    PLIB_SB_T3_REGION_0 = 0x0300,
    PLIB_SB_T3_REGION_1 = 0x0301,
    PLIB_SB_T3_REGION_2 = 0x0302,
    PLIB_SB_T4_REGION_0 = 0x0400,
    PLIB_SB_T4_REGION_2 = 0x0402,
    PLIB_SB_T5_REGION_0 = 0x0500,
    PLIB_SB_T5_REGION_1 = 0x0501,
    PLIB_SB_T5_REGION_2 = 0x0502,
    PLIB_SB_T6_REGION_0 = 0x0600,
    PLIB_SB_T6_REGION_1 = 0x0601,
    PLIB_SB_T7_REGION_0 = 0x0700,
    PLIB_SB_T7_REGION_1 = 0x0701,
    PLIB_SB_T8_REGION_0 = 0x0800,
    PLIB_SB_T8_REGION_1 = 0x0801,
    PLIB_SB_T9_REGION_0 = 0x0900,
    PLIB_SB_T9_REGION_1 = 0x0901,
    PLIB_SB_T10_REGION_0 = 0x0A00,
    PLIB_SB_T11_REGION_0 = 0x0B00,
    PLIB_SB_T11_REGION_1 = 0x0B01,
    PLIB_SB_T12_REGION_0 = 0x0C00,
    PLIB_SB_T13_REGION_0 = 0x0D00

} PLIB_SB_TGT_REGION;

typedef enum {

    PLIB_SB_ERROR_NONE = 0x00,
    PLIB_SB_ERROR_PGV = 0x03

} PLIB_SB_ERROR;

typedef enum {

    REGION_PG_0 = 0x01,
    REGION_PG_1 = 0x02,
    REGION_PG_2 = 0x04,
    REGION_PG_3 = 0x08

} PLIB_SB_REGION_PG;

typedef enum {

    PLIB_SB_INIT_PG_0 = 0x00,
    PLIB_SB_INIT_PG_1 = 0x01,
    PLIB_SB_INIT_PG_2 = 0x02,
    PLIB_SB_INIT_PG_3 = 0x03

} PLIB_SB_INIT_PG;

typedef enum {

    PRIORITY_LRS = 0x00,
    PRIORITY_HI = 0x01

} PLIB_SB_ARB_POLICY;

typedef enum {

    PLIB_SB_OCP_CMD_IDLE = 0x00,
    PLIB_SB_OCP_CMD_WRITE = 0x01,
    PLIB_SB_OCP_CMD_READ = 0x02,
    PLIB_SB_OCP_CMD_READEX = 0x03,
    PLIB_SB_OCP_CMD_WRITE_NON_POST = 0x05,
    PLIB_SB_OCP_CMD_BROADCAST = 0x07

} PLIB_SB_OCP_CMD_CODE;

typedef enum {

    PLIB_SB_PGV_GROUP_ID_NONE

} PLIB_SB_PGV_GROUP_ID;

typedef enum {

    PLIB_SB_PGV_GROUP0_TGT_NONE

} PLIB_SB_PGV_GROUP0_TGT;

typedef enum {

    PLIB_SB_PGV_GROUP1_TGT_NONE

} PLIB_SB_PGV_GROUP1_TGT;

typedef enum {

    PLIB_SB_PGV_GROUP2_TGT_NONE

} PLIB_SB_PGV_GROUP2_TGT;

typedef enum {

    PLIB_SB_PGV_GROUP3_TGT_NONE

} PLIB_SB_PGV_GROUP3_TGT;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/sb_PGVErrGroupStatus_Unsupported.h"
#include "../templates/sb_PGVErrGroup0Status_Unsupported.h"
#include "../templates/sb_PGVErrGroup1Status_Unsupported.h"
#include "../templates/sb_PGVErrGroup2Status_Unsupported.h"
#include "../templates/sb_PGVErrGroup3Status_Unsupported.h"
#include "../templates/sb_PGVErrStatus_Default.h"
#include "../templates/sb_PGVErrClear_Default.h"
#include "../templates/sb_PGVErrInitID_Default.h"
#include "../templates/sb_PGVErrRegion_Default.h"
#include "../templates/sb_PGVErrCmdCode_Default.h"
#include "../templates/sb_PGVErrPG_Default.h"
#include "../templates/sb_PGVErrRptPri_Default.h"
#include "../templates/sb_PGVErrClrSingle_Default.h"
#include "../templates/sb_PGVErrClrMulti_Default.h"
#include "../templates/sb_PGRegAddr_Default.h"
#include "../templates/sb_PGRegSize_Default.h"
#include "../templates/sb_PGRegRdPerm_Default.h"
#include "../templates/sb_PGRegWrPerm_Default.h"
#include "../templates/sb_InitPermGrp_Default.h"
#include "../templates/sb_CPUPriority_Default.h"
#include "../templates/sb_DMAPriority_Default.h"
#include "../templates/sb_ADCPriority_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrGroupStatus(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrGroupStatus_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_SB_PGVErrGroupStatus(SB_MODULE_ID index, PLIB_SB_PGV_GROUP_ID groupId)
{
     return SB_PGVErrGroupStatus_Unsupported(index, groupId);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrGroup0Status(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrGroup0Status_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_SB_PGVErrGroup0Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP0_TGT targetId)
{
     return SB_PGVErrGroup0Status_Unsupported(index, targetId);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrGroup1Status(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrGroup1Status_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_SB_PGVErrGroup1Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP1_TGT targetId)
{
     return SB_PGVErrGroup1Status_Unsupported(index, targetId);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrGroup2Status(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrGroup2Status_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_SB_PGVErrGroup2Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP2_TGT targetId)
{
     return SB_PGVErrGroup2Status_Unsupported(index, targetId);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrGroup3Status(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrGroup3Status_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_SB_PGVErrGroup3Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP3_TGT targetId)
{
     return SB_PGVErrGroup3Status_Unsupported(index, targetId);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrStatus(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_SB_PGVErrorStatus(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorStatus_Default(index, target);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrClear(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrClear_Default(index);
}

PLIB_INLINE_API bool PLIB_SB_PGVErrorMulti(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorMulti_Default(index, target);
}

PLIB_INLINE_API PLIB_SB_ERROR PLIB_SB_PGVErrorCode(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorCode_Default(index, target);
}

PLIB_INLINE_API void PLIB_SB_PGVErrorLogClearSingle(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     SB_PGVErrorLogClearSingle_Default(index, target);
}

PLIB_INLINE_API void PLIB_SB_PGVErrorLogClearMulti(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     SB_PGVErrorLogClearMulti_Default(index, target);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrInitID(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrInitID_Default(index);
}

PLIB_INLINE_API PLIB_SB_INIT_ID PLIB_SB_PGVErrorInitiatorID(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorInitiatorID_Default(index, target);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrRegion(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrRegion_Default(index);
}

PLIB_INLINE_API int PLIB_SB_PGVErrorRegion(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorRegion_Default(index, target);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrCmdCode(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrCmdCode_Default(index);
}

PLIB_INLINE_API PLIB_SB_OCP_CMD_CODE PLIB_SB_PGVErrorCommandCode(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorCommandCode_Default(index, target);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrPG(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrPG_Default(index);
}

PLIB_INLINE_API int PLIB_SB_PGVErrorPermissionGroup(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorPermissionGroup_Default(index, target);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrRptPri(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrRptPri_Default(index);
}

PLIB_INLINE_API void PLIB_SB_PGVErrorReportPrimaryEnable(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     SB_PGVErrorReportPrimaryEnable_Default(index, target);
}

PLIB_INLINE_API void PLIB_SB_PGVErrorReportPrimaryDisable(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     SB_PGVErrorReportPrimaryDisable_Default(index, target);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrClrSingle(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrClrSingle_Default(index);
}

PLIB_INLINE_API bool PLIB_SB_PGVErrorClearSingle(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorClearSingle_Default(index, target);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGVErrClrMulti(SB_MODULE_ID index)
{
     return SB_ExistsPGVErrClrMulti_Default(index);
}

PLIB_INLINE_API bool PLIB_SB_PGVErrorClearMulti(SB_MODULE_ID index, PLIB_SB_TGT_ID target)
{
     return SB_PGVErrorClearMulti_Default(index, target);
}

PLIB_INLINE_API void PLIB_SB_PGRegionAddrSet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, uint32_t phys_addr)
{
     SB_PGRegionAddrSet_Default(index, region, phys_addr);
}

PLIB_INLINE_API uint32_t PLIB_SB_PGRegionAddrGet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region)
{
     return SB_PGRegionAddrGet_Default(index, region);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGRegAddr(SB_MODULE_ID index)
{
     return SB_ExistsPGRegAddr_Default(index);
}

PLIB_INLINE_API void PLIB_SB_PGRegionSizeSet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, uint32_t size)
{
     SB_PGRegionSizeSet_Default(index, region, size);
}

PLIB_INLINE_API uint32_t PLIB_SB_PGRegionSizeGet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region)
{
     return SB_PGRegionSizeGet_Default(index, region);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGRegSize(SB_MODULE_ID index)
{
     return SB_ExistsPGRegSize_Default(index);
}

PLIB_INLINE_API void PLIB_SB_PGRegionReadPermSet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, PLIB_SB_REGION_PG readPerm)
{
     SB_PGRegionReadPermSet_Default(index, region, readPerm);
}

PLIB_INLINE_API void PLIB_SB_PGRegionReadPermClear(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, PLIB_SB_REGION_PG readPerm)
{
     SB_PGRegionReadPermClear_Default(index, region, readPerm);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGRegRdPerm(SB_MODULE_ID index)
{
     return SB_ExistsPGRegRdPerm_Default(index);
}

PLIB_INLINE_API void PLIB_SB_PGRegionWritePermSet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, PLIB_SB_REGION_PG writePerm)
{
     SB_PGRegionWritePermSet_Default(index, region, writePerm);
}

PLIB_INLINE_API void PLIB_SB_PGRegionWritePermClear(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, PLIB_SB_REGION_PG writePerm)
{
     SB_PGRegionWritePermClear_Default(index, region, writePerm);
}

PLIB_INLINE_API bool PLIB_SB_ExistsPGRegWrPerm(SB_MODULE_ID index)
{
     return SB_ExistsPGRegWrPerm_Default(index);
}

PLIB_INLINE_API bool PLIB_SB_ExistsInitPermGrp(SB_MODULE_ID index)
{
     return SB_ExistsInitPermGrp_Default(index);
}

PLIB_INLINE_API void PLIB_SB_InitPermGrpSet(SB_MODULE_ID index, PLIB_SB_PG_INITIATOR initiator, PLIB_SB_INIT_PG pg)
{
     SB_InitPermGrpSet_Default(index, initiator, pg);
}

PLIB_INLINE_API bool PLIB_SB_ExistsCPUPriority(SB_MODULE_ID index)
{
     return SB_ExistsCPUPriority_Default(index);
}

PLIB_INLINE_API void PLIB_SB_CPUPrioritySet(SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority)
{
     SB_CPUPrioritySet_Default(index, priority);
}

PLIB_INLINE_API bool PLIB_SB_ExistsDMAPriority(SB_MODULE_ID index)
{
     return SB_ExistsDMAPriority_Default(index);
}

PLIB_INLINE_API void PLIB_SB_DMAPrioritySet(SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority)
{
     SB_DMAPrioritySet_Default(index, priority);
}

PLIB_INLINE_API bool PLIB_SB_ExistsADCPriority(SB_MODULE_ID index)
{
     return SB_ExistsADCPriority_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_SB_ADCPrioritySet(SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority)
{
     SB_ADCPrioritySet_Unsupported(index, priority);
}

#endif
