/*******************************************************************************
  System Bus Peripheral Library Interface Header

  Company:      Microchip Technology Inc.

  File Name:    plib_sb.h

  Summary:
    Defines the System Bus Peripheral Library interface

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the System Bus
    Peripheral Library for Microchip microcontrollers.  The
    definitions in this file are for the System Bus controller module.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2012-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _PLIB_SB_H
#define _PLIB_SB_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Includes
// *****************************************************************************
// *****************************************************************************
/* See Bottom of file for implementation of header include files.
*/

#include "peripheral/sb/processor/sb_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Peripheral Library Interface Routines
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrGroupStatus(SB_MODULE_ID index, PLIB_SB_PGV_GROUP_ID groupId);

  Summary:
    Identifies whether a permission group violation has been reported for the
    specified target group.

  Description:
    This function identifies whether a permission group violation has been
    reported for the specified target group.

  Precondition:
    None.

  Parameters:
    index   - Identifier for the device instance.
    groupId - Target group to be checked.

  Returns:
    - true  - Target group is reporting a permission group violation.
    - false - Target group is not reporting a permission group violation.

  Example:
    <code>
    bool sbPgv;
    sbPgv = PLIB_SB_PGVErrGroupStatus(SB_ID_0, PLIB_SB_PGV_GROUP0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

bool PLIB_SB_PGVErrGroupStatus(SB_MODULE_ID index, PLIB_SB_PGV_GROUP_ID groupId);

/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrGroup0Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP0_TGT targetId);

  Summary:
    Identifies whether a permission group violation has been reported for the
    Target Group 0.

  Description:
    This function identifies whether a permission group violation has been
    reported for Target Group 0.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    targetId - Target to be checked.

  Returns:
    - true  - Target is reporting a permission group violation.
    - false - Target is not reporting a permission group violation.

  Example:
    <code>
    bool sbPgv;
    sbPgv = PLIB_SB_PGVErrGroup0Status(SB_ID_0, PLIB_SB_PGV_GROUP0_T1_PGV);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/
bool PLIB_SB_PGVErrGroup0Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP0_TGT targetId);

/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrGroup1Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP1_TGT targetId);

  Summary:
    Identifies whether a permission group violation has been reported for the
    Target Group 1.

  Description:
    This function identifies whether a permission group violation has been
    reported for Target Group 1.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    targetId - Target to be checked.

  Returns:
    - true  - Target is reporting a permission group violation.
    - false - Target is not reporting a permission group violation.

  Example:
    <code>
    bool sbPgv;
    sbPgv = PLIB_SB_PGVErrGroup1Status(SB_ID_0, PLIB_SB_PGV_GROUP1_T11_PGV);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/
bool PLIB_SB_PGVErrGroup1Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP1_TGT targetId);

/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrGroup2Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP2_TGT targetId);

  Summary:
    Identifies whether a permission group violation has been reported for the
    Target Group 2.

  Description:
    This function identifies whether a permission group violation has been
    reported for Target Group 2.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    targetId - Target to be checked.

  Returns:
    - true  - Target is reporting a permission group violation.
    - false - Target is not reporting a permission group violation.

  Example:
    <code>
    bool sbPgv;
    sbPgv = PLIB_SB_PGVErrGroup2Status(SB_ID_0, PLIB_SB_PGV_GROUP2_T14_PGV);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/
bool PLIB_SB_PGVErrGroup2Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP2_TGT targetId);

/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrGroup3Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP3_TGT targetId);

  Summary:
    Identifies whether a permission group violation has been reported for the
    Target Group 3.

  Description:
    This function identifies whether a permission group violation has been
    reported for Target Group 3.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    targetId - Target to be checked.

  Returns:
    - true  - Target is reporting a permission group violation.
    - false - Target is not reporting a permission group violation.

  Example:
    <code>
    bool sbPgv;
    sbPgv = PLIB_SB_PGVErrGroup3Status(SB_ID_0, PLIB_SB_PGV_GROUP3_T16_PGV);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/
bool PLIB_SB_PGVErrGroup3Status(SB_MODULE_ID index, PLIB_SB_PGV_GROUP3_TGT targetId);

/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrorStatus ( SB_MODULE_ID index , PLIB_SB_TGT_ID target )

  Summary:
    Identifies whether a permission group violation has been reported for the
    specified target.

  Description:
    This function identifies whether a permission group violation has been
    reported for the specified target.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - Target to be checked.

  Returns:
    - true  - Target is reporting a permission group violation.
    - false - Target is not reporting a permission group violation.

  Example:
    <code>
    bool sbPgv;
    sbPgv = PLIB_SB_PGVErrorStatus(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

bool PLIB_SB_PGVErrorStatus(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrorMulti( SB_MODULE_ID index , PLIB_SB_TGT_ID target )

  Summary:
    Indicates if more than one permission group violation has occurred since
    last cleared.

  Description:
    This function indicates if more than one permission group violation has
    occurred since last cleared.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target to be read.

  Returns:
    - true  - Multiple permission group violations.
    - false - Single or no permission group violations.

  Example:
    <code>
    bool multiPGV;
    multiPGV = PLIB_SB_PGVErrorMulti(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

bool PLIB_SB_PGVErrorMulti(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    PLIB_SB_ERROR PLIB_SB_PGVErrorCode( SB_MODULE_ID index, PLIB_SB_TGT_ID target )

  Summary:
    Returns a value corresponding to the type of error logged for the
    specified target.

  Description:
    This function returns a value corresponding to the type of error logged for
    the specified target.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target to be read.

  Returns:
    PLIB_SB_ERROR enumeration representing the type of SB error logged
    states.

  Example:
    <code>
    PLIB_SB_ERROR error;
    error = PLIB_SB_ERROR PLIB_SB_PGVErrorCode(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

PLIB_SB_ERROR PLIB_SB_PGVErrorCode(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    void PLIB_SB_PGVErrorLogClearSingle( SB_MODULE_ID index, PLIB_SB_TGT_ID target )

  Summary:
    Clears a single protection group violation error from the specified target
    error log register.

  Description:
    This function clears a single protection group violation error from the
    specified target error log register. Single errors are cleared by writing
    a '1' to the CODE field of the SBTxELOG1 register for the specified target.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target to be cleared.

  Returns:
    None.

  Example:
    <code>
    PLIB_SB_PGVErrorLogClearSingle(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

void PLIB_SB_PGVErrorLogClearSingle(SB_MODULE_ID index , PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    void PLIB_SB_PGVErrorLogClearMulti( SB_MODULE_ID index, PLIB_SB_TGT_ID target )

  Summary:
    Clears a multiple protection group violations error from the specified
    target error log register.

  Description:
    This function clears a multiple protection group violations error from the
    specified target error log register. Multiple errors are cleared by writing
    a '1' to both the MULTI and CODE fields of the SBTxELOG1 register for the
    specified target.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target to be cleared.

  Returns:
    None.

  Example:
    <code>
    PLIB_SB_PGVErrorLogClearMulti(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

void PLIB_SB_PGVErrorLogClearMulti(SB_MODULE_ID index , PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
   PLIB_SB_INIT_ID PLIB_SB_PGVErrorInitiatorID( SB_MODULE_ID index , PLIB_SB_TGT_ID target )

  Summary:
    Returns the ID of the Initiator that caused the protection violation

  Description:
    This function returns the ID of the Initiator that caused the protection
    violation.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target.

  Returns:
    An enumerated value representing the ID of the initiator that caused the
    protection violation.

  Example:
    <code>
    PLIB_SB_INIT_ID id;
    id = PLIB_SB_PGVErrorInitiatorID(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

PLIB_SB_INIT_ID PLIB_SB_PGVErrorInitiatorID( SB_MODULE_ID index , PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    int PLIB_SB_PGVErrorRegion( SB_MODULE_ID index, PLIB_SB_TGT_ID target )

  Summary:
    Returns the number of the protection region in the specified target address
    space that caused the protection violation.

  Description:
    This function returns the number of the protection region in the specified
    target address space that caused the protection violation. Note that if
    there are no other region matches, region 0 (the default region that spans
    the entire target address space) will always match, and this function will
    return 0.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance.
    target - The target.

  Returns:
    Region number in target address space or -1 on unrecognized target.

  Example:
    <code>
    int region;
    region = PLIB_SB_PGVErrorRegion(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

int PLIB_SB_PGVErrorRegion(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    PLIB_SB_OCP_CMD_CODE PLIB_SB_PGVErrorCommandCode( SB_MODULE_ID index , PLIB_SB_TGT_ID target )

  Summary:
    Returns the OCP command code of the transaction that caused the protection
    violation for the specified target.

  Description:
    This function returns the OCP command code of the transaction that caused
    the protection violation for the specified target.

  Precondition:
    None.

  Parameters:
    index - Identifier for the device instance.
    target - The target.

  Returns:
    OCP command code

  Example:
    <code>
    PLIB_SB_OCP_CMD_CODE commandCode;
    commandCode = PLIB_SB_PGVErrorCommandCode(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the specific
    device data sheet for availability.
*/

PLIB_SB_OCP_CMD_CODE PLIB_SB_PGVErrorCommandCode(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    int PLIB_SB_PGVErrorPermissionGroup( SB_MODULE_ID index , PLIB_SB_TGT_ID target )

  Summary:
    Returns the permission group of the protection region in a target address
    space that caused the protection violation for the specified target.

  Description:
    This function returns the permission group of the protection region in a
    target address space that caused the protection violation for the specified
    target.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target.

  Return:
    None.

  Example:
    <code>
    int pg;
    pg = PLIB_SB_PGVErrorPermissionGroup(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.
*/

int PLIB_SB_PGVErrorPermissionGroup(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    void PLIB_SB_PGVErrorReportPrimaryEnable( SB_MODULE_ID index , PLIB_SB_TGT_ID target )

  Summary:
    Enables primary permission group error reporting for the specified target
    to the SB flag register.

  Description:
    This function enables primary permission group error reporting for the
    specified target to the SB flag register.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target.

  Return:
    None.

  Example:
    <code>
    PLIB_SB_PGVErrorReportPrimaryEnable(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Reporting of primary errors is disabled at reset.
*/

void PLIB_SB_PGVErrorReportPrimaryEnable(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    void PLIB_SB_PGVErrorReportPrimaryDisable( SB_MODULE_ID index, PLIB_SB_TGT_ID target )

  Summary:
    Disables primary permission group error reporting for the specified target
    to the SB flag register.

  Description:
    This function disables primary permission group error reporting for the
    specified target to the SB flag register.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target.

  Return:
    None.

  Example:
    <code>
    PLIB_SB_PGVErrorReportPrimaryDisable(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Reporting of primary errors is disabled at reset.
*/

void PLIB_SB_PGVErrorReportPrimaryDisable(SB_MODULE_ID index, PLIB_SB_TGT_ID target );


/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrorClearSingle( SB_MODULE_ID index, PLIB_SB_TGT_ID target )

  Summary:
    Clears a single permission group error for the specified target.

  Description:
    This function clears a single permission group error for the specified target.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target.

  Return:
    sing   - Returns the value of the CLEAR bit in the SBTxECLRM register
             for the specified target. The act of reading this bit clears
             the error.

  Example:
    <code>
    bool sing;
    sing = PLIB_SB_PGVErrorClearSingle(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.
*/

bool PLIB_SB_PGVErrorClearSingle(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    bool PLIB_SB_PGVErrorClearMulti( SB_MODULE_ID index, PLIB_SB_TGT_ID target )

  Summary:
    Clears multiple permission group errors for the specified target.

  Description:
    This function clears multiple permission group errors for the specified target.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    target - The target.

  Return:
    mult   - Returns the value of the CLEAR bit in the SBTxECLRM register
             for the specified target. The act of reading this bit clears
             the error.

  Example:
    <code>
    bool mult;
    mult = PLIB_SB_PGVErrorClearMulti(SB_ID_1, PLIB_SB_TGT_ID_T0);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.
*/

bool PLIB_SB_PGVErrorClearMulti(SB_MODULE_ID index, PLIB_SB_TGT_ID target);


/*******************************************************************************
  Function:
    void PLIB_SB_PGRegionAddrSet( SB_MODULE_ID index, PLIB_SB_TGT_REGION region,
                                    uint32_t phys_addr)

  Summary:
    Sets the base address for a permission group region within a target's
    physical address space. Not all regions are programmable.

  Description:
    This function sets the base address for a permission group region within a
    target's physical address space. Not all regions are programmable.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance.
    region    - The region for which the base address is set.
    phys_addr - The base address of the region. Must be aligned to the
                intended size of the region.

  Return:
    None.

  Example:
    <code>
    // Set up two regions in PFM with full read and write permission
    #define REGION_5_BASE_ADDR  0x1D000000
    #define REGION_6_BASE_ADDR  (REGION5_BASE_ADDR + (32*1024))
    #define REGION_5_SIZE       0x06 // 32KB
    #define REGION_6_SIZE       0x05 // 16KB
    #define FULL_PERM           (REGION_PG_0 | REGION_PG_1 | REGION_PG_2 | REGION_PG_3)

    PLIB_SB_PGRegionAddrSet(SB_ID_1, PLIB_SB_T1_REGION_5, REGION_5_BASE_ADDR);
    PLIB_SB_PGRegionAddrSet(SB_ID_1, PLIB_SB_T1_REGION_6, REGION_6_BASE_ADDR);

    PLIB_SB_PGRegionSizeSet(SB_ID_1, PLIB_SB_T1_REGION_5, REGION_5_SIZE);
    PLIB_SB_PGRegionSizeSet(SB_ID_1, PLIB_SB_T1_REGION_6, REGION_6_SIZE);

    PLIB_SB_PGRegionReadPermSet(SB_ID_1, PLIB_T1_REGION_5, FULL_PERM);
    PLIB_SB_PGRegionReadPermSet(SB_ID_1, PLIB_T1_REGION_6, FULL_PERM);

    PLIB_SB_PGRegionWritePermSet(SB_ID_1, PLIB_T1_REGION_5, FULL_PERM);
    PLIB_SB_PGRegionWritePermSet(SB_ID_1, PLIB_T1_REGION_6, FULL_PERM);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Region 0 of all targets have a defined start address which may not be
    changed. Some regions (such as those containing peripheral SFRs) are
    fixed and may not be changed. Please refer to the specific device data
    sheet for details.
*/

void PLIB_SB_PGRegionAddrSet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, uint32_t phys_addr);

/*******************************************************************************
  Function:
    uint32_t PLIB_SB_PGRegionAddrGet( SB_MODULE_ID index, PLIB_SB_TGT_REGION region )

  Summary:
    Returns the base address for a permission group region within a target's
    physical address space.

  Description:
    This function returns the base address for a permission group region within
    a target's physical address space.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    region   - The region for which the address is returned.

  Return:
    uint32_t - The base address of the region.

  Example:
    <code>
    uint32_t T1R6BaseAddress;
    T1R6BaseAddress = PLIB_SB_PGRegionAddrGet(SB_ID_1, PLIB_T1_REGION_6);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.
*/

uint32_t PLIB_SB_PGRegionAddrGet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region);


/*******************************************************************************
  Function:
    void PLIB_SB_PGRegionSizeSet( SB_MODULE_ID index, PLIB_SB_TGT_REGION region, uint32_t size)

  Summary:
    Sets the size for a permission group region within a target's physical
    address space. Not all regions are programmable.

  Description:
    This function sets the size for a permission group region within a target's
    physical address space. Not all regions are programmable.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    region - The region for which the size is to be set. Not all regions are
             programmable.
    size   - The actual size of the region being programmed is calculated as
             follows: 2**(size-1)*1024 bytes

  Return:
    None.

  Example:
    See the code example for PLIB_SB_PGRegionAddrSet.

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Region 0 of all targets encompasses the entire address range of the target,
    and may not be changed. Some regions (such as those containing peripheral
    SFRs) may not be changed. Please refer to the specific device data sheet
    for details.
*/

void PLIB_SB_PGRegionSizeSet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, uint32_t size);


/*******************************************************************************
  Function:
    uint32_t PLIB_SB_PGRegionSizeGet( SB_MODULE_ID index, PLIB_SB_TGT_REGION region )

  Summary:
    Returns the size for a permission group region within a target's physical
    address space.

  Description:
    This function returns the size for a permission group region within a
    target's physical address space. Not all regions are programmable.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.
    region - The region for which the size is to be set. Not all regions are
             programmable.

  Return:
    The size of the region.

  Example:
    <code>
    uint32_t T1R6Size;
    T1R6Size = PLIB_SB_PGRegionSizeGet(SB_ID_1, PLIB_T1_REGION_6);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Region 0 of all targets encompasses the entire address range of the target,
    and may not be changed. Some regions (such as those containing peripheral
    SFRs) may not be changed. Please refer to the specific device data sheet
    for details.
*/

uint32_t PLIB_SB_PGRegionSizeGet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region);


/*******************************************************************************
  Function:
    void PLIB_SB_PGRegionReadPermSet( SB_MODULE_ID index, PLIB_SB_TGT_REGION region,
                                        PLIB_SB_REGION_PG readPerm )

  Summary:
    Sets the permission bit(s) corresponding to the requested read permissions
    for a permission group region within a target's physical address space.
    Not all regions are programmable.

  Description:
    This function sets the permission bit(s) corresponding to the requested read
    permissions for a permission group region within a target's physical address
    space. Not all regions are programmable.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    region   - The region number within the target's address space to be given
               permissions. Not all regions are programmable.
    readPerm - Bitwise OR of the groups given read permission for the region.

  Return:
    None.

  Example:
    See the code example for PLIB_SB_PGRegionAddrSet.

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Default reset value for all programmable regions is read permission for
    all groups.
*/

void PLIB_SB_PGRegionReadPermSet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, PLIB_SB_REGION_PG readPerm);


/*******************************************************************************
  Function:
    void PLIB_SB_PGRegionReadPermClear( SB_MODULE_ID index, PLIB_SB_TGT_REGION region,
                                        PLIB_SB_REGION_PG readPerm )

  Summary:
    Clears the permission bit(s) corresponding to the requested read permissions
    for a permission group region within a target's physical address space.
    Not all regions are programmable.

  Description:
    This function clears the permission bit(s) corresponding to the requested read
    permissions for a permission group region within a target's physical address
    space. Not all regions are programmable.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    region   - The region number within the target's address space to be given
               permissions. Not all regions are programmable.
    readPerm - Bitwise OR of the groups given read permission for the region.

  Return:
    None.

  Example:
    See the code example for PLIB_SB_PGRegionAddrSet.

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Default reset value for all programmable regions is read permission for
    all groups.
*/

void PLIB_SB_PGRegionReadPermClear(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, PLIB_SB_REGION_PG readPerm);


/*******************************************************************************
  Function:
    void PLIB_SB_PGRegionWritePermSet( SB_MODULE_ID index, PLIB_SB_TGT_REGION region,
                                        PLIB_SB_REGION_PG writePerm )

  Summary:
    Sets the permission bit(s) corresponding to the requested write permissions
    for a permission group region within a target's physical address space.
    Not all regions are programmable.

  Description:
    This function sets the permission bit(s) corresponding to the requested write
    permissions for a permission group region within a target's physical address
    space. Not all regions are programmable.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance.
    region    - The region number within the target's address space to be given
                permissions. Not all regions are programmable.
    writePerm - Bitwise OR of the groups given read permission for the region.

  Return:
    None.

  Example:
    See the code example for PLIB_SB_PGRegionAddrSet.

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Default reset value for all programmable regions is write permission for
    all groups. Some regions may not be programmable by all initiators (flash,
    for example). Please refer to the specific device data sheet for details.
*/

void PLIB_SB_PGRegionWritePermSet(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, PLIB_SB_REGION_PG writePerm);


/*******************************************************************************
  Function:
    void PLIB_SB_PGRegionWritePermClear( SB_MODULE_ID index, PLIB_SB_TGT_REGION region,
                                        PLIB_SB_REGION_PG writePerm )

  Summary:
    Clears the permission bit(s) corresponding to the requested write permissions
    for a permission group region within a target's physical address space.
    Not all regions are programmable.

  Description:
    This function clears the permission bit(s) corresponding to the requested
    write permissions for a permission group region within a target's physical
    address space. Not all regions are programmable.

  Precondition:
    None.

  Parameters:
    index     - Identifier for the device instance.
    region    - The region number within the target's address space to be given
                permissions. Not all regions are programmable.
    writePerm - Bitwise OR of the groups given read permission for the region.

  Return:
    None.

  Example:
    See the code example for PLIB_SB_PGRegionAddrSet.

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    Default reset value for all programmable regions is write permission for
    all groups. Some regions may not be programmable by all initiators (flash,
    for example). Please refer to the specific device data sheet for details.
*/

void PLIB_SB_PGRegionWritePermClear(SB_MODULE_ID index, PLIB_SB_TGT_REGION region, PLIB_SB_REGION_PG writePerm);


/*******************************************************************************
  Function:
    void PLIB_SB_InitPermGrpSet( SB_MODULE_ID index, PLIB_SB_PG_INITIATOR initiator, PLIB_SB_INIT_PG pg )

  Summary:
    Sets the read/write permission group(s) for an initiator. The region must
    also allow read/write access for the permission group for a read/write to
    occur.

  Description:
    This function sets the read/write permission group(s) for an initiator.
    The region must also allow read/write access for the permission group for a
    read/write to occur.

  Precondition:
    None.

  Parameters:
    initiator - The initiator for which permission groups are assigned.
    pg        - The permission group(s) to which the initiator is assigned.

  Return:
    None.

  Example:
    <code>
    PLIB_SB_InitPermGrpSet(SB_ID_1, PLIB_SB_PG_INITIATOR_CPU, PLIB_SB_INIT_PG_1);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    This function writes to the soft configuration register CFGPG, which is not
    part of the SB. Permission groups should be assigned by the boot code prior
    to programming the SB.

    Default permission group at reset for all initiators is 0.

    After an NMI, the CPU permission group reverts to 0. All other initiator
    permission groups remain unchanged.

    The effective CPU permission group value in debug mode is controlled by boot
    configuration value DBGPER[2:0]. If DBGPER denies access to the group CPU1PG
    selects, the effective value selects group 3.
*/

void PLIB_SB_InitPermGrpSet(SB_MODULE_ID index, PLIB_SB_PG_INITIATOR initiator, PLIB_SB_INIT_PG pg);


/*******************************************************************************
  Function:
    void PLIB_SB_CPUPrioritySet( SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority )

  Summary:
    Sets the CPU arbitration policy to SRAM when servicing an interrupt

  Description:
    This function sets the CPU arbitration policy to SRAM when servicing an
    interrupt.

  Precondition:
    A system unlock PLIB_DEVCON_SystemUnlock must be 
    performed before this function can be executed.

  Parameters:
    priority - Use either high priority or least-recently-serviced algorithm.

  Return:
    None.

  Example:
    <code>
	// Call system service to unlock oscillator
    PLIB_SB_CPUPrioritySet(SB_ID_1, PRIORITY_HI);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    This function writes to the soft configuration register CFGCON, which is
    not part of the SB. This should be done by the boot code prior to
    programming the SB. Note that the system must be unlocked before the
    priority can be set.

    Default at reset is PRIORITY_LRS.
*/

void PLIB_SB_CPUPrioritySet(SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority);


/*******************************************************************************
  Function:
    void PLIB_SB_DMAPrioritySet( SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority )

  Summary:
    Sets the DMA arbitration policy

  Description:
    This function sets the DMA arbitration.

  Precondition:
    A system unlock PLIB_DEVCON_SystemUnlock must be 
    performed before this function can be executed.
	
  Parameters:
    priority - Use either high priority or least-recently-serviced algorithm.

  Return:
    None.

  Example:
    <code>
	// Call system service to unlock oscillator
    void PLIB_SB_DMAPrioritySet(SB_ID_1, PRIORITY_HI);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    This function writes to the soft configuration register CFGCON, which is
    not part of the SB. This should be done by the boot code prior to
    programming the SB. Note that the system must be unlocked before the
    priority can be set.

    Default at reset is PRIORITY_LRS.
*/

void PLIB_SB_DMAPrioritySet(SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority);

/*******************************************************************************
  Function:
    void PLIB_SB_ADCPrioritySet( SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority )

  Summary:
    Sets the ADC arbitration policy

  Description:
    This function sets the ADC arbitration.

  Precondition:
    A system unlock PLIB_DEVCON_SystemUnlock must be 
    performed before this function can be executed.
  Parameters:
    priority - Use either high priority or least-recently-serviced algorithm.

  Return:
    None.

  Example:
    <code>
	// Call system service to unlock oscillator
    PLIB_SB_ADCPrioritySet(SB_ID_1, PRIORITY_HI);
    </code>

  Remarks:
    This feature is not available on all devices. Please refer to the
    specific device data sheet for availability.

    This function writes to the soft configuration register CFGCON, which is
    not part of the SB. This should be done by the boot code prior to
    programming the SB. Note that the system must be unlocked before the
    priority can be set.

    Default at reset is PRIORITY_LRS.
*/

void PLIB_SB_ADCPrioritySet(SB_MODULE_ID index, PLIB_SB_ARB_POLICY priority);


// *****************************************************************************
// *****************************************************************************
// Section: System Bus Peripheral Library Exists Functions
// *****************************************************************************
// *****************************************************************************
/* The following functions indicate the existence of the features on the device. 
*/

//******************************************************************************
/* Function :  PLIB_SB_ExistsPGVErrGroupStatus( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrGroupStatus feature exists on the SB module 

  Description:
    This function identifies whether the PGVErrGroupStatus feature is available on the SB module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGVErrGroupStatus

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrGroupStatus feature is supported on the device
    - false  - The PGVErrGroupStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrGroupStatus( SB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_SB_ExistsPGVErrGroup0Status( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrGroup0Status feature exists on the SB module 

  Description:
    This function identifies whether the PGVErrGroup0Status feature is available on the SB module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGVErrGroup0Status

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrGroup0Status feature is supported on the device
    - false  - The PGVErrGroup0Status feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrGroup0Status( SB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_SB_ExistsPGVErrGroup1Status( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrGroup1Status feature exists on the SB module 

  Description:
    This function identifies whether the PGVErrGroup1Status feature is available on the SB module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGVErrGroup1Status

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrGroup1Status feature is supported on the device
    - false  - The PGVErrGroup1Status feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrGroup1Status( SB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_SB_ExistsPGVErrGroup2Status( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrGroup2Status feature exists on the SB module 

  Description:
    This function identifies whether the PGVErrGroup2Status feature is available on the SB module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGVErrGroup2Status

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrGroup2Status feature is supported on the device
    - false  - The PGVErrGroup2Status feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrGroup2Status( SB_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_SB_ExistsPGVErrGroup3Status( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrGroup3Status feature exists on the SB module 

  Description:
    This function identifies whether the PGVErrGroup3Status feature is available on the SB module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGVErrGroup3Status

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrGroup3Status feature is supported on the device
    - false  - The PGVErrGroup3Status feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrGroup3Status( SB_MODULE_ID index );

//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrStatus( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrStatus feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrStatus feature is available on the 
	System Bus module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGVErrorStatus

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrStatus feature is supported on the device
    - false  - The PGVErrStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrStatus( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrClear( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrClear feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrClear feature is available on the 
	System Bus module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGVErrorMulti
    - PLIB_SB_PGVErrorCode
    - PLIB_SB_PGVErrorLogClearSingle
    - PLIB_SB_PGVErrorLogClearMulti

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrClear feature is supported on the device
    - false  - The PGVErrClear feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrClear( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrInitID( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrInitID feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrInitID feature is available on the 
	System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_PGVErrorInitiatorID

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrInitID feature is supported on the device
    - false  - The PGVErrInitID feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrInitID( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrRegion( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrRegion feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrRegion feature is available on the 
	System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_PGVErrorRegion

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrRegion feature is supported on the device
    - false  - The PGVErrRegion feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrRegion( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrCmdCode( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrCmdCode feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrCmdCode feature is available on 
	the System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_PGVErrorCommandCode

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrCmdCode feature is supported on the device
    - false  - The PGVErrCmdCode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrCmdCode( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrPG( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrPG feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrPG feature is available on the 
	System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_PGVErrorPermissionGroup

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrPG feature is supported on the device
    - false  - The PGVErrPG feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrPG( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrRptPri( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrRptPri feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrRptPri feature is available on the 
	System Bus module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGVErrorReportPrimaryEnable
    - PLIB_SB_PGVErrorReportPrimaryDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrRptPri feature is supported on the device
    - false  - The PGVErrRptPri feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrRptPri( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrClrSingle( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrClrSingle feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrClrSingle feature is available on 
	the System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_PGVErrorClearSingle

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrClrSingle feature is supported on the device
    - false  - The PGVErrClrSingle feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrClrSingle( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGVErrClrMulti( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGVErrClrMulti feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGVErrClrMulti feature is available on the 
	System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_PGVErrorClearMulti

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGVErrClrMulti feature is supported on the device
    - false  - The PGVErrClrMulti feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGVErrClrMulti( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGRegAddr( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGRegAddr feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGRegAddr feature is available on the 
	System Bus module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGRegionAddrSet
    - PLIB_SB_PGRegionAddrGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGRegAddr feature is supported on the device
    - false  - The PGRegAddr feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGRegAddr( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGRegSize( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGRegSize feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGRegSize feature is available on the 
	System Bus module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGRegionSizeSet
    - PLIB_SB_PGRegionSizeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGRegSize feature is supported on the device
    - false  - The PGRegSize feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGRegSize( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGRegRdPerm( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGRegRdPerm feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGRegRdPerm feature is available on the 
	System Bus module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGRegionReadPermSet
    - PLIB_SB_PGRegionReadPermClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGRegRdPerm feature is supported on the device
    - false  - The PGRegRdPerm feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGRegRdPerm( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsPGRegWrPerm( SB_MODULE_ID index )

  Summary:
    Identifies whether the PGRegWrPerm feature exists on the System Bus module. 

  Description:
    This function identifies whether the PGRegWrPerm feature is available on the 
	System Bus module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_SB_PGRegionWritePermSet
    - PLIB_SB_PGRegionWritePermClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PGRegWrPerm feature is supported on the device
    - false  - The PGRegWrPerm feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsPGRegWrPerm( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsInitPermGrp( SB_MODULE_ID index )

  Summary:
    Identifies whether the InitPermGrp feature exists on the System Bus module. 

  Description:
    This function identifies whether the InitPermGrp feature is available on the 
	System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_InitPermGrpSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The InitPermGrp feature is supported on the device
    - false  - The InitPermGrp feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsInitPermGrp( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsCPUPriority( SB_MODULE_ID index )

  Summary:
    Identifies whether the CPUPriority feature exists on the System Bus module. 

  Description:
    This function identifies whether the CPUPriority feature is available on 
	the System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_CPUPrioritySet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CPUPriority feature is supported on the device
    - false  - The CPUPriority feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsCPUPriority( SB_MODULE_ID index );


//******************************************************************************
/* Function:
    PLIB_SB_ExistsDMAPriority( SB_MODULE_ID index )

  Summary:
    Identifies whether the DMAPriority feature exists on the System Bus module. 

  Description:
    This function identifies whether the DMAPriority feature is available on the 
	System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_DMAPrioritySet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DMAPriority feature is supported on the device
    - false  - The DMAPriority feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsDMAPriority( SB_MODULE_ID index );

//******************************************************************************
/* Function:
    PLIB_SB_ExistsADCPriority( SB_MODULE_ID index )

  Summary:
    Identifies whether the ADCPriority feature exists on the System Bus module. 

  Description:
    This function identifies whether the ADCPriority feature is available on the 
	System Bus module.
    When this function returns true, this function is supported on the device: 
    - PLIB_SB_ADCPrioritySet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ADCPriority feature is supported on the device
    - false  - The ADCPriority feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_SB_ExistsADCPriority( SB_MODULE_ID index );


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // #ifndef _PLIB_SB_H
/*******************************************************************************
 End of File
*/
