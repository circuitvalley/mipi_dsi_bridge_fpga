/*******************************************************************************
  MPFS Local Data Structures

Company:
Microchip Technology Inc.

File Name:
sys_mpfs_local.h

Summary:
MPFS local declarations and definitions.

Description:
This file contains the MPFS local declarations and definitions.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
  Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _SYS_MPFS_LOCAL_H
#define _SYS_MPFS_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include "driver/driver_common.h"
#include "system/fs/mpfs/mpfs.h"
#include "system/fs/sys_fs_media_manager.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define MPFS_HANDLE_TOKEN_MAX (0xFF)
#define MPFS_MAKE_HANDLE(token, disk, index) (((token) << 24) | ((disk) << 16) | (index))
#define MPFS_UPDATE_HANDLE_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == MPFS_HANDLE_TOKEN_MAX) ? 0: (token); \
}

/****************************************************************************
Section:
Type Definitions
 ***************************************************************************/
#define MPFS2_FLAG_ISZIPPED		((uint16_t)0x0001)	// Indicates a file is compressed with GZIP compression
#define MPFS2_FLAG_HASINDEX		((uint16_t)0x0002)	// Indicates a file has an associated index of dynamic variables
#define MPFS_INVALID			(0xffffffffu)	// Indicates a position pointer is invalid
#define MPFS_INVALID_FAT		(0xffffu)		// Indicates an invalid FAT cache
#define MPFS_INVALID_HANDLE 	(0xff)			// Indicates that a handle is not valid
typedef uint32_t MPFS_PTR;							// MPFS Pointers are currently DWORDs


// Stores each file handle's information
// Handles are free when currentOffset = MPFS_INVALID
typedef struct
{
    MPFS_PTR currentOffset;  // Current address in the file system
    /* Hash index of the file. */
    uint32_t hashIndex;
    uint32_t bytesRemaining; // How many bytes remain in this file
    MPFS_HANDLE handle;	     // ID of which file in the FAT was accessed
} MPFS_FILE_OBJ;

// Indicates the method for MPFSSeek
typedef enum
{
    MPFS_SEEK_START		= 0u,	// Seek forwards from the front of the file
    MPFS_SEEK_END,				// Seek backwards from the end of the file
    MPFS_SEEK_FORWARD,			// Seek forward from the current position
    MPFS_SEEK_REWIND			// See backwards from the current position
} MPFS_SEEK_MODE;

// Stores the data for an MPFS2 FAT record
typedef struct
{
    uint32_t fileNameOffset;// Pointer to the file name
    uint32_t dataOffset;// Address of the file data
    uint32_t length;// Length of file data
    uint32_t timestamp;// Timestamp of file
    uint32_t microtime;// Microtime stamp of file
    uint16_t flags;// Flags for this file
} MPFS_FILE_RECORD;

/* File status structure (FILINFO) */
typedef struct {
    unsigned long	fsize;			/* File size */
    unsigned short	fdate;			/* Last modified date */
    unsigned short	ftime;			/* Last modified time */
    unsigned char	fattrib;		/* Attribute */
    /* Short file name (8.3 format) */
    char        fname[13];
#if FAT_FS_USE_LFN
    /* Pointer to the LFN buffer */
    char       *lfname;
    /* Size of LFN buffer in TCHAR */
    uint32_t    lfsize;
#endif
} MPFS_STATUS;

// Alias of MPFSGetPosition
#define MPFSTell(a)	MPFSGetPosition(a)


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* MPFS local data

Summary:
Defines the variables required for internal operation of MPFS.

Description:
This structure defines the variables required for internal operation of MPFS.

Remarks:
None.
*/

typedef struct _SYS_MPFS_OBJ_STRUCT
{
    /* Disk number associated with the media containing the MPFS2 image. */
    uint8_t diskNum;

    /* Number of files in this MPFS2 image */
    uint16_t numFiles;

    /* Base address of the media where the MPFS2 image is located. */
    uint32_t baseAddress;

    /* Current file index. */
    uint32_t fileIndex;

    /* Handle of the current file record. */
    MPFS_HANDLE currentHandle;

    /* Handle to the root directory. */
    MPFS_HANDLE dirHandle;

} SYS_MPFS_OBJECT;

// *****************************************************************************
// *****************************************************************************
// Section: External variables
// *****************************************************************************
// *****************************************************************************


#endif //#ifndef _SYS_MPFS_LOCAL_H

/*******************************************************************************
  End of File
*/

