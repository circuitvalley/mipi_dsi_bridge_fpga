/*******************************************************************************
  Microchip File System (MPFS) Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    sys_fs_mpfs.h

  Summary:
    API for handling Microchip File System (MPFS)

  Description:
    This file contains the interface definition for handling Microchip File
    System (MPFS). It provides a way to interact with the Clock subsystem to
    manage the timing requests supported by the system.
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

#ifndef _SYS_FS_MPFS_H
#define _SYS_FS_MPFS_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system/fs/sys_fs.h"

#include <stdint.h>

extern const SYS_FS_FUNCTIONS MPFSFunctions;

typedef uint32_t MPFS_HANDLE;	// MPFS Handles are currently stored as BYTEs
// *****************************************************************************
// *****************************************************************************
// Section: Enumerations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  MPFS Results

  Summary:
	Lists all the possible return values for SYS_FS_MPFS_Open function.

  Description:
	This enumeration lists all the possible return values for SYS_FS_MPFS_Open
	function.

  Remarks:
*/

typedef enum
{
    /* Succeeded */
    MPFS_OK = 0,
    /* (1) A hard error occurred in the low level disk I/O layer */
    MPFS_DISK_ERR,
    /* (2) Assertion failed */
    MPFS_INT_ERR,
    /* (3) The physical drive cannot work */
    MPFS_NOT_READY,
    /* (4) Could not find the file */
    MPFS_NO_FILE,
    /* (5) Could not find the path */
    MPFS_NO_PATH,
    /* (6) The path name format is invalid */
    MPFS_INVALID_NAME,
    /* (7) Access denied due to prohibited access or directory full */
    MPFS_DENIED,
    /* (8) Access denied due to prohibited access */
    MPFS_EXIST,
    /* (9) The file/directory object is invalid */
    MPFS_INVALID_OBJECT,
    /* (10) The physical drive is write protected */
    MPFS_WRITE_PROTECTED,
    /* (11) The logical drive number is invalid */
    MPFS_INVALID_DRIVE,
    /* (12) The volume has no work area */
    MPFS_NOT_ENABLED,
    /* (13) There is no valid FAT volume */
    MPFS_NO_FILESYSTEM,
    /* (14) The f_mkfs() aborted due to any parameter error */
    MPFS_MKFS_ABORTED,
    /* (15) Could not get a grant to access the volume within defined period */
    MPFS_TIMEOUT,
    /* (16) The operation is rejected according to the file sharing policy */
    MPFS_LOCKED,
    /* (17) LFN working buffer could not be allocated */
    MPFS_NOT_ENOUGH_CORE,
    /* (18) Number of open files > _FS_SHARE */
    MPFS_TOO_MANY_OPEN_FILES,
    /* (19) Given parameter is invalid */
    MPFS_INVALID_PARAMETER
}MPFS_RESULT;


// *****************************************************************************
// *****************************************************************************
// Section: SYS MPFS Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    int MPFS_Mount ( uint8_t diskNo )

  Summary:
    Initializes hardware and internal data structure of the System Clock.

  Description:
    This function initializes the hardware and internal data structure of the System
    Clock.

  Precondition:
    None.

  Parameters:
	diskNo - disk index in the Filesystem framework

  Returns:
    MPFS_RESULT - One of the possible value from MPFS_RESULT stating the status.

  Example:
    <code>

    </code>

  Remarks:
    This could be called at the time of system initialization to initialize the
    oscillator or after initialization to change any of the initialization
    settings.
*/

int MPFS_Mount ( uint8_t diskNo );


// *****************************************************************************
/* Function:
    MPFS_RESULT MPFS_Open ( SYS_FS_HANDLE handle, const uint8_t* filewithDisk, uint8_t mode )

  Summary:
    Initializes hardware and internal data structure of the System Clock.

  Description:
    This function initializes the hardware and internal data structure of the System
    Clock.

  Precondition:
    None.

  Parameters:
	cFile - A null terminated file name to open.
	path  - Path of the specified file, Not applicable in case of MPFS
	mode  - Mode in which the file should be opened.
  Returns:
    SYS_FS_HANDLE.

  Example:
    <code>

    </code>

  Remarks:
    This could be called at the time of system initialization to initialize the
    oscillator or after initialization to change any of the initialization
    settings.
*/

int MPFS_Open ( uintptr_t handle, const char* filewithDisk, uint8_t mode );

/*****************************************************************************
  Function:
    MPFS_RESULT MPFS_Unmount ( uint8_t diskNo );

  Description:
    Unmounts a file MPFS file system.

  Precondition:
    None

  Parameters:
    diskNo - disk index in the Filesystem framework
  Returns:
    None.
*/
int MPFS_Unmount ( uint8_t diskNo );

/*****************************************************************************
  Function:
    int MPFS_Read ( uintptr_t handle, void* buff, uint16_t btr, uint16_t * br )

  Description:
    Reads a file in the MPFS2 file system.

  Precondition:
    None

  Parameters:
    handle  - a valie handle to the file
    buff    - pointer to buffer to read data
    btr     - number of bytes to read
    br      - pointer to variable which holds the number of bytes actually read

  Returns:
    MPFS_OK
*/
int MPFS_Read ( uintptr_t handle, void* buff, uint32_t btr, uint32_t* br );

/*****************************************************************************
  Function:
    int MPFS_Close ( uintptr_t handle )

  Description:
    Closes a file in the MPFS2 file system.

  Precondition:
    None

  Parameters:
    handle - a valie handle to the file

  Returns:
    MPFS_OK;
*/
int MPFS_Close ( uintptr_t handle );
/*****************************************************************************
  Function:
    uint32_t MPFS_GetSize ( uintptr_t handle )

  Description:
    Obtains the size of the file

  Precondition:
    None

  Parameters:
    handle - a valie handle to the file

  Returns:
    The size of file.
*/
uint32_t MPFS_GetSize ( uintptr_t handle );
/*****************************************************************************
  Function:
    uint32_t MPFS_GetPosition ( uintptr_t handle)

  Description:
    Obtains the present file pointer position

  Precondition:
    None

  Parameters:
    handle - a valie handle to the file

  Returns:
    The present file pointer.
*/
uint32_t MPFS_GetPosition ( uintptr_t handle);
/*****************************************************************************
  Function:
    bool MPFS_EOF( uintptr_t handle )

  Description:
    Returns if the present file pointer already reached end of file?

  Precondition:
    None

  Parameters:
    handle - a valie handle to the file

  Returns:
    End of file     - true
    Not end of file - false
*/
bool MPFS_EOF ( uintptr_t handle);
/*****************************************************************************
  Function:
	int MPFS_Seek ( uintptr_t handle, uint32_t dwOffset )

  Description:
	Moves the current read pointer to a new location.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle to seek with
	dwOffset - offset from the specified position

  Returns:
    Success     - MPFS_OK
    Failure     - MPFS_DISK_ERR
*/

int MPFS_Seek ( uintptr_t handle, uint32_t dwOffset );

/*****************************************************************************
  Function:
    int MPFS_Stat ( const char* filewithDisk, uintptr_t stat_str )

  Description:
    Returns the status (property) of the file

  Precondition:
    None

  Parameters:
    filewithDisk    -   string containing the Disk number appended to file name
    stat_str        -   pointer to structure which will return the file status

  Returns:
    Success     - MPFS_OK
    Failure      - MPFS_NO_FILE
*/
int MPFS_Stat ( const char* filewithDisk, uintptr_t stat_str );


/*****************************************************************************
  Function:
    int MPFS_DirOpen
    (
	    uintptr_t handle,
        const char *path
    );

  Summary
    Open a directory

  Description:
    This function opens the directory at the root level for the specified disk.

  Precondition:
    The volume on which the directory is present should be mounted.

  Parameters:
    handle   - Handle to the directory.
    path     - Path to the directory along with the volume name. The string of
               volume and directory name has to be preceded by "/mnt/". Also,
               the volume name and directory name has to be separated by a
               slash "/". This function can only operate at the root directory
               level. FOr the root directory, the path has to be ended with
               "/".

  Returns:
    Success     - MPFS_OK
    Failure     - MPFS_INVALID_PARAMETER if the parameters are invalid.
                - MPFS_DENIED if the directory is already open.
*/

int MPFS_DirOpen
(
	uintptr_t handle,
	const char *path
);

/*****************************************************************************
  Function:
    int MPFS_DirClose
    (
	    uintptr_t handle
    );

  Summary
    Closes the opened directory.

  Description:
    This function closes the opened directory.

  Precondition:
    The MPFS_OpenDir should have been called and a valid directory handle be
    obtained.

  Parameters:
    handle   - Handle to the directory.

  Returns:
    Success     - MPFS_OK
    Failure     - MPFS_INVALID_PARAMETER if the parameters are invalid.
*/

int MPFS_DirClose
(
	uintptr_t handle
);

/*****************************************************************************
  Function:
    int MPFS_DirRead
    (
	    uintptr_t handle,
	    uintptr_t statPtr
    );

  Summary
    Returns the file information of the directory entry.

  Description:
    This function returns the information about files present in the directory
    one file at a time. The function internally uses a file pointer for the
    directory to point to the file to be read next. This file pointer is
    incremented every read operation. This function sets the fname and the
    lfname(if LFN is used) fields of the statPtr structure to '\0' when there
    are no more files left to be read in the directory.

    The directory's next file pointer can be reset by passing NULL to statPtr.

    If the length of the file name is longer than the SFN(8.3) format then only
    the SFN length is copied into the file name member of the statPtr
    structure. If the Long File Name has been enabled and the size of the file
    name does not exceed the long file name buffer then the long file name
    buffer will be populated with the file name as well.

  Precondition:
    The MPFS_OpenDir should have been called and a valid directory handle be
    obtained.

  Parameters:
    handle   - Handle to the directory.
    statPtr  - Pointer to structure which will return the file information
               including the file name.

  Returns:
    Success     - MPFS_OK
                  The fname and lfname(if LFN is used) fields of the statPtr
                  structure will be set to '\0' to indicate the end of the
                  directory condition.
    Failure     - MPFS_INVALID_PARAMETER if the parameters are invalid.
                - MPFS_DISK_ERR if there was an error while reading from the
                  underlying media disk
*/

int MPFS_DirRead
(
	uintptr_t handle,
	uintptr_t statPtr
);

#endif //SYS_FS_MPFS_H

/*******************************************************************************
 End of File
*/

