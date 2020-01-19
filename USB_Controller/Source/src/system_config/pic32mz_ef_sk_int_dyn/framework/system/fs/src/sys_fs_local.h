/*******************************************************************************
  File System System-Library Local Types and Defintions

  Company:
    Microchip Technology Inc.

  File Name:
    sys_fs_local.h

  Summary:
    Contains local types and defintions required by the SYS_FS functions.

  Description:
    This file contains local types and defintions required by the SYS_FS 
    functions. These types are internal to the SYS_FS function implementation
    and should not be used directly by the application. 
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is fsegrated fso your product or third party product
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

#ifndef _SYS_FS_PRIVATE_H
#define _SYS_FS_PRIVATE_H

#include "system_config.h"
#include "system/system.h"
#include "system/fs/sys_fs.h"
#include "system/fs/fat_fs/src/file_system/ff.h"
#include "system/fs/sys_fs_media_manager.h"
#include "osal/osal.h"

#define SYS_FS_PATH_LEN_WITH_DISK_NUM (FAT_FS_MAX_LFN + 3)

// *****************************************************************************
/* Mount point

  Summary:
    Defines the mount point for each volume.

  Description:
    This structure defines the mount point that is utilized when a volume is
 * mounted by application code. This mount point is one per volume mounted.

  Remarks:
    None.
*/
typedef struct
{
    /* If the mount point is in use or is it free */
    bool inUse;
    /* Type of native file system implemented on the volume mounted */
    SYS_FS_FILE_SYSTEM_TYPE fsType;
    /* List of function pointers associated with the native file system */
    const SYS_FS_FUNCTIONS * fsFunctions;
    /* Mount name. The format for mount name should follow: "/mnt/myDrive1" */
    const char *mountName;
    /* Volume number */
    uint8_t diskNumber;
   /* Mount/Volume instance mutex */
    OSAL_MUTEX_DECLARE(mutexDiskVolume);
}
SYS_FS_MOUNT_POINT;

// *****************************************************************************
/* File Object

  Summary:
    Defines the file object for each file opened.

  Description:
    This structure defines the file object which is allocated when a file is
 * opened by application code.

  Remarks:
    None.
*/
typedef struct
{
    /* If the file object is in use or is it free */
    bool                inUse;
    /* pointer to the mount point associated with the volume from where file
       is being opened */
    SYS_FS_MOUNT_POINT  *mountPoint;
    /* File object as obtained from the native file system */
    uintptr_t            nativeFSFileObj;
    
    /* File specific error value */
    SYS_FS_ERROR errorValue;
    
    /* Name of file is stored in a buffer for future use */
    uint8_t fileName[FAT_FS_MAX_LFN];

}SYS_FS_OBJ;

// *****************************************************************************
/* Directory Object

  Summary:
    Defines the directory object for each directory opened.

  Description:
    This structure defines the directory object which is allocated when a directory is
    opened by application code.

  Remarks:
    None.
*/
typedef struct
{
    /* If the file object is in use or is it free */
    bool                inUse;
    /* pointer to the mount point associated with the volume from where file
       is being opened */
    SYS_FS_MOUNT_POINT  *mountPoint;
    /* Directory object as obtained from the native file system */
    uintptr_t            nativeFSDirObj;
    /* Directory specific error value */
    SYS_FS_ERROR errorValue;

}SYS_FS_DIR_OBJ;

// *****************************************************************************
/* Current Mount point

  Summary:
    Defines the mount point currently being used.

  Description:
  This structure defines the mount which is used currently. If the application
  do not specify the full path name (along with the drive name), then this mount
  point will be taken as the default value.

  Remarks:
    None.
*/
typedef struct
{
    /* If the current mount point is valid or not */
    bool                inUse;
    /* Pointer to the current disk object */
    SYS_FS_MOUNT_POINT *currentDisk;
}
SYS_FS_CURRENT_MOUNT_POINT;

//******************************************************************************
/*Function:
    SYS_FS_RESULT _SYS_FS_DiskGet(const char *path, SYS_FS_MOUNT_POINT **disk);

    Summary:
        Retrives the disk from the name of the disk.

    Description:
        Retrives the disk from the name of the disk.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.


    Parameters:
        path            - Path for the file/ directory.

        disk     	- pointer of type SYS_FS_MOUNT_POINT.


    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT _SYS_FS_DiskGet(const char *path, SYS_FS_MOUNT_POINT **disk);

//******************************************************************************
/*Function:
    bool _SYS_FS_DiskNumberAppend(const char *path, uint8_t diskNumber, uint8_t *buffer)

    Summary:
        Appends the disk number to the begining of the file name.

    Description:
        Appends the disk number to the begining of the file name. This is required for
        the native FAT FS working.
        The function assumes that the max buffer be of size "SYS_FS_PATH_LEN_WITH_DISK_NUM".

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.


    Parameters:
        path        - Path for the file/ directory.

        disk     	- pointer of type SYS_FS_MOUNT_POINT.


    Returns:
        True - If the append operation was successful.
        False - If the append oepration was unsuccessful.

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
    None
***************************************************************************/
bool _SYS_FS_DiskNumberAppend(const char *path, uint8_t diskNumber, uint8_t *buffer);

//******************************************************************************
/*Function:
    bool _SYS_FS_StringWildCardCompare(const char * ptr1, const char *ptr2)

    Summary:
    Compares the 2 file names with wild character.

    Description:
        Compares the 2 file names with wild character. This is required for
        file search of a directory.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.


    Parameters:
        ptr1            - First file name. This can have wild characters (*, ?).

        ptr2     	- Second file name. This should be complete file name.
                          No wild char should be passed here.


    Returns:
        If comparison is success	-	true

        If comparison is failure	-	false

  Remarks:
	None
***************************************************************************/
bool _SYS_FS_StringWildCardCompare(const char * ptr1, const char *ptr2);


#endif // _SYS_FS_PRIVATE_H

