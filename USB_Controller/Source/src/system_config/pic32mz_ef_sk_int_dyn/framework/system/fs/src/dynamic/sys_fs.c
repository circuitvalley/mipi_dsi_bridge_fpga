/*******************************************************************************
  File System System-Library Interface Implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    sys_fs.c

  Summary:
    This file contains implementation of SYS_FS functions.

  Description:
    This file contains implementation of SYS_FS functions.
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


#include "system/fs/src/sys_fs_local.h"
#include "system/fs/sys_fs_media_manager.h"
#include <stdarg.h>
#include <string.h>

// *****************************************************************************
/* Registration table for each native file system

  Summary:
    Defines the registration table for each native file system.

  Description:
    Defines the registration table for each native file system.

  Remarks:
    None
*/
SYS_FS_REGISTRATION_TABLE gSYSFSObj[SYS_FS_MAX_FILE_SYSTEM_TYPE];

// *****************************************************************************
/* Mount point

  Summary:
    Defines the mount point required for mounting each volume.

  Description:
    Defines the mount point required for mounting each volume.

  Remarks:
    None
*/
SYS_FS_MOUNT_POINT gSYSFSMountPoint[SYS_FS_VOLUME_NUMBER];

// *****************************************************************************
/* Current Mount point

  Summary:
    Defines the mount point currently being used.

  Description:
    Defines the mount point which will be used as default, in case the application
    do not specify the full path name along with the drive name.

  Remarks:
    None
*/
SYS_FS_CURRENT_MOUNT_POINT  gSYSFSCurrentMountPoint;

// *****************************************************************************
/* File object

  Summary:
    Defines the File object for each file opened.

  Description:
    Defines the File object for each file opened on the system.

  Remarks:
    None
*/
SYS_FS_OBJ gSYSFSFileObj[SYS_FS_MAX_FILES];

// *****************************************************************************
/* Directory object

  Summary:
    Defines the Directory object for each directory opened.

  Description:
    Defines the Directorye object for each directory opened on the system.

  Remarks:
    None
*/
SYS_FS_DIR_OBJ  gSYSFSDirObj[SYS_FS_MAX_FILES];
// *****************************************************************************
/* File System Event Handler

  Summary:
    Defines the File system Event handler that stores the application event handler.

  Description:
    This data type defines the file system event handler
    the part.
  Remarks:
    None
*/
SYS_FS_EVENT_HANDLER gSYSFSEventHandler = NULL;

/* File system object instance mutex */
OSAL_MUTEX_DECLARE(mutexFileSystemObj);
/* File system object instance mutex */
OSAL_MUTEX_DECLARE(mutexDirObj);
/* Variable to hold the error value */
SYS_FS_ERROR errorValue;

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_Initialize(const void* initData)

    Summary:
        Initializes the Abstration Layer

    Description:
        Initializes the abstraction Layer and sets up the necessary parameters

    Precondition:
        This is the first function to be called during initialization of SYS FS.
		Calling other functions from SYS FS without initializing the SYS FS will
		cause un-predictable behavior.

    Parameters:
        initData	-	The Set of initialization parameters

    Returns:
        If Success: SYS_FS_RES_SUCCESS
        If Failure: SYS_FS_RES_FAILURE
            Sets error code which can be retrieved with SYS_FS_Error

***************************************************************************/
SYS_FS_RESULT SYS_FS_Initialize(const void* initData)
{
    SYS_FS_REGISTRATION_TABLE *inData = ( SYS_FS_REGISTRATION_TABLE *)initData;
    uint8_t index;
    
    for( index = 0; index != SYS_FS_MAX_FILES; index++ )
    {
        gSYSFSFileObj[index].inUse = false;
        gSYSFSFileObj[index].mountPoint = NULL;
        gSYSFSFileObj[index].nativeFSFileObj = (uintptr_t)NULL;
        memset(gSYSFSFileObj[index].fileName, 0, FAT_FS_MAX_LFN);
        gSYSFSFileObj[index].errorValue = SYS_FS_ERROR_OK;

        gSYSFSDirObj[index].inUse = false;
        gSYSFSDirObj[index].mountPoint = NULL;
        gSYSFSDirObj[index].nativeFSDirObj = (uintptr_t)NULL;
        gSYSFSDirObj[index].errorValue = SYS_FS_ERROR_OK;
    }
    
    for( index = 0; index != SYS_FS_MAX_FILE_SYSTEM_TYPE; index++ )
    {
        gSYSFSObj[index].nativeFileSystemType = inData->nativeFileSystemType;
        gSYSFSObj[index].nativeFileSystemFunctions = inData->nativeFileSystemFunctions;
        inData++;
    }
   /* Create mutex forFile system object */
    if(OSAL_MUTEX_Create(&mutexFileSystemObj) != OSAL_RESULT_TRUE)
    {
        return SYS_FS_RES_FAILURE;
    }
     /* Create mutex forFile system object */
    if(OSAL_MUTEX_Create(&mutexDirObj) != OSAL_RESULT_TRUE)
    {
        return SYS_FS_RES_FAILURE;
    }

    return SYS_FS_RES_SUCCESS;
}

//******************************************************************************
/*Function:
    void SYS_FS_EventHandlerSet(const void* eventHandler, const uintptr_t context)

    Summary:
        Sets the Mount/Unmount Event Handler 

    Description:
        Interface to set the event handler, event handler would get invoked on
        Media Mount or Unmount Event.
    Precondition:
        This interface is used to set the event handler when the Auto Mount 
        feature of File System is Enable.

    Parameters:
        eventHandler	-	Event Handler for Mount/Unmount Events
        context         -   Pointer to the context

    Returns:
        None

***************************************************************************/
void SYS_FS_EventHandlerSet
(
    const void * eventHandler,
    const uintptr_t context
)
{
    /* Set the event handler received from application when it is not NULL*/
    /* Context parameter is not used for now */
    if (eventHandler != NULL)
    {
        /* Set the event handler */
        gSYSFSEventHandler = eventHandler;
    }
}
//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_Mount(const char *devName, const char *mountName,
	SYS_FS_FILE_SYSTEM_TYPE filesystemtype, unsigned long mountflags, const void *data);

    Summary:
        Mount filesystems

    Description:
        Attaches the filesystem specified by source to a specified media.

    Precondition:
        The "devName" for the media has to be known. The convention that is
		followed in Harmony file system is: -

		For NVM			- "nvm"<media number><volume number>
		For SD card		- "mmcblk"<media number><volume number>
		For MSD			- "sd"<media number><volume number>

		<media number> 	- a, b, c... depending upon number of media of certain type
						  connected.
	    <volume number>	- 1, 2, 3... depending upon number of partitions in that
						  media.

    Parameters:
        devName 		- The device name (name of media) which needs to be mounted
        mountName 		- Mount name for the device to be mounted
        filesystemtype 	- native file system type
        mountflags 		- mounting control flags. This parameter is kept for future
						  expansion. Hence, always pass zero.
        data 			- The data argument is interpreted by the different file systems.
					      This parameter is kept for future expansion. Hence, always
						  pass NULL.

    Returns:
        If Success: SYS_FS_RES_SUCCESS
        If Failure: SYS_FS_RES_FAILURE
            Sets error code which can be retrieved with SYS_FS_Error
***************************************************************************/
SYS_FS_RESULT SYS_FS_Mount
(
    const char *devName,
    const char *mountName,
    SYS_FS_FILE_SYSTEM_TYPE filesystemtype,
    unsigned long mountflags,
    const void *data
 )
{
    int fileStatus = -1;
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *)NULL;
    uint8_t mountPoint = 0;
    SYS_FS_VOLUME_PROPERTY volProperty  = {};
    uint8_t index = 0;

    /* First check if the media is attached or not */
    if(SYS_FS_MEDIA_MANAGER_MediaStatusGet(devName) != true)
            
    {
        errorValue = SYS_FS_ERROR_NOT_READY;
        /* The media name specified is not attached */
        return SYS_FS_RES_FAILURE;
    }
    /* Clear the error value when mount is sucessful */
    errorValue = SYS_FS_ERROR_OK;
        /* Verify if the requested file system is supported by SYS_FS */
    for( index = 0; index != SYS_FS_MAX_FILE_SYSTEM_TYPE; index++ )
    {
        if(gSYSFSObj[index].nativeFileSystemType == filesystemtype)
            break;  /* Now, index, holds the element number for the requested file system */
    }

    /* If the requested file system type is not supported by SYS_FS */
    if(index >= SYS_FS_MAX_FILE_SYSTEM_TYPE)
    {
        errorValue = SYS_FS_ERROR_FS_NOT_SUPPORTED;
        return SYS_FS_RES_FAILURE;
    }
    // Start with 0th disk and find a disk which is available (not in use)
    for(mountPoint = 0; mountPoint != SYS_FS_VOLUME_NUMBER; mountPoint++)
    {
        if(!gSYSFSMountPoint[mountPoint].inUse) // not in use, hence this is available
        {
            if(true == SYS_FS_MEDIA_MANAGER_VolumePropertyGet(devName, &volProperty))
            {
                if ((volProperty.fsType != filesystemtype) && (volProperty.fsType != UNSUPPORTED_FS))
                {
                     errorValue = SYS_FS_ERROR_FS_NOT_MATCH_WITH_VOLUME;
                     return SYS_FS_RES_FAILURE;
                }
            }
            else
            {
                 errorValue = SYS_FS_ERROR_DISK_ERR;
                 return SYS_FS_RES_FAILURE;
            }
            disk = &gSYSFSMountPoint[mountPoint];

            disk->inUse = true;
            disk->fsType = filesystemtype;
            disk->fsFunctions = gSYSFSObj[index].nativeFileSystemFunctions;
            disk->mountName = (mountName + 5);  // Save only mountName. Do not save "/mnt/"
            disk->diskNumber = volProperty.volNumber;
          /* Create mutex for this Mount/volume */
            if(OSAL_MUTEX_Create(&(disk->mutexDiskVolume)) != OSAL_RESULT_TRUE)
            {
                return SYS_FS_RES_FAILURE;
            }
            break;
        }
    }

    if(mountPoint >= SYS_FS_VOLUME_NUMBER)
    {
        SYS_ASSERT(false, "Invalid Disk");
        errorValue = SYS_FS_ERROR_NOT_ENOUGH_FREE_VOLUME;
        return SYS_FS_RES_FAILURE;
    }
    /* Put the recently assigned disk as the current disk */
    gSYSFSCurrentMountPoint.inUse = true;
    gSYSFSCurrentMountPoint.currentDisk = disk;

    if(disk->fsType == FAT)
    {
        disk->fsFunctions->chdrive(disk->diskNumber);
    }

    /* Check, if the mount function in native FS is NULL or not */
    if(disk->fsFunctions->mount == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }

    fileStatus = disk->fsFunctions->mount(disk->diskNumber);

    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}


//******************************************************************************
/*Function:
    SYS_FS_RESULT _SYS_FS_DiskGet(const char *path, SYS_FS_MOUNT_POINT **disk)

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
SYS_FS_RESULT _SYS_FS_DiskGet(const char *path, SYS_FS_MOUNT_POINT **disk)
{
    uint8_t diskIndex;
    SYS_FS_MOUNT_POINT *diskMounted = (SYS_FS_MOUNT_POINT *)NULL;

    if (strncmp((const char*)path, (const char *)"/mnt/", 5) == 0)
    {
        // Start with 0th disk and find a disk which is available (not in use)
        for(diskIndex = 0; diskIndex != SYS_FS_VOLUME_NUMBER; diskIndex++)
        {
            diskMounted = &gSYSFSMountPoint[diskIndex];
            if(diskMounted->inUse == true)
            {
                // Find the element from "gSYSFSMediaDiskObject" which has matching "mountName"
                // I did not use "strcmp" as not sure if its implementation is re-entrant or not
                // ignore the first 5 chars --> "/mnt/"

                if(0 == strncmp((path + 5),diskMounted->mountName,strlen(diskMounted->mountName)))
                {
                    //we have got the "disk" with required name
                    break;
                }
            }
        }

        if(diskIndex >= SYS_FS_VOLUME_NUMBER)
        {
            SYS_ASSERT(false, "Invalid Disk");
            errorValue = SYS_FS_ERROR_INVALID_NAME;
            return SYS_FS_RES_FAILURE;
        }

    }
    else
    {   /* Since drive name is not expecitely mentioned, take the default current drive */
        if(gSYSFSCurrentMountPoint.inUse == false)
        {
            SYS_ASSERT(false, "Invalid mount point. Was the disk mounted?");
            errorValue = SYS_FS_ERROR_NO_FILESYSTEM;
            return SYS_FS_RES_FAILURE;
        }

        diskMounted = gSYSFSCurrentMountPoint.currentDisk;
    }

    *disk = diskMounted;

    return SYS_FS_RES_SUCCESS;
}
//******************************************************************************
/*Function:
    bool _SYS_FS_DiskNumberAppend(const char *path, uint8_t diskNumber, uint8_t *buffer, uint32_t len)

    Summary:
        Appends the disk number to the begining of the file name.

    Description:
        Appends the disk number to the begining of the file name. This is required for
        the native FAT FS working.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.


    Parameters:
        path            - Path for the file/ directory.

        disk     	- pointer of type SYS_FS_MOUNT_POINT.


    Returns:
        True - If the append operation was successful.
        False - If the append oepration was unsuccessful.

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
    None
***************************************************************************/
bool _SYS_FS_DiskNumberAppend
(
    const char *path, 
    uint8_t diskNumber, 
    uint8_t *buffer
)
{
    const char *ptr;
    uint32_t len = 0;

    ptr = path;
    len = strlen(path);

    if (strncmp(path, "/mnt/", 5) == 0)
    {
        /* Ignore the first 5 chars --> "/mnt/" */
        ptr = (path + 5);
        len = len - 5;

        /* Compare until the beginning of the path name is reached.
           Ex: path name = "/mnt/mydrive1/test/file.txt" where test is a
           subdirectory and file.txt a file under the sub directory test. Move
           the pointer to point to /test/file.txt
        */
        while ((len) && (*ptr != '/'))
        {
            len--;
            ptr++;
        }
    }

    if ((len + 3) > SYS_FS_PATH_LEN_WITH_DISK_NUM)
    {
        return false;
    }

    memset(buffer, 0, len + 3);

     /* Append "0:" before the file name. This is required for different disks
      * */
    *buffer++ = diskNumber + '0';
    *buffer++ = ':';

    if (len)
    {
        strncpy((char *)buffer, (const char *)ptr, len);
        len ++;
    }

    buffer[len] = '\0';

    return true;
}

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
bool _SYS_FS_StringWildCardCompare(const char * ptr1, const char *ptr2)
{
    /* Now, do comparison */
    do
    {
        if((*ptr1 != '?') && (*ptr1 != '*') && (*ptr1 != *ptr2))
        {
            return false;
        }
        else
        {
           if(*ptr1 == '*')
            {
                ptr1++;
                if(*ptr1 == '\0')
                    return true;
                else if(*ptr1 == '?')
                    continue;
                else
                {
                    // Find the same character in second string as well
                    while((*ptr1 != *ptr2) && (*ptr2 != '\0'))
                    {
                        ptr2++;
                    }
                    if(*ptr2 == '\0')
                        return false;
                }
            }
            else
            {
                ptr1++;
                ptr2++;
            }
        }
    }while(*ptr1 != '\0');

    if((*ptr1 == '\0') && (*ptr2 == '\0'))
    {
        return true;
    }
    return false;
}

//******************************************************************************
/* Function:
    SYS_FS_HANDLE SYS_FS_FileOpen
    (
        const char* fname, 
        SYS_FS_FILE_OPEN_ATTRIBUTES attributes
    );

  Summary:
     Opens a file

  Description:
     This function opens a file with the requested attributes.

  Precondition:
    Prior to opening a file, the name of the volume on which the file resides
    should be known and the volume should be mounted. 

  Parameters:
    fname         - The name of the file to be opened along with the path.
                    The fname format is as follows:
                    "/mnt/volumeName/dirName/fileName" where 
                    volumeName - name of the volume/drive
                    dirName - name of the directory under which the file is
                              located 
                    fileName - name of the file to be opened
                      
                    The "/mnt/volumeName" portion from the fName can be omitted
                    if the SYS_FS_CurrentDriveSet () has been invoked to set
                    the current drive/volume.

    attributes    - Access mode of the file, of type
                    SYS_FS_FILE_OPEN_ATTRIBUTES

  Returns:
        If Success: Valid handle will be returned
        If Failure: returned handle will be SYS_FS_HANDLE_INVALID

  Remarks:
	None
*/

SYS_FS_HANDLE SYS_FS_FileOpen
(
    const char *fname,
    SYS_FS_FILE_OPEN_ATTRIBUTES attributes
)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    uint32_t j = 0;
    SYS_FS_OBJ * obj = (SYS_FS_OBJ *)NULL;
    uint8_t pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    const uint8_t *Temp  = (const uint8_t *)NULL;
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;
    uint8_t mode = 0;

    /* Get disk number */
    if(_SYS_FS_DiskGet(fname, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_HANDLE_INVALID;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(fname, (uint8_t)disk->diskNumber, pathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_HANDLE_INVALID;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if((disk->fsType == MPFS2) && (attributes != SYS_FS_FILE_OPEN_READ))
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_HANDLE_INVALID;
    }

    obj = NULL;
    /* Mutex Lock needed here since File Open can be called
    simultaneously from two different tasks */
    if(OSAL_MUTEX_Lock(&mutexFileSystemObj, OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        for(j = 0; j < SYS_FS_MAX_FILES; j ++)
        {
            if(gSYSFSFileObj[j].inUse == false)
            {
                gSYSFSFileObj[j].inUse = true;
                gSYSFSFileObj[j].mountPoint = disk;
                obj = &gSYSFSFileObj[j];
                break;
            }
        }
        OSAL_ASSERT((OSAL_MUTEX_Unlock(&mutexFileSystemObj)),"Unable to unlock mutex");
    }

    /* If the object is NULL, then we dont have a free
     * file system object */

    if(obj == NULL)
    {
        errorValue = SYS_FS_ERROR_TOO_MANY_OPEN_FILES;
        return(SYS_FS_HANDLE_INVALID);
    }

    /* Save the file name for future use */
    Temp = &pathWithDiskNo[3];   // ignore the "0:/"
    if (pathWithDiskNo[2] != '/')
        Temp = &pathWithDiskNo[2];   // ignore the "0:"

    for(j = 0;  j < FAT_FS_MAX_LFN; j ++)
    {
        obj->fileName[j] = *Temp;
        if(*Temp++ == '\0')
            break;
    }

    /* Now, call the real file open function */

    if(disk->fsFunctions->open == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        obj->inUse = false;
        return (SYS_FS_HANDLE_INVALID);
    }
    /* Convert the SYS_FS file open attributes to FAT FS attributes */
    switch(attributes)
    {
        case SYS_FS_FILE_OPEN_READ:
            mode = FA_READ;
            break;
        case SYS_FS_FILE_OPEN_WRITE:
             mode = FA_WRITE | FA_OPEN_ALWAYS;
            break;
        case SYS_FS_FILE_OPEN_APPEND:
            mode = FA_WRITE | FA_OPEN_ALWAYS;
            break;
        case SYS_FS_FILE_OPEN_READ_PLUS:
            mode = FA_READ | FA_WRITE;
            break;
        case SYS_FS_FILE_OPEN_WRITE_PLUS:
            mode = FA_READ | FA_WRITE | FA_OPEN_ALWAYS;
            break;
        case SYS_FS_FILE_OPEN_APPEND_PLUS:
            mode = FA_READ | FA_WRITE | FA_OPEN_ALWAYS;
            break;
        default:
            mode = FA__ERROR;
            break;
    }

    if (OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->open((uintptr_t)&obj->nativeFSFileObj, (const char *)pathWithDiskNo, mode);
        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }

    if (fileStatus != 0)
    {
	    /* File open failed. */
        errorValue = fileStatus;
    }
    else
    {
        if ((SYS_FS_FILE_OPEN_APPEND == attributes) || (SYS_FS_FILE_OPEN_APPEND_PLUS == attributes))
        {
            fileStatus = SYS_FS_FileSeek((SYS_FS_HANDLE)obj, 0, SYS_FS_SEEK_END);
        }
    }

    if (fileStatus != 0)
    {
	    /* File open/seek failed. */
        obj->inUse = false;
        return (SYS_FS_HANDLE_INVALID);
    }

    return ((SYS_FS_HANDLE)obj);
}

//******************************************************************************
/*Function:
    bool SYS_FS_FileNameGet(SYS_FS_HANDLE handle, uint8_t* cName, uint16_t wLen)

    Summary:
        Reads the file name.

    Description:
        Reads the file name of a file that is already open.

    Precondition:
        The file handle referenced by handle is already open.

    Parameters:
        handle 	- file handle obtaind during file Open.
 	cName - where to store the name of the file.
	wLen - the maximum length of data to store in cName.

    Returns:
		If Success	-

			The file name was successfully located	- true

        	If Failure
                        The file handle provided is not currently open	- false

		The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
***************************************************************************/
bool SYS_FS_FileNameGet(SYS_FS_HANDLE handle, uint8_t* cName, uint16_t wLen)
{
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
    uint32_t j;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return false;
    }

    if(obj->inUse == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return false;
    }

    if(wLen > FAT_FS_MAX_LFN)
    {
        wLen = FAT_FS_MAX_LFN;
    }

    for(j = 0;  j < wLen; j ++)
    {
        cName[j] = obj->fileName[j];
        /* break, in case end of string reached */
        if(obj->fileName[j] == '\0')
            break;
    }

    return true;

}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_Unmount(const char *mountName);


    Summary:
        unmount filesystems

    Description:
        remove the attachment of the (topmost) filesystem mounted on target

    Precondition:
        none

    Parameters:
        mountName - Mount name for the device to be mounted

    Returns:
        If Success: SYS_FS_RES_SUCCESS
        If Failure: SYS_FS_RES_FAILURE
            Sets error code which can be retrieved with SYS_FS_Error
***************************************************************************/
SYS_FS_RESULT SYS_FS_Unmount(const char *fname)
{
    int fileStatus = -1;
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;
    uint32_t index = 0;

    /* Get disk number */
    if(_SYS_FS_DiskGet(fname, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file mount function */
    if(disk->fsFunctions->unmount == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }

    fileStatus = disk->fsFunctions->unmount(disk->diskNumber);

    if(fileStatus == 0)
    {
        for( index = 0; index != SYS_FS_MAX_FILES; index++ )
        {
            if(gSYSFSFileObj[index].mountPoint == disk)
            {
                gSYSFSFileObj[index].inUse = false;
            }
            if(gSYSFSDirObj[index].mountPoint == disk)
            {
                gSYSFSDirObj[index].inUse = false;
            }            
        }       
        disk->inUse = 0;
        disk->mountName = NULL;
        disk->fsFunctions = NULL;
        
        OSAL_MUTEX_Delete(&disk->mutexDiskVolume);
        
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }

}

//******************************************************************************
/* Function:
	size_t SYS_FS_FileRead(SYS_FS_HANDLE handle, void *buf, size_t nbyte)


  Summary:
     Read specified bytes from a file

  Description:
	The SYS_FS_FileRead() function shall attempt to read nbyte bytes from the file associated with
	the open file handle into the buffer pointed to by buf.

  Precondition:
    A valid file handle must be obtained before reading a file.

  Parameters:
        handle			- File handle obtained during file open.
		buf				- Pointer to buffer in which data is read into.
		nbyte			- No of bytes to be read


  Returns:
        If Success: The number of bytes read (0 or positive number)
        If Failure: -1

  Remarks:
	None
*/
size_t SYS_FS_FileRead
(
    SYS_FS_HANDLE handle,
    void *buffer,
    size_t nbyte
 )
{
    int fileStatus = -1;
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
    uint32_t nosOfDataRead = 0;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    /* Now, call the real file open function */

    if(obj->mountPoint->fsFunctions->read == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }
    /* Protect the read from simultaneous access */
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = 	obj->mountPoint->fsFunctions->read(obj->nativeFSFileObj, buffer,
            nbyte, &nosOfDataRead);

        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return nosOfDataRead;
    }
    else
    {
        obj->errorValue = fileStatus;
        return -1;
    }
}

//******************************************************************************
/* Function:
	size_t SYS_FS_FileWrite(SYS_FS_HANDLE handle, const void *buf, size_t nbyte)

  Summary:
     Write on the file

  Description:
	The SYS_FS_FileWrite() function shall attempt to write nbyte bytes from the buffer pointed to by buf to
	the file associated with the open file handle.

  Precondition:
    A valid file handle must be obtained before reading a file.

  Parameters:
        handle			- File handle obtained during file open.
		buf				- Pointer to buffer from which data is to be written
		nbyte			- No of bytes to be written


  Returns:
        If Success: The number of bytes written (0 or positive number)
        If Failure: -1

  Remarks:
	None
*/
size_t SYS_FS_FileWrite
(
    SYS_FS_HANDLE handle,
    const void *buffer,
    size_t nbyte

 )
{
    int fileStatus = -1;
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
    uint32_t nosOfDataWritten = 0;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    /* Now, call the real file open function */
    if(obj->mountPoint->fsFunctions->write == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {
        fileStatus = obj->mountPoint->fsFunctions->write(obj->nativeFSFileObj,
                                              buffer, nbyte, &nosOfDataWritten);

        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return nosOfDataWritten;
    }
    else
    {
        obj->errorValue = fileStatus;
        return -1;
    }
}

//******************************************************************************
/* Function:
	int SYS_FS_FileSeek(SYS_FS_HANDLE handle, int offset, SYS_FS_FILE_SEEK_CONTROL whence)

  Summary:
     Move the read/write file pointer

  Description:
	The SYS_FS_FileSeek() function shall set the file offset for the open file description associated with
	the file handle, as follows:

    If whence is SYS_FS_SEEK_SET, the file offset shall be set to offset bytes from the begining.
    If whence is SYS_FS_SEEK_CUR, the file offset shall be set to its current location plus offset.
    If whence is SYS_FS_SEEK_END, the file offset shall be set to the size of the file plus offset.

	The behavior of SYS_FS_FileSeek() on devices which are incapable of seeking is implementation-defined.
	The value of the file offset associated with such a device is undefined.

  Precondition:
    None.

  Parameters:
    handle				- A valid file handle
	offset				- The number of bytes which act as file offset
	whence				- File seek control input

  Returns:
        If Success: The number of bytes written (0 or positive number)
        If Failure: -1

  Remarks:
	None
*/
int32_t SYS_FS_FileSeek
(
    SYS_FS_HANDLE handle,
    int32_t offset,
    SYS_FS_FILE_SEEK_CONTROL whence
 )
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
    long tell = 0;
    uint32_t size = 0;
    int temp;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if((obj->mountPoint->fsFunctions->seek == NULL) || (obj->mountPoint->fsFunctions->tell == NULL) ||
            (obj->mountPoint->fsFunctions->size == NULL))
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }

    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        switch (whence)
        {
            case SYS_FS_SEEK_SET:
                fileStatus = obj->mountPoint->fsFunctions->seek(obj->nativeFSFileObj, offset);
                break;
            case SYS_FS_SEEK_CUR:
                tell = obj->mountPoint->fsFunctions->tell(obj->nativeFSFileObj);
                temp = (offset + tell);
                fileStatus = obj->mountPoint->fsFunctions->seek(obj->nativeFSFileObj, temp);
                break;
            case SYS_FS_SEEK_END:
                size = obj->mountPoint->fsFunctions->size(obj->nativeFSFileObj);
                temp = (offset + size);
                fileStatus = obj->mountPoint->fsFunctions->seek(obj->nativeFSFileObj, temp);
                break;
        }

       OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }


    if(fileStatus == 0)
    {
        /* This is success value, but required in special case, where requested offset was (-1) */
        obj->errorValue = SYS_FS_ERROR_OK;
        return offset;
    }
    else
    {
        obj->errorValue = fileStatus;
        return -1;
    }
}

//******************************************************************************
/*Function:
    int32_t SYS_FS_FileTell(SYS_FS_HANDLE handle)

    Summary:
        Obtains the file pointer position

    Description:
        Obtains the current value of the file position indicator for the
        handle pointed to by handle.

    Precondition:
        none

    Parameters:
        handle - File handle

    Returns:
        If Success: current offset
        If Failure:  -1
            Sets error code which can be retrieved with SYS_FS_FileError

  Remarks:
	None

***************************************************************************/
int32_t SYS_FS_FileTell
(
    SYS_FS_HANDLE handle
 )
{
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
    long status = -1;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if(obj->mountPoint->fsFunctions->tell == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }

    status = obj->mountPoint->fsFunctions->tell(obj->nativeFSFileObj);

    return status;
}

//******************************************************************************
/*Function:
    int32_t SYS_FS_FileSize( SYS_FS_HANDLE handle )

    Summary:
    Returns the size of the file

    Description:
        Returns the size of the file as pointed by the handle.

    Precondition:
        none

    Parameters:
        handle - File handle

    Returns:
        If Success: file size
        If Failure: -1
			Sets error code which can be retrieved with SYS_FS_FileError

  Remarks:
	None
***************************************************************************/
int32_t SYS_FS_FileSize
(
    SYS_FS_HANDLE handle
 )
{
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
    long status = -1;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if(obj->mountPoint->fsFunctions->size == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }

    status = obj->mountPoint->fsFunctions->size(obj->nativeFSFileObj);

    return status;
}

//******************************************************************************
/*Function:
    bool SYS_FS_FileEOF(SYS_FS_HANDLE handle)

    Summary:
        check handle status

    Description:
        Checks weather or not the file position indicator is at the end of
        the file.

    Precondition:
        none

    Parameters:
        handle - file handle

    Returns:
		When file pointer not reached the end of file:	false
		When file pointer reached the end of file:		true
		When failure:						false
			Sets error code which can be retrieved with SYS_FS_FileError
  Remarks:
	None
***************************************************************************/
bool SYS_FS_FileEOF
(
    SYS_FS_HANDLE handle
 )
{
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
    volatile bool status = -1;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return -1;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return false;
    }

    if(obj->mountPoint->fsFunctions->eof == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return false;
    }

    status = obj->mountPoint->fsFunctions->eof(obj->nativeFSFileObj);

    /* This is success value, but required in special case, where requested offset was (-1) */
    obj->errorValue = SYS_FS_ERROR_OK;
    return (bool)status;
}

//******************************************************************************
/* Function:
	SYS_FS_RESULT SYS_FS_FileStat(const char* path, SYS_FS_FSTAT *buf)

  Summary:
     Get file status

  Description:
	The SYS_FS_FileStat() function shall obtain information about a file associated
	with the file name, and shall write it to the area pointed to by buf.

	The buf argument is a pointer to a SYS_FS_FSTAT structure,into which information
	is placed concerning the file.

	This function can read the status of file irrespective of a file is opened or not.

  Precondition:
    None.

  Parameters:
    path      	     			- Path to the file
	buf							- pointer to SYS_FS_FSTAT variable

  Returns:
        If Success: SYS_FS_RES_SUCCESS
        If Failure: SYS_FS_RES_FAILURE

  Remarks:
	None
*/

SYS_FS_RESULT SYS_FS_FileStat
(
    const char *fname,
    SYS_FS_FSTAT *buf
)
{
    int fileStatus = -1;
    uint8_t pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;

    /* Get disk number */
    /* There is possibility of data corruption inside _SYS_FS_DiskGet()\
     * If the unmount API is called from another task when gSYSFSMountPoint
     * is being accessed by one task. This can be a restriction for application
     */
    if(_SYS_FS_DiskGet(fname, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(fname, (uint8_t)disk->diskNumber, pathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_RES_FAILURE;
    }

    if(disk->fsFunctions->fstat == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }

    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->fstat((const char *)pathWithDiskNo, (uintptr_t)buf);

        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }
    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/* Function:
	SYS_FS_RESULT SYS_FS_FileClose(SYS_FS_HANDLE handle);

  Summary:
     Close a file descriptor

  Description:
	The SYS_FS_FileClose() function closes an opened file

  Precondition:
    None.

  Parameters:
    handle			- A valid handle, which was obtained while opening the file.

  Returns:
        If Success: SYS_FS_RES_SUCCESS
        If Failure: SYS_FS_RES_FAILURE

  Remarks:
	None
*/
SYS_FS_RESULT SYS_FS_FileClose
(
 SYS_FS_HANDLE handle
 )
{
    int fileStatus = -1;;
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        SYS_ASSERT(false,"File object is not in use");
        return SYS_FS_RES_FAILURE;
    }

    if(obj->mountPoint->fsFunctions->close == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = obj->mountPoint->fsFunctions->close(obj->nativeFSFileObj);

        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }
    if(fileStatus == 0)
    {
        /* Return the SYS_FS file system object. */
        obj->inUse = false;
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }

    /* Do the following in disk un-mount only */
    //	sysFsObject[mountPoint].inUse = 0;
}

//******************************************************************************
/*Function:
    SYS_FS_ERROR SYS_FS_Error(void)

    Summary:
        check the type of error

    Description:
        When a file system operation fails, the application can know the exact
        reason of failure by calling the SYS_FS_Error. This function only reports
        the errors which are not file (or file handle) specific. For example, for
        functions such as SYS_FS_Mount and SYS_FS_FileOpen, which do not take handle,
        any errors happening inside such function calls could be reported using
        SYS_FS_Error function. Even for functions, which take handle as its input
        parameters, the SYS_FS_Error function can be used to report the type of
        error for cases where the passed handle itself is invalid.

    Precondition:
        none

    Parameters:
        none

    Returns:
		Error code of type SYS_FS_ERROR

  Remarks:
	None
***************************************************************************/
SYS_FS_ERROR SYS_FS_Error(void)
{
    return errorValue;
}

//******************************************************************************
/*Function:
    SYS_FS_ERROR SYS_FS_FileError(SYS_FS_HANDLE handle)

    Summary:
        check the type of file specific error

    Description:
        For file system functions which accepts valid handle, any error happening
        in those functions could be retrived with SYS_FS_FileError. This function
        returns errors which are file specific.

        Please note that if an invalid handle is passed to a file system function,
        in such a case, SYS_FS_FileError will not return the correct type of error,
        as the handle was invalid. Hence it would be prudent to check the errors
        using the SYS_FS_Error function.

    Precondition:
        This function has to be called immediately after a failure is observed while doing
        a file operation. Any subsequent failure will overwrite the cause
        of pervious failure.

    Parameters:
        handle      -   A valid file handle

    Returns:
		Error code of type SYS_FS_ERROR

  Remarks:
	None
***************************************************************************/
SYS_FS_ERROR SYS_FS_FileError(SYS_FS_HANDLE handle)
{
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        return SYS_FS_ERROR_INVALID_OBJECT;
    }

    if(obj->inUse == false)
    {
        return SYS_FS_ERROR_INVALID_OBJECT;
    }

    return obj->errorValue;
}
// *****************************************************************************
/* Function:
    void SYS_FS_Tasks ( void )

  Summary:
    Tasks for the sys_fs layer

  Description:
    This routine is used to run the varioius tasks and functionalities of sys_fs
    layer.

  Precondition:
    The SYS_FS_Initialize routine must have been called before running the tasks.

  Parameters:
    None.

  Returns:
    None
*/

void SYS_FS_Tasks ( void )
{
    /* Task routine for media manager */
    SYS_FS_MEDIA_MANAGER_Tasks();
}
//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_DirectoryMake(const char* path)

    Summary:
        Make directory

    Description:
        Make a new directory as per the specified path.

    Precondition:
        The disk has to be mounted before a directory could be made.

    Parameters:
        path 	- A path for making the directory.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_DirectoryMake(const char* path)
{
    volatile int fileStatus = SYS_FS_ERROR_NOT_READY;
    uint8_t pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *)NULL;;


    /* Get disk number */
    if(_SYS_FS_DiskGet(path, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(path, (uint8_t)disk->diskNumber, pathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_RES_FAILURE;
    }

    if(disk->fsFunctions->mkdir == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->mkdir((const char *)pathWithDiskNo);

        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}
//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_DirectoryChange(const char* path)

    Summary:
        Change directory

    Description:
        Change the present directory to a new directory.

    Precondition:
        The disk has to be mounted before a directory could be made.
        The directory to be changed has to present.

    Parameters:
        path 	- A path for changing the directory.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_DirectoryChange(const char* path)
{
    volatile int fileStatus = SYS_FS_ERROR_NOT_READY;
    uint8_t pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;

    /* Get disk number */
    if(_SYS_FS_DiskGet(path, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if(_SYS_FS_DiskNumberAppend(path, (uint8_t)disk->diskNumber, pathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_RES_FAILURE;
    }

    if(disk->fsFunctions->chdir == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->chdir((const char *)pathWithDiskNo);

        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}
//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileDirectoryRemove(const char* path)

    Summary:
        Remove a file or directory

    Description:
        Remove a file or directory as specified by the path.

    Precondition:
        - The disk has to be mounted before a directory could be removed.
        - The file or directory to be removed has to present.
        - The file/sub-directory must not have read-only attribute (AM_RDO),
          or the function will be rejected with FR_DENIED.
        - The sub-directory must be empty and must not be current directory,
          or the function will be rejected with FR_DENIED.
        - The file/sub-directory must not be opened.

    Parameters:
        path 	- A path for removing the file or directory.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileDirectoryRemove(const char* path)
{
    volatile int fileStatus = SYS_FS_ERROR_NOT_READY;
    uint8_t pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;

    /* Get disk number */
    if(_SYS_FS_DiskGet(path, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(path, (uint8_t)disk->diskNumber, pathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_RES_FAILURE;
    }

    if(disk->fsFunctions->remove == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->remove((const char *)pathWithDiskNo);

        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }
    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}
//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_CurrentDriveGet(char* buffer)

    Summary:
        Get current drive

    Description:
        Get the present drive being used and put the name of drive into the buffer.

    Precondition:
        The disk has to be mounted before this funcion could be set.


    Parameters:
        Pointer to buffer which will hold the name of present drive being used.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error

    Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_CurrentDriveGet(char* buffer)
{
    const char *ptr = gSYSFSCurrentMountPoint.currentDisk->mountName;
    char *localBuffer = "/mnt/";

    if(gSYSFSCurrentMountPoint.inUse == false)
    {
        errorValue = SYS_FS_ERROR_NO_FILESYSTEM;
        return SYS_FS_RES_FAILURE;
    }
    strcpy(buffer, localBuffer);
    strcat(buffer, ptr);

    return SYS_FS_RES_SUCCESS;

}
//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_CurrentDriveSet(const char* path)

    Summary:
        Set directory

    Description:
        Set the present directory to the one as specified by the path.

    Precondition:
        The disk has to be mounted before it could be set.


    Parameters:
        path 	- A path for the drive to be set.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
    Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_CurrentDriveSet(const char* path)
{
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;
    int fileStatus = SYS_FS_ERROR_NOT_READY;

    /* Get disk number */
    if(_SYS_FS_DiskGet(path, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        gSYSFSCurrentMountPoint.currentDisk = disk;
        return SYS_FS_RES_SUCCESS;
    }

    if(gSYSFSCurrentMountPoint.inUse == false)
    {
        SYS_ASSERT(false, "Invalid mount point. Was the disk mounted?");
        errorValue = SYS_FS_ERROR_NO_FILESYSTEM;
        return SYS_FS_RES_FAILURE;
   }

   if(disk->fsFunctions->chdrive == NULL)
   {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
   }

   fileStatus = disk->fsFunctions->chdrive(disk->diskNumber);

    if(fileStatus == 0)
    {
        gSYSFSCurrentMountPoint.currentDisk = disk;
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_DriveLabelGet(const char* drive, char *buff, uint32_t *sn)

    Summary:
        Get drive label

    Description:
        Get the label for the drive specified. If no drive is specified, then the label
        for the current drive is obtained.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.


    Parameters:
        drive           -       Pointer to buffer which will hold the name of
                                drive being for which, the label is requested. If
                                this string is NULL, then then label of the current
                                drive is obtained by using this function.
        buff            -       Buffer which will hold the string of label.
        sn              -       Serial number of the drive. If this information is
                                not needed, it can be set as NULL.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_DriveLabelGet(const char* drive, char *buff, uint32_t *sn)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;
    uint8_t pathWithDiskNo[3] = {};

    if(drive != NULL)
    {
        /* Get disk number */
        if(_SYS_FS_DiskGet(drive, &disk) == SYS_FS_RES_FAILURE)
        {
            // reason or cause of error is alredy present in "errorValue" variable
            return SYS_FS_RES_FAILURE;
        }
    }
    else    /* if(drive == NULL */
    {
        if(gSYSFSCurrentMountPoint.inUse == false)
        {
            SYS_ASSERT(false, "Invalid mount point. Was the disk mounted?");
            errorValue = SYS_FS_ERROR_NO_FILESYSTEM;
            return SYS_FS_RES_FAILURE;
        }

        disk = gSYSFSCurrentMountPoint.currentDisk;

    }

    /* For MPFS file system, label name is not supported */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Append "0:" before the file name. This is required
     * for different disks */
    pathWithDiskNo[0] = (uint8_t)disk->diskNumber + '0';
    pathWithDiskNo[1] = ':';
    pathWithDiskNo[2] = '\0';

    if(disk->fsFunctions->getlabel == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->getlabel((const char *)pathWithDiskNo, buff, sn);
        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }

}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_DriveLabelSet(const char* drive, const char *label)

    Summary:
        Set drive label

    Description:
        Set the label for the drive specified. If no drive is specified, then the label
        for the current drive is set.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.


    Parameters:
        drive           -       Pointer to string which holds the name of
                                drive being for which, the label is to be set. If
                                this string is NULL, then then label of the current
                                drive is set by using this function.
        label           -       Pointer to string which contains the label to be set.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_DriveLabelSet(const char *drive, const char *label)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;
    char pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};

    if(label == NULL)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    if(drive != NULL)
    {
        /* Get disk number */
        if(_SYS_FS_DiskGet(drive, &disk) == SYS_FS_RES_FAILURE)
        {
            // reason or cause of error is alredy present in "errorValue" variable
            return SYS_FS_RES_FAILURE;
        }
    }
    else    /* if(drive == NULL */
    {
        if(gSYSFSCurrentMountPoint.inUse == false)
        {
            SYS_ASSERT(false, "Invalid mount point. Was the disk mounted?");
            errorValue = SYS_FS_ERROR_NO_FILESYSTEM;
            return SYS_FS_RES_FAILURE;
        }

        disk = gSYSFSCurrentMountPoint.currentDisk;
    }

    /* For MPFS file system, label name is not supported */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Append "0:" before the file name. This is required
     * for different disks */
    pathWithDiskNo[0] = (uint8_t)disk->diskNumber + '0';
    pathWithDiskNo[1] = ':';

    /* Form the name with the drive letter */
    strcpy((char *)&pathWithDiskNo[2], (const char *)label);


    if(disk->fsFunctions->setlabel == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->setlabel((const char *)pathWithDiskNo);
        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }
    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}
//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileTruncate( SYS_FS_HANDLE handle )

    Summary:
        Truncate a file

    Description:
        The function truncates the file size to the current file read/write pointer.
        This function has no effect if the file read/write pointer is already pointing
        end of the file.

    Precondition:
        A valid handle of a file has to be passed as input to the function.
         The file has to be opened in a mode where write to file is possible
        (such as read plus or write mode).

    Parameters:
        handle           -      A valid handle which was obtained while opening the file.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileTruncate( SYS_FS_HANDLE handle )
{
    int fileStatus = -1;
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */
    if(obj->mountPoint->fsFunctions->truncate == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {
        fileStatus = obj->mountPoint->fsFunctions->truncate(obj->nativeFSFileObj);

        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_CurrentWorkingDirectoryGet(char *buff, uint32_t len)

    Summary:
    Get the current working directory

    Description:
        Get the current working directory path along with the working drive.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.


    Parameters:
        buff            -       Pointer to a buffer which will contain the name
                                of the current working directory and drive, once
                                the function completes.

        len           -         Size of the buffer.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_CurrentWorkingDirectoryGet(char *buffer, uint32_t len)
{
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *)NULL;
    int fileStatus = -1;
    char directoryHolder[FAT_FS_MAX_LFN] = {};
    char *ptr = (char *)NULL, *tempBuffer = (char *)NULL;
    const char *mntName = (const char *)NULL;

    tempBuffer = buffer;

    if(gSYSFSCurrentMountPoint.inUse == false)
    {
        errorValue = SYS_FS_ERROR_NO_FILESYSTEM;
        return SYS_FS_RES_FAILURE;
    }

    /* This is the current drive in sys_fs. For this current
       drive, get the current working directory */
    disk = gSYSFSCurrentMountPoint.currentDisk;


    /* For MPFS file system, label name is not supported */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */
    if(disk->fsFunctions->currWD == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->currWD(buffer, len);
        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }

    if(fileStatus == 0) /* If function call was success */
    {
        /* Check if the first value of the returned buffer is a ASCII for digit */
        if((*buffer >= '0') && (*buffer <= '9'))
        {
            /* verify if the current drive in native file system matches with
               the current drive in sys_fs layer */
            if(disk->diskNumber == (*buffer - '0'))
            {
                buffer++;  // to ignore the drive number
                buffer++;  // to ignore the ":"

                /* store the current working directory name in a temp buffer */
                ptr = &directoryHolder[0];

                while(*buffer != '\0')
                {
                    *ptr++  = *buffer++;
                }
                *ptr = '\0';
                buffer = tempBuffer;

                *buffer++ = '/';
                *buffer++ = 'm';
                *buffer++ = 'n';
                *buffer++ = 't';
                *buffer++ = '/';

                len = len - 5;  /* for "/mnt/" */

                mntName = disk->mountName;

                while(*mntName != '\0')
                {
                    *buffer++ = *mntName++;
                    len--;
                }
                /* Now, add the current working directory name which is stored in temp buffer */
                ptr = &directoryHolder[0];

                while(*ptr != '\0')
                {
                    *buffer++  = *ptr++;
                    len--;
                }

                /* Fill up remaining unused area of buffer with NULL */
                while(len != 0)
                {
                    *buffer++ = '\0';
                    len--;
                }
            }
            else
            {
                errorValue = SYS_FS_ERROR_INVALID_NAME;
                return SYS_FS_RES_FAILURE;
            }
        }
        else /* If first value of the returned buffer is a not an ASCII for digit */
        {
            /* Must be becuse, the application as 1 volume and hence, the
             return name of current directory from native file system does
             not have the drive name explecitely mentioned. We will add it
             from the sys_fs layer as below */

            /* store the current working directory name in a temp buffer */
            ptr = &directoryHolder[0];

            while(*buffer != '\0')
            {
                *ptr++  = *buffer++;
            }
            *ptr = '\0';
            buffer = tempBuffer;


            *buffer++ = '/';
            *buffer++ = 'm';
            *buffer++ = 'n';
            *buffer++ = 't';
            *buffer++ = '/';

            len = len - 5;  /* for "/mnt/" */

            mntName = disk->mountName;

            while(*mntName != '\0')
            {
                *buffer++ = *mntName++;
                len--;
            }
            /* Now, add the current working directory name which is stored in temp buffer */
            ptr = &directoryHolder[0];

            while(*ptr != '\0')
            {
                *buffer++  = *ptr++;
                len--;
            }

            /* Fill up remaining unused area of buffer with NULL */
            while(len != 0)
            {
                *buffer++ = '\0';
                len--;
            }
        }

        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileDirectoryModeSet(const char* path, SYS_FS_FILE_DIR_ATTR attr,
        SYS_FS_FILE_DIR_ATTR mask);

    Summary:
        Mode set for file/ directory.

    Description:
        Sets the mode for a file or directory from the spcified list of attributes.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.
        The file/ directory for this the mode is set, has to be present.


    Parameters:
        path 	- A path for the file/ directory, for which the mode is to
                  be set.

        attr     - Attribute flags to be set in one or more combination of
                   the type SYS_FS_FILE_DIR_ATTR. The specified flags are
                   set and others are cleard.

        mask     - Attribute mask  of type SYS_FS_FILE_DIR_ATTR that specifies
                   which attribute is changed. The specified aattributes are set
                   or cleard.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileDirectoryModeSet(const char* fname, SYS_FS_FILE_DIR_ATTR attr,
        SYS_FS_FILE_DIR_ATTR mask)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    uint8_t pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;

    /* Get disk number */
    if(_SYS_FS_DiskGet(fname, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(fname, (uint8_t)disk->diskNumber, pathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    if(disk->fsFunctions->chmode == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return (SYS_FS_RES_FAILURE);
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->chmode((const char *)pathWithDiskNo, attr, mask);

        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return(SYS_FS_RES_SUCCESS);
    }
    else
    {
        errorValue = fileStatus;
        return(SYS_FS_RES_FAILURE);
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileDirectoryTimeSet(const char* path, SYS_FS_TIME *time)

    Summary:
        Time set for file/ directory.

    Description:
        Sets or change the time for a file or directory.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.
        The file/ directory for which time has to be set, has to be present.


    Parameters:
        path 	- A path for the file/ directory, for which the mode is to
                  be set.

        ptr     - Pointer to structure of type SYS_FS_TIME, which contains the
                  time data already set in.


    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileDirectoryTimeSet(const char* fname, SYS_FS_TIME *time)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    uint8_t pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;
    SYS_FS_FSTAT stat = {};

    /* Get disk number */
    if(_SYS_FS_DiskGet(fname, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(fname, (uint8_t)disk->diskNumber, pathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, transfer the time from elements of "SYS_FS_TIME" to elements of "SYS_FS_FSTAT" */
    stat.fdate = time->timeDate.date;
    stat.ftime = time->timeDate.time;

    /* Now, call the real file open function */

    if(disk->fsFunctions->chtime == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return (SYS_FS_RES_FAILURE);
    }

    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {
        fileStatus = disk->fsFunctions->chtime((const char *)pathWithDiskNo, (uintptr_t)&stat);
        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return(SYS_FS_RES_SUCCESS);
    }
    else
    {
        errorValue = fileStatus;
        return(SYS_FS_RES_FAILURE);
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileDirectoryRenameMove(const char *oldPath, const char *newPath)

    Summary:
        Rename or move a file/ directory.

    Description:
        Renames or moves a file or directory.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.
        The file/ directory which has to be renamed/ moved, has to be present.

        This function cannot move files/ directory from one drive to another.
        Do not rename/ move files which are open.

    Parameters:
        oldPath 	- Path for the file/ directory, which has to be renamed/ moved.

        newPath 	- New Path for the file/ directory.


    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
        This function cannot move files/ directory from one drive to another.
        Do not rename/ move files which are open.
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileDirectoryRenameMove(const char *oldPath, const char *newPath)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    uint8_t oldPathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    uint8_t newPathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;

    /* Get disk number */
    if(_SYS_FS_DiskGet(oldPath, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(oldPath, (uint8_t)disk->diskNumber, oldPathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_RES_FAILURE;
    }

    /***********************************************************************************/
    /**************************NOW, repeat the above steps for other new path***********/
    /***********************************************************************************/
    /* Get disk number */
    if(_SYS_FS_DiskGet(newPath, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(newPath, (uint8_t)disk->diskNumber, newPathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */
    if(disk->fsFunctions->rename == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return (SYS_FS_RES_FAILURE);
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {
         /* This function does not take disk number (like "0:"). Hence use from index "2" onwards */
        fileStatus = disk->fsFunctions->rename((const char *)&oldPathWithDiskNo[2], (const char *)&newPathWithDiskNo[2]);
        OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));

    }
    if(fileStatus == 0)
    {
        return(SYS_FS_RES_SUCCESS);
    }
    else
    {
        errorValue = fileStatus;
        return(SYS_FS_RES_FAILURE);
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileSync( SYS_FS_HANDLE handle )

    Summary:
        File flush.

    Description:
        The function flushes the cached information of a writing file.
        The SYS_FS_FileSync() function performs the same process as SYS_FS_FileClose()
        function but the file is left opened and can continue read/write/seek operations
        to the file.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.
        The file which has to be flushed, has to be present and should have been opened
        in write mode.


    Parameters:
        handle         -       Handle for the file received when the file was opened.


    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
        None.
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileSync( SYS_FS_HANDLE handle )
{
   int fileStatus = -1;
   SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    if(obj->mountPoint->fsFunctions->sync == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {

        fileStatus = obj->mountPoint->fsFunctions->sync(obj->nativeFSFileObj);
        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileStringGet( SYS_FS_HANDLE handle, char* buff, uint32_t len )

    Summary:
        String read.

    Description:
        The function reads a string of specified length from the file into a buffer.
        The read operation continues until a '\n' is stored, reached end of the file
        or the buffer is filled with len - 1 characters. The read string is terminated
        with a '\0'.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.
        The file from which a string has to be read, has to be present and should have
        been opened.


    Parameters:
        handle         -       Handle for the file received when the file was opened.
        buff           -       Buffer to read string.
        len            -       length of string to be read.


    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
        None.
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileStringGet( SYS_FS_HANDLE handle, char* buff, uint32_t len )
{
   int fileStatus = SYS_FS_ERROR_NOT_READY;
   SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
   char *ptr = NULL;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    if(obj->mountPoint->fsFunctions->getstrn == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {

        ptr = obj->mountPoint->fsFunctions->getstrn(buff, len, obj->nativeFSFileObj);
        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(buff == ptr)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else //if(ptr == NULL)
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileCharacterPut( SYS_FS_HANDLE handle, char data )

    Summary:
        Character write.

    Description:
        The function writes a character into a file.
        When the function failed due to disk full or any error, SYS_FS_RES_FAILURE
        will be returned.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.
        The file into which a character has to be written, has to be present and
        should have been opened.


    Parameters:
        handle         -       Handle for the file received when the file was opened.
        data           -       A character to be written to the file.


    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
        None.
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileCharacterPut( SYS_FS_HANDLE handle, char data )
{
   int fileStatus = SYS_FS_ERROR_NOT_READY;
   SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
   int res = 0;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    if(obj->mountPoint->fsFunctions->putchr == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        res = obj->mountPoint->fsFunctions->putchr(data, obj->nativeFSFileObj);

        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(res == 1)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FileStringPut( SYS_FS_HANDLE handle, const char *string )

    Summary:
        String write.

    Description:
        The function writes a string into a file.
        When the function succeeded, it returns SYS_FS_RES_SUCCESS.
        When the write operation is aborted due to disk full or any error a
        SYS_FS_RES_FAILURE is returned.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.
        The file into which a string has to be written, has to be present and
        should have been opened.


    Parameters:
        handle         -       Handle for the file received when the file was opened.
        string         -       Pointer to string which has to be written into file.


    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
        None.
***************************************************************************/
SYS_FS_RESULT SYS_FS_FileStringPut( SYS_FS_HANDLE handle, const char *string )
{
   int fileStatus = SYS_FS_ERROR_NOT_READY;
   SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
   int res = 0;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    if(obj->mountPoint->fsFunctions->putstrn == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        res = obj->mountPoint->fsFunctions->putstrn(string, obj->nativeFSFileObj);

        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(res == EOF)
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
    else
    {
        return SYS_FS_RES_SUCCESS;
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_FilePrintf( SYS_FS_HANDLE handle, const char *string, ... )

    Summary:
        Formatted string write.

    Description:
        The function writes a formatted string into a file.
        When the function succeeded, it returns SYS_FS_RES_SUCCESS.
        When the write operation is aborted due to disk full or any error a
        SYS_FS_RES_FAILURE is returned.

    Precondition:
        Atlesat, one disk has to be mounted before this funcion could be set.
        The file into which a string has to be written, has to be present and
        should have been opened.


    Parameters:
        handle         -       Handle for the file received when the file was opened.
        string         -       Pointer to string which has to be written into file.


    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
        None.
***************************************************************************/
SYS_FS_RESULT SYS_FS_FilePrintf( SYS_FS_HANDLE handle, const char *string, ... )
{
   int fileStatus = SYS_FS_ERROR_NOT_READY;
   SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
   int res;
   va_list ap;
   const char *ptr;


    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    if(obj->mountPoint->fsFunctions->formattedprint == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return -1;
    }

    va_start (ap, string);
    ptr = va_arg(ap, const char*);
    res = obj->mountPoint->fsFunctions->formattedprint(obj->nativeFSFileObj, string, ptr);
    va_end (ap);

    if(res == EOF)
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
    else
    {
        return SYS_FS_RES_SUCCESS;
    }
}

//******************************************************************************
/*Function:
    bool SYS_FS_FileTestError(SYS_FS_HANDLE handle);

    Summary:
        Check for errors in the file

    Description:
        Checks whether or not the file has any errors.

    Precondition:
        A valid file handle must be obtained before passing to the function

    Parameters:
        handle 	- file handle obtaind during file Open.

    Returns:
		If Success	-

			When file has an error	- true

			When file has no errors	- false

                        If Failure		- true
  Remarks:
	None
***************************************************************************/
bool SYS_FS_FileTestError(SYS_FS_HANDLE handle)
{
    SYS_FS_OBJ *obj = (SYS_FS_OBJ *)handle;
    volatile bool status = false;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return true;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return true;
    }

    if(obj->mountPoint->fsFunctions->testerror == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return true;
    }

    status = obj->mountPoint->fsFunctions->testerror(obj->nativeFSFileObj);

    /* This is success value, but required in special case, where requested offset was (-1) */
    obj->errorValue = SYS_FS_ERROR_OK;

    return (bool)status;
}

//******************************************************************************
/*Function:
        SYS_FS_RESULT SYS_FS_DriveFormat(const char* drive, SYS_FS_FORMAT fmt, uint32_t clusterSize)

    Summary:
        Format a drive

    Description:
        Formats a logic drive (create a FAT file system on the logical drive), as per the format specified.

        If the logical drive that has to be formatted has been bound to any partition (1-4) by
        multiple partition feature, the FAT volume is created into the specified partition. In
        this case, the second argument fmt is ignored. The physical drive must have been
        partitioned prior to use this function.

    Precondition:
        Atlesat, one drive has to be mounted before this funcion could be set.


    Parameters:
        drive           -       Pointer to buffer which will hold the name of
                                drive being for which, the label is requested. If
                                this string is NULL, then then label of the current
                                drive is obtained by using this function.It is important
                                to end the drive name with a "/".
        fmt             -       Format into which the disk has to be formatted. It could
                                be of the type SYS_FS_FORMAT.
        clusterSize     -       Cluster size. The value must be sector (size * n), where
                                n is 1 to 128 and power of 2. When a zero is given, the
                                cluster size is determined depends on the volume size.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_DriveFormat(const char* drive, SYS_FS_FORMAT fmt, uint32_t clusterSize)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;

    /* Get disk number */
    if(_SYS_FS_DiskGet(drive, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */
    if(disk->fsFunctions->formatDisk == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return (SYS_FS_RES_FAILURE);
    }

    fileStatus = disk->fsFunctions->formatDisk((uint8_t)disk->diskNumber, fmt, clusterSize);


    if(fileStatus == 0)
    {
        return(SYS_FS_RES_SUCCESS);
    }
    else
    {
        errorValue = fileStatus;
        return(SYS_FS_RES_FAILURE);
    }
}

//******************************************************************************
/* Function:
	SYS_FS_HANDLE SYS_FS_DirOpen(const char* path)

  Summary:
     Open a directory

  Description:
	The SYS_FS_DirOpen opens the requested directory.

  Precondition:
    Prior to opening a file, the name of the volume on which the
	directory resides should be known. Also, that volume should be already mounted.

  Parameters:
    path      	     			- Path to the directory along with the volume name. The string of
                                          volume and directory name has to be preceeded by "/mnt/". Also,
                                          the volume name and directory name has to be separated by a
                                          slash "/".

  Returns:
	If Success					- Valid handle will be returned

	If Failure 					- Returned handle will be SYS_FS_HANDLE_INVALID

	The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
*/

SYS_FS_HANDLE SYS_FS_DirOpen(const char* path)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    uint8_t pathWithDiskNo[SYS_FS_PATH_LEN_WITH_DISK_NUM] = {};
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;
    uint32_t j = 0;
    SYS_FS_DIR_OBJ * obj = (SYS_FS_DIR_OBJ *) NULL;

    /* Get disk number */
    if(_SYS_FS_DiskGet(path, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_HANDLE_INVALID;
    }

    /* Now, get the file name with disk number appended in front like this "0:file.txt" */
    if (_SYS_FS_DiskNumberAppend(path, (uint8_t)disk->diskNumber, pathWithDiskNo) == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_NAME;
        return SYS_FS_HANDLE_INVALID;
    }

    obj = NULL;

    if(OSAL_MUTEX_Lock((OSAL_MUTEX_HANDLE_TYPE *)&mutexDirObj, OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* The data should be protected from simulateous access */
        for(j = 0; j < SYS_FS_MAX_FILES; j ++)
        {
            if(gSYSFSDirObj[j].inUse == false)
            {
                gSYSFSDirObj[j].inUse = true;
                gSYSFSDirObj[j].mountPoint = disk;
                obj = &gSYSFSDirObj[j];
                break;
            }
        }
        OSAL_MUTEX_Unlock(&mutexDirObj);
    }

    /* If the object is NULL, then we dont have a free
     * file system object */

    if(obj == NULL)
    {
        errorValue = SYS_FS_ERROR_TOO_MANY_OPEN_FILES;
        return(SYS_FS_HANDLE_INVALID);
    }

    /* Now, call the real file open function */
    if(disk->fsFunctions->openDir == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        obj->inUse = false;
        return (SYS_FS_HANDLE_INVALID);
    }

    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Convert the SYS_FS file open attributes to FAT FS attributes */
        fileStatus =disk->fsFunctions->openDir((uintptr_t)&obj->nativeFSDirObj,
                                                        (const char *)pathWithDiskNo);

        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return((SYS_FS_HANDLE)obj);
    }
    else
    {
        errorValue = fileStatus;
        obj->inUse = false;
        return(SYS_FS_HANDLE_INVALID);
    }
}

//******************************************************************************
/* Function:
	SYS_FS_RESULT SYS_FS_DirRead(SYS_FS_HANDLE handle, SYS_FS_FSTAT stat)


  Summary:
     Read specified directory

  Description:
	The SYS_FS_DirRead() function shall attempt to read the files and directories
        specified in the open directory.

  Precondition:
    A valid directory handle must be obtained before reading a directory.

  Parameters:
        handle			- directory handle obtained during directory open.

        stat                    - Empty structure of type SYS_FS_FSTAT, where the properties
                                  of the open directory will be populated after the SYS_FS_DirRead()
                                  function returns successfully.
                                  If LFN is used, then the "lfname" member of the
                                  SYS_FS_FSTAT structure should be initialized with address of
                                  suitable buffer. Also, the "lfsize" should be initialized with
                                  the size of buffer.
                                  Once the function returns, the buffer whose address is held in "lfname"
                                  will have the file name (long file name).



  Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

	The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
*/

SYS_FS_RESULT SYS_FS_DirRead(SYS_FS_HANDLE handle, SYS_FS_FSTAT *stat)
{
    int fileStatus = -1;
    SYS_FS_DIR_OBJ *obj = (SYS_FS_DIR_OBJ *)handle;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    if(obj->mountPoint->fsFunctions->readDir == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        fileStatus = obj->mountPoint->fsFunctions->readDir(obj->nativeFSDirObj, (uintptr_t)stat);

        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }

    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/* Function:
	SYS_FS_RESULT SYS_FS_DirRewind(SYS_FS_HANDLE handle)


  Summary:
     Read specified directory

  Description:
	The SYS_FS_DirRewind() function shall attempt to rewind the directory to the
        start. Once a seach or directory read is completed, the rewind function is
        used to begin searching the directory from the start.

  Precondition:
    A valid directory handle must be obtained before reading a directory.

  Parameters:
        handle			- directory handle obtained during directory open.

  Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

	The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
*/

SYS_FS_RESULT SYS_FS_DirRewind(SYS_FS_HANDLE handle)
{
    int fileStatus = -1;
    SYS_FS_DIR_OBJ *obj = (SYS_FS_DIR_OBJ *)handle;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == 0)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    if(obj->mountPoint->fsFunctions->readDir == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
        if(OSAL_MUTEX_Lock(&(obj->mountPoint->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {

        fileStatus = obj->mountPoint->fsFunctions->readDir(obj->nativeFSDirObj, (uintptr_t)NULL);
        OSAL_MUTEX_Unlock(&(obj->mountPoint->mutexDiskVolume));
    }


    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/* Function:
	SYS_FS_RESULT SYS_FS_DirSearch(SYS_FS_HANDLE handle, const char * name, SYS_FS_FILE_DIR_ATTR attr, SYS_FS_FSTAT *stat)


  Summary:
        Find a file/ directory

  Description:
	Find a file or directory as specified by the name. The type of name is specified
        by attr variable, which if of type SYS_FS_FILE_DIR_ATTR.

  Precondition:
    A valid directory handle must be obtained before reading a directory.

  Parameters:
        handle			- directory handle obtained during directory open.

        name                    - name of file or directory needed to be opened.
                                  The file name can have wild card entries as mentioned below: -
                                - * - Indicates the rest of the filename or extension can vary (e.g. FILE.*)
                                - ? - Indicates that one character in a filename can vary (e.g. F?LE.T?T)

        attr                    - Attribute of the name of type SYS_FS_FILE_DIR_ATTR. This
                                  attribute specifies whether the name is for a file or a
                                  directory. Also, other attribute types could be spcified.

        stat                    - Empty structure of type SYS_FS_FSTAT, where the properties
                                  of the open directory will be populated after the SYS_FS_DirRead()
                                  function returns successfully.
                                  If LFN is used, then the "lfname" member of the
                                  SYS_FS_FSTAT structure should be initialized with address of
                                  suitable buffer. Also, the "lfsize" should be initialized with
                                  the size of buffer.
                                  Once the function returns, the buffer whose address is held in "lfname"
                                  will have the file name (long file name)



  Returns:
        If spcified file/ directory found	-	SYS_FS_RES_SUCCESS

        If spcified file/ directory not found	-	SYS_FS_RES_FAILURE

	The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
*/

SYS_FS_RESULT SYS_FS_DirSearch(SYS_FS_HANDLE handle, const char * name, SYS_FS_FILE_DIR_ATTR attr, SYS_FS_FSTAT *stat)
{
    int fileStatus = -1;

    while(1)
    {
        fileStatus = SYS_FS_DirRead(handle, stat);

        if(fileStatus != 0)
        {
            errorValue = fileStatus;
            return SYS_FS_RES_FAILURE;
        }
        /* If we have come to end of directory */
        if((*stat->lfname == '\0') && (*stat->fname == '\0'))
        {
            errorValue = SYS_FS_ERROR_NO_FILE;
            return SYS_FS_RES_FAILURE;
        }

        /* Firstly, match the file attribute with the requested attribute */
        if(stat->fattrib & attr)
        {
            if(*stat->lfname != '\0')
            {
                /* File name is LFN */
                if(_SYS_FS_StringWildCardCompare(name, stat->lfname) == true)
                {
                    return SYS_FS_RES_SUCCESS;
                }
            }
            else
            {
                /* File name fits in 8.3 format */
                if(_SYS_FS_StringWildCardCompare(name, stat->fname) == true)
                {
                    return SYS_FS_RES_SUCCESS;
                }
            }
        }
    }

    errorValue = SYS_FS_ERROR_NO_FILE;
    return SYS_FS_RES_FAILURE;
}

//******************************************************************************
/* Function:
	SYS_FS_RESULT SYS_FS_DirClose(SYS_FS_HANDLE handle)


  Summary:
     Close an opened directory

  Description:
	The SYS_FS_DirClose() function closes a directory which was earlier opened
        with the function SYS_FS_DirOpen(). Internally, this function just clears
        the allocated objects, so that they can be reused.

  Precondition:
    A valid directory handle must be obtained before closing the directory.

  Parameters:
        handle			- directory handle obtained during directory open.

  Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

	The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
*/

SYS_FS_RESULT SYS_FS_DirClose(SYS_FS_HANDLE handle)
{
    int fileStatus = -1;
    SYS_FS_DIR_OBJ *obj = (SYS_FS_DIR_OBJ *)handle;

    if(handle == SYS_FS_HANDLE_INVALID)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        return SYS_FS_RES_FAILURE;
    }

    if(obj->inUse == false)
    {
        errorValue = SYS_FS_ERROR_INVALID_OBJECT;
        SYS_ASSERT(false,"File object is not in use");
        return SYS_FS_RES_FAILURE;
    }

    if(obj->mountPoint->fsFunctions->closeDir == NULL)
    {
        obj->errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */

    fileStatus = obj->mountPoint->fsFunctions->closeDir(obj->nativeFSDirObj);
    if(fileStatus == 0)
    {
        /* Return the SYS_FS file system object. */
        obj->inUse = false;
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        obj->errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

//******************************************************************************
/* Function:
    SYS_FS_RESULT SYS_FS_DrivePartition(const char *path, const uint32_t partition[], void *work)

  Summary:
    Partitions a physical drive (media).

  Description:
    This function partitions a physical drive (media) into requested partition sizes. This
    function will alter the MBR of the physical drive and make it into multi paritions.
    Windows operating systems do not support multi paritioned removable media.

    Maximum 4 partitions can be created on a media.

  Precondition:
    Prior to partitioning the media, the media should have a valid MBR and it should be
    mounted as a volume with the file system.

  Parameters:
    path      	     			- Path to the volume with the volume name. The string of
                                          volume name has to be preceeded by "/mnt/". Also,
                                          the volume name and directory name has to be separated by a
                                          slash "/".
    partition[]                         - Array with 4 items, where each items mentions the sizes
                                          of each parition in terms of number of sector. 0th element of
                                          array specifies the number of sectors for first partition and
                                          3rd element of array specifies the number of sectors for fourth
                                          partition.

    work                                - Pointer to the buffer for function work area. The size must be
                                          at least FAT_FS_MAX_SS bytes.

  Returns:
	If Success                      - SYS_FS_RES_SUCCESS

	If Failure 			- SYS_FS_RES_FAILURE

	The reason for failure could be retrieved with SYS_FS_Error

  Remarks:
	None
*/

SYS_FS_RESULT SYS_FS_DrivePartition(const char *path, const uint32_t partition[], void *work)
{
    int fileStatus = SYS_FS_ERROR_NOT_READY;
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;

    /* Get disk number */
    if(_SYS_FS_DiskGet(path, &disk) == SYS_FS_RES_FAILURE)
    {
        // reason or cause of error is alredy present in "errorValue" variable
        return SYS_FS_RES_FAILURE;
    }

    /* For MPFS file system, opening a file is possible only in "READ" mode */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Now, call the real file open function */
    if(disk->fsFunctions->partitionDisk == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return (SYS_FS_RES_FAILURE);
    }

    fileStatus = disk->fsFunctions->partitionDisk((uint8_t)disk->diskNumber, partition, work);


    if(fileStatus == 0)
    {
        return(SYS_FS_RES_SUCCESS);
    }
    else
    {
        errorValue = fileStatus;
        return(SYS_FS_RES_FAILURE);
    }
}

//******************************************************************************
/*Function:
    SYS_FS_RESULT SYS_FS_DriveSectorGet(const char* path, uint32_t *totalSectors, uint32_t *freeSectors)

    Summary:
        Obtain total and free sectors

    Description:
        Function to obtain the total number of sectors and number of free sectors in a
        drive (media).

    Precondition:
        Atlesat, one drive has to be mounted before this funcion could be set.

    Parameters:
        path      	     		- Path to the volume with the volume name. The string of
                                          volume name has to be preceeded by "/mnt/". Also,
                                          the volume name and directory name has to be separated by a
                                          slash "/".

        totalSectors                    - Pointer to a variable passed to the
                                          function, which will contain the total
                                          number of sectors available in the
                                          drive (media). This data will be
                                          available only if the function
                                          returns successfully.

        freeSectors                     - Pointer to a variable passed to the
                                          function, which will contain the free
                                          number of sectors available in the
                                          drive (media). This data will be
                                          available only if the function
                                          returns successfully.

    Returns:
        If Success	-	SYS_FS_RES_SUCCESS

        If Failure	-	SYS_FS_RES_FAILURE

		The reason for failure could be retrieved with SYS_FS_Error
  Remarks:
	None
***************************************************************************/
SYS_FS_RESULT SYS_FS_DriveSectorGet(const char* path, uint32_t *totalSectors, uint32_t *freeSectors)
{
    volatile int fileStatus = SYS_FS_ERROR_NOT_READY;
    SYS_FS_MOUNT_POINT *disk = (SYS_FS_MOUNT_POINT *) NULL;
    uint8_t pathWithDiskNo[3] = {};

    if(path != NULL)
    {
        /* Get disk number */
        if(_SYS_FS_DiskGet(path, &disk) == SYS_FS_RES_FAILURE)
        {
            // reason or cause of error is alredy present in "errorValue" variable
            return SYS_FS_RES_FAILURE;
        }
    }
    else    /* if(drive == NULL */
    {
        if(gSYSFSCurrentMountPoint.inUse == false)
        {
            SYS_ASSERT(false, "Invalid mount point. Was the disk mounted?");
            errorValue = SYS_FS_ERROR_NO_FILESYSTEM;
            return SYS_FS_RES_FAILURE;
        }

        disk = gSYSFSCurrentMountPoint.currentDisk;

    }

    /* For MPFS file system, label name is not supported */
    if(disk->fsType == MPFS2)
    {
        errorValue = SYS_FS_ERROR_DENIED;
        return SYS_FS_RES_FAILURE;
    }

    /* Append "0:" before the file name. This is required
     * for different disks */
    pathWithDiskNo[0] = (uint8_t)disk->diskNumber + '0';
    pathWithDiskNo[1] = ':';
    pathWithDiskNo[2] = '\0';

    if(disk->fsFunctions->getCluster == NULL)
    {
        errorValue = SYS_FS_ERROR_NOT_SUPPORTED_IN_NATIVE_FS;
        return SYS_FS_RES_FAILURE;
    }
    if(OSAL_MUTEX_Lock(&(disk->mutexDiskVolume), OSAL_WAIT_FOREVER)
                                                        == OSAL_RESULT_TRUE)
    {

       fileStatus = disk->fsFunctions->getCluster((const char *)pathWithDiskNo, totalSectors, freeSectors);
       OSAL_MUTEX_Unlock(&(disk->mutexDiskVolume));
    }


    if(fileStatus == 0)
    {
        return SYS_FS_RES_SUCCESS;
    }
    else
    {
        errorValue = fileStatus;
        return SYS_FS_RES_FAILURE;
    }
}

/*************************************************************************
*
* END OF sys_fs.c
***************************************************************************/
