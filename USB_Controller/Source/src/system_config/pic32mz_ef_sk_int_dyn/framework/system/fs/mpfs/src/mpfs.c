/*******************************************************************************
  Microchip File System (MPFS)

  Company:
    Microchip Technology Inc.

  File Name:
    sys_fs_mpfs.c

  Summary:
    Microchip File System (MPFS) APIs.

  Description:
    This file contains Microchip File System (MPFS) APIs. It is mainly used for
	accessing web pages and other files from internal program memory or an
	external serial EEPROM memory.
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system/fs/mpfs/src/mpfs_local.h"


/****************************************************************************
  Section: Module-Only Globals and Functions
  ***************************************************************************/

/* Array of File Objects. */
static MPFS_FILE_OBJ gSysMpfsFileObj[SYS_FS_MAX_FILES];

/* Current File Record */
static MPFS_FILE_RECORD gSysMpfsFileRecord;

/* MPFS Object. */
static SYS_MPFS_OBJECT gSysMpfsObj = 
{
    0xFF, /* Disk Number. */

    0, /* Number of files. */

    0, /* Base address. */

    0, /* Current file index of the directory. */

    MPFS_INVALID_HANDLE, /* Current File Handle. */

    MPFS_INVALID_HANDLE, /* Directory Handle. */
};

/* MPFS Handle Token. */
uint8_t gSysMpfsHandleToken = 0;
/****************************************************************************
 Function pointers
*****************************************************************************/
const SYS_FS_FUNCTIONS MPFSFunctions =
{
    .mount  = MPFS_Mount,
    .unmount = MPFS_Unmount,
    .open   = MPFS_Open,
    .read   = MPFS_Read,
    .write  = NULL,
    .close  = MPFS_Close,
    .seek   = MPFS_Seek,
    .tell   = MPFS_GetPosition,
    .eof    = MPFS_EOF,
    .size   = MPFS_GetSize,
    .fstat   = MPFS_Stat,
    .mkdir = NULL,
    .chdir = NULL,
    .remove = NULL,
    .getlabel = NULL,
    .setlabel = NULL,
    .truncate = NULL,
    .currWD = NULL,
    .chdrive = NULL,
    .chmode = NULL,
    .chtime = NULL,
    .rename = NULL,
    .sync = NULL,
    .getstrn = NULL,
    .putchr = NULL,
    .putstrn = NULL,
    .formattedprint = NULL,
    .testerror = NULL,
    .formatDisk = NULL,
    .openDir = MPFS_DirOpen,
    .readDir = MPFS_DirRead,
    .closeDir = MPFS_DirClose,
    .partitionDisk = NULL,
    .getCluster = NULL
};

/****************************************************************************
  Section:Handle Management Functions
*****************************************************************************/

static bool MPFSGetArray
(
    uint8_t diskNum,
    uint32_t address,
    uint32_t length,
    uint8_t *buffer
);

/* This function checks if the MPFS handle is valid or not. */
static bool MPFSIsHandleValid
(
    MPFS_HANDLE handle
)
{
    uint16_t index = handle & 0xFFFF;
    if (index > SYS_FS_MAX_FILES)
    {
        return false;
    }

    if (gSysMpfsFileObj[index].handle != handle)
    {
        return false;
    }

    return true;
}

/* This function fetches the file record information for the requested handle.
 * */
static bool MPFSGetFileRecord
(
    MPFS_HANDLE handle
)
{
    uint16_t index = 0;
    uint8_t diskNum = 0;
    uint32_t address = 0;
    bool status = false;

    if (gSysMpfsObj.currentHandle != handle)
    {
        index = handle & 0xFFFF;
        diskNum = ((handle >> 16) & 0xFF);

        /* Read the FAT record to the cache */
        address = 8 + (gSysMpfsObj.numFiles * 2) + (gSysMpfsFileObj[index].hashIndex * 22);
        if (MPFSGetArray (diskNum, address, 22, (uint8_t*)&gSysMpfsFileRecord) == true)
        {
            /* Update the handle. */
            gSysMpfsObj.currentHandle = handle;

            status = true;
        }
    }
    else
    {
        status = true;
    }

    return status;
}

/* This function searches the MPFS2 Image for the file requested and returns
 * the index associated with the file and also the updates the current file
 * record with the file information. */
static int MPFSFindFile
(
    uint8_t diskNum,
    uint8_t *file,
    MPFS_FILE_RECORD *fileRecord
)
{
    uint32_t address = 0;
    uint32_t temp = 0;
    uint8_t *ptr = NULL;
    int32_t index = 0;
    uint16_t hash = 0;
    uint16_t hashBuffer[16];
    uint8_t fileName[FAT_FS_MAX_LFN];

    /* Calculate the hash value for the file name. */
    ptr = file;
    while (*ptr != '\0')
    {
        hash += *ptr++;
        hash <<= 1;
    }

    /* Read in hashes, and check remainder on a match. Store 8 in cache for
     * performance. */
    for (index = 0; index < gSysMpfsObj.numFiles; index++)
    {
        if ((index & 0x07) == 0u)
        {
            /* Address of the hash location. */
            address = 8 + (index << 1);
            temp = gSysMpfsObj.numFiles - index;
            if (temp > 8)
            {
                temp = 8;
            }

            if (MPFSGetArray (diskNum, address, temp << 1, (uint8_t *)hashBuffer) == false)
            {
                return -1;
            }
        }

        /* If the hash matches, compare the full filename. */
        if (hashBuffer[index & 0x07] == hash)
        {
            address = 8 + (gSysMpfsObj.numFiles * 2) + (index * 22);
            MPFSGetArray (diskNum, address, 22, (uint8_t *)fileRecord);
            temp = strlen ((const char *)file);

            if (MPFSGetArray (diskNum, fileRecord->fileNameOffset, temp, (uint8_t *)fileName) == false)
            {
                return -1;
            }

            if (strncmp ((const char *)fileName, (const char *)file, temp) == 0)
            {
                /* Found the matching file. */
                return index;
            }
        }
    }

    return -1;
}

static bool MPFSDiskRead
(
    uint16_t diskNum,
    uint8_t *destination,
    uint8_t *source,
    const uint32_t nBytes
)
{
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE commandHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    SYS_FS_MEDIA_COMMAND_STATUS commandStatus = SYS_FS_MEDIA_COMMAND_UNKNOWN;

    commandHandle = SYS_FS_MEDIA_MANAGER_Read (diskNum, destination, source, nBytes);
    if (commandHandle == SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID)
    {
        return false;
    }

    do 
    {
        SYS_FS_MEDIA_MANAGER_TransferTask (diskNum);
        commandStatus = SYS_FS_MEDIA_MANAGER_CommandStatusGet(diskNum, commandHandle);
    } while (commandStatus == SYS_FS_MEDIA_COMMAND_IN_PROGRESS);

    return (commandStatus == SYS_FS_MEDIA_COMMAND_COMPLETED) ? true : false;
}

/* Wrapper for the MPFSDiskRead (). */
static bool MPFSGetArray
(
    uint8_t diskNum,
    uint32_t address,
    uint32_t length,
    uint8_t *buffer
)
{
    return MPFSDiskRead (diskNum, buffer, ((uint8_t *)gSysMpfsObj.baseAddress + address), length);
}

/* This function mounts the MPFS2 File system. */
int MPFS_Mount
(
    uint8_t diskNo
)
{
    uint32_t index = 0;

    if (diskNo > SYS_FS_VOLUME_NUMBER)
    {
        return MPFS_DISK_ERR;
    }

    if (gSysMpfsObj.diskNum != 0xFF)
    {
        /* Mount operation has already been done. */
        return MPFS_OK;
    }

    /* Initialize the MPFS Object members. */
    gSysMpfsObj.numFiles = 0;
    gSysMpfsObj.currentHandle = MPFS_INVALID_HANDLE;

    /* Find the base address of the MPFS2 Image. */
    gSysMpfsObj.baseAddress = SYS_FS_MEDIA_MANAGER_AddressGet(diskNo);

    for (index = 0; index < SYS_FS_MAX_FILES; index++)
    {
        gSysMpfsFileObj[index].currentOffset = MPFS_INVALID;
        gSysMpfsFileObj[index].bytesRemaining = 0;
    }

    /* Read the MPFS2 Image signature and the version information. */
    if (MPFSGetArray(diskNo, 0, 6, (uint8_t*)&gSysMpfsFileRecord) == false)
    {
        return MPFS_DISK_ERR;
    }

    if (memcmp((void*)&gSysMpfsFileRecord, (const void*)"MPFS\x02\x01", 6) != 0)
    {
        return MPFS_DISK_ERR;
    }

    if (MPFSGetArray(diskNo, 6, 2, (uint8_t*)&gSysMpfsObj.numFiles) == false)
    {
        return MPFS_DISK_ERR;
    }

    /* Store the disk number. */
    gSysMpfsObj.diskNum = diskNo;

    return MPFS_OK;
}

/* Unmounts the MPFS2 file system. */
int MPFS_Unmount
(
    uint8_t diskNo
)
{
    if ((diskNo > SYS_FS_VOLUME_NUMBER) || (diskNo != gSysMpfsObj.diskNum))
    {
        return MPFS_DISK_ERR;
    }

    gSysMpfsObj.numFiles = 0;
    gSysMpfsObj.currentHandle = MPFS_INVALID_HANDLE;
    gSysMpfsObj.diskNum = 0xFF;

    return MPFS_OK;
}

/* Opens a file in the MPFS2 file system. */
int MPFS_Open
(
    uintptr_t handle,
    const char* filewithDisk,
    uint8_t mode
)
{
    int32_t index = 0;
    int32_t hashIndex = 0;
    uint8_t diskNum = 0;
    diskNum = filewithDisk[0] - '0';

    if ((diskNum > SYS_FS_VOLUME_NUMBER) || (diskNum != gSysMpfsObj.diskNum))
    {
        return MPFS_INVALID_PARAMETER;
    }

    /* Find a free file object. */
    for (index = 0; index < SYS_FS_MAX_FILES; index++)
    {
        if (gSysMpfsFileObj[index].currentOffset == MPFS_INVALID)
        {
            break;
        }
    }

    if (index > SYS_FS_MAX_FILES)
    {
        return MPFS_INVALID_PARAMETER;
    }

    hashIndex = MPFSFindFile (diskNum, (uint8_t *)(filewithDisk + 3), &gSysMpfsFileRecord);
    if (hashIndex >= 0)
    {
        /* Found the file. */
        gSysMpfsFileObj[index].currentOffset = gSysMpfsFileRecord.dataOffset;
        gSysMpfsFileObj[index].bytesRemaining = gSysMpfsFileRecord.length;
        gSysMpfsFileObj[index].hashIndex = hashIndex;
        gSysMpfsFileObj[index].handle = MPFS_MAKE_HANDLE(gSysMpfsHandleToken, diskNum, index);
        MPFS_UPDATE_HANDLE_TOKEN(gSysMpfsHandleToken);
        *(uintptr_t *)handle = gSysMpfsFileObj[index].handle;
        gSysMpfsObj.currentHandle = gSysMpfsFileObj[index].handle;

        /* Return the status */
        return MPFS_OK;
    }

    return MPFS_NO_FILE;
}

/* This function reads data from the specified file. */
int MPFS_Read
(
    uintptr_t handle,
    void* buff, 
    uint32_t btr, 
    uint32_t *br
)
{
    uint16_t index = 0;
    uint8_t diskNum = 0;

    *br = 0;
    index = handle & 0xFFFF;
    if (index > SYS_FS_MAX_FILES)
    {
        return MPFS_INVALID_PARAMETER;
    }

    if (gSysMpfsFileObj[index].handle != handle)
    {
        return MPFS_INVALID_PARAMETER;
    }

    /* Extract the disk number from the handle. */
    diskNum = ((handle >> 16) & 0xFF);

    /* Find the number of bytes to read. */
    if (btr > gSysMpfsFileObj[index].bytesRemaining)
    {
        btr = gSysMpfsFileObj[index].bytesRemaining;
    }

    if (btr > 0)
    {
        if (buff == 0)
        {
            *br = btr;

            /* Update the current address offset and the bytes remaining offset. */
            gSysMpfsFileObj[index].currentOffset += *br;
            gSysMpfsFileObj[index].bytesRemaining -= *br;
        }
        else
        {
            if (MPFSGetArray (diskNum, gSysMpfsFileObj[index].currentOffset, btr, buff) == true)
            {
                *br = btr;

                /* Update the current address offset and the bytes remaining offset. */
                gSysMpfsFileObj[index].currentOffset += *br;
                gSysMpfsFileObj[index].bytesRemaining -= *br;
            }
        }
    }

    return MPFS_OK;
}


/* Closes a file in the MPFS2 file system. */
int MPFS_Close
(
    uintptr_t handle
)
{
    uint16_t index = 0;
    index = handle & 0xFFFF;
    if (index > SYS_FS_MAX_FILES)
    {
        return MPFS_INVALID_PARAMETER;
    }

    if (gSysMpfsFileObj[index].handle != handle)
    {
        return MPFS_INVALID_PARAMETER;
    }

	gSysMpfsFileObj[index].currentOffset = MPFS_INVALID;
	gSysMpfsFileObj[index].bytesRemaining = 0;
	gSysMpfsFileObj[index].handle = MPFS_INVALID_HANDLE;
    return MPFS_OK;
}


/* This function returns the size of the file associated with the handle. */
uint32_t MPFS_GetSize
(
    uintptr_t handle
)
{
    if (MPFSIsHandleValid(handle) == false)
    {
        /* Invalid handle. */
        return 0;
    }

    MPFSGetFileRecord (handle);

	return gSysMpfsFileRecord.length;
}

/* This function returns the current file position. */
uint32_t MPFS_GetPosition
(
    uintptr_t handle
)
{
    uint16_t index = 0;
    if (MPFSIsHandleValid(handle) == false)
    {
        /* Invalid handle. */
        return 0;
    }

    MPFSGetFileRecord (handle);

    index = handle & 0xFFFF;
    return (uint32_t)(gSysMpfsFileObj[index].currentOffset - gSysMpfsFileRecord.dataOffset);
}

/* Returns if the present file pointer already reached end of file */
bool MPFS_EOF
(
    uintptr_t handle
)
{
    uint16_t index = 0;

    if (MPFSIsHandleValid(handle) == false)
    {
        /* Invalid handle. */
        return false;
    }

    MPFSGetFileRecord (handle);

    index = handle & 0xFFFF;

    if ((gSysMpfsFileObj[index].currentOffset - gSysMpfsFileRecord.dataOffset) == gSysMpfsFileRecord.length)
    {
        return true;
    }

    return false;
}

/* Returns the File statistics. */
int MPFS_Stat
(
    const char* filewithDisk,
    uintptr_t stat_str
)
{
    /* Take the file name without the disk number (ignore "0:/"). */
    const char* file = filewithDisk + 3;
    uint16_t fileLen = 0;
    uint8_t diskNum = 0;
    MPFS_FILE_RECORD fileRecord;

    MPFS_STATUS *stat = (MPFS_STATUS *)stat_str;

    diskNum = filewithDisk[0] - '0';

    if ((diskNum > SYS_FS_VOLUME_NUMBER) || (diskNum != gSysMpfsObj.diskNum))
    {
        return MPFS_INVALID_PARAMETER;
    }

    if (MPFSFindFile (diskNum, (uint8_t *)file, &fileRecord) >= 0)
    {
        fileLen = strlen (file);

#if FAT_FS_USE_LFN
        if (stat->lfname && stat->lfsize)
        {
            if (fileLen > stat->lfsize)
            {
                stat->lfname[0] = '\0';
            }
            else
            {
                /* Populate the file details. */
                strncpy (stat->lfname, file, fileLen);
                stat->lfname[fileLen] = '\0';
            }
        }
#endif

        /* Check if the name of the file is longer than the SFN 8.3 format. */
        if (fileLen > 12)
        {
            fileLen = 12;
        }

        /* Populate the file details. */
        strncpy (stat->fname, file, fileLen);
        stat->fname[fileLen] = '\0';

        stat->fattrib = fileRecord.flags;
        stat->fdate = (unsigned short)(fileRecord.timestamp >> 16);
        stat->ftime = (unsigned short)(fileRecord.timestamp);
        stat->fsize = fileRecord.length;

        return MPFS_OK;
    }

    return MPFS_NO_FILE;
}

int MPFS_Seek
(
    uintptr_t handle, 
    uint32_t dwOffset
)
{
    uint16_t index = 0;

    if (MPFSIsHandleValid (handle) == false)
    {
        /* Invalid handle. */
        return 1;
    }

    MPFSGetFileRecord (handle);

    if (dwOffset > gSysMpfsFileRecord.length)
    {
        return 1;
    }
    
    index = handle & 0xFFFF;

    gSysMpfsFileObj[index].currentOffset = gSysMpfsFileRecord.dataOffset + dwOffset;
    gSysMpfsFileObj[index].bytesRemaining = gSysMpfsFileRecord.length - dwOffset;

    return 0;
}

int MPFS_DirOpen
(
	uintptr_t handle,  /* Pointer to directory object to create */
	const char *path  /* Pointer to the directory path */
)
{
    uint8_t diskNum = 0;
    if (path == NULL)
    {
        return MPFS_INVALID_PARAMETER;
    }

    diskNum = path[0] - '0';

    if ((diskNum > SYS_FS_VOLUME_NUMBER) || (diskNum != gSysMpfsObj.diskNum))
    {
        return MPFS_INVALID_PARAMETER;
    }

    if (gSysMpfsObj.dirHandle != MPFS_INVALID_HANDLE)
    {
        /* The directory is already open. */
        return MPFS_DENIED;
    }

    gSysMpfsObj.dirHandle = MPFS_MAKE_HANDLE(gSysMpfsHandleToken, diskNum, 0);
    MPFS_UPDATE_HANDLE_TOKEN(gSysMpfsHandleToken);
    *(uintptr_t *)handle = gSysMpfsObj.dirHandle;

    /* Set the file index for the directory to 0. */
    gSysMpfsObj.fileIndex = 0;

    return MPFS_OK;
}

int MPFS_DirClose
(
	uintptr_t handle
)
{
    if ((handle == MPFS_INVALID_HANDLE) || (handle != gSysMpfsObj.dirHandle))
    {
        /* Invalid directory handle. */
        return MPFS_INVALID_PARAMETER;
    }

    gSysMpfsObj.dirHandle = MPFS_INVALID_HANDLE;
    /* Reset the file index for the directory to 0. */
    gSysMpfsObj.fileIndex = 0;

    return MPFS_OK;
}

int MPFS_DirRead
(
	uintptr_t handle,
	uintptr_t statPtr
)
{
    uint16_t fileLen = 0;
    uint8_t diskNum = 0;
    uint32_t address = 0;
    MPFS_FILE_RECORD fileRecord;
    uint8_t fileName[FAT_FS_MAX_LFN];

    MPFS_STATUS *stat = (MPFS_STATUS *)statPtr;

    /* Find the directory index and check if the handle is valid. */
    if ((handle == MPFS_INVALID_HANDLE) || (gSysMpfsObj.dirHandle != handle))
    {
        /* Invalid handle. */
        return MPFS_INVALID_PARAMETER;
    }

    if (stat == NULL)
    {
        /* Reset the directory file index. */
        gSysMpfsObj.fileIndex = 0;

        return MPFS_OK;
    }

    /* Fetch the disk number from the handle. */
    diskNum = ((handle >> 16) & 0xFF);

    /* Check if there are still files left to be read. */
    if (gSysMpfsObj.fileIndex != gSysMpfsObj.numFiles)
    {
        /* Fetch the file record. */
        address = 8 + (gSysMpfsObj.numFiles * 2) + (gSysMpfsObj.fileIndex * 22);
        if (MPFSGetArray (diskNum, address, 22, (uint8_t *)&fileRecord) == false)
        {
            /* Failed to fetch the file record. */
            return MPFS_DISK_ERR;
        }

        /* Since the length of the file is not known, fetch FAT_FS_MAX_LFN
         * number of bytes starting the from the file name offset. */
        if (MPFSGetArray (diskNum, fileRecord.fileNameOffset, FAT_FS_MAX_LFN, (uint8_t *)fileName) == false)
        {
            /* Failed to fetch the file name. */
            return MPFS_DISK_ERR;
        }

        fileLen = strlen ((const char *)fileName);

#if FAT_FS_USE_LFN
        if (stat->lfname && stat->lfsize)
        {
            if ((fileLen + 1) > stat->lfsize)
            {
                stat->lfname[0] = '\0';
            }
            else
            {
                /* Populate the file details. */
                strncpy (stat->lfname, (const char *)fileName, fileLen);
                stat->lfname[fileLen] = '\0';
            }
        }
#endif
        /* Check if the name of the file is longer than the SFN 8.3 format. */
        if (fileLen > 12)
        {
            fileLen = 12;
        }

        /* Populate the file details. */
        strncpy (stat->fname, (const char *)fileName, fileLen);
        stat->fname[fileLen] = '\0';

        stat->fattrib = fileRecord.flags;
        stat->fdate = (unsigned short)(fileRecord.timestamp >> 16);
        stat->ftime = (unsigned short)(fileRecord.timestamp);
        stat->fsize = fileRecord.length;

        /* Update the file index for the directory. */
        gSysMpfsObj.fileIndex ++;
    }
    else
    {
        /* Reached the end of the directory. Reset the directory file index. */
        gSysMpfsObj.fileIndex = 0;

        /* Set fname[0] and lfname[0](if LFN is enabled) to '\0' to indicate
         * the end of the directory condition. */
        stat->fname[0] = '\0';
#if FAT_FS_USE_LFN
        if (stat->lfname && stat->lfsize)
        {
            stat->lfname[0] = '\0';
        }
#endif
    }

    return MPFS_OK;
}

/* EOF mpfs.c */

