/*******************************************************************************
  File System System-Library Interface Implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    sys_fs_media_manager.c

  Summary:
    This file contains implementation of SYS FS Media Manager functions. 

  Description:
    This file contains implementation of SYS FS Media Manager functions. 
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is fsegrated fso your product or third party product (pursuant
to the sublicense terms in the accompanying license agreement).

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

#include "system/fs/src/sys_fs_media_manager_local.h"
#include "system/fs/src/sys_fs_local.h"

const char *gSYSFSVolumeName [] = {
    "nvm",
    "sd",
    "mmcblk",
    "ram",
    "mtd",
};

/**/
const uint16_t gPartitionTypeOffset [4] =
{
    450,
    466,
    482,
    498
};

// *****************************************************************************
/* Media object

  Summary:
    Defines the media object for each media that should be controlled by media manager

  Description:
    This data type defines the media objects that are available on
    the part.
  Remarks:
    None
*/

SYS_FS_MEDIA gSYSFSMediaObject[SYS_FS_MEDIA_NUMBER];

// *****************************************************************************
/* Volume object

  Summary:
    Defines the volume object for each media that should be controlled by media manager

  Description:
     Each partition of the media is assigned as a volume using the volume object.

  Remarks:
    None
*/
SYS_FS_VOLUME gSYSFSVolumeObject[SYS_FS_VOLUME_NUMBER];
// *****************************************************************************
/* Media Event Handler

  Summary:
    Defines the media Event handler that stores the application event handler.

  Description:
    This data type defines the media event handler
    the part.
  Remarks:
    None
*/
extern SYS_FS_EVENT_HANDLER gSYSFSEventHandler;

// *****************************************************************************
/* Media Page Buffer

  Summary:
    Defines the media page buffer to store the data on media.

  Description:
    This data type defines the media page buffer.
  Remarks:
    None
*/
uint8_t COHERENT_ALIGNED_16BIT gSYSFSMediaBlockBuffer[SYS_FS_MEDIA_MANAGER_BUFFER_SIZE] = {0};

// *****************************************************************************
/* Media Mount Table

  Summary:
    Defines the mount table specified by application.

  Description:
    This data type defines the mount table that maps device and mount name.
  Remarks:
    None
*/
extern const SYS_FS_MEDIA_MOUNT_DATA sysfsMountTable[];

// *****************************************************************************
/* Volume to Partition translation

  Summary:
    Defines the volume to partition translation table, used by FAT FS.

  Description:
 * The following array was added to enable the "multipartition"
 * feature of FAT FS. This array is already declared in ff.h and
 * the intention was to make as little change on ff.h. Also, type of this
 * array is non-standard since, it is defined in ff.c. The implementation
 * is kept unchanged to maintain compatibility with future releases of FAT FS

  Remarks:
    None
*/
PARTITION VolToPart[SYS_FS_VOLUME_NUMBER];

uint8_t gSYSFSMediaBuffer[SYS_FS_MEDIA_MAX_BLOCK_SIZE] COHERENT_ALIGNED_16BIT;

/* Following structure holds the variables for media manager, including the task states */
SYS_FS_MEDIA_MANAGER_OBJ gSYSFSMediaManagerObj =
{
    gSYSFSMediaObject,
    gSYSFSVolumeObject,
    sysfsMountTable,
    NULL,
    gSYSFSMediaBuffer,
    0,
    0,
    false
};

/*****************************************************************************
 * Function:
 * void _SYS_FS_MEDIA_MANAGER_updateVolToPart(uint8_t volNumber, uint8_t pd, uint8_t pt)
 *
 * Description:
 * This function is present to enable the multipartition operation
 * of FAT FS. FAT FS uses the structure VolToPart to know the physical
 * drive media number and partition number of that media, using this
 * structure. This function is used to populate the structure.
 * REFER TO MORE EXPLANATION on SYS_fs_media_manager_local.h
 *
 * Precondition:
 *   None
 *
 * Parameters:
 *   volNumber      - volume number
 *   pd             - physical drive number (media number)
 *   pt             - partition number from that media
 *
 * Returns:
 *   none.
*/

static void _SYS_FS_MEDIA_MANAGER_UpdateVolToPart
(
    uint8_t volNumber,
    uint8_t pd,
    uint8_t pt
)
{
    /* Update the "VolToPart" table for multipartition support on FAT FS */
    /* pd = Physical drive, starting from "zero". This is to be compatible with FAT FS code */
    VolToPart[volNumber].pd = pd;
    /* pt = partition, starting from "one". This is to be compatible with FAT FS code */
    VolToPart[volNumber].pt = pt;
}

static void _SYS_FS_MEDIA_MANAGER_HandleMediaDetach
(
    SYS_FS_MEDIA *mediaObj
)
{
    uint8_t volIndex = 0;
    uint8_t index = 0;
    SYS_FS_VOLUME *volumeObj = NULL;
    const SYS_FS_MEDIA_MOUNT_DATA *fsMount = (const SYS_FS_MEDIA_MOUNT_DATA *)&gSYSFSMediaManagerObj.fsMountTable[0];

    for (volIndex = 0; volIndex < SYS_FS_VOLUME_NUMBER; volIndex++)
    {
        volumeObj = &gSYSFSMediaManagerObj.volumeObj[volIndex];
        if ((volumeObj->inUse == false) || (volumeObj->obj != mediaObj))
        {
            continue;
        }

        volumeObj->inUse = false;

        /* Unmount the media if auto mount feature is enabled */
        if (SYS_FS_AUTOMOUNT_ENABLE == false)
        {
            continue;
        }

        for (index = 0; index < SYS_FS_VOLUME_NUMBER; index++)
        {
            /* Find out the mount name from the media mount table */
            if (mediaObj->mediaType != fsMount[index].mediaType)
            {
                continue;
            }

            if(0 != strcmp((const char *)(volumeObj->volumeName), (const char *)(fsMount[index].devName+5)))
            {
                continue;
            }

            /* If the device is present in the mount table then fetch the mount name
               which is the input for the unmount function */
            if (SYS_FS_RES_SUCCESS != SYS_FS_Unmount(fsMount[index].mountName))
            {
                continue;
            }

            if (gSYSFSEventHandler)
            {
                gSYSFSEventHandler (SYS_FS_EVENT_UNMOUNT, (void*)fsMount[index].mountName, (uintptr_t)NULL);
            }

            break;
        }
    }
}

static void _SYS_FS_MountVolume 
(
    SYS_FS_MEDIA_TYPE mediaType,
    const uint8_t *volumeName
)
{
    uint8_t volumeIndex = 0;
    const SYS_FS_MEDIA_MOUNT_DATA *fsMount = (const SYS_FS_MEDIA_MOUNT_DATA *)&gSYSFSMediaManagerObj.fsMountTable[0];

    for (volumeIndex = 0; volumeIndex < SYS_FS_VOLUME_NUMBER; volumeIndex++)
    {
        if (mediaType != fsMount[volumeIndex].mediaType)
        {
            continue;
        }

        if (strcmp((const char *)volumeName, (const char *)(fsMount[volumeIndex].devName + 5)) != 0)
        {
            continue;
        }

        if (SYS_FS_RES_SUCCESS == SYS_FS_Mount(fsMount[volumeIndex].devName, fsMount[volumeIndex].mountName, fsMount[volumeIndex].fsType, 0, NULL))
        {
            if (gSYSFSEventHandler)
            {
                gSYSFSEventHandler(SYS_FS_EVENT_MOUNT, (void*)fsMount[volumeIndex].mountName, (uintptr_t)NULL);
            }
        }

        break;
    }
}

static void _SYS_FS_MEDIA_MANAGER_PopulateVolume
(
    SYS_FS_MEDIA *mediaObj,
    uint8_t isMBR,
    uint8_t partitionMap,
    uint8_t fsType
)
{
    uint8_t volumeIndex = 0;
    uint8_t volumeNameLen = 0;
    uint8_t partitionNum = 0;
    uint16_t offset = 0;

    SYS_FS_VOLUME *volumeObj = NULL;
    uint8_t *readBuffer = NULL;

    volumeObj = &gSYSFSMediaManagerObj.volumeObj[0];
    readBuffer = gSYSFSMediaManagerObj.mediaBuffer;

    for (volumeIndex = 0; volumeIndex < SYS_FS_VOLUME_NUMBER; volumeIndex++)
    {
        if (volumeObj->inUse == true) 
        {
            volumeObj++;
            continue;
        }

        /* Found a free volume */
        mediaObj->numVolumes++;

        volumeNameLen = strlen(gSYSFSVolumeName[mediaObj->mediaType]);
        strncpy (volumeObj->volumeName, gSYSFSVolumeName[mediaObj->mediaType], volumeNameLen);

        /* Store the volume name */
        volumeObj->volumeName[volumeNameLen++] = mediaObj->mediaId;
        volumeObj->volumeName[volumeNameLen++] = mediaObj->numVolumes + '0';
        volumeObj->volumeName[volumeNameLen] = '\0';

        volumeObj->obj = mediaObj;
        volumeObj->fsType = fsType;

        if ('M' == volumeObj->fsType)
        {
            /* MPFS File System */
            volumeObj->numSectors = 0;
            volumeObj->startSector = 0;
        }
        else
        {
            /* Register Media and Volume mapping with FAT File System */
            /* Register the volumes to partition table only if this device has
             * partition table entry or MBR. Skip if media contains VBR */
            if (partitionMap)
            {
                if (partitionMap & 0x01)
                {
                    partitionNum = 0;
                    partitionMap &= ~0x01;
                }
                else if (partitionMap & 0x02)
                {
                    partitionNum = 1;
                    partitionMap &= ~0x02;
                }
                else if (partitionMap & 0x04)
                {
                    partitionNum = 2;
                    partitionMap &= ~0x04;
                }
                else if (partitionMap & 0x08)
                {
                    partitionNum = 3;
                    partitionMap &= ~0x08;
                }

                if (fsType != 0xFF)
                {
                    /* File system type offset */
                    offset = gPartitionTypeOffset[partitionNum];
                    volumeObj->fsType = readBuffer[offset];

                    /* Number of sectors */
                    volumeObj->numSectors = ((readBuffer[offset + 11] << 24) + (readBuffer[offset + 10] << 16) + (readBuffer[offset + 9] << 8) + readBuffer[offset + 8]);

                    /* Start address of the volume */
                    volumeObj->startSector = ((readBuffer[offset + 7] << 24) + (readBuffer[offset + 6] << 16) + (readBuffer[offset + 5] << 8) + readBuffer[offset + 4]);
                }
            }

            if (isMBR)
            {
                _SYS_FS_MEDIA_MANAGER_UpdateVolToPart (volumeIndex, mediaObj->mediaIndex, partitionNum + 1);
            }
            else
            {
                _SYS_FS_MEDIA_MANAGER_UpdateVolToPart (volumeIndex, mediaObj->mediaIndex, 0);                        
            }
        }
        /* Update the inUse flag to indicate that the volume is now in use.*/
        volumeObj->inUse = true;

        /* Mount the Media if Auto mount feature is enable */
        if (SYS_FS_AUTOMOUNT_ENABLE && (fsType != 0xFF))
        {
            _SYS_FS_MountVolume (mediaObj->mediaType, (const uint8_t *)(volumeObj->volumeName));
        }

        /* Continue if there is more than one partition on media */
        if (!partitionMap)
        {
            break;
        }

        /* Go to the next volume object */
        volumeObj++;
    }
}

static uint8_t _SYS_FS_MEDIA_MANAGER_FindNextMedia
(
    SYS_FS_MEDIA *mediaObj,
    uint8_t *index
)
{
    uint8_t indexLow = 0;
    
    indexLow = *index;
    while (*index < SYS_FS_MEDIA_NUMBER)
    {
        if (mediaObj[*index].inUse == true)
        {
            /* Media found. Return the index. */
            indexLow = *index;
            _SYS_FS_MEDIA_MANAGER_UPDATE_MEDIA_INDEX(*index);
            return indexLow;
        }

        (*index)++;
    }

    if (indexLow == 0)
    {
        /* No media. Reset the media index. */
        *index = 0;
        return 0xFF;
    }

    /* No media found in the upper range. Start searching from 0 till *index.
     * */
    indexLow = 0;
    while (indexLow < *index)
    {
        if (mediaObj[indexLow].inUse == true)
        {
            /* Media found. Return the media index. */
            *index = indexLow;
            _SYS_FS_MEDIA_MANAGER_UPDATE_MEDIA_INDEX(*index);
            return indexLow;
        }

        indexLow++;
    }

    /* No media. */
    return 0xFF;
}

static bool SYS_FS_MEDIA_MANAGER_IsFSFat
(
    uint8_t fsType
)
{
    switch (fsType)
    {
        case 0x01: // FAT12
        case 0x04: // FAT16
        case 0x05: // Extended partition
        case 0x06: // FAT16
        case 0x0B: // FAT32
        case 0x0C: // FAT32
        case 0x0E: // FAT16
        case 0x0F: // FAT16
            {
                return true;
            }
        default:
            {
                return false;
            }
    }
}

static uint8_t _SYS_FS_MEDIA_MANAGER_AnalyzeFileSystem
(
    uint8_t *firstSector,
    uint8_t *numPartition,
    uint8_t *isMBR,
    uint8_t *partitionMap
)
{
    uint8_t fsType = 0xFF;

    *partitionMap = 0;
    *numPartition = 0;

    /* Check for the Boot Signature */
    if((firstSector[510] == 0x55) && (firstSector[511] == 0xAA))
    {
        /* Check if the first sector of the media is Volume Boot Record or the
           Master Boot Record */
        if((0xEB == firstSector[0]) &&
                ((0x3C == firstSector[1]) || (0x58 == firstSector[1]) || (0xFE == firstSector[1])) &&
                (0x90 == firstSector[2]))
        {
            /* Volume Boot Record */
            *numPartition = 1;
            *isMBR = 0;

            /* The extended BPB contains FAT32 in this field */
            if ((0x46 == firstSector[82]) || (0x41 == firstSector[83]) ||
                    (0x54 == firstSector[84]))
            {
                fsType = 0x0B;
            }
            /* The extended BPB contains FAT, FAT12 or FAT16 in these fields */
            else if ((0x46 == firstSector[54]) || (0x41 == firstSector[55]) ||
                    (0x54 == firstSector[56]))
            {
                fsType = 0x06;
            }
        }
        else
        {     
            /* The partition table in the MBR sector has room for four 16-byte
             * entries that each specify the sectors that belong to a
             * partition. The table is in bytes 446 through 509. An entry can
             * begin at byte 446, 462, 478, or 494. */

            /* Determine total partitions in the media */
            *isMBR = 1;
            if(SYS_FS_MEDIA_MANAGER_IsFSFat(firstSector[450]))
            {
                *partitionMap |= (1 << 0);
                (*numPartition)++;
            }
            if(SYS_FS_MEDIA_MANAGER_IsFSFat(firstSector[466]))
            {
                *partitionMap |= (1 << 1);
                (*numPartition)++;
            }
            if(SYS_FS_MEDIA_MANAGER_IsFSFat(firstSector[482]))
            {
                *partitionMap |= (1 << 2);
                (*numPartition)++;
            }
            if(SYS_FS_MEDIA_MANAGER_IsFSFat(firstSector[498]))
            {
                *partitionMap |= (1 << 3);
                (*numPartition)++;
            }

            /* Found at least one valid partition. Assign a non 0xFF value to
             * indicate that atleast one of the partitions has a valid file
             * system. */
            if (*partitionMap)
            {
                fsType = 1;
            }
        }
    }
    else if (0 == memcmp(firstSector, "MPFS", 4))
    {
        /* No partitions in MPFS, hence, go to other state */
        /* allocate a new volume to each partition */
        (*numPartition) = 1;
        /* This is 0x4D which also mean file system Primary QNX POSIX volume on disk */
        /* Need to find an unused value from the partition type*/
        fsType = 'M';
    }
    else /* If MBR is not detected, make media as unsupported */
    {
        /* File system either not present or not supported */
        fsType = 0xFF;
    }

    return fsType;
}

SYS_FS_MEDIA_HANDLE SYS_FS_MEDIA_MANAGER_Register
(
    SYS_MODULE_OBJ obj,
    SYS_MODULE_INDEX index,
    const SYS_FS_MEDIA_FUNCTIONS *mediaFunctions,
    SYS_FS_MEDIA_TYPE mediaType
)
{
    uint8_t mediaIndex = 0;
    uint8_t mediaId = 'a';

    SYS_FS_MEDIA *mediaObj = NULL;

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[0];
    for (mediaIndex = 0; mediaIndex < SYS_FS_MEDIA_NUMBER; mediaIndex++)
    {
         if ((mediaObj->inUse) && (mediaType == mediaObj->mediaType))
         {
             /* For every type of media, increment the mediaID as a, b, c... */
             mediaId++;
         }
         mediaObj++;
    }

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[0];
    /* Start with 0th disk and find a disk which is free */
    for (mediaIndex = 0; mediaIndex < SYS_FS_MEDIA_NUMBER; mediaIndex++)
    {
        if (mediaObj->inUse == false)
        {
            mediaObj->inUse = true;

            mediaObj->driverFunctions = mediaFunctions;
            mediaObj->driverObj = obj;
            mediaObj->driverIndex = index;
            mediaObj->driverHandle = DRV_HANDLE_INVALID;

            mediaObj->mediaState = SYS_FS_MEDIA_STATE_REGISTERED;
            mediaObj->mediaType = mediaType;

            mediaObj->numPartitions = 0;
            mediaObj->numVolumes = 0;

            mediaObj->mediaIndex = mediaIndex;
            mediaObj->mediaId = mediaId;
            mediaObj->attachStatus = SYS_FS_MEDIA_DETACHED;

            return (SYS_FS_MEDIA_HANDLE)mediaObj;
        }

        mediaObj++;
    }

    return SYS_FS_MEDIA_HANDLE_INVALID;
}

void SYS_FS_MEDIA_MANAGER_DeRegister
(
    SYS_FS_MEDIA_HANDLE handle
)
{
    SYS_FS_MEDIA *mediaObj = (SYS_FS_MEDIA *)handle;

    if (handle == SYS_FS_MEDIA_HANDLE_INVALID)
    {
        return;
    }

    mediaObj->isMediaDisconnected = true;
}

SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE SYS_FS_MEDIA_MANAGER_SectorRead
(
    uint16_t diskNum,
    uint8_t *dataBuffer,
    uint32_t sector,
    uint32_t numSectors
)
{
    SYS_FS_MEDIA *mediaObj = NULL;
    uint32_t blocksPerSector = 0;
    uint32_t mediaReadBlockSize = 0;

    if (diskNum >= SYS_FS_MEDIA_NUMBER)
    {
        SYS_ASSERT(false, "Invalid Disk");
        return SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    }

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[diskNum];

    if (mediaObj->driverHandle == DRV_HANDLE_INVALID)
    {
        return SYS_FS_MEDIA_HANDLE_INVALID;
    }

    mediaReadBlockSize = mediaObj->mediaGeometry->geometryTable[0].blockSize;

    if (mediaReadBlockSize < 512)
    {
        /* Find the number of blocks per sector */
        blocksPerSector = 512 / mediaReadBlockSize;
        /* Perform sector to block translation */
        sector *= blocksPerSector; 
        numSectors *= blocksPerSector; 
    }
    else
    {
        /* TODO: Handle cases where the block size is greater than 512 bytes.
         * */
    }

    mediaObj->commandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;
    mediaObj->driverFunctions->sectorRead (mediaObj->driverHandle, &(mediaObj->commandHandle), dataBuffer, sector, numSectors);

    return (mediaObj->commandHandle);
}

SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE SYS_FS_MEDIA_MANAGER_Read
(
    uint16_t diskNum,
    uint8_t *destination,
    uint8_t *source,
    const uint32_t nBytes
)
{
    SYS_FS_MEDIA *mediaObj = NULL;
    uint32_t startAddress = 0;
    uint32_t address = 0;

    if (diskNum >= SYS_FS_MEDIA_NUMBER)
    {
        SYS_ASSERT(false, "Invalid Disk");
        return SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    }

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[diskNum];

    if (mediaObj->driverHandle == DRV_HANDLE_INVALID)
    {
        return SYS_FS_MEDIA_HANDLE_INVALID;
    }

    startAddress = mediaObj->driverFunctions->addressGet(mediaObj->driverHandle);
    address = (uint32_t)source - (uint32_t)startAddress;

    mediaObj->commandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;
    mediaObj->driverFunctions->Read(mediaObj->driverHandle, &(mediaObj->commandHandle), destination, address, nBytes);

    return (mediaObj->commandHandle);
}

SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE SYS_FS_MEDIA_MANAGER_SectorWrite
(
    uint16_t diskNum,
    uint32_t sector,
    uint8_t *dataBuffer,
    uint32_t numSectors
)
{
    SYS_FS_MEDIA *mediaObj = NULL;
    uint8_t *data = dataBuffer;
    uint32_t sectorOffsetInBlock = 0;
    uint32_t memoryBlock = 0;
    uint32_t sectorsPerBlock = 0;
    uint32_t numSectorsToWrite = 0;
    uint32_t mediaWriteBlockSize = 0;
    uint32_t blocksPerSector = 0;

    if(diskNum >= SYS_FS_VOLUME_NUMBER)
    {
        SYS_ASSERT(false, "Invalid Disk");
        return SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    }

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[diskNum];

    if (mediaObj->driverHandle == DRV_HANDLE_INVALID)
    {
        return SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    }

    mediaWriteBlockSize = mediaObj->mediaGeometry->geometryTable[1].blockSize;

    if (mediaWriteBlockSize > 512)
    {
        sectorsPerBlock = mediaWriteBlockSize / 512;
    }
    else if (mediaWriteBlockSize == 512)
    {
        sectorsPerBlock = 1;
        blocksPerSector = 1;
    }
    else
    {
        blocksPerSector = 512 / mediaWriteBlockSize;
        sector *= blocksPerSector;
        numSectors *= blocksPerSector;
    }

    if ((sectorsPerBlock == 1) || (blocksPerSector > 0))
    {
        mediaObj->commandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;
        mediaObj->driverFunctions->sectorWrite (mediaObj->driverHandle, &(mediaObj->commandHandle), dataBuffer, sector, numSectors);
        return (mediaObj->commandHandle);
    }
    else
    {
        /* Mute the event notification */
        gSYSFSMediaManagerObj.muteEventNotification = true;

        while (numSectors > 0)
        {
            /* Find the memory block for the starting sector */
            memoryBlock = sector / sectorsPerBlock;

            /* Find the number of sectors to be updated in this block. */
            sectorOffsetInBlock = (sector % sectorsPerBlock);
            numSectorsToWrite = (sectorsPerBlock - sectorOffsetInBlock);

            if (numSectors < numSectorsToWrite)
            {
                numSectorsToWrite = numSectors;
            }

            if (numSectorsToWrite != sectorsPerBlock)
            {
                /* Read the memory block from the media. Update the media data. */
                mediaObj->commandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;

                mediaObj->driverFunctions->sectorRead(mediaObj->driverHandle, &(mediaObj->commandHandle), gSYSFSMediaBlockBuffer, memoryBlock * mediaWriteBlockSize, SYS_FS_MEDIA_MANAGER_BUFFER_SIZE);

                while (mediaObj->commandStatus == SYS_FS_MEDIA_COMMAND_IN_PROGRESS)
                {
                    if(mediaObj->driverFunctions->tasks != NULL)
                    {
                        mediaObj->driverFunctions->tasks(mediaObj->driverObj);
                    }
                }

                if (mediaObj->commandStatus != SYS_FS_MEDIA_COMMAND_COMPLETED)
                {
                    /* Unmute the event notification */
                    gSYSFSMediaManagerObj.muteEventNotification = false;

                    /* Media read operation failed. */
                    return SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
                }

                /* Multiply by the sector size */
                sectorOffsetInBlock <<= 9;
                memcpy ((void *)&gSYSFSMediaBlockBuffer[sectorOffsetInBlock], (const void *)dataBuffer, numSectorsToWrite << 9);

                data = gSYSFSMediaBlockBuffer;
            }
            else
            {
                /* Since the whole block is being updated, there is no need to
                 * perform a read-modify-write operation of the block. */
                data = dataBuffer;
            }

            if ((numSectors - numSectorsToWrite) == 0)
            {
                /* This is the last write operation. */
                break;
            }

            /* Write the block to the media */
            mediaObj->commandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;
            mediaObj->driverFunctions->sectorWrite (mediaObj->driverHandle, &(mediaObj->commandHandle), data, memoryBlock, 1);
            while (mediaObj->commandStatus == SYS_FS_MEDIA_COMMAND_IN_PROGRESS)
            {
                if(mediaObj->driverFunctions->tasks != NULL)
                {
                    mediaObj->driverFunctions->tasks(mediaObj->driverObj);
                }
            }

            if (mediaObj->commandStatus != SYS_FS_MEDIA_COMMAND_COMPLETED)
            {
                /* Unmute the event notification */
                gSYSFSMediaManagerObj.muteEventNotification = false;

                /* Media write operation failed. */
                return SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
            }

            /* Update the number of block still to be written, sector address
             * and the buffer pointer */
            numSectors -= numSectorsToWrite;
            sector += numSectorsToWrite;
            dataBuffer += (numSectorsToWrite << 9);
        }
    }

    /* Unmute the event notification */
    gSYSFSMediaManagerObj.muteEventNotification = false;

    mediaObj->commandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;
    mediaObj->driverFunctions->sectorWrite (mediaObj->driverHandle, &(mediaObj->commandHandle), data, memoryBlock, 1);

    return (mediaObj->commandHandle);
}

uintptr_t SYS_FS_MEDIA_MANAGER_AddressGet
(
    uint16_t diskNum
)
{
    SYS_FS_MEDIA *mediaObj = NULL;

    if (diskNum >= SYS_FS_MEDIA_NUMBER)
    {
        SYS_ASSERT(false, "Invalid Disk");
        return 0;
    }

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[diskNum];
    return (mediaObj->driverFunctions->addressGet(mediaObj->driverHandle));
}

SYS_FS_MEDIA_COMMAND_STATUS SYS_FS_MEDIA_MANAGER_CommandStatusGet
(
    uint16_t diskNum,
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE commandHandle
)
{
    SYS_FS_MEDIA *mediaObj = NULL;

    if (diskNum >= SYS_FS_MEDIA_NUMBER)
    {
        SYS_ASSERT(false, "Invalid Disk");
        return SYS_FS_MEDIA_COMMAND_UNKNOWN;
    }

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[diskNum];

    if (mediaObj->driverHandle == DRV_HANDLE_INVALID)
    {
        return SYS_FS_MEDIA_COMMAND_UNKNOWN;
    }

    return (mediaObj->driverFunctions->commandStatusGet(mediaObj->driverHandle, commandHandle));
}

bool SYS_FS_MEDIA_MANAGER_MediaStatusGet
(
    const char *volumeName
)
{
    SYS_FS_VOLUME * volumeObj =  NULL;
    uint8_t volumeIndex = 0;

    for (volumeIndex = 0; volumeIndex < SYS_FS_VOLUME_NUMBER; volumeIndex++)
    {
        volumeObj = &gSYSFSMediaManagerObj.volumeObj[volumeIndex];
        if (volumeObj->inUse == true)
        {
            if (strncmp("/dev/", volumeName, 5))
            {
                if (strcmp((const char*)(volumeName), (const char *)volumeObj->volumeName) == 0)
                {
                    return (volumeObj->obj->driverFunctions->mediaStatusGet(volumeObj->obj->driverHandle));
                }
            }
            else
            {
                if (strcmp((const char*)(volumeName + 5), (const char *)volumeObj->volumeName) == 0)
                {
                    return (volumeObj->obj->driverFunctions->mediaStatusGet(volumeObj->obj->driverHandle));                    
                }
            }
        }
    }

    return SYS_FS_MEDIA_DETACHED;
}

bool SYS_FS_MEDIA_MANAGER_VolumePropertyGet
(
    const char *volumeName,
    SYS_FS_VOLUME_PROPERTY *property
)
{
    SYS_FS_VOLUME *volumeObj = NULL;
    uint8_t volumeIndex = 0;

    for (volumeIndex = 0; volumeIndex < SYS_FS_VOLUME_NUMBER; volumeIndex++)
    {
        volumeObj = &gSYSFSMediaManagerObj.volumeObj[volumeIndex];

        if (volumeObj->inUse == true)
        {
            if (strcmp((const char*)(volumeName + 5), (const char *)volumeObj->volumeName) == 0)
            {
                if (volumeObj->fsType == 'M')
                {
                    /* MPFS File System */
                    property->fsType = MPFS2;
                }
                else if (SYS_FS_MEDIA_MANAGER_IsFSFat (volumeObj->fsType))
                {
                    /* FAT File System */
                    property->fsType = FAT;
                }
                else
                {
                    /* Unsupported file system or no file system */
                    property->fsType = UNSUPPORTED_FS;
                }

                property->volNumber = volumeIndex;
                return true;
            }
        }
    }

    return false;
}

void SYS_FS_MEDIA_MANAGER_RegisterTransferHandler
(
    const void *eventHandler
)
{
    gSYSFSMediaManagerObj.eventHandler = eventHandler;
}

void SYS_FS_MEDIA_MANAGER_EventHandler
(
    SYS_FS_MEDIA_BLOCK_EVENT event,
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE commandHandle,
    uintptr_t context
)
{
    switch(event)
    {
        case SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE:
            ((SYS_FS_MEDIA*)context)->commandStatus = SYS_FS_MEDIA_COMMAND_COMPLETED;
            break;
        case SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR:
            ((SYS_FS_MEDIA*)context)->commandStatus = SYS_FS_MEDIA_COMMAND_UNKNOWN;
            break;
        default:
            break;
    }

    if ((gSYSFSMediaManagerObj.eventHandler != NULL) && (gSYSFSMediaManagerObj.muteEventNotification == false))
    {
        gSYSFSMediaManagerObj.eventHandler (event, (void *)commandHandle, context);
    }
}

SYS_FS_MEDIA_GEOMETRY * SYS_FS_MEDIA_MANAGER_GetMediaGeometry
(
    uint16_t diskNum
)
{
    SYS_FS_MEDIA *mediaObj = NULL;

    if (diskNum >= SYS_FS_MEDIA_NUMBER)
    {
        SYS_ASSERT(false, "Invalid Disk");
        return NULL;
    }

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[diskNum];

    if (mediaObj->attachStatus == SYS_FS_MEDIA_DETACHED)
    {
        return NULL;
    }

    return mediaObj->mediaGeometry;
}

void SYS_FS_MEDIA_MANAGER_TransferTask
(
    uint8_t mediaIndex
)
{
    SYS_FS_MEDIA *mediaObj;

    if (mediaIndex >= SYS_FS_MEDIA_NUMBER)
    {
        return;
    }
    
    mediaObj = &gSYSFSMediaManagerObj.mediaObj[mediaIndex];

    if (mediaObj->inUse == false)
    {
        return;
    }

    if (mediaObj->driverFunctions->tasks != NULL)
    {
        mediaObj->driverFunctions->tasks(mediaObj->driverObj);
    }
}

void SYS_FS_MEDIA_MANAGER_Tasks
(
    void
)
{
    uint8_t mediaIndex = 0;
    uint8_t fsType = 0xFF;
    uint8_t partitionMap = 0;
    uint8_t isMBR = 0;
    uint32_t numSectors = 0;
    uint32_t mediaReadBlockSize = 0;

    SYS_FS_MEDIA *mediaObj = NULL;

    /* Find the next media to be processed */
    mediaIndex = _SYS_FS_MEDIA_MANAGER_FindNextMedia (&gSYSFSMediaManagerObj.mediaObj[0], &gSYSFSMediaManagerObj.mediaIndex);
    if (mediaIndex == 0xFF)
    {
        /* No media attached. Do nothing. */
        return;
    }

    mediaObj = &gSYSFSMediaManagerObj.mediaObj[mediaIndex];

    if (mediaObj->isMediaDisconnected == true)
    {
        /* If media driver has de-registered then switch to the DEREGISTERED
         * state and handle the media detach. */
        mediaObj->mediaState = SYS_FS_MEDIA_STATE_DEREGISTERED;
    }

    switch (mediaObj->mediaState)
    {
        case SYS_FS_MEDIA_STATE_REGISTERED:
            {
                /* Initial state. Open the media driver. */
                mediaObj->driverHandle = mediaObj->driverFunctions->open(mediaObj->driverIndex, DRV_IO_INTENT_READWRITE);
                if (mediaObj->driverHandle != DRV_HANDLE_INVALID)
                {
                    /* Media driver open successful. Register an event handler for
                     * the media driver events. */
                    mediaObj->driverFunctions->eventHandlerset(mediaObj->driverHandle, SYS_FS_MEDIA_MANAGER_EventHandler, (uintptr_t)mediaObj);

                    /* Transition to the next state. */
                    mediaObj->mediaState = SYS_FS_MEDIA_CHECK_ATTACH_STATUS;
                }
                else
                {
                    /* Media driver open failed. Stay in the same state and
                     * retry. */
                }

                break;
            }

        case SYS_FS_MEDIA_CHECK_ATTACH_STATUS:
            {
                /* Check if the Media is attached. */
                if (mediaObj->driverFunctions->mediaStatusGet(mediaObj->driverHandle) == true)
                {
                    if (mediaObj->attachStatus == SYS_FS_MEDIA_DETACHED)
                    {
                        /* The media was earlier detached. But now it is
                         * attached. Read the media geometry. */
                        mediaObj->mediaGeometry = mediaObj->driverFunctions->mediaGeometryGet(mediaObj->driverHandle);

                        /* Transition to the next state to kick start the
                         * analysis of the FS on the media. */
                        mediaObj->mediaState = SYS_FS_MEDIA_READ_FIRST_SECTOR;
                    }

                    /* Update the media status */
                    mediaObj->attachStatus = SYS_FS_MEDIA_ATTACHED;
                }
                else
                {
                    if (mediaObj->attachStatus == SYS_FS_MEDIA_ATTACHED)
                    {
                        /* The media was earlier attached. But now it is
                         * detached. Handle the media detach. */
                        _SYS_FS_MEDIA_MANAGER_HandleMediaDetach (mediaObj);

                        /* Reset the media's number of volumes field */
                        mediaObj->numVolumes = 0;
                    }

                    /* Update the media status */
                    mediaObj->attachStatus = SYS_FS_MEDIA_DETACHED;

                    /* Stay in the same state */
                }

                break;
            }

        case SYS_FS_MEDIA_READ_FIRST_SECTOR:
            {
                if (gSYSFSMediaManagerObj.bufferInUse == true)
                {
                    /* Stay in the same state till the buffer becomes free. */
                    break;
                }

                gSYSFSMediaManagerObj.bufferInUse = true;

                numSectors = 1;

                mediaReadBlockSize = mediaObj->mediaGeometry->geometryTable[0].blockSize;

                if (mediaReadBlockSize < 512)
                {
                    /* Perform sector to block translation */
                    numSectors *= (512 / mediaReadBlockSize);
                }

                memset (gSYSFSMediaManagerObj.mediaBuffer, 0, SYS_FS_MEDIA_MAX_BLOCK_SIZE);

                /* Update the command status */
                mediaObj->commandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;

                /* Read the first sector of the media */
                mediaObj->driverFunctions->sectorRead(mediaObj->driverHandle, &(mediaObj->commandHandle), gSYSFSMediaManagerObj.mediaBuffer, 0, numSectors);

                if (mediaObj->commandHandle != SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID)
                {
                    mediaObj->mediaState = SYS_FS_MEDIA_ANALYZE_FS;
                }
                else
                {
                    /* Failed to queue the Media read operation. Retry the
                     * operation. Clear the buffer in use flag. */
                    gSYSFSMediaManagerObj.bufferInUse = false;
                }

                break;
            }

        case SYS_FS_MEDIA_ANALYZE_FS:
            {
                if (SYS_FS_MEDIA_COMMAND_IN_PROGRESS == mediaObj->commandStatus)
                {
                    /* The request is not complete yet. Stay in the same state.
                     * */
                    if(mediaObj->driverFunctions->tasks != NULL)
                    {
                        mediaObj->driverFunctions->tasks(mediaObj->driverObj);
                    }
                    break;
                }

                if (mediaObj->commandStatus != SYS_FS_MEDIA_COMMAND_COMPLETED)
                {
                    /* Clear the buffer in use flag. */
                    gSYSFSMediaManagerObj.bufferInUse = false;

                    /* Media read operation has failed. Retry the read
                     * operation. */
                    mediaObj->mediaState = SYS_FS_MEDIA_READ_FIRST_SECTOR;

                    break;
                }

                fsType = _SYS_FS_MEDIA_MANAGER_AnalyzeFileSystem(gSYSFSMediaManagerObj.mediaBuffer, &mediaObj->numPartitions, &isMBR, &partitionMap);

                if (fsType == 0xFF)
                {
                    /* File system not found or found an unsupported file
                     * system. Allocate a volume so as to allow for formatting
                     * of the disk. */
                    if (isMBR)
                    {
                        partitionMap = 0x01;
                    }
                    else
                    {
                        partitionMap = 0x00;
                    }
                }

                _SYS_FS_MEDIA_MANAGER_PopulateVolume (mediaObj, isMBR, partitionMap, fsType);

                /* Clear the buffer in use flag. */
                gSYSFSMediaManagerObj.bufferInUse = false;

                /* Transition state to check for media attach/detach */
                mediaObj->mediaState = SYS_FS_MEDIA_CHECK_ATTACH_STATUS;

                break;
            }

        case SYS_FS_MEDIA_STATE_DEREGISTERED:
            {
                if (mediaObj->attachStatus == SYS_FS_MEDIA_ATTACHED)
                {
                    /* The media was earlier attached. But now it is
                     * detached. Handle the media detach. */
                    _SYS_FS_MEDIA_MANAGER_HandleMediaDetach (mediaObj);
                }

                mediaObj->inUse = false;
                mediaObj->attachStatus = SYS_FS_MEDIA_DETACHED;
                mediaObj->isMediaDisconnected = false;
                mediaObj->mediaId = 0;
                mediaObj->mediaIndex = 0;
                mediaObj->numPartitions = 0;
                mediaObj->numVolumes = 0;
                break;
            }

        default:
            break;
    }
}


