/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "system/fs/fat_fs/src/hardware_access/diskio.h"		/* FatFs lower layer API */
#include "system/fs/sys_fs_media_manager.h"

SYS_FS_MEDIA_COMMAND_STATUS gDiskCommandStatus = SYS_FS_MEDIA_COMMAND_UNKNOWN;
SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE gDiskCommandHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;

void diskEventHandler
(
    SYS_FS_MEDIA_BLOCK_EVENT event,
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE commandHandle,
    uintptr_t context
)
{
    switch(event)
    {
        case SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE:
            gDiskCommandStatus = SYS_FS_MEDIA_COMMAND_COMPLETED;
            break;
        case SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR:
            gDiskCommandStatus= SYS_FS_MEDIA_COMMAND_UNKNOWN;
            break;
        default:
            break;
    }
}

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/* This function is left unchanged from what was implemted in FAT FS. This
 is done to maintain compatibility with future versions of FAT FS. Also,
 no task is done in this function. It is not required as Harmony File system
 has its own media manager layer, which takes care of disk_initialization */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
        uint8_t pdrv				/* Physical drive nmuber (0..) */
        )
{
    switch(pdrv)
    {
        case 0:
            {
                break;
            }
        case 1:
            {
                break;
            }
    }

    SYS_FS_MEDIA_MANAGER_RegisterTransferHandler (diskEventHandler);
    return 0;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/* This function is left unchanged from what was implemted in FAT FS. This
 is done to maintain compatibility with future versions of FAT FS. Also,
 no task is done in this function. It is not required as Harmony File system
 has its own media manager layer, which takes care of disk_status        */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
        uint8_t pdrv		/* Physical drive nmuber (0..) */
        )
{
    return 0;
}

/*****************************************************************************
 * Function:
 * DRESULT disk_read
 *   (
 *       uint8_t pdrv,
 *       uint8_t *buff,
 *       uint32_t sector,
 *       uint8_t count
 *   )
 *
 * Description:
 * Function to read a sector of specified media (disk). This is the function
 * called from ff.c. This function is block untill the read is complete.
 * Since this function is called by ff.c, the name of function, parameter passed
 * and return types are not changed. This is to avoid compatibility issues if we
 * use future releases of ff.c
 *
 * Precondition:
 *   None
 *
 * Parameters:
 *   pdrv         - media number
 *   buff         - pointer to buffer where data to be placed after read
 *   sector       - Sector # which is to be read
 *   count        - Number of sectors to read
 *
 * Returns:
 *   DRESULT
*/

DRESULT disk_read
(
    uint8_t pdrv,   /* Physical drive nmuber (0..) */
    uint8_t *buff,  /* Data buffer to store read data */
    uint32_t sector,/* Sector address (LBA) */
    uint8_t count   /* Number of sectors to read (1..128) */
)
{
    gDiskCommandHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;

    gDiskCommandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;
    /* submit the read request */
    gDiskCommandHandle = SYS_FS_MEDIA_MANAGER_SectorRead(pdrv /* DISK 0 */ ,
            buff /* Destination Sector*/,
            sector,
            count /* Number of Sectors */);
    
    /* Buffer is invalid report error */
    if (SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID == gDiskCommandHandle)
    {
        return RES_PARERR;
    }

    /* process the read request by blocking on the task routine that process the 
     I/O request */
    while (SYS_FS_MEDIA_COMMAND_IN_PROGRESS == gDiskCommandStatus)
    {
        SYS_FS_MEDIA_MANAGER_TransferTask (pdrv);
    }

    if (SYS_FS_MEDIA_COMMAND_COMPLETED == gDiskCommandStatus)
    {
        /* Buffer processed successfully */
        return RES_OK;
    }        
    else
    {
        /* Buffer processing failed */
        return RES_ERROR;
    }
}

/*****************************************************************************
 * Function:
 * DRESULT disk_write
 *   (
 *       uint8_t pdrv,
 *       uint8_t *buff,
 *       uint32_t sector,
 *       uint8_t count
 *   )
 *
 * Description:
 * Function to write to a sector of specified media (disk). This is the function
 * called from ff.c. This function is block untill the write is complete.
 * Since this function is called by ff.c, the name of function, parameter passed
 * and return types are not changed. This is to avoid compatibility issues if we
 * use future releases of ff.c
 *
 * Precondition:
 *   None
 *
 * Parameters:
 *   pdrv         - media number
 *   buff         - pointer to buffer which holds the data to write
 *   sector       - Sector # where data to be written
 *   count        - Number of sectors to write
 *
 * Returns:
 *   DRESULT
*/

#if _USE_WRITE
DRESULT disk_write
(
    uint8_t pdrv,       /* Physical drive nmuber (0..) */
    const uint8_t *buff,/* Data to be written */
    uint32_t sector,    /* Sector address (LBA) */
    uint8_t count       /* Number of sectors to write (1..128) */
)
{
    gDiskCommandHandle = SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID;
    gDiskCommandStatus = SYS_FS_MEDIA_COMMAND_IN_PROGRESS;

    /* Submit the write request to media */
    gDiskCommandHandle = SYS_FS_MEDIA_MANAGER_SectorWrite(pdrv /* DISK 0 */ ,
            sector /* Destination Sector*/,
            (uint8_t *)buff,
            count /* Number of Sectors */);
    /* Write request failed , return with error */
    if(SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID == gDiskCommandHandle)
    {
        return RES_PARERR;
    }

    /* Run the task routine of media to process the request  */
    while(SYS_FS_MEDIA_COMMAND_IN_PROGRESS == gDiskCommandStatus)
    {
        SYS_FS_MEDIA_MANAGER_TransferTask (pdrv);
    }
    
    if(SYS_FS_MEDIA_COMMAND_COMPLETED == gDiskCommandStatus)
    {
        /* Buffer processed successfully */
        return RES_OK;
    }        
    else
    {
        /* Buffer processing failed */
        return RES_ERROR;
    }
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/* This function is left unchanged from what was implemted in FAT FS. This
 is done to maintain compatibility with future versions of FAT FS. Also,
 no task is done in this function. This function will be implemented in
 future                                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
        uint8_t pdrv,		/* Physical drive nmuber (0..) */
        uint8_t cmd,		/* Control code */
        void *buff		/* Buffer to send/receive control data */
        )
{
    if (cmd == GET_SECTOR_COUNT)
    {
        uint32_t numSectors = 0;
        uint32_t mediaBlockSize = 0;
        uint32_t numBlocksPerSector = 1;
        SYS_FS_MEDIA_GEOMETRY *mediaGeometry = NULL;

        mediaGeometry = SYS_FS_MEDIA_MANAGER_GetMediaGeometry (pdrv);
        if (mediaGeometry == NULL)
        {
            return RES_ERROR;
        }
        mediaBlockSize = mediaGeometry->geometryTable[0].blockSize;

        if (mediaBlockSize < 512)
        {
            /* Perform block to sector translation */
            numBlocksPerSector = (512 / mediaBlockSize);
        }

        numSectors = mediaGeometry->geometryTable[0].numBlocks / numBlocksPerSector;

        *(uint32_t *)buff = numSectors;
    }

    return RES_OK;
}
#endif

/****************************************************************************
 The get_fattime function is used to know the present time, which is used by
 FAT FS code. The present time should ideally be updated by a Real time clock
 hardware module. Since this module is not integrated with Harmony FS framework,
 a fixed time is set as given by the implementation of the function below   */
uint32_t get_fattime(void)
{
    /* RTC should return time here */
    /* For now, just a value */
    SYS_FS_TIME time;
    time.packedTime = 0;

    // All FAT FS times are calculated based on 0 = 1980
    time.discreteTime.year = (2013 - 1980); // year is 2013
    time.discreteTime.month = 8;    // August
    time.discreteTime.day = 9;     // 9th date
    time.discreteTime.hour = 15;    // 3pm afternoon
    time.discreteTime.minute = 06;  // 06 min
    time.discreteTime.second = 00;  // 00 sec

    return (time.packedTime);
}

