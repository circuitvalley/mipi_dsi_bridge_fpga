#ifndef _SYS_FS_MEDIA_MANAGER_LOCAL_H_
#define _SYS_FS_MEDIA_MANAGER_LOCAL_H_

#include "system_config.h"
#include "system/int/sys_int.h"
#include "system/common/sys_module.h"
#include "system/fs/sys_fs_media_manager.h"
#include "system/fs/fat_fs/src/hardware_access/diskio.h"

#if defined (__PIC32C__)
#define COHERENT_ALIGNED_16BIT
#else
#define COHERENT_ALIGNED_16BIT              __attribute__((coherent, aligned(16)))
#endif

/* Read entry in geometry table */
#define SYS_FS_MEDIA_GEOMETRY_READ          (0)

/* Write entry in geometry table */
#define SYS_FS_MEDIA_GEOMETRY_WRITE         (1)

/* Erase entry in geometry table */
#define SYS_FS_MEDIA_GEOMETRY_ERASE         (2)

/* Shift Value for multiply or divide by a sector of size 512 bytes*/
#define SYS_FS_MEDIA_SHIFT_SECTOR_VALUE     (9)

#define _SYS_FS_MEDIA_MANAGER_UPDATE_MEDIA_INDEX(token) \
{ \
    (token)++; \
    (token) = ((token) == SYS_FS_MEDIA_NUMBER) ? 0: (token); \
}

// *****************************************************************************
/* Media object

  Summary:
    Defines the media object.

  Description:
    This structure defines the object that defines a media. Whenever a media is
 * connected, the media calls the function "SYS_FS_MEDIA_MANAGER_Register" to
 * register itself to the media manager. This object holds the property of media.

  Remarks:
    None.
*/
typedef struct _SYS_FS_MEDIA
{
    /* Indicates if the media object is in use. */
    bool inUse;

    /* Number of partitions available in the media */
    uint8_t numPartitions;

    /* Number identifying the media. This is a alphabet name for each media. It
       starts with a, b etc. */
    uint8_t mediaId;

    /* This is a count for media. For every new media (any type) registered,
     * this counter is assigned a new value */
    uint8_t mediaIndex;

    /* Number of volumes that can be assigned in the present media. This
     * reflects the number of valid partitions available on the media */
    uint8_t numVolumes;

    /* Flag tracking the media de-register state */
    uint8_t isMediaDisconnected;

    /* Media driver index used to open the media driver. */
    SYS_MODULE_INDEX driverIndex;

    /* Pointer to Media driver functions. */
    const SYS_FS_MEDIA_FUNCTIONS *driverFunctions;

    /* State of the media (Registered,  opened, attached, analyzed) */
    SYS_FS_MEDIA_STATE mediaState;

    /* Media driver object. This object is used to run the media task routine
     * */
    SYS_MODULE_OBJ driverObj;

    /* Handle received after the media driver is opened */
    DRV_HANDLE driverHandle;

    /* Type of the media */
    SYS_FS_MEDIA_TYPE mediaType;

    /* Handle received when trying to read or write a sector from the media */
    SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE commandHandle;

    /* Media attach/detach status */
    SYS_FS_MEDIA_STATUS attachStatus;

    /* Command status of Media */
    SYS_FS_MEDIA_COMMAND_STATUS commandStatus;

    /* Pointer to the media geometry */
    SYS_FS_MEDIA_GEOMETRY *mediaGeometry;

} SYS_FS_MEDIA;

// *****************************************************************************
/* Volume object

  Summary:
    Defines the volume object.

  Description:
    This structure defines the object that defines a volume. A volume is assigned
    for a media, if there is only 1 partition. If there are multipartitions, then,
    each volume is assigned to each partition.

  Remarks:
    None.
*/
typedef struct _SYS_FS_VOLUME
{
    /* Indicates if the volume object is being used. */
    bool inUse;

    /* Type of file system associated with this volume */
    uint8_t fsType;

    /* Name of volume (nvma1 or mmcblka1 etc..) */
    char volumeName[13];

    /* Padding byte */
    uint8_t pad1;

    /* Starting sector number for the volume */
    uint32_t startSector;

    /* Number of sectors in the volume */
    uint32_t numSectors;

    /* Pointer to the media object to which this volume belongs. */
    SYS_FS_MEDIA *obj;

} SYS_FS_VOLUME;

// *****************************************************************************
/* Media manager task object

  Summary:
    Defines the object required for the operation and control of media manger task.

  Description:
    This structure defines the object required for the opeation of the media
    manager task.

  Remarks:
    None.
*/
typedef struct SYS_FS_MEDIA_MANAGER_OBJ
{
    /* Pointer to the media object array */
    SYS_FS_MEDIA *mediaObj;

    /* Pointer to the volume object array */
    SYS_FS_VOLUME *volumeObj;

    /* Pointer to the FS mount table */
    const SYS_FS_MEDIA_MOUNT_DATA *fsMountTable;

    /* Media Manager event handler */
    SYS_FS_EVENT_HANDLER eventHandler;

    /* Pointer to the buffer used for media sector reads */
    uint8_t *mediaBuffer;

    /* Media index */
    uint8_t mediaIndex;

    /* Flag to track the usage of the mediaBuffer */
    uint8_t bufferInUse;

    /* Flag used to mute/unmute event notifications */
    bool muteEventNotification;

} SYS_FS_MEDIA_MANAGER_OBJ;

/***************************************************************
 * The following structure was added to enable the "multipartition"
 * feature of FAT FS. This strucre is already declared in ff.h and
 * the intention was to make as little change on ff.h
 * To use multipartition on FAT FS, we need to enable "_MULTI_PARTITION".
 * And, when we do that, the FAT FS code expects an array named "VolToPart".
 * The explanation for each element of the array is given below, and
 * this function places the elements of this array.
 *
 * Lets consider a case where 2 media are attached = SD card with 4 partitions
 * and NVM with 1 partition.
 *
 * PARTITION VolToPart[SYS_FS_VOLUME_NUMBER] = {
 *        {0, 1},    // 0th volume # assigned by sys_fs_media_manager (mmcblka1), media # = 0 (SD card), partition # = 1
 *        {0, 2},    // 1st volume # assigned by sys_fs_media_manager (mmcblka2), media # = 0 (SD card), partition # = 2
 *        {0, 3},    // 2nd volume # assigned by sys_fs_media_manager (mmcblka3), media # = 0 (SD card), partition # = 3
 *        {0, 4},    // 3rd volume # assigned by sys_fs_media_manager (mmcblka4), media # = 0 (SD card), partition # = 4
 *        {1, 1}     // 4th volume # assigned by sys_fs_media_manager (nvma1), media # = 1 (NVM), partition # = 1
 *    };
 ***************************************************************/
#endif

