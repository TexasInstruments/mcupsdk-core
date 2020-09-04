/*
 * FreeRTOS+FAT V2.3.3
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */
/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portable.h"

/* FreeRTOS+FAT includes. */
#include "ff_headers.h"
#include "ff_mmcsd.h"
#include "ff_sys.h"

/* MMCSD includes */
#include <drivers/mmcsd.h>

#define MMCSD_SECTOR_SIZE            512UL
#define MMCSD_IOMAN_CACHE_SIZE       4096U
#define MMCSD_HIDDEN_SECTOR_COUNT    8U
#define MMCSD_PRIMARY_PARTITIONS     1U

/* Used as a magic number to indicate that an FF_Disk_t structure is a MMCSD
 * disk. */
#define MMCSD_SIGNATURE              0xFF202100

/*
 * The function that writes to the media
 */
static int32_t prvWriteMmcsd( uint8_t * pucSource,
                            uint32_t ulSectorNumber,
                            uint32_t ulSectorCount,
                            FF_Disk_t * pxDisk );

/*
 * The function that reads from the media
 */
static int32_t prvReadMmcsd( uint8_t * pucDestination,
                           uint32_t ulSectorNumber,
                           uint32_t ulSectorCount,
                           FF_Disk_t * pxDisk );

/* This is the prototype of the function used to initialize the MMCSD disk driver.
 * Other media drivers do not have to have the same prototype.
 *
 * In this example:
 + pcName is the name to give the disk within FreeRTOS+FAT's virtual file system.
 + pxDisk is a pointer to the FF_Disk_t structure pre allocated in the application
 */
FF_Disk_t * FF_MMCSDDiskInit( char * pcName,
                           FF_Disk_t * pxDisk,
                           FF_MMCSD_Config *config,
                           uint32_t partitionNum )
{
    FF_Error_t xError;
    FF_CreationParameters_t xParameters;

    if( (pxDisk != NULL) && (config != NULL) )
    {
        /* Start with every member of the structure set to zero. */
        memset( pxDisk, '\0', sizeof( FF_Disk_t ) );

        /* Set the pvTag to be MMCSD config */
        pxDisk->pvTag = ( void * )config;

        MMCSD_Handle deviceHandle = MMCSD_getHandle(config->mmcsdInstance);
        uint32_t ulSectorCount = MMCSD_getBlockCount(deviceHandle);

        /* The signature is used by the disk read and disk write functions to
         * ensure the disk being accessed is a MMCSD disk. */
        pxDisk->ulSignature = MMCSD_SIGNATURE;

        /* The number of sectors is recorded for bounds checking in the read and
         * write functions. */
        pxDisk->ulNumberOfSectors = ulSectorCount;

        /* Create the IO manager that will be used to control the SD disk. */
        memset( &xParameters, '\0', sizeof( xParameters ) );
        xParameters.pucCacheMemory = NULL;
        xParameters.ulMemorySize = MMCSD_IOMAN_CACHE_SIZE;
        xParameters.ulSectorSize = MMCSD_SECTOR_SIZE;
        xParameters.fnWriteBlocks = prvWriteMmcsd;
        xParameters.fnReadBlocks = prvReadMmcsd;
        xParameters.pxDisk = pxDisk;

        /* Driver is reentrant so xBlockDeviceIsReentrant can be set to pdTRUE.
         * In this case the semaphore is only used to protect FAT data
         * structures. */
        xParameters.pvSemaphore = NULL;
        xParameters.xBlockDeviceIsReentrant = pdFALSE;

        pxDisk->pxIOManager = FF_CreateIOManger( &xParameters, &xError );

        if( ( pxDisk->pxIOManager != NULL ) && ( FF_isERR( xError ) == pdFALSE ) )
        {
            /* Record that the SD disk has been initialized. */
            pxDisk->xStatus.bIsInitialised = pdTRUE;

            /* If the media is partitioned already, then mount it. If not, later use the APIs
             * FF_MMCSDCreateAndFormatPartition() and FF_MMCSDMountPartition() to do so.
             */
            /* Record the partition number the FF_Disk_t structure is, then
             * mount the partition. */
            pxDisk->xStatus.bPartitionNumber = partitionNum;

            /* Mount the partition. */
            xError = FF_Mount( pxDisk, partitionNum );
            FF_PRINTF( "FF_SDDiskInit: FF_Mount: %s\n", ( const char * ) FF_GetErrMessage( xError ) );

            if( FF_isERR( xError ) == pdFALSE )
            {
                /* The partition mounted successfully, add it to the virtual
                 * file system - where it will appear as a directory off the file
                 * system's root directory. */
                FF_FS_Add( pcName, pxDisk );
            }
        }
        else
        {
            FF_PRINTF( "FF_SDDiskInit: FF_CreateIOManger: %s\n", ( const char * ) FF_GetErrMessage( xError ) );

            /* The disk structure was allocated, but the disk's IO manager could
             * not be allocated, so free the disk again. */
            FF_MMCSDDiskDelete( pxDisk );
            pxDisk = NULL;
        }
    }
    else
    {
        FF_PRINTF( "FF_SDDiskInit: Malloc failed\n" );
    }

    return pxDisk;
}

BaseType_t FF_MMCSDDiskDelete( FF_Disk_t * pxDisk )
{
    if( pxDisk != NULL )
    {
        pxDisk->ulSignature = 0;
        pxDisk->xStatus.bIsInitialised = 0;

        if( pxDisk->pxIOManager != NULL )
        {
            FF_DeleteIOManager( pxDisk->pxIOManager );
        }
    }

    return pdPASS;
}

FF_Error_t FF_MMCSDCreateAndFormatPartition( FF_Disk_t *pxDisk , uint32_t sectorCount )
{
    FF_Error_t xError = pdPASS;

    if( pxDisk != NULL )
    {
        uint32_t partitionNum = pxDisk->xStatus.bPartitionNumber;

        FF_PartitionParameters_t xPartition;
        /* Create a single partition that fills the sectors provided. */
        memset( &xPartition, '\0', sizeof( xPartition ) );
        xPartition.ulSectorCount = sectorCount;
        xPartition.ulHiddenSectors = MMCSD_HIDDEN_SECTOR_COUNT;
        xPartition.xPrimaryCount = MMCSD_PRIMARY_PARTITIONS;
        xPartition.eSizeType = eSizeIsQuota;

        /* Partition the disk */
        xError = FF_Partition( pxDisk, &xPartition );

        if( FF_isERR( xError ) == pdFALSE )
        {
            /* Format the partition */
            xError = FF_Format( pxDisk, partitionNum, pdTRUE, pdTRUE );
        }
    }

    return xError;
}

FF_Error_t FF_MMCSDMountPartition( FF_Disk_t *pxDisk , char* partitionName )
{
    FF_Error_t xError = pdPASS;

    if(pxDisk != NULL)
    {
        uint32_t partitionNum = pxDisk->xStatus.bPartitionNumber;

        if(pxDisk->pxIOManager != NULL)
        {
            /* Mount the partition. */
            xError = FF_Mount( pxDisk, partitionNum );

            FF_PRINTF( "FF_MMCSDMountPartition: FF_Mount: %s\n", ( const char * ) FF_GetErrMessage( xError ) );

            if( FF_isERR( xError ) == pdFALSE )
            {
                /* The partition mounted successfully, add it to the virtual
                 * file system - where it will appear as a directory off the file
                 * system's root directory. */
                FF_FS_Add( partitionName, pxDisk );
            }
        }
        else
        {
            /* IOManager not found */
            xError = pdFAIL;
        }
    }
    else
    {
        /* NULL Disk Structure */
        xError = pdFAIL;
    }

    return xError;
}

BaseType_t FF_MMCSDGetPartitionDetails( FF_Disk_t *pxDisk, FF_MMCSD_PartitionDetails *partitionDetails )
{
    FF_Error_t xError;
    uint64_t ullFreeSectors;
    uint32_t ulTotalSizeKB, ulFreeSizeKB;
    FF_IOManager_t * pxIOManager;
    const char * pcTypeName = "unknown type";
    BaseType_t xReturn = pdPASS;

    if( (pxDisk == NULL) || (partitionDetails == NULL))
    {
        xReturn = pdFAIL;
    }
    else
    {
        pxIOManager = pxDisk->pxIOManager;

        switch( pxIOManager->xPartition.ucType )
        {
            case FF_T_FAT12:
                pcTypeName = "FAT12";
                break;

            case FF_T_FAT16:
                pcTypeName = "FAT16";
                break;

            case FF_T_FAT32:
                pcTypeName = "FAT32";
                break;

            default:
                pcTypeName = "UNKNOWN";
                break;
        }

        FF_GetFreeSize( pxIOManager, &xError );

        ullFreeSectors = pxIOManager->xPartition.ulFreeClusterCount * pxIOManager->xPartition.ulSectorsPerCluster;

        ulTotalSizeKB = pxIOManager->xPartition.ulDataSectors / 2U;
        ulFreeSizeKB = ( uint32_t ) ( ullFreeSectors / 2U );

        partitionDetails->partitionNumber    = pxDisk->xStatus.bPartitionNumber;
        partitionDetails->partitionType      = pcTypeName;
        partitionDetails->volumeLabel        = pxIOManager->xPartition.pcVolumeLabel;
        partitionDetails->sectorCount        = pxIOManager->xPartition.ulTotalSectors;
        partitionDetails->sectorsPerCluster  = pxIOManager->xPartition.ulSectorsPerCluster;
        partitionDetails->partitionSize      = ulTotalSizeKB;
        partitionDetails->partitionFreeSize  = ulFreeSizeKB;
    }

    return xReturn;
}

static int32_t prvWriteMmcsd( uint8_t * pucSource,
                            uint32_t ulSectorNumber,
                            uint32_t ulSectorCount,
                            FF_Disk_t * pxDisk )
{
    int32_t lReturn = FF_ERR_NONE;

    if( pxDisk != NULL )
    {
        FF_MMCSD_Config *cfg = (FF_MMCSD_Config *)(pxDisk->pvTag);
        MMCSD_Handle deviceHandle = MMCSD_getHandle(cfg->mmcsdInstance);

        if( pxDisk->ulSignature != MMCSD_SIGNATURE )
        {
            /* The disk structure is not valid because it doesn't contain a
             * magic number written to the disk when it was created. */
            lReturn = FF_ERR_IOMAN_DRIVER_FATAL_ERROR | FF_ERRFLAG;
        }
        else if( pxDisk->xStatus.bIsInitialised == pdFALSE )
        {
            /* The disk has not been initialized. */
            lReturn = FF_ERR_IOMAN_OUT_OF_BOUNDS_WRITE | FF_ERRFLAG;
        }
        else if( ulSectorNumber >= pxDisk->ulNumberOfSectors )
        {
            /* The start sector is not within the bounds of the disk. */
            lReturn = ( FF_ERR_IOMAN_OUT_OF_BOUNDS_WRITE | FF_ERRFLAG );
        }
        else if( ( pxDisk->ulNumberOfSectors - ulSectorNumber ) < ulSectorCount )
        {
            /* The end sector is not within the bounds of the disk. */
            lReturn = ( FF_ERR_IOMAN_OUT_OF_BOUNDS_WRITE | FF_ERRFLAG );
        }
        else
        {
            /* Copy the data to the MMCSD Device */
            MMCSD_write(deviceHandle, ( uint8_t * ) pucSource, ulSectorNumber, ulSectorCount );

            lReturn = FF_ERR_NONE;
        }
    }
    else
    {
        lReturn = FF_ERR_NULL_POINTER | FF_ERRFLAG;
    }

    return lReturn;
}

static int32_t prvReadMmcsd( uint8_t * pucDestination,
                           uint32_t ulSectorNumber,
                           uint32_t ulSectorCount,
                           FF_Disk_t * pxDisk )
{
    int32_t lReturn;

    if( pxDisk != NULL )
    {
        FF_MMCSD_Config *cfg = (FF_MMCSD_Config *)(pxDisk->pvTag);
        MMCSD_Handle deviceHandle = MMCSD_getHandle(cfg->mmcsdInstance);

        if( pxDisk->ulSignature != MMCSD_SIGNATURE )
        {
            /* The disk structure is not valid because it doesn't contain a
             * magic number written to the disk when it was created. */
            lReturn = FF_ERR_IOMAN_DRIVER_FATAL_ERROR | FF_ERRFLAG;
        }
        else if( pxDisk->xStatus.bIsInitialised == pdFALSE )
        {
            /* The disk has not been initialized. */
            lReturn = FF_ERR_IOMAN_OUT_OF_BOUNDS_WRITE | FF_ERRFLAG;
        }
        else if( ulSectorNumber >= pxDisk->ulNumberOfSectors )
        {
            /* The start sector is not within the bounds of the disk. */
            lReturn = ( FF_ERR_IOMAN_OUT_OF_BOUNDS_WRITE | FF_ERRFLAG );
        }
        else if( ( pxDisk->ulNumberOfSectors - ulSectorNumber ) < ulSectorCount )
        {
            /* The end sector is not within the bounds of the disk. */
            lReturn = ( FF_ERR_IOMAN_OUT_OF_BOUNDS_WRITE | FF_ERRFLAG );
        }
        else
        {
            /* Copy the data from the SD card */
            MMCSD_read(deviceHandle, ( uint8_t * ) pucDestination, ulSectorNumber, ulSectorCount);

            lReturn = FF_ERR_NONE;
        }
    }
    else
    {
        lReturn = FF_ERR_NULL_POINTER | FF_ERRFLAG;
    }

    return lReturn;
}
