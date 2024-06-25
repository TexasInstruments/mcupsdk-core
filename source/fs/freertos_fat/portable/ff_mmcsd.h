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

#ifndef __FF_MMCSD_H__

    #define __FF_MMCSD_H__

    #ifdef __cplusplus
        extern "C" {
    #endif

    #include "ff_headers.h"

    /* FF MMCSD Partition Struct. This is used during init time */
    typedef struct {

      uint32_t partitionNumber;
      /* Partition number of the media related to a particular instance of FF */

      char* partitionName;
      /* Partition name of the media related to a particular instance of FF */

    } FF_MMCSD_PartitionId;

    /* FF MMCSD Config */
    typedef struct {

      uint32_t mediaType;
      /* Media type of the MMCSD device connected - SD or EMMC */

      uint32_t mmcsdInstance;
      /* Instance number selected for the MMCSD device */

    } FF_MMCSD_Config;

    /* FF MMCSD Partition Details */
    typedef struct {

      uint32_t partitionNumber;
      /* Partition Number */

      const char *partitionType;
      /* Partition Type */

      char *volumeLabel;
      /* Volume label of the partition in PC */

      uint32_t sectorCount;
      /* Number of sectors/blocks in the partition */

      uint32_t sectorsPerCluster;
      /* Number of sectors/block per cluster (allocation unit) */

      uint32_t partitionSize;
      /* Size of the partition */

      uint32_t partitionFreeSize;
      /* Free size in the partition */
      
    } FF_MMCSD_PartitionDetails;

/* Create a SD disk */
    FF_Disk_t * FF_MMCSDDiskInit( char * pcName,
                              FF_Disk_t * pxDisk,
                              FF_MMCSD_Config *config,
                              uint32_t partitionNum );

/* Release all resources */
    BaseType_t FF_MMCSDDiskDelete( FF_Disk_t * pxDisk );

/* Create and format a partition, might be used for non-removable storage, like eMMC */
    FF_Error_t FF_MMCSDCreateAndFormatPartition( FF_Disk_t *pxDisk, 
                                              uint32_t sectorCount );

/* Mount the partition created using the FF_MMCSDPartitionAndFormatDisk API */
    FF_Error_t FF_MMCSDMountPartition( FF_Disk_t *pxDisk ,
                                       char* partitionName );

/* Get the partition details like volume label, free size etc */
    BaseType_t FF_MMCSDGetPartitionDetails( FF_Disk_t *pxDisk,
                                        FF_MMCSD_PartitionDetails *partitionDetails );

    #ifdef __cplusplus
        } /* extern "C" */
    #endif

#endif /* __FF_MMCSD_H__ */