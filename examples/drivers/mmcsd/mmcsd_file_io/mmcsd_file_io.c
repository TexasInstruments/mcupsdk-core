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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <string.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define MMCSD_FILE_IO_FAT_PARTITION_SIZE (128U * 1024U * 1024U)

void mmcsd_file_io_main(void *args)
{
    int32_t status = SystemP_SUCCESS;

    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    DebugP_log("[MMCSD FILE IO] Starting...\r\n");

    FF_MMCSD_PartitionDetails partDetails;

    memset(&partDetails, 0, sizeof(FF_MMCSD_PartitionDetails));

    /* If the partition is not present, create it first - maybe 128 MB */

    /* Whenever an FF instance is added in SYSCONFIG, a partition number macro is generated,
     * which also specifies which media is connected to the MMCSD layer underneath. Here we try
     * to check for the 0th partition in SD Card, so we use the FF_PARTITION_SD0 macro.
     * Later when we give the pcName to the partition during mount, it has to match this. In this
     * case we give the mount point as /sd0. If it was FF_PARTITION_SD1, then /sd1.
     */
    FF_Disk_t *pDisk = &gFFDisks[FF_PARTITION_SD0];

    FF_MMCSDGetPartitionDetails(pDisk, &partDetails);

    if(partDetails.sectorCount == 0U)
    {
        /* No partition, create partition */
        uint32_t sectorCount = (MMCSD_FILE_IO_FAT_PARTITION_SIZE /  512U);

        FF_MMCSDCreateAndFormatPartition(pDisk, sectorCount);

        FF_MMCSDMountPartition(pDisk, "/sd0");

        /* Check the sector count now */
        FF_MMCSDGetPartitionDetails(pDisk, &partDetails);

        if(partDetails.sectorCount > 0U)
        {
            /* Success, can continue */
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }
    else
    {
        /* Partition is present. Can proceed further */
    }

    FF_FILE *fp;

    char fileName[] = "/sd0/file.dat";
    char fileData[] = "This is a test string. Hello, World!";
    char buf[100];

    /* Create file */
    fp = ff_fopen(fileName, "w+");

    /* Write file data */
    ff_fwrite(fileData, strlen(fileData)+1, 1, fp);

    /* Close file */
    ff_fclose(fp);

    /* Re-open now for reading */
    fp = ff_fopen(fileName, "r");

    /* Count chars */
    int fileLength = ff_filelength(fp);

    /* Read back file data */
    ff_fread(buf, fileLength, 1, fp);

    if(strncmp(fileData, buf, strlen(fileData)+1) != 0U)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    else
    {
        DebugP_log("All tests have passed!!\r\n");
    }

    /* Close file */
    ff_fclose(fp);

    /* Delete file */
    ff_remove(fileName);

    Board_driversClose();
    Drivers_close();
}
