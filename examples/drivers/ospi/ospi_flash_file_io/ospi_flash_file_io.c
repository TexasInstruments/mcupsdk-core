/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <board/flash.h>

/*File write counter need to reach this value*/
#define APP_OSPI_FILE_WRITE_COUNT 32

extern struct lfs_config gLfsCfg[CONFIG_LFS_NUM_INSTANCES];
extern LFS_FLASH_Config *gLfsFlashConfig;

void ospi_flash_file_io_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    lfs_t lfs[CONFIG_LFS_NUM_INSTANCES];
    lfs_file_t file;
    lfs_dir_t dir;
    uint32_t fileWriteCounter = 0,prevFileWriteCounter;
    int err;
    uint32_t blk, page;

    Drivers_open();
    Board_driversOpen();

    for(int lfs_inst = 0; lfs_inst < CONFIG_LFS_NUM_INSTANCES; lfs_inst++)
    {
        /* mount the filesystem*/
        err = lfs_mount(&lfs[lfs_inst], &gLfsCfg[lfs_inst]);

        /* reformat if not able to mount the filesystem
         */
        if (err) {
            DebugP_log("Mount failure!!\r\n");
            DebugP_log("Erasing the blocks as per given block count. This may take few minutes...\r\n");
            /* Mount failure. Erase flash blocks and try format and mount */
            Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], gLfsFlashConfig[lfs_inst].lfsFlashOffset, &blk, &page);
            for(uint32_t blkCount = 0; blkCount < gLfsCfg[lfs_inst].block_count; blkCount++)
            {
                Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk + blkCount);
            }
            DebugP_log("Formatting and remounting\r\n");
            lfs_format(&lfs[lfs_inst], &gLfsCfg[lfs_inst]);
            err = lfs_mount(&lfs[lfs_inst], &gLfsCfg[lfs_inst]);
            if(err)
            {   /* Mount failure */
                status = SystemP_FAILURE;
                DebugP_log("Mount failure even after trying recovery!!\r\n");
                goto exit;
            }
        }
        DebugP_log("APP_OSPI_FILE_WRITE_COUNT: %d\r\n",APP_OSPI_FILE_WRITE_COUNT);

        lfs_mkdir(&lfs[lfs_inst], "test");
        lfs_dir_open(&lfs[lfs_inst], &dir, "test");

        for(int i = 0;i<APP_OSPI_FILE_WRITE_COUNT;i++)
        {
            prevFileWriteCounter = fileWriteCounter;
            /* update boot count */
            fileWriteCounter++;

            lfs_file_open(&lfs[lfs_inst], &file, "test/test_file_write_count", LFS_O_RDWR | LFS_O_CREAT);
            lfs_file_rewind(&lfs[lfs_inst], &file);
            lfs_file_write(&lfs[lfs_inst], &file, &fileWriteCounter, sizeof(fileWriteCounter));

            /* remember the storage is not updated until the file is closed successfully */
            lfs_file_close(&lfs[lfs_inst], &file);

            fileWriteCounter = 0;
            lfs_file_open(&lfs[lfs_inst], &file, "test/test_file_write_count", LFS_O_RDWR );
            lfs_file_read(&lfs[lfs_inst], &file, &fileWriteCounter, sizeof(fileWriteCounter));
            lfs_file_close(&lfs[lfs_inst], &file);

            if(fileWriteCounter == prevFileWriteCounter + 1)
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
                goto exit;
            }

            DebugP_log("fileWriteCounter: %d\r\n", fileWriteCounter);
        }

        lfs_remove(&lfs[lfs_inst],"test/test_file_write_count"); //delete file
        lfs_dir_close(&lfs[lfs_inst], &dir);
        lfs_remove(&lfs[lfs_inst],"test"); //delete directory
        lfs_unmount(&lfs[lfs_inst]);
    }

 exit:
    if(SystemP_SUCCESS == status)
    {
        DebugP_log("fileWriteCounter reaches APP_OSPI_FILE_WRITE_COUNT\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("fileWriteCounter does not match APP_OSPI_FILE_WRITE_COUNT\r\n");
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}