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

#include <fs/littlefs/portable/lfs_ospi.h>

/*File write counter need to reach this value*/
#define APP_OSPI_FILE_WRITE_COUNT 32

/*LFS configuration inputs*/
#define APP_OSPI_TEST_PAGE_SIZE   256
#define APP_OSPI_LFS_BLOCK_CYCLES 500
#define APP_OSPI_FILE_NAME_MAX    255
#define APP_OSPI_FILE_SIZE_MAX    16*1024U
#define APP_OSPI_ATTR_MAX         1022
#define APP_OSPI_METADATA_MAX     4096

extern Flash_Config gFlashConfig[CONFIG_FLASH_NUM_INSTANCES];

/*Static buffers for lfs*/
uint8_t __attribute__((aligned(8))) gLfsReadBuffer[APP_OSPI_TEST_PAGE_SIZE] = {0};
uint8_t __attribute__((aligned(8))) gLfsWriteBuffer[APP_OSPI_TEST_PAGE_SIZE] = {0};
uint8_t __attribute__((aligned(8))) gLfsLookaheadBuffer[APP_OSPI_TEST_PAGE_SIZE] = {0};

void ospi_flash_file_io_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    struct lfs_config lfsCfg;
    lfs_t lfs;
    lfs_file_t file;
    int err = 0;
    uint32_t blk, page;
    uint32_t offset = LFS_OSPI_FLASH_OFFSET_BASE;
    uint32_t fileWriteCounter = 0,prevFileWriteCounter;

    Drivers_open();
    Board_driversOpen();

    lfsOspiInitOps(&lfsCfg, gFlashHandle[CONFIG_FLASH0]);

    lfsCfg.read_size = APP_OSPI_TEST_PAGE_SIZE;
    lfsCfg.prog_size = APP_OSPI_TEST_PAGE_SIZE;
    lfsCfg.block_size = gFlashConfig[CONFIG_FLASH0].attrs->blockSize;
    lfsCfg.block_count = gFlashConfig[CONFIG_FLASH0].attrs->blockCount/2;
    lfsCfg.cache_size = APP_OSPI_TEST_PAGE_SIZE;
    lfsCfg.lookahead_size = APP_OSPI_TEST_PAGE_SIZE;
    lfsCfg.block_cycles = APP_OSPI_LFS_BLOCK_CYCLES;
    lfsCfg.compact_thresh = lfsCfg.block_size;
    lfsCfg.name_max = APP_OSPI_FILE_NAME_MAX;
    lfsCfg.file_max = APP_OSPI_FILE_SIZE_MAX;
    lfsCfg.attr_max = APP_OSPI_ATTR_MAX;
    lfsCfg.metadata_max = APP_OSPI_METADATA_MAX;
    lfsCfg.inline_max = APP_OSPI_TEST_PAGE_SIZE;
    lfsCfg.read_buffer = gLfsReadBuffer;
    lfsCfg.prog_buffer = gLfsWriteBuffer;
    lfsCfg.lookahead_buffer = gLfsLookaheadBuffer;

    /* mount the filesystem*/
    err = lfs_mount(&lfs, &lfsCfg);

    /* reformat if not able to mount the filesystem
     * this should only happen on the first boot
     */
    if (err) {
        DebugP_log("Mount failure!!\r\n");
        DebugP_log("Erasing the blocks as per given block count. This may take few minutes...\r\n");
        /* Mount failure. Erase flash blocks and try format and mount */
        Flash_offsetToBlkPage(gFlashHandle[CONFIG_FLASH0], offset, &blk, &page);
        for(uint32_t blkCount = 0; blkCount < lfsCfg.block_count; blkCount++)
        {
            Flash_eraseBlk(gFlashHandle[CONFIG_FLASH0], blk + blkCount);
        }
        DebugP_log("Formatting and remounting\r\n");
        lfs_format(&lfs, &lfsCfg);
        err = lfs_mount(&lfs, &lfsCfg);
        if(err)
        {   /* Mount failure */
            status = SystemP_FAILURE;
            DebugP_log("Mount failure even after trying recovery!!\r\n");
            goto exit;
        }
    }

    DebugP_log("APP_OSPI_FILE_WRITE_COUNT: %d\r\n",APP_OSPI_FILE_WRITE_COUNT);

    for(int i = 0;i<APP_OSPI_FILE_WRITE_COUNT;i++)
    {
        prevFileWriteCounter = fileWriteCounter;
        /* update boot count */
        fileWriteCounter++;

        lfs_file_open(&lfs, &file, "test_file_write_count", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_rewind(&lfs, &file);
        lfs_file_write(&lfs, &file, &fileWriteCounter, sizeof(fileWriteCounter));

        /* remember the storage is not updated until the file is closed successfully */
        lfs_file_close(&lfs, &file);

        fileWriteCounter = 0;
        lfs_file_open(&lfs, &file, "test_file_write_count", LFS_O_RDWR );
        lfs_file_read(&lfs, &file, &fileWriteCounter, sizeof(fileWriteCounter));
        lfs_file_close(&lfs, &file);

        /* release any resources we were using */
        lfs_unmount(&lfs);

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