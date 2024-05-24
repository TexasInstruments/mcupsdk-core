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

/*Include files */
#include "lfs_ospi.h"
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

/*Block read flash api wrapper */
static int lfsOspiBlockRead( const struct lfs_config *cfg, lfs_block_t block,
                            lfs_off_t offset,  void *buffer, lfs_size_t size);

/*Block write flash api wrapper */
static int lfsOspiBlockWrite( const struct lfs_config *cfg, lfs_block_t block,
                            lfs_off_t offset, const void *buffer, lfs_size_t size);

/*Block erase flash api wrapper */
static int lfsOspiBlockErase( const struct lfs_config *cfg, lfs_block_t block);

/*Sync flash api wrapper */
static int lfsOspiFlashSync( const struct lfs_config *cfg);

/* static variable to store handle to flash instance*/
static Flash_Handle flashHandleObject;

/*This function registers the block level apis*/
void lfsOspiInitOps(struct lfs_config *cfg, Flash_Handle flashHandleObj)
{
    cfg->read  = lfsOspiBlockRead;
    cfg->prog  = lfsOspiBlockWrite;
    cfg->erase = lfsOspiBlockErase;
    cfg->sync  = lfsOspiFlashSync;
    flashHandleObject = flashHandleObj;
}

/*This api calculates the flash offset from the lfs block number, offset arguments. Then
 * reads the bytes from calculated offset as per the given size and stores into buffer
 */
static int lfsOspiBlockRead( const struct lfs_config *cfg, lfs_block_t block,
                            lfs_off_t offset,  void *buffer, lfs_size_t size)
{
    int32_t retVal;
    uint32_t flash_offset = LFS_OSPI_FLASH_OFFSET_BASE + block * cfg->block_size + offset;

    retVal = Flash_read(flashHandleObject, flash_offset, buffer, size);

    /*Successful read operation must return 0*/
    return retVal;
}

/*This api calculates the flash offset from the lfs block number, offset arguments. Then
 writes the bytes to calculated offset as per the given size*/
static int lfsOspiBlockWrite( const struct lfs_config *cfg, lfs_block_t block,
                            lfs_off_t offset, const void *buffer, lfs_size_t size)
{
    int32_t retVal;
    uint32_t flash_offset = LFS_OSPI_FLASH_OFFSET_BASE + block * cfg->block_size + offset;

    retVal = Flash_write(flashHandleObject, flash_offset, (uint8_t*)buffer, size);
    CacheP_wbInvAll(CacheP_TYPE_ALLD);

    return retVal;
}

/* This api erases the physical block*/
static int lfsOspiBlockErase( const struct lfs_config *cfg, lfs_block_t block)
{
    int32_t retVal;
    uint32_t page, blk;
    uint32_t flash_offset = LFS_OSPI_FLASH_OFFSET_BASE + block * cfg->block_size;

    /*Actual block to be erased*/
    Flash_offsetToBlkPage(flashHandleObject, flash_offset, &blk, &page);

    retVal = Flash_eraseBlk(flashHandleObject, blk);

    return retVal;
}

/*Sync implementation required by lfs*/
static int lfsOspiFlashSync( const struct lfs_config *cfg)
{
    return 0;
}