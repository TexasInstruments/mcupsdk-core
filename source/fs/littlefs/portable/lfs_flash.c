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

/**
 *  \file lfs_flash.c
 *
 *  \brief LittleFS flash Driver API/interface implementation file.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "lfs_flash.h"

/* ========================================================================== */
/*                          Global Declarations                             */
/* ========================================================================== */
extern LFS_FLASH_Config *gLfsFlashConfig;

/* ========================================================================== */
/*                          Function Declarations                     */
/* ========================================================================== */
static int32_t LFS_getCurrentLfsInstance(const struct lfs_config *lfsCfg);


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* This function returns the LFS instance index/number of given
 * lfs_config structure pointer
 */
static int32_t LFS_getCurrentLfsInstance(const struct lfs_config *lfsCfg)
{
    int32_t lfsInst = SystemP_FAILURE;
    for(lfsInst = 0; lfsInst < gLfsFlashConfig->numLfsInstances; lfsInst++)
    {
        if(gLfsFlashConfig[lfsInst].lfsStructcfg == lfsCfg)
        {
            break;
        }
    }
    if(lfsInst == gLfsFlashConfig->numLfsInstances)
    {
        lfsInst = SystemP_FAILURE;
    }

    return lfsInst;
}

/* This api calculates the flash offset from the lfs block number, offset arguments. Then
 * reads the bytes from calculated offset as per the given size and stores into buffer
 */
int LFS_blockRead( const struct lfs_config *cfg, lfs_block_t block,
                            lfs_off_t offset,  void *buffer, lfs_size_t size)
{
    int32_t retVal;
    int32_t curLfsInst = LFS_getCurrentLfsInstance(cfg);
    if(curLfsInst != SystemP_FAILURE)
    {
        Flash_Handle flashHandle = Flash_getHandle(gLfsFlashConfig[curLfsInst].lfsFlashIndex);
        uint32_t flash_offset = gLfsFlashConfig[curLfsInst].lfsFlashOffset + block * cfg->block_size + offset;

        retVal = Flash_read(flashHandle, flash_offset, (uint8_t *)buffer, size);
    }
    else{
        retVal = SystemP_FAILURE;
    }

    /*Successful read operation must return 0*/
    return retVal;
}

/* This api calculates the flash offset from the lfs block number, offset arguments. Then
 * writes the bytes to calculated offset as per the given size
 */
int LFS_blockWrite( const struct lfs_config *cfg, lfs_block_t block,
                            lfs_off_t offset, const void *buffer, lfs_size_t size)
{
    int32_t retVal;
    int32_t curLfsInst = LFS_getCurrentLfsInstance(cfg);
    if(curLfsInst != SystemP_FAILURE)
    {
        Flash_Handle flashHandle = Flash_getHandle(gLfsFlashConfig[curLfsInst].lfsFlashIndex);
        uint32_t flash_offset = gLfsFlashConfig[curLfsInst].lfsFlashOffset + block * cfg->block_size + offset;

        retVal = Flash_write(flashHandle, flash_offset, (uint8_t*)buffer, size);
    }
    else{
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

/* This api erases the physical block */
int LFS_blockErase( const struct lfs_config *cfg, lfs_block_t block)
{
    int32_t retVal;
    uint32_t page, blk;
    int32_t curLfsInst = LFS_getCurrentLfsInstance(cfg);
    if(curLfsInst != SystemP_FAILURE)
    {
        Flash_Handle flashHandle = Flash_getHandle(gLfsFlashConfig[curLfsInst].lfsFlashIndex);
        uint32_t flash_offset = gLfsFlashConfig[curLfsInst].lfsFlashOffset + block * cfg->block_size;

        /* Actual block to be erased */
        Flash_offsetToBlkPage(flashHandle, flash_offset, &blk, &page);

        retVal = Flash_eraseBlk(flashHandle, blk);
    }
    else{
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

/* Sync implementation required by lfs */
int LFS_blockSync( const struct lfs_config *cfg)
{
    return 0;
}