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

#include <string.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_mmcsd_raw.h>
#include <drivers/mmcsd.h>
#include <kernel/dpl/CacheP.h>

static int32_t MMCSDRaw_imgOpen(void *args, Bootloader_Params *params);
static int32_t MMCSDRaw_imgRead(void *dst, uint32_t len, void *args);
static uint32_t MMCSDRaw_imgGetCurOffset(void *args);
static void MMCSDRaw_imgSeek(uint32_t location, void *args);
static void MMCSDRaw_imgClose(void *handle, void *args);

Bootloader_Fxns gBootloaderMmcsdFxns = {
    .imgOpenFxn   = MMCSDRaw_imgOpen,
    .imgReadFxn   = MMCSDRaw_imgRead,
    .imgOffsetFxn = MMCSDRaw_imgGetCurOffset,
    .imgSeekFxn   = MMCSDRaw_imgSeek,
    .imgCloseFxn  = MMCSDRaw_imgClose,
};

static int32_t MMCSDRaw_imgOpen(void *args, Bootloader_Params *params)
{
    Bootloader_MmcsdArgs *MMCSDArgs = (Bootloader_MmcsdArgs *)args;
    MMCSDArgs->curOffset = MMCSDArgs->appImageOffset;
    return SystemP_SUCCESS;
}

static int32_t MMCSDRaw_imgRead(void *dst, uint32_t len, void *args)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_MmcsdArgs *MMCSDArgs = (Bootloader_MmcsdArgs *)args;
    MMCSD_Handle handle = MMCSD_getHandle(MMCSDArgs->MMCSDIndex);

    if(handle == NULL)
	{
	   status = SystemP_FAILURE;
	}

    if(status == SystemP_SUCCESS)
    {
        status = MMCSD_enableBootPartition(handle, 1);
        if(status != SystemP_SUCCESS)
        {

        }
        else
        {
            status = Bootloader_MmcsdRaw_readFromOffset(handle, dst, len, MMCSDArgs->curOffset);

            if(status != SystemP_SUCCESS)
            {

            }
            else
            {
                CacheP_wb(dst, len, CacheP_TYPE_ALL);
                MMCSDArgs->curOffset += len;
            }
        }
    }

    return status;
}

static uint32_t MMCSDRaw_imgGetCurOffset(void *args)
{
    Bootloader_MmcsdArgs *MMCSDArgs = (Bootloader_MmcsdArgs *)args;
    return MMCSDArgs->curOffset;
}

static void MMCSDRaw_imgSeek(uint32_t location, void *args)
{
    Bootloader_MmcsdArgs *MMCSDArgs = (Bootloader_MmcsdArgs *)args;
    MMCSDArgs->curOffset = MMCSDArgs->appImageOffset + location;
    return;
}

static void MMCSDRaw_imgClose(void *handle, void *args)
{
    return;
}

int32_t Bootloader_MmcsdRaw_readFromOffset(MMCSD_Handle handle, void *dst, uint32_t len, uint32_t offset)
{
    uint32_t status = SystemP_SUCCESS;

    uint32_t blockSize = MMCSD_getBlockSize(handle);

    uint32_t offsetFromBlock = offset % blockSize;
    uint32_t numBlocks = (len + (blockSize -1) + offsetFromBlock)/blockSize;

    uint32_t blockStart = offset / blockSize;

    uint8_t tmpDst[blockSize];

    if(numBlocks == 1)
    {
        /* Read data to temp buffer */
        status = MMCSD_read(handle, tmpDst, blockStart, numBlocks);
        if(status != SystemP_SUCCESS)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            /* Copy the required length of data to the destination */
            memcpy(dst, tmpDst + offsetFromBlock, len);
        }
    }
    else
    {
        int32_t i = numBlocks;

        /* Read data from first block from MMCSD */
        {
            status = MMCSD_read(handle, tmpDst, blockStart, 1);
            if(status != SystemP_SUCCESS)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                i--;

                /* Copy required data from the first block to the destination */
                memcpy(dst, tmpDst + offsetFromBlock, blockSize - offsetFromBlock);

                dst = (uint8_t *)(dst) + (blockSize - offsetFromBlock);
            }
        }

        /* Read the middle blocks if any */
        if(status == SystemP_SUCCESS)
        {
            if(i != 1)
            {
                status = MMCSD_read(handle, (uint8_t *)dst, blockStart + 1, numBlocks - 2);
                if(status != SystemP_SUCCESS)
                {
                    status = SystemP_FAILURE;
                }
                else
                {
                    dst = (uint8_t *)(dst) + ((numBlocks -2) * blockSize);
                }
            }
        }

        /* Read data from the last block  */
        if(status == SystemP_SUCCESS)
        {
            status = MMCSD_read(handle, tmpDst, blockStart + (numBlocks - 1) , 1);
            if(status != SystemP_SUCCESS)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* Copy required data from the last block to the destination */
                if(((offsetFromBlock + len) % blockSize) == 0)
                {
                    memcpy(dst, tmpDst, blockSize);
                }
                else
                {
                    memcpy(dst, tmpDst, (offsetFromBlock + len) % blockSize );
                }
            }
        }
    }

    return status;

}

int32_t Bootloader_MmcsdRaw_writeToOffset(MMCSD_Handle handle, void *buf, uint32_t len, uint32_t offset)
{
    uint32_t status = SystemP_SUCCESS;

    uint32_t blockSize = MMCSD_getBlockSize(handle);

    uint32_t offsetFromBlock = offset % blockSize;
    uint32_t numBlocks = (len + (blockSize -1) + offsetFromBlock)/blockSize;

    uint32_t blockStart = offset / blockSize;

    uint8_t tmpBuf[blockSize];

    if(numBlocks == 1)
    {
        /* Read the block on to a temp buffer */
        status = MMCSD_read(handle, tmpBuf, blockStart, numBlocks);
        if(status != SystemP_SUCCESS)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            memcpy(tmpBuf + offsetFromBlock, buf, blockSize - offsetFromBlock);

            /* Write back the temp buffer to MMCSD */
            status = MMCSD_write(handle, tmpBuf, blockStart, numBlocks);
        }
    }
    else
    {
        int32_t i = numBlocks;

        /* Write the first block */
        {
            /* Read first block into tmp buffer */
            status = MMCSD_read(handle, tmpBuf, blockStart, 1);
            if(status != SystemP_SUCCESS)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                memcpy(tmpBuf + offsetFromBlock, buf, blockSize - offsetFromBlock);

                /* Write back the temp buffer to MMCSD */
                status = MMCSD_write(handle, tmpBuf, blockStart, 1);
                if(status != SystemP_SUCCESS)
                {
                    status = SystemP_FAILURE;
                }
                else
                {
                    buf = (uint8_t *)(buf) + (blockSize - offsetFromBlock);
                    i--;
                }
            }
        }

        /* Write the middle blocks, if any (Last block is not written here)*/
        if(status == SystemP_SUCCESS)
        {
            if(i != 1)
            {
                status = MMCSD_write(handle, (uint8_t *)buf, blockStart + 1, numBlocks - 2);
                if(status != SystemP_SUCCESS)
                {
                    status = SystemP_FAILURE;
                }
                else
                {
                    buf = (uint8_t *)(buf) + ((numBlocks -2) * blockSize);
                }
            }
        }

        /* Write the last block */
        if(status == SystemP_SUCCESS)
        {
            /* Read last block into tmp buffer */
            status = MMCSD_read(handle, tmpBuf, blockStart + (numBlocks -1), 1);
            if(status != SystemP_SUCCESS)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* Modify temp buffer with new data */
                if(((offsetFromBlock + len) % blockSize) == 0)
                {
                    memcpy(tmpBuf, buf, blockSize);
                }
                else
                {
                    memcpy(tmpBuf, buf, (offsetFromBlock + len) % blockSize);
                }

                /* Write back last block into MMCSD */
                status = MMCSD_write(handle, tmpBuf, blockStart + (numBlocks -1), 1);
            }
        }

    }

    return status;
}