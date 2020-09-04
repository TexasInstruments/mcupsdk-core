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

#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_flash.h>
#include <board/flash.h>
#include <kernel/dpl/CacheP.h>

static int32_t Flash_imgOpen(void *args, Bootloader_Params *params);
static int32_t Flash_imgRead(void *dst, uint32_t len, void *args);
static uint32_t Flash_imgGetCurOffset(void *args);
static void Flash_imgSeek(uint32_t location, void *args);
static void Flash_imgClose(void *handle, void *args);

Bootloader_Fxns gBootloaderFlashFxns = {
    .imgOpenFxn   = Flash_imgOpen,
    .imgReadFxn   = Flash_imgRead,
    .imgOffsetFxn = Flash_imgGetCurOffset,
    .imgSeekFxn   = Flash_imgSeek,
    .imgCloseFxn  = Flash_imgClose,
};

static int32_t Flash_imgOpen(void *args, Bootloader_Params *params)
{
    Bootloader_FlashArgs *flashArgs = (Bootloader_FlashArgs *)args;
    flashArgs->curOffset = flashArgs->appImageOffset;
    return SystemP_SUCCESS;
}

static int32_t Flash_imgRead(void *dst, uint32_t len, void *args)
{
    Bootloader_FlashArgs *flashArgs = (Bootloader_FlashArgs *)args;
    Flash_Handle handle = Flash_getHandle(flashArgs->flashIndex);
    Flash_Attrs *attrs = Flash_getAttrs(flashArgs->flashIndex);

    if(attrs != NULL)
    {
        uint32_t flashSize = attrs->blockCount * attrs->pageCount * attrs->pageSize;
        if(flashArgs->curOffset + len < flashSize)
        {
            Flash_read(handle, flashArgs->curOffset, (uint8_t *) dst, len);
            CacheP_wb(dst, len, CacheP_TYPE_ALL);
            flashArgs->curOffset += len;
        }
    }

    return SystemP_SUCCESS;
}

static uint32_t Flash_imgGetCurOffset(void *args)
{
    Bootloader_FlashArgs *flashArgs = (Bootloader_FlashArgs *)args;
    return flashArgs->curOffset;
}

static void Flash_imgSeek(uint32_t location, void *args)
{
    Bootloader_FlashArgs *flashArgs = (Bootloader_FlashArgs *)args;
    flashArgs->curOffset = flashArgs->appImageOffset + location;
    return;
}

static void Flash_imgClose(void *handle, void *args)
{
    return;
}
