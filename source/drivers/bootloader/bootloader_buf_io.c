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
#include <drivers/uart.h>
#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/bootloader/bootloader_buf_io.h>
/* For memcpy */
#include <string.h>
#include <kernel/dpl/CacheP.h>

#define BUFIO_VIRT_MEM_UNINIT (0xF055C0DEU)

static int32_t BufIo_imgOpen(void *args, Bootloader_Params *params);
static int32_t BufIo_imgRead(void *dst, uint32_t len, void *args);
static uint32_t BufIo_imgGetCurOffset(void *args);
static void BufIo_imgSeek(uint32_t location, void *args);
static void BufIo_imgClose(void *handle, void *args);

/* Private function declarations */
static int32_t BufIo_requestBufferFromIODevice(Bootloader_BufIoArgs *bufIoArgs, uint32_t virtMemOffset);

Bootloader_Fxns gBootloaderBufIoFxns = {
    .imgOpenFxn   = BufIo_imgOpen,
    .imgReadFxn   = BufIo_imgRead,
    .imgOffsetFxn = BufIo_imgGetCurOffset,
    .imgSeekFxn   = BufIo_imgSeek,
    .imgCloseFxn  = BufIo_imgClose,
};

static int32_t BufIo_imgOpen(void *args, Bootloader_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    Bootloader_BufIoArgs *bufIoArgs = (Bootloader_BufIoArgs *)args;

    bufIoArgs->curOffset = 0U;
    bufIoArgs->appImageOffset = 0U;
    bufIoArgs->virtMemOffset = BUFIO_VIRT_MEM_UNINIT;
    bufIoArgs->tempBufSize = params->bufIoTempBufSize;
    bufIoArgs->tempBuf = params->bufIoTempBuf;
    bufIoArgs->drvIdx = params->bufIoDeviceIndex;

    if(bufIoArgs->tempBuf == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        status = SystemP_SUCCESS;
    }

    return status;
}

static int32_t BufIo_imgRead(void *dst, uint32_t len, void *args)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_BufIoArgs *bufIoArgs = (Bootloader_BufIoArgs *)args;

    /* Save some typing, this is never gonna change :) */
    uint32_t bufSize = bufIoArgs->tempBufSize;

    /* Check if curOffset is in range of present temporary buf */
    if((bufIoArgs->curOffset >= bufIoArgs->virtMemOffset) &&
       (bufIoArgs->curOffset < (bufIoArgs->virtMemOffset + bufSize)) &&
       (bufIoArgs->virtMemOffset != BUFIO_VIRT_MEM_UNINIT))
    {
        /* continue, do nothing */
    }
    else
    {
        /* Request buffer */
        status = BufIo_requestBufferFromIODevice(bufIoArgs, bufIoArgs->curOffset);
        /* Update local virtual memory offset */
        bufIoArgs->virtMemOffset = bufIoArgs->curOffset;
    }

    /* Current offset is inside the temporary buffer. If it was freshly requested,
     * then the starting of temporary buffer would equal the current offset.
     * Now we need to check if the length of the data to be read is
     * less than temporary buffer size. If yes, do a simple memcopy.
     * Else, do a memcopy till end of temporary buffer, and request for
     * next chunk of data and update the virtMemOffset. Keep doing this
     * until remaining length is < temp buffer size.
     */
    uint32_t readableLen = bufSize - (bufIoArgs->curOffset);
    if(len <= readableLen)
    {
        /* Data in temp buffer, do memcopy */
        memcpy(dst, bufIoArgs->tempBuf + (bufIoArgs->curOffset - bufIoArgs->virtMemOffset), len);
    }
    else
    {
        /* Do memcopy till end of temp buffer */
        memcpy(dst, bufIoArgs->tempBuf + (bufIoArgs->curOffset - bufIoArgs->virtMemOffset), readableLen);

        uint32_t remainingLength = len - readableLen; /* This will be a +ve value since len > readableLen */
        uint32_t numCompleteIOReads = remainingLength / bufSize;
        uint32_t vMemOffset = bufIoArgs->virtMemOffset + bufSize;
        uint8_t *tempDst = (uint8_t *)dst;
        uint32_t i;

        for(i = 0U; i < numCompleteIOReads; i++)
        {
            /* Request buffer with new virtMemOffset */
            status = BufIo_requestBufferFromIODevice(bufIoArgs, vMemOffset);

            /* Do memcopy to the updated destination */
            memcpy(tempDst,  bufIoArgs->tempBuf, bufSize);

            /* Update variables */
            vMemOffset += bufSize;
            tempDst    += bufSize;
            remainingLength -= bufSize;
        }

        /* Request buffer with new virtMemOffset if remainingLength is now non-zero. If len < bufSize, this will always be the case */
        if(remainingLength != 0)
        {
            status = BufIo_requestBufferFromIODevice(bufIoArgs, vMemOffset);

            /* Do memcpy from start of tempBuf to remaining length */
            memcpy(tempDst, bufIoArgs->tempBuf, remainingLength);
        }
    }
    bufIoArgs->curOffset += len;

    return status;
}

static uint32_t BufIo_imgGetCurOffset(void *args)
{
    Bootloader_BufIoArgs *bufIoArgs = (Bootloader_BufIoArgs *)args;
    return bufIoArgs->curOffset;
}

static void BufIo_imgSeek(uint32_t location, void *args)
{
    Bootloader_BufIoArgs *bufIoArgs = (Bootloader_BufIoArgs *)args;
    bufIoArgs->curOffset = bufIoArgs->appImageOffset + location;
    return;
}

static void BufIo_imgClose(void *handle, void *args)
{
    return;
}

static int32_t BufIo_requestBufferFromIODevice(Bootloader_BufIoArgs *bufIoArgs, uint32_t virtMemOffset)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t ioDrvIdx = bufIoArgs->drvIdx;
    uint8_t *buf = bufIoArgs->tempBuf;
    uint32_t len = bufIoArgs->tempBufSize;

    Bootloader_BufIoProtocolReq req;

    req.magic = BOOTLOADER_BUF_IO_MAGIC;
    req.cmd = BOOTLOADER_BUF_IO_SEND_FILE;
    req.virtMemOffset = virtMemOffset;
    req.len = len;

    uint32_t readBytes = 0U;

    /* Request for file */
    uint32_t retries = 32U;
    while(retries--)
    {
        status = Bootloader_xmodemTransmit(ioDrvIdx, (uint8_t *)&req, sizeof(Bootloader_BufIoProtocolReq));
        if(SystemP_SUCCESS == status)
        {
            break;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = Bootloader_xmodemReceive(ioDrvIdx, buf, len, &readBytes);
    }

    return status;
}

int32_t BufIo_sendTransferComplete(uint32_t ioDrvIdx)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_BufIoProtocolReq req;

    req.magic = BOOTLOADER_BUF_IO_MAGIC;
    req.cmd = BOOTLOADER_BUF_IO_FILE_RECEIVE_COMPLETE;
    req.virtMemOffset = 0U;
    req.len = 0U;

    /* Send transfer completion request */
    uint32_t retries = 32U;
    while(retries--)
    {
        status = Bootloader_xmodemTransmit(ioDrvIdx, (uint8_t *)&req, sizeof(Bootloader_BufIoProtocolReq));
        if(SystemP_SUCCESS == status)
        {
            break;
        }
    }

    return status;
}
