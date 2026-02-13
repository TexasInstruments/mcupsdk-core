/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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
#include <drivers/bootloader/bootloader_uart.h>
#include <drivers/bootloader/bootloader_xmodem.h>

/* For memcpy */
#include <stdbool.h>
#include <string.h>
#include <drivers/utils/utils.h>
#include <kernel/dpl/CacheP.h>

static int32_t Uart_imgOpen(void *args, Bootloader_Params *params);
static int32_t Uart_imgRead(void *dst, uint32_t len, void *args);
static uint32_t Uart_imgGetCurOffset(void *args);
static void Uart_imgSeek(uint32_t location, void *args);
static void Uart_imgClose(void *handle, void *args);
static int32_t Uart_transferSegments(void *args);
static int32_t Uart_requestSegmentsFromHost(Bootloader_UartArgs *uartArgs);
static int32_t Uart_sendTransferComplete(Bootloader_UartArgs *uartArgs);

Bootloader_Fxns gBootloaderUartFxns = {
    .imgOpenFxn   = Uart_imgOpen,
    .imgReadFxn   = Uart_imgRead,
    .imgOffsetFxn = Uart_imgGetCurOffset,
    .imgSeekFxn   = Uart_imgSeek,
    .imgCloseFxn  = Uart_imgClose,
    .imgCustomFxn = Uart_transferSegments,
};

static int32_t Uart_imgOpen(void *args, Bootloader_Params *params)
{
    Bootloader_UartArgs *uartArgs = (Bootloader_UartArgs *)args;
    if(params != 0 && params->memArgsAppImageBaseAddr != BOOTLOADER_INVALID_ID)
    {
        uartArgs->appImageBaseAddr = params->memArgsAppImageBaseAddr;
        uartArgs->drvIdx = params->bufIoDeviceIndex;
    }
    uartArgs->curOffset = 0U;
    return SystemP_SUCCESS;
}

static int32_t Uart_imgRead(void *dst, uint32_t len, void *args)
{
    Bootloader_UartArgs *uartArgs = (Bootloader_UartArgs *)args;

    Utils_memcpyWord((uint8_t *)(uartArgs->appImageBaseAddr + uartArgs->curOffset), (uint8_t*)dst, len);
    CacheP_wbInv(dst, len, CacheP_TYPE_ALL);

    uartArgs->curOffset += len;
    return SystemP_SUCCESS;
}

static void Uart_imgSeek(uint32_t location, void *args)
{
    Bootloader_UartArgs *uartArgs = (Bootloader_UartArgs *)args;
    uartArgs->curOffset = location;
    return;
}

static uint32_t Uart_imgGetCurOffset(void *args)
{
    Bootloader_UartArgs *uartArgs = (Bootloader_UartArgs *)args;
    return uartArgs->curOffset;
}

static void Uart_imgClose(void *handle, void *args)
{
    return;
}

static int32_t Uart_transferSegments(void *args)
{
    int32_t status = SystemP_SUCCESS;
    Bootloader_UartArgs *uartArgs = (Bootloader_UartArgs *)args;

    uint8_t finalPacket = uartArgs->finalPacket;

    if(finalPacket != true)
    {
        /* Request Segments from Host */
        status = Uart_requestSegmentsFromHost(uartArgs);
    }
    else if(finalPacket == true)
    {
        /* All segments received, send the transfer complete ack*/
        status = Uart_sendTransferComplete(uartArgs);
    }

    return status;
}

static int32_t Uart_requestSegmentsFromHost(Bootloader_UartArgs *uartArgs)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t ioDrvIdx = uartArgs->drvIdx;
    uint32_t len = uartArgs->segmentSize;
    uint32_t dest = uartArgs->loadAddress;

    Bootloader_UartProtocolReq req;

    req.magic = BOOTLOADER_UART_MAGIC;
    req.virtMemOffset = uartArgs->virtMemOffset;
    req.cmd = BOOTLOADER_UART_SEND_FILE;
    req.len = len;

    uint32_t readBytes = 0U;

    /* Request for data */
    uint32_t retries = 32U;
    while(retries--)
    {
        status = Bootloader_xmodemTransmit(ioDrvIdx, (uint8_t *)&req, sizeof(Bootloader_UartProtocolReq));
        if(SystemP_SUCCESS == status)
        {
            break;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = Bootloader_xmodemReceive(ioDrvIdx,(uint8_t*) dest, len, &readBytes);
        CacheP_wb((void*)dest, (uint32_t)len, CacheP_TYPE_ALL);
    }

    return status;
}

static int32_t Uart_sendTransferComplete(Bootloader_UartArgs *uartArgs)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t ioDrvIdx = uartArgs->drvIdx;

    Bootloader_UartProtocolReq req;

    req.magic = BOOTLOADER_UART_MAGIC;
    req.virtMemOffset = 0U;
    req.cmd = BOOTLOADER_UART_FILE_RECEIVE_COMPLETE;
    req.len = 0U;

    /* Send transfer completion request */
    uint32_t retries = 32U;
    while(retries--)
    {
        status = Bootloader_xmodemTransmit(ioDrvIdx, (uint8_t *)&req, sizeof(Bootloader_UartProtocolReq));
        if(SystemP_SUCCESS == status)
        {
            break;
        }
    }

    return status;
}