/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <drivers/soc.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "printf.h"

static DebugP_ShmLog *gDebugShmLogWriter = NULL;
static const char *gDebugShmLogWriterSelfCoreName = "unknown";

void DebugP_shmLogWriterInit(DebugP_ShmLog *shmLog, uint16_t selfCoreId)
{
    gDebugShmLogWriterSelfCoreName = SOC_getCoreName(selfCoreId);
    gDebugShmLogWriter = shmLog;
    gDebugShmLogWriter->rdIndex = 0;
    gDebugShmLogWriter->wrIndex = 0;
    gDebugShmLogWriter->isValid = DebugP_SHM_LOG_IS_VALID;
}

void DebugP_shmLogWriterPutLine(uint8_t *buf, uint16_t num_bytes)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t max_bytes;
    volatile uint32_t wr_idx, rd_idx;

    if (gDebugShmLogWriter == NULL)
    {
        status = SystemP_FAILURE;
    }
    if (SystemP_SUCCESS == status)
    {
        wr_idx = gDebugShmLogWriter->wrIndex;
        if (wr_idx >= DebugP_SHM_LOG_SIZE)
        {
            status = SystemP_FAILURE; /* This condition should never happen */
        }
        rd_idx = gDebugShmLogWriter->rdIndex;
        if (rd_idx >= DebugP_SHM_LOG_SIZE)
        {
            status = SystemP_FAILURE; /* This condition should never happen */
        }
    }
    if (SystemP_SUCCESS == status)
    {
        if (wr_idx < rd_idx)
        {
            max_bytes = rd_idx - wr_idx;
        }
        else
        {
            max_bytes = (DebugP_SHM_LOG_SIZE - wr_idx) + rd_idx;
        }
        if (num_bytes > max_bytes)
        {
            status = SystemP_FAILURE;
        }
    }
    if (SystemP_SUCCESS == status)
    {
        uint32_t copy_bytes, idx;
        uint8_t *dst;

        dst = &gDebugShmLogWriter->buffer[0];
        idx = 0;
        for (copy_bytes = 0; copy_bytes < num_bytes; copy_bytes++)
        {
            dst[wr_idx] = buf[idx];
            wr_idx ++;
            if (wr_idx >= DebugP_SHM_LOG_SIZE)
            {
                wr_idx = 0;
            }
            idx ++;
        }
        gDebugShmLogWriter->wrIndex = wr_idx;
        /* dummy read to resure data is written to memory */
        wr_idx = gDebugShmLogWriter->wrIndex;
    }
}

void DebugP_shmLogWriterPutChar(char character)
{
#define DebugP_SHM_LOG_WRITER_LINE_BUF_SIZE (120u)
static uint8_t lineBuf[DebugP_SHM_LOG_WRITER_LINE_BUF_SIZE+2]; /* +2 to add \r\n char at end of string in worst case */
static uint32_t lineBufIndex = 0;

    if(lineBufIndex==0)
    {
        uint64_t curTime = ClockP_getTimeUsec();

        lineBufIndex = snprintf_((char*)lineBuf, DebugP_SHM_LOG_WRITER_LINE_BUF_SIZE, "[%6s] %5d.%06ds : ",
                            gDebugShmLogWriterSelfCoreName,
                            (uint32_t)(curTime/1000000U),
                            (uint32_t)(curTime%1000000U)
                            );
    }
    lineBuf[lineBufIndex++]=character;
    if( (character == '\n') ||
        (lineBufIndex >= (DebugP_SHM_LOG_WRITER_LINE_BUF_SIZE)))
    {
        if(lineBufIndex >= (DebugP_SHM_LOG_WRITER_LINE_BUF_SIZE))
        {
            /* add EOL */
            lineBuf[lineBufIndex++]='\r';
            lineBuf[lineBufIndex++]='\n';
        }
        if(lineBuf[lineBufIndex-2]!='\r')
        {
            /* if line did not terminate with \r followed by \n, then add the \r */
            lineBuf[lineBufIndex-1]='\r';
            lineBuf[lineBufIndex++]='\n';
        }
        /* flush line to shared memory */
        DebugP_shmLogWriterPutLine(lineBuf, lineBufIndex);
        lineBufIndex = 0;
    }
}

