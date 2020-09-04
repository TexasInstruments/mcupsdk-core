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
#include <kernel/dpl/CacheP.h>
#include "printf.h"

/* This globals are defined in SysConfig when Linux IPC is enabled, else these will remain undefined */
extern char gDebugMemLog[];
extern uint32_t gDebugMemLogSize;

static uint32_t gDebugMemLogWriteIndex = 0;
static volatile uint32_t gDebugMemLogIsWrapAround = 0;

static const char *gDebugP_memTraceLogWriterSelfCoreName = "unknown";

void DebugP_memLogWriterInit(uint16_t selfCoreId)
{
    gDebugP_memTraceLogWriterSelfCoreName = SOC_getCoreName(selfCoreId);
}

void DebugP_memTraceLogWriterPutLine(uint8_t *buf, uint16_t num_bytes)
{
    int32_t status = SystemP_SUCCESS;
    volatile uint32_t wr_idx;

    if (gDebugMemLogSize == 0)
    {
        status = SystemP_FAILURE;
    }
    if (SystemP_SUCCESS == status)
    {
        uint32_t copy_bytes, idx;
        uint8_t *dst;

        wr_idx = gDebugMemLogWriteIndex;
        dst = (uint8_t*)&gDebugMemLog[0];
        idx = 0;
        for (copy_bytes = 0; copy_bytes < num_bytes; copy_bytes++)
        {
            dst[wr_idx] = buf[idx];
            wr_idx ++;
            if (wr_idx >= gDebugMemLogSize)
            {
                /* flush to memory for linux to see */
                CacheP_wbInv(
                        &dst[gDebugMemLogWriteIndex],
                        (wr_idx - gDebugMemLogWriteIndex),
                        CacheP_TYPE_ALL);
                wr_idx = 0;
                gDebugMemLogWriteIndex = 0;
                gDebugMemLogIsWrapAround = 1;
            }
            idx ++;
        }
        /* flush to memory for linux to see */
        CacheP_wbInv(
                &dst[gDebugMemLogWriteIndex],
                (wr_idx - gDebugMemLogWriteIndex),
                CacheP_TYPE_ALL);

        gDebugMemLogWriteIndex = wr_idx;
    }
}

void DebugP_memLogWriterPutChar(char character)
{
#define DEBUGP_MEM_TRACE_LOG_WRITER_LINE_BUF_SIZE (120u)
static uint8_t lineBuf[DEBUGP_MEM_TRACE_LOG_WRITER_LINE_BUF_SIZE+2]; /* +2 to add \r\n char at end of string in worst case */
static uint32_t lineBufIndex = 0;

    if(lineBufIndex==0)
    {
        uint64_t curTime = ClockP_getTimeUsec();

        lineBufIndex = snprintf_((char*)lineBuf, DEBUGP_MEM_TRACE_LOG_WRITER_LINE_BUF_SIZE, "[%6s] %5d.%06ds : ",
                            gDebugP_memTraceLogWriterSelfCoreName,
                            (uint32_t)(curTime/1000000U),
                            (uint32_t)(curTime%1000000U)
                            );
    }
    lineBuf[lineBufIndex++]=character;
    if( (character == '\n') ||
        (lineBufIndex >= (DEBUGP_MEM_TRACE_LOG_WRITER_LINE_BUF_SIZE)))
    {
        if(lineBufIndex >= (DEBUGP_MEM_TRACE_LOG_WRITER_LINE_BUF_SIZE))
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
        DebugP_memTraceLogWriterPutLine(lineBuf, lineBufIndex);
        lineBufIndex = 0;
    }
}

