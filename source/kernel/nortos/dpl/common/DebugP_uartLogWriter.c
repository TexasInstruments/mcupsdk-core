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

#include <drivers/uart.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include "printf.h"

uint32_t gDebugP_uartDrvIndex = 0xFFFFFFFF;

void DebugP_uartSetDrvIndex(uint32_t uartDrvIndex)
{
    gDebugP_uartDrvIndex = uartDrvIndex;
}

void DebugP_uartLogWriterPutLine(uint8_t *buf, uint16_t num_bytes)
{
    UART_Handle uartHandle = UART_getHandle(gDebugP_uartDrvIndex);
    if(uartHandle!=NULL)
    {
        UART_Transaction trans;
        UART_Transaction_init(&trans);
        trans.buf   = buf;
        trans.count = num_bytes;
        UART_write(uartHandle, &trans);
    }
}

void DebugP_uartLogWriterPutChar(char character)
{
    UART_Handle uartHandle = UART_getHandle(gDebugP_uartDrvIndex);
    if(uartHandle!=NULL)
    {
        UART_Transaction trans;
        UART_Transaction_init(&trans);
        trans.buf   = &character;
        trans.count = 1;
        UART_write(uartHandle, &trans);
    }
}

void DebugP_uartLogWriterPutCharBuffered(char character)
{
#define DebugP_UART_LOG_WRITER_LINE_BUF_SIZE (128u)
static uint8_t lineBuf[DebugP_UART_LOG_WRITER_LINE_BUF_SIZE+2]; /* +2 to add \r\n char at end of string in worst case */
static uint32_t lineBufIndex = 0;

    lineBuf[lineBufIndex++]=character;
    if( (character == '\n') ||
        (lineBufIndex >= (DebugP_UART_LOG_WRITER_LINE_BUF_SIZE)))
    {
        if(lineBufIndex >= (DebugP_UART_LOG_WRITER_LINE_BUF_SIZE))
        {
            /* add EOL */
            lineBuf[lineBufIndex++]='\r';
            lineBuf[lineBufIndex++]='\n';
        }
        if(lineBufIndex < 2)
        {
            /* only \n in buffer */
            /* if line did not terminate with \r followed by \n, then add the \r */
            lineBuf[lineBufIndex-1]='\r';
            lineBuf[lineBufIndex++]='\n';
        }
        if(lineBuf[lineBufIndex-2]!='\r')
        {
            /* if line did not terminate with \r followed by \n, then add the \r */
            lineBuf[lineBufIndex-1]='\r';
            lineBuf[lineBufIndex++]='\n';
        }
        /* flush line to UART */
        DebugP_uartLogWriterPutLine(lineBuf, lineBufIndex);
        lineBufIndex = 0;
    }
}

