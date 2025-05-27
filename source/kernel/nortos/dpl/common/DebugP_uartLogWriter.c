/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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

void DebugP_uartLogWriterPutLine(uint8_t *buf, uint16_t num_bytes);
void DebugP_uartLogWriterPutCharBuffered(char character);

uint32_t gDebugP_uartDrvIndex = 0xFFFFFFFFU;

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
        (void) UART_write(uartHandle, &trans);
    }
}

void DebugP_uartLogWriterPutChar(char character)
{
    char value = character;
    UART_Handle uartHandle = UART_getHandle(gDebugP_uartDrvIndex);
    if(uartHandle!=NULL)
    {
        UART_Transaction trans;
        UART_Transaction_init(&trans);
        trans.buf   = &value;
        trans.count = 1;
        (void) UART_write(uartHandle, &trans);
    }
}

void DebugP_uartLogWriterPutCharBuffered(char character)
{
#define DebugP_UART_LOG_WRITER_LINE_BUF_SIZE (128u)
static uint8_t lineBuf[DebugP_UART_LOG_WRITER_LINE_BUF_SIZE+UNSIGNED_INTEGERVAL_TWO]; /* +2 to add \r\n char at end of string in worst case */
static uint32_t lineBufIndex = 0;

    lineBuf[lineBufIndex] = (uint8_t)character;
	lineBufIndex = lineBufIndex + 1U;
    if( (character == '\n') ||
        (lineBufIndex >= (DebugP_UART_LOG_WRITER_LINE_BUF_SIZE)))
    {
        if(lineBufIndex >= (DebugP_UART_LOG_WRITER_LINE_BUF_SIZE))
        {
            /* add EOL */
            lineBuf[lineBufIndex]=(uint8_t)'\r';
			lineBufIndex = lineBufIndex + 1U;
            lineBuf[lineBufIndex]=(uint8_t)'\n';
			lineBufIndex = lineBufIndex + 1U;
        }
        if(lineBufIndex < UNSIGNED_INTEGERVAL_TWO)
        {
            /* only \n in buffer */
            /* if line did not terminate with \r followed by \n, then add the \r */
            lineBuf[lineBufIndex-1U]=(uint8_t)'\r';
            lineBuf[lineBufIndex]=(uint8_t)'\n';
			lineBufIndex = lineBufIndex + 1U;
        }
        if(lineBuf[lineBufIndex-UNSIGNED_INTEGERVAL_TWO]!=(uint8_t)'\r')
        {
            /* if line did not terminate with \r followed by \n, then add the \r */
            lineBuf[lineBufIndex-1U] = (uint8_t)'\r';
            lineBuf[lineBufIndex] = (uint8_t)'\n';
			lineBufIndex = lineBufIndex + 1U;
        }
        /* flush line to UART */
        DebugP_uartLogWriterPutLine(lineBuf,(uint16_t)lineBufIndex);
        lineBufIndex = 0;
    }
}

