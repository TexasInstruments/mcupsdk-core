/*
 * Copyright (C) 2021-2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Serial_Cmd_HAL.h"
#include "Serial_Cmd_Monitor.h"

/*
 * Variables for Serial Cmd Hardware Abstraction Layer
 */
uint32_t    gSerialCmdBaseAddr;
uint16_t    gSerialCmdCnt;
uint8_t     gSerialCmdRxChar;
int32_t     gSerialCmdTxChar;


void SerialCmd_init(void);
void SerialCmd_read(void);
void SerialCmd_send(int a);

void SerialCmd_init(void)
{
    gSerialCmdBaseAddr = UART_getBaseAddr(gUartHandle[CONFIG_UART_CONSOLE]);

    gSerialCmdCnt=0;
    gSerialCmdRxChar=0;
    gSerialCmdTxChar=0;
}

void SerialCmd_read(void)
{
    uint8_t *readBuf = &gSerialCmdRxChar;

    uint32_t isRxReady   = (uint32_t) FALSE;
    isRxReady = (uint32_t) UART_checkCharsAvailInFifo(gSerialCmdBaseAddr);
    if ((uint32_t) TRUE == isRxReady)
    {
        gSerialCmdCnt++;
        /* once the data is ready, read from the FIFO */
        *readBuf = (uint8_t) UART_getCharFifo(gSerialCmdBaseAddr, readBuf);

        receivedDataCommand(*readBuf);
    }

    return;
}

/* Transmit a character from the SCI */
void SerialCmd_send(int a)
{
    gSerialCmdTxChar = a;
    int *ptr= &gSerialCmdTxChar;


    uint32_t lineStatus            = 0U;
    int32_t maxTrialCount          = (int32_t) UART_TRANSMITEMPTY_TRIALCOUNT;

    /* Before we could write no of bytes, we should have
     * no of free buffers. Hence, we check for shiftregister
     * empty (ensure the FIFO is empty) to write num of bytes */
    do
    {
        lineStatus = (uint32_t) UART_readLineStatus(gSerialCmdBaseAddr);
        maxTrialCount--;
    }
    while (((uint32_t) (UART_LSR_TX_SR_E_MASK | UART_LSR_TX_FIFO_E_MASK) !=
            ((uint32_t) (UART_LSR_TX_SR_E_MASK |
                       UART_LSR_TX_FIFO_E_MASK) & lineStatus))
           && (0U < maxTrialCount));

    UART_putChar(gSerialCmdBaseAddr, *ptr);

    /* Ensure TX FIFO Empty before exiting transfer function */
    do
    {
        lineStatus = UART_readLineStatus(gSerialCmdBaseAddr);
    }
    while ((uint32_t) (UART_LSR_TX_FIFO_E_MASK |
                     UART_LSR_TX_SR_E_MASK) !=
           (lineStatus & (uint32_t) (UART_LSR_TX_FIFO_E_MASK |
                                   UART_LSR_TX_SR_E_MASK)));

    return;
}


