/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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

/*!
 * \file  cli_common.c
 *
 * \brief This file contains definitions of some commmon functions used by
 *        the CLI application.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Destructive backspace */
char UART_destructiveBackSpc[3] = "\b \b";

/* End of line */
char UART_endOfLine[3] = "\r\n";

/* UART transaction object */
UART_Transaction UART_trans;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void UART_readCLI(char *rxBuffer, uint32_t rxBufferLen)
{
    char rxChar;
    uint8_t isEnd = 0;
    uint8_t isArrow = 0;

    /* Initiate read */
    while (!isEnd)
    {
        UART_trans.count = 1;
        UART_trans.buf = (void*) &rxChar;
        UART_trans.args = NULL;
        UART_read(gUartHandle[0], &UART_trans);

        /* End of command */
        if (rxChar == '\r' || strlen(rxBuffer) > rxBufferLen)
        {
            isEnd = 1;
            rxBuffer[strlen(rxBuffer)] = '\0';
            UART_trans.count = 3;
            UART_trans.buf = (void*) &UART_endOfLine;
            UART_trans.args = NULL;
            UART_write(gUartHandle[0], &UART_trans);
        }

        /* Handle backspaces */
        else if (rxChar == '\b')
        {
            if (strlen(rxBuffer) != 0)
            {
                UART_trans.count = 3;
                UART_trans.buf = (void*) &UART_destructiveBackSpc;
                UART_trans.args = NULL;
                UART_write(gUartHandle[0], &UART_trans);
                rxBuffer[strlen(rxBuffer) - 1] = '\0';
            }
        }

        /* Ignore arrow keys */
        else if (rxChar == '\x1b' || isArrow != 0)
        {
            isArrow++;
            if (isArrow == 3)
                isArrow = 0;
        }

        /* Alphanumeric and special characters will be appended to the receive buffer */
        else
        {
            strncat(rxBuffer, &rxChar, 1);
            UART_trans.count = 1;
            UART_trans.buf = (void*) &rxChar;
            UART_trans.args = NULL;
            UART_write(gUartHandle[0], &UART_trans);
        }
        rxChar = 0x00;
    }
}

void UART_writeCLI(char *txBuffer)
{
    size_t txBufferLen = strlen(txBuffer);
    if (txBufferLen == 0)
        return;
    UART_trans.count = txBufferLen;
    UART_trans.buf = (void*) txBuffer;
    UART_trans.args = NULL;
    UART_write(gUartHandle[0], &UART_trans);
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

/* None */
