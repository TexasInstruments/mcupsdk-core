/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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

/*!
 * \file  cli_main.c
 *
 * \brief This file contains the main task of the Enet CLI example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "cli_functions.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Maximum write buffer length for CLI */
#define MAX_WRITE_BUFFER_LEN 1000

/* Maximum read buffer length for CLI */
#define MAX_READ_BUFFER_LEN 500

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Function to read user input from the terminal */
static int8_t UART_readCLI(UART_Handle handle, UART_Transaction *transaction,
        char *rxBuffer, uint32_t rxBufferLen);

/* Function to display string on the terminal */
static int8_t UART_writeCLI(UART_Handle handle, UART_Transaction *transaction,
        char *txBuffer, uint32_t txBufferLen);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Destructive backspace */
char UART_destructiveBackSpc[3] = "\b \b";

/* End of line */
char UART_endOfLine[3] = "\r\n";

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetCLI_mainTask(void *args)
{
    Drivers_open();
    Board_driversOpen();
    memset(&EnetApp_Inst, 0, sizeof(EnetApp_Inst));

    /* Create a UART transaction */
    UART_Transaction transaction;
    UART_Transaction_init(&transaction);

    /* Create read and write buffer */
    char rxBuffer[MAX_READ_BUFFER_LEN] = "";
    char txBuffer[MAX_WRITE_BUFFER_LEN] = "";

    strncpy(txBuffer, "CLI for Sitara MCU (Testing)\r\n", MAX_WRITE_BUFFER_LEN);
    UART_writeCLI(gUartHandle[0], &transaction, txBuffer, MAX_WRITE_BUFFER_LEN);

    /* Dummy command */
    CLI_Command_Definition_t greetingCommand = { .pcCommand = "hello",
            .pcHelpString = "hello:\r\n Gives a greeting\r\n",
            .pxCommandInterpreter = EnetCLI_greet,
            .cExpectedNumberOfParameters = 0 };

    /* Command to quit CLI application */
    CLI_Command_Definition_t quitCommand = { .pcCommand = "quit",
            .pcHelpString = "quit:\r\n Closes CLI application\r\n",
            .pxCommandInterpreter = EnetCLI_quitTerminal,
            .cExpectedNumberOfParameters = 0 };

    /* Command to initialize ethernet drivers */
    CLI_Command_Definition_t initEnetCommand =
            { .pcCommand = "init",
                    .pcHelpString =
                            "init [<link_speed>]:\r\n Initializes drives for ethernet networking with the given link speed.\r\n Link speeds: 0->10Mbps, 1->100Mbps(default), 2->1Gbps, 3->Auto\r\n",
                    .pxCommandInterpreter = EnetCLI_init,
                    .cExpectedNumberOfParameters = -1 };

    /* Command to open Tx DMA channel */
    CLI_Command_Definition_t openTxDmaCommand = { .pcCommand = "openTxDma",
            .pcHelpString =
                    "openTxDma [<tx_ch_num>]:\r\n Opens a Tx DMA channel\r\n",
            .pxCommandInterpreter = EnetCLI_openTxDma,
            .cExpectedNumberOfParameters = -1 };

    /* Command to open Rx DMA channel */
    CLI_Command_Definition_t openRxDmaCommand = { .pcCommand = "openRxDma",
            .pcHelpString =
                    "openRxDma [<rx_ch_num>]:\r\n Opens a Rx DMA channel\r\n",
            .pxCommandInterpreter = EnetCLI_openRxDma,
            .cExpectedNumberOfParameters = -1 };

    /* Command to add Unicast entry to the ALE */
    CLI_Command_Definition_t addUcastCommand =
            { .pcCommand = "addUcast",
                    .pcHelpString =
                            "addUcast [-d] <mac_addr>:\r\n Adds a unicast ALE entry with the given MAC address.\r\n Use the -d tag to make the MAC address as the default source address for sending packets.\r\n",
                    .pxCommandInterpreter = EnetCLI_addUcast,
                    .cExpectedNumberOfParameters = -1 };

    /* Command to add classifier entry to the ALE */
    CLI_Command_Definition_t classifyCommand =
            { .pcCommand = "classify",
                    .pcHelpString =
                            "classify [-e <ether_type>] [-c <rx_ch_num>]:\r\n Adds a classifier entry with the given rules as filters and directs all filtered packets to the given channel. (default: 1)\r\n",
                    .pxCommandInterpreter = EnetCLI_addClassifier,
                    .cExpectedNumberOfParameters = -1 };

    /* Command to transmit packets from a specific channel to a specific MAC address */
    CLI_Command_Definition_t txPktCommand =
            { .pcCommand = "txPkt",
                    .pcHelpString =
                            "txPkt [-m <dest_mac_addr>] [-c <tx_ch_num>] [<message>]:\r\n Transmit the message to the destination MAC address (defaults: ff:ff:ff:ff:ff:ff) using the provided dma channel (default: 0)\r\n",
                    .pxCommandInterpreter = EnetCLI_transmitPkt,
                    .cExpectedNumberOfParameters = -1 };

    /* Command to activate a specific Rx channel to listen to incoming packets and store the packet info on a buffer */
    CLI_Command_Definition_t rxPktCommand =
            { .pcCommand = "rxPkt",
                    .pcHelpString =
                            "rxPkt [<rx_ch_num>]:\r\n Recieves messages from the given DMA channel (default: 0)\r\n",
                    .pxCommandInterpreter = EnetCLI_receivePkt,
                    .cExpectedNumberOfParameters = -1 };

    /* Command to deactivate a specific Rx channel from listening to an incoming packet */
    CLI_Command_Definition_t stopRxPktCommand =
            { .pcCommand = "stopRxPkt",
                    .pcHelpString =
                            "stopRxPkt [<rx_ch_num>]:\r\n Stops recieving packets from the given DMA channel (default: 0)\r\n",
                    .pxCommandInterpreter = EnetCLI_stopRxPkt,
                    .cExpectedNumberOfParameters = -1 };

    /* Command to display the packets captured by an Rx channel from the buffer */
    CLI_Command_Definition_t rxDumpCommand =
            { .pcCommand = "rxDump",
                    .pcHelpString =
                            "rxDump [<rx_ch_num>]:\r\n Returns the last 4 packets recieved by the given DMA channel (default: 0)\r\n",
                    .pxCommandInterpreter = EnetCLI_dumpRxBuffer,
                    .cExpectedNumberOfParameters = -1 };

    /* Command to display CPSW statistics */
    CLI_Command_Definition_t cpswStatsCommand = { .pcCommand =
            "dispCpswStats", .pcHelpString =
            "dispCpswStats:\r\n Displays CPSW statistics\r\n",
            .pxCommandInterpreter = EnetCLI_dispCpswStats,
            .cExpectedNumberOfParameters = 0 };

    /* Registering the commands to the CLI interpreter */
    FreeRTOS_CLIRegisterCommand(&greetingCommand);
    FreeRTOS_CLIRegisterCommand(&quitCommand);
    FreeRTOS_CLIRegisterCommand(&initEnetCommand);
    FreeRTOS_CLIRegisterCommand(&openTxDmaCommand);
    FreeRTOS_CLIRegisterCommand(&openRxDmaCommand);
    FreeRTOS_CLIRegisterCommand(&addUcastCommand);
    FreeRTOS_CLIRegisterCommand(&classifyCommand);
    FreeRTOS_CLIRegisterCommand(&txPktCommand);
    FreeRTOS_CLIRegisterCommand(&rxPktCommand);
    FreeRTOS_CLIRegisterCommand(&stopRxPktCommand);
    FreeRTOS_CLIRegisterCommand(&rxDumpCommand);
    FreeRTOS_CLIRegisterCommand(&cpswStatsCommand);

    BaseType_t xMoreDataToFollow;
    int8_t inTerminal = 1;

    /* Continuously take commands from the user and process the commands till application quits */
    while (inTerminal)
    {
        strncpy(txBuffer, "> ", MAX_WRITE_BUFFER_LEN);
        UART_writeCLI(gUartHandle[0], &transaction, txBuffer,
        MAX_WRITE_BUFFER_LEN);
        UART_readCLI(gUartHandle[0], &transaction, rxBuffer,
        MAX_READ_BUFFER_LEN);
        do
        {
            xMoreDataToFollow = FreeRTOS_CLIProcessCommand(rxBuffer, txBuffer,
            MAX_WRITE_BUFFER_LEN);

            UART_writeCLI(gUartHandle[0], &transaction, txBuffer,
            MAX_WRITE_BUFFER_LEN);
        } while (xMoreDataToFollow != pdFALSE);
        if (strcmp(rxBuffer, "quit") == 0)
        {
            inTerminal = 0;
        }
        memset(rxBuffer, 0x00, MAX_READ_BUFFER_LEN);
    }
    DebugP_log("\n\rFinished\r\n");

    Board_driversClose();
    Drivers_close();
    return;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static int8_t UART_readCLI(UART_Handle handle, UART_Transaction *transaction,
        char *rxBuffer, uint32_t rxBufferLen)
{
    char rxChar;
    uint8_t isEnd = 0;
    uint8_t isArrow = 0;
    int8_t transferOK;

    /* Initiate read */
    while (!isEnd)
    {
        transaction->count = 1;
        transaction->buf = (void*) &rxChar;
        transaction->args = NULL;
        transferOK = UART_read(handle, transaction);
        if (SystemP_SUCCESS != transferOK)
            return 1;

        /* End of command */
        if (rxChar == '\r' || strlen(rxBuffer) > rxBufferLen)
        {
            isEnd = 1;
            rxBuffer[strlen(rxBuffer)] = '\0';
            transaction->count = 3;
            transaction->buf = (void*) &UART_endOfLine;
            transaction->args = NULL;
            transferOK = UART_write(handle, transaction);
        }

        /* Handle backspaces */
        else if (rxChar == '\b')
        {
            if (strlen(rxBuffer) != 0)
            {
                transaction->count = 3;
                transaction->buf = (void*) &UART_destructiveBackSpc;
                transaction->args = NULL;
                transferOK = UART_write(handle, transaction);
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
            transaction->count = 1;
            transaction->buf = (void*) &rxChar;
            transaction->args = NULL;
            transferOK = UART_write(handle, transaction);
        }
        rxChar = 0x00;
    }
    return 0;
}

static int8_t UART_writeCLI(UART_Handle handle, UART_Transaction *transaction,
        char *txBuffer, uint32_t txBufferLen)
{
    int8_t transferOK;
    transaction->count = txBufferLen;
    transaction->buf = (void*) txBuffer;
    transaction->args = NULL;
    transferOK = UART_write(handle, transaction);
    if (SystemP_SUCCESS != transferOK)
        return 1;
    memset(txBuffer, 0x00, txBufferLen);
    return 0;
}
