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
 * \file  cli_main.c
 *
 * \brief This file contains the main task of the Enet CLI example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "l2_networking.h"
#include "ale_unicast.h"
#include "ale_vlan.h"
#include "gptp_stack.h"
#include "cli_lwip.h"
#include "enet_cli.h"

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

CLI_Command_Definition_t commandList[] =
        { { .pcCommand = "enet_addtxchn",
                .pcHelpString =
                        "enet_addtxchn <tx_ch_num>:\r\n Opens a Tx channel.\r\n\n",
                .pxCommandInterpreter = EnetCLI_openTxChn,
                .cExpectedNumberOfParameters = 1 },
            { .pcCommand = "enet_addrxchn",
                .pcHelpString =
                        "enet_addrxchn <rx_ch_num>:\r\n Opens an Rx channel.\r\n\n",
                .pxCommandInterpreter = EnetCLI_openRxChn,
                .cExpectedNumberOfParameters = 1 },
            { .pcCommand = "enet_sendraw",
                .pcHelpString =
                        "enet_sendraw <tx_ch_num> [-dm <dest_mac_addr>] [-sm <src_mac_addr>] [-v <vlan_id>] [-pcp <priority>] [-m <message>]:\r\n Sends a raw ethernet packet.\r\n\n",
                .pxCommandInterpreter = EnetCLI_transmitPkt,
                .cExpectedNumberOfParameters = -1 },
            { .pcCommand = "enet_capture",
                .pcHelpString =
                        "enet_capture {start | stop} <rx_ch_num>:\r\n Start/stop capturing incoming ethernet packets.\r\n\n",
                .pxCommandInterpreter = EnetCLI_capturePkt,
                .cExpectedNumberOfParameters = 2 },
            { .pcCommand = "enet_capturedump",
                .pcHelpString =
                        "enet_capturedump <rx_ch_num>:\r\n Returns the last 4 packets recieved at the specified channel.\r\n\n",
                .pxCommandInterpreter = EnetCLI_dumpRxBuffer,
                .cExpectedNumberOfParameters = 1 }, { .pcCommand = "quit",
                .pcHelpString = "quit:\r\n Closes the CLI application.\r\n\n",
                .pxCommandInterpreter = EnetCLI_quitTerminal,
                .cExpectedNumberOfParameters = 0 },
            { .pcCommand = "enet_adducast",
                .pcHelpString =
                        "enet_adducast <mac_addr> [-d]:\r\n Adds a unicast ALE entry with the given MAC address.\r\n Use the -d tag to make the MAC address as the default source address for sending packets.\r\n\n",
                .pxCommandInterpreter = EnetCLI_addUcast,
                .cExpectedNumberOfParameters = -1 },
            { .pcCommand = "enet_remucast",
                .pcHelpString =
                        "enet_remucast <mac_addr>:\r\n Removes unicast ALE entry with the given MAC address.\r\n\n",
                .pxCommandInterpreter = EnetCLI_removeUcast,
                .cExpectedNumberOfParameters = 1 },
            { .pcCommand = "enet_addvlan",
                .pcHelpString =
                        "enet_addvlan <vlan_id> {<port1> ...}:\r\n Configures the given ports to the specified VLAN id.\r\n\n",
                .pxCommandInterpreter = EnetCLI_addVlan,
                .cExpectedNumberOfParameters = -1 },
            { .pcCommand = "enet_remvlan",
                .pcHelpString =
                        "enet_remvlan <vlan_id>:\r\n Removes VLAN config with specified ID.\r\n\n",
                .pxCommandInterpreter = EnetCLI_removeVlan,
                .cExpectedNumberOfParameters = 1 },
            { .pcCommand = "enet_ptpd",
                .pcHelpString =
                        "enet_ptpd {start | stop} <dma_channel> [-p1 <priority_1>] [-p2 <priority_2>] [-s <sync_interval>]:\r\n Start/stop gPTP stack. \r\n\n",
                .pxCommandInterpreter = EnetCLI_ptpService,
                .cExpectedNumberOfParameters = -1 },
            { .pcCommand = "enet_hostmacaddr",
                .pcHelpString =
                        "enet_hostmacaddr:\r\n Prints the host port MAC address. \r\n\n",
                .pxCommandInterpreter = EnetCLI_getHostMac,
                .cExpectedNumberOfParameters = 0 },
            { .pcCommand = "lwip_shell", .pcHelpString =
                    "lwip_shell {start | stop}:\r\n Opens an LwIP shell.\r\n\n",
                .pxCommandInterpreter = EnetCLI_lwipShell,
                .cExpectedNumberOfParameters = 1 } };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetCLI_mainTask(void *args)
{
    Drivers_open();
    Board_driversOpen();

    /* Initialize UART transaction */
    UART_Transaction_init(&UART_trans);

    /* Initialize Enet Drivers */
    EnetApp_init();

    /* Add commands provided by enet_cli library */
    EnetCli_init(EnetApp_inst.enetType, EnetApp_inst.instId);

    /* Register custom commands */
    EnetCli_registerCustomCommands(commandList, sizeof(commandList)/sizeof(commandList[0]));

    /* Register commands that are built in in enet_cli lib */
    EnetCli_registerBuiltInCommands();

    bool xMoreDataToFollow;
    int8_t inTerminal = 1;

    /* Create read and write buffer */
    char txBuffer[MAX_WRITE_BUFFER_LEN] = "";
    char rxBuffer[MAX_READ_BUFFER_LEN] = "";

    /* Start of CLI application */
    UART_writeCLI("---------------------------------------\r\n");
    UART_writeCLI(
            "\nCLI for AM243x\r\nUse 'help' to list all available commands\r\n");

    /* Continuously take commands from the user and process the commands till application quits */
    while (inTerminal)
    {
        UART_writeCLI("\n> ");
        UART_readCLI(rxBuffer, MAX_READ_BUFFER_LEN);
        do
        {
            xMoreDataToFollow = EnetCli_processCommand(rxBuffer, txBuffer,
            MAX_WRITE_BUFFER_LEN);

            UART_writeCLI(txBuffer);
            memset(txBuffer, 0x00, MAX_WRITE_BUFFER_LEN);
        } while (xMoreDataToFollow);
        if (strcmp(rxBuffer, "quit") == 0)
        {
            inTerminal = 0;
        }
        memset(rxBuffer, 0x00, MAX_READ_BUFFER_LEN);
    }

    /* End of CLI Application */
    UART_writeCLI("\nCLI Application Closed\r\n");

    Board_driversClose();
    Drivers_close();
    return;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

/* None */
