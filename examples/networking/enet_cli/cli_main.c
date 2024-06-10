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
#include "ale_classifier.h"
#include "ale_vlan.h"
#include "cli_debug.h"
#include "cli_phy.h"
#include "gptp_stack.h"

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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetCLI_mainTask(void *args)
{
    Drivers_open();
    Board_driversOpen();

    /* Initialize UART transaction */
    UART_Transaction_init(&UART_trans);

    /* Command to initialize ethernet drivers */
    CLI_Command_Definition_t initEnetCommand =
            { .pcCommand = "init",
                .pcHelpString =
                        "init [<link_speed>]:\r\n Initializes drives for ethernet networking with the given link speed.\r\n Link speeds: 0->10Mbps, 1->100Mbps(default), 2->1Gbps, 3->Auto\r\n\n",
                .pxCommandInterpreter = EnetCLI_init,
                .cExpectedNumberOfParameters = -1 };

    /* Command to open Tx DMA channel */
    CLI_Command_Definition_t openTxDmaCommand = { .pcCommand = "openTxDma",
        .pcHelpString =
                "openTxDma [<tx_ch_num>]:\r\n Opens a Tx DMA channel\r\n\n",
        .pxCommandInterpreter = EnetCLI_openTxDma,
        .cExpectedNumberOfParameters = -1 };

    /* Command to open Rx DMA channel */
    CLI_Command_Definition_t openRxDmaCommand = { .pcCommand = "openRxDma",
        .pcHelpString =
                "openRxDma [<rx_ch_num>]:\r\n Opens a Rx DMA channel\r\n\n",
        .pxCommandInterpreter = EnetCLI_openRxDma,
        .cExpectedNumberOfParameters = -1 };

    /* Command to transmit packets from a specific channel to a specific MAC address */
    CLI_Command_Definition_t txPktCommand =
            { .pcCommand = "txPkt",
                .pcHelpString =
                        "txPkt [-dm <dest_mac_addr>] [-sm <src_mac_addr>] [-c <tx_ch_num>] [<message>]:\r\n Transmit the message to the destination MAC address (defaults: ff:ff:ff:ff:ff:ff) using the provided dma channel (default: 0)\r\n\n",
                .pxCommandInterpreter = EnetCLI_transmitPkt,
                .cExpectedNumberOfParameters = -1 };

    /* Command to activate a specific Rx channel to listen to incoming packets and store the packet info on a buffer */
    CLI_Command_Definition_t rxPktCommand =
            { .pcCommand = "rxPkt",
                .pcHelpString =
                        "rxPkt [<rx_ch_num>]:\r\n Recieves messages from the given DMA channel (default: 0)\r\n\n",
                .pxCommandInterpreter = EnetCLI_receivePkt,
                .cExpectedNumberOfParameters = -1 };

    /* Command to deactivate a specific Rx channel from listening to an incoming packet */
    CLI_Command_Definition_t stopRxPktCommand =
            { .pcCommand = "stopRxPkt",
                .pcHelpString =
                        "stopRxPkt [<rx_ch_num>]:\r\n Stops recieving packets from the given DMA channel (default: 0)\r\n\n",
                .pxCommandInterpreter = EnetCLI_stopRxPkt,
                .cExpectedNumberOfParameters = -1 };

    /* Command to display the packets captured by an Rx channel from the buffer */
    CLI_Command_Definition_t rxDumpCommand =
            { .pcCommand = "rxDump",
                .pcHelpString =
                        "rxDump [<rx_ch_num>]:\r\n Returns the last 4 packets recieved by the given DMA channel (default: 0)\r\n\n",
                .pxCommandInterpreter = EnetCLI_dumpRxBuffer,
                .cExpectedNumberOfParameters = -1 };

    /* Command to quit CLI application */
    CLI_Command_Definition_t quitCommand = { .pcCommand = "quit",
        .pcHelpString = "quit:\r\n Closes CLI application\r\n\n",
        .pxCommandInterpreter = EnetCLI_quitTerminal,
        .cExpectedNumberOfParameters = 0 };

    /* Command to add Unicast entry to the ALE */
    CLI_Command_Definition_t addUcastCommand =
            { .pcCommand = "addUcast",
                .pcHelpString =
                        "addUcast [-d] <mac_addr>:\r\n Adds a unicast ALE entry with the given MAC address.\r\n Use the -d tag to make the MAC address as the default source address for sending packets.\r\n\n",
                .pxCommandInterpreter = EnetCLI_addUcast,
                .cExpectedNumberOfParameters = -1 };

    /* Command to add classifier entry to the ALE */
    CLI_Command_Definition_t classifyCommand =
            { .pcCommand = "classify",
                .pcHelpString =
                        "classify [-e <ether_type>] [-sm <src_mac_addr>] [-sd <dest_mac_addr>] [-p <port_num>] [-c <rx_ch_num>]:\r\n Adds a classifier entry with the given rules as filters and directs all filtered packets to the given channel. (default: 1)\r\n\n",
                .pxCommandInterpreter = EnetCLI_addClassifier,
                .cExpectedNumberOfParameters = -1 };

    /* Command to configure VLAN for ports */
    CLI_Command_Definition_t addVlanCommand =
            { .pcCommand = "addVlan",
                .pcHelpString =
                        "addVlan {<list_of_ports>} <vlan_id>:\r\n Configures the given ports to the specified VLAN id.\r\n\n",
                .pxCommandInterpreter = EnetCLI_addVlan,
                .cExpectedNumberOfParameters = 2 };

    /* Command to display CPSW statistics */
    CLI_Command_Definition_t cpswStatsCommand = { .pcCommand = "cpswStats",
        .pcHelpString =
                "cpswStats:\r\n Displays CPSW statistics on the console.\r\n\n",
        .pxCommandInterpreter = EnetCLI_showCpswStats,
        .cExpectedNumberOfParameters = 0 };

    /* Command to display alive PHYs */
    CLI_Command_Definition_t phyAliveCommand = { .pcCommand = "phyAlive",
        .pcHelpString = "phyAlive:\r\n Displays the PHYs that are alive.\r\n\n",
        .pxCommandInterpreter = EnetCLI_showPhyAliveStatus,
        .cExpectedNumberOfParameters = 0 };

    /* Command to display link status of PHYs */
    CLI_Command_Definition_t phyLinkCommand = { .pcCommand = "phyLink",
        .pcHelpString =
                "phyLink:\r\n Displays the PHYs that have been linked.\r\n\n",
        .pxCommandInterpreter = EnetCLI_showPhyLinkStatus,
        .cExpectedNumberOfParameters = 0 };

    /* Command to display PHY link mode */
    CLI_Command_Definition_t phyModeCommand =
            { .pcCommand = "phyMode",
                .pcHelpString =
                        "phyMode:\r\n Displays the link duplexity and speed of each PHY.\r\n\n",
                .pxCommandInterpreter = EnetCLI_showPhyLinkMode,
                .cExpectedNumberOfParameters = 0 };

    /* Command to display PHY regs */
    CLI_Command_Definition_t phyRegsCommand = { .pcCommand = "phyRegs",
        .pcHelpString =
                "phyRegs:\r\n Displays PHY registers on the console.\r\n\n",
        .pxCommandInterpreter = EnetCLI_showPhyRegs,
        .cExpectedNumberOfParameters = 0 };

    /* Command to display ALE table */
    CLI_Command_Definition_t aleDumpCommand =
            { .pcCommand = "aleDump",
                .pcHelpString =
                        "aleDump:\r\n Displays all ALE entries on the console. (Works in debug mode only)\r\n\n",
                .pxCommandInterpreter = EnetCLI_aleDump,
                .cExpectedNumberOfParameters = 0 };

    /* Command to start gPTP */
    CLI_Command_Definition_t startTsnCommand =
            { .pcCommand = "initTsn",
                .pcHelpString =
                        "initTsn [-p1 <priority_1>] [-p2 <priority_2>] [-s <sync_interval>] [-c <dma_channel>]:\r\n Starts gPTP stack with specified priority values (default: 248) and sync rate. (in power of 2, default: -3)\r\n\n",
                .pxCommandInterpreter = EnetCLI_startTsn,
                .cExpectedNumberOfParameters = -1 };

    /* Command to stop gPTP */
    CLI_Command_Definition_t stopTsnCommand = { .pcCommand = "stopTsn",
        .pcHelpString = "stopTsn:\r\n Stops the gPTP stack.\r\n\n",
        .pxCommandInterpreter = EnetCLI_stopTsn, .cExpectedNumberOfParameters =
                0 };

    /* Registering the commands to the CLI interpreter */
    FreeRTOS_CLIRegisterCommand(&initEnetCommand);
    FreeRTOS_CLIRegisterCommand(&openTxDmaCommand);
    FreeRTOS_CLIRegisterCommand(&openRxDmaCommand);
    FreeRTOS_CLIRegisterCommand(&txPktCommand);
    FreeRTOS_CLIRegisterCommand(&rxPktCommand);
    FreeRTOS_CLIRegisterCommand(&stopRxPktCommand);
    FreeRTOS_CLIRegisterCommand(&rxDumpCommand);
    FreeRTOS_CLIRegisterCommand(&quitCommand);
    FreeRTOS_CLIRegisterCommand(&addUcastCommand);
    FreeRTOS_CLIRegisterCommand(&classifyCommand);
    FreeRTOS_CLIRegisterCommand(&addVlanCommand);
    FreeRTOS_CLIRegisterCommand(&cpswStatsCommand);
    FreeRTOS_CLIRegisterCommand(&phyAliveCommand);
    FreeRTOS_CLIRegisterCommand(&phyLinkCommand);
    FreeRTOS_CLIRegisterCommand(&phyModeCommand);
    FreeRTOS_CLIRegisterCommand(&phyRegsCommand);
    FreeRTOS_CLIRegisterCommand(&aleDumpCommand);
    FreeRTOS_CLIRegisterCommand(&startTsnCommand);
    FreeRTOS_CLIRegisterCommand(&stopTsnCommand);

    BaseType_t xMoreDataToFollow;
    int8_t inTerminal = 1;

    /* Create read and write buffer */
    char rxBuffer[MAX_READ_BUFFER_LEN] = "";
    char txBuffer[MAX_WRITE_BUFFER_LEN] = "";

    /* Start of CLI application */
    UART_writeCLI(
            "CLI for AM243x\r\nUse 'help' to list all available commands\r\n");

    /* Continuously take commands from the user and process the commands till application quits */
    while (inTerminal)
    {
        UART_writeCLI("\n> ");
        UART_readCLI(rxBuffer, MAX_READ_BUFFER_LEN);
        do
        {
            xMoreDataToFollow = FreeRTOS_CLIProcessCommand(rxBuffer, txBuffer,
            MAX_WRITE_BUFFER_LEN);

            UART_writeCLI(txBuffer);
            memset(txBuffer, 0x00, MAX_WRITE_BUFFER_LEN);
        } while (xMoreDataToFollow != pdFALSE);
        if (strcmp(rxBuffer, "quit") == 0)
        {
            inTerminal = 0;
        }
        memset(rxBuffer, 0x00, MAX_READ_BUFFER_LEN);
    }

    /* End of CLI Application */
    UART_writeCLI("\nCLI Application Closed\r\n");
    DebugP_log("CLI Application Closed\r\n");

    Board_driversClose();
    Drivers_close();
    return;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

/* None */
