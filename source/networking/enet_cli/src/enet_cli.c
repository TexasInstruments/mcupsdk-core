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
 * \file  enet_cli.c
 *
 * \brief This file contains the function definitions for enet_cli library.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* Standard Libraries */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Networking Libraries */
#include <enet.h>

/* FreeRTOS Libraries */
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#include "enet_cli.h"
#include "enet_cli_phy.h"

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

/* Command to scan for PHYs */
static CLI_Command_Definition_t phyScanCommand = { .pcCommand = "phy_scan",
    .pcHelpString = "phy_scan:\r\n Lists available PHYs.\r\n\n",
    .pxCommandInterpreter = EnetCli_phyScan, .cExpectedNumberOfParameters = 0 };

/* Command to display PHY link mode */
static CLI_Command_Definition_t phyStatusCommand =
        { .pcCommand = "phy_status",
            .pcHelpString =
                    "phy_status <mac_port>:\r\n Prints PHY link status at the specified MAC port.\r\n\n",
            .pxCommandInterpreter = EnetCli_getPhyLinkStatus,
            .cExpectedNumberOfParameters = 1 };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetCli_init(Enet_Type enetType, uint32_t instId)
{
    /* Retrieve board info and enet handle */
    memset(&EnetInfo_inst, 0, sizeof(EnetInfo_Obj));
    EnetInfo_inst.enetType = enetType;
    EnetInfo_inst.instId = instId;
    EnetInfo_inst.coreId = Enet_getCoreId();
    EnetInfo_inst.hEnet = Enet_getHandle(enetType, instId);
    EnetInfo_inst.numMacPorts = Enet_getMacPortMax(enetType, instId);

    /* Register all commands */
    FreeRTOS_CLIRegisterCommand(&phyScanCommand);
    FreeRTOS_CLIRegisterCommand(&phyStatusCommand);
}

BaseType_t EnetCli_processCommand(const char *commandInput, char *writeBuffer,
        size_t writeBufferLen)
{
    return FreeRTOS_CLIProcessCommand(commandInput, writeBuffer, writeBufferLen);
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

/* None */
