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
 * \file  enet_cli_phy.c
 *
 * \brief This file contains the function definitions for phy related commands.
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
#include <enet_board.h>

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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCli_phyScan(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = 0;
    static uint32_t macPort = 0;
    Enet_IoctlPrms prms;
    EnetPhy_GenericInArgs inArgs;
    EnetPhy_Version phyInfo;
    const EnetBoard_PhyCfg *phyCfg;
    EnetBoard_EthPort ethPort;
    bool isAlive, isLinked;

    inArgs.macPort = macPort;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &isAlive);
    ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
            ENET_PHY_IOCTL_IS_ALIVE, &prms, status);
    if (status == 0)
    {
        if (isAlive)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &phyInfo);
            ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                    ENET_PHY_IOCTL_GET_ID, &prms, status);
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &isLinked);
            ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                    ENET_PHY_IOCTL_IS_LINKED, &prms, status);
            ethPort.boardId = EnetBoard_getId();
            ethPort.enetType = EnetInfo_inst.enetType;
            ethPort.instId = EnetInfo_inst.instId;
            ethPort.macPort = macPort;
            EnetBoard_getMiiConfig(&ethPort.mii);
            phyCfg = EnetBoard_getPhyCfg(&ethPort);
            snprintf(writeBuffer, writeBufferLen,
                    "\nMAC Port %d -->\r\n OUI: %d\r\n Model: %d\r\n Revision: %d\r\n MDIO Address: %d\r\n Linked: %s\r\n",
                    macPort, phyInfo.oui, phyInfo.model, phyInfo.revision,
                    phyCfg->phyAddr, (isLinked ? "Yes" : "No"));
        }
        else
            snprintf(writeBuffer, writeBufferLen,
                    "\nMAC Port %d -->\r\n No PHY device found\r\n", macPort);
    }
    else
        snprintf(writeBuffer, writeBufferLen,
                "\nMAC Port %d -->\r\n Scan failed\r\n", macPort);

    macPort++;
    if (macPort == EnetInfo_inst.numMacPorts)
    {
        macPort = 0;
        return pdFALSE;
    }
    return pdTRUE;
}

BaseType_t EnetCli_getPhyLinkStatus(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = 0;
    char speed[10] = "Auto";
    char duplexity[10] = "Auto";
    uint8_t macPort = 0;
    char *parameter;
    BaseType_t paramLen;
    Enet_IoctlPrms prms;
    EnetPhy_GenericInArgs inArgs;
    bool isLinked;
    EnetMacPort_LinkCfg linkCfg;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    macPort = atoi(parameter);

    if (macPort < 0 || macPort >= EnetInfo_inst.numMacPorts)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Mac Port\r\n");
        return pdFALSE;
    }

    inArgs.macPort = macPort;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &isLinked);
    ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
            ENET_PHY_IOCTL_IS_LINKED, &prms, status);
    if (isLinked)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &linkCfg);
        ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                ENET_PHY_IOCTL_GET_LINK_MODE, &prms, status);
        if (status == ENET_SOK)
        {
            if (linkCfg.speed == 0)
                strcpy(speed, "10Mbps");
            else if (linkCfg.speed == 1)
                strcpy(speed, "100Mbps");
            else if (linkCfg.speed == 2)
                strcpy(speed, "1Gbps");

            if (linkCfg.duplexity == 0)
                strcpy(duplexity, "Half");
            else if (linkCfg.duplexity == 1)
                strcpy(duplexity, "Full");

            snprintf(writeBuffer, writeBufferLen,
                    "MAC Port %d Link Status--> Speed: %s, Duplexity: %s\r\n",
                    macPort, speed, duplexity);
        }
    }
    else
        snprintf(writeBuffer, writeBufferLen,
                "PHY at MAC port %d is not linked\r\n", macPort);

    return pdFALSE;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

/* None */
