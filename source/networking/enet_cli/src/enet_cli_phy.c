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

/* Networking Libraries */
#include <enet_board.h>

#include "enet_cli_mod.h"

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

static bool EnetCli_phyHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_phyScan(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_getPhyLinkStatus(char *writeBuffer,
        size_t writeBufferLen, const char *commandString);

static bool EnetCli_dumpPhyRegs(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_writePhyRegs(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_readPhyRegs(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

uint32_t EnetBoard_getId(void);

void EnetBoard_getMiiConfig(EnetMacPort_Interface *mii);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

bool EnetCli_phyCommandHandler(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    char *parameter;
    uint32_t paramLen;
    parameter = (char*) EnetCli_getParameter(commandString, 1, &paramLen);

    if (parameter == NULL || strncmp(parameter, "help", paramLen) == 0)
        return EnetCli_phyHelp(writeBuffer, writeBufferLen, commandString);
    else if (strncmp(parameter, "scan", paramLen) == 0)
        return EnetCli_phyScan(writeBuffer, writeBufferLen, commandString);
    else if (strncmp(parameter, "status", paramLen) == 0)
        return EnetCli_getPhyLinkStatus(writeBuffer, writeBufferLen,
                commandString);
    else if (strncmp(parameter, "dump", paramLen) == 0)
        return EnetCli_dumpPhyRegs(writeBuffer, writeBufferLen, commandString);
    else if (strncmp(parameter, "write", paramLen) == 0)
        return EnetCli_writePhyRegs(writeBuffer, writeBufferLen, commandString);
    else if (strncmp(parameter, "read", paramLen) == 0)
        return EnetCli_readPhyRegs(writeBuffer, writeBufferLen, commandString);
    else
    {
        snprintf(writeBuffer, writeBufferLen,
                "Bad argument\r\nFor more info run \'phy help\'\r\n");
        return false;
    }
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static bool EnetCli_phyHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    EnetAppUtils_print("Commands to access ethernet PHY.\r\nUsage:\r\n");
    EnetAppUtils_print("\tphy scan\t\t\t\tLists available PHYs.\r\n");
    EnetAppUtils_print(
            "\tphy status [<mac_port>]\t\t\tPrints PHY link status.\r\n");
    EnetAppUtils_print(
            "\t\t\t\t\t\tIf MAC port is not specified, prints link status for all PHYs.\r\n");
    EnetAppUtils_print(
            "\tphy dump <mac_port>\t\t\tPrints data of registers from PHY.\r\n");
    EnetAppUtils_print(
            "\tphy write <mac_port> <addr> <data>\tWrite to register in PHY.\r\n");
    EnetAppUtils_print(
            "\tphy read <mac_port> <addr>\t\tRead data of register from PHY.\r\n");
    EnetAppUtils_print("\tphy help\t\t\t\tPrints this message.\r\n");
    return false;
}

static bool EnetCli_phyScan(char *writeBuffer, size_t writeBufferLen,
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

    inArgs.macPort = (Enet_MacPort) macPort;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &isAlive);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            ENET_PHY_IOCTL_IS_ALIVE, &prms, status);
    if (status == 0)
    {
        if (isAlive)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &phyInfo);
            ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                    ENET_PHY_IOCTL_GET_ID, &prms, status);
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &isLinked);
            ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                    ENET_PHY_IOCTL_IS_LINKED, &prms, status);
            ethPort.boardId = EnetBoard_getId();
            ethPort.enetType = EnetCli_inst.enetType;
            ethPort.instId = EnetCli_inst.instId;
            ethPort.macPort = (Enet_MacPort) macPort;
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
    if (macPort == EnetCli_inst.numMacPorts)
    {
        macPort = 0;
        return false;
    }
    return true;
}

static bool EnetCli_getPhyLinkStatus(char *writeBuffer,
        size_t writeBufferLen, const char *commandString)
{
    int32_t status = 0;
    char speed[10] = "Auto";
    char duplexity[10] = "Auto";
    static uint8_t macPort = 0;
    char *parameter;
    uint32_t paramLen;
    Enet_IoctlPrms prms;
    EnetPhy_GenericInArgs inArgs;
    bool isLinked, allPorts = true;
    EnetMacPort_LinkCfg linkCfg;

    parameter = (char*) EnetCli_getParameter(commandString, 2, &paramLen);
    if (parameter != NULL)
    {
        macPort = atoi(parameter);
        if (macPort < 0 || macPort >= EnetCli_inst.numMacPorts)
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid Mac Port\r\n");
            return false;
        }
        allPorts = false;
    }

    inArgs.macPort = (Enet_MacPort) macPort;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &isLinked);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            ENET_PHY_IOCTL_IS_LINKED, &prms, status);
    if (isLinked)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &linkCfg);
        ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
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
                    "\nMAC Port %d Link Status -->\r\n Speed: %s\r\n Duplexity: %s\r\n",
                    macPort, speed, duplexity);
        }
    }
    else
        snprintf(writeBuffer, writeBufferLen,
                "\nMAC Port %d Link Status -->\r\n Not Linked\r\n", macPort);

    if (!allPorts)
    {
        macPort = 0;
        return false;
    }
    else
    {
        macPort++;
        if (macPort == EnetCli_inst.numMacPorts)
        {
            macPort = 0;
            return false;
        }
        return true;
    }
}

static bool EnetCli_dumpPhyRegs(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = 0;
    char *parameter;
    uint32_t paramLen;
    Enet_IoctlPrms prms;
    EnetPhy_GenericInArgs inArgs;

    parameter = (char*) EnetCli_getParameter(commandString, 2, &paramLen);
    if (parameter == NULL)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Missing argument(s)\r\nFor more info run \'phy help\'\r\n");
        return false;
    }
    inArgs.macPort = (Enet_MacPort) atoi(parameter);
    if (inArgs.macPort < 0 || inArgs.macPort >= EnetCli_inst.numMacPorts)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid MAC Port\r\n");
        return false;
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            ENET_PHY_IOCTL_PRINT_REGS, &prms, status);
    if (status != ENET_SOK)
        snprintf(writeBuffer, writeBufferLen,
                "Failed to get PHY register dump\r\n");

    return false;
}

static bool EnetCli_writePhyRegs(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    char *parameter;
    int32_t status;
    uint32_t paramLen;
    EnetPhy_WriteRegInArgs inArgs;
    Enet_IoctlPrms prms;

    for (uint32_t paramCnt = 2; paramCnt < 5; paramCnt++)
    {
        parameter = (char*) EnetCli_getParameter(commandString, paramCnt,
                &paramLen);
        if (parameter == NULL)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Missing argument(s)\r\nFor more info run \'phy help\'\r\n");
            return false;
        }
        if (paramCnt == 2)
        {
            inArgs.macPort = (Enet_MacPort) atoi(parameter);
            if (inArgs.macPort < 0
                    || inArgs.macPort >= EnetCli_inst.numMacPorts)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid MAC Port\r\n");
                return false;
            }
        }
        if (paramCnt == 3)
            inArgs.reg = (uint16_t) strtol(parameter, NULL, 16);
        if (paramCnt == 4)
            inArgs.val = (uint16_t) strtol(parameter, NULL, 16);
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            ENET_PHY_IOCTL_WRITE_REG, &prms, status);

    if (status != ENET_SOK)
        snprintf(writeBuffer, writeBufferLen, "Failed to write register\r\n");
    else
        snprintf(writeBuffer, writeBufferLen,
                "Wrote value %04x to register %04x\r\n", inArgs.val,
                inArgs.reg);

    return false;
}

static bool EnetCli_readPhyRegs(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    char *parameter;
    int32_t status;
    uint32_t paramLen;
    EnetPhy_ReadRegInArgs inArgs;
    uint16_t val;
    Enet_IoctlPrms prms;

    for (uint32_t paramCnt = 2; paramCnt < 4; paramCnt++)
    {
        parameter = (char*) EnetCli_getParameter(commandString, paramCnt,
                &paramLen);
        if (parameter == NULL)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Missing argument(s)\r\nFor more info run \'phy help\'\r\n");
            return false;
        }
        if (paramCnt == 2)
        {
            inArgs.macPort = (Enet_MacPort) atoi(parameter);
            if (inArgs.macPort < 0
                    || inArgs.macPort >= EnetCli_inst.numMacPorts)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid MAC Port\r\n");
                return false;
            }
        }
        if (paramCnt == 3)
            inArgs.reg = (uint16_t) strtol(parameter, NULL, 16);
    }

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &val);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            ENET_PHY_IOCTL_READ_REG, &prms, status);

    if (status != ENET_SOK)
        snprintf(writeBuffer, writeBufferLen, "Failed to read register\r\n");
    else
        snprintf(writeBuffer, writeBufferLen, "Value at %04x: %04x\r\n",
                inArgs.reg, val);

    return false;
}
