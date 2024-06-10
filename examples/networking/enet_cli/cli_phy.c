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
 * \file  cli_phy.c
 *
 * \brief This file contains all functions that are used to interact with the PHY ICs.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "cli_phy.h"

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

BaseType_t EnetCLI_showPhyAliveStatus(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int status = 0;
    static uint32_t phyAddr = 0;
    Enet_IoctlPrms prms;
    bool isAlive;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAddr, &isAlive);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            ENET_MDIO_IOCTL_IS_ALIVE, &prms, status);
    if (status == ENET_SOK)
    {
        if (isAlive)
            snprintf(writeBuffer, writeBufferLen, "PHY %d is alive\r\n",
                    phyAddr);
    }
    else
        snprintf(writeBuffer, writeBufferLen, "Failed to get PHY %d status\r\n",
                phyAddr);

    phyAddr++;
    if (phyAddr == ENET_MDIO_PHY_CNT_MAX)
    {
        phyAddr = 0;
        return pdFALSE;
    }
    return pdTRUE;
}

BaseType_t EnetCLI_showPhyLinkStatus(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int status = 0;
    static uint32_t phyAddr = 0;
    Enet_IoctlPrms prms;
    bool isAlive;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAddr, &isAlive);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            ENET_MDIO_IOCTL_IS_LINKED, &prms, status);
    if (status == ENET_SOK)
    {
        if (isAlive)
            snprintf(writeBuffer, writeBufferLen, "PHY %d is linked\r\n",
                    phyAddr);
    }
    else
        snprintf(writeBuffer, writeBufferLen, "Failed to get PHY %d status\r\n",
                phyAddr);

    phyAddr++;
    if (phyAddr == ENET_MDIO_PHY_CNT_MAX)
    {
        phyAddr = 0;
        return pdFALSE;
    }
    return pdTRUE;
}

BaseType_t EnetCLI_showPhyLinkMode(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int status = 0;
    char speed[10] = "Auto";
    char duplexity[10] = "Auto";
    static Enet_MacPort macPort = 0;
    Enet_IoctlPrms prms;
    EnetPhy_GenericInArgs inArgs;
    inArgs.macPort = macPort;
    EnetMacPort_LinkCfg linkCfg;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &linkCfg);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
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
                "MAC Port %d-> Speed: %s, Duplexity: %s\r\n", macPort, speed,
                duplexity);
    }
    else
        snprintf(writeBuffer, writeBufferLen,
                "Failed to get PHY link mode\r\n");

    macPort++;
    if (macPort == EnetApp_inst.numMacPorts)
    {
        macPort = 0;
        return pdFALSE;
    }
    return pdTRUE;
}

BaseType_t EnetCLI_showPhyRegs(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int status = 0;
    static Enet_MacPort macPort = 0;
    Enet_IoctlPrms prms;
    EnetPhy_GenericInArgs inArgs;
    inArgs.macPort = macPort;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            ENET_PHY_IOCTL_PRINT_REGS, &prms, status);
    if (status != ENET_SOK)
        snprintf(writeBuffer, writeBufferLen,
                "Failed to get PHY register data\r\n");

    macPort++;
    if (macPort == EnetApp_inst.numMacPorts)
    {
        macPort = 0;
        return pdFALSE;
    }
    return pdTRUE;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

/* None */
