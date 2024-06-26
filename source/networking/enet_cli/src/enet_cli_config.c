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
 * \file  enet_cli_config.c
 *
 * \brief This file contains function definitions for ethernet configuration
 *        related commands
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cli.h"
#include "enet_cli_config.h"

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

static BaseType_t EnetCli_configHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static BaseType_t EnetCli_setPriorityMap(char *writeBuffer,
        size_t writeBufferLen, const char *commandString);

static int32_t EnetConfig_setRemap(uint8_t port, uint8_t remapType,
        uint8_t *map);

static BaseType_t EnetCli_getPriorityMap(char *writeBuffer,
        size_t writeBufferLen, const char *commandString);

static BaseType_t EnetCli_traceLvl(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCli_configCommandHandler(char *writeBuffer,
        size_t writeBufferLen, const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);

    if (parameter == NULL || strncmp(parameter, "help", paramLen) == 0)
        return EnetCli_configHelp(writeBuffer, writeBufferLen, commandString);
    else if (strncmp(parameter, "setmap", paramLen) == 0)
        return EnetCli_setPriorityMap(writeBuffer, writeBufferLen,
                commandString);
    else if (strncmp(parameter, "getmap", paramLen) == 0)
        return EnetCli_getPriorityMap(writeBuffer, writeBufferLen,
                commandString);
    else if (strncmp(parameter, "tracelvl", paramLen) == 0)
        return EnetCli_traceLvl(writeBuffer, writeBufferLen, commandString);
    else
    {
        snprintf(writeBuffer, writeBufferLen,
                "Bad argument\r\nFor more info run \'enetconfig help\'\r\n");
        return pdFALSE;
    }
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static BaseType_t EnetCli_configHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    EnetAppUtils_print(
            "Commands to modify ethernet configurations.\r\nUsage:\r\n");
    EnetAppUtils_print(
            "\tenetconfig setmap <port_num> {-i|-e}\tModifies priority remap data.\r\n");
    EnetAppUtils_print(
            "\t\t   <map0> ... <map7>\t\tUse -i tag to set VLAN priority regeneration map.\r\n");
    EnetAppUtils_print(
            "\t\t\t\t\t\tUse -e tag to set QoS egress priority map.\r\n");
    EnetAppUtils_print(
            "\tenetconfig getmap <port_num>\t\tPrints priority remap data.\r\n");
    EnetAppUtils_print("\tenetconfig tracelvl [<lvl>]\t\tSets the trace level for ...\r\n");
    EnetAppUtils_print("\t\t\t\t\t\tPrints current trace level when lvl is not specified.\r\n");
    EnetAppUtils_print("\tenetconfig help\t\t\t\tPrints this message.\r\n");
    return pdFALSE;
}

static BaseType_t EnetCli_setPriorityMap(char *writeBuffer,
        size_t writeBufferLen, const char *commandString)
{
    int32_t status = ENET_SOK;
    char *parameter;
    BaseType_t paramLen;
    uint8_t remapType, portNum;
    uint8_t map[8];

    for (uint8_t paramCnt = 2; paramCnt < 12; paramCnt++)
    {
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);
        if (parameter == NULL)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Missing argument(s)\r\nFor more info run \'enetconfig help\'\r\n");
            return pdFALSE;
        }

        if (paramCnt == 2)
        {
            portNum = atoi(parameter);
            if (portNum < 0 || portNum > EnetInfo_inst.numMacPorts)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid MAC port\r\n");
                return pdFALSE;
            }
        }
        else if (paramCnt == 3)
        {
            if (strncmp(parameter, "-i", paramLen) == 0)
                remapType = REMAP_INGRESS;
            else if (strncmp(parameter, "-e", paramLen) == 0)
                remapType = REMAP_EGRESS;
            else
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid remap type\r\n");
                return pdFALSE;
            }
        }
        else
        {
            map[paramCnt - 4] = atoi(parameter);
            if (map[paramCnt - 4] < 0 || map[paramCnt - 4] > 7)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid priority value\r\n");
                return pdFALSE;
            }
        }
    }

    status = EnetConfig_setRemap(portNum, remapType, map);
    if (status)
        snprintf(writeBuffer, writeBufferLen, "Remap Failed\r\n");
    else
        snprintf(writeBuffer, writeBufferLen, "Remap Successful\r\n");
    return pdFALSE;
}

static int32_t EnetConfig_setRemap(uint8_t port, uint8_t remapType,
        uint8_t *map)
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    /* For host port */
    if (port == 0)
    {
        EnetPort_PriorityMap inArgs;
        memset(&inArgs, 0, sizeof(inArgs));

        for (uint8_t i = 0; i < 8; i++)
            inArgs.priorityMap[i] = *(map + i);
        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        if (remapType == REMAP_INGRESS)
            ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                    ENET_HOSTPORT_IOCTL_SET_PRI_REGEN_MAP, &prms, status);
        else if (remapType == REMAP_EGRESS)
            ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                    ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, status);
    }

    /* For MAC port */
    else
    {
        if (remapType == REMAP_INGRESS)
        {
            EnetMacPort_SetPriorityRegenMapInArgs inArgs;
            memset(&inArgs, 0, sizeof(inArgs));
            inArgs.macPort = port - 1;
            for (uint8_t i = 0; i < 8; i++)
                inArgs.priorityRegenMap.priorityMap[i] = *(map + i);
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                    ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP, &prms, status);
        }
        else if (remapType == REMAP_EGRESS)
        {
            EnetMacPort_SetEgressPriorityMapInArgs inArgs;
            memset(&inArgs, 0, sizeof(inArgs));
            inArgs.macPort = port - 1;
            for (uint8_t i = 0; i < 8; i++)
                inArgs.priorityMap.priorityMap[i] = *(map + i);
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                    ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, status);
        }
    }

    return status;
}

static BaseType_t EnetCli_getPriorityMap(char *writeBuffer,
        size_t writeBufferLen, const char *commandString)
{
    int32_t status = ENET_SOK;
    char *parameter;
    BaseType_t paramLen;
    uint8_t portNum;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if (parameter == NULL)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Missing argument(s)\r\nFor more info run \'enetconfig help\'\r\n");
        return pdFALSE;
    }
    portNum = atoi(parameter);
    if (portNum < 0 || portNum > EnetInfo_inst.numMacPorts)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid MAC port\r\n");
        return pdFALSE;
    }

    Enet_IoctlPrms prms;
    if (portNum == 0)
    {
        EnetPort_PriorityMap outArgs;
        ENET_IOCTL_SET_OUT_ARGS(&prms, &outArgs);
        ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                ENET_HOSTPORT_IOCTL_GET_PRI_REGEN_MAP, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("VLAN Priority Regen Map --> ");
            for (uint8_t i = 0; i < 8; i++)
                EnetAppUtils_print("%d ", outArgs.priorityMap[i]);
        }
        else
            EnetAppUtils_print("Failed to fetch VLAN priority regen map");
        EnetAppUtils_print("\r\n");
        ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                ENET_HOSTPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("Egress QoS Priority Map --> ");
            for (uint8_t i = 0; i < 8; i++)
                EnetAppUtils_print("%d ", outArgs.priorityMap[i]);
        }
        else
            EnetAppUtils_print("Failed to fetch egress Qos priority map");
        EnetAppUtils_print("\r\n");
    }
    else
    {
        EnetMacPort_GenericInArgs inArgs;
        EnetPort_PriorityMap outArgs;
        inArgs.macPort = portNum - 1;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
        ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("VLAN Priority Regen Map --> ");
            for (uint8_t i = 0; i < 8; i++)
                EnetAppUtils_print("%d ", outArgs.priorityMap[i]);
        }
        else
            EnetAppUtils_print("Failed to fetch VLAN priority regen map");
        EnetAppUtils_print("\r\n");
        ENET_IOCTL(EnetInfo_inst.hEnet, EnetInfo_inst.coreId,
                ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("Egress QoS Priority Map --> ");
            for (uint8_t i = 0; i < 8; i++)
                EnetAppUtils_print("%d ", outArgs.priorityMap[i]);
        }
        else
            EnetAppUtils_print("Failed to fetch egress Qos priority map");
        EnetAppUtils_print("\r\n");
    }
    return pdFALSE;
}

static BaseType_t EnetCli_traceLvl(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    char *parameter;
    BaseType_t paramLen;
    EnetTrace_TraceLevel lvl;

    parameter = (char*)FreeRTOS_CLIGetParameter(commandString, 2, &paramLen);
    if(parameter == NULL)
    {
        lvl = EnetTrace_getLevel();
        snprintf(writeBuffer, writeBufferLen, "Current trace level: %d\r\n", lvl);
    }
    else
    {
        lvl = atoi(parameter);
        if(lvl < 0 || lvl > 5)
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid trace level\r\n");
            return pdFALSE;
        }
        EnetTrace_setLevel(lvl);
        snprintf(writeBuffer, writeBufferLen, "Set trace level to %d\r\n",lvl);
    }
    return pdFALSE;
}
