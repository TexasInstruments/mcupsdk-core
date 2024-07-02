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

static bool EnetCli_configHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_priorityMap(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static int32_t EnetConfig_setPriorityMap(uint8_t port,
        EnetCli_RemapType remapType, uint8_t *map);

static void EnetConfig_getPriorityMap(uint8_t portNum);

static bool EnetCli_traceLvl(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_classifier(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static int32_t EnetConfig_addClassifierEntry(
        CpswAle_PolicerMatchParams *matchPrms, uint8_t rxCh);

static int32_t EnetConfig_remClassifierEntry(
        CpswAle_PolicerMatchParams *matchPrms, int8_t rxCh);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

bool EnetCli_configCommandHandler(char *writeBuffer,
        size_t writeBufferLen, const char *commandString)
{
    char *parameter;
    uint32_t paramLen;
    parameter = (char*) EnetCli_getParameter(commandString, 1, &paramLen);

    if (parameter == NULL || strncmp(parameter, "help", paramLen) == 0)
    {
        return EnetCli_configHelp(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "mqprio", paramLen) == 0)
    {
        return EnetCli_priorityMap(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "tracelvl", paramLen) == 0)
    {
        return EnetCli_traceLvl(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "classifier", paramLen) == 0)
    {
        return EnetCli_classifier(writeBuffer, writeBufferLen, commandString);
    }
    else
    {
        snprintf(writeBuffer, writeBufferLen,
                "Bad argument\r\nFor more info run \'enet_cfg help\'\r\n");
        return false;
    }
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static bool EnetCli_configHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    EnetAppUtils_print(
            "Commands to modify ethernet configurations.\r\nUsage:\r\n");
    EnetAppUtils_print(
            "\tenet_cfg mqprio <port_num> [{-i|-e}\tModifies priority remap data.\r\n");
    EnetAppUtils_print(
            "\t\t   <map0> ... <map7>]\t\tUse -i tag to set VLAN priority regeneration map.\r\n");
    EnetAppUtils_print(
            "\t\t\t\t\t\tUse -e tag to set QoS egress priority map.\r\n");
    EnetAppUtils_print(
            "\t\t\t\t\t\tPrints current priority remap data when only <port_num> is specified.\r\n");
    EnetAppUtils_print(
            "\tenet_cfg tracelvl [<lvl>]\t\tSets the trace level for enet processes.\r\n");
    EnetAppUtils_print(
            "\t\t\t\t\t\tPrints current trace level when <lvl> is not specified.\r\n");
    EnetAppUtils_print(
            "\tenet_cfg classifier [-r]\t\tConfigures classifier with the given rules.\r\n");
    EnetAppUtils_print(
            "\t\t   [-e <ether_type>]\t\tUse -r tag to remove the classifier with the given rules.\r\n");
    EnetAppUtils_print(
            "\t\t   [-p <mac_port>]\t\tSpecify <rx_chn> along with -r tag to remove the classifiers\r\n");
    EnetAppUtils_print(
            "\t\t   [-c <rx_chn>]\t\tassociated with the channel.\r\n");
    EnetAppUtils_print(
            "\t\t   [-sm <src_mac_addr>]\r\n\t\t   [-dm <dest_mac_addr>]\r\n");
    EnetAppUtils_print(
            "\t\t   [-sip <src_ipv4_addr>]\r\n\t\t   [-dip <dest_ipv4_addr>]\r\n\t\t   [-iv <inner_vid>]\r\n");
    EnetAppUtils_print(
            "\t\t   [-ov <outer_vid>]\r\n\t\t   [-pcp <priority>]\r\n");
    EnetAppUtils_print("\tenet_cfg help\t\t\t\tPrints this message.\r\n");
    return false;
}

static bool EnetCli_priorityMap(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = ENET_SOK;
    char *parameter;
    uint32_t paramLen;
    uint8_t portNum;
    EnetCli_RemapType remapType;
    uint8_t map[8];

    for (uint32_t paramCnt = 2; paramCnt < 12; paramCnt++)
    {
        parameter = (char*) EnetCli_getParameter(commandString, paramCnt,
                &paramLen);
        if (parameter == NULL)
        {
            if (paramCnt == 3)
            {
                EnetConfig_getPriorityMap(portNum);
                return false;
            }
            snprintf(writeBuffer, writeBufferLen,
                    "Missing argument(s)\r\nFor more info run \'enet_cfg help\'\r\n");
            return false;
        }

        if (paramCnt == 2)
        {
            portNum = atoi(parameter);
            if (portNum < 0 || portNum > EnetCli_inst.numMacPorts)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid port number\r\n");
                return false;
            }
        }
        else if (paramCnt == 3)
        {
            if (strncmp(parameter, "-i", paramLen) == 0)
            {
                remapType = ENET_CLI_REMAP_INGRESS;
            }
            else if (strncmp(parameter, "-e", paramLen) == 0)
            {
                remapType = ENET_CLI_REMAP_EGRESS;
            }
            else
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid remap type\r\n");
                return false;
            }
        }
        else
        {
            map[paramCnt - 4] = atoi(parameter);
            if (map[paramCnt - 4] < 0 || map[paramCnt - 4] > 7)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid priority value\r\n");
                return false;
            }
        }
    }

    status = EnetConfig_setPriorityMap(portNum, remapType, map);
    if (status)
        snprintf(writeBuffer, writeBufferLen, "Remap failed!!\r\n");
    else
        snprintf(writeBuffer, writeBufferLen, "Remap successful\r\n");
    return false;
}

static int32_t EnetConfig_setPriorityMap(uint8_t port,
        EnetCli_RemapType remapType, uint8_t *map)
{
    Enet_IoctlPrms prms;
    int32_t status = ENET_SOK;

    /* For host port */
    if (port == CPSW_ALE_HOST_PORT_NUM)
    {
        EnetPort_PriorityMap inArgs;
        memset(&inArgs, 0, sizeof(inArgs));

        for (uint32_t mapIdx = 0; mapIdx < 8; mapIdx++)
        {
            inArgs.priorityMap[mapIdx] = *(map + mapIdx);
        }
        ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
        if (remapType == ENET_CLI_REMAP_INGRESS)
        {
            ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                    ENET_HOSTPORT_IOCTL_SET_PRI_REGEN_MAP, &prms, status);
        }
        else if (remapType == ENET_CLI_REMAP_EGRESS)
        {
            ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                    ENET_HOSTPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, status);
        }
    }

    /* For MAC port */
    else
    {
        if (remapType == ENET_CLI_REMAP_INGRESS)
        {
            EnetMacPort_SetPriorityRegenMapInArgs inArgs;
            memset(&inArgs, 0, sizeof(inArgs));
            inArgs.macPort = (Enet_MacPort) (port - 1);
            for (uint32_t mapIdx = 0; mapIdx < 8; mapIdx++)
            {
                inArgs.priorityRegenMap.priorityMap[mapIdx] = *(map + mapIdx);
            }
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                    ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP, &prms, status);
        }
        else if (remapType == ENET_CLI_REMAP_EGRESS)
        {
            EnetMacPort_SetEgressPriorityMapInArgs inArgs;
            memset(&inArgs, 0, sizeof(inArgs));
            inArgs.macPort = (Enet_MacPort) (port - 1);
            for (uint32_t mapIdx = 0; mapIdx < 8; mapIdx++)
            {
                inArgs.priorityMap.priorityMap[mapIdx] = *(map + mapIdx);
            }
            ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
            ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                    ENET_MACPORT_IOCTL_SET_EGRESS_QOS_PRI_MAP, &prms, status);
        }
    }

    return status;
}

static void EnetConfig_getPriorityMap(uint8_t portNum)
{
    Enet_IoctlPrms prms;
    int32_t status;

    /* For host port */
    if (portNum == CPSW_ALE_HOST_PORT_NUM)
    {
        EnetPort_PriorityMap outArgs;
        ENET_IOCTL_SET_OUT_ARGS(&prms, &outArgs);
        ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                ENET_HOSTPORT_IOCTL_GET_PRI_REGEN_MAP, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("VLAN Priority Regen Map --> ");
            for (uint32_t mapIdx = 0; mapIdx < 8; mapIdx++)
            {
                EnetAppUtils_print("%d ", outArgs.priorityMap[mapIdx]);
            }
        }
        else
        {
            EnetAppUtils_print("Failed to fetch VLAN priority regen map");
        }
        EnetAppUtils_print("\r\n");
        ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                ENET_HOSTPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("Egress QoS Priority Map --> ");
            for (uint32_t mapIdx = 0; mapIdx < 8; mapIdx++)
            {
                EnetAppUtils_print("%d ", outArgs.priorityMap[mapIdx]);
            }
        }
        else
        {
            EnetAppUtils_print("Failed to fetch egress Qos priority map");
        }
        EnetAppUtils_print("\r\n");
    }

    /* For MAC ports */
    else
    {
        EnetMacPort_GenericInArgs inArgs;
        EnetPort_PriorityMap outArgs;
        inArgs.macPort = (Enet_MacPort) (portNum - 1);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);
        ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                ENET_MACPORT_IOCTL_GET_PRI_REGEN_MAP, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("VLAN Priority Regen Map --> ");
            for (uint32_t mapIdx = 0; mapIdx < 8; mapIdx++)
            {
                EnetAppUtils_print("%d ", outArgs.priorityMap[mapIdx]);
            }
        }
        else
        {
            EnetAppUtils_print("Failed to fetch VLAN priority regen map");
        }
        EnetAppUtils_print("\r\n");
        ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                ENET_MACPORT_IOCTL_GET_EGRESS_QOS_PRI_MAP, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("Egress QoS Priority Map --> ");
            for (uint32_t mapIdx = 0; mapIdx < 8; mapIdx++)
            {
                EnetAppUtils_print("%d ", outArgs.priorityMap[mapIdx]);
            }
        }
        else
        {
            EnetAppUtils_print("Failed to fetch egress Qos priority map");
        }
        EnetAppUtils_print("\r\n");
    }
}

static bool EnetCli_traceLvl(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    char *parameter;
    uint32_t paramLen;
    EnetTrace_TraceLevel lvl;

    parameter = (char*) EnetCli_getParameter(commandString, 2, &paramLen);
    if (parameter == NULL)
    {
        lvl = EnetTrace_getLevel();
        snprintf(writeBuffer, writeBufferLen, "Current trace level: %d\r\n",
                lvl);
    }
    else
    {
        lvl = (EnetTrace_TraceLevel) atoi(parameter);
        if ((lvl < 0) || (lvl > 5))
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid trace level\r\n");
            return false;
        }
        EnetTrace_setLevel(lvl);
        snprintf(writeBuffer, writeBufferLen, "Set trace level to %d\r\n", lvl);
    }
    return false;
}

static bool EnetCli_classifier(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = 0;
    char *parameter;
    uint32_t paramLen;
    uint32_t paramCnt = 2;
    CpswAle_PolicerMatchParams args;
    memset(&args, 0, sizeof(args));
    int8_t rxCh = -1;
    bool remove = false;

    parameter = (char*) EnetCli_getParameter(commandString, paramCnt,
            &paramLen);
    if (strncmp(parameter, "-r", paramLen) == 0)
    {
        remove = true;
        paramCnt += 1;
    }
    parameter = (char*) EnetCli_getParameter(commandString, paramCnt,
            &paramLen);
    while (parameter != NULL)
    {
        /* Classifier based on ether type */
        if (strncmp(parameter, "-e", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_ETHERTYPE;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            args.etherType = (uint16_t) strtol(parameter, NULL, 16);
        }
        /* Classifier based on source MAC address */
        else if (strncmp(parameter, "-sm", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_MACSRC;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            status = EnetAppUtils_macAddrAtoI(parameter,
                    args.srcMacAddrInfo.addr.addr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid MAC address\r\n");
                return false;
            }
        }
        /* Classifier based on destination MAC address */
        else if (strncmp(parameter, "-dm", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_MACDST;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            status = EnetAppUtils_macAddrAtoI(parameter,
                    args.dstMacAddrInfo.addr.addr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid MAC address\r\n");
                return false;
            }
        }
        /* Classifier based on source IP address */
        else if (strncmp(parameter, "-sip", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_IPSRC;
            args.srcIpInfo.ipAddrType = CPSW_ALE_IPADDR_CLASSIFIER_IPV4;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            status = EnetAppUtils_ipAddrAtoI(parameter,
                    args.srcIpInfo.ipv4Info.ipv4Addr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid IP address\r\n");
                return false;
            }
        }
        /* Classifier based on destination IP address */
        else if (strncmp(parameter, "-dip", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_IPDST;
            args.dstIpInfo.ipAddrType = CPSW_ALE_IPADDR_CLASSIFIER_IPV4;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            status = EnetAppUtils_ipAddrAtoI(parameter,
                    args.dstIpInfo.ipv4Info.ipv4Addr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid IP address\r\n");
                return false;
            }
        }
        /* Classifier based on inner vlan id */
        else if (strncmp(parameter, "-iv", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_IVLAN;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            if (atoi(parameter) > 4096 || atoi(parameter) < 0)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid VLAN ID\r\n");
                return false;
            }
            args.ivlanId = atoi(parameter);
        }
        /* Classifier based on outer vlan id */
        else if (strncmp(parameter, "-ov", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_OVLAN;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            if (atoi(parameter) > 4096 || atoi(parameter) < 0)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid VLAN ID\r\n");
                return false;
            }
            args.ovlanId = atoi(parameter);
        }
        /* Classifier based on priority */
        else if (strncmp(parameter, "-pcp", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_OVLAN;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            if (atoi(parameter) > 7 || atoi(parameter) < 0)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid priority value\r\n");
                return false;
            }
            args.priority = atoi(parameter);
        }
        /* Classifier based on MAC port */
        else if (strncmp(parameter, "-p", paramLen) == 0)
        {
            args.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_PORT;
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            if (atoi(parameter) >= EnetCli_inst.numMacPorts
                    || atoi(parameter) < 0)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid MAC port\r\n");
                return false;
            }
            args.portNum = atoi(parameter) + 1;
        }
        /* The DMA channel to use for forwarding classified packets */
        else if (strncmp(parameter, "-c", paramLen) == 0)
        {
            parameter = (char*) EnetCli_getParameter(commandString,
                    paramCnt + 1, &paramLen);
            rxCh = atoi(parameter);
        }
        else
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid args\r\n");
            return false;
        }
        paramCnt += 2;
        parameter = (char*) EnetCli_getParameter(commandString, paramCnt,
                &paramLen);
    }

    if (args.policerMatchEnMask == 0 && (!remove || (remove && rxCh == -1)))
    {
        snprintf(writeBuffer, writeBufferLen,
                "No classifier params specified\r\n");
        return false;
    }
    if (rxCh == -1 && !remove)
    {
        snprintf(writeBuffer, writeBufferLen, "No Rx channel specified\r\n");
        return false;
    }

    if (remove)
    {
        /* Remove classifier entry from ALE */
        status = EnetConfig_remClassifierEntry(&args, rxCh);
        if (status)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Failed to remove classifier entry from ALE\r\n");
        }
        else
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Removed classifier entry(s) from ALE\r\n");
        }
    }
    else
    {
        /* Add classifier entry to ALE */
        status = EnetConfig_addClassifierEntry(&args, rxCh);
        if (status)
            snprintf(writeBuffer, writeBufferLen,
                    "Failed to add classifier entry to ALE\r\n");
        else
            snprintf(writeBuffer, writeBufferLen,
                    "Added classifier entry to ALE\r\n");
    }
    return false;
}

static int32_t EnetConfig_addClassifierEntry(
        CpswAle_PolicerMatchParams *matchPrms, uint8_t rxCh)
{
    CpswAle_SetPolicerEntryInArgs setPolicerInArgs;
    CpswAle_SetPolicerEntryOutArgs setPolicerOutArgs;
    Enet_IoctlPrms prms;
    int32_t status;

    memset(&setPolicerInArgs, 0, sizeof(setPolicerInArgs));

    /* Add policer entry with the given parameters and route the classified
     * packets to the specified rx dma channel.
     */
    setPolicerInArgs.policerMatch = *matchPrms;
    setPolicerInArgs.threadIdEn = true;
    /* The thread ID ranges from 1 to 8, rxCh passed to this function ranges from 0 to 7 */
    setPolicerInArgs.threadId = rxCh + 1;
    setPolicerInArgs.peakRateInBitsPerSec = 0;
    setPolicerInArgs.commitRateInBitsPerSec = 0;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerInArgs, &setPolicerOutArgs);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            CPSW_ALE_IOCTL_SET_POLICER, &prms, status);

    return status;
}

static int32_t EnetConfig_remClassifierEntry(
        CpswAle_PolicerMatchParams *matchPrms, int8_t rxCh)
{
    /* Remove classifier entry from ALE */
    CpswAle_DelPolicerEntryInArgs inArgs;
    int32_t status;
    Enet_IoctlPrms prms;

    /* Remove policer entry which matches the given parameters */
    inArgs.policerMatch = *matchPrms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            CPSW_ALE_IOCTL_DEL_POLICER, &prms, status);

    /* If rxCh is specified, remove all classifier associated to the channel */
    if (rxCh != -1 && status == ENET_SOK)
    {
        /* The thread ID ranges from 1 to 8, rxCh passed to this function ranges from 0 to 7 */
        uint32_t threadId = rxCh + 1;
        ENET_IOCTL_SET_IN_ARGS(&prms, &threadId);
        ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                CPSW_ALE_IOCTL_DEL_ALL_POLICER_THREADID, &prms, status);
    }

    return status;
}
