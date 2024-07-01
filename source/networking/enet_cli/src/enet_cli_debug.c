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
 * \file  enet_cli_debug.c
 *
 * \brief This file contains function definitions for debug commands
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* Networking Libraries */
#include <include/per/cpsw.h>

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

static bool EnetCli_debugHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_showCpswStats(char *writeBuffer,
        size_t writeBufferLen, const char *commandString);

static void EnetDebug_showCpswStats(uint8_t portNum, bool reset);

static bool EnetCli_dumpAleTable(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCLI_dumpPolicerTable(char *writeBuffer,
        size_t writeBufferLen, const char *commandString);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

bool EnetCli_debugCommandHandler(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    char *parameter;
    uint32_t paramLen;
    parameter = (char*) EnetCli_getParameter(commandString, 1, &paramLen);

    if (parameter == NULL || strncmp(parameter, "help", paramLen) == 0)
    {
        return EnetCli_debugHelp(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "cpswstats", paramLen) == 0)
    {
        return EnetCli_showCpswStats(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "dumpale", paramLen) == 0)
    {
        return EnetCli_dumpAleTable(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "dumppolicer", paramLen) == 0)
    {
        return EnetCLI_dumpPolicerTable(writeBuffer, writeBufferLen,
                commandString);
    }
    else
    {
        snprintf(writeBuffer, writeBufferLen,
                "Bad argument\r\nFor more info run \'enet_dbg help\'\r\n");
        return false;
    }
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static bool EnetCli_debugHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    EnetAppUtils_print("Commands to print debug data.\r\nUsage:\r\n");
    EnetAppUtils_print(
            "\tenet_dbg cpswstats <port_num> [-r]\tPrints CPSW statistics.\r\n");
    EnetAppUtils_print(
            "\t\t\t\t\t\tUsing -r tag will reset the stats after printing.\r\n");
    EnetAppUtils_print("\tenet_dbg dumpale\t\t\tPrints the ALE table.\r\n");
    EnetAppUtils_print(
            "\tenet_dbg dumppolicer\t\t\tPrints the policer table.\r\n");
    EnetAppUtils_print("\tenet_dbg help\t\t\t\tPrints this message.\r\n");
    return false;
}

static bool EnetCli_showCpswStats(char *writeBuffer,
        size_t writeBufferLen, const char *commandString)
{
    uint8_t portNum;
    bool reset = false;
    char *parameter;
    uint32_t paramLen;

    parameter = (char*) EnetCli_getParameter(commandString, 2, &paramLen);
    if (parameter == NULL)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Missing argument(s)\r\nFor more info run \'enet_dbg help\'\r\n");
        return false;
    }
    portNum = atoi(parameter);
    if (portNum < 0 || portNum > EnetCli_inst.numMacPorts)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid MAC port\r\n");
        return false;
    }
    parameter = (char*) EnetCli_getParameter(commandString, 3, &paramLen);
    if (parameter != NULL && strncmp(parameter, "-r", paramLen) == 0)
    {
        reset = true;
    }

    EnetDebug_showCpswStats(portNum, reset);
    return false;
}

static void EnetDebug_showCpswStats(uint8_t portNum, bool reset)
{
    Enet_IoctlPrms prms;
    CpswStats_PortStats portStats;
    int32_t status;

    /* Show host port statistics */
    if (portNum == CPSW_ALE_HOST_PORT_NUM)
    {
        ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
        ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Host Port Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printHostPortStats2G(
                    (CpswStats_HostPort_2g*) &portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get host stats: %d\r\n", status);
        }

        if (reset)
        {
            ENET_IOCTL_SET_NO_ARGS(&prms);
            ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                    ENET_STATS_IOCTL_RESET_HOSTPORT_STATS, &prms, status);
            EnetAppUtils_print("Host port stats reset\r\n");
        }
    }

    /* Show MAC port statistics */
    else
    {
        Enet_MacPort macPort = (Enet_MacPort) (portNum - 1);
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &portStats);
        ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port %d Statistics\r\n", portNum);
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G(
                    (CpswStats_MacPort_2g*) &portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }

        if (reset)
        {
            ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
            ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
                    ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
            EnetAppUtils_print("MAC port %d stats reset\r\n", portNum - 1);
        }
    }
}

static bool EnetCli_dumpAleTable(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
#ifdef ENET_DEBUG_MODE
    Enet_IoctlPrms prms;
    int32_t status;
    ENET_IOCTL_SET_NO_ARGS(&prms);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            CPSW_ALE_IOCTL_DUMP_TABLE, &prms, status);
    if (status != ENET_SOK)
    {
        snprintf(writeBuffer, writeBufferLen, "Failed to fetch ALE table data\r\n");
    }
#else
    snprintf(writeBuffer, writeBufferLen,
            "This commands is only available in debug mode!!\r\n");
#endif
    return false;
}

static bool EnetCLI_dumpPolicerTable(char *writeBuffer,
        size_t writeBufferLen, const char *commandString)
{
    Enet_IoctlPrms prms;
    int32_t status;
    ENET_IOCTL_SET_NO_ARGS(&prms);
    ENET_IOCTL(EnetCli_inst.hEnet, EnetCli_inst.coreId,
            CPSW_ALE_IOCTL_DUMP_POLICER_ENTRIES, &prms, status);
    if (status != ENET_SOK)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Failed to fetch policer entries\r\n");
    }
    return false;
}
