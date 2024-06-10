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
 * \file  cli_debug.c
 *
 * \brief This file contains all the functions for debugging purposes.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "cli_debug.h"

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

static void EnetApp_showCpswStats(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_showCpswStats(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    EnetApp_showCpswStats();
    return pdFALSE;
}

BaseType_t EnetCLI_aleDump(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
#ifdef ENET_DEBUG_MODE
    Enet_IoctlPrms prms;
    int32_t status;
    ENET_IOCTL_SET_NO_ARGS(&prms);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_ALE_IOCTL_DUMP_TABLE, &prms, status);
    if (status != ENET_SOK)
        snprintf(writeBuffer, writeBufferLen, "Failed to fetch ALE table data\r\n");
#else
    snprintf(writeBuffer, writeBufferLen,
            "This commands is only available in debug mode\r\n");
#endif
    return pdFALSE;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static void EnetApp_showCpswStats(void)
{
    Enet_IoctlPrms prms;
    CpswStats_PortStats portStats;
    int32_t status;

    /* Show host port statistics */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\r\n Port 0 Statistics\r\n");
        EnetAppUtils_print("-----------------------------------------\r\n");
        EnetAppUtils_printHostPortStats2G((CpswStats_HostPort_2g*) &portStats);
        EnetAppUtils_print("\r\n");
    }
    else
    {
        EnetAppUtils_print("Failed to get host stats: %d\r\n", status);
    }

    /* Show MAC port statistics */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &EnetApp_inst.macPort[0], &portStats);
        ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
                ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port 1 Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G(
                    (CpswStats_MacPort_2g*) &portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &EnetApp_inst.macPort[1], &portStats);
        ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
                ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port 2 Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G(
                    (CpswStats_MacPort_2g*) &portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }
    }
}
