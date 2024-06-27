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
 * \file  ale_vlan.c
 *
 * \brief This file contains the functions used for configuring VLAN.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "ale_vlan.h"

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

static int32_t EnetApp_addVlanEntry(CpswAle_VlanEntryInfo args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_addVlan(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int status = ENET_SOK;
    char *parameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 2;
    CpswAle_VlanEntryInfo inArgs;
    memset(&inArgs, 0, sizeof(inArgs));

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    if (atoi(parameter) > 0 && atoi(parameter) < 4096)
    {
        inArgs.vlanIdInfo.tagType = ENET_VLAN_TAG_TYPE_INNER;
        inArgs.vlanIdInfo.vlanId = atoi(parameter);
    }
    else
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Vlan ID\r\n");
    }

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
            &paramLen);
    while (parameter != NULL)
    {
        uint8_t portNum = atoi(parameter);
        if (portNum > EnetApp_inst.numMacPorts)
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid port number %d\r\n",
                    portNum);
            return pdFALSE;
        }
        inArgs.vlanMemberList |= (1 << portNum);
        inArgs.regMcastFloodMask |= (1 << portNum);
        inArgs.unregMcastFloodMask |= (1 << portNum);

        paramCnt++;
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);
    }

    if (paramCnt == 2)
    {
        snprintf(writeBuffer, writeBufferLen, "No ports specified\r\n");
        return pdFALSE;
    }

    status = EnetApp_addVlanEntry(inArgs);
    if (status)
        snprintf(writeBuffer, writeBufferLen, "Failed to configure VLAN\r\n");
    else
        snprintf(writeBuffer, writeBufferLen, "VLAN configured\r\n");
    return pdFALSE;
}

BaseType_t EnetCLI_removeVlan(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int status = ENET_SOK;
    char *parameter;
    BaseType_t paramLen;
    CpswAle_VlanIdInfo inArgs;
    memset(&inArgs, 0, sizeof(inArgs));

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    if (atoi(parameter) > 0 && atoi(parameter) < 4096)
    {
        inArgs.tagType = ENET_VLAN_TAG_TYPE_INNER;
        inArgs.vlanId = atoi(parameter);
    }
    else
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Vlan ID\r\n");
    }

    /* Remove VLAN config from ALE */
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_ALE_IOCTL_REMOVE_VLAN, &prms, status);
    if (status)
        snprintf(writeBuffer, writeBufferLen, "Failed to remove VLAN\r\n");
    else
        snprintf(writeBuffer, writeBufferLen, "VLAN configuration removed\r\n");
    return pdFALSE;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static int32_t EnetApp_addVlanEntry(CpswAle_VlanEntryInfo args)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    uint32_t outArgs;

    args.forceUntaggedEgressMask = 0U;
    args.noLearnMask = 0U;
    args.vidIngressCheck = false;
    args.limitIPNxtHdr = false;
    args.disallowIPFrag = false;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &args, &outArgs);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId, CPSW_ALE_IOCTL_ADD_VLAN,
            &prms, status);
    if (status)
    {
        EnetAppUtils_print("[ERR] %s: Failed to configure VLAN %d", __func__,
                status);
        return 1;
    }
    return 0;
}

