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
 * \file  ale_unicast.c
 *
 * \brief This file contains scripts to add unicast entry to ALE
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "ale_unicast.h"

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

static int32_t EnetApp_addUcastEntry(uint8_t *macAddr);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_addUcast(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status;
    uint8_t macAddr[ENET_MAC_ADDR_LEN];
    char *parameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;
    uint8_t makeDefault = 0;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
            &paramLen);
    while (parameter != NULL)
    {
        if (strncmp(parameter, "-d", paramLen) == 0)
            makeDefault = 1;
        else
        {
            status = EnetAppUtils_macAddrAtoI(parameter, macAddr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen, "Invalid Parameter\r\n");
                return pdFALSE;
            }
            break;
        }
        paramCnt++;
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);
    }

    /* Add unicast entry to ALE */
    status = EnetApp_addUcastEntry(macAddr);
    if (status)
        snprintf(writeBuffer, writeBufferLen,
                "Failed to add unicast entry to ALE\r\n");
    else
    {
        snprintf(writeBuffer, writeBufferLen, "Added unicast entry to ALE\r\n");
        if (makeDefault)
        {
            EnetUtils_copyMacAddr(EnetApp_inst.hostMacAddr, macAddr);
            EnetAppUtils_print("[INF] %s: Default MAC address set to ",
                    __func__);
            EnetAppUtils_printMacAddr(EnetApp_inst.hostMacAddr);
        }
    }
    return pdFALSE;
}

BaseType_t EnetCLI_removeUcast(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status;
    CpswAle_MacAddrInfo inArgs;
    char *parameter;
    BaseType_t paramLen;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, 1, &paramLen);
    status = EnetAppUtils_macAddrAtoI(parameter, inArgs.addr);
    if (status)
    {
        snprintf(writeBuffer, writeBufferLen, "Invalid Parameter\r\n");
        return pdFALSE;
    }

    /* Remove unicast entry from ALE */
    Enet_IoctlPrms prms;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);
    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_ALE_IOCTL_REMOVE_ADDR, &prms, status);
    if (status)
        snprintf(writeBuffer, writeBufferLen,
                "Failed to remove unicast entry to ALE\r\n");
    else
        snprintf(writeBuffer, writeBufferLen,
                "Removed unicast entry from ALE\r\n");
    return pdFALSE;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static int32_t EnetApp_addUcastEntry(uint8_t *macAddr)
{
    CpswAle_SetUcastEntryInArgs setUcastInArgs;
    uint32_t entryIdx;
    Enet_IoctlPrms prms;
    int32_t status;

    setUcastInArgs.addr.vlanId = 0U;
    setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
    setUcastInArgs.info.blocked = false;
    setUcastInArgs.info.secure = false;
    setUcastInArgs.info.super = false;
    setUcastInArgs.info.ageable = false;
    setUcastInArgs.info.trunk = false;
    EnetUtils_copyMacAddr(&setUcastInArgs.addr.addr[0U], macAddr);
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ERR] %s: Failed to add unicast entry: %d\r\n",
                __func__, status);
        return 1;
    }
    EnetAppUtils_print("[INF] %s: Added Unicast entry with MAC address: ",
            __func__);
    EnetAppUtils_printMacAddr(macAddr);
    return 0;
}
