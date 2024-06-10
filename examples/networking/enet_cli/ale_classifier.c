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
 * \file  ale_classifier.c
 *
 * \brief This file has teh function to add classifier to ALE
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "cli_common.h"
#include "ale_classifier.h"

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

static int32_t EnetApp_addClassifierEntry(CpswAle_PolicerMatchParams prm,
        int8_t rxCh);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

BaseType_t EnetCLI_addClassifier(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    int32_t status = 0;
    char *parameter;
    BaseType_t paramLen;
    uint32_t paramCnt = 1;
    CpswAle_PolicerMatchParams prms;
    memset(&prms, 0, sizeof(prms));
    uint8_t rxCh = 1;

    parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
            &paramLen);
    while (parameter != NULL)
    {
        /* Classifier based on ether type */
        if (strncmp(parameter, "-e", paramLen) == 0)
        {
            prms.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_ETHERTYPE;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            prms.etherType = (uint16_t) strtol(parameter, NULL, 16);
        }

        /* Classifier based on source MAC address */
        else if (strncmp(parameter, "-sm", paramLen) == 0)
        {
            prms.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_MACSRC;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            status = EnetAppUtils_macAddrAtoI(parameter,
                    prms.srcMacAddrInfo.addr.addr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid source MAC address\r\n");
                return pdFALSE;
            }
        }

        /* Classifier based on destination MAC address */
        else if (strncmp(parameter, "-dm", paramLen) == 0)
        {
            prms.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_MACDST;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            status = EnetAppUtils_macAddrAtoI(parameter,
                    prms.dstMacAddrInfo.addr.addr);
            if (status)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid destination MAC address\r\n");
                return pdFALSE;
            }
        }

        /* Classifier based on port number */
        else if (strncmp(parameter, "-p", paramLen) == 0)
        {
            prms.policerMatchEnMask |= CPSW_ALE_POLICER_MATCH_PORT;
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            if (atoi(parameter) > EnetApp_inst.numMacPorts
                    || atoi(parameter) <= 0)
            {
                snprintf(writeBuffer, writeBufferLen,
                        "Invalid port number\r\n");
                return pdFALSE;
            }
            prms.portNum = atoi(parameter);
        }

        /* The DMA channel to use for forwarding classified packets */
        else if (strncmp(parameter, "-c", paramLen) == 0)
        {
            parameter = (char*) FreeRTOS_CLIGetParameter(commandString,
                    paramCnt + 1, &paramLen);
            rxCh = atoi(parameter);
        }
        else
        {
            snprintf(writeBuffer, writeBufferLen, "Invalid args\r\n");
            return pdFALSE;
        }
        paramCnt += 2;
        parameter = (char*) FreeRTOS_CLIGetParameter(commandString, paramCnt,
                &paramLen);
    }
    if (prms.policerMatchEnMask == 0)
    {
        snprintf(writeBuffer, writeBufferLen,
                "No classifier rules specified\r\n");
        return pdFALSE;
    }
    else if (rxCh >= EnetApp_inst.numRxDmaCh)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Rx DMA channel %d is not open\r\n", rxCh);
        return pdFALSE;
    }

    /* Add classifier (policer) entrt to ALE */
    status = EnetApp_addClassifierEntry(prms, rxCh);
    if (status)
        snprintf(writeBuffer, writeBufferLen,
                "Failed to add classifier entry to ALE\r\n");
    else
        snprintf(writeBuffer, writeBufferLen,
                "Added classifier entry to ALE\r\n");
    return pdFALSE;
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static int32_t EnetApp_addClassifierEntry(CpswAle_PolicerMatchParams prm,
        int8_t rxCh)
{
    CpswAle_SetPolicerEntryInArgs setPolicerInArgs;
    CpswAle_SetPolicerEntryOutArgs setPolicerOutArgs;
    Enet_IoctlPrms prms;
    int32_t status;

    memset(&setPolicerInArgs, 0, sizeof(setPolicerInArgs));
    setPolicerInArgs.policerMatch = prm;
    setPolicerInArgs.threadIdEn = true;
    setPolicerInArgs.threadId = rxCh + 1;
    setPolicerInArgs.peakRateInBitsPerSec = 0;
    setPolicerInArgs.commitRateInBitsPerSec = 0;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerInArgs, &setPolicerOutArgs);

    ENET_IOCTL(EnetApp_inst.hEnet, EnetApp_inst.coreId,
            CPSW_ALE_IOCTL_SET_POLICER, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("[ERR] %s: Failed to add classifier entry: %d\r\n",
                __func__, status);
        return 1;
    }
    return 0;
}
