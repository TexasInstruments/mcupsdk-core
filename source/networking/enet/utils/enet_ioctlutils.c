/*
 *  Copyright (c) Texas Instruments Incorporated 2020
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  enet_ioctlutils.c
 *
 * \brief This file contains the implementation of the Enet IOCTL utils functions.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdarg.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/per/cpsw.h>

#include "include/enet_apputils.h"

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

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetAppUtils_allocMac(Enet_Handle hEnet,
                              uint32_t coreKey,
                              uint32_t coreId,
                              uint8_t *macAddress)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(RM_PRESENT)

    Enet_IoctlPrms prms;
    EnetRm_AllocMacAddrOutArgs allocMacOutArgs;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &coreKey, &allocMacOutArgs);
    ENET_IOCTL(hEnet,
               coreId,
               ENET_RM_IOCTL_ALLOC_MAC_ADDR,
               &prms,
               status);

    if (status == ENET_SOK)
    {
        memcpy(macAddress, allocMacOutArgs.macAddr, sizeof(allocMacOutArgs.macAddr));
    }
    else
    {
        EnetAppUtils_print("EnetAppUtils_allocMac() failed : %d\r\n", status);
    }
#else
    uint8_t mac[6] = { 0x70U, 0xFFU, 0x76U, 0x1DU, 0xECU, 0xF2U };

    memcpy(macAddress, mac, sizeof(mac));
#endif
    return status;
}

int32_t EnetAppUtils_freeMac(Enet_Handle hEnet,
                             uint32_t coreKey,
                             uint32_t coreId,
                             uint8_t *macAddress)
{
    int32_t status = ENET_SOK;
#if ENET_CFG_IS_ON(RM_PRESENT)

    Enet_IoctlPrms prms;
    EnetRm_FreeMacAddrInArgs freeMacInArgs;

    freeMacInArgs.coreKey = coreKey;
    EnetUtils_copyMacAddr(freeMacInArgs.macAddr, macAddress);

    ENET_IOCTL_SET_IN_ARGS(&prms, &freeMacInArgs);
    ENET_IOCTL(hEnet,
               coreId,
               ENET_RM_IOCTL_FREE_MAC_ADDR,
               &prms,
               status);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_freeMac() failed : %d\r\n", status);
    }
#endif

    return status;
}

bool EnetAppUtils_isPortLinkUp(Enet_Handle hEnet,
                               uint32_t coreId,
                               Enet_MacPort portNum)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    bool linked;
    int32_t status;

    macPort = portNum;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

    ENET_IOCTL(hEnet, coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_isPortLinkUp() failed to get port %u's link status: %d\r\n",
                           portNum, status);
        linked = false;
    }

    return linked;
}

void EnetAppUtils_addHostPortEntry(Enet_Handle hEnet,
                                   uint32_t coreId,
                                   uint8_t *macAddr)
{
    int32_t status;
    Enet_IoctlPrms prms;
    uint32_t entryIdx;
    CpswAle_SetUcastEntryInArgs setUcastInArgs;

    memset(&setUcastInArgs, 0, sizeof(setUcastInArgs));
    memcpy(&setUcastInArgs.addr.addr[0U], macAddr, sizeof(setUcastInArgs.addr.addr));
    setUcastInArgs.addr.vlanId  = 0U;
    setUcastInArgs.info.portNum = CPSW_ALE_HOST_PORT_NUM;
    setUcastInArgs.info.blocked = false;
    setUcastInArgs.info.secure  = true;
    setUcastInArgs.info.super   = false;
    setUcastInArgs.info.ageable = false;
    setUcastInArgs.info.trunk   = false;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setUcastInArgs, &entryIdx);

    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_UCAST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_addHostPortEntry() failed CPSW_ALE_IOCTL_ADD_UCAST: %d\r\n",
                           status);
    }

    EnetAppUtils_assert(status == ENET_SOK);
}

void EnetAppUtils_delAddrEntry(Enet_Handle hEnet,
                              uint32_t coreId,
                              uint8_t *macAddr)
{
    int32_t status;
    Enet_IoctlPrms prms;
    CpswAle_MacAddrInfo addrInfo;

    memset(&addrInfo, 0, sizeof(addrInfo));
    memcpy(&addrInfo.addr[0U], macAddr, sizeof(addrInfo.addr));
    addrInfo.vlanId = 0U;

    ENET_IOCTL_SET_IN_ARGS(&prms, &addrInfo);

    ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_REMOVE_ADDR, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("EnetAppUtils_delAddrEntry() failed CPSW_ALE_IOCTL_REMOVE_ADDR: %d\r\n",
                           status);
    }

    EnetAppUtils_assert(status == ENET_SOK);
}

