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
 * \file     cpsw_test_policer_nomatch.c
 *
 * \brief    This file contains the enet_policer nomatch test implementation.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include <networking/enet/core/include/enet.h>
#include <networking/enet/utils/include/enet_appmemutils_cfg.h>
#include <networking/enet/utils/include/enet_appmemutils.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_ethutils.h>

#include "cpsw_test_intervlan.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ENET_TEST_INTERVLAN_PKT_TRAFFIC_SRC                 (CPSW_ALE_HOST_PORT_NUM)
#define ENET_TEST_INTERVLAN_UNMODIFIED_VLAN_EGRESS_PORT_NUM (ENET_MAC_PORT_1)

#if (CPSW_MAC_PORT_NUM == 1)
#define ENET_TEST_INTERVLAN_MODIFIED_VLAN_EGRESS_PORT_NUM   (ENET_MAC_PORT_1)
#else
#define ENET_TEST_INTERVLAN_MODIFIED_VLAN_EGRESS_PORT_NUM   (ENET_MAC_PORT_2)
#endif



#define ENET_TEST_INTERVLAN_UNMODIFIED_VLANID    (0x10)
#define ENET_TEST_INTERVLAN_MODIFIED_VLANID      (0x20)

#define ENET_TEST_INTERVLAN_HOSTPORT_PVID     (300)
#define ENET_TEST_INTERVLAN_MACPORT_PVID_BASE (400)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
static int32_t EnetTestInterVlan_setInterVlanMultiEgress(EnetCpswInterVlan_Obj *stateObj,
                                                         CpswMacPort_InterVlanRouteId expectedAllocRouteId,
                                                         uint32_t *pNumRoutesUsed);

static int32_t EnetTestInterVlan_setShortIPG(EnetCpswInterVlan_Obj *stateObj);

static int32_t EnetTestCommon_setAleMulticastEntry(EnetCpswInterVlan_Obj *taskObj,
                                            uint8_t macAddr[ENET_MAC_ADDR_LEN],
                                            uint32_t vlanId,
                                            uint32_t numIgnBits,
                                            uint32_t portMask);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
uint8_t testDstMcastMacAddr[] = {0x01, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
const uint16_t hostTxVlanId = ENET_TEST_INTERVLAN_UNMODIFIED_VLANID;
static Enet_MacPort testMultiEgressPortList[] =
{
     [0] = ENET_TEST_INTERVLAN_UNMODIFIED_VLAN_EGRESS_PORT_NUM,
#if (CPSW_MAC_PORT_NUM > 1)
     [1] = ENET_TEST_INTERVLAN_MODIFIED_VLAN_EGRESS_PORT_NUM
#endif
};

static uint32_t testMultiEgressVlanId[] =
{
     [0] = ENET_TEST_INTERVLAN_MODIFIED_VLANID,
#if (CPSW_MAC_PORT_NUM > 1)
     [1] = ENET_TEST_INTERVLAN_UNMODIFIED_VLANID
#endif
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EnetTestInterVlan_Run(EnetCpswInterVlan_Obj *stateObj)
{
    int32_t status              = ENET_SOK;
    uint32_t numRoutesAllocated = 0;

    status = EnetTestInterVlan_setShortIPG(stateObj);

    if (ENET_SOK == status)
    {
        /* EnetTestInterVlan_setInterVlanUniEgress will use two routes */
        status = EnetTestInterVlan_setInterVlanMultiEgress(stateObj, (CpswMacPort_InterVlanRouteId)(CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST + numRoutesAllocated), &numRoutesAllocated);
    }

    if (ENET_SOK == status)
    {
        Enet_IoctlPrms prms;

#if (_DEBUG_ == 1)
        ENET_IOCTL_SET_NO_ARGS(&prms);
        ENET_IOCTL(stateObj->hEnet, stateObj->coreId, CPSW_ALE_IOCTL_DUMP_TABLE,
                            &prms, status);
        EnetAppUtils_assert(status == ENET_SOK);
#endif

        ENET_IOCTL_SET_NO_ARGS(&prms);
        ENET_IOCTL(stateObj->hEnet, stateObj->coreId, CPSW_ALE_IOCTL_DUMP_POLICER_ENTRIES,
                            &prms, status);
        EnetAppUtils_assert(status == ENET_SOK);

#if FIX_LATER
        ENET_IOCTL_SET_NO_ARGS(&prms);
        ENET_IOCTL(stateObj->hEnet, stateObj->coreId, ENET_IOCTL_PRINT_REGISTERS,
                            &prms,
                            status);
        EnetAppUtils_assert(status == ENET_SOK);
#endif

    }

    return status;
}

void EnetTestInterVlan_setOpenPrms(EnetCpswInterVlan_Obj *stateObj,
                                   Cpsw_Cfg *pCpswCfg)
{
    Enet_MacPort i;

    /* pCpswCfg->aleCfg.policerGlobalCfg.policingEn SHOULD BE TRUE for interVLan.
     */
    pCpswCfg->aleCfg.policerGlobalCfg.policingEn = TRUE;

    pCpswCfg->aleCfg.policerGlobalCfg.redDropEn      = FALSE;
    pCpswCfg->aleCfg.policerGlobalCfg.yellowDropEn   = FALSE;
    pCpswCfg->aleCfg.policerGlobalCfg.policerNoMatchMode = CPSW_ALE_POLICER_NOMATCH_MODE_GREEN;
    pCpswCfg->aleCfg.vlanCfg.aleVlanAwareMode            = TRUE;
    pCpswCfg->aleCfg.vlanCfg.cpswVlanAwareMode           = TRUE;
    pCpswCfg->aleCfg.nwSecCfg.vid0ModeEn                = FALSE;

    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].learningCfg.noLearn              = FALSE;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].vlanCfg.dropUntagged             = FALSE;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.unregMcastFloodMask      = 0x0;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.regMcastFloodMask        = CPSW_ALE_ALL_PORTS_MASK;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.forceUntaggedEgressMask  = 0;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.noLearnMask              = 0x0;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.vidIngressCheck          = 0x0;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.limitIPNxtHdr            = false;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.disallowIPFrag  = false;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.vlanIdInfo.vlanId        = ENET_TEST_INTERVLAN_HOSTPORT_PVID;
    pCpswCfg->aleCfg.portCfg[CPSW_ALE_HOST_PORT_NUM].pvidCfg.vlanMemberList           = CPSW_ALE_ALL_PORTS_MASK;

    for (i = ENET_MAC_PORT_FIRST; i < CPSW_ALE_NUM_MAC_PORTS; i++)
    {
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].learningCfg.noLearn              = FALSE;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].vlanCfg.dropUntagged             = FALSE;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.unregMcastFloodMask      = 0x0;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.regMcastFloodMask        = CPSW_ALE_ALL_PORTS_MASK;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.forceUntaggedEgressMask  = 0;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.noLearnMask              = 0x0;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.vidIngressCheck          = 0x0;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.limitIPNxtHdr            = false;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.disallowIPFrag  = false;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.vlanIdInfo.vlanId        = ENET_TEST_INTERVLAN_MACPORT_PVID_BASE + ENET_MACPORT_NORM(i);
        pCpswCfg->aleCfg.portCfg[CPSW_ALE_MACPORT_TO_ALEPORT(i)].pvidCfg.vlanMemberList           = CPSW_ALE_ALL_PORTS_MASK;
    }

    pCpswCfg->hostPortCfg.vlanCfg.portPri = 7;
    pCpswCfg->hostPortCfg.vlanCfg.portCfi = 0;
    pCpswCfg->hostPortCfg.vlanCfg.portVID = ENET_TEST_INTERVLAN_HOSTPORT_PVID;
    pCpswCfg->vlanCfg.vlanAware           = TRUE;
}

void EnetTestInterVlan_updatePortLinkCfg(EnetPer_PortLinkCfg *pLinkArgs,
                                         Enet_MacPort portNum)
{
    CpswMacPort_Cfg *macCfg      = (CpswMacPort_Cfg*)pLinkArgs->macCfg;

    macCfg->loopbackEn = FALSE;
    macCfg->vlanCfg.portPri = ENET_MACPORT_NORM(portNum);
    macCfg->vlanCfg.portCfi = 0;
    macCfg->vlanCfg.portVID = ENET_TEST_INTERVLAN_MACPORT_PVID_BASE + ENET_MACPORT_NORM(portNum);
}

static uint32_t EnetTestInterVlan_getUnmodifiedVlanMembershipMask(void)
{
    uint32_t memberShipMask;

    memberShipMask =
        (1 << CPSW_ALE_MACPORT_TO_ALEPORT(ENET_TEST_INTERVLAN_UNMODIFIED_VLAN_EGRESS_PORT_NUM));
    memberShipMask |= CPSW_ALE_HOST_PORT_MASK;
    return memberShipMask;
}

static uint32_t EnetTestInterVlan_getModifiedVlanMembershipMask(void)
{
    uint32_t memberShipMask;

    memberShipMask =
        (1 << CPSW_ALE_MACPORT_TO_ALEPORT(ENET_TEST_INTERVLAN_MODIFIED_VLAN_EGRESS_PORT_NUM));
    memberShipMask |= CPSW_ALE_HOST_PORT_MASK;
    return memberShipMask;
}

static uint32_t EnetTestInterVlan_getMultiEgressDestPortMask(void)
{
    uint32_t i;
    uint32_t dstPortMask;

    dstPortMask = 0;
    for (i = 0; i < ENET_ARRAYSIZE(testMultiEgressPortList); i++)
    {
        dstPortMask |= (1 << CPSW_ALE_MACPORT_TO_ALEPORT(testMultiEgressPortList[i]));
    }

    return dstPortMask;
}

static int32_t EnetTestInterVlan_addMultiEgressAleTableEntries(EnetCpswInterVlan_Obj *stateObj)
{
    int32_t status = ENET_SOK;
    Enet_IoctlPrms prms;
    CpswAle_VlanEntryInfo inArgs;


    if (status == ENET_SOK)
    {
        uint32_t outArgs;

        inArgs.vlanIdInfo.vlanId        = ENET_TEST_INTERVLAN_UNMODIFIED_VLANID;
        inArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        inArgs.vlanMemberList           = EnetTestInterVlan_getUnmodifiedVlanMembershipMask();
        inArgs.unregMcastFloodMask      = EnetTestInterVlan_getUnmodifiedVlanMembershipMask();
        inArgs.regMcastFloodMask        = EnetTestInterVlan_getUnmodifiedVlanMembershipMask();
        inArgs.forceUntaggedEgressMask  = 0U;
        inArgs.noLearnMask              = 0U;
        inArgs.vidIngressCheck          = false;
        inArgs.limitIPNxtHdr            = false;
        inArgs.disallowIPFrag  = false;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);

        ENET_IOCTL(stateObj->hEnet, stateObj->coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
        if (status != ENET_SOK)
        {
           EnetAppUtils_print("%s() failed ADD_VLAN ioctl failed: %d\n",
                               __func__, status);
        }
    }

    if (status == ENET_SOK)
    {
        uint32_t outArgs;

        inArgs.vlanIdInfo.vlanId        = ENET_TEST_INTERVLAN_MODIFIED_VLANID;
        inArgs.vlanIdInfo.tagType       = ENET_VLAN_TAG_TYPE_INNER;
        inArgs.vlanMemberList           = EnetTestInterVlan_getModifiedVlanMembershipMask();
        inArgs.unregMcastFloodMask      = EnetTestInterVlan_getModifiedVlanMembershipMask();
        inArgs.regMcastFloodMask        = EnetTestInterVlan_getModifiedVlanMembershipMask();
        inArgs.forceUntaggedEgressMask  = 0U;
        inArgs.noLearnMask              = 0U;
        inArgs.vidIngressCheck          = false;
        inArgs.limitIPNxtHdr            = false;
        inArgs.disallowIPFrag  = false;

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);

        ENET_IOCTL(stateObj->hEnet, stateObj->coreId, CPSW_ALE_IOCTL_ADD_VLAN, &prms, status);
        if (status != ENET_SOK)
        {
           EnetAppUtils_print("%s() failed ADD_VLAN ioctl failed: %d\n",
                               __func__, status);
        }
    }

    if (ENET_SOK == status)
    {
        status = EnetTestCommon_setAleMulticastEntry(stateObj,
                                                     testDstMcastMacAddr,
                                                     ENET_TEST_INTERVLAN_UNMODIFIED_VLANID,
                                                     0,
                                                     EnetTestInterVlan_getUnmodifiedVlanMembershipMask());
    }

    if (ENET_SOK == status)
    {
        status = EnetTestCommon_setAleMulticastEntry(stateObj,
                                                     testDstMcastMacAddr,
                                                     ENET_TEST_INTERVLAN_MODIFIED_VLANID,
                                                     0,
                                                     EnetTestInterVlan_getModifiedVlanMembershipMask());
    }

    return status;
}

static int32_t EnetTestInterVlan_setInterVlanMultiEgress(EnetCpswInterVlan_Obj *stateObj,
                                                         CpswMacPort_InterVlanRouteId expectedAllocRouteId,
                                                         uint32_t *pNumRoutesUsed)
{
    int32_t status;
    Enet_IoctlPrms prms;
    Cpsw_SetInterVlanRouteMultiEgressInArgs inArgs;
    Cpsw_SetInterVlanRouteMultiEgressOutArgs outArgs;
    uint32_t i;

    *pNumRoutesUsed = 0;
    status          = EnetTestInterVlan_addMultiEgressAleTableEntries(stateObj);

    if (ENET_SOK == status)
    {
        /* Reset pkt match flag to 0*/
        inArgs.inPktMatchCfg.packetMatchEnMask = 0;

        memcpy(inArgs.inPktMatchCfg.dstMacAddrInfo.addr.addr, testDstMcastMacAddr, sizeof(inArgs.inPktMatchCfg.dstMacAddrInfo.addr.addr));
        inArgs.inPktMatchCfg.dstMacAddrInfo.addr.vlanId   = ENET_TEST_INTERVLAN_UNMODIFIED_VLANID;
        inArgs.inPktMatchCfg.dstMacAddrInfo.portNum = CPSW_ALE_MACPORT_TO_ALEPORT(ENET_TEST_INTERVLAN_UNMODIFIED_VLAN_EGRESS_PORT_NUM);
        inArgs.inPktMatchCfg.packetMatchEnMask   |= CPSW_INTERVLAN_INGRESSPKT_MATCH_MACDST;

        inArgs.inPktMatchCfg.vlanId         = ENET_TEST_INTERVLAN_UNMODIFIED_VLANID;
        inArgs.inPktMatchCfg.ttlCheckEn = FALSE;

        inArgs.numEgressPorts = ENET_ARRAYSIZE(testMultiEgressPortList);

        for (i = 0; i < inArgs.numEgressPorts; i++)
        {
            inArgs.egressCfg[i].egressPort                       = testMultiEgressPortList[i];
            inArgs.egressCfg[i].outPktModCfg.decrementTTL        = FALSE;
            inArgs.egressCfg[i].outPktModCfg.forceUntaggedEgress = FALSE;
            inArgs.egressCfg[i].outPktModCfg.replaceDASA         = FALSE;
            inArgs.egressCfg[i].outPktModCfg.vlanId = testMultiEgressVlanId[i];
        }

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &outArgs);

        ENET_IOCTL(stateObj->hEnet, stateObj->coreId, CPSW_PER_IOCTL_SET_INTERVLAN_ROUTE_MULTI_EGRESS,
                            &prms, status);
        if (status != ENET_SOK)
        {
           EnetAppUtils_print("EnetTestInterVlan_setInterVlanUniEgress() failed CPSW_PER_IOCTL_CLEAR_INTERVLAN_ROUTE_UNI_EGRESS: %d\n",
                               status);
        }
    }

    if (status == ENET_SOK)
    {
        *pNumRoutesUsed += 1;
       EnetAppUtils_assert(outArgs.egressPortRouteId == expectedAllocRouteId);
       EnetAppUtils_assert(outArgs.ingressPacketClassifierInfo.portIsTrunk == FALSE);
       EnetAppUtils_assert(outArgs.ingressPacketClassifierInfo.egressOpEn == TRUE);
       EnetAppUtils_assert(outArgs.ingressPacketClassifierInfo.egressOpcode == (1 + (outArgs.egressPortRouteId - CPSW_MACPORT_INTERVLAN_ROUTEID_FIRST)));
       EnetAppUtils_assert(outArgs.ingressPacketClassifierInfo.ttlCheckEn == FALSE);
       EnetAppUtils_assert(outArgs.ingressPacketClassifierInfo.dstPortMask == EnetTestInterVlan_getMultiEgressDestPortMask());
       EnetAppUtils_assert(outArgs.ingressPacketClassifierInfo.policerMatchEnMask == (CPSW_ALE_POLICER_MATCH_MACDST |
                                                                                      CPSW_ALE_POLICER_MATCH_IVLAN));
    }

    return status;
}


#define ENET_TEST_INTERVLAN_DEFAULT_SHORTIPG_THRESHOLD                    (11)

static int32_t EnetTestInterVlan_setShortIPG(EnetCpswInterVlan_Obj *stateObj)
{
    Enet_IoctlPrms prms;
    Cpsw_SetTxShortIpgCfgInArgs setShortIPGInArgs;
    int32_t status;

    ENET_IOCTL_SET_IN_ARGS(&prms, &setShortIPGInArgs);
    setShortIPGInArgs.configureGapThresh                           = TRUE;
    setShortIPGInArgs.ipgTriggerThreshBlkCnt                     = 0;
    setShortIPGInArgs.numMacPorts                                     = CPSW_MAC_PORT_NUM;
    setShortIPGInArgs.portShortIpgCfg[0].macPort                           = ENET_TEST_INTERVLAN_UNMODIFIED_VLAN_EGRESS_PORT_NUM;
    setShortIPGInArgs.portShortIpgCfg[0].shortIpgCfg.txShortGapEn      = true;
    setShortIPGInArgs.portShortIpgCfg[0].shortIpgCfg.txShortGapLimitEn = false;

#if (CPSW_MAC_PORT_NUM > 1)
    setShortIPGInArgs.portShortIpgCfg[1].macPort                           = ENET_TEST_INTERVLAN_MODIFIED_VLAN_EGRESS_PORT_NUM;
    setShortIPGInArgs.portShortIpgCfg[1].shortIpgCfg.txShortGapEn      = true;
    setShortIPGInArgs.portShortIpgCfg[1].shortIpgCfg.txShortGapLimitEn = false;
#endif

    ENET_IOCTL(stateObj->hEnet, stateObj->coreId, CPSW_PER_IOCTL_SET_SHORT_IPG_CFG,
                        &prms, status);
    if (ENET_SOK == status)
    {
        Cpsw_TxShortIpgCfg getShortIPGOutArgs;

        ENET_IOCTL_SET_OUT_ARGS(&prms, &getShortIPGOutArgs);

        ENET_IOCTL(stateObj->hEnet, stateObj->coreId, CPSW_PER_IOCTL_GET_SHORT_IPG_CFG,
                            &prms, status);
        if (ENET_SOK == status)
        {
            CpswMacPort_PortTxShortIpgCfg *ipgCfg;
            uint32_t i;

            EnetAppUtils_assert(getShortIPGOutArgs.ipgTriggerThreshBlkCnt == 0U);

            for (i = 0U; i < getShortIPGOutArgs.numMacPorts; i++)
            {
                ipgCfg = &getShortIPGOutArgs.portShortIpgCfg[i];
                if ((ipgCfg->macPort == ENET_TEST_INTERVLAN_UNMODIFIED_VLAN_EGRESS_PORT_NUM) ||
                    (ipgCfg->macPort == ENET_TEST_INTERVLAN_MODIFIED_VLAN_EGRESS_PORT_NUM))
                {
                    EnetAppUtils_assert(ipgCfg->shortIpgCfg.txShortGapEn == true);
                    EnetAppUtils_assert(ipgCfg->shortIpgCfg.txShortGapLimitEn == false);
                }
            }
        }
    }

    return status;
}


int32_t EnetTestCommon_setAleMulticastEntry(EnetCpswInterVlan_Obj *stateObj,
                                            uint8_t macAddr[ENET_MAC_ADDR_LEN],
                                            uint32_t vlanId,
                                            uint32_t numIgnBits,
                                            uint32_t portMask)
{
    int32_t status;
    Enet_IoctlPrms prms;
    uint32_t setMcastOutArgs;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;

    memcpy(&setMcastInArgs.addr.addr[0], macAddr,
           sizeof(setMcastInArgs.addr.addr));
    setMcastInArgs.addr.vlanId = vlanId;

    setMcastInArgs.info.super  = false;
    setMcastInArgs.info.fwdState   = CPSW_ALE_FWDSTLVL_FWD;
    setMcastInArgs.info.portMask   = portMask;
    setMcastInArgs.info.numIgnBits = numIgnBits;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);

    ENET_IOCTL(stateObj->hEnet, stateObj->coreId, CPSW_ALE_IOCTL_ADD_MCAST,
                        &prms, status);
    if (status != ENET_SOK)
    {
       EnetAppUtils_print("%s: failed CPSW_ALE_IOCTL_ADD_MCAST: %d\n",  __func__,
                           status);
    }

    return status;
}
