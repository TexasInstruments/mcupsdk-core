/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  enet_cpsw_est_cfg.c
 *
 * \brief This file contains the implementation of the APIs for peripheral
 *        configuration for the CPSW EST example app.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_est_common.h"
#include "enet_cpsw_est_cfg.h"
#include "enet_cpsw_est_dataflow.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* First admin will have AdminBaseTime = CurrentTime + ENET_APP_EST_ADMIN_LIST_DELAY */
#define ENET_APP_EST_ADMIN_LIST_DELAY             (1000000000ULL)

/* Maximum number of times to check the oper list update status */
#define ENET_APP_OPER_LIST_UPDATE_CHECK_RETRY_MAX (10U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern uint32_t Board_getEthBoardId(void);

static int32_t EnetApp_waitForLinkUp(Enet_MacPort macPort);

static uint64_t EnetApp_getTestAdminBaseTime(Enet_MacPort macPort,
                                             EnetTas_TasState state);

static int32_t EnetApp_setupEst(Enet_MacPort macPort,
                                EnetTas_ControlList *controlList);

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_init(void)
{
    gEnetApp.coreId = EnetSoc_getCoreId();

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetApp.txFreePktInfoQ);
}

void EnetApp_deinit(void)
{




}

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");

        if (!info->isLinked)
        {
            gEnetApp.usingDfltSched = false;
        }
    }
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
    CpswAle_Cfg *aleCfg = &cpswCfg->aleCfg;
    CpswHostPort_Cfg *hostPortCfg = &cpswCfg->hostPortCfg;
    CpswCpts_Cfg *cptsCfg = &cpswCfg->cptsCfg;

    /* Peripheral-level config */
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb     = NULL;
    cpswCfg->mdioLinkStateChangeCbArg  = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb     = EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg  = &gEnetApp;
#endif

    cpswCfg->portLinkStatusChangeCb    = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = &gEnetApp;
    cpswCfg->vlanCfg.vlanAware         = true;

    /* ALE config */
    aleCfg->modeFlags                          = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn               = true;
    aleCfg->agingCfg.agingPeriodInMs           = 1000U;
    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = TRUE;
    aleCfg->vlanCfg.cpswVlanAwareMode          = cpswCfg->vlanCfg.vlanAware;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->policerGlobalCfg.policingEn        = false;

    /* Host port config */
    hostPortCfg->removeCrc         = true;
    hostPortCfg->padShortPacket    = true;
    hostPortCfg->passCrcErrors     = false;
    /* Hardware switch priority is taken from packet's PCP or DSCP */
    hostPortCfg->rxVlanRemapEn     = true;
    hostPortCfg->rxDscpIPv4RemapEn = true;
    hostPortCfg->rxDscpIPv6RemapEn = true;

    /* CPTS config (CPTS_RFT_CLK = 200MHz) */
    cptsCfg->cptsRftClkFreq = CPSW_CPTS_RFTCLK_FREQ_200MHZ;
}

int32_t EnetApp_open(void)
{
    Enet_MacPort macPort;
    EnetTas_ControlList *controlList;
    uint32_t i;
    int32_t status = ENET_SOK;
    EnetApp_HandleInfo handleInfo;

    /* Do peripheral dependent initalization */
    EnetAppUtils_enableClocks(gEnetApp.enetType, gEnetApp.instId);

    /* Create RX task */
    EnetApp_createRxTask();

    EnetApp_driverInit();

    status = EnetApp_driverOpen(gEnetApp.enetType, gEnetApp.instId);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
    }

    EnetApp_acquireHandleInfo(gEnetApp.enetType, gEnetApp.instId, &handleInfo);
    gEnetApp.hEnet = handleInfo.hEnet;
    gEnetApp.hMainUdmaDrv = handleInfo.hUdmaDrv;

    /* Attach the core with RM */
    if (status == ENET_SOK)
    {
        EnetPer_AttachCoreOutArgs attachCoreOutArgs;

        EnetApp_coreAttach(gEnetApp.enetType, gEnetApp.instId, gEnetApp.coreId, &attachCoreOutArgs);
        gEnetApp.coreKey = attachCoreOutArgs.coreKey;
    }

    /* Open DMA for peripheral/port */
    if (status == ENET_SOK)
    {
        status = EnetApp_openDma();
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open DMA: %d\r\n", status);
        }
    }

    /* Wait for PHY link up */
    if (status == ENET_SOK)
    {
        for (i = 0U; i < gEnetApp.macPortNum; i++)
        {
            macPort = gEnetApp.macPort[i];
            controlList = &gEnetApp.tasControlList[i];

            status = EnetApp_waitForLinkUp(macPort);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Port %u: Failed to wait for link up: %d\r\n", ENET_MACPORT_ID(macPort), status);
                break;
            }

            /* Configure EST */
            status = EnetApp_setupEst(macPort, controlList);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Port %u: Failed to setup EST: %d\r\n", ENET_MACPORT_ID(macPort), status);
                break;
            }
        }

        gEnetApp.usingDfltSched = (status == ENET_SOK);
    }

    /* Print our address */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("MAC addr: ");
        EnetAppUtils_printMacAddr(&gEnetApp.macAddr[0U]);
    }

    return status;
}

void EnetApp_close(void)
{
    /* Close DMA */
    EnetApp_closeDma();

    /* Delete RX tasks created for all peripherals */
    EnetApp_destroyRxTask();

    /*Detach Core*/
    EnetApp_coreDetach(gEnetApp.enetType, gEnetApp.instId, gEnetApp.coreId, gEnetApp.coreKey);

    /*Release Handle Info*/
    EnetApp_releaseHandleInfo(gEnetApp.enetType, gEnetApp.instId);
    gEnetApp.hEnet = NULL;

    EnetApp_driverDeInit();

    /* Do peripheral dependent deinitalization */
    EnetAppUtils_disableClocks(gEnetApp.enetType, gEnetApp.instId);
}

static int32_t EnetApp_waitForLinkUp(Enet_MacPort macPort)
{
    Enet_IoctlPrms prms;
    bool linked = false;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("Port %u: Waiting for link up...\r\n", ENET_MACPORT_ID(macPort));

    while (gEnetApp.run && !linked)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

        ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get link status: %d\r\n", status);
            linked = false;
            break;
        }

        if (!linked)
        {
            ClockP_sleep(1);
        }
    }

    if (gEnetApp.run && (status == ENET_SOK))
    {
        EnetAppUtils_print("Port %u: Link is %s\r\n", ENET_MACPORT_ID(macPort), linked ? "up" : "down");
    }

    return status;
}

void EnetApp_printStats(void)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    int32_t status;

    /* Get host port statistics */
    EnetAppUtils_print("\n   Host Port statistics\r\n");
    EnetAppUtils_print("--------------------------------\r\n");

    ENET_IOCTL_SET_OUT_ARGS(&prms, &gEnetApp_cpswStats);
    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to get host port stats\r\n");
    }
    else
    {
        EnetAppUtils_printHostPortStats9G((CpswStats_HostPort_Ng *)&gEnetApp_cpswStats);
        EnetAppUtils_print("\n");
    }

    /* Get MAC ports statistics */
    for (i = 0U; i < gEnetApp.macPortNum; i++)
    {
        macPort = gEnetApp.macPort[i];

        EnetAppUtils_print("   MAC Port %u statistics\r\n", ENET_MACPORT_ID(macPort));
        EnetAppUtils_print("--------------------------------\r\n");

        ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &gEnetApp_cpswStats);
        ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get MAC port %u stats\r\n", ENET_MACPORT_ID(macPort));
            continue;
        }

        EnetAppUtils_printMacPortStats9G((CpswStats_MacPort_Ng *)&gEnetApp_cpswStats);
        EnetAppUtils_print("\n");
    }
}

void EnetApp_resetStats(void)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    int32_t status;

    ENET_IOCTL_SET_NO_ARGS(&prms);
    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_RESET_HOSTPORT_STATS, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to reset host port stats\r\n");
    }

    for (i = 0U; i < gEnetApp.macPortNum; i++)
    {
        macPort = gEnetApp.macPort[i];

        ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
        ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_RESET_MACPORT_STATS, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to reset MAC port %u stats\r\n", ENET_MACPORT_ID(macPort));
            continue;
        }
    }
}

static void EnetApp_printEstList(EnetTas_ControlList *list)
{
    uint8_t gateMask = 0U;
    uint32_t start = 0U;
    uint32_t end;
    uint32_t dur;
    uint32_t i;

    for (i = 0U; i < list->listLength; i++)
    {
        gateMask = list->gateCmdList[i].gateStateMask;
        dur = list->gateCmdList[i].timeInterval;
        end = start + dur - 1U;

        /* o = Gate open, C = Gate closed */
        EnetAppUtils_print("Gate mask=%s%s%s%s%s%s%s%s (0x%02x), start=%u ns, end=%u ns, dur=%u ns\r\n",
                           ENET_IS_BIT_SET(gateMask, 7U) ? "o" : "C",
                           ENET_IS_BIT_SET(gateMask, 6U) ? "o" : "C",
                           ENET_IS_BIT_SET(gateMask, 5U) ? "o" : "C",
                           ENET_IS_BIT_SET(gateMask, 4U) ? "o" : "C",
                           ENET_IS_BIT_SET(gateMask, 3U) ? "o" : "C",
                           ENET_IS_BIT_SET(gateMask, 2U) ? "o" : "C",
                           ENET_IS_BIT_SET(gateMask, 1U) ? "o" : "C",
                           ENET_IS_BIT_SET(gateMask, 0U) ? "o" : "C",
                           gateMask,
                           start, end, dur);

        start += dur;
    }

    EnetAppUtils_print("Cycle time=%llu ns\r\n", list->cycleTime);
    EnetAppUtils_print("Base time=%llu ns\r\n", list->baseTime);
}

static void EnetApp_showAdminList(Enet_MacPort macPort)
{
    Enet_IoctlPrms prms;
    EnetTas_GenericInArgs inArgs;
    EnetTas_ControlList adminList;
    int32_t status;

    inArgs.macPort = macPort;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &adminList);

    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_TAS_IOCTL_GET_ADMIN_LIST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to get TAS admin list: %d\r\n", status);
    }
    else
    {
        EnetAppUtils_print("\r\nMAC %u: Admin List\r\n", ENET_MACPORT_ID(macPort));
        EnetAppUtils_print("-------------------------------------------\r\n");
        EnetApp_printEstList(&adminList);
    }
}

static void EnetApp_showOperList(Enet_MacPort macPort)
{
    Enet_IoctlPrms prms;
    EnetTas_GenericInArgs inArgs;
    EnetTas_ControlList operList;
    int32_t status;

    inArgs.macPort = macPort;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &operList);

    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_TAS_IOCTL_GET_OPER_LIST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to get TAS oper list: %d\r\n", status);
    }
    else
    {
        EnetAppUtils_print("\r\nMAC %u: Oper List\r\n", ENET_MACPORT_ID(macPort));
        EnetAppUtils_print("-------------------------------------------\r\n");
        EnetApp_printEstList(&operList);
    }
}

static uint64_t EnetApp_getTestAdminBaseTime(Enet_MacPort macPort,
                                             EnetTas_TasState state)
{
    uint64_t tsVal;
    uint32_t i;

    /* If EST is not enabled (reset or disabled), CPSW driver allows setting an
     * admin basetime in the future.
     * If EST is already enabled, CPSW driver only allows setting an admin time
     * in the past, so setting it to 0. */
    if ((state == ENET_TAS_DISABLE) ||
        (state == ENET_TAS_RESET))
    {
        tsVal = EnetApp_getCurrentTime() + ENET_APP_EST_ADMIN_LIST_DELAY;
        EnetAppUtils_print("MAC Port %u: suggested start time: %llu\r\n", ENET_MACPORT_ID(macPort), tsVal);

        /* Save the ESTF start time in order to compute the normalized timestamp of EST
         * events at a later point.  This is needed solely for test verification purpose */
        for (i = 0U; i < gEnetApp.macPortNum; i++)
        {
            if (gEnetApp.macPort[i] == macPort)
            {
                gEnetApp.estfStartTime[i] = tsVal;
            }
        }
    }
    else
    {
        tsVal = 0ULL;
    }

    return tsVal;
}

int32_t EnetApp_setAdminList(Enet_MacPort macPort,
                             EnetTas_ControlList *controlList)
{
    Enet_IoctlPrms prms;
    EnetTas_SetAdminListInArgs adminListInArgs;
    EnetTas_TasState state;
    EnetTas_OperStatus operStatus = ENET_TAS_OPER_LIST_NOT_YET_UPDATED;
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Get the current EST state */
    state = EnetApp_getEstState(macPort);

    /* Setup new TAS admin list */
    adminListInArgs.macPort = macPort;
    memcpy(&adminListInArgs.adminList, controlList, sizeof(EnetTas_ControlList));
    /* Update baseTime to future time if EST is not enabled */
    if (controlList->baseTime == 0ULL)
    {
        adminListInArgs.adminList.baseTime = EnetApp_getTestAdminBaseTime(macPort, state);
        controlList->baseTime = adminListInArgs.adminList.baseTime;
    }
    ENET_IOCTL_SET_IN_ARGS(&prms, &adminListInArgs);

    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_TAS_IOCTL_SET_ADMIN_LIST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set TAS admin list: %d\r\n", status);
    }

    /* Print admin list */
    if (status == ENET_SOK)
    {
        EnetApp_showAdminList(macPort);
    }

    /* Wait until the operational list is updated */
    if (status == ENET_SOK)
    {
        for (i = 0U; i < ENET_APP_OPER_LIST_UPDATE_CHECK_RETRY_MAX; i++)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &operStatus);

            ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_TAS_IOCTL_GET_OPER_LIST_STATUS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("Failed to check TAS operational list update status: %d\r\n", status);
                break;
            }

            if (operStatus == ENET_TAS_OPER_LIST_UPDATED)
            {
                break;
            }
        }
    }

    return status;
}

int32_t EnetApp_setEstState(Enet_MacPort macPort,
                            EnetTas_TasState state)
{
    Enet_IoctlPrms prms;
    EnetTas_SetStateInArgs setStateInArgs;
    int32_t status;

    /* Set TAS state to requested state */
    setStateInArgs.macPort = macPort;
    setStateInArgs.state   = state;
    ENET_IOCTL_SET_IN_ARGS(&prms, &setStateInArgs);

    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_TAS_IOCTL_SET_STATE, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set TAS state %u: %d\r\n", (uint32_t)state, status);
    }
    else
    {
        EnetAppUtils_print("TAS state is set to %u\r\n", (uint32_t)state);
    }

    return status;
}

EnetTas_TasState EnetApp_getEstState(Enet_MacPort macPort)
{
    Enet_IoctlPrms prms;
    EnetTas_GenericInArgs stateInArgs;
    EnetTas_TasState state = ENET_TAS_RESET;
    int32_t status;

    /* Get the current EST state */
    stateInArgs.macPort = macPort;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &stateInArgs, &state);

    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_TAS_IOCTL_GET_STATE, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to get TAS state: %d\r\n", status);
    }

    return state;
}

static int32_t EnetApp_setupEst(Enet_MacPort macPort,
                                EnetTas_ControlList *controlList)
{
    int32_t status;

    /* Set TAS state to 'RESET' */
    status = EnetApp_setEstState(macPort, ENET_TAS_RESET);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set TAS state to 'RESET': %d\r\n", status);
    }

    /* Setup TAS admin list */
    if (status == ENET_SOK)
    {
        status = EnetApp_setAdminList(macPort, controlList);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to set admin list: %d\r\n", status);
        }
    }

    /* Set TAS state to 'ENABLE' */
    if (status == ENET_SOK)
    {
        status = EnetApp_setEstState(macPort, ENET_TAS_ENABLE);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to set TAS state to 'ENABLE': %d\r\n", status);
        }
    }

    /* Print operational list */
    if (status == ENET_SOK)
    {
        EnetApp_showOperList(macPort);
    }

    return status;
}

