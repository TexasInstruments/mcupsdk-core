/*
 *  Copyright (c) Texas Instruments Incorporated 2022-23
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
 * \file  tsnapp_lwip_cpsw_main.c
 *
 * \brief This file contains the implementation of the Enet CPSW gPTP stack along with LwIP stack.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/QueueP.h>

#include <enet_apputils.h>
#include <enet_board.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "nrt_flow/app_tcpserver.h"
#include "ti_enet_lwipif.h"
#include <tsn_combase/combase.h>
#include "debug_log.h"
#include "nrt_flow/dataflow.h"
#include "tsninit.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct EnetApp_AppEnetInfo
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* MAC ports List to use for the above EnetType & InstId*/
    uint8_t     numMacPort;

    /* Num MAC ports to use for the above EnetType & InstId*/
    Enet_MacPort macPortList[ENET_SYSCFG_MAX_MAC_PORTS];

    uint8_t macAddr[ENET_MAC_ADDR_LEN];
} EnetApp_AppEnetInfo;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void EnetApp_printCpuLoad();

static void EnetApp_tcpipInitCompleteCb(void *pArg);

static void EnetApp_setupNetif();

static void EnetApp_allocateIPAddress();

static void EnetApp_setupNetworkStack();

static void EnetApp_shutdownNetworkStack();

static void EnetApp_netifStatusChangeCb(struct netif *state_netif);

static void EnetApp_netifLinkChangeCb(struct netif *state_netif);

static inline int32_t EnetApp_isNetworkUp(struct netif* netif_);

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg);

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg);

static void EnetApp_addMCastEntry(Enet_Type enetType,
                                  uint32_t instId,
                                  uint32_t coreId,
                                  const uint8_t *testMCastAddr,
                                  uint32_t portMask);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* dhcp struct for the ethernet netif */
static struct dhcp g_netifDhcp[ENET_SYSCFG_NETIF_COUNT];
static struct netif *g_pNetif[ENET_SYSCFG_NETIF_COUNT];
static EnetApp_AppEnetInfo gEnetAppParams;

/* Handle to the Application interface for the LwIPIf Layer
 */
LwipifEnetApp_Handle hlwipIfApp = NULL;

#define EnetAppAbort(message) \
    EnetAppUtils_print(message);                \
    EnetAppUtils_assert(false);

/* these vars are shared with gptp task to configure gptp, put it in the global mem */
static char g_netdevices[MAX_NUM_MAC_PORTS][CB_MAX_NETDEVNAME] = {0};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#define LOG_BUFFER_SIZE (1024)

void ConsolePrint(const char *pcString, ...)
{
    /* Use DebugP_log() because EnetAppUtils_print() has limit bufsize */
    va_list args;
    char buffer[LOG_BUFFER_SIZE];

    va_start(args, pcString);
    vsnprintf(buffer, sizeof(buffer), pcString, args);
    va_end(args);

    DebugP_log("%s", buffer);
}

static int EnetApp_initTsn(void)
{
    lld_ethdev_t ethdevs[MAX_NUMBER_ENET_DEVS] = {0};
    int i;
    int res = 0;
    AppTsnCfg_t appCfg =
    {
        .consoleOutCb = ConsolePrint,
    };

    for (i = 0; i < gEnetAppParams.numMacPort; i++)
    {
        snprintf(&g_netdevices[i][0], CB_MAX_NETDEVNAME, "tilld%d", i);
        appCfg.netdevs[i] = &g_netdevices[i][0];
        ethdevs[i].netdev = g_netdevices[i];
        ethdevs[i].macport = gEnetAppParams.macPortList[i];
        if (i == 0)
        {
            /* tilld0 reuses the allocated source mac, other interfaces will allocate
             * the mac by themself */
            memcpy(ethdevs[i].srcmac, gEnetAppParams.macAddr, ENET_MAC_ADDR_LEN);
        }
    }
    appCfg.netdevs[i] = NULL;
    if (EnetApp_initTsnByCfg(&appCfg) < 0)
    {
        EnetAppAbort("Failed to int tsn!\r\n");
    }
    if (cb_lld_init_devs_table(ethdevs, i, gEnetAppParams.enetType,
                               gEnetAppParams.instId) < 0)
    {
        EnetAppAbort("Failed to int devs table!\r\n");
    }
    cb_socket_set_lldcfg_update_cb(EnetApp_lldCfgUpdateCb);

    if (EnetApp_startTsn() < 0)
    {
        EnetAppAbort("Failed to start TSN App!\r\n");
    }
    EnetAppUtils_print("%s:TSN app start done!\r\n", __func__);

    return res;
}

static bool IsMacAddrSet(uint8_t *mac)
{
    return ((mac[0]|mac[1]|mac[2]|mac[3]|mac[4]|mac[5]) != 0);
}

uint32_t EnetApp_applyClassifier(Enet_Handle hEnet, uint32_t coreId, uint8_t *dstMacAddr, uint32_t vlanId,
                                    uint32_t ethType, uint32_t rxFlowIdx)
{
    Enet_IoctlPrms prms;
    CpswAle_SetPolicerEntryOutArgs setPolicerEntryOutArgs;
    CpswAle_SetPolicerEntryInArgs setPolicerEntryInArgs;
    int32_t status;

    if (IsMacAddrSet(dstMacAddr) == true)
    {
        status = EnetAppUtils_addAllPortMcastMembership(hEnet, dstMacAddr);
        if (status != ENET_SOK) {
            EnetAppUtils_print("EnetAppUtils_addAllPortMcastMembership failed: %d\r\n", status);
        }
    }
    memset(&setPolicerEntryInArgs, 0, sizeof (setPolicerEntryInArgs));

    if (ethType > 0) {
        setPolicerEntryInArgs.policerMatch.policerMatchEnMask |=
            CPSW_ALE_POLICER_MATCH_ETHERTYPE;
        setPolicerEntryInArgs.policerMatch.etherType = ethType;
    }
    setPolicerEntryInArgs.policerMatch.portIsTrunk = false;
    setPolicerEntryInArgs.threadIdEn = true;
    setPolicerEntryInArgs.threadId = rxFlowIdx;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &setPolicerEntryInArgs, &setPolicerEntryOutArgs);
    ENET_IOCTL(hEnet, coreId,
            CPSW_ALE_IOCTL_SET_POLICER, &prms, status);
    return status;
}

static void EnetApp_enableTsSync()
{
    Enet_IoctlPrms prms;
    CpswCpts_OutputBitSel bitSelect;
    int32_t status;

    Enet_Handle hEnet = Enet_getHandle(gEnetAppParams.enetType, gEnetAppParams.instId);
    bitSelect = CPSW_CPTS_TS_OUTPUT_BIT_17;
    ENET_IOCTL_SET_IN_ARGS(&prms, &bitSelect);
    ENET_IOCTL(hEnet, EnetSoc_getCoreId(), CPSW_CPTS_IOCTL_SELECT_TS_OUTPUT_BIT, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to set TS SYNC OUT BIT : %d\r\n", status);
    }
    return;
}

int EnetApp_mainTask(void *args)
{
    int32_t status = ENET_SOK;
    EnetApp_GetMacAddrOutArgs outArgs;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==================================\r\n");
    DebugP_log("  CPSW GPTP LWIP TCP ECHO SERVER \r\n");
    DebugP_log("==================================\r\n");

    /* Read MAC Port details and enable clock for ENET instance */
    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0 , &gEnetAppParams.enetType, &gEnetAppParams.instId);
    EnetApp_getEnetInstMacInfo(gEnetAppParams.enetType,
                               gEnetAppParams.instId,
                               &gEnetAppParams.macPortList[0],
                               &gEnetAppParams.numMacPort);
    EnetAppUtils_enableClocks(gEnetAppParams.enetType, gEnetAppParams.instId);

    /* Open ENET driver for the ENET instance */
    EnetApp_driverInit();
    status = EnetApp_driverOpen(gEnetAppParams.enetType, gEnetAppParams.instId);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        EnetAppUtils_assert(status == ENET_SOK);
    }
    EnetApp_getMacAddress(ENET_DMA_RX_CH0, &outArgs);
    EnetAppUtils_assert(outArgs.macAddressCnt == 1);
    EnetUtils_copyMacAddr(gEnetAppParams.macAddr, outArgs.macAddr[outArgs.macAddressCnt - 1]);
    EnetApp_addMCastEntry(gEnetAppParams.enetType,
                          gEnetAppParams.instId,
                          EnetSoc_getCoreId(),
                          BROADCAST_MAC_ADDRESS,
                          CPSW_ALE_ALL_PORTS_MASK);

    EnetApp_enableTsSync();
    if (EnetApp_initTsn())
    {
        DebugP_log("EnetApp_initTsn failed\r\n");
    }
    EnetApp_setupNetworkStack();

    uint32_t netupMask = 0;
    /* wait for atleast one Network Interface to get IP */
    while (netupMask == 0)
    {
        for(uint32_t netifIdx = 0; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
        {
            if (EnetApp_isNetworkUp(g_pNetif[netifIdx]))
            {
                netupMask |= (1 << netifIdx);
            }
            else
            {
                DebugP_log("[%d]Waiting for network UP ...\r\n",g_pNetif[netifIdx]->num);
            }
            ClockP_sleep(2);
        }
    }

    DebugP_log("Network is UP ...\r\n");
    ClockP_sleep(2);
    AppTcp_startServer();

    while (1)
    {
        ClockP_usleep(1000);
        EnetApp_printCpuLoad();
        TaskP_yield();
    }
    EnetApp_stopTsn();
    EnetApp_deInitTsn();

    EnetApp_shutdownNetworkStack();
    return 0;
}

static void EnetApp_setupNetworkStack()
{
    sys_sem_t pInitSem;
    const err_t err = sys_sem_new(&pInitSem, 0);
    EnetAppUtils_assert(err == ERR_OK);

    tcpip_init(EnetApp_tcpipInitCompleteCb, &pInitSem);

    /* wait for TCP/IP initialization to complete */
    sys_sem_wait(&pInitSem);
    sys_sem_free(&pInitSem);

    return;
}

static void EnetApp_shutdownNetworkStack()
{
    for (uint32_t netifIdx = 0U; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
    {
        LwipifEnetApp_netifClose(hlwipIfApp, NETIF_INST_ID0 + netifIdx);
    }
    return;
}

static void EnetApp_tcpipInitCompleteCb(void *pArg)
{
    sys_sem_t *pSem = (sys_sem_t*)pArg;
    EnetAppUtils_assert(pArg != NULL);

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    EnetApp_setupNetif();

    EnetApp_allocateIPAddress();

    sys_sem_signal(pSem);
}

static void EnetApp_setupNetif()
{
    ip4_addr_t ipaddr, netmask, gw;

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    DebugP_log("Starting lwIP, local interface IP is dhcp-enabled\r\n");
    hlwipIfApp = LwipifEnetApp_getHandle();
    for (uint32_t netifIdx = 0U; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
    {
        /* Open the netif and get it populated*/
        g_pNetif[netifIdx] = LwipifEnetApp_netifOpen(hlwipIfApp, NETIF_INST_ID0 + netifIdx, &ipaddr, &netmask, &gw);
        netif_set_status_callback(g_pNetif[netifIdx], EnetApp_netifStatusChangeCb);
        netif_set_link_callback(g_pNetif[netifIdx], EnetApp_netifLinkChangeCb);
        netif_set_up(g_pNetif[NETIF_INST_ID0 + netifIdx]);
    }
    LwipifEnetApp_startSchedule(hlwipIfApp, g_pNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX]);
}

static void EnetApp_allocateIPAddress()
{
    sys_lock_tcpip_core();
    for (uint32_t  netifIdx = 0U; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
    {
        dhcp_set_struct(g_pNetif[NETIF_INST_ID0 + netifIdx], &g_netifDhcp[NETIF_INST_ID0 + netifIdx]);

        const err_t err = dhcp_start(g_pNetif[NETIF_INST_ID0 + netifIdx]);
        EnetAppUtils_assert(err == ERR_OK);
    }
    sys_unlock_tcpip_core();
    return;
}

static void EnetApp_netifStatusChangeCb(struct netif *pNetif)
{
    if (netif_is_up(pNetif))
    {
        DebugP_log("[%d]Enet IF UP Event. Local interface IP:%s\r\n",
                    pNetif->num, ip4addr_ntoa(netif_ip4_addr(pNetif)));
    }
    else
    {
        DebugP_log("[%d]Enet IF DOWN Event\r\n", pNetif->num);
    }
    return;
}

static void EnetApp_netifLinkChangeCb(struct netif *pNetif)
{
    if (netif_is_link_up(pNetif))
    {
        DebugP_log("[%d]Network Link UP Event\r\n", pNetif->num);
    }
    else
    {
        DebugP_log("[%d]Network Link DOWN Event\r\n", pNetif->num);
    }
    return;
}

static int32_t EnetApp_isNetworkUp(struct netif* netif_)
{
    return (netif_is_up(netif_) && netif_is_link_up(netif_) && !ip4_addr_isany_val(*netif_ip4_addr(netif_)));
}

static void EnetApp_printCpuLoad()
{
    static uint32_t startTime_ms = 0;
    const  uint32_t currTime_ms  = ClockP_getTimeUsec()/1000;
    const  uint32_t printInterval_ms = 5000;

    if (startTime_ms == 0)
    {
        startTime_ms = currTime_ms;
    }
    else if ( (currTime_ms - startTime_ms) > printInterval_ms )
    {
        const uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
                    currTime_ms/1000, currTime_ms%1000,
                    cpuLoad/100, cpuLoad%100 );

        startTime_ms = currTime_ms;
        TaskP_loadResetAll();
    }
    return;
}

void EnetApp_addMCastEntry(Enet_Type enetType,
                          uint32_t instId,
                          uint32_t coreId,
                          const uint8_t *testMCastAddr,
                          uint32_t portMask)
{
    Enet_IoctlPrms prms;
    int32_t status;
    CpswAle_SetMcastEntryInArgs setMcastInArgs;
    uint32_t setMcastOutArgs;

    if (Enet_isCpswFamily(enetType))
    {
        Enet_Handle hEnet = Enet_getHandle(enetType, instId);

        EnetAppUtils_assert(hEnet != NULL);
        memset(&setMcastInArgs, 0, sizeof(setMcastInArgs));
        memcpy(&setMcastInArgs.addr.addr[0U], testMCastAddr,
               sizeof(setMcastInArgs.addr.addr));
        setMcastInArgs.addr.vlanId  = 0;
        setMcastInArgs.info.super = false;
        setMcastInArgs.info.numIgnBits = 0;
        setMcastInArgs.info.fwdState = CPSW_ALE_FWDSTLVL_FWD;
        setMcastInArgs.info.portMask = portMask;
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &setMcastInArgs, &setMcastOutArgs);
        ENET_IOCTL(hEnet, coreId, CPSW_ALE_IOCTL_ADD_MCAST, &prms, status);
        if (status != ENET_SOK)
        {
           EnetAppUtils_print("EnetTestBcastMcastLimit_AddAleEntry() failed CPSW_ALE_IOCTL_ADD_MCAST: %d\n",
                               status);
        }
    }
}


void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA)
    EnetCpdma_Cfg * dmaCfg = (EnetCpdma_Cfg *)cpswCfg->dmaCfg;

    EnetAppUtils_assert(dmaCfg != NULL);
    EnetAppUtils_assert(EnetAppUtils_isDescCached() == false);
    dmaCfg->rxInterruptPerMSec = 8;
    dmaCfg->txInterruptPerMSec = 2;
    dmaCfg->enChOverrideFlag = true;
#endif


#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb    = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb    = &EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#endif
    cpswCfg->portLinkStatusChangeCb = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;
}

static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

int32_t EnetApp_filterPriorityPacketsCfg(Enet_Handle hEnet, uint32_t coreId)
{
    EnetMacPort_SetPriorityRegenMapInArgs params;
    Enet_IoctlPrms prms;
    int32_t retVal = ENET_SOK;

    params.macPort = ENET_MAC_PORT_1;

    params.priorityRegenMap.priorityMap[0] =0U;
    for (int i = 1; i < 8U; i++)
    {
        params.priorityRegenMap.priorityMap[i] =1U;  // Map all priorities from (1 to 7) to priority 1, these packets will be received on DMA channel 1.
    }

    ENET_IOCTL_SET_IN_ARGS(&prms, &params);

    ENET_IOCTL(hEnet, coreId, ENET_MACPORT_IOCTL_SET_PRI_REGEN_MAP, &prms, retVal);

    return retVal;
}
