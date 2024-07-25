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
 * \brief This file contains the implementation of the Enet ICSSG gPTP stack along with LwIP stack.
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
#include "nrt_flow/app_tcpserver.h"
#include "ti_enet_lwipif.h"
#include <tsn_combase/combase.h>
#include "debug_log.h"
#include "nrt_flow/dataflow.h"
#include "tsninit.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

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
static void App_printCpuLoad();

static void App_tcpipInitCompleteCb(void *pArg);

static void App_setupNetif();

static void App_allocateIPAddress();

static void App_setupNetworkStack();

static void App_shutdownNetworkStack();

static void App_netifStatusChangeCb(struct netif *state_netif);

static void App_netifLinkChangeCb(struct netif *state_netif);

static inline int32_t App_isNetworkUp(struct netif* netif_);


#if (ENET_SYSCFG_ENABLE_EXTPHY == 0U)
static void EnetApp_mdioLinkStatusChange(Icssg_MdioLinkStateChangeInfo *info,
                                         void *appArg);
#endif

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#define MII_LINK0_EVENT      41
#define MII_LINK1_EVENT      53

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

int EnetApp_mainTask(void *args)
{
    int32_t status = ENET_SOK;
    EnetApp_GetMacAddrOutArgs outArgs;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("===============================\r\n");
    DebugP_log("ICSSG GPTP LWIP TCP ECHO SERVER\r\n");
    DebugP_log("================================\r\n");

    /* Read MAC Port details and enable clock for ENET instance */
    EnetApp_getEnetInstInfo(CONFIG_ENET_ICSS0 , &gEnetAppParams.enetType, &gEnetAppParams.instId);
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
    if (EnetApp_initTsn())
    {
        DebugP_log("EnetApp_initTsn failed\r\n");
    }
    App_setupNetworkStack();

    uint32_t netupMask = 0;
    /* wait for atleast one Network Interface to get IP */
    while (netupMask == 0)
    {
        for(uint32_t netifIdx = 0; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
        {
            if (App_isNetworkUp(g_pNetif[netifIdx]))
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
        App_printCpuLoad();
        TaskP_yield();
    }

    EnetApp_stopTsn();
    EnetApp_deInitTsn();
    App_shutdownNetworkStack();
    return 0;
}

static void App_setupNetworkStack()
{
    sys_sem_t pInitSem;
    const err_t err = sys_sem_new(&pInitSem, 0);
    EnetAppUtils_assert(err == ERR_OK);

    tcpip_init(App_tcpipInitCompleteCb, &pInitSem);

    /* wait for TCP/IP initialization to complete */
    sys_sem_wait(&pInitSem);
    sys_sem_free(&pInitSem);

    return;
}

static void App_shutdownNetworkStack()
{
    for (uint32_t netifIdx = 0U; netifIdx < ENET_SYSCFG_NETIF_COUNT; netifIdx++)
    {
        LwipifEnetApp_netifClose(hlwipIfApp, NETIF_INST_ID0 + netifIdx);
    }
    return;
}

static void App_tcpipInitCompleteCb(void *pArg)
{
    sys_sem_t *pSem = (sys_sem_t*)pArg;
    EnetAppUtils_assert(pArg != NULL);

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    App_setupNetif();

    App_allocateIPAddress();

    sys_sem_signal(pSem);
}

static void App_setupNetif()
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
        LwipifEnetApp_startSchedule(hlwipIfApp, g_pNetif[netifIdx]);
        netif_set_status_callback(g_pNetif[netifIdx], App_netifStatusChangeCb);
        netif_set_link_callback(g_pNetif[netifIdx], App_netifLinkChangeCb);
        netif_set_up(g_pNetif[NETIF_INST_ID0 + netifIdx]);
    }
}

static void App_allocateIPAddress()
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

static void App_netifStatusChangeCb(struct netif *pNetif)
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

static void App_netifLinkChangeCb(struct netif *pNetif)
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

static int32_t App_isNetworkUp(struct netif* netif_)
{
    return (netif_is_up(netif_) && netif_is_link_up(netif_) && !ip4_addr_isany_val(*netif_ip4_addr(netif_)));
}

static void App_printCpuLoad()
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

static void EnetApp_updateMdioLinkIntCfg(Enet_Type enetType, uint32_t instId, Icssg_mdioLinkIntCfg *mdioLinkIntCfg)
{
    /*! INTC Module mapping data passed by application for configuring PRU to R5F interrupts */
#if (ENET_SYSCFG_ICSSG0_ENABLED == 1)
    mdioLinkIntCfg->prussIntcInitData =  &icss0_intc_initdata;
#endif
#if (ENET_SYSCFG_ICSSG1_ENABLED == 1)
    mdioLinkIntCfg->prussIntcInitData =  &icss1_intc_initdata;
#endif
    mdioLinkIntCfg->coreIntrNum = 254;
    mdioLinkIntCfg->pruEvtNum[0] = MII_LINK0_EVENT;
    mdioLinkIntCfg->pruEvtNum[1] = MII_LINK1_EVENT;
    mdioLinkIntCfg->isPulseIntr = 0;
    mdioLinkIntCfg->intrPrio = 15;
}

void EnetApp_updateIcssgInitCfg(Enet_Type enetType, uint32_t instId, Icssg_Cfg *icssgCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = NULL;
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = NULL;
#else
    #if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
        EnetApp_initMdioLinkIntCfg(enetType, instId, icssgCfg);
    #else
        icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = &EnetApp_mdioLinkStatusChange;
        icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = NULL;
    #endif
    EnetApp_updateMdioLinkIntCfg(enetType, instId, &icssgCfg->mdioLinkIntCfg);
#endif
}

#if (ENET_SYSCFG_ENABLE_EXTPHY == 0U)
static void EnetApp_mdioLinkStatusChange(Icssg_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
            info->phyAddr,
            info->isLinked? "up" : "down");
}
#endif

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}
