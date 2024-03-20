/*
 * Copyright (c) 2001,2002 Florian Schulze.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * app_main.c - This file is part of lwIP test
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "lwip/timeouts.h"
#include <apps/tcpecho_raw/tcpecho_raw.h>

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/EventP.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_board.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "app_cpswconfighandler.h"
#include "ti_enet_lwipif.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

static const uint8_t BROADCAST_MAC_ADDRESS[ENET_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

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

static void App_handleEvent(const uint32_t eventMask);

static uint32_t App_receiveEvents(EventP_Object* pEvent);

void httpd_init(void);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* dhcp struct for the ethernet netif */
static struct dhcp g_netifDhcp[ENET_SYSCFG_NETIF_COUNT];
struct netif *g_pNetif[ENET_SYSCFG_NETIF_COUNT];

EventP_Object hEvent;

/* Handle to the Application interface for the LwIPIf Layer
 */
LwipifEnetApp_Handle hlwipIfApp = NULL;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int appMain(void *args)
{
    Enet_Type enetType;
    uint32_t instId;
    int32_t status;
    status = EventP_construct(&hEvent);
    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log("  CPSW LWIP HTTP WEB SERVER \r\n");
    DebugP_log("==========================\r\n");

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType, &instId);

    EnetAppUtils_enableClocks(enetType, instId);

    EnetApp_driverInit();

    status = EnetApp_driverOpen(enetType, instId);
    EnetApp_initPhyStateHandlerTask(&hEvent);
    if (ENET_SOK != status)
    {
        EnetAppUtils_print("Failed to open ENET: %d\r\n", status);
        EnetAppUtils_assert(false);
        return -1;
    }

    EnetApp_addMCastEntry(enetType,
                          instId,
                          EnetSoc_getCoreId(),
                          BROADCAST_MAC_ADDRESS,
                          CPSW_ALE_ALL_PORTS_MASK);
    //call createPhyHandler here


    App_setupNetworkStack();

    if (ENET_SOK != status)
    {
        EnetAppUtils_print("Failed to construct Event: %d\r\n", status);
        EnetAppUtils_assert(false);
        return -1;
    }

    while (false == App_isNetworkUp(netif_default))
    {
        //DebugP_log("Waiting for network UP ...\r\n");
        sys_check_timeouts();

        const uint32_t recvdEventsMask = App_receiveEvents(&hEvent);

        if (recvdEventsMask != AppEventId_NONE)
        {
            App_handleEvent(recvdEventsMask);
        }
    }

    DebugP_log("Network is UP ...\r\n");
    ClockP_sleep(1);

//    AppTcp_startServer();
    tcpecho_raw_init();
    httpd_init();

    while (1)
    {
        sys_check_timeouts();

        const uint32_t recvdEventsMask = App_receiveEvents(&hEvent);

        if (recvdEventsMask != AppEventId_NONE)
        {
            App_handleEvent(recvdEventsMask);
        }

    }

    App_shutdownNetworkStack();
    EnetApp_driverDeInit();
    EventP_destruct(&hEvent);
    return 0;
}


static void App_handleEvent(const uint32_t eventMask)
{

    // lwip raw API examples learn about

    if (AppEventId_NETIFMNGR_RXPKT & eventMask)
    {
        LWIPIF_LWIP_rxPktHandler(g_pNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX]);
    }

    if (AppEventId_NETIFMNGR_POLL & eventMask)
    {
        LWIPIF_LWIP_periodic_polling(g_pNetif[ENET_SYSCFG_NETIF_COUNT - 1]);
    }

    if ( AppEventId_NETIFMNGR_TXPKT & eventMask)
    {
        LWIPIF_LWIP_txPktHandler(g_pNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX]);
    }

    if (AppEventId_CPSW_PERIODIC_POLL & eventMask)
    {
        EnetApp_phyStateHandler();
    }
}

static uint32_t App_receiveEvents(EventP_Object* pEvent)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t recvdEventsMask = AppEventId_NONE;

    status = EventP_waitBits(pEvent,
                        AppEventId_ANY_EVENT, // bitsToWaitFor
                        1,
                        0,
                        SystemP_NO_WAIT,
                        &recvdEventsMask);

     if ((status != SystemP_SUCCESS) && (status != SystemP_TIMEOUT))
     {
         EnetAppUtils_print("Failed to receive Event handle\r\n");
         EnetAppUtils_assert(false);
     }

     return recvdEventsMask;
}

static void App_setupNetworkStack()
{
    lwip_init();

    srand(ClockP_getTicks()/1000);

    App_setupNetif();

    App_allocateIPAddress();

    return;
}

static void App_shutdownNetworkStack()
{
    return;
}

static void App_setupNetif()
{
    ip4_addr_t ipaddr, netmask, gw;

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    DebugP_log("Starting lwIP, local interface IP is dhcp-enabled\r\n");
    hlwipIfApp = LwipifEnetApp_getHandle();
    for (uint32_t i = 0U; i < ENET_SYSCFG_NETIF_COUNT; i++)
    {
        /* Open the netif and get it populated*/
        g_pNetif[i] = LwipifEnetApp_netifOpen(hlwipIfApp, NETIF_INST_ID0 + i, &ipaddr, &netmask, &gw);
        netif_set_status_callback(g_pNetif[i], App_netifStatusChangeCb);
        netif_set_link_callback(g_pNetif[i], App_netifLinkChangeCb);
        netif_set_up(g_pNetif[NETIF_INST_ID0 + i]);
    }
    LwipifEnetApp_startSchedule(hlwipIfApp, g_pNetif[ENET_SYSCFG_DEFAULT_NETIF_IDX], &hEvent);
}

static void App_allocateIPAddress()
{
    for (uint32_t  i = 0U; i < ENET_SYSCFG_NETIF_COUNT; i++)
    {
        dhcp_set_struct(g_pNetif[NETIF_INST_ID0 + i], &g_netifDhcp[NETIF_INST_ID0 + i]);

        const err_t err = dhcp_start(g_pNetif[NETIF_INST_ID0 + i]);
        EnetAppUtils_assert(err == ERR_OK);
    }
    return;
}

static void App_netifStatusChangeCb(struct netif *pNetif)
{
    if (netif_is_up(pNetif))
    {
        DebugP_log("Enet IF UP Event. Local interface IP:%s\r\n",
                    ip4addr_ntoa(netif_ip4_addr(pNetif)));
    }
    else
    {
        DebugP_log("Enet IF DOWN Event\r\n");
    }
    return;
}

static void App_netifLinkChangeCb(struct netif *pNetif)
{
    if (netif_is_link_up(pNetif))
    {
        DebugP_log("Network Link UP Event\r\n");
    }
    else
    {
        DebugP_log("Network Link DOWN Event\r\n");
    }
    return;
}

static int32_t App_isNetworkUp(struct netif* netif_)
{
    return (netif_is_up(netif_) && netif_is_link_up(netif_) && !ip4_addr_isany_val(*netif_ip4_addr(netif_)));
}

#ifndef NO_SYS
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
#endif
