/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 * \file  app_netif.c
 *
 * \brief This file contains the implementation of the Enet Remote Connect example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "netif/bridgeif.h"
#include <examples/lwiperf/lwiperf_example.h>

#include <lwipific/inc/lwip_ic.h>
#include <lwipific/inc/lwip2lwipif_ic.h>

#include <lwip2lwipif.h>
#include <custom_pbuf.h>
#include "ti_enet_config.h"
#include "ti_enet_lwipif.h"
#include "netif_common.h"
#include "app_netif.h"
#include "udp_iperf.h"
#include "ti_ic_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Max length of shared mcast address list */
#define ETHAPP_MAX_SHARED_MCAST_ADDR        (8U)

/* Required size of the MAC address pool (specific to the TI EVM configuration):
 *  1 x MAC address for Ethernet Firmware
 *  2 x MAC address for mpu1_0 virtual switch and MAC-only ports (Linux, 1 for QNX)
 *  2 x MAC address for mcu2_1 virtual switch and MAC-only ports (RTOS)
 *  1 x MAC address for mcu2_1 virtual switch port (AUTOSAR) */
#define ETHAPP_MAC_ADDR_POOL_SIZE               (6U)

/* UDP Iperf task should be highest priority task to ensure processed buffers
 * are freed without delay so that we get maximum throughput for
 * UDP Iperf.
 */
#define UDP_IPERF_THREAD_PRIO  (14U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static struct netif netif_ic[IC_ETH_MAX_VIRTUAL_IF];

/* dhcp struct for the ethernet netif */
static struct dhcp g_netifDhcp[IC_ETH_MAX_VIRTUAL_IF];

static uint32_t netif_ic_state[IC_ETH_MAX_VIRTUAL_IF] =
{
    IC_ETH_IF_R5_0_0_R5_0_1,
    IC_ETH_IF_R5_0_1_R5_0_0,
    IC_ETH_IF_R5_0_0_A53
};

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static void EthApp_createTimer(Ic_Object_Handle hIcObj);

static void EthApp_timerCb(ClockP_Object *hClk, void * arg);

void EthApp_initNetif(void)
{
    ip4_addr_t ipaddr, netmask, gw;
    Ic_Object_Handle hIcObj;
    err_t err;

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    DebugP_log("Starting lwIP, local interface IP is dhcp-enabled \r\n");


    hIcObj = App_doIcOpen(IC_ETH_IF_R5_0_1_R5_0_0);
    DebugP_assert(hIcObj != NULL);

    err = SemaphoreP_constructBinary(&hIcObj->rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == err);
    EthApp_createTimer(hIcObj);

    /* Create inter-core virtual ethernet interface: MCU2_0 <-> MCU2_1 */
    netif_add(&netif_ic[IC_ETH_IF_R5_0_1_R5_0_0], NULL, NULL, NULL,
              (void*)&netif_ic_state[IC_ETH_IF_R5_0_1_R5_0_0],
              LWIPIF_LWIP_IC_init, tcpip_input);

    err = LWIPIF_LWIP_IC_start(&netif_ic[IC_ETH_IF_R5_0_1_R5_0_0], IC_ETH_IF_R5_0_1_R5_0_0, hIcObj);
    DebugP_assert(err == ERR_OK);

    /* Set IC interface as the default */
    netif_set_default(&netif_ic[IC_ETH_IF_R5_0_1_R5_0_0]);

    netif_set_up(netif_default);
    EthApp_setNetifCbs(netif_default);

    sys_lock_tcpip_core();
    dhcp_set_struct(netif_default, &g_netifDhcp[0]);
    err = dhcp_start(netif_default);
    sys_unlock_tcpip_core();
    if (err != ERR_OK)
    {
        DebugP_log("Failed to start DHCP: %d\n", err);
    }
}

void EthApp_waitForNetifUp()
{
    while (!App_isNetworkUp(netif_default))
    {
        DebugP_log("[%d]Waiting for network UP ...\r\n",netif_default->num);
        ClockP_sleep(2);
    }
}

void EthApp_startNetifTask()
{
    sys_lock_tcpip_core();
    lwiperf_example_init();
    sys_thread_new("UDP Iperf", start_application, NULL, DEFAULT_THREAD_STACKSIZE,
                               UDP_IPERF_THREAD_PRIO);
    sys_unlock_tcpip_core();
}

static void EthApp_createTimer(Ic_Object_Handle hIcObj)
{
    ClockP_Params clkPrms;
    int32_t status;

    ClockP_Params_init(&clkPrms);
    clkPrms.start  = false;
    clkPrms.timeout = ClockP_usecToTicks(1000U); // 1ms
    clkPrms.period = ClockP_usecToTicks(1000U); // 1ms
    clkPrms.callback = &EthApp_timerCb;
    clkPrms.args = hIcObj;

    status =  ClockP_construct(&hIcObj->pacingClkObj, &clkPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    ClockP_start(&hIcObj->pacingClkObj);
}

static void EthApp_timerCb(ClockP_Object *hClk, void * arg)
{
#if (IC_ETH_RX_POLLING_MODE)
    Ic_Object_Handle hIcObj = (Ic_Object_Handle) arg;
    if (hIcObj->initComplete)
    {
        SemaphoreP_post(&hIcObj->rxSemObj);
    }
#endif
}
