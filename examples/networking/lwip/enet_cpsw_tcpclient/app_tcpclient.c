/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <stdio.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "enet_apputils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define HOST_SERVER_IP6  ("FE80::12:34FF:FE56:78AB")

#define HOST_SERVER_PORT  (8888)

#define APP_MAX_RX_DATA_LEN (1024U)

#define APP_NUM_ITERATIONS (1U)

#define MAX_IPV4_STRING_LEN (20U)

static const uint8_t APP_CLIENT_TX_MSG1[] = "Greetings from Texas Instruments!";

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

struct App_hostInfo_t
{
    ip_addr_t ipAddr;
    uint16_t port;
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void AppTcp_fillHostSocketInfo(struct App_hostInfo_t* pHostInfo);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static struct App_hostInfo_t gHostInfo;
static char   gHostServerIp4[MAX_IPV4_STRING_LEN] = "";

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


static void AppTcp_fillHostSocketInfo(struct App_hostInfo_t* pHostInfo)
{
    EnetAppUtils_print(" IP eneterd is: %s \r\n", gHostServerIp4);
    int32_t addr_ok;
    pHostInfo->port = HOST_SERVER_PORT;
    memset(&pHostInfo->ipAddr, 0, sizeof(pHostInfo->ipAddr));
    ip_addr_t*  pAddr = &pHostInfo->ipAddr;
    IP_SET_TYPE_VAL(*pAddr, IPADDR_TYPE_V4);
    addr_ok = ip4addr_aton(gHostServerIp4, ip_2_ip4(pAddr));
    EnetAppUtils_assert(addr_ok);

    return;
}


static void AppTcp_simpleclient(void *pArg)
{
    struct netconn *pConn = NULL;
    err_t err = ERR_OK, connectError = ERR_OK;
    struct App_hostInfo_t* pHostInfo = (struct App_hostInfo_t*) pArg;

    const enum netconn_type connType = NETCONN_TCP;

    struct netbuf *rxBbuf = NULL;
    /* Create a new connection identifier. */
    pConn = netconn_new(connType);

    if (pConn != NULL)
    {
        for (uint32_t pktIdx = 0; pktIdx < APP_NUM_ITERATIONS; pktIdx++)
        {
            /* Connect to the TCP Server */
            EnetAppUtils_print("<<<< ITERATION %d >>>>\r\n", (pktIdx + 1));
            EnetAppUtils_print(" Connecting to: %s:%d \r\n", gHostServerIp4, HOST_SERVER_PORT);
            connectError = netconn_connect(pConn, &pHostInfo->ipAddr, pHostInfo->port);
            if (connectError != ERR_OK)
            {
                netconn_close(pConn);
                DebugP_log("Connection with the server isn't established\r\n");
                continue;
            }

            DebugP_log("Connection with the server is established\r\n");
            // send the data to the server
            err = netconn_write(pConn, APP_CLIENT_TX_MSG1, sizeof(APP_CLIENT_TX_MSG1), NETCONN_COPY);
            if (err == ERR_OK)
            {
                printf("\"%s\" was sent to the Server\r\n", APP_CLIENT_TX_MSG1);
            }
            else
            {
                DebugP_log("couldn't send packet to server\r\n");
                continue;
            }

            /* wait until the data is sent by the server */
            if (netconn_recv(pConn, &rxBbuf) == ERR_OK)
            {
                DebugP_log("Successfully received the packet %d\r\n", pktIdx);
                netbuf_delete(rxBbuf);
            }
            else
            {
                DebugP_log("No response from server\r\n");
            }
            netconn_close(pConn);
            DebugP_log("Connection closed\r\n");
            ClockP_sleep(1);
        }
        netconn_delete(pConn);
    }
}

void AppTcp_showMenu(void)
{
    ip_addr_t ipAddr;
    int32_t addr_ok = 0;
    EnetAppUtils_print(" TCP Client Menu: \r\n");

    do
    {
        EnetAppUtils_print(" Enter TCP server IPv4 address:(example: 192.168.101.100)\r\n");
        DebugP_scanf("%s", gHostServerIp4);
        addr_ok = ip4addr_aton(gHostServerIp4, ip_2_ip4(&ipAddr));
        TaskP_yield();
    } while (addr_ok != 1);
}

void AppTcp_startClient(void)
{
    AppTcp_showMenu();
    AppTcp_fillHostSocketInfo(&gHostInfo);
    sys_thread_new("tcpinit_thread", AppTcp_simpleclient, &gHostInfo, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

  /* USER CODE END 5 */
}
