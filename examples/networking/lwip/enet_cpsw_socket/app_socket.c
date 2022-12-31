/*
 * Copyright (c) 2017 Simon Goldschmidt
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
 * Author: Simon Goldschmidt <goldsimon@gmx.de>
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "app_socket.h"
#include "lwip/opt.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include <string.h>
#include <stdio.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include "enet_apputils.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define SOCK_HOST_SERVER_IP6  ("FE80::12:34FF:FE56:78AB")

#define SOCK_HOST_SERVER_PORT  (8888)

#define APP_SOCKET_MAX_RX_DATA_LEN (1024U)

#define APP_SOCKET_NUM_ITERATIONS (5U)

#define MAX_IPV4_STRING_LEN (16U)

static const uint8_t APP_CLIENT_TX_MSG1[] = "Greetings from Texas Instruments!";

static const uint8_t APP_CLIENT_TX_MSG2[] = "This is a sample message";

#if !LWIP_SOCKET
#error "LWIP_SOCKET is not set! enable socket support in LwIP"
#endif



/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

struct App_hostInfo_t
{
    struct sockaddr_in socketAddr;
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static void Appsocket_fillHostSocketInfo(struct App_hostInfo_t* pHostInfo);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint8_t gRxDataBuff[APP_SOCKET_MAX_RX_DATA_LEN];

static struct App_hostInfo_t gHostInfo;

static char   gHostServerIp4[MAX_IPV4_STRING_LEN] = "";

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


static void Appsocket_fillHostSocketInfo(struct App_hostInfo_t* pHostInfo)
{
    ip_addr_t ipAddr;
    int32_t addr_ok;
    memset(&pHostInfo->socketAddr, 0, sizeof(pHostInfo->socketAddr));

    struct sockaddr_in*  pAddr = &pHostInfo->socketAddr;
    IP_SET_TYPE_VAL(dstaddr, IPADDR_TYPE_V4);
    addr_ok = ip4addr_aton(gHostServerIp4, ip_2_ip4(&ipAddr));
    pAddr->sin_len = sizeof(pHostInfo->socketAddr);
    pAddr->sin_family = AF_INET;
    pAddr->sin_port = PP_HTONS(SOCK_HOST_SERVER_PORT);
    inet_addr_from_ip4addr(&pAddr->sin_addr, ip_2_ip4(&ipAddr));
    EnetAppUtils_assert(addr_ok);

    return;
}

static void AppSocket_simpleClient(void* pArg)
{
    struct sockaddr* pAddr = pArg;
    int32_t sock = -1, ret = 0;
    struct timeval opt = {0}, tv = {0};
    fd_set readset = {0}, writeset = {0}, errset = {0};

    for (uint32_t i = 0; i < APP_SOCKET_NUM_ITERATIONS; i++)
    {
        EnetAppUtils_print("<<< Iteration %d >>>> \r\n", i+1);
        EnetAppUtils_print(" Connecting to: %s:%d \r\n", gHostServerIp4, SOCK_HOST_SERVER_PORT);

        /* create the socket */
        sock = lwip_socket(pAddr->sa_family, SOCK_STREAM, 0);
        if (sock < 0)
        {
            EnetAppUtils_print("ERR: unable to open socket\r\n");
            continue;
        }

        /* connect */
        ret = lwip_connect(sock, pAddr, pAddr->sa_len);
        if (ret != 0)
        {
            ret = lwip_close(sock);
            EnetAppUtils_print("ERR: unable to connect\r\n");
            continue;
        }
        EnetAppUtils_print("Connected to host\r\n");

        /* set recv timeout (100 ms) */
        opt.tv_sec = 0;
        opt.tv_usec = 100 * 1000;
        ret = lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &opt, sizeof(opt));
        if (ret != 0)
        {
            ret = lwip_close(sock);
            EnetAppUtils_print("ERR: set sockopt failed\r\n");
            continue;
        }

        /* Send data to Host */
        ret = lwip_write(sock, APP_CLIENT_TX_MSG1, sizeof(APP_CLIENT_TX_MSG1));
        if (ret != sizeof(APP_CLIENT_TX_MSG1))
        {
            ret = lwip_close(sock);
            EnetAppUtils_print("ERR: socket write failed\r\n");
            continue;
        }
        EnetAppUtils_print("Message to host: %s\r\n", APP_CLIENT_TX_MSG1);

        FD_ZERO(&readset);
        FD_ZERO(&writeset);
        FD_ZERO(&errset);
        FD_SET(sock, &readset);
        FD_SET(sock, &writeset);
        FD_SET(sock, &errset);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        ret = lwip_select(sock + 1, &readset, &writeset, &errset, &tv);
        if (ret <= 0)
        {
            ret = lwip_close(sock);
            EnetAppUtils_print("ERR: socket select failed\r\n");
            continue;
        }
        EnetAppUtils_assert(!FD_ISSET(sock, &errset));

        if (FD_ISSET(sock, &readset))
        {
            ret = lwip_read(sock, gRxDataBuff, APP_SOCKET_MAX_RX_DATA_LEN);
            if (ret <= 0)
            {
                 ret = lwip_close(sock);
                 EnetAppUtils_print("ERR: socket read failed\r\n");
                 continue;
            }
            gRxDataBuff[ret] = '\0';
            EnetAppUtils_print("Message from host: %s\r\n", gRxDataBuff);
        }
        if (FD_ISSET(sock, &writeset))
        {
            ret = lwip_write(sock, APP_CLIENT_TX_MSG2, sizeof(APP_CLIENT_TX_MSG2));
            if (ret != sizeof(APP_CLIENT_TX_MSG2))
            {
                 ret = lwip_close(sock);
                 EnetAppUtils_print("ERR: socket write failed!\r\n");
                 continue;
            }
            EnetAppUtils_assert(ret ==  sizeof(APP_CLIENT_TX_MSG2));
            EnetAppUtils_print("Message to host: %s\r\n", APP_CLIENT_TX_MSG2);
        }
        /* close */
        ret = lwip_close(sock);
        EnetAppUtils_print("Closed Socket connection\r\n");
        ClockP_sleep(2);
    }
    return;
}

void AppSocket_showMenu(void)
{
    ip_addr_t ipAddr;
    int32_t addr_ok = 0;
    EnetAppUtils_print(" UDP socket Menu: \r\n");

    do
    {
        EnetAppUtils_print(" Enter server IPv4 address:(example: 192.168.101.100)\r\n");
        DebugP_scanf("%s", gHostServerIp4);
        addr_ok = ip4addr_aton(gHostServerIp4, ip_2_ip4(&ipAddr));
        TaskP_yield();
    } while (addr_ok != 1);
}

void AppSocket_startClient(void)
{
    AppSocket_showMenu();
    Appsocket_fillHostSocketInfo(&gHostInfo);
    sys_thread_new("AppSocket_simpleClient", AppSocket_simpleClient, &gHostInfo.socketAddr, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}
