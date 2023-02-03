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
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include "enet_apputils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define UDP_CONN_PORT_IGMP    2638
#define MCAST_IPV4_ADDR       LWIP_MAKEU32(224, 0, 1, 129)
#define UDP_RECV_BUFSIZE      1024
#define UTILS_ALIGN(x,align)  ((((x) + ((align) - 1))/(align)) * (align))

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

static void AppUdp_ServerTask(void *arg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static struct App_hostInfo_t gHostInfo;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void Appsocket_fillSocketInfo(struct App_hostInfo_t* pHostInfo)
{
    memset(&pHostInfo->socketAddr, 0, sizeof(pHostInfo->socketAddr));

    struct sockaddr_in*  pAddr = &pHostInfo->socketAddr;
    pAddr->sin_family = AF_INET;
    pAddr->sin_port = htons(UDP_CONN_PORT_IGMP);
    pAddr->sin_addr.s_addr = htonl(INADDR_ANY);

    return;
}

static void AppUdp_ServerTask(void *pArg)
{
    err_t err;
    int sock;
    struct sockaddr* pAddr = pArg;
    const ip4_addr_t multi_addr = { .addr = PP_HTONL(MCAST_IPV4_ADDR) };
    char recv_buf[UTILS_ALIGN(UDP_RECV_BUFSIZE, 32)] __attribute__ ((aligned(32)));
    struct sockaddr_in from = { 0 };
    socklen_t fromlen = sizeof(from);
    int32_t count;
    struct ip_mreq mcast_group;

    DebugP_log("UDP server: Port %d\r\r\n",UDP_CONN_PORT_IGMP);
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        DebugP_log("UDP server: Error creating Socket\r\r\n");
        return;
    }

    err = bind(sock, pAddr, sizeof(*pAddr));
    if (err != ERR_OK)
    {
        DebugP_log("UDP server: Error on bind: %d\r\r\n", err);
        close(sock);
        return;
    }
    mcast_group.imr_multiaddr.s_addr = multi_addr.addr;
    mcast_group.imr_interface.s_addr = htonl(INADDR_ANY);

    /* Join the multicast group using setsockopt which calls igmp_joingroup */
    err = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mcast_group, sizeof(mcast_group));
    if (err != ERR_OK)
    {
        DebugP_log("UDP server: Error joining Multicast group %d\r\n", err);
        close(sock);
        return;
    }
    else
    {
        DebugP_log("UDP server: joined Multicast group \r\n");
    }

    while (true)
    {
        count = lwip_recvfrom(sock, recv_buf, UDP_RECV_BUFSIZE, 0, (struct sockaddr*) &from, &fromlen);
        if (count <= 0)
            continue;

        DebugP_log("Packet recieved \r\n\r");
        count = (count >= UDP_RECV_BUFSIZE) ? UDP_RECV_BUFSIZE - 1 : count;
        recv_buf[count] = '\0';
        CacheP_wbInv(recv_buf, sizeof(recv_buf), CacheP_TYPE_ALLD);
        DebugP_log("Client: %s\r\n", recv_buf);
        if (sendto(sock, recv_buf, count, 0, (struct sockaddr*) &from, fromlen) < 0)
        {
            DebugP_log("Error in write\r\n\r");
        }
        else
        {
            DebugP_log("Echo pkt completed\r\n\r");
        }
        ClockP_sleep(2);
    }
}

void AppUdp_startServer()
{
    Appsocket_fillSocketInfo(&gHostInfo);
    sys_thread_new("AppUdp_ServerTask", AppUdp_ServerTask, &gHostInfo.socketAddr, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/
