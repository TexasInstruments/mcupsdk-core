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
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_board.h>
#include <enet.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/QueueP.h>
#include <include/per/icssg.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "ti_drivers_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define APP_SERVER_PORT  (8888)

static const uint8_t APP_CLIENT_TX_MSG1[] = "Greetings from Texas Instruments!";

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void AppTcp_echoPckt(struct netconn *pClientConn);

static void AppTcp_ServerTask(void *arg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void AppTcp_echoPckt(struct netconn *pClientConn)
{
    struct netbuf *buf;
    void *data;
    u16_t len;
    err_t err;

    while ((err = netconn_recv(pClientConn, &buf)) == ERR_OK)
    {
        err = netconn_write(pClientConn, APP_CLIENT_TX_MSG1, sizeof(APP_CLIENT_TX_MSG1), NETCONN_COPY);
        if (err != ERR_OK)
        {
            printf("tcpecho: netconn_write: error \"%s\"\r\n", lwip_strerr(err));
            break;
        }
        do
        {
            netbuf_data(buf, &data, &len);
            err = netconn_write(pClientConn, data, len, NETCONN_COPY);
            if (err != ERR_OK)
            {
                printf("tcpecho: netconn_write: error \"%s\"\r\n", lwip_strerr(err));
            }

        } while (netbuf_next(buf) >= 0);
        netbuf_delete(buf);
    }
}

static void AppTcp_ServerTask(void *arg)
{
    struct netconn *pConn = NULL, *pClientConn = NULL;
    err_t err;
    LWIP_UNUSED_ARG(arg);

    pConn = netconn_new(NETCONN_TCP);
    netconn_bind(pConn, IP_ADDR_ANY, APP_SERVER_PORT);
    LWIP_ERROR("tcpecho: invalid conn\r\n", (pConn != NULL), return;);

    /* Tell connection to go into listening mode. */
    netconn_listen(pConn);

    while (1)
    {

        /* Grab new connection. */
        err = netconn_accept(pConn, &pClientConn);
        printf("accepted new connection %p\r\n", pClientConn);

        /* Process the new connection. */
        if (err < ERR_OK)
        {
            DebugP_log("Unable to accept connection: errno %d\r\n", err);
            break;
        }

        AppTcp_echoPckt(pClientConn);

        /* Close connection and discard connection identifier. */
        netconn_close(pClientConn);
        netconn_delete(pClientConn);
    }
}

void AppTcp_startServer()
{
    sys_thread_new("AppTcp_ServerTask", AppTcp_ServerTask, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/
