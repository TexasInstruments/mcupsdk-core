/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
#include <nrt_flow/dataflow.h>
#include "enetapp_cpsw.h"

/* ========================================================================== */
/*                                Function Declarations                       */
/* ========================================================================== */
extern EnetApp_Cfg gEnetAppCfg;
static uint8_t gIpv4TaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static TaskP_Object g_Ipv4RxTaskObj;
static SemaphoreP_Object gIpv4RxSemObj;

void rxDefaultDataCb(void *data, int size, int port, void *cbArg)
{
    if (size > 14) {
        uint16_t ethtype = ntohs(*(uint16_t *)((uint8_t *)data + 12));
        EnetAppUtils_print("%s:pkt ethtype=0x%x, size=%d, port=%d\r\n",
                           __func__, ethtype, size, port);
    } else {
        EnetAppUtils_print("%s:pkt size=%d, port=%d\r\n",
                           __func__, size, port);
    }
}

static void rxNotifyCb(void *cbArg)
{
    SemaphoreP_post(cbArg);
}

static void EthIpv4RxTask(void *args)
{
    cb_rawsock_paras_t llrawp;
    CB_SOCKET_T lldsock;
    ub_macaddr_t bmac;
    uint8_t buf[1518];
    int res;
    CB_SOCKADDR_LL_T addr;

    /* The RX DMA channel is shared with gptp, we need to make sure the
     * gptp has already done its DMA initialization before re-using that channel.
     * Some delay is needed otherwise it will fail to create socket */
    ClockP_usleep(3000000);
    EnetAppUtils_print("%s: RX task for IPv4 running\r\n", __func__);

    (void)memset(&llrawp, 0, sizeof(llrawp));
    llrawp.dev = "tilld0";
    llrawp.proto = ETH_P_IPV4;
    llrawp.vlan_proto = 0;
    llrawp.vlanid = 0;
    llrawp.rw_type = CB_RAWSOCK_RDWR;
    if(cb_rawsock_open(&llrawp, &lldsock, NULL, NULL, bmac) < 0)
    {
        EnetAppUtils_print("%s:failed to open raw socket\r\n", __func__);
        return;
    }
    cb_lld_set_rxnotify_cb(lldsock, rxNotifyCb, &gIpv4RxSemObj);

    EnetAppUtils_print("%s:Started IPv4 task\r\n", __func__);

    while (true)
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gIpv4RxSemObj, SystemP_WAIT_FOREVER);
        res = cb_lld_recv(lldsock, buf, sizeof(buf), &addr, sizeof(CB_SOCKADDR_LL_T));
        if (res <= 0)
        {
            EnetAppUtils_print("%s:failed to recv pkt: res=%d\r\n",
                               __func__, res);
        }
        else
        {
            if (res > 14)
            {
                uint8_t *p = buf;
                uint16_t ethtype = ntohs(*(uint16_t *)(p + 12));
                EnetAppUtils_print("%s:pkt ethtype=0x%x, size=%d, port=%d\r\n",
                                __func__, ethtype, res, addr.macport);
            }
            else
            {
                EnetAppUtils_print("%s:pkt size=%d, port=%d\r\n",
                                   __func__, res, addr.macport);
            }
        }
    }
}

void EnetApp_createRxTask()
{
    TaskP_Params taskParams;
    int32_t status;
    EnetApp_GetMacAddrOutArgs outArgs;

    /* get the source mac address */
    EnetApp_getMacAddress(ENET_DMA_RX_CH0, &outArgs);
    if (outArgs.macAddressCnt > 0)
    {
        EnetUtils_copyMacAddr(gEnetAppCfg.macAddr, outArgs.macAddr[0]);
    }

    status = SemaphoreP_constructBinary(&gIpv4RxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    TaskP_Params_init(&taskParams);
    taskParams.priority       = 2U;
    taskParams.stack          = gIpv4TaskStackRx;
    taskParams.stackSize      = sizeof(gIpv4TaskStackRx);
    taskParams.args           = NULL;
    taskParams.name           = "Rx Ipv4 Task";
    taskParams.taskMain       = &EthIpv4RxTask;

    status = TaskP_construct(&g_Ipv4RxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_destroyRxTask()
{
    SemaphoreP_destruct(&gIpv4RxSemObj);
    TaskP_destruct(&g_Ipv4RxTaskObj);
}
