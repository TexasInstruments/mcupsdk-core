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
#ifndef _ENETAPP_H_
#define _ENETAPP_H_

#include "ti_enet_config.h"
/* ========================================================================== */
/*                          Structure Declarations                            */
/* ========================================================================== */

#define ENETAPP_PORT_MAX                          (ENET_SYSCFG_NUM_EXT_MAC_PORTS)

#define ENETAPP_ICSSG_INSTANCE_MAX                (2U)
#define ENETAPP_PER_MAX                           (ENETAPP_ICSSG_INSTANCE_MAX)

/* Context of a peripheral/port */
typedef struct EnetApp_PerCtxt_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;

    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreKey;

    /* Index of this peripheral object */
    uint32_t perIdx;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort[ENETAPP_PORT_MAX];

    /* Number of MAC ports in macPorts array */
    uint8_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;

    /* Number of valid MAC address entries present in macAddr variable below*/
    uint8_t numValidMacAddress;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* non-PTP Tx channel Ids */
    uint32_t nonPtpTxChId[ENET_SYSCFG_TX_CHANNELS_NUM];

    /* non-PTP Tx channel array count */
    uint8_t nonPtpTxChNum;

    /* non-PTP Rx flow Ids */
    uint32_t nonPtpRxFlowId[ENET_SYSCFG_RX_FLOWS_NUM];

    /* non-PTP Rx flow array count */
    uint8_t nonPtpRxFlowNum;

    /* TX channel number */
    uint32_t txChNum[ENET_SYSCFG_TX_CHANNELS_NUM];

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh[ENET_SYSCFG_TX_CHANNELS_NUM];

    /* Start flow index */
    uint32_t rxStartFlowIdx[ENET_SYSCFG_RX_FLOWS_NUM];

    /* Flow index */
    uint32_t rxFlowIdx[ENET_SYSCFG_RX_FLOWS_NUM];

    /* RX channel handle */
    EnetDma_RxChHandle hRxCh[ENET_SYSCFG_RX_FLOWS_NUM];

    /* RX task handle - receives packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* Semaphore posted from RX callback when packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Semaphore posted from event callback upon asynchronous IOCTL completion */
    SemaphoreP_Object ayncIoctlSemObj;

    /* Semaphore posted from event callback upon asynchronous IOCTL completion */
    SemaphoreP_Object txTsSemObj;
} EnetApp_PerCtxt;

typedef struct EnetApp_Obj_s
{
    /* Flag which indicates if test shall run */
    volatile bool run;

    /* This core's id */
    uint32_t coreId;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Array of all peripheral/port contexts used in the test */
    EnetApp_PerCtxt perCtxt[ENETAPP_PER_MAX];

    /* Number of active contexts being used */
    uint32_t numPerCtxts;

    /* Ports on which PTP stack runs */
    Enet_MacPort ptpMacPorts[ENETAPP_PORT_MAX];

    /* Number of ports in PTP port array */
    uint8_t numPtpPorts;

    /* Index of gPTP perCtxt */
    uint8_t gptpPerIdx;
} EnetApp_Cfg;

void EnetApp_destroyRxTask(EnetApp_PerCtxt *perCtxt);
void EnetApp_createRxTask(EnetApp_PerCtxt *perCtxt);

#endif //_ENETAPP_H_
