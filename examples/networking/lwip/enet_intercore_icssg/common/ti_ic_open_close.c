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
 * \file  ti_ic_open_close.c
 *
 * \brief This file contains the implementation of the Intercore driver initialization.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "FreeRTOS.h"
#include "task.h"
/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include <examples/lwiperf/lwiperf_example.h>

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>

#include "ti_ic_open_close.h"
#include "custom_pbuf_ic.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define IC_MAX_QUEUE_LEN (64U)

/* These must be sufficient for total number of rx pbufs and tx packets */
pbufIcNode gFreeIcPbufArr[IC_MAX_QUEUE_LEN * SHDMEM_CIRCULAR_BUFFER_MAX_QUEUES];

LWIP_MEMPOOL_DECLARE(IC_CUSTOM_POOL, IC_MAX_QUEUE_LEN, sizeof(Ic_CustomPbuf), "Ic Custom Pbuf pool");

uint8_t gBufferPool[IC_MAX_QUEUE_LEN][PBUF_POOL_BUFSIZE];

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Ic_Object gIcObjPool[IC_ETH_MAX_VIRTUAL_IF];

/*! Maximum Number of buffers per queue: 512 + padding space. Extending buffers need MPU memory alloc changes */
const LwipIc_QueueTbl gLwipIcQueueTbl[SHDMEM_CIRCULAR_BUFFER_MAX_QUEUES]=
{
    {
        .pShdMemBuffStartAdd  = (void *)0xA0400000U,
        .elemCount            = 16U,
        .elemSize             = SHDMEM_CIRCULAR_BUFFER_MAX_ELEM_SIZE,
    },
    {
        .pShdMemBuffStartAdd  = (void *)0xA0500000U,
        .elemCount            = 16U,
        .elemSize             = SHDMEM_CIRCULAR_BUFFER_MAX_ELEM_SIZE,
    },
    {
        .pShdMemBuffStartAdd  = (void *)0xA0600000U,
        .elemCount            = 16U,
        .elemSize             = SHDMEM_CIRCULAR_BUFFER_MAX_ELEM_SIZE,
    },
    {
        .pShdMemBuffStartAdd  = (void *)0xA0700000U,
        .elemCount            = 16U,
        .elemSize             = SHDMEM_CIRCULAR_BUFFER_MAX_ELEM_SIZE,
    },
};

LwipIc_Params gLwipIcParams[IC_ETH_MAX_VIRTUAL_IF]=
{
    {
        .instId             = IC_ETH_IF_R5_0_0_R5_0_1,
        .ownerId            = IPC_MCU1_0,
        .txQId              = ICQ_R5_0_0_TO_R5_0_1,
        .rxQId              = ICQ_R5_0_1_TO_R5_0_0,
        .reqEndPtId         = ICETH_IPC_ENDPT_R5_0_0,
        .remoteCoreId       = IPC_MCU1_1,
        .endPtName          = "ENDPT_ICETH_MCU2_0_R5",
        .remoteEndPtName    = "ENDPT_ICETH_MCU2_1",
        .macAddr            = {0x00,0x01,0x02,0x03,0x04,0x05},
    },
    {
        .instId             = IC_ETH_IF_R5_0_1_R5_0_0,
        .ownerId            = IPC_MCU1_0,
        .txQId              = ICQ_R5_0_1_TO_R5_0_0,
        .rxQId              = ICQ_R5_0_0_TO_R5_0_1,
        .reqEndPtId         = ICETH_IPC_ENDPT_R5_0_1,
        .remoteCoreId       = IPC_MCU1_0,
        .endPtName          = "ENDPT_ICETH_MCU2_1",
        .remoteEndPtName    = "ENDPT_ICETH_MCU2_0_R5",
        .macAddr            = {0x00,0x01,0x02,0x04,0x05,0x06},
    },
    {
        .instId             = IC_ETH_IF_R5_0_0_A53,
        .ownerId            = IPC_MCU1_0,
        .txQId              = ICQ_R5_0_0_TO_A53,
        .rxQId              = ICQ_A53_TO_R5_0_0,
        .reqEndPtId         = ICETH_IPC_ENDPT_R5_0_0_A53,
        .remoteCoreId       = IPC_MPU1_0,
        .endPtName          = "ENDPT_ICETH_MCU2_0_A72",
        .remoteEndPtName    = "ENDPT_ICETH_A72",
        .macAddr            = {0x00,0x01,0x02,0x05,0x06,0x07},
    },
};

ShdMemCircularBufferP_Rsv gShmQueueHandleTbl[SHDMEM_CIRCULAR_BUFFER_MAX_QUEUES];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

Ic_Object_Handle App_acquireIcHandle(uint32_t instId)
{
    Ic_Object_Handle hIcObj = NULL;

    if((instId < IC_ETH_MAX_VIRTUAL_IF) && (gIcObjPool[instId].initComplete == FALSE))
    {
        hIcObj = &gIcObjPool[instId];
    }
    return hIcObj;
}

Ic_Object_Handle App_getIcHandle(uint32_t instId)
{
    Ic_Object_Handle hIcObj = NULL;
    bool isOpen = false;

    if(instId < IC_ETH_MAX_VIRTUAL_IF)
    {
        hIcObj = &gIcObjPool[instId];
        isOpen = (hIcObj->initComplete == TRUE);
    }

    return isOpen ? hIcObj : NULL;
}

Ic_Object_Handle App_doIcOpen(uint32_t instId)
{
    Ic_Object_Handle hIcObj = NULL;
    int32_t status;
    Ic_CustomPbuf *cPbuf;
    uint32_t numCustomPbuf = 0U;

    hIcObj = App_acquireIcHandle(instId);
    LwipIc_assert(hIcObj != NULL);

    hIcObj->selfCoreId = App_getSelfCoreId();

    /* Only MCU1_0 and MCU1_1 supported for now */
    LwipIc_assert((hIcObj->selfCoreId == IPC_MCU1_0) ||
                  (hIcObj->selfCoreId == IPC_MCU1_1) );

    hIcObj->ownerId      = gLwipIcParams[instId].ownerId;
    hIcObj->remoteCoreId = gLwipIcParams[instId].remoteCoreId;
    hIcObj->txQId        = gLwipIcParams[instId].txQId;
    hIcObj->rxQId        = gLwipIcParams[instId].rxQId;
    hIcObj->myReqEndPtId = gLwipIcParams[instId].reqEndPtId;

    memset(hIcObj->myEndPtName, '\0', sizeof(hIcObj->myEndPtName));
    memset(hIcObj->remoteEndPtName, '\0', sizeof(hIcObj->remoteEndPtName));
    strcpy(hIcObj->myEndPtName, gLwipIcParams[instId].endPtName);
    strcpy(hIcObj->remoteEndPtName, gLwipIcParams[instId].remoteEndPtName);

    hIcObj->shmTxQ = ShdMemCircularBufferP_create(&gShmQueueHandleTbl[hIcObj->txQId],
                                                  gLwipIcQueueTbl[hIcObj->txQId].pShdMemBuffStartAdd,
                                                  gLwipIcQueueTbl[hIcObj->txQId].elemCount,
                                                  gLwipIcQueueTbl[hIcObj->txQId].elemSize);
    hIcObj->shmRxQ = ShdMemCircularBufferP_create(&gShmQueueHandleTbl[hIcObj->rxQId],
                                                  gLwipIcQueueTbl[hIcObj->rxQId].pShdMemBuffStartAdd,
                                                  gLwipIcQueueTbl[hIcObj->rxQId].elemCount,
                                                  gLwipIcQueueTbl[hIcObj->rxQId].elemSize);
    LwipIc_assert(hIcObj->shmTxQ != NULL);
    LwipIc_assert(hIcObj->shmRxQ != NULL);
    numCustomPbuf = gLwipIcQueueTbl[hIcObj->txQId].elemCount + gLwipIcQueueTbl[hIcObj->rxQId].elemCount;

#if (!IC_ETH_RX_POLLING_MODE)
    /* Initialize IPC */
    status = LwipIc_ipcInit(hLwipIc);
    LwipIc_assert(status == LWIPIC_OK);
    hIcObj->pktNotifyThresh = IC_PKT_NOTIFY_THRESHOLD;
#endif


    pbufQ_ic_init(&hIcObj->freePbufQ);
    pbufQ_ic_init_freeQ(gFreeIcPbufArr, numCustomPbuf);
    LWIP_MEMPOOL_INIT(IC_CUSTOM_POOL);
    for (uint32_t i = 0U; i < numCustomPbuf; i++)
    {
        /* Allocate the Custom Pbuf structures and put them in freePbufInfoQ */
        cPbuf = NULL;
        cPbuf = (Ic_CustomPbuf*)LWIP_MEMPOOL_ALLOC(IC_CUSTOM_POOL);
        LwipIc_assert(cPbuf != NULL);
        cPbuf->p.custom_free_function = custom_pbuf_ic_free;
        cPbuf->customPbufArgs         = (Ic_CustomPbuf_Args)(hIcObj);
        cPbuf->orgBufLen              = 0U;
        cPbuf->orgBufPtr              = &gBufferPool[i][0];
        cPbuf->p.pbuf.payload         = &gBufferPool[i][0];
        cPbuf->p.pbuf.next            = NULL;
        cPbuf->p.pbuf.flags          |= PBUF_FLAG_IS_CUSTOM;
        pbufQ_ic_enQ(&hIcObj->freePbufQ, (struct pbuf*)(cPbuf));
    }

    /* If we are the owner of this interface then initialize shared memory
     * transport otherwise wait here until it is initialized by the owner
     */
    if(hIcObj->ownerId == hIcObj->selfCoreId)
    {
        /* Initialize our TX i.e. peer's RX queue */
        status = ShdMemCircularBufferP_initQ(hIcObj->shmTxQ, hIcObj->txQId, gLwipIcQueueTbl[hIcObj->txQId].elemCount);
        LwipIc_assert(status == LWIPIC_OK);

        /* Initialize our RX i.e. peer's TX queue */
        status = ShdMemCircularBufferP_initQ(hIcObj->shmRxQ, hIcObj->rxQId, gLwipIcQueueTbl[hIcObj->rxQId].elemCount);
        LwipIc_assert(status == LWIPIC_OK);
    }
    else
    {
        /* Wait for the shared memory transport to be initialized
         * NOTE: This is called by LWIPIF_LWIP_IC_init which is called
         * by netif_add so netif_add will be blocked untill transport
         * initialization is complete
         */
        while(!ShdMemCircularBufferP_isQValid(hIcObj->shmTxQ) ||
                !ShdMemCircularBufferP_isQValid(hIcObj->shmRxQ) )
        {
            ClockP_usleep(100);
        }
    }
    hIcObj->initComplete = TRUE;

    return hIcObj;
}

int32_t App_getSharedMemInfo(Icve_respMsg *respMsg, uint32_t remoteCoreId)
{
    uint32_t txIdx, rxIdx;
    int32_t status = ICVE_OK;

    /*! Clean up the structure with proper params.
     * ToDo: Get the info based on remote core Id
     */

    if(status == ICVE_OK)
    {
        txIdx = gLwipIcParams[0].txQId;
        rxIdx = gLwipIcParams[0].rxQId;
        /*! tx Queue for main core is rx queue for remote core */
        respMsg->shm_info.tx_shm_info.total_shm_size  = 0x00400000U;
        respMsg->shm_info.tx_shm_info.num_pkt_bufs    = gLwipIcQueueTbl[rxIdx].elemCount;
        respMsg->shm_info.tx_shm_info.buff_slot_size  = (gLwipIcQueueTbl[rxIdx].elemSize + 8U);
        respMsg->shm_info.tx_shm_info.base_addr       = (uint32_t)gLwipIcQueueTbl[rxIdx].pShdMemBuffStartAdd;

        respMsg->shm_info.rx_shm_info.total_shm_size  = 0x00400000U;
        respMsg->shm_info.rx_shm_info.num_pkt_bufs    = gLwipIcQueueTbl[txIdx].elemCount;
        respMsg->shm_info.rx_shm_info.buff_slot_size  = (gLwipIcQueueTbl[txIdx].elemSize + 8U);
        respMsg->shm_info.rx_shm_info.base_addr       = (uint32_t)gLwipIcQueueTbl[txIdx].pShdMemBuffStartAdd;
    }
    return status;
}
