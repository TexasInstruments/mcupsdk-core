/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  txsg_common.h
 *
 * \brief This is the common header file of txsg application.
 */

#ifndef _TXSG_COMMON_H_
#define _TXSG_COMMON_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>

#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_appmemutils.h>
#include <networking/enet/utils/include/enet_appmemutils_cfg.h>


#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"


#ifdef __cplusplus
extern "C" {
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Task stack size */
#define ENETTXSG_TASK_STACK_SZ                     (10U * 1024U)

#define ENETTXSG_NUM_ITERATION                     (5U)

#define ENETTXSG_TEST_PKT_NUM                      (100U * 85U * 1000U)

#define ENETTXSG_TEST_PKT_LEN                      (1500U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef enum EnetTxSG_type_e
{
    /* PHY txsg (internal) */
    TXSG_LOOPBACK_TYPE_PHY  = 0,
    /* No loopback. trasmit packets to network */
    TXSG_LOOPBACK_TYPE_NONE = 1
} EnetTxSG_type;

typedef struct EnetTxSG_Obj_s
{
    /* Enet driver */
    Enet_Handle hEnet;
    Enet_Type enetType;
    uint32_t instId;
    uint32_t coreId;
    uint32_t coreKey;
    uint32_t boardId;
    Enet_MacPort macPort;
    emac_mode macMode;      /* MAC mode (defined in board library) */

    uint32_t rxChNum;
    EnetDma_RxChHandle hRxCh;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ rxReadyQ;
    EnetDma_TxChHandle hTxCh;
    EnetDma_PktQ txFreePktInfoQ;
    uint32_t txChNum;
    uint8_t hostMacAddr[ENET_MAC_ADDR_LEN];

    /* Test config params */
    EnetTxSG_type testLoopBackType;
    bool printFrame;        /* Print received Ethernet frames? */

    /* Test runtime params */
    volatile bool exitFlag;
    volatile bool exitFlagDone;

    /* Packet transmission */
    TaskP_Object txTaskObj;
    SemaphoreP_Object txSemObj;
    SemaphoreP_Object txDoneSemObj;
    uint32_t totalTxCnt;

    /* Packet reception */
    TaskP_Object rxTaskObj;
    SemaphoreP_Object rxSemObj;
    SemaphoreP_Object rxDoneSemObj;
    uint32_t totalRxCnt;
    uint32_t totalRxBytesCnt;
    uint32_t totalTxBytesCnt;
    uint32_t rxIsrCount;
    uint32_t txIsrCount;
    TaskP_Object cpuLoadTask;
} EnetTxSG_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t EnetTxSG_txsgTest(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet txsg test object declaration */
extern EnetTxSG_Obj gEnetTxSG;

#ifdef __cplusplus
}
#endif

#endif /* _TXSG_COMMON_H_ */
