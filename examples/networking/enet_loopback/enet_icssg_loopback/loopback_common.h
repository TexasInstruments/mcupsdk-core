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
 * \file  loopback_common.h
 *
 * \brief This is the common header file of loopback application.
 */

#ifndef _LOOPBACK_COMMON_H_
#define _LOOPBACK_COMMON_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/EventP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>
#include <drivers/sciclient.h>
#include <drivers/udma/udma_priv.h>

#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>

/* SDK includes */
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Helper macro used to create loopback port menu options */
#define ENETLPBK_PORT_OPT(macPort, macMode, boardId) { #macPort " - "  #macMode, (macPort), (macMode), (boardId) }


/* Loopback test iteration count */
#define ENETLPBK_NUM_ITERATION                     (1U)

#define ENETLPBK_TEST_PKT_NUM                      (1000U) /* Total number of packets transmitted in an iteration */

#define ENETLPBK_TEST_PKT_LEN                      (500U)  /* Transmit packet length */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


typedef enum EnetLpbk_type_e
{
    /* MAC loopback - not supported for ICSSG */
    LOOPBACK_TYPE_MAC = 0,
    /* PHY loopback (internal) */
    LOOPBACK_TYPE_PHY = 1
} EnetLpbk_type;

typedef enum AppEventId_e
{
    AppEventId_NONE = 0,
    AppEventId_RXPKT = (1 << 0),
    AppEventId_TXPKT = (1 << 1),
    AppEventId_PERIODIC_POLL = (1 << 2),
    AppEventId_TERMINATE = (1 << 3),
    AppEventId_ANY_EVENT = (AppEventId_RXPKT |
                             AppEventId_TXPKT |
                             AppEventId_PERIODIC_POLL |
                             AppEventId_TERMINATE),
} AppEventId_t;

typedef struct EnetLpbk_Obj_s
{
    /* Enet driver */
    Enet_Handle hEnet;
    Enet_Type enetType;
    uint32_t instId;
    uint32_t coreId;
    uint32_t coreKey;
    uint8_t numMacPorts;
    Enet_MacPort macPortList[ENET_SYSCFG_NUM_EXT_MAC_PORTS];
    Udma_DrvHandle hUdmaDrv;
    uint32_t rxFlowIdx;
    uint32_t rxStartFlowIdx;
    EnetDma_RxChHandle hRxCh[ENET_SYSCFG_RX_FLOWS_NUM];
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ rxReadyQ;
    EnetDma_TxChHandle hTxCh;
    EnetDma_PktQ txFreePktInfoQ;
    uint32_t txChNum;
    uint8_t hostMacAddr[ENET_MAC_ADDR_LEN];

    /* Test config params */
    bool printFrame;        /* Print received Ethernet frames? */

    /* Test runtime params */
    volatile bool exitFlag;
    volatile bool exitFlagDone;

    /* Packet transmission */
    uint32_t totalTxCnt;

    /* Packet reception */
    uint32_t totalRxCnt;

    EventP_Object appEvents;

} EnetLpbk_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t EnetApp_loopbackTest(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet loopback test object declaration */
extern EnetLpbk_Obj gEnetLpbk;

#ifdef __cplusplus
}
#endif

#endif /* _LOOPBACK_COMMON_H_ */
