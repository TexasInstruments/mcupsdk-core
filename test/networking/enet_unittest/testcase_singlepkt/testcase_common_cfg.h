/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 * \file  testcase_common_cfg.h
 *
 * \brief This file contains the common testcase configuration.
 */

#ifndef _TESTCASE_COMMON_CONFIG_H_
#define _TESTCASE_COMMON_CONFIG_H_

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

#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>

/* SDK includes */
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PROFILE_TIME_MAX__TIME_ENTRIES                       (100U)
#define PROFILE_TIME_MAX_COLUMN_ENTRIES                      (2U)

/* Task stack size */
#define ENETLPBK_TASK_STACK_SZ                     (10U * 1024U)

/* Loopback test iteration count */
#define ENETLPBK_NUM_ITERATION                     (1U)

#define ENETLPBK_TEST_PKT_NUM                      (600U)

#define ENETLPBK_TEST_PKT_LEN                      (404U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef enum EnetLpbk_type_e
{
    /* MAC loopback */
    LOOPBACK_TYPE_MAC = 0,
    /* PHY loopback (internal) */
    LOOPBACK_TYPE_PHY = 1
} EnetLpbk_type;

typedef enum TestLpbk_result_e
{
    /* Success */
    LOOPBACK_TEST_PASS = 0,
    /* Failure */
    LOOPBACK_TEST_FAIL = 1
} TestLpbk_result;

typedef struct TestApp_Obj_s
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
    EnetDma_RxChHandle hRxCh;
    EnetDma_PktQ rxFreeQ;
    EnetDma_PktQ rxReadyQ;
    EnetDma_Pkt* rxReadyPkt;
    EnetDma_Pkt* txFreePkt;
    EnetDma_TxChHandle hTxCh;
    EnetDma_PktQ txFreePktInfoQ;
    uint8_t hostMacAddr[ENET_MAC_ADDR_LEN];

    /* Test config params */
    EnetLpbk_type testLoopBackType;
    bool printFrame;        /* Print received Ethernet frames? */

    /* Test runtime params */
    volatile bool exitFlag;
    volatile bool exitFlagDone;

    /* Packet transmission */
    TaskP_Object txTaskObj;
    SemaphoreP_Object txSemObj;
    SemaphoreP_Object txDoneSemObj;
    uint32_t totalTxCnt;
    uint32_t totalTxTaskCnt;
    uint32_t totalTxTimerCBCnt;

    /* Packet reception */
    TaskP_Object rxTaskObj;
    SemaphoreP_Object rxSemObj;
    SemaphoreP_Object rxDoneSemObj;
    uint32_t totalRxCnt;

    ClockP_Object timerObj;
    /* Semaphore posted by tick timer to run tick task */
    SemaphoreP_Object timerSemObj;
    TestLpbk_result testResult;
} TestApp_Obj;

typedef struct TestCfg_Obj_s
{
    uint32_t rxNumPkt;
    uint32_t txNumPkt;
    uint32_t txScatterSeg;
    uint32_t rxScatterSeg;
} TestCfg_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

bool EnetLpbk_verifyRxFrame(EnetDma_Pkt *pktInfo, uint8_t rxCnt);

int32_t TestApp_waitForLinkUp(void);

void TestApp_showCpswStats(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test App object declaration */
extern TestApp_Obj gTestApp;

/* Test App Configuration Object */
extern TestCfg_Obj gTestCfg;

#ifdef __cplusplus
}
#endif

#endif /* _TESTCASE_COMMON_CONFIG_H_ */
