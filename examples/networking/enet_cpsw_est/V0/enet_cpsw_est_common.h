/*
 * Copyright (C) 2022 Texas Instruments Incorporated
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

#ifndef _TIMESYNC_COMMON_H_
#define _TIMESYNC_COMMON_H_

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
#include <drivers/sciclient.h>
#include <drivers/udma/udma_priv.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>
#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_appmemutils.h>
#include <networking/enet/utils/include/enet_appmemutils_cfg.h>
/* SDK includes */
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void Board_cpswMuxSel(void);
extern void Board_TxRxDelaySet(const EnetBoard_PhyCfg *boardPhyCfg);

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Max number of ports that can be tested with this app */
#define ENETAPP_PORT_MAX                          (2U)

/* Task stack size */
#define ENETAPP_TASK_STACK_SZ                     (10U * 1024U)

/* 100-ms periodic tick */
#define ENETAPP_PERIODIC_TICK_MS                  (100U)

/* Number of test packets to send */
#define ENETAPP_TEST_TX_PKT_CNT                   (16U)

/* Counting Semaphore count*/
#define COUNTING_SEM_COUNT                        (10U)

/* Value of seconds in nanoseconds. Useful for calculations */
#define TIME_SEC_TO_NS                            (1000000000U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* Port's test parameters */
typedef struct EnetApp_PortTestParams_s
{
    /* MAC port number */
    Enet_MacPort macPort;

    /* EST/TAS control list */
    EnetTas_ControlList tasControlList;
} EnetApp_PortTestParams;

/* Test parameters for the EST app */
typedef struct EnetApp_TestParams_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* MAC ports and their corresponding gate command lists */
    EnetApp_PortTestParams portTestParams[ENETAPP_PORT_MAX];

    /* Number of MAC ports in macPorts array */
    uint32_t macPortNum;
} EnetApp_TestParams;

/* EST interval represented as start/end pair */
typedef struct EnetApp_EstIntvl_s
{
    /* Interval start time in nsecs */
    uint64_t start;

    /* Interval end time in nsecs */
    uint64_t end;

    /* Gate mask */
    uint8_t gateMask;
} EnetApp_EstIntvl;

/* EST intervals */
typedef struct EnetApp_EstIntvls_s
{
    /* Intervals: start, end, dur, gateMask */
    EnetApp_EstIntvl intvl[ENET_TAS_MAX_CMD_LISTS];

    /* Cycle time in nsecs */
    uint64_t cycleTime;

    /* Number of used intervals */
    uint32_t numIntvls;
} EnetApp_EstIntvls;

/* Example app object */
typedef struct EnetApp_Obj_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* MAC ports being used */
    Enet_MacPort macPort[ENETAPP_PORT_MAX];

    /* EST/TAS control list */
    EnetTas_ControlList tasControlList[ENETAPP_PORT_MAX];

    /* Whether test app's default EST schedule is still being used */
    bool usingDfltSched;

    /* Number of MAC ports being used */
    uint32_t macPortNum;

    /* This core's id */
    uint32_t coreId;

    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreKey;

    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;

    /* CPSW configuration */
    Cpsw_Cfg cpswCfg;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* TX channel number */
    uint32_t txChNum;

    /* Start flow index */
    uint32_t rxStartFlowIdx;

    /* Regular traffic RX flow index */
    uint32_t rxFlowIdx;

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;

    /* RX channel handle */
    EnetDma_RxChHandle hRxCh;

    /* RX task handle */
    TaskP_Object rxTaskObj;

   /* Semaphore posted from RX callback when packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Flag which indicates if test shall run */
    volatile bool run;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Periodic tick timer - used only to post a semaphore */
    ClockP_Object tickTimerObj;

    /* Periodic tick task - Enet period tick needs to be called from task context
     * hence it's called from this task (i.e. as opposed to in timer callback) */
    TaskP_Object tickTaskObj;

    /* Semaphore posted by tick timer to run tick task */
    SemaphoreP_Object timerSemObj;

    /* Whether timestamp printing is enabled or not */
    bool enableTs;

    /* ESTF start time in nsecs */
    uint64_t estfStartTime[ENETAPP_PORT_MAX];

    /* Saved intervals of the current oper list */
    EnetApp_EstIntvls estIntvls[ENETAPP_PORT_MAX];
    
    /* Main UDMA driver handle */
    Udma_DrvHandle hMainUdmaDrv;

} EnetApp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetApp_mainTask(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet timesync test object */
EnetApp_Obj gEnetApp;

/* Statistics */
CpswStats_PortStats gEnetApp_cpswStats;

/* Test application stack */
static uint8_t gEnetAppTaskStackTick[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppTaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

#ifdef __cplusplus
}
#endif

#endif /* _TIMESYNC_COMMON_H_ */
