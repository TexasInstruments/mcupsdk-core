/*
 * Copyright (c) Texas Instruments Incorporated 2022
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

#ifndef _MULTI_CHANNEL_COMMON_H_
#define _MULTI_CHANNEL_COMMON_H_

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
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "timeSync_ptp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Max number of ports that can be tested with this app */
#define ENETAPP_PER_MAX                           (1U)

/* Offset for MSG ID in a PTP packet */
#define PTP_MSG_ID_OFFSET                        (14U)

/* Offset for SEQ ID in a PTP packet */
#define PTP_SEQ_ID_OFFSET                        (44U)

/* EtherType for PTP packet to set a classfier */
#define PTP_ETHERTYPE                            (0x88F7U)

/* Task stack size */
#define ENETAPP_TASK_STACK_SZ                     (10U * 1024U)

/* 100-ms periodic tick */
#define ENETAPP_PERIODIC_TICK_MS                  (100U)

/*Counting Semaphore count*/
#define COUNTING_SEM_COUNT                       (10U)

/** Value of seconds in nanoseconds. Useful for calculations*/
#define TIME_SEC_TO_NS                           (1000000000U)


#define TIMESYNC_TX_PKTINFO_COUNT           (64U)

/**
 * @def TIMESYNC_ANNEX_F_HEADER_SIZE
 *      Number of bytes in Annex F(bare Ethernet) header
 */
#define TIMESYNC_ANNEX_F_HEADER_SIZE        (14U)

/**
 * @def TIMESYNC_ANNEX_D_ANNEX_F_DIFF
 *      Number of extra bytes for Annex D(IPv4) from Annex F(802.3)
 */
#define TIMESYNC_ANNEX_D_ANNEX_F_DIFF       (28U)

/**
 * @def TIMESYNC_ANNEX_E_ANNEX_F_DIFF
 *      Number of extra bytes for Annex E(IPv6) from Annex F(802.3)
 */
#define TIMESYNC_ANNEX_E_ANNEX_F_DIFF       (48U)

/**
 * @def TIMESYNC_SINGLE_TAG_VLAN_HDR_SIZE
 *      Number of extra bytes for single tagged VLAN
 */
#define TIMESYNC_SINGLE_TAG_VLAN_HDR_SIZE   (4U)

/**
 * @def TIMESYNC_DOUBLE_TAG_VLAN_HDR_SIZE
 *      Number of extra bytes for double tagged VLAN
 */
#define TIMESYNC_DOUBLE_TAG_VLAN_HDR_SIZE   (8U)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct TimeSync_TxPktInfoEle_s
{
    uint8_t portNum;

    uint8_t frameType;

    uint16_t seqId;
} TimeSync_TxPktInfoEle;

typedef struct TimeSync_TxPktInfo_s
{
    TimeSync_TxPktInfoEle txTsPktInfoArr[TIMESYNC_TX_PKTINFO_COUNT];

    uint32_t rdIdx;

    uint32_t wrIdx;
}TimeSync_TxPktInfo;

/* Test parameters for each port in the multi-channel test */
typedef struct EnetApp_TestParams_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort;

    /* Name of this port to be used for logging */
    char *name;
} EnetApp_TestParams;

/* Context of a peripheral/port */
typedef struct EnetApp_PerCtxt_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort macPort;

    /* Name of this port to be used for logging */
    char *name;

    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* Regular Traffic TX channel number */
    uint32_t txChNum;

    /* PTP Traffic TX channel number */
    uint32_t txPtpChNum;

    /* TX channel handle for Regular traffic*/
    EnetDma_TxChHandle hTxCh;

	/* TX channel handle for PTP traffic */
    EnetDma_TxChHandle hTxPtpCh;


    /* Start flow index */
    uint32_t rxStartFlowIdx;

    /* Regular traffic RX flow index */
    uint32_t rxFlowIdx;

    /* Ptp traffic RX flow index */
    uint32_t rxPtpFlowIdx;

    /* RX channel handle for regular traffic */
    EnetDma_RxChHandle hRxCh;

    /* RX channel handle for PTP traffic */
    EnetDma_RxChHandle hRxPtpCh;

    /* RX task handle - receives Regular packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all REgular RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreKey;

    /* Main UDMA driver handle */
    Udma_DrvHandle hMainUdmaDrv;

    TimeSync_Config timeSyncConfig;

    TimeSync_TxPktInfo txTsPktInfo;
} EnetApp_PerCtxt;

typedef struct EnetApp_Obj_s
{
    /* Flag which indicates if test shall run */
    volatile bool run;

    /* This core's id */
    uint32_t coreId;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

	/* Queue of free PTP TX packets */
    EnetDma_PktQ txPtpFreePktInfoQ;

    /* Array of all peripheral/port contexts used in the test */
    EnetApp_PerCtxt perCtxt[ENETAPP_PER_MAX];

    /* Number of active contexts being used */
    uint32_t numPerCtxts;

    /* Whether timestamp printing is enabled or not */
    bool enableTs;

    /* Handle to PTP stack */
    TimeSyncPtp_Handle hTimeSyncPtp;
} EnetApp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetApp_mainTask(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet multi-channel test object */
EnetApp_Obj gEnetApp;

/* Statistics */
CpswStats_PortStats gEnetApp_cpswStats;

/* Test application stack */
static uint8_t gEnetAppTaskStackTick[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppTaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

/**PTP Multicast addresses for comparison*/
static const uint8_t peerDlyMsgMAC[ENET_MAC_ADDR_LEN] = {0x01, 0x80, 0xc2, 0x00, 0x00, 0x0E};

#ifdef __cplusplus
}
#endif

#endif /* _MULTI_CHANNEL_COMMON_H_ */
