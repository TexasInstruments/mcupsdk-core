/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
/*                             Include Files                                  */
/* ========================================================================== */
/*
 * pnDrvConfig.h needs to be first first file included in driver!
 * It is application dependent and as such part of the application code.
 * It defines all basic feature options in driver (compile time!)
 */
#include <stdint.h>
#include <string.h>
#include "pnDrvConfig.h"
#include "PN_Handle.h"
#include "PN_HandleDef.h"
#include "iRtcDrv2.h"
#include "iPtcpDrv.h"
#include "iPtcpUtils.h"
#include "iPNLegacy.h"
#include "iPnOs.h"
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/mdio.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifdef PTCP_SUPPORT
/*enable RED sync LED on ICE*/
#define PTCP_SYNC_SIGNAL

#define DEBUG_SYNC_EVENTS
#ifdef DEBUG_SYNC_EVENTS
uint32_t syncmissCounter = 0;

#endif

/* OPT Field specific defines */
#define OPT_SYNCDIM_SHIFT                   (0x00000002u)
#define OPT_TCC_MASK                        (0x0003F000u)
#define OPT_TCC_SHIFT                       (0x0000000Cu)
#define OPT_ITCINTEN_SHIFT                  (0x00000015u)
#define OPT_TCINTEN_SHIFT                   (0x00000014u)
#define OPT_STATIC_SHIFT                    (0x00000003u)

/* Macros to define DELAY REQ and RES packets */
#define DA_ID1 0x8001
#define DA_ID2 0x00c2
#define DA_ID3 0x0e00


#define SA_ID1 0x0000
#define SA_ID2 0x0000
#define SA_ID3 0x0000

#define ETHER_TYPE 0x9288
#define DELAY_REQ_FRAME_ID   0x40ff
#define DELAY_RES_FRAME_ID   0x43ff

#define DELAY_RES_TYPE_LENGTH1 0x0a0a
#define DELAY_RES_TYPE_LENGTH2 0x0000

#define DELAY_RES_PORT_RX_DELAY1 0x0000
#define DELAY_RES_PORT_RX_DELAY2 ((PORT_RX_DELAY & 0xFF) << 8) |  ((PORT_RX_DELAY & 0xFF00) >> 8)

#define DELAY_RES_PORT_TX_DELAY1 0x0000
#define DELAY_RES_PORT_TX_DELAY2 ((PORT_TX_DELAY & 0xFF) << 8) |  ((PORT_TX_DELAY & 0xFF00) >> 8)

#define DELAY_RES_TYPE_LENGTH 0x060e

#define ZERO_PADDING 0x0000

#define TLV_DELAY 0x060c

/* ========================================================================== */
/*                             Static variables                               */
/* ========================================================================== */

/**Syslog sync packet*/
uint8_t syncSysLogFrame[] = \

{
    0x00, 0x1b, 0x1b, 0x35, 0x4e, 0xcf, 0xc4, 0xed, 0xba, 0x86, 0xfe, 0x39,
    0x08, 0x00, 0x45, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x80, 0x11,
    0x00, 0x00, 0xC0, 0xA8, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02,
    0x02, 0x02, 0x00, 0x2A, 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20
};
#ifdef LATCH_DEBUG
#define LDBG_SZ 40

LatchVars       g_dbg_latch[LDBG_SZ];
PNIO_TimeStamp  g_dbg_T1[LDBG_SZ];
uint32_t        td1[LDBG_SZ];
uint64_t        tickdiff[LDBG_SZ];
int32_t         index_dbg_api = 0;
#endif

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */

void PN_PTCP_timerHandler(void* arg);

void PN_PTCP_registerSyncStatusCall(PN_Handle pnHandle, ptcpCallBack_t callBack)
{
    (pnHandle->pnPtcpConfig).ptcpSyncStatusCall = callBack;
}

void PN_PTCP_registerDelayUpdateCall(PN_Handle pnHandle,
                                     ptcpCallBack_t callBack)
{
    (pnHandle->pnPtcpConfig).ptcpDelayUpdateCall = callBack;
}

void PN_PTCP_start(PN_Handle pnHandle)
{
    SemaphoreP_post(&((pnHandle->pnPtcpConfig).ptcpStartSem));
}

void PN_PTCP_triggerMeasurement(PN_Handle pnHandle)
{
    SemaphoreP_post(&((pnHandle->pnPtcpConfig).ptcpTriggerMeasurementSem));
}

void PN_PTCP_ClockChange(PN_Handle pnHandle, uint32_t cycleTime)
{
    if((pnHandle->pnPtcpConfig).pnCyclePeriod == cycleTime)
    {
        return;
    }

    (pnHandle->pnPtcpConfig).pnCyclePeriod = cycleTime;
    (pnHandle->pnPtcpConfig).clkChangeNotifyDelay = 1;

    /* reset ptcp sync : direct adjustment and reset the filter*/
    PN_PTCP_reset(pnHandle);
}

void PN_PTCP_init(PN_Handle pnHandle)
{
    uint16_t *pTemp16;
    uint8_t *pTemp8;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);
    PN_PtcpConfig *ptcpConfig = &(pnHandle->pnPtcpConfig);
    uint32_t ocmcBaseAddress = (((pnHandle->emacHandle)->attrs)->l3OcmcBaseAddr);

    /*Write the Delay Req Frame for PORT1 in ICSS Shared RAM*/
    pTemp16 = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                           PORT1_DELAY_REQ_FRAME_OFFSET);
    /*Write DA*/
    *pTemp16++ = DA_ID1;
    *pTemp16++ = DA_ID2;
    *pTemp16++ = DA_ID3;
    /*Write SA .. Port MAC Address*/
    *pTemp16++ = SA_ID1;
    *pTemp16++ = SA_ID2;
    *pTemp16++ = SA_ID3;
    /*Write Type and Frame ID*/
    *pTemp16++ = ETHER_TYPE;
    *pTemp16++ = DELAY_REQ_FRAME_ID;
    /*Write 0 padding*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /* Write the Initial value of Seq ID*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /* Write the TLV - Delay Parameter*/
    *pTemp16++ = TLV_DELAY;
    /*Write SA .. Port MAC Address*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /*crypto: check reason for extra padding: min length ??*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;

    /* write Dst MAC addr at two locations*/
    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         PORT1_DELAY_REQ_FRAME_OFFSET + 6);
    PN_EmacSocMACAddrGet(pnHandle, PORT1_MAC, pTemp8);

    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         PORT1_DELAY_REQ_FRAME_OFFSET + 38);
    PN_EmacSocMACAddrGet(pnHandle, PORT1_MAC, pTemp8);

    /*Write the Delay Req Frame for PORT2 in ICSS Shared RAM*/
    pTemp16 = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                           PORT2_DELAY_REQ_FRAME_OFFSET);
    /*Write DA*/
    *pTemp16++ = DA_ID1;
    *pTemp16++ = DA_ID2;
    *pTemp16++ = DA_ID3;
    /*Write SA .. Port MAC Address*/
    *pTemp16++ = SA_ID1;
    *pTemp16++ = SA_ID2;
    *pTemp16++ = SA_ID3;
    /*Write Type and Frame ID*/
    *pTemp16++ = ETHER_TYPE;
    *pTemp16++ = DELAY_REQ_FRAME_ID;
    /*Write 0 padding*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /* Write the Initial value of Seq ID*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /* Write the TLV - Delay Parameter*/
    *pTemp16++ = TLV_DELAY;
    /*Write SA .. Port MAC Address*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /*crypto: check reason for extra padding: min length ??*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;

    /* write Src MAC addr at two locations*/
    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         PORT2_DELAY_REQ_FRAME_OFFSET + 6);
    PN_EmacSocMACAddrGet(pnHandle, PORT2_MAC, pTemp8);

    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         PORT2_DELAY_REQ_FRAME_OFFSET + 38);
    PN_EmacSocMACAddrGet(pnHandle, PORT2_MAC, pTemp8);

    /* Write the Delay_Response Frame for Port1 in ICSS Shared RAM*/
    pTemp16 = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                           PORT1_DELAY_RESP_FRAME_OFFSET);
    /*Write DA*/
    *pTemp16++ = DA_ID1;
    *pTemp16++ = DA_ID2;
    *pTemp16++ = DA_ID3;
    /*Write SA .. Port MAC Address*/
    *pTemp16++ = SA_ID1;
    *pTemp16++ = SA_ID2;
    *pTemp16++ = SA_ID3;
    /*Write Type and Frame ID*/
    *pTemp16++ = ETHER_TYPE;
    *pTemp16++ = DELAY_RES_FRAME_ID;
    /* Write the Initial value of Seq ID / padding/delay*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /* Write the TLV - Delay Parameter*/
    *pTemp16++ = TLV_DELAY;
    /*Write requestor Port MAC Address*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /* Write TypeLength*/
    *pTemp16++ = DELAY_RES_TYPE_LENGTH1;
    *pTemp16++ = DELAY_RES_TYPE_LENGTH2;
    /* T2 PortRxDelay*/
    *pTemp16++ = DELAY_RES_PORT_RX_DELAY1;
    *pTemp16++ = DELAY_RES_PORT_RX_DELAY2;
    /* T3 PortTxDelay*/
    *pTemp16++ = DELAY_RES_PORT_TX_DELAY1;
    *pTemp16++ = DELAY_RES_PORT_TX_DELAY2;
    /* TypeLength*/
    *pTemp16++ = DELAY_RES_TYPE_LENGTH;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;

    /* write Src MAC addr*/
    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         PORT1_DELAY_RESP_FRAME_OFFSET + 6);
    PN_EmacSocMACAddrGet(pnHandle, PORT1_MAC, pTemp8);

    /* Write the Delay_Response Frame for Port2 in ICSS Shared RAM*/
    pTemp16 = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                           PORT2_DELAY_RESP_FRAME_OFFSET);
    /*Write DA*/
    *pTemp16++ = DA_ID1;
    *pTemp16++ = DA_ID2;
    *pTemp16++ = DA_ID3;
    /*Write SA .. Port MAC Address*/
    *pTemp16++ = SA_ID1;
    *pTemp16++ = SA_ID2;
    *pTemp16++ = SA_ID3;
    /*Write Type and Frame ID*/
    *pTemp16++ = ETHER_TYPE;
    *pTemp16++ = DELAY_RES_FRAME_ID;
    /* Write the Initial value of Seq ID / padding/delay*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /* Write the TLV - Delay Parameter*/
    *pTemp16++ = TLV_DELAY;
    /*Write requestor Port MAC Address*/
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    /* Write TypeLength*/
    *pTemp16++ = DELAY_RES_TYPE_LENGTH1;
    *pTemp16++ = DELAY_RES_TYPE_LENGTH2;
    /* T2 PortRxDelay*/
    *pTemp16++ = DELAY_RES_PORT_RX_DELAY1;
    *pTemp16++ = DELAY_RES_PORT_RX_DELAY2;
    /* T3 PortTxDelay*/
    *pTemp16++ = DELAY_RES_PORT_TX_DELAY1;
    *pTemp16++ = DELAY_RES_PORT_TX_DELAY2;
    /* TypeLength*/
    *pTemp16++ = DELAY_RES_TYPE_LENGTH;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;
    *pTemp16++ = ZERO_PADDING;

    /* write Src MAC addr*/
    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         PORT2_DELAY_RESP_FRAME_OFFSET + 6);
    PN_EmacSocMACAddrGet(pnHandle, PORT2_MAC, pTemp8);

    /* Write to PRU0 Handshake Reg*/
    pTemp16 = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                           PRU0_HANDSHAKE_OFFSET);
    *pTemp16++ = 0x0000;
    *pTemp16++ = 0x0000;
    /* Write to PRU1 Handshake Reg*/
    pTemp16 = (uint16_t *)(pruicssHwAttrs->sharedDramBase +
                           PRU1_HANDSHAKE_OFFSET);
    *pTemp16++ = 0x0000;
    *pTemp16++ = 0x0000;

    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pT1TS = (uint32_t *)(
                pruicssHwAttrs->sharedDramBase + P1_T1_TIME_STAMP_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pT4TS = (uint32_t *)(
                pruicssHwAttrs->sharedDramBase + P1_T4_TIME_STAMP_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pT1CycleCtr = (uint32_t *)(
                pruicssHwAttrs->sharedDramBase + P1_T1_CYCLE_CTR_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pT4CycleCtr = (uint32_t *)(
                pruicssHwAttrs->sharedDramBase + P1_T4_CYCLE_CTR_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pDelayReqPacket =
        (uint8_t *)(pruicssHwAttrs->sharedDramBase + PORT1_DELAY_REQ_FRAME_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pSeqIdInDelayPacket =
        (uint16_t *)(pruicssHwAttrs->sharedDramBase + PORT1_DELAY_REQ_FRAME_OFFSET
                     + 28);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pInDelayResPacket =
        (uint8_t *)(ocmcBaseAddress + P1_DLY_RSP_PACKET_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pInDelayFupResPacket =
        (uint8_t *)(ocmcBaseAddress + P1_DLY_FUP_PACKET_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pInDelayResCtrl =
        (uint8_t *)(ocmcBaseAddress + P1_PTCP_CTRL_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_1 - 1].pSmaLineDelay =
        (uint32_t *)(pruicssHwAttrs->sharedDramBase + P1_SMA_LINE_DELAY_OFFSET);


    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pT1TS = (uint32_t *)(
                pruicssHwAttrs->sharedDramBase + P2_T1_TIME_STAMP_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pT4TS = (uint32_t *)(
                pruicssHwAttrs->sharedDramBase + P2_T4_TIME_STAMP_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pT1CycleCtr = (uint32_t *)(
                pruicssHwAttrs->sharedDramBase + P2_T1_CYCLE_CTR_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pT4CycleCtr = (uint32_t *)(
                pruicssHwAttrs->sharedDramBase + P2_T4_CYCLE_CTR_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pDelayReqPacket =
        (uint8_t *)(pruicssHwAttrs->sharedDramBase + PORT2_DELAY_REQ_FRAME_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pSeqIdInDelayPacket =
        (uint16_t *)(pruicssHwAttrs->sharedDramBase + PORT2_DELAY_REQ_FRAME_OFFSET
                     + 28);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pInDelayResPacket =
        (uint8_t *)(ocmcBaseAddress + P2_DLY_RSP_PACKET_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pInDelayFupResPacket =
        (uint8_t *)(ocmcBaseAddress + P2_DLY_FUP_PACKET_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pInDelayResCtrl =
        (uint8_t *)(ocmcBaseAddress + P2_PTCP_CTRL_OFFSET);
    ptcpConfig->devicePortOffsets[ICSS_EMAC_PORT_2 - 1].pSmaLineDelay =
        (uint32_t *)(pruicssHwAttrs->sharedDramBase + P2_SMA_LINE_DELAY_OFFSET);

    PN_PTCP_resetDelayValues(pnHandle);
    ptcpConfig->seqId      = 0;

    /* reset the flag, as ptcp measurement hasn't started yet*/
    ptcpConfig->clkChangeNotifyDelay = 0;

    memset((void *) & (ptcpConfig->syncDeltaTs), 0,
           ptcpConfig->ptcpSyncFilterfactor);

    ptcpConfig->syncIndex = 0;

    ptcpConfig->pSyncTorgNs         = (uint32_t *)(
                                          pruicssHwAttrs->sharedDramBase + SYNC_TORG_TIME_OFFSET +
                                          4); /* 4 byte offset for ns field*/
    ptcpConfig->pSyncTorgSecs       = (uint32_t *)(
                                          pruicssHwAttrs->sharedDramBase + SYNC_TORG_TIME_OFFSET);
    ptcpConfig->pSyncInDelayPlusLD  = (uint32_t *)(
                                          pruicssHwAttrs->sharedDramBase + SYNC_INDELAY_PLUS_LD_OFFSET);
    ptcpConfig->pSyncRxSOF          = (uint32_t *)(
                                          pruicssHwAttrs->sharedDramBase + SYNC_RX_SOF_OFFSET);
    ptcpConfig->pPmCycleCounter     = (uint16_t *)(pruicssHwAttrs->pru0DramBase
                                      + PTCP_PM_CYCLE_COUNTER_OFFSET);
    ptcpConfig->pPmPhaseCounter     = (uint16_t *)(pruicssHwAttrs->pru0DramBase
                                      + PTCP_PM_PHASE_COUNTER_OFFSET);

    ptcpConfig->pSubDomainUUID      = (uint8_t *)(
                                          pruicssHwAttrs->sharedDramBase + SYNC_UUID_OFFSET);
    ptcpConfig->pSyncMasterMac      = (uint8_t *)(
                                          pruicssHwAttrs->sharedDramBase + SYNC_MASTER_MAC_OFFSET);
    ptcpConfig->pSyncInitFlag       = (uint8_t *)(
                                          pruicssHwAttrs->sharedDramBase + SYNC_INIT_FLAG_OFFSET);

    PN_PTCP_setupIsr(pnHandle);
    PN_PTCP_reset(pnHandle);
    PN_PTCP_configureSync0Pin(pnHandle);

#ifdef PTCP_DEBUG
    PN_PtcpDebug *pnPtcpDebugAttrs = pnHandle->pnPtcpConfig->pnPtcpDebugAttrs;
    pnPtcpDebugAttrs->debugDelayIndex = 1;
    pnPtcpDebugAttrs->debugSyncIndex    = 1;
#endif

/*TODO: Review this*/
// #ifdef PTCP_SYNC_SIGNAL
//     GPIO_write(0, 0);
// #endif

    memset((void *)((ptcpConfig->deviceSyncInfo).masterSA), 0, 6);
    memset((void *)((ptcpConfig->deviceSyncInfo).subdomainUUID), 0, 16);
    memset((void *)((ptcpConfig->currentPtcpStatus).masterSA), 0, 6);
    memset((void *)((ptcpConfig->currentPtcpStatus).subdomainUUID), 0, 16);

    memset((void *)(ptcpConfig->pSubDomainUUID), 0, 16);
    memset((void *)(ptcpConfig->pSyncMasterMac), 0, 6);
    memset((void *)(ptcpConfig->pSyncInitFlag), 0, 1);

    SemaphoreP_constructBinary(&(ptcpConfig->ptcpStartSem), 0);
    SemaphoreP_constructBinary(&( ptcpConfig->ptcpTriggerMeasurementSem), 0);

    /*enable by default*/
    (ptcpConfig->currentPtcpStatus).cDelayEnable[ICSS_EMAC_PORT_1 - 1] = enable;
    (ptcpConfig->currentPtcpStatus).cDelayEnable[ICSS_EMAC_PORT_2 - 1] = enable;

    /* enabling Single shot mode for capture register 4/5, i.e. TX PORT1 and TX PORT2*/
    HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CAP_CFG_REG, 0x0001FC30);

    /* enable sync forwarding by default*/
    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         SYNC_FWD_ENABLED_OFFSET);
    *pTemp8 = 1;

    /* enable resp for delay req in firmware by default*/
    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         P1_DELAY_RESP_CTRL_OFFSET);
    *pTemp8 = 1;

    /* enable resp for delay req in firmware by default*/
    pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                         P2_DELAY_RESP_CTRL_OFFSET);
    *pTemp8 = 1;
}

void PN_PTCP_reset(PN_Handle pnHandle)
{
    PN_PtcpConfig *ptcpConfig = &(pnHandle->pnPtcpConfig);
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    ((pnHandle->pnPtcpConfig).pnPtcpDebugAttrs).debugSyncIndex = 0;

    /* Initialize current PTCP status of the device*/
    ptcpConfig->currentPtcpStatus.syncState = OUT_OF_SYNC;
    ptcpConfig->currentPtcpStatus.syncPllWnd =
        1000;           /* default value of 1 us*/
    ptcpConfig->currentPtcpStatus.syncTimeoutFactor =
        6;       /* default value of 6 sync interval*/
    ptcpConfig->currentPtcpStatus.takeoverTimeoutFactor =
        3;   /* default value of 3 sync interval*/
    ptcpConfig->currentPtcpStatus.firstSyncRcv = 0;
    ptcpConfig->currentPtcpStatus.syncRcv = 0;
    ptcpConfig->currentPtcpStatus.nSyncMissed = 0;

    if(ptcpConfig->pnCyclePeriod == 0)
    {
        ptcpConfig->pnCyclePeriod =
            250000;    /*default value, if it is not initialized yet!*/
    }

    ptcpConfig->deviceSyncInfo.syncState = OUT_OF_SYNC;
    ptcpConfig->numInSync = 0;
    ptcpConfig->syncIndex = 0;
    ptcpConfig->initPmCycleCtrDone = 0;

    HW_WR_REG8(pruicssHwAttrs->pru0DramBase
           + RTC_DEVICE_SYNC_STATUS_OFFSET, 0);

    if((pnHandle->pnPtcpConfig).ptcpSyncStatusCall != NULL)
    {
        (pnHandle->pnPtcpConfig).ptcpSyncStatusCall((
                    pnHandle->pnPtcpConfig).currentPtcpStatus.syncState, (uint32_t)NULL);
    }
/*TODO: Review this*/
// #ifdef PTCP_SYNC_SIGNAL
//     GPIO_write(0, 0);
// #endif
#ifdef SYNC_ANALYSIS
    nResets++;
    nSyncTrans = 0;
#endif
}

void PN_PTCP_delayMeasurement(PN_Handle pnHandle)
{
    int32_t i = 0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    (pnHandle->pnPtcpConfig).mandCtrAdj = MAX_CTR_VAL * (INTER_DEL_REQ_GAP / ((
            pnHandle->pnPtcpConfig).pnCyclePeriod * MAX_CTR_VAL));

    for(i = 0; i < NUM_DELAYS_BURST; i++)
    {
        /* increment the seqId and delayIndex*/
        (pnHandle->pnPtcpConfig).seqId = ((pnHandle->pnPtcpConfig).seqId + 1) %
                                          MAX_SEQID;
        (pnHandle->pnPtcpConfig).delayIndex = ((pnHandle->pnPtcpConfig).delayIndex +
                                                1) % NUM_DELAYS_SMA;

        /* reset the values*/
        PN_PTCP_resetDelayTimings(pnHandle, ICSS_EMAC_PORT_1 - 1);
        PN_PTCP_resetDelayTimings(pnHandle, ICSS_EMAC_PORT_2 - 1);

        /* clear the flag if clk changed before delay measurement*/
        if((pnHandle->pnPtcpConfig).clkChangeNotifyDelay == 1)
        {
            /* check whether PRU has programmed the IEP or not*/
            uint8_t *cycleChangeDone = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                                                   RTC_BASE_CLK_CHANGED_OFFSET);

            if((*cycleChangeDone) == 0)
            {
                (pnHandle->pnPtcpConfig).clkChangeNotifyDelay = 0;
                /* recalculate if cycle period has changed*/
                (pnHandle->pnPtcpConfig).mandCtrAdj = MAX_CTR_VAL * (INTER_DEL_REQ_GAP / ((
                        pnHandle->pnPtcpConfig).pnCyclePeriod * MAX_CTR_VAL));
            }
        }

        /* fill queue1 with delay req*/
        if((pnHandle->pnPtcpConfig).currentPtcpStatus.cDelayEnable[0] == enable)
            PN_OS_txPacket(pnHandle->emacHandle,
                           (pnHandle->pnPtcpConfig).devicePortOffsets[0].pDelayReqPacket,
                           ICSS_EMAC_PORT_1, ICSS_EMAC_QUEUE1, PTCP_DELAY_REQ_LEN);

        if((pnHandle->pnPtcpConfig).currentPtcpStatus.cDelayEnable[1] == enable)
            PN_OS_txPacket(pnHandle->emacHandle,
                           (pnHandle->pnPtcpConfig).devicePortOffsets[1].pDelayReqPacket,
                           ICSS_EMAC_PORT_2, ICSS_EMAC_QUEUE1, PTCP_DELAY_REQ_LEN);

        PN_PTCP_taskSleep(INTER_DEL_REQ_GAP_MS);


        /* process the delay res*/
        if((pnHandle->pnPtcpConfig).currentPtcpStatus.cDelayEnable[0] == enable)
        {
            PN_PTCP_processDelayResponse(pnHandle, ICSS_EMAC_PORT_1 - 1);
        }

        if((pnHandle->pnPtcpConfig).currentPtcpStatus.cDelayEnable[1] == enable)
        {
            PN_PTCP_processDelayResponse(pnHandle, ICSS_EMAC_PORT_2 - 1);
        }

#ifdef PTCP_DEBUG

        /* for debug: capturing cable delay for 100 iterations*/
        if((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs).debugDelayIndex <
                DEBUG_DELAY_N_ITER - 1)
        {
            (pnHandle->pnPtcpConfig->pnPtcpDebugAttrs).debugDelayIndex++;
        }

#endif
    }
}

void PN_PTCP_resetDelayValues(PN_Handle pnHandle)
{
    int32_t i, j;
    (pnHandle->pnPtcpConfig).delayIndex        =
        -1;   /* FW setting index to -1 may cause array out of bounds!*/
    (pnHandle->pnPtcpConfig).firstDelayBurst = 1;


    for(i = ICSS_EMAC_PORT_1 - 1; i < PTCP_NUM_PORTS; i++)
    {
        (pnHandle->pnPtcpConfig).portTimes[i].cableDelay     = 0;
        (pnHandle->pnPtcpConfig).portTimes[i].rxDelayLocal = PORT_RX_DELAY;
        (pnHandle->pnPtcpConfig).portTimes[i].txDelayLocal = PORT_TX_DELAY;
        (pnHandle->pnPtcpConfig).portTimes[i].rxDelayRemote = 0;
        (pnHandle->pnPtcpConfig).portTimes[i].txDelayRemote = 0;

        (pnHandle->pnPtcpConfig).T1_prev[i]        = 0;
        (pnHandle->pnPtcpConfig).T2_prev[i]        = 0;
        (pnHandle->pnPtcpConfig).T1_CTR_prev[i]    = 0;

        /*currentPtcpStatus.cDelay[i] = 0;*/

        for(j = 0; j < NUM_DELAYS_SMA; j++)
        {
            (pnHandle->pnPtcpConfig).deviceDelays[i].lDelays[j] = 0;
            (pnHandle->pnPtcpConfig).deviceDelays[i].cDelays[j] = 0;
        }
    }

}

void PN_PTCP_resetDelayTimings(PN_Handle pnHandle, uint8_t portNum)
{
    /* reset the timing values*/
    *((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pT1TS)           =
        0x00000000;
    *((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pT4TS)           =
        0x00000000;
    *((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pInDelayResCtrl)     =
        0x00000000;
    /* set the seqId in delay req packet*/
    *((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pSeqIdInDelayPacket) =
        (((pnHandle->pnPtcpConfig).seqId << 8) & 0xFF00) | (((
                    pnHandle->pnPtcpConfig).seqId >> 8) & 0x00FF);
}

void PN_PTCP_processDelayResponse(PN_Handle pnHandle, uint8_t portNum)
{
    ptcp_iDelayResp_struct_t    ptcp_iDelayResp_parsed;

    /* parse the delay resp and fup packet*/
    if(*((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pInDelayResCtrl) ==
            1)/*resp without FUP*/
        PN_PTCP_parseInDelayResp(&ptcp_iDelayResp_parsed,
                                 ((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pInDelayResPacket),
                                 ((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pInDelayFupResPacket), 0);

    else if(*((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pInDelayResCtrl)
            == 6)/*resp with FUP and also FUP received*/
        PN_PTCP_parseInDelayResp(&ptcp_iDelayResp_parsed,
                                 ((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pInDelayResPacket),
                                 ((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pInDelayFupResPacket), 1);

    else                            /* no response received or double resp recv or other error combination*/
    {
        (pnHandle->pnPtcpConfig).deviceDelays[portNum].cDelays[(pnHandle->pnPtcpConfig).delayIndex]
            = 0;
        (pnHandle->pnPtcpConfig).T2_prev[portNum]                              =
            0;    /*discard the previous values for next cable delay measurement*/
#ifdef PTCP_DEBUG
        ((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelay[portNum]).cDelay[(pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelayIndex]
            = -1;
#endif
        return;
    }

    /* update timestamps*/
    ptcp_iDelayResp_parsed.T1TimeStamp = *((
            pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pT1TS);
    ptcp_iDelayResp_parsed.T4TimeStamp = *((
            pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pT4TS);
    ptcp_iDelayResp_parsed.T1_cycle_ctr = *((
            pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pT1CycleCtr);
    ptcp_iDelayResp_parsed.T4_cycle_ctr = *((
            pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pT4CycleCtr);
    ptcp_iDelayResp_parsed.T1PortTXDelay = PORT_TX_DELAY;
    ptcp_iDelayResp_parsed.T4PortRXDelay = PORT_RX_DELAY;

    /* caculate line delay, rcf_peer and cable delay*/
    PN_PTCP_lineDelayCalc(pnHandle, &ptcp_iDelayResp_parsed);
    PN_PTCP_cableDelayCalc(pnHandle, &ptcp_iDelayResp_parsed, portNum);

    (pnHandle->pnPtcpConfig).deviceDelays[portNum].cDelays[(pnHandle->pnPtcpConfig).delayIndex]
        = ptcp_iDelayResp_parsed.cable_delay;
    (pnHandle->pnPtcpConfig).deviceDelays[portNum].lDelays[(pnHandle->pnPtcpConfig).delayIndex]
        = ptcp_iDelayResp_parsed.line_delay;
    (pnHandle->pnPtcpConfig).portTimes[portNum].rxDelayRemote =
        ptcp_iDelayResp_parsed.T2PortRXDelay;
    (pnHandle->pnPtcpConfig).portTimes[portNum].txDelayRemote =
        ptcp_iDelayResp_parsed.T3PortTXDelay;

#ifdef PTCP_DEBUG
    /*for debug purposes*/
    ((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelay[portNum]).cDelay[(pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelayIndex]
        = ptcp_iDelayResp_parsed.cable_delay;
    ((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelay[portNum]).lDelay[(pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelayIndex]
        = ptcp_iDelayResp_parsed.line_delay;
    ((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelay[portNum]).rcfPeer[(pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelayIndex]
        = ptcp_iDelayResp_parsed.rcf_peer;
    ((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelay[portNum]).reqDelay[(pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelayIndex]
        = ptcp_iDelayResp_parsed.reqDelay;
    ((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelay[portNum]).resDelay[(pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelayIndex]
        = ptcp_iDelayResp_parsed.resDelay;
    ((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelay[portNum]).seqId[(pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugDelayIndex]
        = ptcp_iDelayResp_parsed.seqId;
#endif

#ifdef PTCP_DEBUG_LI

    if((pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->debugSyncIndex > 50
            && portNum == 0)
    {
        if(ptcp_iDelayResp_parsed.cable_delay >
                (pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->maxDelay)
        {
            (pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->maxDelay =
                ptcp_iDelayResp_parsed.cable_delay;
        }

        if(ptcp_iDelayResp_parsed.cable_delay <
                (pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->minDelay)
        {
            (pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->minDelay =
                ptcp_iDelayResp_parsed.cable_delay;
        }

        if(ptcp_iDelayResp_parsed.cable_delay > 100)
        {
            (pnHandle->pnPtcpConfig->pnPtcpDebugAttrs)->delayOutOfRange++;
        }
    }

#endif
}
void PN_PTCP_smaDelayMeasurement(PN_Handle pnHandle)
{
    uint32_t  portNum;
    /*TODO: Review this. ClockP_getTickPeriod was used for TI-RTOS osal*/
    uint32_t  measurementTimeout = (((INTER_DEL_REQ_BURST_GAP_MS-INTER_DEL_REQ_GAP_MS) * 1000) /
                                    ClockP_ticksToUsec(1)) + 1;
    uint8_t linkStatus = 0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((pnHandle->pruicssHandle)->hwAttrs);

    PN_PTCP_enableIsr(pnHandle);
    PN_PTCP_start(pnHandle);

    SemaphoreP_pend(&((pnHandle->pnPtcpConfig).ptcpStartSem), SystemP_WAIT_FOREVER); /* wait for trigger to start measurement*/

    do
    {
        PN_PTCP_delayMeasurement(pnHandle);


        for(portNum = ICSS_EMAC_PORT_1 - 1; portNum < PTCP_NUM_PORTS; portNum++)
        {
            /*TODO: Review this*/
            linkStatus = 0;
            if((MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase, ((ICSS_EMAC_Attrs *)((pnHandle->emacHandle)->attrs))->phyAddr[portNum]) == SystemP_SUCCESS))
                linkStatus = 1;
            // linkStatus = ((ICSS_EmacObject *)
            //               pnHandle->emacHandle->object)->linkStatus[portNum];

            if((linkStatus == 0)
                    || (pnHandle->pnPtcpConfig).currentPtcpStatus.cDelayEnable[portNum] ==
                    disable)
            {
                (pnHandle->pnPtcpConfig).portTimes[portNum].cableDelay = 0;
            }

            else
            {
                PN_PTCP_portDelaySmaCalc(pnHandle, portNum);
            }

            /* inform stack about new measurement update*/
            if((pnHandle->pnPtcpConfig).currentPtcpStatus.cDelay[portNum] !=
                    (pnHandle->pnPtcpConfig).portTimes[portNum].cableDelay)
            {
                (pnHandle->pnPtcpConfig).currentPtcpStatus.cDelay[portNum] =
                    (pnHandle->pnPtcpConfig).portTimes[portNum].cableDelay;

                if((pnHandle->pnPtcpConfig).ptcpDelayUpdateCall != NULL)
                {
                    (pnHandle->pnPtcpConfig).ptcpDelayUpdateCall(portNum,
                            (pnHandle->pnPtcpConfig).portTimes[portNum].cableDelay);
                }
            }
        }

        if(SemaphoreP_pend(&((pnHandle->pnPtcpConfig).ptcpTriggerMeasurementSem),
                           measurementTimeout))
        {
            PN_PTCP_resetDelayValues(pnHandle);
        }
    }
    while(1);
}

void PN_PTCP_portDelaySmaCalc(PN_Handle pnHandle, uint8_t portNum)
{
    uint32_t    lDelaySum = 0, cDelaySum = 0;
    uint32_t    i = 0;

    for(i = 0; i < NUM_DELAYS_SMA; i++)
    {
        lDelaySum = lDelaySum + ((
                                     pnHandle->pnPtcpConfig).deviceDelays[portNum]).lDelays[i];
        cDelaySum = cDelaySum + ((
                                     pnHandle->pnPtcpConfig).deviceDelays[portNum]).cDelays[i];
    }

    if((pnHandle->pnPtcpConfig).firstDelayBurst == 1)
    {
        *((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pSmaLineDelay) =
            lDelaySum / NUM_DELAYS_BURST;
        (pnHandle->pnPtcpConfig).portTimes[portNum].cableDelay = cDelaySum /
                NUM_DELAYS_BURST;
    }

    else
    {
        *((pnHandle->pnPtcpConfig).devicePortOffsets[portNum].pSmaLineDelay) =
            lDelaySum / NUM_DELAYS_SMA;
        (pnHandle->pnPtcpConfig).portTimes[portNum].cableDelay = cDelaySum /
                NUM_DELAYS_SMA;
    }

    (pnHandle->pnPtcpConfig).firstDelayBurst = 0;
}

int32_t PN_PTCP_lineDelayCalc(PN_Handle pnHandle,
                              ptcp_iDelayResp_struct_t *ptcp_port_desc)
{
    int32_t ctr_diff;
    ctr_diff = (ptcp_port_desc->T4_cycle_ctr) - (ptcp_port_desc->T1_cycle_ctr);

    if(ctr_diff < 0)
    {
        ctr_diff = ctr_diff + 512;
    }

    ptcp_port_desc->reqDelay = ctr_diff * (pnHandle->pnPtcpConfig).pnCyclePeriod;
    ptcp_port_desc->reqDelay = (int32_t)(ptcp_port_desc->reqDelay) + (int32_t)(
                                   ptcp_port_desc->T4TimeStamp - ptcp_port_desc->T1TimeStamp);

    return 0;
}

int32_t PN_PTCP_parseInDelayResp(ptcp_iDelayResp_struct_t
                                 *ptcp_iDelayResp_parsed, uint8_t *ptcp_iDelayResp_packet,
                                 uint8_t *ptcp_iDelayFupResp_packet, int32_t w_FUP)
{
    ptcp_iDelayResp_parsed->seqId           =   PN_PTCP_rotUshort((uint16_t *)(
                ptcp_iDelayResp_packet + 28));
    ptcp_iDelayResp_parsed->resDelay        =   PN_PTCP_rotUint((uint32_t *)(
                ptcp_iDelayResp_packet + 32));
    ptcp_iDelayResp_parsed->T2PortRXDelay   =   PN_PTCP_rotUint((uint32_t *)(
                ptcp_iDelayResp_packet + 48));
    ptcp_iDelayResp_parsed->T3PortTXDelay   =   PN_PTCP_rotUint((uint32_t *)(
                ptcp_iDelayResp_packet + 52));
    ptcp_iDelayResp_parsed->T2TimeStamp     =   PN_PTCP_rotUint((uint32_t *)(
                ptcp_iDelayResp_packet + 60));

    if(w_FUP == 1)
    {
        if(ptcp_iDelayResp_parsed->seqId != PN_PTCP_rotUshort((uint16_t *)(
                    ptcp_iDelayFupResp_packet + 28)))
        {
            return -4;    /* incorrect FUP response*/
        }

        ptcp_iDelayResp_parsed->resDelay     = PN_PTCP_rotUint((uint32_t *)(
                ptcp_iDelayFupResp_packet + 32));
    }

    return 0;
}

int32_t PN_PTCP_cableDelayCalc(PN_Handle pnHandle,
                               ptcp_iDelayResp_struct_t *ptcpDelayRespParsed, uint8_t port)
{
    uint32_t temp;
    int32_t ctrDiff;
    int64_t numerator;
    numerator = ptcpDelayRespParsed->T2TimeStamp -
                (pnHandle->pnPtcpConfig).T2_prev[port];

    if(numerator < 0)
    {
        numerator = numerator + 4294967296;    /*2^32*/
    }

    ctrDiff = (ptcpDelayRespParsed->T1_cycle_ctr -
               (pnHandle->pnPtcpConfig).T1_CTR_prev[port]);

    ctrDiff = PN_PTCP_adjCtrDiff(pnHandle, ctrDiff);

    temp = ctrDiff * (pnHandle->pnPtcpConfig).pnCyclePeriod;
    temp = temp + (int32_t)((int32_t)ptcpDelayRespParsed->T1TimeStamp - (int32_t)(
                                pnHandle->pnPtcpConfig).T1_prev[port]);

    /* rcf_peer calculation */
    if((pnHandle->pnPtcpConfig).T2_prev[port] ==
            0)                                /* for the first time*/
    {
        ptcpDelayRespParsed->rcf_peer = 1;
    }

    else if((((pnHandle->pnPtcpConfig).seqId) % 5) == 1)
    {
        ptcpDelayRespParsed->rcf_peer = (pnHandle->pnPtcpConfig).rcf_prev[port];
    }

    else
    {
        ptcpDelayRespParsed->rcf_peer = (float)(numerator) / (float)temp;
    }

    (pnHandle->pnPtcpConfig).T2_prev[port] = ptcpDelayRespParsed->T2TimeStamp;
    (pnHandle->pnPtcpConfig).T1_prev[port] = ptcpDelayRespParsed->T1TimeStamp;
    (pnHandle->pnPtcpConfig).T1_CTR_prev[port] = ptcpDelayRespParsed->T1_cycle_ctr;
    (pnHandle->pnPtcpConfig).rcf_prev[port] = ptcpDelayRespParsed->rcf_peer;

    /* cable_delay calculation */
    ptcpDelayRespParsed->resDelay_peer = (ptcpDelayRespParsed->resDelay) /
                                         (ptcpDelayRespParsed->rcf_peer);

    ptcpDelayRespParsed->cable_delay = (ptcpDelayRespParsed->reqDelay -
                                        ptcpDelayRespParsed->resDelay_peer - ptcpDelayRespParsed->T1PortTXDelay -
                                        ptcpDelayRespParsed->T2PortRXDelay - ptcpDelayRespParsed->T3PortTXDelay -
                                        ptcpDelayRespParsed->T4PortRXDelay) / 2;

    if(ptcpDelayRespParsed->reqDelay < ptcpDelayRespParsed->resDelay_peer +
            ptcpDelayRespParsed->T1PortTXDelay + ptcpDelayRespParsed->T2PortRXDelay +
            ptcpDelayRespParsed->T3PortTXDelay +
            ptcpDelayRespParsed->T4PortRXDelay)          /* Make sure that it should never occur*/
    {
        ptcpDelayRespParsed->cable_delay = 0;
    }

    if(ptcpDelayRespParsed->cable_delay >
            100000)    /*Discard the value completely(wire greater than 10 KM!!) Make sure that it should never occur*/
    {
#ifdef PTCP_DEBUG
        DebugP_log("Wire greater that 10KM!! This should not happen");
#endif
        ptcpDelayRespParsed->cable_delay = 0;
    }

    ptcpDelayRespParsed->line_delay = (ptcpDelayRespParsed->cable_delay) +
                                      (ptcpDelayRespParsed->T3PortTXDelay) + (ptcpDelayRespParsed->T4PortRXDelay);

    /*ignore the measurement and use previous value, in case of clock change*/
    /* use the previous values even in case of phase counter reset*/
    if((pnHandle->pnPtcpConfig).clkChangeNotifyDelay == 1
            || (pnHandle->pnPtcpConfig).phaseCtrChange == 1)
    {
        (pnHandle->pnPtcpConfig).T2_prev[port] =
            0;        /*don't use this data for next measurement of rcf*/
        /*completely discard this value*/
        ptcpDelayRespParsed->cable_delay = (pnHandle->pnPtcpConfig).prev_cDelay[port];
        ptcpDelayRespParsed->line_delay = (pnHandle->pnPtcpConfig).prev_lDelay[port];

        if((pnHandle->pnPtcpConfig).phaseCtrChange == 1)
        {
            (pnHandle->pnPtcpConfig).phaseCtrChange = 0;
        }
    }

    (pnHandle->pnPtcpConfig).prev_cDelay[port] = ptcpDelayRespParsed->cable_delay;
    (pnHandle->pnPtcpConfig).prev_lDelay[port] = ptcpDelayRespParsed->line_delay;
    return 0;
}

int32_t PN_PTCP_adjCtrDiff(PN_Handle pnHandle, int32_t ctrDiff)
{
    if(ctrDiff < 0)
    {
        return ctrDiff + MAX_CTR_VAL + (pnHandle->pnPtcpConfig).mandCtrAdj;
    }

    else
    {
        return ctrDiff + (pnHandle->pnPtcpConfig).mandCtrAdj;
    }
}

uint32_t PN_PTCP_rotUint(uint32_t *input)
{
    return ((*input & 0xFF000000) >> 24) | ((*input & 0x00FF0000) >> 8) | ((
                *input & 0x0000FF00) << 8) | ((*input & 0x000000FF) << 24);
}

uint16_t PN_PTCP_rotUshort(uint16_t *input)
{
    return ((*input & 0xFF00) >> 8) | ((*input & 0x00FF) << 8);
}


int32_t PN_PTCP_parseSyncFields(PN_Handle pnHandle,
                                volatile ptcpSyncInfo_t *ptcp_sync_parsed, uint8_t *sync_sblock)
{
    int32_t i;
    uint8_t syncInitFlag = *((pnHandle->pnPtcpConfig).pSyncInitFlag);

    if((pnHandle->pnPtcpConfig).currentPtcpStatus.firstSyncRcv == 0
            || (pnHandle->pnPtcpConfig).masterChange == 1)
    {
        for(i = 0; i < 6; i++)
        {
            *(ptcp_sync_parsed->masterSA + i) = *(sync_sblock + 6 + i);
            (pnHandle->pnPtcpConfig).currentPtcpStatus.masterSA[i] = *
                    (sync_sblock + 6 + i);

            *(Uint8 *)((pnHandle->pnPtcpConfig).pSyncMasterMac + i) = *(Uint8 *)(
                        sync_sblock + 6 + i);
        }

        *((pnHandle->pnPtcpConfig).pSyncInitFlag) = syncInitFlag | (1 << 1);
    }

    for(i = 0; i < 16; i++)
    {
        *(ptcp_sync_parsed->subdomainUUID + i) = *(sync_sblock + 12 + i);
    }

    return 0;
}


#ifdef ENABLE_LATCH_SUPPORT
void PN_PTCP_latchIsrHandler(uintptr_t arg)
{
    uint64_t PTCP_time_tL;
    uint32_t tick_elapsed;
    int32_t time_elapsed ;
    PN_Handle pnHandle = (PN_Handle)arg;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    PN_clearPruIRQ(pruicssHwAttrs, LATCH0_EVENT);


    /* Timing parameters of latch event and latch isr */
    HW_WR_REG32(pruicssHwAttrs->intcRegBase + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_STATUS_CLR_INDEX_REG, LATCH0_EVENT);
    (pnHandle->pnPtcpConfig).g_Latch.IEP_count_tL_R    = HW_RD_REG32(
                pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CAPR6_REG0);
    (pnHandle->pnPtcpConfig).g_Latch.IEP_count_tL_F    = HW_RD_REG32(
                pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CAPF6_REG0);


    /*TODO: Review this*/
    /*Redundant timer */
    // Types_Timestamp64 result;
    // Timestamp_get64(&result);
    // (pnHandle->pnPtcpConfig).g_Latch.tick_L = (uint64_t)result.lo;
    // (pnHandle->pnPtcpConfig).g_Latch.tick_L |= ((uint64_t)result.hi << 32);
    (pnHandle->pnPtcpConfig).g_Latch.tick_L = ClockP_getTimeUsec();

    tick_elapsed = (((pnHandle->pnPtcpConfig).g_Latch.tick_L -
                              (pnHandle->pnPtcpConfig).g_Latch.tick_S) * 10) / 6;

    /* Absolute time calculation - start*/
    int32_t IEP_diff = (pnHandle->pnPtcpConfig).g_Latch.IEP_count_tL_R -
                       (pnHandle->pnPtcpConfig).g_Latch.IEP_count_tS;

    /* Find cycle counter difference so that calculated time_elapsed  matches with system clock difference*/
    int n_CycCntr, Curr_min_diff = SYNC_INTERVAL, CycCntr_elapsed = -1;
    int SCF = HW_RD_REG16(pruicssHwAttrs->pru0DramBase + RTC_SCF_OFFSET);

    for(n_CycCntr = 0; n_CycCntr <= 960; n_CycCntr = n_CycCntr + SCF)
    {
        int t_elap, diff;
        t_elap = IEP_diff + (n_CycCntr * 31250);

        if(t_elap < 0)
        {
            t_elap = 0;
        }

        if(t_elap > tick_elapsed)
        {
            diff = t_elap - tick_elapsed;
        }

        else
        {
            diff = tick_elapsed - t_elap;
        }

        if(diff <= Curr_min_diff)
        {
            CycCntr_elapsed = n_CycCntr;
            Curr_min_diff = diff;
        }
    }

    time_elapsed = IEP_diff + (CycCntr_elapsed * 31250);

    /* Add time_elapsed to PTCP time got from Sync frame */
    PTCP_time_tL = ((uint64_t)((pnHandle->pnPtcpConfig).g_Latch.TorgSec) *
                             1000000000) + (pnHandle->pnPtcpConfig).g_Latch.TorgNsec +
                            (pnHandle->pnPtcpConfig).g_Latch.TDelay + time_elapsed;

    /* Copy to Timestamp structure */
    (pnHandle->pnPtcpConfig).g_T1.Status       = 0;
    (pnHandle->pnPtcpConfig).g_T1.SecondsHigh  = 0;
    (pnHandle->pnPtcpConfig).g_T1.SecondsLow   = PTCP_time_tL / 1000000000;
    (pnHandle->pnPtcpConfig).g_T1.Nanoseconds  = PTCP_time_tL % 1000000000;

#ifdef LATCH_DEBUG

    g_dbg_latch[index_dbg_api] = g_Latch;
    g_dbg_T1[index_dbg_api] = g_T1;

    if(index_dbg_api >= 1)
    {
        td1[index_dbg_api] = g_dbg_T1[index_dbg_api].Nanoseconds -
                             g_dbg_T1[index_dbg_api - 1].Nanoseconds;
        tickdiff[index_dbg_api] = ((g_dbg_latch[index_dbg_api].tick_L -
                                    g_dbg_latch[index_dbg_api - 1].tick_L) * 10) / 6;
    }

    index_dbg_api++;

    if(index_dbg_api >= LDBG_SZ)
    {
        index_dbg_api = 0;
    }
#endif

    return;
}
#endif
/* Absolute time API
 Params:
     LatchEn:
     1 - Triggered by latch input
     0 - Triggered by function call itself

     p_PNIO_TimeStamp: pointer to structure PNIO_TimeStamp

 Return val:
      0  - if device is in sync with PLC
      -1 - if out of sync
 */
int32_t PN_PTCP_getAbsoluteTime(PN_Handle pnHandle,
                                PNIO_TimeStamp *p_PNIO_TimeStamp, int32_t LatchEn)
{
    int32_t time_elapsed;
    uint32_t tick_elapsed;
    uint64_t PTCP_time_tL;
    int32_t IEP_diff;
    int SCF ;
    int t_elap, diff;
    int n_CycCntr, Curr_min_diff, CycCntr_elapsed;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    if(0 == LatchEn)
    {

        (pnHandle->pnPtcpConfig).g_Latch.IEP_count_fn  = HW_RD_REG32(
                    pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0);

        /*TODO: Review this*/
        /*Redundant timer */
        // OSAL_Timestamp_get64(&result);
        // (pnHandle->pnPtcpConfig).g_Latch.tick_fn = (uint64_t)result.lo;
        // (pnHandle->pnPtcpConfig).g_Latch.tick_fn |= ((uint64_t)result.hi << 32);

        (pnHandle->pnPtcpConfig).g_Latch.tick_fn = ClockP_getTimeUsec();

        tick_elapsed = (((pnHandle->pnPtcpConfig).g_Latch.tick_fn -
                                  (pnHandle->pnPtcpConfig).g_Latch.tick_S) * 10) / 6;

        /* Absolute time calculation - start*/
         IEP_diff = (pnHandle->pnPtcpConfig).g_Latch.IEP_count_fn -
                           (pnHandle->pnPtcpConfig).g_Latch.IEP_count_tS;

        /* Find cycle counter difference so that calculated time_elapsed matches with system clock difference*/
        Curr_min_diff = SYNC_INTERVAL;
        CycCntr_elapsed = -1;
        SCF = HW_RD_REG16(pruicssHwAttrs->pru0DramBase + RTC_SCF_OFFSET);

        for(n_CycCntr = 0; n_CycCntr <= 960; n_CycCntr = n_CycCntr + SCF)
        {
            t_elap = IEP_diff + (n_CycCntr * 31250);

            if(t_elap < 0)
            {
                t_elap = 0;
            }

            if(t_elap > tick_elapsed)
            {
                diff = t_elap - tick_elapsed;
            }

            else
            {
                diff = tick_elapsed - t_elap;
            }

            if(diff <= Curr_min_diff)
            {
                CycCntr_elapsed = n_CycCntr;
                Curr_min_diff = diff;
            }
        }

        time_elapsed = IEP_diff + (CycCntr_elapsed * 31250);

        /* Add time_elapsed to PTCP time got from Sync frame */
        PTCP_time_tL = ((uint64_t)((pnHandle->pnPtcpConfig).g_Latch.TorgSec) *
                                 1000000000) + (pnHandle->pnPtcpConfig).g_Latch.TorgNsec +
                                (pnHandle->pnPtcpConfig).g_Latch.TDelay + time_elapsed;

        (pnHandle->pnPtcpConfig).g_T1.Status = 0;
        (pnHandle->pnPtcpConfig).g_T1.SecondsHigh = 0;
        (pnHandle->pnPtcpConfig).g_T1.SecondsLow = PTCP_time_tL / 1000000000;
        (pnHandle->pnPtcpConfig).g_T1.Nanoseconds = PTCP_time_tL % 1000000000;

#ifdef LATCH_DEBUG
        g_dbg_latch[index_dbg_api] = g_Latch;
        g_dbg_T1[index_dbg_api] = g_T1;

        if(index_dbg_api >= 1)
        {
            td1[index_dbg_api] = g_dbg_T1[index_dbg_api].Nanoseconds -
                                 g_dbg_T1[index_dbg_api - 1].Nanoseconds;
            tickdiff[index_dbg_api] = ((g_dbg_latch[index_dbg_api].tick_fn -
                                        g_dbg_latch[index_dbg_api - 1].tick_fn) * 10) / 6;
        }

        index_dbg_api++;

        if(index_dbg_api >= LDBG_SZ)
        {
            index_dbg_api = 0;
        }

#endif

    }

    /* Output time stamp and status */
    p_PNIO_TimeStamp->Nanoseconds = (pnHandle->pnPtcpConfig).g_T1.Nanoseconds;
    p_PNIO_TimeStamp->SecondsHigh = (pnHandle->pnPtcpConfig).g_T1.SecondsHigh;
    p_PNIO_TimeStamp->SecondsLow = (pnHandle->pnPtcpConfig).g_T1.SecondsLow;
    p_PNIO_TimeStamp->Status = (pnHandle->pnPtcpConfig).g_T1.Status;

    if((pnHandle->pnPtcpConfig).currentPtcpStatus.syncState == IN_SYNC)
    {
        return 0;
    }

    else
    {
        return -1;
    }
}

#ifdef ENABLE_LATCH_SUPPORT
/* Setup latch ISR*/
int32_t PN_PTCP_latchSetupIsr(PN_Handle pnHandle)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);
    PN_IntAttrs *latchIntConfig = &((pnHandle->pnIntConfig).latchIntConfig);
    HwiP_Params hwiParams;
    uint32_t status = SystemP_FAILURE;
    uint32_t regVal;

    /*Set capture to first event mode*/
    regVal = HW_RD_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CAP_CFG_REG);
    regVal |= 0x000000C0;
    HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CAP_CFG_REG, regVal);

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = latchIntConfig->coreIntNum;
    hwiParams.callback = (HwiP_FxnCallback)(latchIntConfig->isrFnPtr);
    hwiParams.args = (void *)(latchIntConfig->args);
    hwiParams.priority = latchIntConfig->intPrio;
    /* TODO : Review this line. This feature is not available in Hwi DPL*/
    /* hwiParams.maskSetting = Hwi_MaskingOption_SELF; */

    status = HwiP_construct(&(latchIntConfig->interruptObject), &hwiParams);
    DebugP_assert(status == SystemP_SUCCESS);

    return status;
}

/* Setup latch ISR - needs emac handle*/
void PN_PTCP_latchInit(PN_Handle pnHandle)
{
    PN_PTCP_latchSetupIsr(pnHandle);
}
#endif
int32_t PN_PTCP_setupIsr(PN_Handle pnHandle)
{
    PN_IntAttrs *ptcpIntConfig = &((pnHandle->pnIntConfig).ptcpIntConfig);
    HwiP_Params hwiParams;
    uint32_t status = SystemP_FAILURE;

    HwiP_Params_init(&hwiParams);

    /* setup Sync ISR*/
    hwiParams.intNum = ptcpIntConfig->coreIntNum;
    hwiParams.callback = (HwiP_FxnCallback)(ptcpIntConfig->isrFnPtr);
    hwiParams.args = (void *)(ptcpIntConfig->args);
    hwiParams.priority = ptcpIntConfig->intPrio;

    status = HwiP_construct(&(ptcpIntConfig->interruptObject), &hwiParams);

    DebugP_assert(status == SystemP_SUCCESS);

    return 0;
}

void PN_PTCP_timerHandler(void* arg)
{
    PN_Handle pnHandle = (PN_Handle)arg;
    PN_PTCP_syncHandling(pnHandle);
}


void PN_PTCP_isrHandler(void* arg)
{
    PN_Handle pnHandle = (PN_Handle)arg;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    PN_IntAttrs *ptcpIntConfig = &((pnHandle->pnIntConfig).ptcpIntConfig);

    PN_PTCP_syncHandling(pnHandle);

    if((pnHandle->pnPtcpConfig).cycleCtrInitPending == 0)      /*clear only if cycle initialization is done*/
    {
        PN_clearPruIRQ(pruicssHwAttrs, ptcpIntConfig->pruIntNum);
    }
    return;
}

int32_t PN_PTCP_enableIsr(PN_Handle pnHandle)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);
    PN_IntAttrs *ptcpIntConfig = &((pnHandle->pnIntConfig).ptcpIntConfig);
#ifdef ENABLE_LATCH_SUPPORT
    PN_IntAttrs *latchIntConfig = &((pnHandle->pnIntConfig).latchIntConfig);
#endif
    /* clear pending PRU ISR flags*/
    PN_clearPruIRQ(pruicssHwAttrs, ptcpIntConfig->pruIntNum);
    /*enable IRQ*/
    HwiP_enableInt(ptcpIntConfig->coreIntNum);

    PN_clearPruIRQ(pruicssHwAttrs, LATCH0_EVENT);

#ifdef ENABLE_LATCH_SUPPORT
    HwiP_enableInt(latchIntConfig->coreIntNum);
#endif

    return 0;
}

int32_t PN_PTCP_disableIsr(PN_Handle pnHandle)
{
    /*disable IRQ*/
    PN_IntAttrs *ptcpIntConfig = &((pnHandle->pnIntConfig).ptcpIntConfig);
    /*TODO: Review this*/
    HwiP_disableInt(ptcpIntConfig->coreIntNum);
    return 0;
}

void PN_PTCP_syncHandling(PN_Handle pnHandle)
{

    int32_t iep_counter = 0;
    int32_t adjDeltaT = 0;
    uint64_t ptcp_time;
    uint16_t    cycle_counter;
    uint32_t    current_iep_val, processing_delay, incoming_delay,
                raw_incoming_delay;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    uint32_t PN_PTCP_Cyclecounter_in_ISR;
    uint32_t PN_PTCP_Cyclecounter_in_Sync;

    /*TODO: Review this*/
    /* Absolute time API */
    // TypesP_Timestamp64 result;
    // OSAL_Timestamp_get64(&result);
    // pnHandle->pnPtcpConfig.g_Latch.tick_S = (uint64_t)result.lo;
    // pnHandle->pnPtcpConfig.g_Latch.tick_S |= ((uint64_t)result.hi << 32);
    pnHandle->pnPtcpConfig.g_Latch.tick_S = ClockP_getTimeUsec();

    //PN_IntAttrs *ptcpIntConfig = (pnHandle->pnIntConfig).ptcpIntConfig;
    uint8_t *temp = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                SYNC_CTRL_BYTE_OFFSET);
    uint8_t *pFupCtrlByte = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                        SYNC_W_FUP_CTRL_BYTE_OFFSET);
    uint32_t    *pFupDelay = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                                          SYNC_FUP_DELAY_OFFSET);

    uint8_t ctrlByte = *temp;

    if(ctrlByte != 0)
    {
        PN_PTCP_syncPreprocess(pnHandle, ctrlByte);
        *temp = 0;
    }

    if(*pFupCtrlByte == 1)
    {
        *pFupCtrlByte = 0;
        *((pnHandle->pnPtcpConfig).pSyncInDelayPlusLD) = (*((
                    pnHandle->pnPtcpConfig).pSyncInDelayPlusLD))
                + PN_PTCP_rotUint((uint32_t *)pFupDelay);
    }

    int32_t send_clock_factor = HW_RD_REG16(pruicssHwAttrs->pru0DramBase +
                                       RTC_SCF_OFFSET);
    uint32_t    syncRxSOF = *((pnHandle->pnPtcpConfig).pSyncRxSOF);
    uint32_t syncInDelayPlusLD = *((pnHandle->pnPtcpConfig).pSyncInDelayPlusLD);
    uint8_t smaFactor = (pnHandle->pnPtcpConfig).ptcpSyncFilterfactor;

#ifdef  SYNC_SYS_LOG
    uint16_t *pSyncSysLogInfo = (uint16_t *)((uint8_t *)syncSysLogFrame + 44);
    int32_t *pSyncSysLogInfo_32 = (int32_t *)((uint8_t *)syncSysLogFrame + 44);
#endif

    /* If cycle initialization didn't have for the first time due to boundary conditions, then initialization is done here*/
    /* assumption: interrupt latency less than (cycle time - 8 us)*/
    if((pnHandle->pnPtcpConfig).cycleCtrInitPending == 1)
    {
        current_iep_val = HW_RD_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0);

        /* 4 us boundary on both sides*/
        if((pnHandle->pnPtcpConfig).pnCyclePeriod - current_iep_val > 4000
                && current_iep_val > 4000)
        {

            /* Cycle counter update guideline */
            uint32_t curr_cycle_counter     = HW_RD_REG16((uint16_t *)(
                                                  pruicssHwAttrs->pru0DramBase
                                                  + PTCP_PM_CYCLE_COUNTER_OFFSET));
            uint32_t new_cycle_counter = (pnHandle->pnPtcpConfig).calculatedCycleCtr;
            uint32_t curr = curr_cycle_counter & 0x00007FFF;
            uint32_t new_cc = new_cycle_counter & 0x00007FFF;

            if(new_cc >= curr)
            {
                (pnHandle->pnPtcpConfig).calculatedCycleCtr = new_cycle_counter;
            }

            if(new_cc < curr)
            {
                (pnHandle->pnPtcpConfig).calculatedCycleCtr = new_cycle_counter ^ 0x00008000;
            }

            (pnHandle->pnPtcpConfig).cycleCtrInitPending = 0;
            *((pnHandle->pnPtcpConfig).pPmCycleCounter) =
                (pnHandle->pnPtcpConfig).calculatedCycleCtr;
            *((pnHandle->pnPtcpConfig).pPmPhaseCounter) = ((
                        pnHandle->pnPtcpConfig).calculatedCycleCtr / send_clock_factor) & 0x1FF;
            (pnHandle->pnPtcpConfig).phaseCtrChange = 1;

#ifdef SYNC_SYS_LOG
            syncSysLogFrame[42] = 0x43;
            syncSysLogFrame[43] = 0x43;
            *pSyncSysLogInfo = calculatedCycleCtr;
            *(pSyncSysLogInfo + 4) = 2;
            TxPacketOS(syncSysLogFrame, ICSS_EMAC_PORT_1, ICSS_EMAC_QUEUE1,
                       sizeof(syncSysLogFrame));
#endif
        }

        return;
    }

    PN_PTCP_parseSyncFields(pnHandle, &((pnHandle->pnPtcpConfig).deviceSyncInfo),
                            (uint8_t *)(pruicssHwAttrs->sharedDramBase + SYNC_SBLOCK_OFFSET));


    if((pnHandle->pnPtcpConfig).currentPtcpStatus.firstSyncRcv == 1
            && (pnHandle->pnPtcpConfig).initPmCycleCtrDone == 0)
    {
        /*cycle counter => 31.25*/
        /*cycle         => 31.25 * scf*/
        /*phase     => 31.25 * scf % 512 => cycle counter /scf % 512*/

        /*calculate cycle counter*/

        /* extract the fields from sync packet*/
        ptcp_time = PN_PTCP_rotUint((pnHandle->pnPtcpConfig).pSyncTorgSecs);
        ptcp_time *= 1000000000;
        ptcp_time += PN_PTCP_rotUint((pnHandle->pnPtcpConfig).pSyncTorgNs);

        /* take care of processing delay: interrupt latency, computation delay, etc.*/
        current_iep_val = HW_RD_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0);


        /* Difference in cycle counter used to calculate Sync to ISR latency. Handles multiple wraparounds.*/
        PN_PTCP_Cyclecounter_in_ISR = HW_RD_REG16(pruicssHwAttrs->pru0DramBase +
                                             RTC_CYCLE_COUNTER_OFFSET);
        PN_PTCP_Cyclecounter_in_Sync = HW_RD_REG16(pruicssHwAttrs->sharedDramBase +
                                              SYNC_CYCLE_COUNTER);

        processing_delay = (PN_PTCP_Cyclecounter_in_ISR - PN_PTCP_Cyclecounter_in_Sync)
                           * RTC_3125_CLK_CONST  - syncRxSOF + current_iep_val;


        /* take incoming delay: only consider the part which is multiple of pn_cycle_period*/
        raw_incoming_delay = syncInDelayPlusLD + processing_delay;
        incoming_delay = raw_incoming_delay / (pnHandle->pnPtcpConfig).pnCyclePeriod;
        incoming_delay *= (pnHandle->pnPtcpConfig).pnCyclePeriod;

        /* add incoming delay to ptcp_time*/
        ptcp_time += incoming_delay;

        /* calculate cycle counter*/
        cycle_counter = (ptcp_time / 31250) & 0xFFFF; /* % 65536*/

        /* take more accurate timestamp*/
        current_iep_val = HW_RD_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0);

        /* 4 us boundary on both sides => 4 us empirical value based on assumption that firmware updation of cycle counter will be done by 4 us.*/
        if((pnHandle->pnPtcpConfig).pnCyclePeriod - current_iep_val < 4000
                || current_iep_val <
                4000) /* less than 4 us; store the values and do adjustment later to avoid race condition*/
        {
            (pnHandle->pnPtcpConfig).cycleCtrInitPending = 1;

            if((raw_incoming_delay % (pnHandle->pnPtcpConfig).pnCyclePeriod) <
                    4000)   /* calculated value => already new value*/
            {
                (pnHandle->pnPtcpConfig).calculatedCycleCtr  = cycle_counter;
            }

            else
            {
                (pnHandle->pnPtcpConfig).calculatedCycleCtr  = cycle_counter +
                        send_clock_factor;
            }
        }

        else
        {
            /* Cycle counter update guideline */
            uint32_t curr_cycle_counter     = HW_RD_REG16((uint16_t *)(
                                                  pruicssHwAttrs->pru0DramBase
                                                  + PTCP_PM_CYCLE_COUNTER_OFFSET));
            uint32_t new_cycle_counter = cycle_counter;
            uint32_t curr = curr_cycle_counter & 0x00007FFF;
            uint32_t new_cc = new_cycle_counter & 0x00007FFF;

            if(new_cc >= curr)
            {
                cycle_counter = new_cycle_counter;
            }

            if(new_cc < curr)
            {
                cycle_counter = new_cycle_counter ^ 0x00008000;
            }

            *((pnHandle->pnPtcpConfig).pPmCycleCounter) =
                cycle_counter;               /*exit from here and come back here...before clearing..use flag just for updating...and update late*/
            *((pnHandle->pnPtcpConfig).pPmPhaseCounter) = (cycle_counter /
                    send_clock_factor) & 0x1FF;
            (pnHandle->pnPtcpConfig).phaseCtrChange = 1;
#ifdef SYNC_SYS_LOG
            syncSysLogFrame[42] = 0x43;
            syncSysLogFrame[43] = 0x43;
            *pSyncSysLogInfo = cycle_counter;
            *(pSyncSysLogInfo + 4) = 1;
            TxPacketOS(syncSysLogFrame, ICSS_EMAC_PORT_1, ICSS_EMAC_QUEUE1,
                       sizeof(syncSysLogFrame));
#endif
        }

        (pnHandle->pnPtcpConfig).initPmCycleCtrDone = 1;
    }

    /* Absolute time API */


    pnHandle->pnPtcpConfig.g_Latch.TorgSec             = PN_PTCP_rotUint((
                pnHandle->pnPtcpConfig).pSyncTorgSecs);
    pnHandle->pnPtcpConfig.g_Latch.TorgNsec            = PN_PTCP_rotUint((
                pnHandle->pnPtcpConfig).pSyncTorgNs);
    pnHandle->pnPtcpConfig.g_Latch.TDelay              = *((
                pnHandle->pnPtcpConfig).pSyncInDelayPlusLD);
    pnHandle->pnPtcpConfig.g_Latch.IEP_count_tS        = *((
                pnHandle->pnPtcpConfig).pSyncRxSOF);

    int32_t masterT = ((PN_PTCP_rotUint((pnHandle->pnPtcpConfig).pSyncTorgNs) %
                        (pnHandle->pnPtcpConfig).pnCyclePeriod) +
                       ((*((pnHandle->pnPtcpConfig).pSyncInDelayPlusLD)) %
                        (pnHandle->pnPtcpConfig).pnCyclePeriod)) % ((
                                    pnHandle->pnPtcpConfig).pnCyclePeriod);
    int32_t localT  = (*((pnHandle->pnPtcpConfig).pSyncRxSOF)) % ((
                          pnHandle->pnPtcpConfig).pnCyclePeriod);

    int32_t deltaT = masterT - localT;
    int32_t revDeltaT = (pnHandle->pnPtcpConfig).pnCyclePeriod - PN_PTCP_absVal(
                            deltaT);

    if(revDeltaT < PN_PTCP_absVal(deltaT)
            && (pnHandle->pnPtcpConfig).currentPtcpStatus.firstSyncRcv != 0)
    {
        if(deltaT > 0)
        {
            deltaT = (-1) * revDeltaT;
        }

        else
        {
            deltaT = revDeltaT;
        }
    }

    int32_t tempSum = 0;
    int32_t j = 0;

    for(j = 0; j < smaFactor; j++)
    {
        tempSum = tempSum + (pnHandle->pnPtcpConfig).syncDeltaTs[j];
    }

    int32_t smaDeltaT = tempSum / smaFactor;

    if((pnHandle->pnPtcpConfig).currentPtcpStatus.firstSyncRcv == 0)
    {
        /* check whether transmission is going on or not*/
        iep_counter = HW_RD_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0);
        HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0, PN_PTCP_modFunc(
                    iep_counter + deltaT + 900, (pnHandle->pnPtcpConfig).pnCyclePeriod));
        deltaT = 0;
        adjDeltaT = 0;
    }

    else
    {
#ifdef PTCP_ENABLE_FILTER

        if((pnHandle->pnPtcpConfig).numInSync < 1 *
                smaFactor)      /*1: let it stabilize, before using the filter*/
        {
            adjDeltaT = deltaT;
        }

        else if((pnHandle->pnPtcpConfig).numInSync == smaFactor)
        {
            adjDeltaT = deltaT + smaDeltaT;
        }

        else
        {
            adjDeltaT = (deltaT / smaFactor) + smaDeltaT;
        }

        if((pnHandle->pnPtcpConfig).numInSync !=
                smaFactor)     /*Special case: at the time of transition(non filter to filter)*/
        {
            (pnHandle->pnPtcpConfig).syncDeltaTs[(pnHandle->pnPtcpConfig).syncIndex] =
                adjDeltaT;
        }

#else
        adjDeltaT = deltaT;
#endif

        (pnHandle->pnPtcpConfig).numInSync++;
        (pnHandle->pnPtcpConfig).syncIndex = ((pnHandle->pnPtcpConfig).syncIndex + 1)
                                              % smaFactor;
    }

    /*calculation of ecap period
    adjust adjDeltaT ns     in      30 ms = 30 000 us
    adjust      1        ns     in      (30 000/adjDeltaT) us*/

    int32_t cmpValue = 0;

    cmpValue = (0x5 << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT) |
               CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CNT_ENABLE_MAX;

    if(adjDeltaT == 0)
    {
        cmpValue |= (0x5 << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT);
        PN_PTCP_syncIepAdjustment(pnHandle, SYNC_INTERVAL, cmpValue);
    }

    else
    {
        int32_t ecapPeriod = 5 * (SYNC_INTERVAL / PN_PTCP_absVal(
                                      adjDeltaT));

        if(adjDeltaT > 0)                      /* master is faster*/
        {
            cmpValue |= (0xA << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT);
            PN_PTCP_syncIepAdjustment(pnHandle, ecapPeriod, cmpValue);
        }

        else                                    /* slave is faster*/
        {
            cmpValue |= (0x0 << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT);
            PN_PTCP_syncIepAdjustment(pnHandle, ecapPeriod, cmpValue);
        }
    }

#ifdef  SYNC_ANALYSIS

    if(nResets < SYNC_ANALYSIS_N_RESETS - 1)
    {
        if(debugSyncIndex < SYNC_SINGLE_ITER_SIZE - 1)
        {
            ((syncAnalysis)[nResets]).deltaT[debugSyncIndex]    = deltaT;
            ((syncAnalysis)[nResets]).smaDeltaT[debugSyncIndex] = smaDeltaT;
            ((syncAnalysis)[nResets]).seqId[debugSyncIndex]     = PN_PTCP_rotUshort((
                        uint16_t *)pSeqId);
            ((syncAnalysis)[nResets]).cycleTime[debugSyncIndex] = pn_cycle_period;
        }
    }

#endif

    if((pnHandle->pnPtcpConfig).numInSync > 0)
    {
        if((pnHandle->pnPtcpConfig).pnPtcpDebugAttrs.debugSyncIndex <
                DEBUG_SYNC_N_ITER - 1)
        {

#ifdef SYNC_SYS_LOG_1
            syncSysLogFrame[42] = 0x44;
            syncSysLogFrame[43] = 0x54;
            *pSyncSysLogInfo_32 = deltaT;
            *(pSyncSysLogInfo_32 + 1) = masterT;
            *(pSyncSysLogInfo_32 + 2) = localT;
            TxPacketOS(syncSysLogFrame, ICSS_EMAC_PORT_1, ICSS_EMAC_QUEUE1,
                       sizeof(syncSysLogFrame));
#endif

#ifdef PTCP_DEBUG_SI
            (debugSync).syncDeltaT[debugSyncIndex]      = deltaT;
            (debugSync).syncSmaDeltaT[debugSyncIndex]   = smaDeltaT;

            (debugSync).syncTorgT[debugSyncIndex]           = PN_PTCP_rotUint(pSyncTorgNs);
            (debugSync).syncInDelayPlusLDT[debugSyncIndex]  = *pSyncInDelayPlusLD;

            (debugSync).syncMasterT[debugSyncIndex]     = masterT;
            (debugSync).syncLocalT[debugSyncIndex]      = localT;
#endif
            (pnHandle->pnPtcpConfig).pnPtcpDebugAttrs.debugSyncIndex++;
        }

#ifdef PTCP_DEBUG_LI

        if(deltaT > (pnHandle->pnPtcpConfig).pnPtcpDebugAttrs.maxDeltaT)
        {
            (pnHandle->pnPtcpConfig).pnPtcpDebugAttrs.maxDeltaT = deltaT;
            (pnHandle->pnPtcpConfig).maxSeqId = PN_PTCP_rotUshort((uint16_t *)(
                    pruicssHwAttrs.sharedDramBase + SYNC_SEQID_OFFSET));
        }

        if(deltaT < (pnHandle->pnPtcpConfig).pnPtcpDebugAttrs.minDeltaT)
        {
            (pnHandle->pnPtcpConfig).pnPtcpDebugAttrs.minDeltaT = deltaT;
            (pnHandle->pnPtcpConfig).minSeqId = PN_PTCP_rotUshort((uint16_t *)(
                    pruicssHwAttrs.sharedDramBase + SYNC_SEQID_OFFSET));
        }

        if(PN_PTCP_absVal(deltaT) > 1000)
        {
            (pnHandle->pnPtcpConfig).pnPtcpDebugAttrs.deltaTOutOfRange++;
        }

#endif

    }

    if(PN_PTCP_absVal(deltaT) >
            (pnHandle->pnPtcpConfig).currentPtcpStatus.syncPllWnd ||
            (pnHandle->pnPtcpConfig).currentPtcpStatus.firstSyncRcv == 0 ||
            (pnHandle->pnPtcpConfig).numInSync <= smaFactor +
            2) /*outside PLL window ; let filter get stabilized*/
    {
        (pnHandle->pnPtcpConfig).deviceSyncInfo.syncState = OUT_OF_SYNC;
    }

    else
    {
        (pnHandle->pnPtcpConfig).deviceSyncInfo.syncState = IN_SYNC;
    }

    /*update the flags*/
    (pnHandle->pnPtcpConfig).currentPtcpStatus.syncRcv = 1;
    (pnHandle->pnPtcpConfig).currentPtcpStatus.firstSyncRcv = 1;

    if((pnHandle->pnPtcpConfig).currentPtcpStatus.syncState !=
            (pnHandle->pnPtcpConfig).deviceSyncInfo.syncState ||
            (pnHandle->pnPtcpConfig).masterChange == 1)
    {
/*TODO: Review this*/
// #ifdef PTCP_SYNC_SIGNAL
//         GPIO_write(0, (pnHandle->pnPtcpConfig).deviceSyncInfo.syncState & 0x1);
// #endif
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_DEVICE_SYNC_STATUS_OFFSET,
            (pnHandle->pnPtcpConfig).deviceSyncInfo.syncState);

        (pnHandle->pnPtcpConfig).currentPtcpStatus.syncState =
            (pnHandle->pnPtcpConfig).deviceSyncInfo.syncState;

        if((pnHandle->pnPtcpConfig).ptcpSyncStatusCall != NULL)
        {
            (pnHandle->pnPtcpConfig).ptcpSyncStatusCall((
                        pnHandle->pnPtcpConfig).currentPtcpStatus.syncState, (uint32_t)NULL);
        }

#ifdef SYNC_SYS_LOG

        if(currentPtcpStatus.syncState == IN_SYNC)
        {
            syncSysLogFrame[42] = 0x49;
            syncSysLogFrame[43] = 0x4E;
            syncSysLogFrame[44] = 0x20;

        }

        else
        {
            syncSysLogFrame[42] = 0x4F;
            syncSysLogFrame[43] = 0x55;
            syncSysLogFrame[44] = 0x54;
        }

        TxPacket(syncSysLogFrame, ICSS_EMAC_PORT_1, QUEPRIO1, sizeof(syncSysLogFrame));
#endif

        if((pnHandle->pnPtcpConfig).masterChange == 1)
        {
            (pnHandle->pnPtcpConfig).masterChange = 0;
        }

#ifdef  SYNC_ANALYSIS

        if(nResets < SYNC_ANALYSIS_N_RESETS - 1)
        {
            if(nSyncTrans < SYNC_ANALYSIS_N_ITER - 1)
            {
                ((syncAnalysis)[nResets]).tState[nSyncTrans]        = deviceSyncInfo.syncState;
                ((syncAnalysis)[nResets]).tSeqId[nSyncTrans]        = PN_PTCP_rotUshort((
                            uint16_t *)pSeqId);
                nSyncTrans++;
            }
        }

#endif
    }
}

void PN_PTCP_syncIepAdjustment(PN_Handle pnHandle, int32_t ecapPeriod,
                               uint32_t compensation)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    if((pnHandle->pnPtcpConfig).ptcpEnableSlowCompensation)
    {
        HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, compensation);
        HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_SLOW_COMPEN_REG, ecapPeriod/5);
    }
}

/**
  * Get the current delay values
  *
  * @param portDelays Reference to structure for ptcp delay values
  * @param portNum  port no. for which delay values are requested
  */

void PN_PTCP_getDelayValues(PN_Handle pnHandle, ptcpPortDelayVal_t *portDelays,
                            uint8_t portNum)
{
    portNum = portNum - 1;

    portDelays->rxDelayLocal    =
        (pnHandle->pnPtcpConfig).portTimes[portNum].rxDelayLocal;
    portDelays->txDelayLocal    =
        (pnHandle->pnPtcpConfig).portTimes[portNum].txDelayLocal;
    portDelays->rxDelayRemote   =
        (pnHandle->pnPtcpConfig).portTimes[portNum].rxDelayRemote;
    portDelays->txDelayRemote   =
        (pnHandle->pnPtcpConfig).portTimes[portNum].txDelayRemote;
    portDelays->cableDelay      =
        (pnHandle->pnPtcpConfig).portTimes[portNum].cableDelay;
}

void PN_PTCP_getLocalDelayValues(PN_Handle pnHandle, uint8_t portNum,
                                 uint32_t *outRxDelay, uint32_t *outTxDelay)
{
    portNum = portNum - 1;
    *outRxDelay  = (pnHandle->pnPtcpConfig).portTimes[portNum].rxDelayLocal;
    *outTxDelay  = (pnHandle->pnPtcpConfig).portTimes[portNum].txDelayLocal;
}

void PN_PTCP_getRemoteDelayValues(PN_Handle pnHandle, uint8_t portNum,
                                  uint32_t *outRxDelay, uint32_t *outTxDelay)
{
    portNum = portNum - 1;
    *outRxDelay = (pnHandle->pnPtcpConfig).portTimes[portNum].rxDelayRemote;
    *outTxDelay = (pnHandle->pnPtcpConfig).portTimes[portNum].txDelayRemote;
}

void PN_PTCP_getSyncInfo(PN_Handle pnHandle, ptcpSyncInfo_t *syncInfo)
{
    /*copy the values from internal sync info structure to the application structure*/
    syncInfo->syncState = (pnHandle->pnPtcpConfig).deviceSyncInfo.syncState;
    memcpy((void *)(&(syncInfo->masterSA)),
           (void *)(&((pnHandle->pnPtcpConfig).deviceSyncInfo.masterSA)), 6);
    memcpy((void *)(&(syncInfo->subdomainUUID)),
           (void *)(&((pnHandle->pnPtcpConfig).deviceSyncInfo.subdomainUUID)), 16);
}

void PN_PTCP_getSyncMasterAddress(PN_Handle pnHandle, uint8_t *addr)
{
    /*copy the values from internal sync info structure to the application structure*/
    memcpy((void *)addr, (void *)(&((
                                        pnHandle->pnPtcpConfig).deviceSyncInfo.masterSA)), 6);
}

void PN_PTCP_syncPreprocess(PN_Handle pnHandle, uint8_t ctrlByte)
{
    uint8_t    *syncPacket;
    uint32_t *pSyncSOF;
    uint8_t    *pDelay1nsByte;
    uint32_t inDelay, lineDelay = 0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    if(ctrlByte == 1)
    {
        syncPacket      = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                      SYNC_SF_BUF_OFFSET_P1);
        pSyncSOF        = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                                       SYNC_RX_SOF_OFFSET_P1);
    }

    else if(ctrlByte == 2)
    {
        syncPacket      = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                                      SYNC_SF_BUF_OFFSET_P2);
        pSyncSOF        = (uint32_t *)(pruicssHwAttrs->sharedDramBase +
                                       SYNC_RX_SOF_OFFSET_P2);
    }

    else
    {
        return;
    }

    /* Parse sync packet*/
    pDelay1nsByte           = (uint8_t *)(syncPacket + 30);
    *((pnHandle->pnPtcpConfig).pSyncTorgSecs)          = *((uint32_t *)(
                syncPacket + 64));
    *((pnHandle->pnPtcpConfig).pSyncTorgNs)            = *((uint32_t *)(
                syncPacket + 68));
    *((pnHandle->pnPtcpConfig).pSyncRxSOF)         = (*pSyncSOF) +
            640;     /*for s/f, still SFD is taken in firmware, so from SOF to SFD*/

    /* Parse the stored sync packet and calculate total delay*/
    inDelay = 10 * (PN_PTCP_rotUint((uint32_t *)(syncPacket + 24))) +
              PN_PTCP_rotUint((uint32_t *)(syncPacket + 32));
    inDelay += (uint8_t)(*pDelay1nsByte);

    if(ctrlByte == 1)
    {
        lineDelay = *((pnHandle->pnPtcpConfig).devicePortOffsets[ICSS_EMAC_PORT_1 -
                      1].pSmaLineDelay);
    }

    if(ctrlByte == 2)
    {
        lineDelay = *((pnHandle->pnPtcpConfig).devicePortOffsets[ICSS_EMAC_PORT_2 -
                      1].pSmaLineDelay);
    }

    *((pnHandle->pnPtcpConfig).pSyncInDelayPlusLD) = inDelay + lineDelay;

    memcpy((void *)((pruicssHwAttrs->sharedDramBase + SYNC_SBLOCK_OFFSET)),
           (void *)(syncPacket + 32), 32);
}

int32_t PN_PTCP_absVal(int32_t num)
{
    if(num < 0)
    {
        return (-1) * num;
    }

    else
    {
        return        num;
    }
}

void PN_PTCP_setPllWindow(PN_Handle pnHandle, uint32_t pllWindowSize)
{
    (pnHandle->pnPtcpConfig).currentPtcpStatus.syncPllWnd = pllWindowSize;
}

void PN_PTCP_setSyncTimeoutFactor(PN_Handle pnHandle,
                                  uint32_t syncTimeoutFactor)
{
    (pnHandle->pnPtcpConfig).currentPtcpStatus.syncTimeoutFactor =
        syncTimeoutFactor;
}

void PN_PTCP_setTakeoverTimeoutFactor(PN_Handle pnHandle,
                                      uint32_t takeoverTimeoutFactor)
{
    (pnHandle->pnPtcpConfig).currentPtcpStatus.takeoverTimeoutFactor =
        takeoverTimeoutFactor;
}

void PN_PTCP_syncTimeoutMonitor(PN_Handle pnHandle)
{
    uint8_t syncInitFlag;

    while(1)
    {
        syncInitFlag = *((pnHandle->pnPtcpConfig).pSyncInitFlag);

        PN_PTCP_taskSleep(30 + 1);

        /*put some kind of data synchronization mechanism or lock to avoid corruption
        check whether first sync has been received or not*/
        if((pnHandle->pnPtcpConfig).currentPtcpStatus.firstSyncRcv)
        {

            if((pnHandle->pnPtcpConfig).currentPtcpStatus.syncRcv)
            {
                /* sync received in this cycle of 30 ms*/
                (pnHandle->pnPtcpConfig).currentPtcpStatus.nSyncMissed = 0;
                (pnHandle->pnPtcpConfig).currentPtcpStatus.syncRcv = 0;
            }

            else
            {
                /*sync not received in this cycle of 30 ms*/
                ((pnHandle->pnPtcpConfig).currentPtcpStatus.nSyncMissed)++;
#ifdef DEBUG_SYNC_EVENTS
                (pnHandle->pnPtcpConfig.pnPtcpDebugAttrs).syncmissCounter++;
#endif

                if((pnHandle->pnPtcpConfig).currentPtcpStatus.nSyncMissed ==
                        (pnHandle->pnPtcpConfig).currentPtcpStatus.takeoverTimeoutFactor)
                {
                    (pnHandle->pnPtcpConfig).masterChange = 1;
                    *((pnHandle->pnPtcpConfig).pSyncInitFlag) = syncInitFlag & (~(1 << 1));

                    if((pnHandle->pnPtcpConfig).ptcpSyncStatusCall != NULL)
                    {
                        (pnHandle->pnPtcpConfig).ptcpSyncStatusCall(TAKEOVER_TIMEOUT, (uint32_t)NULL);
                    }
                }

                if((pnHandle->pnPtcpConfig).currentPtcpStatus.nSyncMissed ==
                        (pnHandle->pnPtcpConfig).currentPtcpStatus.syncTimeoutFactor)
                {
                    PN_PTCP_reset(pnHandle);
                    (pnHandle->pnPtcpConfig).masterChange = 0;

                    if((pnHandle->pnPtcpConfig).ptcpSyncStatusCall != NULL)
                    {
                        (pnHandle->pnPtcpConfig).ptcpSyncStatusCall(SYNC_TIMEOUT, (uint32_t)NULL);
                    }

                }
            }
        }
    }
}

void PN_PTCP_setSyncUUID(PN_Handle pnHandle, uint8_t *subdomainUUID)
{
    uint8_t syncInitFlag = *((pnHandle->pnPtcpConfig).pSyncInitFlag);

    if(subdomainUUID == NULL)
    {
        return;
    }

    memcpy((void *)(&((pnHandle->pnPtcpConfig).currentPtcpStatus.subdomainUUID)),
           (void *)subdomainUUID, 16);
    memcpy((void *)((pnHandle->pnPtcpConfig).pSubDomainUUID),
           (void *)subdomainUUID, 16);

    *((pnHandle->pnPtcpConfig).pSyncInitFlag) = syncInitFlag | 1;

    /*Notify the stack about current sync status*/
    if((pnHandle->pnPtcpConfig).ptcpSyncStatusCall != NULL)
    {
        (pnHandle->pnPtcpConfig).ptcpSyncStatusCall((
                    pnHandle->pnPtcpConfig).currentPtcpStatus.syncState, (uint32_t)NULL);
    }
}


void PN_PTCP_taskSleep(uint32_t mSec)
{
    /*TODO: Review this*/
    // uint32_t    ticks = 0;
    // ticks = ((mSec * 1000) / ClockP_getTickPeriod()) + 1;
    // TaskP_sleep(ticks);
    ClockP_usleep(mSec * 1000);
}

void PN_PTCP_configureSync0Pin(PN_Handle pnHandle)
{
    uint32_t iepCmpCfg = 0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    /*enable cmp1 : t2 bit of cfg registers*/

    iepCmpCfg = HW_RD_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);
    iepCmpCfg = iepCmpCfg | 0x4;
    HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, iepCmpCfg);
    /*program cmp1 reg with period, used for sync0 signal generation: 10 us*/
    HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0,
        (pnHandle->pnPtcpConfig).ptcpSync0PinStart);

    /*configure the pulse width for sync0: 5000+1 cycles i.e. 25us*/
    HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG,
        (pnHandle->pnPtcpConfig).ptcpSync0PinPulseWidth);


}

void PN_PTCP_configureDelayMeasurement(PN_Handle pnHandle, uint8_t portNum,
                                       ptcpPortStatus_t state)
{
    portNum = portNum - 1;
    (pnHandle->pnPtcpConfig).currentPtcpStatus.cDelayEnable[portNum] = state;
}

int32_t PN_PTCP_modFunc(int32_t num, uint32_t mod)
{
    if(num >= 0)
    {
        return (num % mod);
    }

    else
    {
        while(num < 0)
        {
            num += mod;
        }

        return num;
    }
}

#endif  /*PTCP_SUPPORT*/

void PN_PTCP_configureSyncFwd(PRUICSS_HwAttrs const *pruicssHwAttrs, ptcpPortStatus_t state)
{
    /*configure the sync forwarding*/
    uint8_t *pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase
                                  + SYNC_FWD_ENABLED_OFFSET);
    *pTemp8 = state;
}
void PN_PTCP_configureDelayResp(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t portNum, ptcpPortStatus_t state)
{
    uint8_t *pTemp8;
    portNum = portNum - 1;

    if(portNum == 0)
    {
        pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                             P1_DELAY_RESP_CTRL_OFFSET);
    }

    else if(portNum == 1)
    {
        pTemp8 = (uint8_t *)(pruicssHwAttrs->sharedDramBase +
                             P2_DELAY_RESP_CTRL_OFFSET);
    }

    else
    {
        return;
    }

    *pTemp8 = state;
}
