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
 * \file timeSync_ptp_init_priv.c
 *
 * \brief This file contains functions related to initialization of
 *        PTP stack.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <math.h>

#include "timeSync.h"
#include "timeSync_tools.h"

#include "timeSync_ptp.h"
#include "timeSync_ptp_priv.h"
#include "timeSync_ptp_init_priv.h"
#include "timeSync_ptp_osal_priv.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* TimeSync driver static buffers */
uint8_t tsSyncTxBufMem[TIMESYNC_PTP_SYNC_BUF_ANNEX_E_SIZE];
uint8_t tsFollowUpTxBufMem[TIMESYNC_PTP_MAX_PORTS_SUPPORTED][TIMESYNC_PTP_FOLLOW_UP_BUF_ANNEX_E_SIZE];
uint8_t tsAnnounceTxBufMem[TIMESYNC_PTP_ANNOUNCE_BUF_ANNEX_E_SIZE];
uint8_t tsPDelayReqBufRx[TIMESYNC_PTP_MAX_PORTS_SUPPORTED][TIMESYNC_PTP_PDELAY_BUF_SIZE];
uint8_t tsPDelayResBufRx[TIMESYNC_PTP_MAX_PORTS_SUPPORTED][TIMESYNC_PTP_PDELAY_BUF_SIZE];
uint8_t tsPDelayResBufFlwUpRx[TIMESYNC_PTP_MAX_PORTS_SUPPORTED][TIMESYNC_PTP_PDELAY_BUF_SIZE];
uint8_t tsPDelayReqBufTx[TIMESYNC_PTP_MAX_PORTS_SUPPORTED][TIMESYNC_PTP_PDELAY_BUF_SIZE];
uint8_t tsPDelayResBufTx[TIMESYNC_PTP_MAX_PORTS_SUPPORTED][TIMESYNC_PTP_PDELAY_BUF_SIZE];
uint8_t tsPDelayResBufFlwUpTx[TIMESYNC_PTP_MAX_PORTS_SUPPORTED][TIMESYNC_PTP_PDELAY_BUF_SIZE];
uint8_t tsDelayReqBufTx[TIMESYNC_PTP_DELAY_REQ_BUF_SIZE];

/**
 * @internal
 * @def PTP Delay Request Frame
 *      Global which contains the packet data
 */
const uint8_t ptpDelayReqPacket[] =
{
    0x01, 0x00, 0x5E, 0x00, 0x01, 0x81, 0x00, 0x00,
    0xBC, 0xCA, 0x8E, 0xF7, 0x08, 0x00, 0x45, 0xEC,
    0x00, 0x48, 0x01, 0x3B, 0x00, 0x00, 0x01, 0x11,
    0x14, 0x23, 0xC0, 0xA8, 0x01, 0x32, 0xE0, 0x00,
    0x01, 0x81, 0x01, 0x3F, 0x01, 0x3F, 0x00, 0x34,
    0x00, 0x00, 0x01, 0x02, 0x00, 0x2C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xBC, 0xFF, 0xFE, 0xCA, 0x8E, 0xF7, 0x00, 0x01,
    0x00, 0x2A, 0x01, 0x7F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/**
 * @internal
 * @def PTP PDelay Request Frame
 *      Global which contains the packet data
 */
const uint8_t ptpPDelayReqPacket[] =
{
    0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e, 0x00, 0x80,
    0x63, 0x00, 0x09, 0xba, 0x88, 0xf7, 0x02, 0x02,
    0x00, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x63, 0xff, 0xff, 0x00,
    0x09, 0xba, 0x00, 0x02, 0x04, 0x61, 0x05, 0x7f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

/**
 * @internal
 * @def PTP PDelay Response Frame for Port 1
 *      Global which contains the packet data
 */
const uint8_t ptpPDelayResPacket[] =
{
    0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e, 0x00, 0x80,
    0x63, 0x00, 0x09, 0xba, 0x88, 0xf7, 0x03, 0x02,
    0x00, 0x36, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x63, 0xff, 0xff, 0x00,
    0x09, 0xba, 0x00, 0x02, 0x04, 0x61, 0x05, 0x7f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

/**
 * @internal
 * @def PTP PDelay Response Follow Up Frame for Port 1
 *      Global which contains the packet data
 */
const uint8_t ptpPDelayResFlwUpPacket[] =
{
    0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e, 0x00, 0x80,
    0x63, 0x00, 0x09, 0xba, 0x88, 0xf7, 0x0a, 0x02,
    0x00, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x63, 0xff, 0xff, 0x00,
    0x09, 0xba, 0x00, 0x02, 0x04, 0x61, 0x05, 0x7f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

/**
 * @internal
 * @def Sample Layer 2 PTP Follow Up Frame
 *      Global which contains the packet data
 */
const uint8_t ptpFlwUpPacketL2[] =
{
    0x01, 0x1b, 0x19, 0x00, 0x00, 0x00, 0xec, 0x74,
    0xba, 0x1e, 0x13, 0x02, 0x88, 0xf7, 0x08, 0x02,
    0x00, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xec, 0x74, 0xba, 0xff, 0xfe, 0x1e,
    0x13, 0x02, 0x00, 0x03, 0xae, 0x2d, 0x02, 0x00,
    0x00, 0x00, 0x56, 0x89, 0xa6, 0x19, 0x37, 0x3b,
    0xdc, 0x24, 0x00, 0x00, 0x70, 0xf8, 0xed, 0xc5
};

/**
 * @internal
 * @def Sample Layer 3 (PTP over UDP/IPv4) PTP Follow Up Frame
 *      Global which contains the packet data
 */
const uint8_t ptpFlwUpPacketL3[] =
{
    0x01, 0x00, 0x5e, 0x00, 0x01, 0x81, 0xe4, 0x90,
    0x69, 0x9d, 0xd9, 0x25, 0x08, 0x00, 0x45, 0xbc,
    0x00, 0x48, 0xfa, 0x81, 0x00, 0x00, 0x01, 0x11,
    0x1b, 0x20, 0xc0, 0xa8, 0x01, 0x1e, 0xe0, 0x00,
    0x01, 0x81, 0x01, 0x40, 0x01, 0x40, 0x00, 0x34,
    0xed, 0x91, 0x08, 0x02, 0x00, 0x2c, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0x90,
    0x69, 0xff, 0xfe, 0x9d, 0xd9, 0x25, 0x00, 0x01,
    0x10, 0xdd, 0x02, 0x00, 0x00, 0x00, 0x35, 0x08,
    0x9b, 0x46, 0x27, 0xd5, 0x32, 0xa8
};

/**
 * @internal
 * @def Sample Layer 2 Sync frame
 *      Global which contains packet data
 */
const uint8_t ptpSyncPacketL2[] =
{
    0x01, 0x1b, 0x19, 0x00, 0x00, 0x00, 0xec, 0x74,
    0xba, 0x1e, 0x13, 0x02, 0x88, 0xf7, 0x00, 0x02,
    0x00, 0x2c, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xec, 0x74, 0xba, 0xff, 0xfe, 0x1e,
    0x13, 0x02, 0x00, 0x03, 0x08, 0x76, 0x00, 0x00,
    0x00, 0x00, 0x56, 0x86, 0x0a, 0x25, 0x02, 0x05,
    0xe9, 0xf0, 0x00, 0x00
};

/**
 * @internal
 * @def Sample Layer 3 (PTP over UDP/IPv4) Sync frame
 *      Global which contains packet data
 */
const uint8_t ptpSyncPacketL3[] =
{
    0x01, 0x00, 0x5e, 0x00, 0x01, 0x81, 0xe4, 0x90,
    0x69, 0x9d, 0xd9, 0x25, 0x08, 0x00, 0x45, 0xec,
    0x00, 0x48, 0xf9, 0x81, 0x00, 0x00, 0x01, 0x11,
    0x1b, 0xf0, 0xc0, 0xa8, 0x01, 0x1e, 0xe0, 0x00,
    0x01, 0x81, 0x01, 0x3f, 0x01, 0x3f, 0x00, 0x34,
    0x8b, 0xee, 0x00, 0x02, 0x00, 0x2c, 0x00, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0xf8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0x90,
    0x69, 0xff, 0xfe, 0x9d, 0xd9, 0x25, 0x00, 0x01,
    0x10, 0xdd, 0x00, 0x00, 0x00, 0x00, 0x35, 0x08,
    0x9b, 0x46, 0x27, 0xd2, 0x62, 0x58
};

/**
 * @internal
 * @def Sample Layer 2 Announce frame
 *      Global which contains packet data
 */
const uint8_t ptpAnnouncePacketL2[] =
{
    0x01, 0x1b, 0x19, 0x00, 0x00, 0x00, 0xec, 0x74,
    0xba, 0x1e, 0x13, 0x02, 0x88, 0xf7, 0x0b, 0x02,
    0x00, 0x40, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xec, 0x74, 0xba, 0xff, 0xfe, 0x1e,
    0x13, 0x02, 0x00, 0x03, 0x04, 0x37, 0x05, 0x01,
    0x00, 0x00, 0x56, 0x86, 0x0a, 0x25, 0x02, 0x05,
    0xe9, 0xf0, 0x00, 0x24, 0x00, 0x80, 0xf8, 0x31,
    0x62, 0x20, 0x80, 0xec, 0x74, 0xba, 0xff, 0xfe,
    0x1e, 0x13, 0x02, 0x00, 0x00, 0xa0
};

/**
 * @internal
 * @def Sample Layer 3 (PTP over UDP/IPv4) Announce frame
 *      Global which contains packet data
 */
const uint8_t ptpAnnouncePacketL3[] =
{
    0x01, 0x00, 0x5e, 0x00, 0x01, 0x81, 0xe4, 0x90,
    0x69, 0x9d, 0xd9, 0x25, 0x08, 0x00, 0x45, 0xbc,
    0x00, 0x5c, 0xfc, 0x81, 0x00, 0x00, 0x01, 0x11,
    0x19, 0x0c, 0xc0, 0xa8, 0x01, 0x1e, 0xe0, 0x00,
    0x01, 0x81, 0x01, 0x40, 0x01, 0x40, 0x00, 0x48,
    0xac, 0x7e, 0x0b, 0x02, 0x00, 0x40, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0x90,
    0x69, 0xff, 0xfe, 0x9d, 0xd9, 0x25, 0x00, 0x01,
    0x08, 0x6f, 0x05, 0x01, 0x00, 0x00, 0x35, 0x08,
    0x9b, 0x47, 0x27, 0xd0, 0xa7, 0x88, 0x00, 0x22,
    0x00, 0x80, 0xf8, 0xfe, 0xff, 0xff, 0x80, 0xe4,
    0x90, 0x69, 0xff, 0xfe, 0x9d, 0xd9, 0x25, 0x00,
    0x00, 0xa0
};

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void TimeSyncPtp_formatPtpFrames(TimeSyncPtp_Handle hTimeSyncPtp,
                                        uint8_t *ifMacID);

static void TimeSyncPtp_addIP(TimeSyncPtp_Handle enet_perctxt,
                              uint8_t *ipAddr);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void TimeSyncPtp_allocPktBuffer(TimeSyncPtp_Handle hTimeSyncPtp)
{
    uint8_t i = 0U;

    /*First configure sizes for allocation*/
    if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
    {
        hTimeSyncPtp->timeSyncBuff.syncBufSize  = TIMESYNC_PTP_SYNC_BUF_ANNEX_F_SIZE;
        hTimeSyncPtp->timeSyncBuff.flwUpBufSize = TIMESYNC_PTP_FOLLOW_UP_BUF_ANNEX_F_SIZE;
        hTimeSyncPtp->timeSyncBuff.announceBufSize = TIMESYNC_PTP_ANNOUNCE_BUF_ANNEX_F_SIZE;
    }

    /*Configure Sync, Announce, Follow Up and Management frame buffer*/
    hTimeSyncPtp->timeSyncBuff.syncTxBuf = &tsSyncTxBufMem[0];
    hTimeSyncPtp->timeSyncBuff.announceTxBuf = &tsAnnounceTxBufMem[0];
    for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
    {
        hTimeSyncPtp->timeSyncBuff.followUpTxBuf[i] = &tsFollowUpTxBufMem[i][0];
    }

    /*Configure P2P buffers. Both Rx and Tx*/
    if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
    {
        for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
        {
            hTimeSyncPtp->timeSyncBuff.pdelayReqRxBuf[i] = &tsPDelayReqBufRx[i][0];
            hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpRxBuf[i] = &tsPDelayResBufFlwUpRx[i][0];
            hTimeSyncPtp->timeSyncBuff.pdelayResRxBuf[i] = &tsPDelayResBufRx[i][0];
            hTimeSyncPtp->timeSyncBuff.pdelayReqTxBuf[i] = &tsPDelayReqBufTx[i][0];
            hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpTxBuf[i] = &tsPDelayResBufFlwUpTx[i][0];
            hTimeSyncPtp->timeSyncBuff.pdelayResTxBuf[i] = &tsPDelayResBufTx[i][0];
        }
    }
}

void TimeSyncPtp_unAllocPktBuffer(TimeSyncPtp_Handle hTimeSyncPtp)
{
    /* all the buffers are now statically allocated, no action to free */
    return;
}

void TimeSyncPtp_enable(TimeSyncPtp_Handle hTimeSyncPtp)
{
    hTimeSyncPtp->enabled = TRUE;
}

void TimeSyncPtp_disable(TimeSyncPtp_Handle hTimeSyncPtp)
{
    hTimeSyncPtp->enabled = FALSE;
}

void TimeSyncPtp_setDefaultValue(TimeSyncPtp_Handle hTimeSyncPtp)
{
    uint8_t i = 0U;
    double logInterval = 0;
    double normalization_factor = 0;

    normalization_factor = (double)1000 / (double)(hTimeSyncPtp->ptpConfig.tickPeriod);

    /*Initialize all structures*/
    for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
    {
        memset(&hTimeSyncPtp->syncParam[i], 0x0, sizeof(TimeSyncPtp_SyncParam));
        memset(&hTimeSyncPtp->pDelayParams[i], 0x0, sizeof(TimeSyncPtp_PeerDelayParams));
    }

    memset(&hTimeSyncPtp->tsSyntInfo, 0x0, sizeof(TimeSyncPtp_SyntInfo));
    memset(&hTimeSyncPtp->tsRunTimeVar, 0x0, sizeof(TimeSyncPtp_RuntimeVar));
    memset(&hTimeSyncPtp->offsetAlgo, 0x0, sizeof(TimeSyncPtp_OffsetStableAlgo));

    /*Calculate actual timer values from config values*/
    logInterval = hTimeSyncPtp->ptpConfig.logPDelReqPktInterval;
    hTimeSyncPtp->ptpConfig.pDelReqPktInterval = (uint32_t)(pow(2.0, logInterval) * 1000.0);
    /*Normalize as per tick period*/
    hTimeSyncPtp->ptpConfig.pDelReqPktInterval *= (normalization_factor);

    /*In microseconds. No normalization required*/
    logInterval = hTimeSyncPtp->ptpConfig.logSyncInterval;
    hTimeSyncPtp->ptpConfig.syncSendInterval = (uint32_t)(pow(2.0, logInterval) * (double)(TIME_SEC_TO_NS / 1000));

    /*Calculate actual value in miliseconds and normalize*/
    logInterval = hTimeSyncPtp->ptpConfig.logAnnounceSendInterval;
    hTimeSyncPtp->ptpConfig.announceSendInterval = (uint32_t)(pow(2.0, logInterval) * 1000.0);
    /*Normalize as per tick period*/
    hTimeSyncPtp->ptpConfig.announceSendInterval *= (normalization_factor);

    /*In miliseconds*/
    logInterval = hTimeSyncPtp->ptpConfig.logAnnounceRcptTimeoutInterval;
    hTimeSyncPtp->ptpConfig.announceRcptTimeoutInterval = (uint32_t)(pow(2.0, logInterval) * 1000.0);
    /*Normalize as per tick period*/
    hTimeSyncPtp->ptpConfig.announceRcptTimeoutInterval *= normalization_factor;

    /*Normalize pdelayBurstInterval*/
    hTimeSyncPtp->ptpConfig.pdelayBurstInterval *= normalization_factor;

    if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
    {
        hTimeSyncPtp->ptpConfig.frameOffset = TIMESYNC_PTP_ANNEX_D_ANNEX_F_DIFF;
    }

    hTimeSyncPtp->tsRunTimeVar.stateMachine = 0;

    hTimeSyncPtp->clockIdentity[0] = hTimeSyncPtp->ptpConfig.ifMacID[0];
    hTimeSyncPtp->clockIdentity[1] = hTimeSyncPtp->ptpConfig.ifMacID[1];
    hTimeSyncPtp->clockIdentity[2] = hTimeSyncPtp->ptpConfig.ifMacID[2];
    hTimeSyncPtp->clockIdentity[3] = 0xFF;
    hTimeSyncPtp->clockIdentity[4] = 0xFE;
    hTimeSyncPtp->clockIdentity[5] = hTimeSyncPtp->ptpConfig.ifMacID[3];
    hTimeSyncPtp->clockIdentity[6] = hTimeSyncPtp->ptpConfig.ifMacID[4];
    hTimeSyncPtp->clockIdentity[7] = hTimeSyncPtp->ptpConfig.ifMacID[5];

    /*copy local clock identity to gm clock identity*/
    memcpy(hTimeSyncPtp->ptpConfig.masterParams.gmIdentity,
           hTimeSyncPtp->clockIdentity,
           8);

    /*HSR specific flag, not applicable for other protocols. See definition
     * This configures Link local frames without a tag by default*/
    hTimeSyncPtp->ptpConfig.ll_has_hsrTag = 0;

    /****************PTP Runtime variables****************/
    /*Running offset is set to standard PPM for crystal on ICEv2*/
    hTimeSyncPtp->tsRunTimeVar.clockDrift = 10000;

    /*set to a high value*/
    hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval = 10000;

    /***************Packet Modification****************/
    TimeSyncPtp_formatPtpFrames(hTimeSyncPtp, hTimeSyncPtp->ptpConfig.ifMacID);
    //TimeSyncPtp_addIP(hTimeSyncPtp, &hTimeSyncPtp->ptpConfig.ipAddr[0U]);

    /*This is a fixed mapping. We need a gap of 1 index for syntonization
     * Refer to syntonization algo in design doc*/
    hTimeSyncPtp->syntIndexMap[0] = 1;
    hTimeSyncPtp->syntIndexMap[1] = 2;
    hTimeSyncPtp->syntIndexMap[2] = 0;

    /*Same as above*/
    hTimeSyncPtp->prevIndexMap[0] = 2;
    hTimeSyncPtp->prevIndexMap[1] = 0;
    hTimeSyncPtp->prevIndexMap[2] = 1;

    hTimeSyncPtp->stackParams.generalFrameFlag = 0;
    memset(&(hTimeSyncPtp->stackParams.ptpGeneralFrame), 0x0, 1518 * sizeof(uint8_t));

    /*Set RCF default value*/
    hTimeSyncPtp->tsSyntInfo.rcf = 1.0;

    /*Set NRR default value*/
    for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
    {
        hTimeSyncPtp->tsNrrInfo[i].nrr = 1.0;
    }

    hTimeSyncPtp->offsetAlgo.driftThreshold = TIMESYNC_PTP_OFFSET_STABLE_ALGO_THRESHOLD;
}

static void TimeSyncPtp_formatPtpFrames(TimeSyncPtp_Handle hTimeSyncPtp,
                                        uint8_t *ifMacID)
{
    uint8_t *bytePtr = NULL;
    uint8_t flag_byte0 = 0;
    uint8_t flag_byte1 = 0;
    uint8_t offset = hTimeSyncPtp->ptpConfig.frameOffset;
    uint8_t i = 0U;

    /*Initialize frame buffers with standard types*/
    if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
    {
        memcpy(hTimeSyncPtp->timeSyncBuff.syncTxBuf, ptpSyncPacketL2,
               hTimeSyncPtp->timeSyncBuff.syncBufSize);

        memcpy(hTimeSyncPtp->timeSyncBuff.announceTxBuf, ptpAnnouncePacketL2,
               hTimeSyncPtp->timeSyncBuff.announceBufSize);
    }

    for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
    {
        if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
        {
            memcpy(hTimeSyncPtp->timeSyncBuff.pdelayReqTxBuf[i], ptpPDelayReqPacket,
                   TIMESYNC_PTP_PDELAY_BUF_SIZE);

            memcpy(hTimeSyncPtp->timeSyncBuff.pdelayResTxBuf[i], ptpPDelayResPacket,
                   TIMESYNC_PTP_PDELAY_BUF_SIZE);

            memcpy(hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpTxBuf[i],
                   ptpPDelayResFlwUpPacket, TIMESYNC_PTP_PDELAY_BUF_SIZE);

            memcpy(hTimeSyncPtp->timeSyncBuff.followUpTxBuf[i], ptpFlwUpPacketL2,
                   hTimeSyncPtp->timeSyncBuff.flwUpBufSize);
        }
    }

    flag_byte0 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_ALTERNATE_MASTER_INDEX]
         << 0);
    flag_byte0 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_TWO_STEP_INDEX] <<
         1);
    flag_byte0 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_UNICAST] << 2);
    flag_byte0 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_PROFILE_SPECIFIC_1_INDEX]
         << 5);
    flag_byte0 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_PROFILE_SPECIFIC_2_INDEX]
         << 6);

    /*prepare the flag*/
    flag_byte1 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_LEAP_61_INDEX] << 0);
    flag_byte1 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_LEAP_59_INDEX] << 1);
    flag_byte1 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_UTC_REASONABLE_INDEX]
         << 2);
    flag_byte1 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_TIMESCALE_INDEX]
         << 3);
    flag_byte1 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_TIME_TRACEABLE_INDEX]
         << 4);
    flag_byte1 |=
        (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_FREQ_TRACEABLE_INDEX]
         << 5);

    /*--------------------Add source MAC ID--------------------*/

    /*Add MAC ID to Sync frame*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.syncTxBuf + TIMESYNC_PTP_SRC_MAC_OFFSET);
    memcpy(bytePtr, ifMacID, 6);

    /*Add MAC ID to Announce frame*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf +
                          TIMESYNC_PTP_SRC_MAC_OFFSET);
    memcpy(bytePtr, ifMacID, 6);

    /*--------------Add source mac----------------*/
    if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
    {
        for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
        {
            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayReqTxBuf[i] +
                                  TIMESYNC_PTP_SRC_MAC_OFFSET);
            memcpy(bytePtr, ifMacID, 6);

            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayResTxBuf[i] +
                                  TIMESYNC_PTP_SRC_MAC_OFFSET);
            memcpy(bytePtr, ifMacID, 6);

            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpTxBuf[i] +
                                  TIMESYNC_PTP_SRC_MAC_OFFSET);
            memcpy(bytePtr, ifMacID, 6);
        }
    }

    /*--------------------Add clock identity--------------------*/
    if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
    {
        for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
        {
            /*add clock identity to Pdelay Req frame*/
            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayReqTxBuf[i] +
                                  TIMESYNC_PTP_SRC_CLK_IDENTITY -
                                  offset);
            memcpy(bytePtr, hTimeSyncPtp->clockIdentity, 8);

            /*add clock identity to Pdelay Res frame*/
            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayResTxBuf[i] +
                                  TIMESYNC_PTP_SRC_CLK_IDENTITY -
                                  offset);
            memcpy(bytePtr, hTimeSyncPtp->clockIdentity, 8);

            /*add clock identity to Pdelay Res Flw Up frame*/
            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpTxBuf[i] +
                                  TIMESYNC_PTP_SRC_CLK_IDENTITY -
                                  offset);
            memcpy(bytePtr, hTimeSyncPtp->clockIdentity, 8);
        }
    }

    /*add clock identity to sync frame*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.syncTxBuf + TIMESYNC_PTP_SRC_CLK_IDENTITY - offset);
    memcpy(bytePtr, hTimeSyncPtp->clockIdentity, 8);

    /*add clock identity to announce frame*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_SRC_CLK_IDENTITY - offset);
    memcpy(bytePtr, hTimeSyncPtp->clockIdentity, 8);

    /*--------------------Add log message interval--------------------*/

    /*Add log message interval to sync and announce frame*/
    bytePtr  = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.syncTxBuf + TIMESYNC_PTP_LOG_MSG_PERIOD - offset);
    *bytePtr = hTimeSyncPtp->ptpConfig.logSyncInterval;

    /*add announce send interval to announce frame*/
    bytePtr  = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_LOG_MSG_PERIOD - offset);
    *bytePtr = hTimeSyncPtp->ptpConfig.logAnnounceSendInterval;

    /*-----------------------Add flags------------------------------*/
    /*--sync--*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.syncTxBuf + TIMESYNC_PTP_FLAG_OFFSET - offset);
    *(bytePtr) = flag_byte0 & TIMESYNC_PTP_SYNC_BYTE0_MASK;
    /*Other flags are applicable only to Announce*/
    *(bytePtr + 1) = 0;

    /*--Announce--*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_FLAG_OFFSET - offset);
    *(bytePtr) = flag_byte0 & TIMESYNC_PTP_ANNOUNCE_BYTE0_MASK;
    *(bytePtr + 1) = flag_byte1;

    if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
    {
        for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
        {
            /*--Pdelay Req--*/
            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayReqTxBuf[i] + TIMESYNC_PTP_FLAG_OFFSET - offset);
            *(bytePtr) = flag_byte0 & TIMESYNC_OTHER_FRAMES_BYTE0_MASK;
            *(bytePtr + 1) = 0;

            /*--Pdelay Res--*/

            /*Override two-step settings
             * DUT only supports 2-step Peer delay measurement*/
            flag_byte0 |= (1 << 1);
            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayResTxBuf[i] + TIMESYNC_PTP_FLAG_OFFSET - offset);
            *(bytePtr) = flag_byte0 & TIMESYNC_PTP_PDELAY_RESP_BYTE0_MASK;
            *(bytePtr + 1) = 0;

            /*--Pdelay Res Follow Up--*/
            bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpTxBuf[i] + TIMESYNC_PTP_FLAG_OFFSET - offset);
            *(bytePtr) = flag_byte0 & TIMESYNC_OTHER_FRAMES_BYTE0_MASK;
            *(bytePtr + 1) = 0;
        }
    }

    /*---------------------Add Announce frame params--------------------*/
    /*Priority 1*/
    bytePtr  = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_PRIORITY1_OFFSET - offset);
    *bytePtr = hTimeSyncPtp->ptpConfig.masterParams.priority1;
    /*Priority 2*/
    bytePtr  = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_PRIORITY2_OFFSET - offset);
    *bytePtr = hTimeSyncPtp->ptpConfig.masterParams.priority2;
    /*clock class*/
    bytePtr  = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_GM_CLK_CLASS_OFFSET - offset);
    *bytePtr = hTimeSyncPtp->ptpConfig.masterParams.clockClass;
    /*clock accuracy*/
    bytePtr  = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_GM_CLK_ACCU_OFFSET - offset);
    *bytePtr = hTimeSyncPtp->ptpConfig.masterParams.clockAccuracy;
    /*clock class*/
    bytePtr  = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_GM_CLK_CLASS_OFFSET - offset);
    *bytePtr = hTimeSyncPtp->ptpConfig.masterParams.clockClass;
    /*time source*/
    bytePtr  = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_TIME_SRC_OFFSET - offset);
    *bytePtr = hTimeSyncPtp->ptpConfig.masterParams.timeSource;
    /*steps removed*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_STEPS_REMOVED_OFFSET - offset);
    TimeSync_addHalfWord(bytePtr, hTimeSyncPtp->ptpConfig.masterParams.stepRemoved);
    /*clock variance*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_GM_CLK_VARIANCE_OFFSET - offset);
    TimeSync_addHalfWord(bytePtr, hTimeSyncPtp->ptpConfig.masterParams.clockVariance);

    /*UTC Offset*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_UTC_OFFSET - offset);
    TimeSync_addHalfWord(bytePtr, hTimeSyncPtp->ptpConfig.masterParams.UTCOffset);

    /*GM clock identity*/
    bytePtr = (uint8_t *)(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_GM_CLK_IDENTITY_OFFSET - offset);
    memcpy(bytePtr, hTimeSyncPtp->ptpConfig.masterParams.gmIdentity, 8);
}

static void TimeSyncPtp_addIP(TimeSyncPtp_Handle hTimeSyncPtp,
                              uint8_t *ipAddr)
{
    uint8_t i = 0U;

    /*For P2P this should not be called*/
    if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
    {
        return;
    }

    /*Destination IP 224.0.1.129*/
    hTimeSyncPtp->udpParams.dstIP = 0xE0000181;

    /*add Destination IP*/
    TimeSync_addWord(hTimeSyncPtp->timeSyncBuff.delayReqTxBuf + TIMESYNC_PTP_DST_IP_OFFSET,
                     hTimeSyncPtp->udpParams.dstIP);

    /*add Source IP for delay request*/
    memcpy(&hTimeSyncPtp->udpParams.srcIP, ipAddr, 4);
    TimeSync_addWord(hTimeSyncPtp->timeSyncBuff.delayReqTxBuf + TIMESYNC_PTP_SRC_IP_OFFSET,
                     hTimeSyncPtp->udpParams.srcIP);

    TimeSync_calcIpChecksum(hTimeSyncPtp->timeSyncBuff.delayReqTxBuf);

    /*add Source IP for Sync*/
    TimeSync_addWord(hTimeSyncPtp->timeSyncBuff.syncTxBuf + TIMESYNC_PTP_SRC_IP_OFFSET,
                     hTimeSyncPtp->udpParams.srcIP);
    TimeSync_calcIpChecksum(hTimeSyncPtp->timeSyncBuff.syncTxBuf);

    /*add Source IP for Announce*/
    TimeSync_addWord(hTimeSyncPtp->timeSyncBuff.announceTxBuf + TIMESYNC_PTP_SRC_IP_OFFSET,
                     hTimeSyncPtp->udpParams.srcIP);
    TimeSync_calcIpChecksum(hTimeSyncPtp->timeSyncBuff.announceTxBuf);

    for (i = 0; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
    {
        /*add Source IP for Follow Up*/
        TimeSync_addWord(hTimeSyncPtp->timeSyncBuff.followUpTxBuf[i] + TIMESYNC_PTP_SRC_IP_OFFSET, hTimeSyncPtp->udpParams.srcIP);
        TimeSync_calcIpChecksum(hTimeSyncPtp->timeSyncBuff.followUpTxBuf[i]);
    }
}
