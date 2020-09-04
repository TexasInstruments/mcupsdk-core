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
 * \file timeSync_ptp.c
 *
 * \brief This file contains the implementation of PTP stack functions.
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
#include <networking/enet/utils/include/enet_apputils.h>

#include "timeSync_ptp.h"
#include "timeSync_ptp_priv.h"
#include "timeSync_ptp_init_priv.h"
#include "timeSync_ptp_osal_priv.h"

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

static void TimeSyncPtp_setTimeSyncConfig(TimeSyncPtp_Handle hTimeSyncPtp,
                                          TimeSync_Config *pTimeSyncConfig);

static void TimeSyncPtp_stripVlanTag(TimeSyncPtp_Handle hTimeSyncPtp,
                                     uint8_t *pktBuffer,
                                     uint32_t *size);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

TimeSyncPtp_Obj gTimeSyncPtpObj;
int32_t min_offset = 1000;
int32_t max_offset = -1000;
uint32_t len = 0;

/**PTP MAC ID for comparison*/
uint8_t timeSyncMAC[6] = {0x1, 0x1b, 0x19, 0x0, 0x0, 0x0};
uint8_t linkLocalMAC[6] = {0x1, 0x80, 0xc2, 0x0, 0x0, 0xE};

TimeSyncPtp_TxNotifyMsg txNotifyMsg;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void TimeSyncPtp_setDefaultPtpConfig(TimeSyncPtp_Config *ptpConfig)
{
    int8_t i = 0;
    ptpConfig->nwDrvHandle = NULL;

    /* VLAN configuration */
    ptpConfig->vlanCfg.vlanType = TIMESYNC_VLAN_TYPE_NONE;
    ptpConfig->vlanCfg.iVlanTag = 0U;
    ptpConfig->vlanCfg.oVlanTag = 0U;

    /* Port configuration */
    ptpConfig->portMask = 0U;

    ptpConfig->hsrEnabled = FALSE;
    ptpConfig->ll_has_hsrTag = FALSE;

    /*Configure PTP. These variables must be configured before doing anything else*/
    ptpConfig->deviceMode = TIMESYNC_ORDINARY_CLOCK;
    ptpConfig->type = TIMESYNC_PTP_DELAY_P2P;
    ptpConfig->protocol = TIMESYNC_PROT_IEEE_802_3;
    ptpConfig->frameOffset = 0U;
    ptpConfig->tickPeriod = 1000U;
    ptpConfig->domainNumber[0] = 0U;
    ptpConfig->domainNumber[1] = 0U;

    ptpConfig->logAnnounceRcptTimeoutInterval = TIMESYNC_PTP_DEFAULT_ANNOUNCE_TIMEOUT_LOG_INTERVAL;
    ptpConfig->logAnnounceSendInterval = TIMESYNC_PTP_DEFAULT_ANNOUNCE_SEND_LOG_INTERVAL;
    ptpConfig->logPDelReqPktInterval = TIMESYNC_PTP_DEFAULT_PDELAY_REQ_LOG_INTERVAL;
    ptpConfig->logSyncInterval = TIMESYNC_PTP_DEFAULT_SYNC_SEND_LOG_INTERVAL;

    /*No asymmetry*/
    for (i = 0U; i < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; i++)
    {
        ptpConfig->asymmetryCorrection[i] = 0U;
    }

    /*3 frames sent in a burst*/
    ptpConfig->pdelayBurstNumPkts = 3U;
    /*gap between each frame is 100ms*/
    ptpConfig->pdelayBurstInterval = 200U;
    /*Register callback*/
    ptpConfig->syncLossNotifyFxn = NULL;

    /*Configure Master params*/
    ptpConfig->isMaster = 0;
    ptpConfig->masterParams.priority1 = TIMESYNC_PTP_DEFAULT_PRIO_1;
    ptpConfig->masterParams.priority2 = TIMESYNC_PTP_DEFAULT_PRIO_2;
    /*greater than 10s */
    ptpConfig->masterParams.clockAccuracy = TIMESYNC_PTP_DEFAULT_CLOCK_ACCURACY;
    ptpConfig->masterParams.clockClass = TIMESYNC_PTP_DEFAULT_CLOCK_CLASS;
    ptpConfig->masterParams.clockVariance = TIMESYNC_PTP_DEFAULT_CLOCK_VARIANCE;
    ptpConfig->masterParams.stepRemoved = TIMESYNC_PTP_DEFAULT_STEPS_REMOVED;
    ptpConfig->masterParams.UTCOffset = TIMESYNC_PTP_DEFAULT_UTC_OFFSET;
    /*Internal oscillator*/
    ptpConfig->masterParams.timeSource = TIMESYNC_PTP_DEFAULT_TIME_SOURCE;

    ptpConfig->masterParams.ptpFlags[TIMESYNC_PTP_TIMESCALE_INDEX] = 1U;
    ptpConfig->masterParams.ptpFlags[TIMESYNC_PTP_TWO_STEP_INDEX]  = 1U;
}

TimeSyncPtp_Handle TimeSyncPtp_init(TimeSyncPtp_Config *ptpConfig)
{
    /*Timer initialization variables*/
    int32_t status = TIMESYNC_OK;
    TimeSyncPtp_Handle hTimeSyncPtp = NULL;

    if (ptpConfig != NULL)
    {
        if ((TIMESYNC_PROT_UDP_IPV4 == ptpConfig->protocol) &&
            (TIMESYNC_PTP_DELAY_P2P == ptpConfig->type))
        {
            EnetAppUtils_print("Unsupported Protocol with P2P\r\n");
            status = TIMESYNC_UNSUPPORTED_FORMAT;
        }

        if ((TIMESYNC_PROT_IEEE_802_3 == ptpConfig->protocol) &&
            (TIMESYNC_PTP_DELAY_E2E == ptpConfig->type))
        {
            EnetAppUtils_print("Unsupported Protocol with E2E\r\n");
            status = TIMESYNC_UNSUPPORTED_FORMAT;
        }

        if (ptpConfig->deviceMode != TIMESYNC_ORDINARY_CLOCK)
        {
            EnetAppUtils_print("TimeSync Clock %d\r\n", ptpConfig->deviceMode);
            status = TIMESYNC_UNSUPPORTED_FORMAT;
        }

        if (status == TIMESYNC_OK)
        {
            hTimeSyncPtp = &gTimeSyncPtpObj;
            memset(hTimeSyncPtp, 0U, sizeof(TimeSyncPtp_Obj));
            hTimeSyncPtp->ptpConfig = *(ptpConfig);

            /*Allocate Rx and Tx packet buffers*/
            TimeSyncPtp_allocPktBuffer(hTimeSyncPtp);

            TimeSyncPtp_setDefaultValue(hTimeSyncPtp);

            TimeSyncPtp_setTimeSyncConfig(hTimeSyncPtp,
                                          &hTimeSyncPtp->timeSyncCfg);

            hTimeSyncPtp->enet_perctxt = EnetApp_TimeSyncOpen(&hTimeSyncPtp->timeSyncCfg);

            if (hTimeSyncPtp->enet_perctxt == NULL)
            {
                status = TIMESYNC_UNABLE_TO_INIT_HAL;
            }

            if (status == TIMESYNC_OK)
            {
                /*Create PTP Interrupts and Tasks*/
                status = TimeSyncPtp_createPtpTasks(hTimeSyncPtp);
            }
        }
    }
    else
    {
        status = TIMESYNC_PARAM_INVALID;
    }

    /* Reset PTP object and close HAL if initialization failed */
    if (status != TIMESYNC_OK)
    {
        memset(&gTimeSyncPtpObj, 0U, sizeof(TimeSyncPtp_Obj));
    }

    return ((status == TIMESYNC_OK) ? (&gTimeSyncPtpObj) : NULL);
}

void TimeSyncPtp_processTxNotify(void *arg,
                                 uint8_t portNum,
                                 uint8_t frameType,
                                 uint16_t seqId)
{
    if (arg != NULL)
    {
        TimeSyncPtp_Handle hTimeSyncPtp = (TimeSyncPtp_Handle)arg;
        txNotifyMsg.portNum   = portNum;
        txNotifyMsg.frameType = frameType;
        txNotifyMsg.seqId     = seqId;
        SemaphoreP_post(&hTimeSyncPtp->pktTxSemHandle);
    }
}

void TimeSyncPtp_processTxNotifyTask(void *arg)
{
    int32_t status = TIMESYNC_OK;
    uint8_t portNum = 0U;
    uint8_t frameType = 0U;
    uint16_t seqId = 0U;
    uint32_t nanoseconds = 0;
    uint64_t seconds = 0;
    TimeSyncPtp_Handle hTimeSyncPtp = NULL;
    volatile bool taskRunFlag = true;

    if (arg != NULL)
    {
        hTimeSyncPtp = (TimeSyncPtp_Handle)arg;
        if (hTimeSyncPtp->enet_perctxt != NULL)
        {
            while (taskRunFlag == true)
            {
                SemaphoreP_pend(&hTimeSyncPtp->pktTxSemHandle, SystemP_WAIT_FOREVER);

                portNum   = txNotifyMsg.portNum;
                frameType = txNotifyMsg.frameType;
                seqId     = txNotifyMsg.seqId;

                /*delay request frame*/
                if (frameType == ENET_TIMESYNC_MESSAGE_DELAY_REQ || frameType == ENET_TIMESYNC_MESSAGE_PDELAY_REQ)
                {
                    if (hTimeSyncPtp->ptpConfig.type == TIMESYNC_PTP_DELAY_P2P)
                    {
                        status = EnetApp_getTxTimestamp(hTimeSyncPtp->enet_perctxt,
                                                         ENET_TIMESYNC_MESSAGE_PDELAY_REQ,
                                                         portNum,
                                                         seqId,
                                                         &nanoseconds,
                                                         &seconds);
                        if (status == TIMESYNC_OK)
                        {
                            hTimeSyncPtp->pDelayParams[portNum].T1Sec = seconds;
                            /*Copy nanoseconds*/
                            hTimeSyncPtp->pDelayParams[portNum].T1Nsec = nanoseconds;
                        }
                    }
                }
            }
        }
    }
}

void TimeSyncPtp_processRxNotify(void *arg)
{
    if (arg != NULL)
    {
        TimeSyncPtp_Handle hTimeSyncPtp = (TimeSyncPtp_Handle)arg;
		SemaphoreP_post(&hTimeSyncPtp->pktRxSemHandle);
    }
}

void TimeSyncPtp_processRxNotifyTask(void *arg)
{
    uint32_t size  = 0;
    uint8_t rxPort = 0;
    uint8_t *dstMacId;
    uint8_t rxFrame[TIMESYNC_PTP_RX_MAX_MTU] = {0};
    TimeSyncPtp_Handle hTimeSyncPtp = NULL;
    volatile bool taskRunFlag = true;
    int32_t status=0;

    if (arg != NULL)
    {
        hTimeSyncPtp = (TimeSyncPtp_Handle)arg;
        if (hTimeSyncPtp->enet_perctxt != NULL)
        {
            while (taskRunFlag == true)
            {
                SemaphoreP_pend(&hTimeSyncPtp->pktRxSemHandle, SystemP_WAIT_FOREVER);

                do
                {
                    status = EnetApp_getPtpFrame(hTimeSyncPtp->enet_perctxt,
                                                  &rxFrame[0],
                                                  &size,
                                                  &rxPort);
                    if (status == TIMESYNC_OK)
                    {
                        dstMacId = rxFrame;

                        if (TIMESYNC_COMPARE_MAC(dstMacId, timeSyncMAC))
                        {
                            TimeSyncPtp_processPtpFrame(hTimeSyncPtp, rxFrame, rxPort, size, 0);
                        }
                        else if (true || TIMESYNC_COMPARE_MAC(dstMacId, linkLocalMAC))
                        {
                            TimeSyncPtp_processPtpFrame(hTimeSyncPtp, rxFrame, rxPort, size, 1);
                        }
                    }
                } while (status == TIMESYNC_OK);
            }
        }
    }
}

void TimeSyncPtp_doFirstAdjustment(TimeSyncPtp_Handle hTimeSyncPtp,
                                   uint8_t portNum)
{
    uint64_t doubleWord  = 0U;
    uint64_t syncRxTime  = 0U;
    uint64_t currentTime = 0U;
    uint32_t nanoSeconds = 0U;
    uint64_t seconds = 0U;
    uint64_t timeElapsed = 0U;

    /*Get origin time stamp in 64 bit format*/
    doubleWord = hTimeSyncPtp->syncParam[portNum].originTsSec *
                 (uint64_t)TIME_SEC_TO_NS +
                 (uint64_t)hTimeSyncPtp->syncParam[portNum].originTsNs;

    /*Now calculate time elapsed since sync was received*/
    syncRxTime = hTimeSyncPtp->syncParam[portNum].rxTsSec *
                 (uint64_t)TIME_SEC_TO_NS +
                 hTimeSyncPtp->syncParam[portNum].rxTs;

    EnetApp_getCurrentTime(hTimeSyncPtp->enet_perctxt,
                            &nanoSeconds,
                            &seconds);
    currentTime = seconds * (uint64_t)TIME_SEC_TO_NS + nanoSeconds;

    timeElapsed = currentTime - syncRxTime;

    /*Add correction field and peer delay*/
    doubleWord += (timeElapsed +
                   hTimeSyncPtp->tsRunTimeVar.pathDelay[portNum] +
                   hTimeSyncPtp->syncParam[portNum].correctionField);

    nanoSeconds = (uint32_t)(doubleWord % (uint64_t)TIME_SEC_TO_NS);
    seconds = doubleWord / (uint64_t)TIME_SEC_TO_NS;

    /* Set adjusted clock time to timer */
    EnetApp_setClockTime(hTimeSyncPtp->enet_perctxt,
                          nanoSeconds,
                          seconds);
	EnetAppUtils_print("Doing first adjustment\r\n");
}

/*Shown below is the Peer delay logic*/

/*
    Self           Peer         |
 *    T1            |           T
 *    |             T2          i
 *    |             |           m
 *    |             |           e
 *    |             T3          |
 *    T4            |           |
 *    |             |          \|/
 */

/* delay on self (device) is T4 - T1 and delay on peer is T3-T2
 * peer delay = ((T4 - T1) - (T3 - T2)) / 2
 *
 * Assumption: Entire transaction is completed within a second, so only nanoseconds fields are used.
 */
void TimeSyncPtp_peerDelayCalc(TimeSyncPtp_Handle hTimeSyncPtp,
                               uint8_t twoStep,
                               uint8_t portNum)
{
    uint64_t T3_T2_diff = 0;
    uint64_t T4_T1_diff = 0;
    uint64_t correctionFieldSum = 0;
	//uint32_t scaledRCF = 0;
    /*compute T4 - T1. Time difference on our device, take care of wrap around
     *Only nanoseconds used */

    if (hTimeSyncPtp->pDelayParams[portNum].T4Nsec < hTimeSyncPtp->pDelayParams[portNum].T1Nsec)
    {
        T4_T1_diff = ((uint64_t)TIME_SEC_TO_NS +
                      (uint64_t)hTimeSyncPtp->pDelayParams[portNum].T4Nsec) -
                     (uint64_t)hTimeSyncPtp->pDelayParams[portNum].T1Nsec;
    }
    else
    {
        T4_T1_diff = hTimeSyncPtp->pDelayParams[portNum].T4Nsec -
                     hTimeSyncPtp->pDelayParams[portNum].T1Nsec;
    }

    /*Multiply device time with NRR*/
    T4_T1_diff = (uint32_t)((double)T4_T1_diff *
                            hTimeSyncPtp->tsNrrInfo[portNum].nrr);

    /*Special processing if 2-step*/
    if (twoStep == 1)
    {
        /*compute T3-T2*. Time difference on Peer*/
        if (hTimeSyncPtp->pDelayParams[portNum].T3Nsec < hTimeSyncPtp->pDelayParams[portNum].T2Nsec)
        {
            T3_T2_diff = ((uint64_t)TIME_SEC_TO_NS +
                          (uint64_t)hTimeSyncPtp->pDelayParams[portNum].T3Nsec) -
                         (uint64_t)hTimeSyncPtp->pDelayParams[portNum].T2Nsec;
        }
        else
        {
            T3_T2_diff = hTimeSyncPtp->pDelayParams[portNum].T3Nsec -
                         hTimeSyncPtp->pDelayParams[portNum].T2Nsec;
        }

        /*Compute sum of correction fields*/
        correctionFieldSum = hTimeSyncPtp->pDelayParams[portNum].delayResCorrField +
                             hTimeSyncPtp->pDelayParams[portNum].delayResFwUpCorrField;

        if (T4_T1_diff >= (T3_T2_diff + correctionFieldSum))
        {
            hTimeSyncPtp->tsRunTimeVar.meanPathDelay = T4_T1_diff - T3_T2_diff - correctionFieldSum;
            /*Average the delay*/
            hTimeSyncPtp->tsRunTimeVar.meanPathDelay >>= 1;
        }
        else
        {
            hTimeSyncPtp->tsRunTimeVar.meanPathDelay = 0;
        }
    }
    else
    {
        /*compute T3-T2*. Time difference on Peer*/
        T3_T2_diff = hTimeSyncPtp->pDelayParams[portNum].delayResCorrField;

        if (T4_T1_diff > T3_T2_diff)
        {
            hTimeSyncPtp->tsRunTimeVar.meanPathDelay = T4_T1_diff - T3_T2_diff;
            /*Average the delay*/
            hTimeSyncPtp->tsRunTimeVar.meanPathDelay >>= 1;
        }
        else
        {
            hTimeSyncPtp->tsRunTimeVar.meanPathDelay = 0;
        }
    }

    if (hTimeSyncPtp->tsRunTimeVar.meanPathDelay > TIMESYNC_PTP_PEER_DELAY_ERROR_THRESHOLD)
    {
        hTimeSyncPtp->tsRunTimeVar.meanPathDelay = 0;
    }

    if ((hTimeSyncPtp->tsRunTimeVar.stateMachine & TIMESYNC_PTP_STATE_MACHINE_LINE_DELAY_COMPUTED) == 0)
    {
        hTimeSyncPtp->tsRunTimeVar.pathDelay[portNum] = hTimeSyncPtp->tsRunTimeVar.meanPathDelay;

        /*Once delay has been computed notify the state machine*/
        hTimeSyncPtp->tsRunTimeVar.stateMachine |= TIMESYNC_PTP_STATE_MACHINE_LINE_DELAY_COMPUTED;
    }
    else
    {
        /*Use exponential averaging filter to get the peer delay*/
        hTimeSyncPtp->tsRunTimeVar.pathDelay[portNum] =
            (uint32_t)((double)hTimeSyncPtp->tsRunTimeVar.pathDelay[portNum] *
                       (double)TIMESYNC_PTP_FILTER_ALPHA_COEFF +
                       (double)hTimeSyncPtp->tsRunTimeVar.meanPathDelay *
                       (double)(1 - TIMESYNC_PTP_FILTER_ALPHA_COEFF));
    }

    /*For 1-step TC delay asymmetry also gets added to the Sync frame correction
     * This is done by adding it to the path delay written in shared RAM*/
    hTimeSyncPtp->tsRunTimeVar.pathDelay[portNum] += hTimeSyncPtp->ptpConfig.asymmetryCorrection[portNum];
}

void TimeSyncPtp_synchronizeClock(TimeSyncPtp_Handle hTimeSyncPtp)
{
    uint8_t syncPortNum = 0;
    uint64_t meanPathDelay = 0;
    int64_t tempVar1  = 0;
    int64_t tempVar2  = 0;
    int32_t adjOffset = 0;
	static int32_t offset_arr[64] = {0};
	static int32_t avg_offset;
	static int32_t sum_offset;

    syncPortNum = hTimeSyncPtp->tsRunTimeVar.syncPortNum;
    /*Once initial adjustment is done, calculate the offset*/
    /*Get mean path delay*/
    meanPathDelay =
        hTimeSyncPtp->tsRunTimeVar.pathDelay[syncPortNum];


    if (hTimeSyncPtp->syncParam[syncPortNum].originTsSec ==
        hTimeSyncPtp->syncParam[syncPortNum].rxTsSec)
    {
        hTimeSyncPtp->tsRunTimeVar.currOffset =
            hTimeSyncPtp->syncParam[syncPortNum].rxTs -
            hTimeSyncPtp->syncParam[syncPortNum].originTsNs -
            meanPathDelay - hTimeSyncPtp->syncParam[syncPortNum].correctionField;
    }
    else
    {
        tempVar1 = ((int64_t)hTimeSyncPtp->syncParam[syncPortNum].rxTsSec - (int64_t)
                    hTimeSyncPtp->syncParam[syncPortNum].originTsSec)
                   * (int64_t)TIME_SEC_TO_NS;
        tempVar1 += (int64_t)hTimeSyncPtp->syncParam[syncPortNum].rxTs;
        tempVar2  = (int64_t)(hTimeSyncPtp->syncParam[syncPortNum].originTsNs +
                              meanPathDelay +
                              hTimeSyncPtp->syncParam[syncPortNum].correctionField);

        hTimeSyncPtp->tsRunTimeVar.currOffset = (int64_t)(tempVar1 - tempVar2);
    }

    /*Take running average of the offset*/
    hTimeSyncPtp->tsRunTimeVar.ltaOffset = (int64_t)((double)(TIMESYNC_PTP_FILTER_ALPHA_COEFF)*
                                                      (double)hTimeSyncPtp->tsRunTimeVar.ltaOffset +
                                                      (double)(1 - TIMESYNC_PTP_FILTER_ALPHA_COEFF) *
                                                      (double)hTimeSyncPtp->tsRunTimeVar.currOffset);

//    EnetAppUtils_print("St:0x%x , Offset: %d ns, init: %d ns, lta: %d ns\r\n",
//            hTimeSyncPtp->tsRunTimeVar.stateMachine,
//            hTimeSyncPtp->tsRunTimeVar.currOffset,
//            hTimeSyncPtp->tsRunTimeVar.initialOffset,
//            hTimeSyncPtp->tsRunTimeVar.ltaOffset);

    if (hTimeSyncPtp->tsRunTimeVar.driftStable)
    {
        adjOffset = hTimeSyncPtp->tsRunTimeVar.currOffset +
                    hTimeSyncPtp->tsRunTimeVar.ltaOffset +
                    hTimeSyncPtp->tsRunTimeVar.initialOffset;
    }
    else
    {
        adjOffset = hTimeSyncPtp->tsRunTimeVar.currOffset;
    }

    EnetApp_adjTimeSlowComp(hTimeSyncPtp->enet_perctxt,
                             adjOffset,
                             hTimeSyncPtp->tsRunTimeVar.ltaSyncInterval);

    /*Check this condition only if device is in sync*/
    if ((hTimeSyncPtp->tsRunTimeVar.stateMachine &
         TIMESYNC_PTP_STATE_MACHINE_DEVICE_IN_SYNC) ==
        TIMESYNC_PTP_STATE_MACHINE_DEVICE_IN_SYNC)
    {
        offset_arr[len++] = hTimeSyncPtp->tsRunTimeVar.currOffset;
        //EnetAppUtils_print("%d,%d\r\n", offset_arr[len], len);
        avg_offset = 0;
        sum_offset = 0;
        if (len == 8)
        {
            for (int32_t i = 0; i < len; i++)
            {
                sum_offset += abs(offset_arr[i]);
                if (offset_arr[i] < min_offset)
                {
                    min_offset = offset_arr[i];
                }

                if (offset_arr[i] > max_offset)
                {
                    max_offset = offset_arr[i];
                }
            }
            avg_offset = sum_offset / len;
            EnetAppUtils_print("Avg=%d\r\n", avg_offset);
            len = 0;
            EnetAppUtils_print("%dns<---->%dns\r\n", min_offset, max_offset);
            min_offset = 1000;
            max_offset = -1000;
        }

        /*This is to track any large change in the timebase*/
        if (abs(hTimeSyncPtp->tsRunTimeVar.ltaOffset) > TIMESYNC_PTP_OFFSET_THRESHOLD_FOR_RESET)
        {
            /*Reset state machine*/
//            TimeSyncPtp_reset(hTimeSyncPtp);
//            return;
        }
    }

    /*The code below is part of Statistics collection
     * some of which is used to implement the stabilization algo
     */

    /*Find clock drift*/
    if (hTimeSyncPtp->tsRunTimeVar.prevOffset != 0)
    {
        hTimeSyncPtp->tsRunTimeVar.clockDrift += abs(
                                                      hTimeSyncPtp->tsRunTimeVar.currOffset -
                                                      hTimeSyncPtp->tsRunTimeVar.prevOffset);
        hTimeSyncPtp->tsRunTimeVar.clockDrift =
            hTimeSyncPtp->tsRunTimeVar.clockDrift / 2;
    }

    /*set prevoffset to current once drift is computed*/
    hTimeSyncPtp->tsRunTimeVar.prevOffset =
        hTimeSyncPtp->tsRunTimeVar.currOffset;

    /*Wait for the drift to stabilize*/
    if (hTimeSyncPtp->tsRunTimeVar.clockDrift <= TIMESYNC_PTP_STABLE_FILTER_THRESHOLD &&
        (hTimeSyncPtp->tsRunTimeVar.driftStable == 0))
    {
        hTimeSyncPtp->tsRunTimeVar.driftStable = 1;
        hTimeSyncPtp->tsRunTimeVar.initialOffset = hTimeSyncPtp->tsRunTimeVar.currOffset;
    }

    /*Wait for offset to become zero*/
    if (abs(hTimeSyncPtp->tsRunTimeVar.ltaOffset) <= TIMESYNC_PTP_STABLE_FILTER_THRESHOLD &&
        (hTimeSyncPtp->tsRunTimeVar.offsetStable == 0) &&
        (hTimeSyncPtp->tsRunTimeVar.driftStable))
    {
        hTimeSyncPtp->tsRunTimeVar.offsetStable = 1;
        /*Indicate that the device is in sync now*/
        hTimeSyncPtp->tsRunTimeVar.stateMachine |= TIMESYNC_PTP_STATE_MACHINE_DEVICE_IN_SYNC;
    }

    /*The stabilization logic collects offsets which have a low drift and are
     * clustered together. The idea is that the PPM of the crystal might have shifted
     * and since we are relying on the initial offset measurement to do zero offset correction
     * we need to constantly track this zero offset value or else the PPM might settle into a zone
     * with high value and low drift.
     *
     * To take an example let's say we sync to a master with a fixed offset of 1000 ns.
     * The DUT as a slave will detect this value in the variable hTimeSyncPtp->tsRunTimeVar.initialOffset
     * After some time, let's say this offset has become 1100 ns. In the absence of this logic the
     * slave will settle into a zone with 100ns offset. If later the offset becomes 1200 ns then DUT
     * will settle into a zone with 200ns offset*/

    /*Run the logic only once we are stable*/
    if (hTimeSyncPtp->tsRunTimeVar.offsetStable)
    {
        hTimeSyncPtp->offsetAlgo.lastSeenGoodDriftIndex++;

        /*If the drift is below our threshold and we have a close cluster
         * then we use this value for our averaging purpose
         */
        if ((hTimeSyncPtp->tsRunTimeVar.clockDrift <
             hTimeSyncPtp->offsetAlgo.driftThreshold) &&
            (hTimeSyncPtp->offsetAlgo.lastSeenGoodDriftIndex < TIMESYNC_PTP_OFFSET_ALGO_CLUSTER_SIZE))
        {
            hTimeSyncPtp->offsetAlgo.lastSeenGoodDriftIndex = 0;

            if (hTimeSyncPtp->offsetAlgo.numEntriesIndex < TIMESYNC_PTP_OFFSET_ALGO_BIN_SIZE)
            {
                /*Store the value for averaging later*/
                hTimeSyncPtp->offsetAlgo.correction[hTimeSyncPtp->offsetAlgo.numEntriesIndex++]
                    = hTimeSyncPtp->tsRunTimeVar.currOffset;
            }
            else
            {
                hTimeSyncPtp->offsetAlgo.binFull = 1;
            }
        }
        /*If cluster is broken then we reset the counters*/
        else if ((hTimeSyncPtp->offsetAlgo.lastSeenGoodDriftIndex >= TIMESYNC_PTP_OFFSET_ALGO_CLUSTER_SIZE) &&
                 (hTimeSyncPtp->offsetAlgo.binFull != 1))
        {
            /*reset the count if we can't find a good match in our window*/
            hTimeSyncPtp->offsetAlgo.lastSeenGoodDriftIndex = 0;
            hTimeSyncPtp->offsetAlgo.numEntriesIndex = 0;
        }
    }
}

void TimeSyncPtp_updateNRRParams(TimeSyncPtp_Handle hTimeSyncPtp,
                                 uint8_t portNum)
{
    uint8_t curIndex = 0;

    TimeSyncPtp_NrrInfo *nrrInfo = NULL;

    /*point to the nrr for correct port*/
    nrrInfo = &hTimeSyncPtp->tsNrrInfo[portNum];

    curIndex = (nrrInfo->nrrIndex++) % 3;

    /*Please note that the RCF calculated here is inverse of the RCF described in spec
     * This is because we multiply our time adjustments with this value
     */
    if (curIndex == TIMESYNC_PTP_SYNT_DEPTH)
    {
        curIndex = 0;
    }

    /*This is used later*/
    nrrInfo->curIndex = curIndex;

    /*Populate the array with TS*/
    nrrInfo->deviceRxTS[curIndex] = (uint64_t)
                                    hTimeSyncPtp->pDelayParams[portNum].T4Sec *
                                    (uint64_t)TIME_SEC_TO_NS + hTimeSyncPtp->pDelayParams[portNum].T4Nsec;

    nrrInfo->correctedPeerTS[curIndex] =
        (uint64_t)hTimeSyncPtp->pDelayParams[portNum].T3Sec * (uint64_t)TIME_SEC_TO_NS
        + hTimeSyncPtp->pDelayParams[portNum].T3Nsec +
        hTimeSyncPtp->tsRunTimeVar.pathDelay[portNum] +
        hTimeSyncPtp->pDelayParams[portNum].delayResCorrField +
        hTimeSyncPtp->pDelayParams[portNum].delayResFwUpCorrField;

    /*Wait for TS array to be full before starting syntonization*/
    if ((!nrrInfo->nrrEnable) &&
        (nrrInfo->nrrIndex == (TIMESYNC_PTP_SYNT_DEPTH + 2)))
    {
        nrrInfo->nrrEnable = 1;
    }
}

void TimeSyncPtp_calcNRR(TimeSyncPtp_Handle hTimeSyncPtp,
                         uint8_t portNum)
{
    uint64_t num = 0;
    uint64_t den = 0;

    uint8_t curIndex  = 0;
    uint8_t prevIndex = 0;

    TimeSyncPtp_NrrInfo *nrrInfo = NULL;

    /*point to the nrr for correct port*/
    nrrInfo = &hTimeSyncPtp->tsNrrInfo[portNum];

    curIndex = nrrInfo->curIndex;

    /*calculate nrr */
    if (nrrInfo->nrrEnable)
    {
        prevIndex = hTimeSyncPtp->syntIndexMap[curIndex];
        num = nrrInfo->correctedPeerTS[curIndex] -
              nrrInfo->correctedPeerTS[prevIndex];

        den = nrrInfo->deviceRxTS[curIndex] -
              nrrInfo->deviceRxTS[prevIndex];

        /*calculate rcf*/
        nrrInfo->nrr = ((double)num) / den;
    }
}

void TimeSyncPtp_calcRcfAndSyncInterval(TimeSyncPtp_Handle hTimeSyncPtp)
{
    uint64_t num = 0;
    uint64_t den = 0;
    //uint32_t scaledRCF = 0;
    uint8_t curIndex  = 0;
    uint8_t prevIndex = 0;
    uint8_t tempIndex = 0;
    uint8_t count = 0;
    uint8_t syncPortNum = 0;
    uint64_t syncIntervalAverage = 0;
    double scalingFactor = 0;

    curIndex = (hTimeSyncPtp->tsSyntInfo.syntIndex++) % 3;

    syncPortNum = hTimeSyncPtp->tsRunTimeVar.syncPortNum;

    /*Please note that the RCF calculated here is inverse of the RCF described in spec
     * This is because we multiply our time adjustments with this value
     */
    if (curIndex == TIMESYNC_PTP_SYNT_DEPTH)
    {
        curIndex = 0;
    }

    /*Populate the array with TS*/
    hTimeSyncPtp->tsSyntInfo.syncIngressTs[curIndex] = (uint64_t)
                                                        hTimeSyncPtp->syncParam[syncPortNum].rxTsSec *
                                                        (uint64_t)TIME_SEC_TO_NS + hTimeSyncPtp->syncParam[syncPortNum].rxTs;

    hTimeSyncPtp->tsSyntInfo.correctedMasterTs[curIndex] =
        (uint64_t)hTimeSyncPtp->syncParam[syncPortNum].originTsSec *
        (uint64_t)TIME_SEC_TO_NS +
        hTimeSyncPtp->syncParam[syncPortNum].originTsNs +
        hTimeSyncPtp->tsRunTimeVar.pathDelay[hTimeSyncPtp->tsRunTimeVar.syncPortNum] +
        hTimeSyncPtp->syncParam[syncPortNum].correctionField;

    /*Wait for TS array to be full before starting syntonization*/
    if ((!hTimeSyncPtp->tsSyntInfo.syntEnable) &&
        (hTimeSyncPtp->tsSyntInfo.syntIndex == (TIMESYNC_PTP_SYNT_DEPTH + 2)))
    {
        hTimeSyncPtp->tsSyntInfo.syntEnable = 1;
    }

    /*calculate rcf */
    if (hTimeSyncPtp->tsSyntInfo.syntEnable)
    {
        prevIndex = hTimeSyncPtp->syntIndexMap[curIndex];
        num = hTimeSyncPtp->tsSyntInfo.correctedMasterTs[curIndex] -
              hTimeSyncPtp->tsSyntInfo.correctedMasterTs[prevIndex];

        den = hTimeSyncPtp->tsSyntInfo.syncIngressTs[curIndex] -
              hTimeSyncPtp->tsSyntInfo.syncIngressTs[prevIndex];

        /*calculate rcf*/
        hTimeSyncPtp->tsSyntInfo.rcf = ((double)num) / den;
        //scaledRCF = (uint32_t)(hTimeSyncPtp->tsSyntInfo.rcf * 1024);

        /*calculate average of difference between two successive sync frames*/
        for (count = 0; count < 3; count++)
        {
            tempIndex = (curIndex + count) % TIMESYNC_PTP_SYNT_DEPTH;
            prevIndex = hTimeSyncPtp->prevIndexMap[tempIndex];

            if (hTimeSyncPtp->tsSyntInfo.correctedMasterTs[tempIndex] >
                hTimeSyncPtp->tsSyntInfo.correctedMasterTs[prevIndex])
            {
                syncIntervalAverage += (hTimeSyncPtp->tsSyntInfo.correctedMasterTs[tempIndex] -
                                        hTimeSyncPtp->tsSyntInfo.correctedMasterTs[prevIndex]);
            }
        }

        syncIntervalAverage = syncIntervalAverage / 2;

        hTimeSyncPtp->tsRunTimeVar.currSyncInterval = syncIntervalAverage;

        if (hTimeSyncPtp->tsRunTimeVar.ltaSyncInterval == 0)
        {
            hTimeSyncPtp->tsRunTimeVar.ltaSyncInterval =
                hTimeSyncPtp->tsRunTimeVar.currSyncInterval;
            hTimeSyncPtp->tsRunTimeVar.firstSyncInterval =
                hTimeSyncPtp->tsRunTimeVar.currSyncInterval;

            /*Add 50% extra so minor variations in master don't cause a timeout*/
            hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval =
                hTimeSyncPtp->tsRunTimeVar.currSyncInterval;
            hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval +=
                hTimeSyncPtp->tsRunTimeVar.currSyncInterval >> 1;

            scalingFactor = ((double)1000 / (double)(
                                                     hTimeSyncPtp->ptpConfig.tickPeriod));

            /*Multiply/Divide by clock scaling factor*/
            hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval =
                (uint64_t)((double)hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval *
                           scalingFactor);

            /*convert in terms of PTP BG ticks*/
            hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval =
                (uint64_t)((double)hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval /
                           (double)(TIMESYNC_PTP_BG_TASK_TICK_PERIOD * 1000000));
        }
        else
        {
            hTimeSyncPtp->tsRunTimeVar.ltaSyncInterval =
                (uint64_t)(((double)(hTimeSyncPtp->tsRunTimeVar.ltaSyncInterval) * TIMESYNC_PTP_FILTER_ALPHA_COEFF) +
                           ((double)(1 - TIMESYNC_PTP_FILTER_ALPHA_COEFF) *
                            (double)(hTimeSyncPtp->tsRunTimeVar.currSyncInterval)));
        }

        hTimeSyncPtp->syncInterval = hTimeSyncPtp->tsRunTimeVar.ltaSyncInterval;

        /*Once sync interval has been calculated notify the state machine*/
        hTimeSyncPtp->tsRunTimeVar.stateMachine |=
            TIMESYNC_PTP_STATE_MACHINE_SYNC_INTERVAL_COMPUTED;
    }
}

void TimeSyncPtp_processPtpFrame(TimeSyncPtp_Handle hTimeSyncPtp,
                                 uint8_t *pktBuffer,
                                 uint8_t portNum,
                                 uint32_t size,
                                 uint8_t isLinkLocal)
{
    int32_t status = TIMESYNC_OK;
    uint8_t pktType = 0U;
    uint8_t ifTwoStep = 0U;
    uint8_t offset = 0U;
    uint32_t nanoseconds = 0U;
    uint64_t seconds = 0U;
    uint16_t etherType = 0U;
    uint16_t seqId = 0U;

    offset = hTimeSyncPtp->ptpConfig.frameOffset;

    if (TRUE == hTimeSyncPtp->ptpConfig.hsrEnabled)
    {
        if (isLinkLocal)
        {
            etherType = TimeSync_convBigEndianToLittleEndianHalfWord(pktBuffer + TIMESYNC_PTP_SRC_DST_MAC_SIZE);
            if (etherType == TIMESYNC_PTP_HSR_ETHERTYPE)
            {
                hTimeSyncPtp->ptpConfig.ll_has_hsrTag = TRUE;
            }

            if (hTimeSyncPtp->ptpConfig.ll_has_hsrTag)
            {
                offset -= TIMESYNC_PTP_HSR_CORRECTION;
            }
        }
        else
        {
            offset -= TIMESYNC_PTP_HSR_CORRECTION;
        }
    }

    if (TIMESYNC_VLAN_TYPE_NONE == hTimeSyncPtp->ptpConfig.vlanCfg.vlanType)
    {
        /* Strip VLAN tag if PTP operates in non-VLAN mode, but packet has VLAN tag */
        TimeSyncPtp_stripVlanTag(hTimeSyncPtp, pktBuffer, &size);
    }

    if (TIMESYNC_VLAN_TYPE_SINGLE_TAG == hTimeSyncPtp->ptpConfig.vlanCfg.vlanType)
    {
        offset -= TIMESYNC_PTP_SINGLE_TAG_VLAN_HDR_SIZE;
    }

    if (TIMESYNC_VLAN_TYPE_DOUBLE_TAG == hTimeSyncPtp->ptpConfig.vlanCfg.vlanType)
    {
        offset -= TIMESYNC_PTP_DOUBLE_TAG_VLAN_HDR_SIZE;
    }

    pktType = (uint8_t)(*(pktBuffer + TIMESYNC_PTP_MSG_ID_OFFSET - offset));
    ifTwoStep  = (uint8_t)(*(pktBuffer + TIMESYNC_PTP_FLAG_OFFSET - offset));
    ifTwoStep &= 2;
    TimeSync_convEndianess(pktBuffer + TIMESYNC_PTP_SEQ_ID_OFFSET - offset, &seqId, 2);

    if (0 && (seqId == 0))
    {
             uint64_t nanoSeconds = 0;
             status = EnetApp_getRxTimestamp(hTimeSyncPtp->enet_perctxt,
                                             pktType,
                                              portNum,
                                              seqId,
                                              &nanoseconds,
                                              &seconds);

             uint64_t tsVal = (uint64_t)(((uint64_t)seconds * (uint64_t)TIME_SEC_TO_NS) + nanoSeconds);
             EnetAppUtils_print("RX PTP time is : %llu\r\n", tsVal);
    }

    /*PTPd stack handles announce and management messages*/
    if ((TIMESYNC_PTP_ANNOUNCE_MSG_ID == pktType) || (TIMESYNC_PTP_MGMT_MSG_ID == pktType))
    {
        if (TIMESYNC_PTP_ANNOUNCE_MSG_ID == pktType)
        {
            /*Just parse the Announce frame for MAC ID
             * and make it master
             */
            TimeSyncPtp_BMCA(hTimeSyncPtp, pktBuffer, portNum);
            return;
        }

        /*Should we use the port number???*/
        if (!(hTimeSyncPtp->stackParams.generalFrameFlag)) /*Make sure the previous packet is trnasfered to PTP Stack*/
        {
            hTimeSyncPtp->stackParams.ptpGeneralSize = size;

            if (TRUE == hTimeSyncPtp->ptpConfig.hsrEnabled)
            {
                hTimeSyncPtp->stackParams.ptpGeneralSize -= TIMESYNC_PTP_HSR_CORRECTION;
                memcpy(hTimeSyncPtp->stackParams.ptpGeneralFrame,
                       pktBuffer,
                       TIMESYNC_PTP_SRC_DST_MAC_SIZE);
                memcpy(hTimeSyncPtp->stackParams.ptpGeneralFrame + TIMESYNC_PTP_SRC_DST_MAC_SIZE,
                       pktBuffer + TIMESYNC_PTP_SRC_DST_MAC_SIZE + TIMESYNC_PTP_HSR_CORRECTION,
                       hTimeSyncPtp->stackParams.ptpGeneralSize - TIMESYNC_PTP_SRC_DST_MAC_SIZE);
            }
            else
            {
                memcpy(hTimeSyncPtp->stackParams.ptpGeneralFrame, pktBuffer,
                       hTimeSyncPtp->stackParams.ptpGeneralSize);
            }

            hTimeSyncPtp->stackParams.generalFrameFlag = 1;
        }
    }
    else
    {
        if (TIMESYNC_PTP_PDLY_RSP_MSG_ID == pktType)        /*Pdelay response frame*/
        {
            status = EnetApp_getRxTimestamp(hTimeSyncPtp->enet_perctxt,
                                             ENET_TIMESYNC_MESSAGE_PDELAY_RESP,
                                             portNum,
                                             seqId,
                                             &nanoseconds,
                                             &seconds);
            if (status == TIMESYNC_OK)
            {

                /*T4 timestamp*/
                hTimeSyncPtp->pDelayParams[portNum].T4Sec = seconds;
                hTimeSyncPtp->pDelayParams[portNum].T4Nsec = nanoseconds;

                /*set two step flag. Used in another task*/
                hTimeSyncPtp->pDelayParams[portNum].ifTwoStep = ifTwoStep;

                if (hTimeSyncPtp->ptpConfig.ll_has_hsrTag)
                {
                    memcpy(hTimeSyncPtp->timeSyncBuff.pdelayResRxBuf[portNum],
                           pktBuffer,
                           TIMESYNC_PTP_SRC_DST_MAC_SIZE);
                    memcpy(hTimeSyncPtp->timeSyncBuff.pdelayResRxBuf[portNum] +
                           TIMESYNC_PTP_SRC_DST_MAC_SIZE,
                           pktBuffer + TIMESYNC_PTP_SRC_DST_MAC_SIZE + TIMESYNC_PTP_HSR_CORRECTION,
                           TIMESYNC_PTP_PDELAY_BUF_SIZE - TIMESYNC_PTP_SRC_DST_MAC_SIZE);
                }
                else
                {
                    /*Copy into buffer and post event*/
                    memcpy(hTimeSyncPtp->timeSyncBuff.pdelayResRxBuf[portNum],
                           pktBuffer, TIMESYNC_PTP_PDELAY_BUF_SIZE);
                }

                EventP_setBits(&hTimeSyncPtp->ptpPdelayResEvtHandle[portNum],
                            hTimeSyncPtp->eventIdPdelayResp);

                /*Another task pends on both Pdelay Resp and
                 * Pdelay Resp Follow Up frame. If this is a single step
                 * Pdelay response then we don't want that task to
                 * pend forever
                 */
                if (!ifTwoStep)
                {
                    EventP_setBits(&hTimeSyncPtp->ptpPdelayResEvtHandle[portNum],
                                hTimeSyncPtp->eventIdPdelayRespFlwUp);
                }
            }
        }
        else if (TIMESYNC_PTP_PDLY_RESP_FLW_UP_MSG_ID == pktType)        /*Pdelay response follow up frame*/
        {
            /*Copy into buffer and post event*/
            if (hTimeSyncPtp->ptpConfig.ll_has_hsrTag)
            {
                memcpy(hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpRxBuf[portNum],
                       pktBuffer,
                       TIMESYNC_PTP_SRC_DST_MAC_SIZE);
                memcpy(hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpRxBuf[portNum] +
                       TIMESYNC_PTP_SRC_DST_MAC_SIZE,
                       pktBuffer + TIMESYNC_PTP_SRC_DST_MAC_SIZE + TIMESYNC_PTP_HSR_CORRECTION,
                       TIMESYNC_PTP_PDELAY_BUF_SIZE - TIMESYNC_PTP_SRC_DST_MAC_SIZE);
            }
            else
            {
                memcpy(hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpRxBuf[portNum],
                       pktBuffer, TIMESYNC_PTP_PDELAY_BUF_SIZE);
            }

            EventP_setBits(&hTimeSyncPtp->ptpPdelayResEvtHandle[portNum],
                        hTimeSyncPtp->eventIdPdelayRespFlwUp);
        }
        else if (TIMESYNC_PTP_SYNC_MSG_ID == pktType)
        {
            status = EnetApp_getRxTimestamp(hTimeSyncPtp->enet_perctxt,
                                             ENET_TIMESYNC_MESSAGE_SYNC,
                                             portNum,
                                             seqId,
                                             &nanoseconds,
                                             &seconds);
            if (status == TIMESYNC_OK)
            {
                /*Store Rx timestamp*/
                hTimeSyncPtp->syncParam[portNum].rxTsSec = seconds;
                hTimeSyncPtp->syncParam[portNum].rxTs = nanoseconds;

                /*set two step flag. Used in another task*/
                hTimeSyncPtp->syncParam[portNum].ifTwoStep = ifTwoStep;
                TimeSyncPtp_processSyncFrame(hTimeSyncPtp, pktBuffer, FALSE, portNum, size);
            }

        }
        else if (TIMESYNC_PTP_FOLLOW_UP_MSG_ID == pktType)
        {
            TimeSyncPtp_processSyncFrame(hTimeSyncPtp, pktBuffer, TRUE, portNum, size);
        }
        else
        {
            return;
        }
    }
}

void TimeSyncPtp_processSyncFrame(TimeSyncPtp_Handle hTimeSyncPtp,
                                  uint8_t *buff,
                                  uint8_t followUp,
                                  uint8_t portNum,
                                  uint32_t size)
{
    uint8_t *bytePtr = NULL;
    uint8_t offset = 0;
    uint16_t halfWord = 0;
    uint64_t followUpCorrectionField = 0;

    /*packet offset*/
    offset = hTimeSyncPtp->ptpConfig.frameOffset;

    /*Since we are processing tagged HSR frames, we need to account for it*/
    if (TRUE == hTimeSyncPtp->ptpConfig.hsrEnabled)
    {
        offset -= TIMESYNC_PTP_HSR_CORRECTION;
    }

    if (TIMESYNC_VLAN_TYPE_SINGLE_TAG == hTimeSyncPtp->ptpConfig.vlanCfg.vlanType)
    {
        offset -= TIMESYNC_PTP_SINGLE_TAG_VLAN_HDR_SIZE;
    }

    /*Take timestamps and calculate BD since this is common to sync
     * frames from both ports. Synchronization OTOH is done only for
     * sync frames from master port*/

    /*For sync frame*/
    if (!followUp)
    {
        if (hTimeSyncPtp->masterPortNum == portNum)
        {
            hTimeSyncPtp->tsRunTimeVar.syncPortNum = portNum;
            /*Reset the last seen counter*/
            hTimeSyncPtp->tsRunTimeVar.syncLastSeenCounter = 0;
        }

        /*Copy correction field*/
        bytePtr = (uint8_t *)(buff + TIMESYNC_PTP_CORRECTION_OFFSET - offset);
        TimeSync_convEnd6to8(bytePtr, &(hTimeSyncPtp->syncParam[portNum].correctionField));

        /*Do asymmetry correction*/
        hTimeSyncPtp->syncParam[portNum].correctionField +=
            hTimeSyncPtp->ptpConfig.asymmetryCorrection[portNum];

        /*Copy sequence ID*/
        bytePtr = (uint8_t *)(buff + TIMESYNC_PTP_SEQ_ID_OFFSET - offset);
        TimeSync_convEndianess(bytePtr, &(hTimeSyncPtp->tsRunTimeVar.curSyncSeqId[portNum]), 2);
        //hTimeSyncPtp->tsRunTimeVar.forced2step[portNum] = FALSE;

        if ((hTimeSyncPtp->syncParam[portNum].ifTwoStep) || (hTimeSyncPtp->masterPortNum != portNum))
        {
            return;
        }

        /*Get origin timestamp*/
        bytePtr = (uint8_t *)(buff + TIMESYNC_PTP_REQ_RCPT_TS_SEC_OFFSET - offset);
        TimeSync_convEnd6to8(bytePtr, &(hTimeSyncPtp->syncParam[portNum].originTsSec));

        bytePtr = (uint8_t *)(buff + TIMESYNC_PTP_REQ_RCPT_TS_NSEC_OFFSET - offset);
        TimeSync_convEndianess(bytePtr, &(hTimeSyncPtp->syncParam[portNum].originTsNs), 4);
    }
    else
    {
        /*Check for Sync mismatch*/
        bytePtr = (uint8_t *)(buff + TIMESYNC_PTP_SEQ_ID_OFFSET - offset);
        TimeSync_convEndianess(bytePtr, &(halfWord), 2);

        if (hTimeSyncPtp->tsRunTimeVar.curSyncSeqId[portNum] != halfWord)
        {
            return;
        }

        /*get correction field in follow up frame*/
        bytePtr = (uint8_t *)(buff + TIMESYNC_PTP_CORRECTION_OFFSET - offset);
        TimeSync_convEnd6to8(bytePtr, &(followUpCorrectionField));

        /*Correction field contains correction field value in sync frame
         * It needs to be added to the correction field value inside follow up*/
        hTimeSyncPtp->syncParam[portNum].correctionField += followUpCorrectionField;

        /*Get origin timestamp*/
        bytePtr = (uint8_t *)(buff + TIMESYNC_PTP_REQ_RCPT_TS_SEC_OFFSET - offset);
        TimeSync_convEnd6to8(bytePtr, &(hTimeSyncPtp->syncParam[portNum].originTsSec));

        bytePtr = (uint8_t *)(buff + TIMESYNC_PTP_REQ_RCPT_TS_NSEC_OFFSET - offset);
        TimeSync_convEndianess(bytePtr, &(hTimeSyncPtp->syncParam[portNum].originTsNs), 4);
    }

    /*First time*/
    if ((hTimeSyncPtp->tsRunTimeVar.stateMachine &
         TIMESYNC_PTP_STATE_MACHINE_FIRST_ADJUSTMENT_DONE) == 0)
    {
        /* Check to make sure we don't latch on to a sync message before we
           select the master */
        if (hTimeSyncPtp->tsRunTimeVar.bmcaDone == 1)
        {
            /*Do first adjustment */
            TimeSyncPtp_doFirstAdjustment(hTimeSyncPtp, portNum);

            /*indicate that first adjustment is done*/
            hTimeSyncPtp->tsRunTimeVar.stateMachine |= TIMESYNC_PTP_STATE_MACHINE_FIRST_ADJUSTMENT_DONE;
        }
    }
    else    /*if not first time sync frame*/
    {
        if (hTimeSyncPtp->tsRunTimeVar.syncPortNum == portNum)
        {
            if ((hTimeSyncPtp->tsRunTimeVar.stateMachine &
                 TIMESYNC_PTP_STATE_MACHINE_READY_FOR_SYNC) == TIMESYNC_PTP_STATE_MACHINE_READY_FOR_SYNC)
            {
                /*Synchronize clock*/
                TimeSyncPtp_synchronizeClock(hTimeSyncPtp);
            }
        }
    }

    if (hTimeSyncPtp->tsRunTimeVar.syncPortNum == portNum)
    {
        TimeSyncPtp_calcRcfAndSyncInterval(hTimeSyncPtp);
    }
}

void TimeSyncPtp_processPdelayRespFrame(TimeSyncPtp_Handle hTimeSyncPtp,
                                        uint8_t *buff,
                                        uint8_t followUp,
                                        uint8_t portNum)
{
    uint8_t offset = hTimeSyncPtp->ptpConfig.frameOffset;

    /*generic buffer used for comparison and storing*/
    uint8_t buf_for_comp[8] = {0};
    uint16_t halfWord = 0;

    if (TIMESYNC_VLAN_TYPE_SINGLE_TAG == hTimeSyncPtp->ptpConfig.vlanCfg.vlanType)
    {
        offset -= TIMESYNC_PTP_SINGLE_TAG_VLAN_HDR_SIZE;
    }

    /*check if this is a valid response frame to our delay request
     * extract parameters and calculate peer delay if so*/

    /*Check for source port identity*/
    memcpy(buf_for_comp, buff + TIMESYNC_PTP_REQ_SRC_PORT_IDENTITY - offset, 8);

    if (0 != memcmp(buf_for_comp, hTimeSyncPtp->clockIdentity, 8))
    {
        return;
    }

    /*check with the sequence id that was sent out*/
    TimeSync_convEndianess(buff + TIMESYNC_PTP_SEQ_ID_OFFSET - offset, &halfWord, 2);

    if (halfWord != (hTimeSyncPtp->tsRunTimeVar.pDelReqSequenceID[portNum] - 1))
    {
        return;
    }

    if (!followUp)
    {
        /*Extract correction field*/
        TimeSync_convEnd6to8(buff + TIMESYNC_PTP_CORRECTION_OFFSET - offset,
                             &(hTimeSyncPtp->pDelayParams[portNum].delayResCorrField));

        /*Now check if it's a two step, if yes then exit*/
        TimeSync_convEndianess(buff + TIMESYNC_PTP_FLAG_OFFSET - offset, &halfWord, 2);

        if (halfWord & TIMESYNC_PTP_TWO_STEP_MASK)
        {
            /*Extract T2 timestamp in seconds*/
            TimeSync_convEnd6to8(buff + TIMESYNC_PTP_REQ_RCPT_TS_SEC_OFFSET - offset,
                                 &(hTimeSyncPtp->pDelayParams[portNum].T2Sec));

            /*Extract T2 timestamp in nanoseconds*/
            TimeSync_convEndianess(buff + TIMESYNC_PTP_REQ_RCPT_TS_NSEC_OFFSET - offset,
                                   &(hTimeSyncPtp->pDelayParams[portNum].T2Nsec), 4);

            return;
        }
    }
    else
    {
        /*Extract correction field*/
        TimeSync_convEnd6to8(buff + TIMESYNC_PTP_CORRECTION_OFFSET - offset,
                             &(hTimeSyncPtp->pDelayParams[portNum].delayResFwUpCorrField));
        /*Extract T2 timestamp in seconds*/
        TimeSync_convEnd6to8(buff + TIMESYNC_PTP_REQ_RCPT_TS_SEC_OFFSET - offset,
                             &(hTimeSyncPtp->pDelayParams[portNum].T3Sec));

        /*Extract T2 timestamp in nanoseconds.*/
        TimeSync_convEndianess(buff + TIMESYNC_PTP_REQ_RCPT_TS_NSEC_OFFSET - offset,
                               &(hTimeSyncPtp->pDelayParams[portNum].T3Nsec),
                               4);
    }

    /*Calculate peer delay*/
    TimeSyncPtp_peerDelayCalc(hTimeSyncPtp, followUp, portNum);
    /*Update params here, NRR gets calculated in the background*/
    TimeSyncPtp_updateNRRParams(hTimeSyncPtp, portNum);
    TimeSyncPtp_calcNRR(hTimeSyncPtp, portNum);

    return;
}

void TimeSyncPtp_reset(TimeSyncPtp_Handle hTimeSyncPtp)
{
	EnetAppUtils_print("RESETTING.-.-.-.-.\r\n");

    /* Call sync loss callback*/
    if (hTimeSyncPtp->ptpConfig.syncLossNotifyFxn != NULL)
    {
        hTimeSyncPtp->ptpConfig.syncLossNotifyFxn();
    }

    /*first disable PTP so firmware is not running*/
    TimeSyncPtp_disable(hTimeSyncPtp);

    /*Now reset all values*/
    EnetApp_reset(hTimeSyncPtp->enet_perctxt);

    /*Reset structures*/
    memset(&hTimeSyncPtp->syncParam[0], 0x0, sizeof(TimeSyncPtp_SyncParam));
    memset(&hTimeSyncPtp->syncParam[1], 0x0, sizeof(TimeSyncPtp_SyncParam));
    //memset(&hTimeSyncPtp->delayParams, 0x0, sizeof(TimeSyncPtp_DelayReqRespParams));
    memset(&hTimeSyncPtp->pDelayParams[0], 0x0, sizeof(TimeSyncPtp_PeerDelayParams));
    memset(&hTimeSyncPtp->pDelayParams[1], 0x0, sizeof(TimeSyncPtp_PeerDelayParams));
    memset(&hTimeSyncPtp->tsSyntInfo, 0x0, sizeof(TimeSyncPtp_SyntInfo));
    memset(&hTimeSyncPtp->tsRunTimeVar, 0x0, sizeof(TimeSyncPtp_RuntimeVar));
    memset(&hTimeSyncPtp->tsNrrInfo[0], 0x0, sizeof(TimeSyncPtp_NrrInfo));
    memset(&hTimeSyncPtp->tsNrrInfo[1], 0x0, sizeof(TimeSyncPtp_NrrInfo));

    hTimeSyncPtp->tsRunTimeVar.stateMachine = 0;
    hTimeSyncPtp->tsSyntInfo.rcf = 1.0;
    hTimeSyncPtp->tsNrrInfo[0].nrr = 1.0;
    hTimeSyncPtp->tsNrrInfo[1].nrr = 1.0;
    hTimeSyncPtp->tsRunTimeVar.clockDrift = 10000;
    hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval = 10000;
    hTimeSyncPtp->offsetAlgo.driftThreshold = TIMESYNC_PTP_OFFSET_STABLE_ALGO_THRESHOLD;
	hTimeSyncPtp->tsRunTimeVar.ltaSyncInterval = 10000;

    /*Finally enable PTP*/
    TimeSyncPtp_enable(hTimeSyncPtp);
}

void TimeSyncPtp_BMCA(TimeSyncPtp_Handle hTimeSyncPtp,
                      uint8_t *pktBuffer,
                      uint8_t portNum)
{
    /*Extract MAC ID and write parent address*/

    if (!hTimeSyncPtp->tsRunTimeVar.bmcaDone)
    {
        /* Update master port number */
        hTimeSyncPtp->masterPortNum = portNum;
        /* Mark BMCA as done */
        hTimeSyncPtp->tsRunTimeVar.bmcaDone = 1;


    }
}

static void TimeSyncPtp_setTimeSyncConfig(TimeSyncPtp_Handle hTimeSyncPtp,
                                          TimeSync_Config *pTimeSyncConfig)
{
    pTimeSyncConfig->protoCfg.protocol  = hTimeSyncPtp->ptpConfig.protocol;
    pTimeSyncConfig->protoCfg.deviceCfg = hTimeSyncPtp->ptpConfig.deviceMode;

    if (hTimeSyncPtp->ptpConfig.masterParams.ptpFlags[TIMESYNC_PTP_TWO_STEP_INDEX])
    {
        pTimeSyncConfig->protoCfg.stepMode = TIMESYNC_DOUBLE_STEP;
    }
    else
    {
        pTimeSyncConfig->protoCfg.stepMode = TIMESYNC_SINGLE_STEP;
    }

    pTimeSyncConfig->protoCfg.vlanCfg = hTimeSyncPtp->ptpConfig.vlanCfg;
    pTimeSyncConfig->protoCfg.portMask = hTimeSyncPtp->ptpConfig.portMask;

    memcpy(&pTimeSyncConfig->protoCfg.domainNumber[0U],
           &hTimeSyncPtp->ptpConfig.domainNumber[0U],
           sizeof(hTimeSyncPtp->ptpConfig.domainNumber));

    memcpy(&pTimeSyncConfig->protoCfg.clockIdentity[0U],
           &hTimeSyncPtp->clockIdentity[0U],
           sizeof(hTimeSyncPtp->clockIdentity));

    pTimeSyncConfig->frameNotifyCfg.nwDrvHandle = hTimeSyncPtp->ptpConfig.nwDrvHandle;
    pTimeSyncConfig->frameNotifyCfg.txNotifyCb  = &TimeSyncPtp_processTxNotify;
    pTimeSyncConfig->frameNotifyCfg.txNotifyCbArg = (void *)hTimeSyncPtp;
    pTimeSyncConfig->frameNotifyCfg.rxNotifyCb = &TimeSyncPtp_processRxNotify;
    pTimeSyncConfig->frameNotifyCfg.rxNotifyCbArg = (void *)hTimeSyncPtp;
}

static void TimeSyncPtp_stripVlanTag(TimeSyncPtp_Handle hTimeSyncPtp,
                                     uint8_t *pktBuffer,
                                     uint32_t *size)
{
    uint8_t *oldBuffer, *newBuffer;
    uint16_t etherType;

    oldBuffer = pktBuffer;
    newBuffer = pktBuffer;
    etherType = TimeSync_convBigEndianToLittleEndianHalfWord(pktBuffer + TIMESYNC_PTP_SRC_DST_MAC_SIZE);

    if (etherType == TIMESYNC_PTP_VLAN_ETHERTYPE)
    {
        *size = *size - TIMESYNC_PTP_SINGLE_TAG_VLAN_HDR_SIZE;
        memcpy(newBuffer + TIMESYNC_PTP_SRC_DST_MAC_SIZE,
               oldBuffer + TIMESYNC_PTP_SRC_DST_MAC_SIZE + TIMESYNC_PTP_SINGLE_TAG_VLAN_HDR_SIZE,
               ((*size) - TIMESYNC_PTP_SRC_DST_MAC_SIZE));
    }
}
