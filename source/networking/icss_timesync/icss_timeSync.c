/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
#include <string.h>
#include "icss_timeSync.h"
#include "icss_timeSync_memory_map.h"
#include "icss_timeSyncApi.h"
#include "icss_timeSync_init.h"
#include "icss_timeSync_utils.h"
#include <drivers/hw_include/hw_types.h>
#include <drivers/mdio.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef TIMESYNC_LOCAL_DEBUG
#define MSIZE 1000
uint32_t Mindex;
uint32_t MoriginTsSec[MSIZE];
uint32_t MoriginTsNs[MSIZE];
uint32_t MrxTsSec[MSIZE];
uint32_t MrxTs[MSIZE];
uint32_t MdelReqTxTsSec[MSIZE];
uint32_t MdelReqTxTsNS[MSIZE];
uint32_t MtimeStampSec[MSIZE];
uint32_t MtimeStampNS[MSIZE];
uint32_t MdelayRespCorrection[MSIZE];
uint32_t McorrectionField[MSIZE];
int32_t McurrOffset[MSIZE];
int32_t MadjOffset[MSIZE];
int32_t MltaOffset[MSIZE];
int32_t MinitialOffset[MSIZE];
int32_t MprevOffset0[MSIZE];
int32_t MprevOffset1[MSIZE];
uint32_t McurrSyncInterval[MSIZE];
uint32_t MltaSyncInterval[MSIZE];
uint32_t MfirstSyncInterval[MSIZE];
uint32_t MmeanPathDelay[MSIZE];
uint32_t MstateMachine[MSIZE];
uint8_t MdriftStable[MSIZE];
uint8_t MoffsetStable[MSIZE];
double Mrcf[MSIZE];
uint32_t MresetCount = 0;
int32_t MmaxOffset = 0;
int32_t MOffset[100];
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void TimeSync_resetIEP(TimeSync_ParamsHandle_t timeSyncHandle)
{
    uint64_t doubleWord = 0;
    uint64_t reminder = 0;
    uint32_t word = 0;
    uint8_t *bytePtr;
    uintptr_t iepBaseAddress = (((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    /*First disable CMP1*/
    word = HW_RD_REG32(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    /*clear the CMP1 enable bit and store*/
    word &= ~((uint32_t)(4));
    HW_WR_REG32(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, word);

    /*Read IEP value and re-program CMP1*/
    bytePtr = (uint8_t *)(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0);
    memcpy(&doubleWord, bytePtr, 8);

    /*We need to make sure that IEP aligns with the start of second boundary
     * This is done by finding the distance to second boundary and setting cmp1
     * to that value. An extra second is added as a margin of safety*/
    reminder = doubleWord % (uint64_t)SEC_TO_NS;

    if(reminder != 0)
    {
        doubleWord += (SEC_TO_NS - reminder) + SEC_TO_NS;
    }

    /*now write this value to CMP1 register*/
    bytePtr = (uint8_t *)(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_CMP1_REG0);
    memcpy(bytePtr, &doubleWord, 8);

    bytePtr = (uint8_t *)(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase
                          +  TIMESYNC_CMP1_CMP_OFFSET);
    memcpy(bytePtr, &doubleWord, 8);

    /*Enable CMP1*/
    word = HW_RD_REG32(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    /*enable CMP1 bit and store back*/
    word |= 0x4;
    HW_WR_REG32(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, word);
}

void TimeSync_doFirstAdjustment(TimeSync_ParamsHandle_t timeSyncHandle,
                                uint8_t portNum)
{
    uint8_t *bytePtr = NULL;
    uint64_t syncRxTime = 0;
    uint64_t doubleWord = 0;
    uint64_t timeElapsed = 0;
    uint64_t iepCount = 0;
    uint32_t iepBaseAddress = (uint32_t)(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    /*Get origin time stamp in 64 bit format*/
    doubleWord = timeSyncHandle->syncParam[portNum - 1]->originTsSec *
                    (uint64_t)SEC_TO_NS + \
                    (uint64_t)timeSyncHandle->syncParam[portNum - 1]->originTsNs;

    /*Now calculate time elapsed since sync was received*/
    syncRxTime = timeSyncHandle->syncParam[portNum - 1]->rxTsSec *
                    (uint64_t)SEC_TO_NS + \
                    timeSyncHandle->syncParam[portNum - 1]->rxTs;
    bytePtr = (uint8_t *)((uint32_t)(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0));
    memcpy(&iepCount, bytePtr, 8);

    timeElapsed = iepCount - syncRxTime;
    /*Add correction field and peer delay*/
    doubleWord += (timeElapsed + timeSyncHandle->tsRunTimeVar->pathDelay[portNum -
                    1]) + \
                    timeSyncHandle->syncParam[portNum - 1]->correctionField;
    memcpy(bytePtr, &doubleWord, 8);

    /*CMP 1 values need to be reset in case of
        * 64 byte counter or else CMP1 will never hit*/
    TimeSync_resetIEP(timeSyncHandle);

}

void TimeSync_getTxTS(TimeSync_ParamsHandle_t timeSyncHandle, uint8_t portNum,
                      ptpFrameTypes_t frameType)
{
    uint8_t *ptpPDelayResFlwUpPacket = NULL;
    ICSS_EMAC_TxArgument txArg;
    uint32_t nanoseconds = 0;
    uint64_t seconds = 0;
    uint8_t oppPort = 0;
    uint8_t offset = timeSyncHandle->timeSyncConfig.frame_offset;

    /*Get the opposite port*/
    oppPort = ~portNum;
    oppPort &= 0x3;

    /*This gets called only in case of forced 2-step slave*/
    if(SYNC_FRAME == frameType)              /*Sync frame*/
    {
        /*Set event flag to indicate sync frame tx*/
        EventP_setBits(&(timeSyncHandle->ptpSendFollowUpEvtObject[oppPort - 1]), timeSyncHandle->eventIdSync);
    }

    if(DELAY_RESP_FRAME == frameType)              /*Pdelay response*/
    {
        ptpPDelayResFlwUpPacket =
            timeSyncHandle->timeSyncBuff.pdelayResFlwUp_TxBuf[portNum - 1];

        TimeSync_getTxTimestamp(timeSyncHandle, DELAY_RESP_FRAME, portNum,
                                &nanoseconds, &seconds);

        /*Encode actual Tx time in Pdelay response frame and send*/
        TimeSync_convEndianess(&seconds, ptpPDelayResFlwUpPacket + \
                      PTP_REQ_RCPT_TS_SEC_OFFSET - offset, 6);
        TimeSync_convEndianess(&nanoseconds, ptpPDelayResFlwUpPacket + \
                      PTP_REQ_RCPT_TS_NSEC_OFFSET - offset, 4);

        if(timeSyncHandle->timeSyncConfig.ll_has_hsrTag ||
                timeSyncHandle->timeSyncConfig.custom_tx_api)
        {
// #ifdef NEW_TX_CALLBACK
//             txArg.customFlag = 0;
// #endif //NEW_TX_CALLBACK
            txArg.icssEmacHandle = timeSyncHandle->emacHandle;
            txArg.lengthOfPacket = TIMESYNC_PDELAY_BUF_SIZE;
            txArg.portNumber = portNum;
            txArg.queuePriority = ICSS_EMAC_QUEUE1;
            txArg.srcAddress = ptpPDelayResFlwUpPacket;

            /*TODO: Review this*/
            /*Send pdelay response follow up frame out*/
            // ((((ICSS_EmacObject *)
            //    timeSyncHandle->emacHandle->object)->callBackHandle)->txCallBack)->callBack(
            //        &txArg,
            //        ((((ICSS_EmacObject *)
            //           timeSyncHandle->emacHandle->object)->callBackHandle)->txCallBack)->userArg);

            ICSS_EMAC_txPacket(&txArg, NULL);
        }
        else
        {
            txArg.icssEmacHandle = timeSyncHandle->emacHandle;
            txArg.lengthOfPacket = TIMESYNC_PDELAY_BUF_SIZE;
            txArg.portNumber = portNum;
            txArg.queuePriority = ICSS_EMAC_QUEUE1;
            txArg.srcAddress = ptpPDelayResFlwUpPacket;

            ICSS_EMAC_txPacket(&txArg, NULL);

            // ICSS_EmacTxPacketEnqueue(timeSyncHandle->emacHandle, ptpPDelayResFlwUpPacket,
            //                          portNum,
            //                          ICSS_EMAC_QUEUE1, TIMESYNC_PDELAY_BUF_SIZE);
        }

    }
}

void TimeSync_lineDelayCalc(TimeSync_ParamsHandle_t timeSyncHandle)
{

    /*The formula for calculation is
     * Mean Path Delay = (Master Delay - Slave Delay - Correction Path)/2.
     * Correction Path = Correction Path of (Delay Req + Sync + Follow Up(if present)*/

    /*Slave Delay = Tx timestamp of Delay Req packet - Rcv Timestamp of Sync Packet
     * Master Delay = Rcv Timestamp of Delay Resp - Precise origin timestamp of Follow Up Packet
     */

    uint64_t correctionField = 0, slaveDelay = 0, masterDelay = 0;
    uint64_t doubleWord = 0;
    uint8_t syncPortNum = timeSyncHandle->tsRunTimeVar->syncPortNum - 1;

    /*find total correction*/
    correctionField = timeSyncHandle->syncParam[syncPortNum]->correctionField +
                      timeSyncHandle->delayParams->correctionField;

    /*slave delay calculation*/
    slaveDelay = (timeSyncHandle->syncParam[syncPortNum]->rxTsSec *
                  (uint64_t)SEC_TO_NS) +
                 (uint64_t)timeSyncHandle->syncParam[syncPortNum]->rxTs;

    doubleWord = (timeSyncHandle->delayParams->delReqTxTsSec * (uint64_t)SEC_TO_NS
                  + (uint64_t)timeSyncHandle->delayParams->delReqTxTsNS);

    slaveDelay = doubleWord - slaveDelay;

    /*Do adjustment for MII Rx and Tx delay.
    * Here is the calculation :
    * delReqTxTS = delReqTxTS - Tx adjustment, rxTs = rxTs - Rx adjustment
    *  => delReqTxTS - rxTs = delReqTxTS - rxTs + Rx Adjustment - Tx Adjustment*/


    slaveDelay = (uint32_t)((double)slaveDelay * timeSyncHandle->tsSyntInfo->rcf);

    masterDelay = ((uint64_t)timeSyncHandle->delayParams->timeStampSec *
                   (uint64_t)SEC_TO_NS) +
                  (uint64_t)timeSyncHandle->delayParams->timeStampNS;
    masterDelay = masterDelay - (((uint64_t)
                                  timeSyncHandle->syncParam[syncPortNum]->originTsSec *
                                  (uint64_t)SEC_TO_NS) + (uint64_t)
                                 timeSyncHandle->syncParam[syncPortNum]->originTsNs);

    if(masterDelay > (slaveDelay + correctionField))
    {
        /*Add path delay minus correction divided by 2*/
        timeSyncHandle->tsRunTimeVar->meanPathDelay = (masterDelay - slaveDelay -
                correctionField)
                >> 1;
    }

    else
    {
        timeSyncHandle->tsRunTimeVar->meanPathDelay = 0;
    }

    /*First time so assign directly*/
    if((timeSyncHandle->tsRunTimeVar->stateMachine &
            TS_STATE_MACHINE_LINE_DELAY_COMPUTED) == 0)
    {
        timeSyncHandle->tsRunTimeVar->pathDelay[syncPortNum] =
            timeSyncHandle->tsRunTimeVar->meanPathDelay;
        /*Once delay has been computed notify the state machine*/
        timeSyncHandle->tsRunTimeVar->stateMachine |=
            TS_STATE_MACHINE_LINE_DELAY_COMPUTED;
    }

    else
    {
        /*Use exponential averaging filter to get the line delay*/
        timeSyncHandle->tsRunTimeVar->pathDelay[syncPortNum] =
            (uint32_t)((double)
                       timeSyncHandle->tsRunTimeVar->pathDelay[syncPortNum] * \
                       (double)FILTER_ALPHA_COEFF + (double)timeSyncHandle->tsRunTimeVar->meanPathDelay
                       * \
                       (double)(1 - FILTER_ALPHA_COEFF));
    }

}

void TimeSync_peerDelayCalc(TimeSync_ParamsHandle_t timeSyncHandle,
                            uint8_t twoStep, uint8_t portNum)
{

    uint8_t *bytePtr = NULL;

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

    /* delay on self (device) is T4 - T1 and delay on Peer is T3-T2
     * 2 x peer delay = (T4 - T1) - (T3 - T2)
     */

    uint8_t index = 0;
    uint64_t T3_T2_diff = 0;
    uint64_t T4_T1_diff = 0;

    uint64_t correctionFieldSum = 0;

    uint32_t sharedRAMbaseAddress = (uint32_t)((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase;

    index = portNum - 1;

    /*compute T4 - T1. Time difference on our device, take care of wrap around
     *Only nanoseconds used. Assumption is that transaction is completed within a second */

    if(timeSyncHandle->pDelayParams[index].T4Nsec <
            timeSyncHandle->pDelayParams[index].T1Nsec)
    {
        T4_T1_diff = ((uint64_t)SEC_TO_NS + (uint64_t)
                      timeSyncHandle->pDelayParams[index].T4Nsec)
                     - (uint64_t)timeSyncHandle->pDelayParams[index].T1Nsec;
    }

    else
    {
        T4_T1_diff = timeSyncHandle->pDelayParams[index].T4Nsec -
                     timeSyncHandle->pDelayParams[index].T1Nsec;
    }

    /*Multiply device time with NRR*/
    T4_T1_diff = (uint32_t)((double)T4_T1_diff *
                            timeSyncHandle->tsNrrInfo[index]->nrr);

    /*Special processing if 2-step*/
    if(twoStep == 1)
    {

        /*compute T3-T2*. Time difference on Peer*/
        if(timeSyncHandle->pDelayParams[index].T3Nsec <
                timeSyncHandle->pDelayParams[index].T2Nsec)
        {
            T3_T2_diff = ((uint64_t)SEC_TO_NS +
                          (uint64_t)timeSyncHandle->pDelayParams[index].T3Nsec) -
                         (uint64_t)timeSyncHandle->pDelayParams[index].T2Nsec;
        }

        else
        {
            T3_T2_diff = timeSyncHandle->pDelayParams[index].T3Nsec -
                         timeSyncHandle->pDelayParams[index].T2Nsec;
        }

        /*Compute sum of correction fields*/
        correctionFieldSum = timeSyncHandle->pDelayParams[index].delayResCorrField +
                             timeSyncHandle->pDelayParams[index].delayResFwUpCorrField;

        if(T4_T1_diff >= (T3_T2_diff + correctionFieldSum))
        {
            timeSyncHandle->tsRunTimeVar->meanPathDelay = T4_T1_diff - T3_T2_diff -
                    correctionFieldSum;
            /*Average the delay*/
            timeSyncHandle->tsRunTimeVar->meanPathDelay >>= 1;
        }

        else
        {
            timeSyncHandle->tsRunTimeVar->meanPathDelay = 0;
        }

    }

    else
    {
        /*compute T3-T2*. Time difference on Peer*/
        T3_T2_diff =  timeSyncHandle->pDelayParams[index].delayResCorrField;

        if(T4_T1_diff > T3_T2_diff)
        {
            timeSyncHandle->tsRunTimeVar->meanPathDelay = T4_T1_diff - T3_T2_diff;
            /*Average the delay*/
            timeSyncHandle->tsRunTimeVar->meanPathDelay >>= 1;
        }

        else
        {
            timeSyncHandle->tsRunTimeVar->meanPathDelay = 0;
        }

    }

    if(timeSyncHandle->tsRunTimeVar->meanPathDelay >
            TIMESYNC_PEER_DELAY_ERROR_THRESHOLD)
    {
        timeSyncHandle->tsRunTimeVar->meanPathDelay = 0;
    }

    if((timeSyncHandle->tsRunTimeVar->stateMachine &
            TS_STATE_MACHINE_LINE_DELAY_COMPUTED) == 0)
    {
        timeSyncHandle->tsRunTimeVar->pathDelay[index] =
            timeSyncHandle->tsRunTimeVar->meanPathDelay;

        /*Once delay has been computed notify the state machine*/
        timeSyncHandle->tsRunTimeVar->stateMachine |=
            TS_STATE_MACHINE_LINE_DELAY_COMPUTED;
    }

    else
    {
        /*Use exponential averaging filter to get the peer delay*/
        timeSyncHandle->tsRunTimeVar->pathDelay[index] =
            (uint32_t)((double)timeSyncHandle->tsRunTimeVar->pathDelay[index] *
                       (double)FILTER_ALPHA_COEFF + \
                       (double)timeSyncHandle->tsRunTimeVar->meanPathDelay * (double)(
                           1 - FILTER_ALPHA_COEFF));
    }

    if(ICSS_EMAC_PORT_1 == portNum)
    {
        bytePtr = (uint8_t *)(sharedRAMbaseAddress + P1_SMA_LINE_DELAY_OFFSET);
    }

    else
    {
        bytePtr = (uint8_t *)(sharedRAMbaseAddress + P2_SMA_LINE_DELAY_OFFSET);
    }

    /*For 1-step TC delay asymmetry also gets added to the Sync frame correction
     * This is done by adding it to the path delay written in shared RAM*/
    timeSyncHandle->tsRunTimeVar->pathDelay[index] +=
        timeSyncHandle->timeSyncConfig.asymmetryCorrection[index];
    memcpy(bytePtr, &(timeSyncHandle->tsRunTimeVar->pathDelay[index]), 4);

}

void TimeSync_synchronizeClock(TimeSync_ParamsHandle_t timeSyncHandle)
{
    uint8_t syncPortNum = 0;
    uint32_t meanPathDelay = 0;
    int64_t tempVar1 = 0;
    int64_t tempVar2 = 0;
    double tempVar3 = 0;
    int32_t adjOffset = 0;
    double variance = 0.0;

    syncPortNum = timeSyncHandle->tsRunTimeVar->syncPortNum - 1;
    /*Once initial adjustment is done, calculate the offset*/
    /*Get mean path delay*/
    meanPathDelay =
        timeSyncHandle->tsRunTimeVar->pathDelay[syncPortNum];

    if(timeSyncHandle->syncParam[syncPortNum]->originTsSec ==
            timeSyncHandle->syncParam[syncPortNum]->rxTsSec)
    {
        timeSyncHandle->tsRunTimeVar->currOffset =
            timeSyncHandle->syncParam[syncPortNum]->rxTs -
            timeSyncHandle->syncParam[syncPortNum]->originTsNs - \
            meanPathDelay - timeSyncHandle->syncParam[syncPortNum]->correctionField;
    }

    else
    {
        tempVar1 = ((int64_t)timeSyncHandle->syncParam[syncPortNum]->rxTsSec - (int64_t)
                    timeSyncHandle->syncParam[syncPortNum]->originTsSec) \
                   * (int64_t)SEC_TO_NS;
        tempVar1 += (int64_t)timeSyncHandle->syncParam[syncPortNum]->rxTs;
        tempVar2 = (int64_t)(timeSyncHandle->syncParam[syncPortNum]->originTsNs +
                             meanPathDelay +
                             timeSyncHandle->syncParam[syncPortNum]->correctionField);

        timeSyncHandle->tsRunTimeVar->currOffset = (int32_t)(tempVar1 - tempVar2);

    }

    /*Take running average of the offset*/

    if(timeSyncHandle->tsRunTimeVar->ltaOffsetValid)
    {
        timeSyncHandle->tsRunTimeVar->ltaOffset = (int32_t)((double)(
                    FILTER_ALPHA_COEFF) *
                (double)timeSyncHandle->tsRunTimeVar->ltaOffset + (double)(
                    1 - FILTER_ALPHA_COEFF) *
                (double)timeSyncHandle->tsRunTimeVar->currOffset);
    }
    else
    {
        timeSyncHandle->tsRunTimeVar->ltaOffset = timeSyncHandle->tsRunTimeVar->currOffset;
        timeSyncHandle->tsRunTimeVar->ltaOffsetValid = 1;
    }


    if(timeSyncHandle->tsRunTimeVar->driftStable)
    {
        adjOffset = timeSyncHandle->tsRunTimeVar->currOffset +
                    timeSyncHandle->tsRunTimeVar->ltaOffset  +
                    timeSyncHandle->tsRunTimeVar->initialOffset;
    }
    else
    {
        adjOffset = timeSyncHandle->tsRunTimeVar->currOffset;
    }

    /*Do adjustment*/
    TimeSync_adjTimeSlowComp(timeSyncHandle, adjOffset);

#ifdef TIMESYNC_LOCAL_DEBUG
    MoriginTsSec[Mindex] = (uint32_t)(timeSyncHandle->syncParam[syncPortNum]->originTsSec);
    MoriginTsNs[Mindex] = (uint32_t)(timeSyncHandle->syncParam[syncPortNum]->originTsNs);
    MrxTsSec[Mindex] = (uint32_t)(timeSyncHandle->syncParam[syncPortNum]->rxTsSec);
    MrxTs[Mindex] = (uint32_t)(timeSyncHandle->syncParam[syncPortNum]->rxTs);
    McorrectionField[Mindex] = (uint32_t)(timeSyncHandle->syncParam[syncPortNum]->correctionField);
    MdelReqTxTsSec[Mindex] = timeSyncHandle->delayParams->delReqTxTsSec;
    MdelReqTxTsNS[Mindex] = timeSyncHandle->delayParams->delReqTxTsNS;
    MtimeStampSec[Mindex] = timeSyncHandle->delayParams->timeStampSec;
    MtimeStampNS[Mindex] = timeSyncHandle->delayParams->timeStampNS;
    MdelayRespCorrection[Mindex] = timeSyncHandle->delayParams->correctionField;
    McurrOffset[Mindex] = (uint32_t)(timeSyncHandle->tsRunTimeVar->currOffset);
    MadjOffset[Mindex] = (adjOffset);
    MltaOffset[Mindex] = (uint32_t)(timeSyncHandle->tsRunTimeVar->ltaOffset);
    MinitialOffset[Mindex] = (uint32_t)(timeSyncHandle->tsRunTimeVar->initialOffset);
    MprevOffset0[Mindex] = (uint32_t)(timeSyncHandle->tsRunTimeVar->prevOffset[0]);
    MprevOffset1[Mindex] = (uint32_t)(timeSyncHandle->tsRunTimeVar->prevOffset[1]);
    McurrSyncInterval[Mindex] = (uint32_t)(timeSyncHandle->tsRunTimeVar->currSyncInterval);
    MltaSyncInterval[Mindex] = (uint32_t)(timeSyncHandle->tsRunTimeVar->ltaSyncInterval);
    MfirstSyncInterval[Mindex] = (uint32_t)(timeSyncHandle->tsRunTimeVar->firstSyncInterval);
    MmeanPathDelay[Mindex] = meanPathDelay;
    MstateMachine[Mindex] = timeSyncHandle->tsRunTimeVar->stateMachine;
    MdriftStable[Mindex] = timeSyncHandle->tsRunTimeVar->driftStable;
    MoffsetStable[Mindex] = timeSyncHandle->tsRunTimeVar->offsetStable;
    Mrcf[Mindex] = timeSyncHandle->tsSyntInfo->rcf;
    Mindex++;
    if(Mindex == MSIZE)
    {
        Mindex = 0;
    }
#endif

    /*Check this condition only if device is in sync*/
    if((timeSyncHandle->tsRunTimeVar->stateMachine &
            TS_STATE_MACHINE_DEVICE_IN_SYNC) ==
            TS_STATE_MACHINE_DEVICE_IN_SYNC)
    {
#ifdef TIMESYNC_LOCAL_DEBUG
        if(abs(timeSyncHandle->tsRunTimeVar->currOffset) > abs(MmaxOffset))
        {
            MmaxOffset = timeSyncHandle->tsRunTimeVar->currOffset;
        }
        MOffset[(abs(timeSyncHandle->tsRunTimeVar->currOffset))/1000]++;
#endif
        /*This is to track any large change in the timebase*/
        if(abs(timeSyncHandle->tsRunTimeVar->currOffset) > OFFSET_THRESHOLD_FOR_RESET)
        {
#ifdef TIMESYNC_LOCAL_DEBUG
            MresetCount++;
#endif
            /*Reset state machine*/
            TimeSync_reset(timeSyncHandle);
            return;
        }
    }

    /*Calculate observed offset scaled log variance as per section 7.6.3.3 in IEEE standard 1588 (2018)*/
    if(timeSyncHandle->tsRunTimeVar->prevOffsetValid[1])
    {
        /* variance is equal to (1/3)*(1/2)*(1/N-2)*(sum of (x(k+2) - 2*x(k+1) + x(k))^2).
           where x(k+2), x(k+1) and x(k) are observed offsets. N = 3 for the calculations
           below as previous 3 samples are used */
        tempVar3 = (timeSyncHandle->tsRunTimeVar->currOffset - (2 * timeSyncHandle->tsRunTimeVar->prevOffset[0]) + timeSyncHandle->tsRunTimeVar->prevOffset[1])/(double)(1000000000.0);
        variance = (double)(tempVar3 * tempVar3)/6.0;

        /* For observedOffsetScaledLogVariance calculation, we take logarithm, multiply by 2^8 and add 0x8000*/
        timeSyncHandle->tsRunTimeVar->parentParams.observedOffsetScaledLogVariance = (uint16_t)((int16_t)((double)log2(variance) * (0x100)) + 0x8000);
    }

    /*The code below is part of Statistics collection
     * some of which is used to implement the stabilization algo
     */

    /*Find clock drift*/
    if(timeSyncHandle->tsRunTimeVar->prevOffsetValid[0])
    {
        timeSyncHandle->tsRunTimeVar->clockDrift += abs(
                    timeSyncHandle->tsRunTimeVar->currOffset -
                    timeSyncHandle->tsRunTimeVar->prevOffset[0]);
        timeSyncHandle->tsRunTimeVar->clockDrift =
            timeSyncHandle->tsRunTimeVar->clockDrift / 2;
    }

    /*set prevoffset to current once drift is computed*/
    if(timeSyncHandle->tsRunTimeVar->prevOffsetValid[0])
    {
        timeSyncHandle->tsRunTimeVar->prevOffset[1] = timeSyncHandle->tsRunTimeVar->prevOffset[0];
        timeSyncHandle->tsRunTimeVar->prevOffsetValid[1] = 1;
    }

    timeSyncHandle->tsRunTimeVar->prevOffset[0] = timeSyncHandle->tsRunTimeVar->currOffset;
    timeSyncHandle->tsRunTimeVar->prevOffsetValid[0] = 1;

    /*Wait for the drift to stabilize*/
    if(timeSyncHandle->tsRunTimeVar->clockDrift <= STABLE_FILTER_THRESHOLD
            && (timeSyncHandle->tsRunTimeVar->driftStable == 0))
    {
        timeSyncHandle->tsRunTimeVar->driftStable = 1;
        timeSyncHandle->tsRunTimeVar->initialOffset =
            timeSyncHandle->tsRunTimeVar->currOffset;
    }

    /*Wait for offset to become zero*/
    if(abs(timeSyncHandle->tsRunTimeVar->ltaOffset) <= STABLE_FILTER_THRESHOLD
            && (timeSyncHandle->tsRunTimeVar->offsetStable == 0) &&
            (timeSyncHandle->tsRunTimeVar->driftStable))
    {
        timeSyncHandle->tsRunTimeVar->offsetStable = 1;
        /*Indicate that the device is in sync now*/
        timeSyncHandle->tsRunTimeVar->stateMachine |= TS_STATE_MACHINE_DEVICE_IN_SYNC;
    }

    /*The stabilization logic collects offsets which have a low drift and are
     * clustered together. The idea is that the PPM of the crystal might have shifted
     * and since we are relying on the initial offset measurement to do zero offset correction
     * we need to constantly track this zero offset value or else the PPM might settle into a zone
     * with high value and low drift.
     *
     * To take an example let's say we sync to a master with a fixed offset of 1000 ns.
     * The DUT as a slave will detect this value in the variable timeSyncHandle->tsRunTimeVar->initialOffset
     * After some time, let's say this offset has become 1100 ns. In the absence of this logic the
     * slave will settle into a zone with 100ns offset. If later the offset becomes 1200 ns then DUT
     * will settle into a zone with 200ns offset*/

    /*Run the logic only once we are stable*/
    if(timeSyncHandle->tsRunTimeVar->offsetStable)
    {
        timeSyncHandle->offsetAlgo->lastSeen_good_drift_index++;

        /*If the drift is below our threshold and we have a close cluster
         * then we use this value for our averaging purpose
         */
        if((timeSyncHandle->tsRunTimeVar->clockDrift <
                timeSyncHandle->offsetAlgo->driftThreshold)
                && (timeSyncHandle->offsetAlgo->lastSeen_good_drift_index <
                    OFFSET_ALGO_CLUSTER_SIZE))
        {
            timeSyncHandle->offsetAlgo->lastSeen_good_drift_index = 0;

            if(timeSyncHandle->offsetAlgo->num_entries_index < OFFSET_ALGO_BIN_SIZE)
            {
                /*Store the value for averaging later*/
                timeSyncHandle->offsetAlgo->correction[timeSyncHandle->offsetAlgo->num_entries_index++]
                    =  timeSyncHandle->tsRunTimeVar->currOffset;
            }

            else
            {
                timeSyncHandle->offsetAlgo->binFull = 1;
            }

        }

        /*If cluster is broken then we reset the counters*/
        else if((timeSyncHandle->offsetAlgo->lastSeen_good_drift_index >=
                 OFFSET_ALGO_CLUSTER_SIZE)
                && (timeSyncHandle->offsetAlgo->binFull != 1))
        {
            /*reset the count if we can't find a good match in our window*/
            timeSyncHandle->offsetAlgo->lastSeen_good_drift_index = 0;
            timeSyncHandle->offsetAlgo->num_entries_index = 0;
        }
    }


}

void TimeSync_updateNRRParams(TimeSync_ParamsHandle_t timeSyncHandle,
                              uint8_t portNum)
{

    uint8_t curIndex = 0;

    timeSync_NrrInfo_t *nrrInfo = NULL;
    /*point to the nrr for correct port*/
    nrrInfo = timeSyncHandle->tsNrrInfo[portNum - 1];

    curIndex = (nrrInfo->nrrIndex++) % 3;

    /*Please note that the RCF calculated here is inverse of the RCF described in spec
     * This is because we multiply our time adjustments with this value
     */

    /*This is used later*/
    nrrInfo->curIndex = curIndex;

    /*Populate the array with TS*/
    nrrInfo->deviceRxTS[curIndex] = (uint64_t)
                                    timeSyncHandle->pDelayParams[portNum - 1].T4Sec *
                                    (uint64_t)SEC_TO_NS + timeSyncHandle->pDelayParams[portNum - 1].T4Nsec;

    nrrInfo->correctedPeerTS[curIndex] = \
                                         (uint64_t)timeSyncHandle->pDelayParams[portNum - 1].T3Sec * (uint64_t)SEC_TO_NS
                                         + timeSyncHandle->pDelayParams[portNum - 1].T3Nsec +
                                         timeSyncHandle->tsRunTimeVar->pathDelay[portNum - 1] +
                                         timeSyncHandle->pDelayParams[portNum - 1].delayResCorrField + \
                                         timeSyncHandle->pDelayParams[portNum - 1].delayResFwUpCorrField;

    /*Wait for TS array to be full before starting syntonization*/
    if((!nrrInfo->nrrEnable)
            && (nrrInfo->nrrIndex == (SYNT_DEPTH + 2)))
    {
        nrrInfo->nrrEnable = 1;
    }

}

void TimeSync_calcNRR(TimeSync_ParamsHandle_t timeSyncHandle, uint8_t portNum)
{
    uint64_t num = 0;
    uint64_t den = 0;

    uint8_t curIndex = 0;
    uint8_t prevIndex = 0;

    timeSync_NrrInfo_t *nrrInfo = NULL;
    /*point to the nrr for correct port*/
    nrrInfo = timeSyncHandle->tsNrrInfo[portNum - 1];

    curIndex = nrrInfo->curIndex;

    /*calculate nrr */
    if(nrrInfo->nrrEnable)
    {
        prevIndex = timeSyncHandle->syntIndexMap[curIndex];
        num = nrrInfo->correctedPeerTS[curIndex] -
              nrrInfo->correctedPeerTS[prevIndex];

        den = nrrInfo->deviceRxTS[curIndex] -
              nrrInfo->deviceRxTS[prevIndex];

        /*calculate rcf*/
        nrrInfo->nrr = ((double)num) / den;

    }
}

void TimeSync_calcRcfAndSyncInterval(TimeSync_ParamsHandle_t timeSyncHandle)
{

    uint64_t num = 0;
    uint64_t den = 0;
    uint32_t scaledRCF = 0;
    uint8_t *ptrScaledRcf = NULL;

    uint8_t curIndex = 0;
    uint8_t prevIndex = 0;
    uint8_t tempIndex = 0;
    uint8_t count = 0;
    uint8_t syncPortNum = 0;

    uint64_t syncIntervalAverage = 0;

    double scalingFactor = 0;

    ptrScaledRcf = (uint8_t *)(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase
                               +  TIMESYNC_TC_RCF_OFFSET);

    curIndex = (timeSyncHandle->tsSyntInfo->syntIndex++) % 3;

    syncPortNum = timeSyncHandle->tsRunTimeVar->syncPortNum - 1;

    /*Please note that the RCF calculated here is inverse of the RCF described in spec
     * This is because we multiply our time adjustments with this value
     */

    /*Populate the array with TS*/
    timeSyncHandle->tsSyntInfo->syncIngressTs[curIndex] = (uint64_t)
            timeSyncHandle->syncParam[syncPortNum]->rxTsSec *
            (uint64_t)SEC_TO_NS + timeSyncHandle->syncParam[syncPortNum]->rxTs;

    timeSyncHandle->tsSyntInfo->correctedMasterTs[curIndex] = \
            (uint64_t)timeSyncHandle->syncParam[syncPortNum]->originTsSec *
            (uint64_t)SEC_TO_NS
            + timeSyncHandle->syncParam[syncPortNum]->originTsNs +
            timeSyncHandle->tsRunTimeVar->pathDelay[timeSyncHandle->tsRunTimeVar->syncPortNum
                    - 1] +
            timeSyncHandle->syncParam[syncPortNum]->correctionField;

    /*Wait for TS array to be full before starting syntonization*/
    if((!timeSyncHandle->tsSyntInfo->syntEnable)
            && (timeSyncHandle->tsSyntInfo->syntIndex == (SYNT_DEPTH + 2)))
    {
        timeSyncHandle->tsSyntInfo->syntEnable = 1;
    }

    /*calculate rcf */
    if(timeSyncHandle->tsSyntInfo->syntEnable)
    {
        prevIndex = timeSyncHandle->syntIndexMap[curIndex];
        num = timeSyncHandle->tsSyntInfo->correctedMasterTs[curIndex] -
              timeSyncHandle->tsSyntInfo->correctedMasterTs[prevIndex];

        den = timeSyncHandle->tsSyntInfo->syncIngressTs[curIndex] -
              timeSyncHandle->tsSyntInfo->syncIngressTs[prevIndex];

        /*calculate rcf*/
        timeSyncHandle->tsSyntInfo->rcf = ((double)num) / den;
        scaledRCF = (uint32_t)(timeSyncHandle->tsSyntInfo->rcf * 1024);
        /* write into ICSS memory for firmware */
        memcpy(ptrScaledRcf, &scaledRCF, 4);

        /* Calculate observed phase change rate as per section 7.6.4.4 in IEEE standard 1588 (2008).
           It is calculated as fractional frequency offset multiplied by 2^40*/
        timeSyncHandle->tsRunTimeVar->parentParams.observedClockPhaseChangeRate = (int32_t)((double)(timeSyncHandle->tsSyntInfo->rcf - 1.0) * (0x10000000000));

        /*calculate average of difference between two successive sync frames*/
        for(count = 0; count < 3; count++)
        {

            tempIndex = (curIndex + count) % SYNT_DEPTH;
            prevIndex = timeSyncHandle->prevIndexMap[tempIndex];

            if(timeSyncHandle->tsSyntInfo->correctedMasterTs[tempIndex] >
                    timeSyncHandle->tsSyntInfo->correctedMasterTs[prevIndex])
            {
                syncIntervalAverage += (timeSyncHandle->tsSyntInfo->correctedMasterTs[tempIndex]
                                        -
                                        timeSyncHandle->tsSyntInfo->correctedMasterTs[prevIndex]);
            }
        }

        syncIntervalAverage = syncIntervalAverage / 2;

        timeSyncHandle->tsRunTimeVar->currSyncInterval = syncIntervalAverage;

        if(timeSyncHandle->tsRunTimeVar->ltaSyncInterval == 0)
        {
            timeSyncHandle->tsRunTimeVar->ltaSyncInterval =
                timeSyncHandle->tsRunTimeVar->currSyncInterval;
            timeSyncHandle->tsRunTimeVar->firstSyncInterval =
                timeSyncHandle->tsRunTimeVar->currSyncInterval;

            /*Add 50% extra so minor variations in master don't cause a timeout*/
            timeSyncHandle->tsRunTimeVar->syncTimeoutInterval =
                timeSyncHandle->tsRunTimeVar->currSyncInterval;
            timeSyncHandle->tsRunTimeVar->syncTimeoutInterval +=
                timeSyncHandle->tsRunTimeVar->currSyncInterval >> 1;

            scalingFactor = ((double)1000 / (double)(
                                 timeSyncHandle->timeSyncConfig.tickPeriod));

            /*Multiply/Divide by clock scaling factor*/
            timeSyncHandle->tsRunTimeVar->syncTimeoutInterval =
                (uint32_t)((double)timeSyncHandle->tsRunTimeVar->syncTimeoutInterval *
                           scalingFactor);

            /*convert in terms of PTP BG ticks*/
            timeSyncHandle->tsRunTimeVar->syncTimeoutInterval =
                (uint32_t)((double)timeSyncHandle->tsRunTimeVar->syncTimeoutInterval / (double)(
                               PTP_BG_TASK_TICK_PERIOD * 1000000));

        }

        else
        {
            timeSyncHandle->tsRunTimeVar->ltaSyncInterval =
                (uint32_t)(((double)(timeSyncHandle->tsRunTimeVar->ltaSyncInterval) *
                            FILTER_ALPHA_COEFF) \
                           + ((double)(1 - FILTER_ALPHA_COEFF) * (double)(
                                  timeSyncHandle->tsRunTimeVar->currSyncInterval)));

        }

        timeSyncHandle->syncInterval = timeSyncHandle->tsRunTimeVar->ltaSyncInterval;

        /*Once sync interval has been calculated notify the state machine*/
        timeSyncHandle->tsRunTimeVar->stateMachine |=
            TS_STATE_MACHINE_SYNC_INTERVAL_COMPUTED;

    }

}

void TimeSync_processPTPFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                              uint8_t *pktBuffer, \
                              uint8_t portNum, uint16_t size, uint8_t isLinkLocal)
{
    uint8_t pktType = 0;
    uint8_t ifTwoStep = 0;
    uint8_t offset = 0;
    uint8_t *bytePtr = 0;
    uint32_t nanoseconds = 0;
    uint64_t seconds = 0;
    uint64_t ptpFlags = 0;
    uint8_t domainNumber = 0;
    uint8_t *TimeStampFromFrame =0;

    offset = timeSyncHandle->timeSyncConfig.frame_offset;

    if(TRUE == timeSyncHandle->timeSyncConfig.hsrEnabled)
    {
        if(isLinkLocal)
        {
            /*Read the flag value from shared RAM*/
            bytePtr = (uint8_t *)(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase
                                  + LINK_LOCAL_FRAME_HAS_HSR_TAG);
            timeSyncHandle->timeSyncConfig.ll_has_hsrTag = *bytePtr;

            if(timeSyncHandle->timeSyncConfig.ll_has_hsrTag)
            {
                offset -= HSR_CORRECTION;
            }
        }

        else
        {
            offset -= HSR_CORRECTION;
        }
    }


    pktType = (uint8_t)(*(pktBuffer + PTP_MSG_ID_OFFSET - offset));
    /* Message ID is a 4 bit value */
    pktType &= 0xF;

    ifTwoStep = (uint8_t)(*(pktBuffer + PTP_FLAG_OFFSET - offset));
    ifTwoStep &= 2;

    domainNumber = (uint8_t)(*(pktBuffer + PTP_DOMAIN_NUM_OFFSET - offset));

    if(domainNumber != timeSyncHandle->timeSyncConfig.domainNumber[0])
    {
        return;
    }

    /*If timestamp is appended to packet, copy them here*/
    if(!timeSyncHandle->timeSyncConfig.timestamp_from_shared_ram)
    {
        TimeStampFromFrame = pktBuffer + size;
        TimeSync_getRxTimestampFromFrame(timeSyncHandle, portNum, \
                                        &nanoseconds, &seconds, TimeStampFromFrame);
    }

    if((PTP_ANNOUNCE_MSG_ID == pktType) || (PTP_MGMT_MSG_ID == pktType))
    {
        /*debug, remove in final code*/
        if(PTP_ANNOUNCE_MSG_ID == pktType)
        {

            /* Store the masterParams */
            memcpy(&(timeSyncHandle->tsRunTimeVar->masterParams.gmIdentity[0]), pktBuffer + PTP_GM_CLK_IDENTITY_OFFSET, PTP_GM_CLK_IDENTITY_SIZE);
            memcpy(&(timeSyncHandle->tsRunTimeVar->masterParams.clockClass), pktBuffer + PTP_GM_CLK_CLASS_OFFSET, PTP_GM_CLK_CLASS_SIZE);
            memcpy(&(timeSyncHandle->tsRunTimeVar->masterParams.clockAccuracy), pktBuffer + PTP_GM_CLK_ACCU_OFFSET, PTP_GM_CLK_ACCU_SIZE);
            TimeSync_convEndianess(pktBuffer + PTP_GM_CLK_VARIANCE_OFFSET, &(timeSyncHandle->tsRunTimeVar->masterParams.clockVariance), PTP_GM_CLK_VARIANCE_SIZE);
            TimeSync_convEndianess(pktBuffer + PTP_UTC_OFFSET, &(timeSyncHandle->tsRunTimeVar->masterParams.UTCOffset), PTP_UTC_SIZE);
            TimeSync_convEndianess(pktBuffer + PTP_FLAG_OFFSET, &(ptpFlags), PTP_FLAG_SIZE);
            memset(&(timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags), 0, sizeof(timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags));
            if(ptpFlags & PTP_LEAP_61_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_LEAP_61_INDEX] = 1;
            }
            if(ptpFlags & PTP_LEAP_59_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_LEAP_59_INDEX] = 1;
            }
            if(ptpFlags & PTP_UTC_REASONABLE_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_UTC_REASONABLE_INDEX] = 1;
            }
            if(ptpFlags & PTP_TIMESCALE_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_PTP_TIMESCALE_INDEX] = 1;
            }
            if(ptpFlags & PTP_TIME_TRACEABLE_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_TIME_TRACEABLE_INDEX] = 1;
            }
            if(ptpFlags & PTP_FREQ_TRACEABLE_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_FREQ_TRACEABLE_INDEX] = 1;
            }
            if(ptpFlags & PTP_ALTERNATE_MASTER_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_ALTERNATE_MASTER_INDEX] = 1;
            }
            if(ptpFlags & PTP_UNICAST_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_PTP_UNICAST] = 1;
            }
            if(ptpFlags & PTP_PROFILE_SPECIFIC_1_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_PROFILE_SPECIFIC_1_INDEX] = 1;
            }
            if(ptpFlags & PTP_PROFILE_SPECIFIC_2_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_PROFILE_SPECIFIC_2_INDEX] = 1;
            }
            if(ptpFlags & PTP_SECURITY_MASK)
            {
                timeSyncHandle->tsRunTimeVar->masterParams.ptp_flags[TS_PTP_SECURITY_INDEX] = 1;
            }
            memcpy(&(timeSyncHandle->tsRunTimeVar->masterParams.timeSource), pktBuffer + PTP_TIME_SRC_OFFSET, PTP_TIME_SRC_SIZE);
            memcpy(&(timeSyncHandle->tsRunTimeVar->masterParams.priority1), pktBuffer + PTP_PRIORITY1_OFFSET, PTP_PRIORITY1_SIZE);
            memcpy(&(timeSyncHandle->tsRunTimeVar->masterParams.priority2), pktBuffer + PTP_PRIORITY2_OFFSET, PTP_PRIORITY2_SIZE);
            TimeSync_convEndianess(pktBuffer + PTP_STEPS_REMOVED_OFFSET, &(timeSyncHandle->tsRunTimeVar->masterParams.stepRemoved), PTP_STEPS_REMOVED_SIZE);

            /* Store the parentParams */
            memcpy(&(timeSyncHandle->tsRunTimeVar->parentParams.clockIdentity[0]), pktBuffer + PTP_SRC_CLK_IDENTITY, PTP_SRC_CLK_IDENTITY_SIZE);
            TimeSync_convEndianess(pktBuffer + PTP_SRC_PORT_ID_OFFSET, &(timeSyncHandle->tsRunTimeVar->parentParams.ptpPortNumber), PTP_SRC_PORT_ID_SIZE);

            /*Just parse the Announce frame for MAC ID
             * and make it master
             */
            TimeSync_dummyBMCA(timeSyncHandle, pktBuffer);
            return;
        }

        /*Should we use the port number???*/
        if(!(timeSyncHandle->stackParams.generalFrameFlag)) /*Make sure the previous packet is transferred to PTP Stack*/
        {
            timeSyncHandle->stackParams.ptpGeneralSize = size;

            if(TRUE == timeSyncHandle->timeSyncConfig.hsrEnabled)
            {
                timeSyncHandle->stackParams.ptpGeneralSize -= HSR_CORRECTION;
                memcpy(timeSyncHandle->stackParams.ptpGeneralFrame, pktBuffer,
                       SRC_DST_MAC_SIZE);

                if(((timeSyncHandle->stackParams.ptpGeneralSize - SRC_DST_MAC_SIZE) > 0) &&
                   (timeSyncHandle->stackParams.ptpGeneralSize <= (ICSS_EMAC_MAXMTU)))
                {
                    memcpy(timeSyncHandle->stackParams.ptpGeneralFrame + SRC_DST_MAC_SIZE,
                        pktBuffer + SRC_DST_MAC_SIZE + HSR_CORRECTION, \
                        timeSyncHandle->stackParams.ptpGeneralSize - SRC_DST_MAC_SIZE);
                }
            }
            else if(timeSyncHandle->stackParams.ptpGeneralSize <= (ICSS_EMAC_MAXMTU))
            {
                memcpy(timeSyncHandle->stackParams.ptpGeneralFrame, pktBuffer,
                       timeSyncHandle->stackParams.ptpGeneralSize);
            }

            timeSyncHandle->stackParams.generalFrameFlag = 1;
        }

    }

    else
    {
        if(PTP_PDLY_REQ_MSG_ID == pktType)         /*Pdelay request frame*/
        {
            if(timeSyncHandle->timeSyncConfig.timestamp_from_shared_ram)
            {
                TimeSync_getRxTimestamp(timeSyncHandle, DELAY_REQ_FRAME, portNum, \
                                        &nanoseconds, &seconds);
            }

            timeSyncHandle->pDelayParams[portNum - 1].pDelayReqRcvdTSSec =
                seconds;
            timeSyncHandle->pDelayParams[portNum - 1].pDelayReqRcvdTSNsec =
                nanoseconds;

            /*Copy into buffer and post event*/

            /*Remove HSR tag*/
            if(timeSyncHandle->timeSyncConfig.ll_has_hsrTag)
            {
                memcpy(timeSyncHandle->timeSyncBuff.pdelayReq_RxBuf[portNum - 1], pktBuffer,
                       SRC_DST_MAC_SIZE);
                memcpy(timeSyncHandle->timeSyncBuff.pdelayReq_RxBuf[portNum - 1] +
                       SRC_DST_MAC_SIZE,
                       pktBuffer + SRC_DST_MAC_SIZE + HSR_CORRECTION, \
                       TIMESYNC_PDELAY_BUF_SIZE - SRC_DST_MAC_SIZE);
            }

            else
            {
                memcpy(timeSyncHandle->timeSyncBuff.pdelayReq_RxBuf[portNum - 1],
                       pktBuffer, TIMESYNC_PDELAY_BUF_SIZE);
            }

            EventP_setBits(&(timeSyncHandle->ptpPdelayReqEvtObject[portNum - 1]), timeSyncHandle->eventIdPdelayReq);
        }

        else if(PTP_PDLY_RSP_MSG_ID == pktType)         /*Pdelay response frame*/
        {
            if(timeSyncHandle->timeSyncConfig.timestamp_from_shared_ram)
            {
                TimeSync_getRxTimestamp(timeSyncHandle, DELAY_RESP_FRAME, portNum, \
                                        &nanoseconds, &seconds);
            }

            /*T4 timestamp*/
            timeSyncHandle->pDelayParams[portNum - 1].T4Sec =
                seconds;
            timeSyncHandle->pDelayParams[portNum - 1].T4Nsec =
                nanoseconds;

            /*set two step flag. Used in another task*/
            timeSyncHandle->pDelayParams[portNum - 1].ifTwoStep = ifTwoStep;

            if(timeSyncHandle->timeSyncConfig.ll_has_hsrTag)
            {
                memcpy(timeSyncHandle->timeSyncBuff.pdelayRes_RxBuf[portNum - 1],
                       pktBuffer,
                       SRC_DST_MAC_SIZE);
                memcpy(timeSyncHandle->timeSyncBuff.pdelayRes_RxBuf[portNum - 1] +
                       SRC_DST_MAC_SIZE,
                       pktBuffer + SRC_DST_MAC_SIZE + HSR_CORRECTION, \
                       TIMESYNC_PDELAY_BUF_SIZE - SRC_DST_MAC_SIZE);
            }

            else
            {
                /*Copy into buffer and post event*/
                memcpy(timeSyncHandle->timeSyncBuff.pdelayRes_RxBuf[portNum - 1],
                       pktBuffer, TIMESYNC_PDELAY_BUF_SIZE);
            }

            EventP_setBits(&(timeSyncHandle->ptpPdelayResEvtObject[portNum - 1]), timeSyncHandle->eventIdPdelayResp);

            /*Another task pends on both Pdelay Resp and
             * Pdelay Resp Follow Up frame. If this is a single step
             * Pdelay response then we don't want that task to
             * pend forever
             */
            if(!ifTwoStep)
            {
                EventP_setBits(&(timeSyncHandle->ptpPdelayResEvtObject[portNum - 1]), timeSyncHandle->eventIdPdelayRespFlwUp);
            }

        }

        else if(PTP_PDLY_RESP_FLW_UP_MSG_ID ==
                pktType)         /*Pdelay response follow up frame*/
        {
            /*Copy into buffer and post event*/
            if(timeSyncHandle->timeSyncConfig.ll_has_hsrTag)
            {
                memcpy(timeSyncHandle->timeSyncBuff.pdelayResFlwUp_RxBuf[portNum - 1],
                       pktBuffer,
                       SRC_DST_MAC_SIZE);
                memcpy(timeSyncHandle->timeSyncBuff.pdelayResFlwUp_RxBuf[portNum - 1] +
                       SRC_DST_MAC_SIZE,
                       pktBuffer + SRC_DST_MAC_SIZE + HSR_CORRECTION, \
                       TIMESYNC_PDELAY_BUF_SIZE - SRC_DST_MAC_SIZE);
            }

            else
            {
                memcpy(timeSyncHandle->timeSyncBuff.pdelayResFlwUp_RxBuf[portNum - 1],
                       pktBuffer, TIMESYNC_PDELAY_BUF_SIZE);
            }

            EventP_setBits(&(timeSyncHandle->ptpPdelayResEvtObject[portNum - 1]), timeSyncHandle->eventIdPdelayRespFlwUp);
        }

        else if(PTP_SYNC_MSG_ID == pktType)
        {
            /*Get the timestamp. If it's in shared RAM, it's copied here, else
             * it's copied by the RT callback to the structure timeSyncHandle->rxTimestamp_gPTP
             */
            if(timeSyncHandle->timeSyncConfig.timestamp_from_shared_ram)
            {
                TimeSync_getRxTimestamp(timeSyncHandle, SYNC_FRAME, portNum, \
                                        &nanoseconds, &seconds);
            }

            /*Store Rx timestamp*/
            timeSyncHandle->syncParam[portNum - 1]->rxTsSec = seconds;
            timeSyncHandle->syncParam[portNum - 1]->rxTs = nanoseconds;


            /*set two step flag. Used in another task*/
            timeSyncHandle->syncParam[portNum - 1]->ifTwoStep = ifTwoStep;
            TimeSync_processSyncFrame(timeSyncHandle, pktBuffer, FALSE, portNum, size);
        }

        else if(PTP_FOLLOW_UP_MSG_ID == pktType)
        {
            TimeSync_processSyncFrame(timeSyncHandle, pktBuffer, TRUE, portNum, size);
        }

        else if(PTP_DLY_RESP_MSG_ID == pktType)   /*PTP_DLY_RSP_MSG_ID*/
        {
            TimeSync_processDelayResFrame(timeSyncHandle, pktBuffer, portNum);
        }

        else
        {
            return;
        }
    }

}

void TimeSync_processSyncFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                               uint8_t *buff, \
                               uint8_t followUp, uint8_t portNum, uint16_t size)
{
    uint8_t *bytePtr =  NULL;
    uint8_t masterPortNum = 0;
    uint8_t oppPort = 0;
    uint8_t offset = 0;
    uint8_t oppPortlinkStatus = 0;
    uint16_t halfWord = 0;
    uint32_t sharedDataRamBaseAddr = 0;
    uint64_t doubleWord = 0;
    uint64_t followUpCorrectionField = 0;
    uint64_t timeElapsed = 0;
    uint32_t retVal = SystemP_FAILURE;
    ICSS_EMAC_TxArgument txArg;

    sharedDataRamBaseAddr = (uint32_t)((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase;
    /*packet offset*/
    offset = timeSyncHandle->timeSyncConfig.frame_offset;

    /*Get the opposite port*/
    oppPort = ~portNum;
    oppPort &= 0x3;

    /*TODO: Review this*/
    /*Opposite port link status*/
    oppPortlinkStatus = 0;
    retVal = MDIO_phyLinkStatus(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->miiMdioRegBase,
                                            ((ICSS_EMAC_Attrs *)(timeSyncHandle->emacHandle->attrs))->phyAddr[oppPort - 1]);
    if(retVal == SystemP_SUCCESS)
        oppPortlinkStatus = 1;
    // oppPortlinkStatus =((ICSS_EmacObject *)
    //                      (timeSyncHandle->emacHandle)->object)->linkStatus[oppPort - 1];

    /*Since we are processing tagged HSR frames, we need to account for it*/
    if(TRUE == timeSyncHandle->timeSyncConfig.hsrEnabled)
    {
        offset -= HSR_CORRECTION;
    }

    bytePtr = (uint8_t *)(sharedDataRamBaseAddr + MASTER_PORT_NUM_OFFSET);
    masterPortNum = *bytePtr;


    /*Take timestamps and calculate BD since this is common to sync
     * frames from both ports. Synchronization OTOH is done only for
     * sync frames from master port*/

    /*For sync frame*/
    if(!followUp)
    {
        if(masterPortNum == portNum)
        {
            timeSyncHandle->tsRunTimeVar->syncPortNum = portNum;
            /*Reset the last seen counter*/
            timeSyncHandle->tsRunTimeVar->syncLastSeenCounter = 0;
        }

        /*Copy correction field*/
        bytePtr = (uint8_t *)(buff + PTP_CORRECTION_OFFSET - offset);
        TimeSync_convEnd6to8(bytePtr, &(timeSyncHandle->syncParam[portNum -
                               1]->correctionField));

        /*Do asymmetry correction*/
        timeSyncHandle->syncParam[portNum - 1]->correctionField += \
                timeSyncHandle->timeSyncConfig.asymmetryCorrection[portNum - 1];

        /*Copy sequence ID*/
        bytePtr = (uint8_t *)(buff + PTP_SEQ_ID_OFFSET - offset);
        TimeSync_convEndianess(bytePtr, &(timeSyncHandle->tsRunTimeVar->curSyncSeqId[portNum -
                                 1]), 2);

        /*if forced mode is on and it's a 1-step then we send the frame out
         * of the opposite port after setting the 2-step bit*/
        if(!timeSyncHandle->syncParam[portNum - 1]->ifTwoStep &&
                timeSyncHandle->timeSyncConfig.masterParams.ptp_flags[TS_PTP_TWO_STEP_INDEX] &&
                !timeSyncHandle->timeSyncConfig.emac_mode && oppPortlinkStatus)
        {
            timeSyncHandle->tsRunTimeVar->forced2step[portNum - 1] = TRUE;
            /*set 2-step bit*/
            bytePtr = (uint8_t *)((buff + PTP_FLAG_OFFSET - offset));
            *bytePtr |= 0x2;

            /*Create the follow up frame by changing the frame type field*/
            bytePtr = (uint8_t *)((buff + PTP_MSG_ID_OFFSET - offset));
            *bytePtr = PTP_FOLLOW_UP_MSG_ID;

            bytePtr = (uint8_t *)((buff + PTP_CONTROL_MSG_ID_OFFSET - offset));
            *bytePtr = PTP_FLW_UP_CTRL_MSG_ID;

            /*Strip the HSR tag before copying the follow up frame
              since we will add a different HSR tag*/
            if(timeSyncHandle->timeSyncConfig.hsrEnabled)
            {
                memcpy(timeSyncHandle->timeSyncBuff.followUp_TxBuf[portNum - 1], buff, 12);
                memcpy(timeSyncHandle->timeSyncBuff.followUp_TxBuf[portNum - 1] + 12, buff + 18,
                       size - 18);

            }

            else
            {
                memcpy(timeSyncHandle->timeSyncBuff.followUp_TxBuf[portNum - 1], buff, size);
            }

            /*Set event flag to indicate follow up frame generation*/
            EventP_setBits(&(timeSyncHandle->ptpSendFollowUpEvtObject[portNum - 1]), timeSyncHandle->eventIdFlwUpGenerated);

        }

        else
        {
            timeSyncHandle->tsRunTimeVar->forced2step[portNum - 1] = FALSE;

        }

        if((timeSyncHandle->syncParam[portNum - 1]->ifTwoStep)
                || (masterPortNum != portNum))
        {
            return;
        }

        /*Get origin timestamp*/
        bytePtr = (uint8_t *)(buff + PTP_REQ_RCPT_TS_SEC_OFFSET - offset);
        TimeSync_convEnd6to8(bytePtr, &(timeSyncHandle->syncParam[portNum - 1]->originTsSec));

        bytePtr = (uint8_t *)(buff + PTP_REQ_RCPT_TS_NSEC_OFFSET -
                              offset);
        TimeSync_convEndianess(bytePtr, &(timeSyncHandle->syncParam[portNum - 1]->originTsNs),
                      4);



    }

    else
    {
        /*Check for Sync mismatch*/
        bytePtr = (uint8_t *)(buff + PTP_SEQ_ID_OFFSET - offset);
        TimeSync_convEndianess(bytePtr, &(halfWord), 2);

        if(timeSyncHandle->tsRunTimeVar->curSyncSeqId[portNum - 1] != halfWord)
        {
            return;
        }

        /*get correction field in follow up frame*/
        bytePtr = (uint8_t *)(buff + PTP_CORRECTION_OFFSET - offset);
        TimeSync_convEnd6to8(bytePtr, &(followUpCorrectionField));

        /*Correction field contains correction field value in sync frame
         * It needs to be added to the correction field value inside follow up*/
        timeSyncHandle->syncParam[portNum - 1]->correctionField += followUpCorrectionField;

        /*Get origin timestamp*/
        bytePtr = (uint8_t *)(buff + PTP_REQ_RCPT_TS_SEC_OFFSET - offset);
        TimeSync_convEnd6to8(bytePtr, &(timeSyncHandle->syncParam[portNum - 1]->originTsSec));

        bytePtr = (uint8_t *)(buff + PTP_REQ_RCPT_TS_NSEC_OFFSET -
                              offset);
        TimeSync_convEndianess(bytePtr, &(timeSyncHandle->syncParam[portNum - 1]->originTsNs),
                      4);


        /*---If link is up on opposite port then
         * calculate Bridge delay and send out follow up frame---*/

        /*If PTP is configured in EMAC mode then don't do Bridge delay computation*/
        if(!timeSyncHandle->timeSyncConfig.emac_mode)
        {

            if(oppPortlinkStatus)
            {
                doubleWord = timeSyncHandle->syncParam[portNum - 1]->txTsSec *
                             (uint64_t)SEC_TO_NS + \
                             timeSyncHandle->syncParam[portNum - 1]->txTs;
                timeElapsed = timeSyncHandle->syncParam[portNum - 1]->rxTsSec *
                              (uint64_t)SEC_TO_NS + \
                              timeSyncHandle->syncParam[portNum - 1]->rxTs;
                /*This is the BD or bridge delay*/
                timeElapsed = doubleWord - timeElapsed;

                /*Multiply BD with RCF. For 802.1 AB this should be the accumulated rate ratio*/
                timeElapsed = (uint32_t)((double)timeElapsed * timeSyncHandle->tsSyntInfo->rcf);

                /*Add Peer delay for P2P mode*/
                if(P2P == timeSyncHandle->timeSyncConfig.type)
                {
                    timeElapsed += timeSyncHandle->tsRunTimeVar->pathDelay[portNum - 1];
                }

                /*Finally add the BD to existing correction field*/
                timeElapsed += followUpCorrectionField;

                /*timeElapsed now contains the corrected BD.
                * Encode into frame and send out*/
                TimeSync_convEndianess(&timeElapsed,
                              (buff + PTP_CORRECTION_OFFSET - offset), 6);

                if(timeSyncHandle->timeSyncConfig.custom_tx_api)
                {
// #ifdef NEW_TX_CALLBACK
//                     txArg.customFlag = 0;
// #endif //NEW_TX_CALLBACK
                    txArg.icssEmacHandle = timeSyncHandle->emacHandle;
                    txArg.lengthOfPacket = size;
                    txArg.portNumber = oppPort;
                    txArg.queuePriority = ICSS_EMAC_QUEUE1;
                    txArg.srcAddress = buff;

                    // ((((ICSS_EmacObject *)
                    //    timeSyncHandle->emacHandle->object)->callBackHandle)->txCallBack)->callBack(
                    //        &txArg,
                    //        ((((ICSS_EmacObject *)
                    //           timeSyncHandle->emacHandle->object)->callBackHandle)->txCallBack)->userArg);
                    ICSS_EMAC_txPacket(&txArg, NULL);
                }
                else
                {
                    txArg.icssEmacHandle = timeSyncHandle->emacHandle;
                    txArg.lengthOfPacket = size;
                    txArg.portNumber = oppPort;
                    txArg.queuePriority = ICSS_EMAC_QUEUE1;
                    txArg.srcAddress = buff;

                    ICSS_EMAC_txPacket(&txArg, NULL);
                    // ICSS_EmacTxPacketEnqueue(timeSyncHandle->emacHandle, buff, oppPort,
                    //                          ICSS_EMAC_QUEUE1, size);
                }
            }
        }
    }

    /*First time*/
    if((timeSyncHandle->tsRunTimeVar->stateMachine &
            TS_STATE_MACHINE_FIRST_ADJUSTMENT_DONE) == 0)
    {

        /*Do first adjustment or copying of timestamp directly into IEP*/
        TimeSync_doFirstAdjustment(timeSyncHandle, portNum);

        /*indicate that first adjustment is done*/
        timeSyncHandle->tsRunTimeVar->stateMachine |=
            TS_STATE_MACHINE_FIRST_ADJUSTMENT_DONE;

    }

    else    /*if not first time sync frame*/
    {
        if(timeSyncHandle->tsRunTimeVar->syncPortNum == portNum)
        {
            if((timeSyncHandle->tsRunTimeVar->stateMachine &
                    TS_STATE_MACHINE_READY_FOR_SYNC) == TS_STATE_MACHINE_READY_FOR_SYNC)
            {
                /*Synchronize clock*/
                TimeSync_synchronizeClock(timeSyncHandle);
            }

        }

    }

    if(timeSyncHandle->tsRunTimeVar->syncPortNum == portNum)
    {
        /*Post interrupt to send a delay request frame*/
        if(timeSyncHandle->timeSyncConfig.type == E2E)
        {
            SemaphoreP_post(&(timeSyncHandle->delayReqTxSemObject));
        }

        TimeSync_calcRcfAndSyncInterval(timeSyncHandle);
    }

}

void TimeSync_processDelayResFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                                   uint8_t *buff, \
                                   uint8_t portNum)
{
    uint8_t *bytePtr = NULL;
    uint16_t halfWord = 0;
    uint8_t offset = timeSyncHandle->timeSyncConfig.frame_offset;

    /*check if the delay response has the correct sequence ID*/
    bytePtr = (uint8_t *)(buff + PTP_SEQ_ID_OFFSET - offset);
    TimeSync_convEndianess(bytePtr, &(halfWord), 2);

    if((timeSyncHandle->tsRunTimeVar->delReqSequenceID - 1) != halfWord)
    {
        return;
    }

    /*Get the correction field*/
    bytePtr = (uint8_t *)(buff + PTP_CORRECTION_OFFSET - offset);
    TimeSync_convEnd6to8(bytePtr, &(timeSyncHandle->delayParams->correctionField));

    /*Get origin timestamp and calculate line delay*/
    bytePtr = (uint8_t *)(buff + PTP_REQ_RCPT_TS_SEC_OFFSET - offset);
    TimeSync_convEnd6to8(bytePtr, &(timeSyncHandle->delayParams->timeStampSec));

    bytePtr = (uint8_t *)(buff + PTP_REQ_RCPT_TS_NSEC_OFFSET -
                          offset);
    TimeSync_convEndianess(bytePtr, &(timeSyncHandle->delayParams->timeStampNS),
                  4);

    /*We have all the data now, calculate the line delay*/
    TimeSync_lineDelayCalc(timeSyncHandle);
}

void TimeSync_processPdelayReqFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                                    uint8_t *buff, \
                                    uint8_t portNum)
{
    /*Copy contents and echo back on the same port*/
    uint8_t *ptpPDelayResPacket = NULL;
    uint8_t *ptpPDelayResFlwUpPacket = NULL;
    uint8_t *bytePtrSrc =  NULL;
    uint8_t *bytePtrPdelReq = NULL;
    uint8_t *bytePtrPdelRes = NULL;
    uint8_t offset = timeSyncHandle->timeSyncConfig.frame_offset;

    ICSS_EMAC_TxArgument txArg;

    if(ICSS_EMAC_PORT_1 == portNum)
    {
        ptpPDelayResPacket = timeSyncHandle->timeSyncBuff.pdelayRes_TxBuf[0];
        ptpPDelayResFlwUpPacket = timeSyncHandle->timeSyncBuff.pdelayResFlwUp_TxBuf[0];
    }

    else
    {
        ptpPDelayResPacket = timeSyncHandle->timeSyncBuff.pdelayRes_TxBuf[1];
        ptpPDelayResFlwUpPacket = timeSyncHandle->timeSyncBuff.pdelayResFlwUp_TxBuf[1];
    }

    /*Add Request port clock Identity*/
    bytePtrSrc = (uint8_t *)(buff + PTP_SRC_CLK_IDENTITY - \
                             offset);
    bytePtrPdelReq = (uint8_t *)(ptpPDelayResPacket + PTP_REQ_SRC_PORT_IDENTITY -
                                 offset);
    bytePtrPdelRes = (uint8_t *)(ptpPDelayResFlwUpPacket + PTP_REQ_SRC_PORT_IDENTITY
                                 - offset);
    memcpy(bytePtrPdelReq, bytePtrSrc, 8);
    memcpy(bytePtrPdelRes, bytePtrSrc, 8);

    /*Add Correction Field from delay request*/
    bytePtrSrc = (uint8_t *)(buff + PTP_CORRECTION_OFFSET - \
                             offset);
    bytePtrPdelReq = (uint8_t *)(ptpPDelayResPacket + PTP_CORRECTION_OFFSET - \
                                 offset);
    bytePtrPdelRes = (uint8_t *)(ptpPDelayResFlwUpPacket + PTP_CORRECTION_OFFSET - \
                                 offset);
    memcpy(bytePtrPdelReq, bytePtrSrc, 8);
    memcpy(bytePtrPdelRes, bytePtrSrc, 8);

    /**Add requested source port identity*/
    bytePtrSrc = (uint8_t *)(buff + PTP_SRC_PORT_ID_OFFSET - \
                             offset);
    bytePtrPdelReq = (uint8_t *)(ptpPDelayResPacket + PTP_REQ_SRC_PORT_ID -
                                 offset);
    bytePtrPdelRes = (uint8_t *)(ptpPDelayResFlwUpPacket + PTP_REQ_SRC_PORT_ID -
                                 offset);
    memcpy(bytePtrPdelReq, bytePtrSrc, 2);
    memcpy(bytePtrPdelRes, bytePtrSrc, 2);

    /**Copy sequence ID from Delay Req packet*/
    bytePtrSrc = (uint8_t *)(buff + PTP_SEQ_ID_OFFSET - \
                             offset);
    bytePtrPdelReq = (uint8_t *)(ptpPDelayResPacket + PTP_SEQ_ID_OFFSET -
                                 offset);
    bytePtrPdelRes = (uint8_t *)(ptpPDelayResFlwUpPacket + PTP_SEQ_ID_OFFSET -
                                 offset);
    memcpy(bytePtrPdelReq, bytePtrSrc, 2);
    memcpy(bytePtrPdelRes, bytePtrSrc, 2);

    /*Copy domain number*/
    bytePtrSrc = (uint8_t *)(buff + PTP_DOMAIN_NUM_OFFSET - \
                             offset);
    bytePtrPdelReq = (uint8_t *)(ptpPDelayResPacket + PTP_DOMAIN_NUM_OFFSET -
                                 offset);
    bytePtrPdelRes = (uint8_t *)(ptpPDelayResFlwUpPacket + PTP_DOMAIN_NUM_OFFSET -
                                 offset);
    memcpy(bytePtrPdelReq, bytePtrSrc, 2);
    memcpy(bytePtrPdelRes, bytePtrSrc, 2);

    /*Add Source port ID*/
    TimeSync_addHalfWord(ptpPDelayResPacket + PTP_SRC_PORT_ID_OFFSET - offset,
                portNum);
    TimeSync_addHalfWord(ptpPDelayResFlwUpPacket + PTP_SRC_PORT_ID_OFFSET -
                offset,
                portNum);

    TimeSync_convEndianess(&(timeSyncHandle->pDelayParams[portNum - 1].pDelayReqRcvdTSSec),
                  ptpPDelayResPacket + PTP_REQ_RCPT_TS_SEC_OFFSET -
                  offset, 6);
    TimeSync_convEndianess(&(timeSyncHandle->pDelayParams[portNum - 1].pDelayReqRcvdTSNsec),
                  ptpPDelayResPacket + PTP_REQ_RCPT_TS_NSEC_OFFSET - offset, 4);

    if(timeSyncHandle->timeSyncConfig.ll_has_hsrTag ||
            timeSyncHandle->timeSyncConfig.custom_tx_api)
    {
// #ifdef NEW_TX_CALLBACK
//         txArg.customFlag = 0;
// #endif //NEW_TX_CALLBACK
        txArg.icssEmacHandle = timeSyncHandle->emacHandle;
        txArg.lengthOfPacket = TIMESYNC_PDELAY_BUF_SIZE;
        txArg.portNumber = portNum;
        txArg.queuePriority = ICSS_EMAC_QUEUE1;
        txArg.srcAddress = ptpPDelayResPacket;

        /*Send pdelay response follow up frame out*/
        // ((((ICSS_EmacObject *)
        //    timeSyncHandle->emacHandle->object)->callBackHandle)->txCallBack)->callBack(
        //        &txArg,
        //        ((((ICSS_EmacObject *)
        //           timeSyncHandle->emacHandle->object)->callBackHandle)->txCallBack)->userArg);
        ICSS_EMAC_txPacket(&txArg, NULL);
    }
    else
    {
        txArg.icssEmacHandle = timeSyncHandle->emacHandle;
        txArg.lengthOfPacket = TIMESYNC_PDELAY_BUF_SIZE;
        txArg.portNumber = portNum;
        txArg.queuePriority = ICSS_EMAC_QUEUE1;
        txArg.srcAddress = ptpPDelayResPacket;

        ICSS_EMAC_txPacket(&txArg, NULL);
        // ICSS_EmacTxPacketEnqueue(timeSyncHandle->emacHandle, ptpPDelayResPacket,
        //                          portNum,
        //                          ICSS_EMAC_QUEUE1, TIMESYNC_PDELAY_BUF_SIZE);
    }
}

void TimeSync_processPdelayRespFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                                     uint8_t *buff, \
                                     uint8_t followUp, uint8_t portNum)
{
    uint8_t offset = timeSyncHandle->timeSyncConfig.frame_offset;

    /*generic buffer used for comparison and storing*/
    uint8_t buf_for_comp[8] = {0};
    uint16_t halfWord = 0;

    /*check if this is a valid response frame to our delay request
     * extract parameters and calculate peer delay if so*/

    /*Check for source port identity*/
    memcpy(buf_for_comp, buff + PTP_REQ_SRC_PORT_IDENTITY - offset,
           8);

    if(0 != memcmp(buf_for_comp, timeSyncHandle->timeSyncConfig.clockIdentity, 8))
    {
        return;
    }

    /*check with the sequence id that was sent out*/
    TimeSync_convEndianess(buff + PTP_SEQ_ID_OFFSET - offset, &halfWord, 2);

    if(halfWord != (timeSyncHandle->tsRunTimeVar->pDelReqSequenceID[portNum - 1] -
                    1))
    {
        return;
    }

    if(!followUp)
    {

        /*Extract correction field*/
        TimeSync_convEnd6to8(buff + PTP_CORRECTION_OFFSET - offset, \
                    & (timeSyncHandle->pDelayParams[portNum - 1].delayResCorrField));

        /*Now check if it's a two step, if yes then exit*/
        TimeSync_convEndianess(buff + PTP_FLAG_OFFSET - offset, &halfWord, 2);

        if(halfWord & PTP_TWO_STEP_MASK)
        {
            /*Extract T2 timestamp in seconds*/
            TimeSync_convEnd6to8(buff + PTP_REQ_RCPT_TS_SEC_OFFSET - offset, \
                        & (timeSyncHandle->pDelayParams[portNum - 1].T2Sec));

            /*Extract T2 timestamp in nanoseconds*/
            TimeSync_convEndianess(buff + PTP_REQ_RCPT_TS_NSEC_OFFSET - offset, \
                          & (timeSyncHandle->pDelayParams[portNum - 1].T2Nsec), 4);

            return;
        }

    }

    else
    {
        /*Extract correction field*/
        TimeSync_convEnd6to8(buff + PTP_CORRECTION_OFFSET - offset, \
                    & (timeSyncHandle->pDelayParams[portNum - 1].delayResFwUpCorrField));
        /*Extract T2 timestamp in seconds*/
        TimeSync_convEnd6to8(buff + PTP_REQ_RCPT_TS_SEC_OFFSET - offset, \
                    & (timeSyncHandle->pDelayParams[portNum - 1].T3Sec));

        /*Extract T2 timestamp in nanoseconds.*/
        TimeSync_convEndianess(buff + PTP_REQ_RCPT_TS_NSEC_OFFSET - offset, \
                      & (timeSyncHandle->pDelayParams[portNum - 1].T3Nsec), 4);

    }

    /*Calculate peer delay*/
    TimeSync_peerDelayCalc(timeSyncHandle, followUp, portNum);
    /*Update params here, NRR gets calculated in the background*/
    TimeSync_updateNRRParams(timeSyncHandle, portNum);
    TimeSync_calcNRR(timeSyncHandle, portNum);

    return;

}

void TimeSync_forced2StepBDCalc(TimeSync_ParamsHandle_t timeSyncHandle,
                                uint8_t portNum)
{
    uint8_t *ptpFlwUpPacket = NULL;
    uint64_t doubleWord = 0;
    uint64_t timeElapsed = 0;
    ICSS_EMAC_TxArgument txArg;
    uint8_t oppPort = 0;

    /*Get the opposite port*/
    oppPort = ~portNum;
    oppPort &= 0x3;

    ptpFlwUpPacket = timeSyncHandle->timeSyncBuff.followUp_TxBuf[portNum - 1];

    /*calculate bridge delay and write into the follow up frame
     * time stamp is already available in syncParam*/
    doubleWord = timeSyncHandle->syncParam[portNum - 1]->txTsSec *
                 (uint64_t)SEC_TO_NS + \
                 timeSyncHandle->syncParam[portNum - 1]->txTs;
    timeElapsed = timeSyncHandle->syncParam[portNum - 1]->rxTsSec *
                  (uint64_t)SEC_TO_NS + \
                  timeSyncHandle->syncParam[portNum - 1]->rxTs;
    /*This is the BD or bridge delay*/
    timeElapsed = doubleWord - timeElapsed;

    /*Multiply BD with RCF. For 802.1 AB this should be the accumulated rate ratio*/
    timeElapsed = (uint32_t)((double)timeElapsed * timeSyncHandle->tsSyntInfo->rcf);

    /*Add Peer delay for P2P mode*/
    if(P2P == timeSyncHandle->timeSyncConfig.type)
    {
        timeElapsed += timeSyncHandle->tsRunTimeVar->pathDelay[portNum - 1];
    }

    /*timeElapsed now contains the corrected BD.
    * Encode into frame and send out*/
    TimeSync_convEndianess(&timeElapsed,
                  (ptpFlwUpPacket + PTP_CORRECTION_OFFSET -
                   timeSyncHandle->timeSyncConfig.frame_offset), 6);

    if(timeSyncHandle->timeSyncConfig.hsrEnabled ||
            timeSyncHandle->timeSyncConfig.custom_tx_api)
    {

// #ifdef NEW_TX_CALLBACK
//         txArg.customFlag = 0;
// #endif //NEW_TX_CALLBACK
        txArg.icssEmacHandle = timeSyncHandle->emacHandle;
        txArg.lengthOfPacket = timeSyncHandle->timeSyncBuff.flwUpBuf_size;
        txArg.portNumber = oppPort;
        txArg.queuePriority = ICSS_EMAC_QUEUE1;
        txArg.srcAddress = ptpFlwUpPacket;

        /*Send follow up frame out*/
        // ((((ICSS_EmacObject *)
        //    timeSyncHandle->emacHandle->object)->callBackHandle)->txCallBack)->callBack(
        //        &txArg,
        //        ((((ICSS_EmacObject *)
        //           timeSyncHandle->emacHandle->object)->callBackHandle)->txCallBack)->userArg);
        ICSS_EMAC_txPacket(&txArg, NULL);
    }
    else
    {
        txArg.icssEmacHandle = timeSyncHandle->emacHandle;
        txArg.lengthOfPacket = timeSyncHandle->timeSyncBuff.flwUpBuf_size;
        txArg.portNumber = oppPort;
        txArg.queuePriority = ICSS_EMAC_QUEUE1;
        txArg.srcAddress = ptpFlwUpPacket;

        ICSS_EMAC_txPacket(&txArg, NULL);
        // ICSS_EmacTxPacketEnqueue(timeSyncHandle->emacHandle, ptpFlwUpPacket,
        //                          oppPort, ICSS_EMAC_QUEUE1, timeSyncHandle->timeSyncBuff.flwUpBuf_size);
    }

}

void TimeSync_getGeneralMessage(TimeSync_ParamsHandle_t timeSyncHandle,
                                int8_t *buff)
{
    uint8_t *macId = NULL;
    uint8_t count = 0;

    if(timeSyncHandle->stackParams.generalFrameFlag)
    {
        if(E2E == timeSyncHandle->timeSyncConfig.type)
        {
            memcpy(buff, timeSyncHandle->stackParams.ptpGeneralFrame +
                   PTP_E2E_BUFFER_OFFSET,
                   timeSyncHandle->stackParams.ptpGeneralSize - PTP_E2E_BUFFER_OFFSET);
        }

        if(P2P == timeSyncHandle->timeSyncConfig.type)
        {
            memcpy(buff, timeSyncHandle->stackParams.ptpGeneralFrame +
                   PTP_P2P_BUFFER_OFFSET,
                   timeSyncHandle->stackParams.ptpGeneralSize - PTP_P2P_BUFFER_OFFSET);
        }

        macId = (uint8_t *)(timeSyncHandle->stackParams.ptpGeneralFrame +
                            SRC_MAC_OFFSET);

        for(count = 0; count < 6; count++)
        {
            timeSyncHandle->stackParams.ptpSrcMacID[count] = macId[count];
        }

        timeSyncHandle->stackParams.generalFrameFlag = 0;
    }

}

uint8_t TimeSync_isEnabled(TimeSync_ParamsHandle_t timeSyncHandle)
{
    volatile uint8_t *ptpCtrlPtr;
    ptpCtrlPtr = (uint8_t *)(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase
                             + TIMESYNC_CTRL_VAR_OFFSET);

    return *ptpCtrlPtr;
}

void TimeSync_updateParentAddress(TimeSync_ParamsHandle_t timeSyncHandle,
                                  uint8_t *parentMac)
{
    /*write master mac id*/
    volatile uint8_t *macId = NULL;
    uint8_t count = 0;
    macId = (uint8_t *)(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase
                        + SYNC_MASTER_MAC_OFFSET);

    for(count = 0; count < 6; count++)
    {
        macId[count] = parentMac[count];
    }
}

void TimeSync_getPrevAddress(TimeSync_ParamsHandle_t timeSyncHandle,
                             uint8_t *prevMac)
{
    uint8_t count = 0;

    for(count = 0; count < 6; count++)
    {
        prevMac[count] = timeSyncHandle->stackParams.ptpSrcMacID[count];
    }
}

void TimeSync_Port1linkResetCallBack(uint8_t linkStatus, void *arg2)
{
    uint8_t *bytePtr = NULL;
    uint8_t *sharedRAMbaseAddress = NULL;
    uint32_t word = 0;

    TimeSync_ParamsHandle_t timeSyncHandle = (TimeSync_ParamsHandle_t)arg2;
    sharedRAMbaseAddress = (uint8_t *)((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase;

    /*Reset Port 0 NRR Info*/
    memset(timeSyncHandle->tsNrrInfo[0], 0x0, sizeof(timeSync_NrrInfo_t));
    timeSyncHandle->tsNrrInfo[0]->nrr = 1.0;

    /*reset sequence id on that port*/
    timeSyncHandle->tsRunTimeVar->pDelReqSequenceID[0] = 0;
    /*Reset peer delay params*/
    memset(&(timeSyncHandle->pDelayParams[0]), 0x0, sizeof(peerDelayParams_t));
    /*make peer delay history 0*/
    timeSyncHandle->tsRunTimeVar->pathDelay[0] = 0;

    /*If link loss on port connected to Master then reset the
         *  Syntonization and Sync params*/
    if(timeSyncHandle->tsRunTimeVar->syncPortNum == ICSS_EMAC_PORT_1)
    {
        /*Reset RCF*/
        memset((timeSyncHandle->tsSyntInfo), 0x0, sizeof(timeSync_SyntInfo_t));
        timeSyncHandle->tsSyntInfo->rcf = 1.0;

        bytePtr = (uint8_t *)(sharedRAMbaseAddress + TIMESYNC_TC_RCF_OFFSET);
        memcpy(bytePtr, &word, 4);

        /*reset sync params*/
        memset((timeSyncHandle->syncParam[0]), 0x0, sizeof(syncParam_t));

    }
}

void TimeSync_Port2linkResetCallBack(uint8_t linkStatus, void *arg2)
{

    uint8_t *bytePtr = NULL;
    uint8_t *sharedRAMbaseAddress = NULL;
    uint32_t word = 0;
    TimeSync_ParamsHandle_t timeSyncHandle = (TimeSync_ParamsHandle_t)arg2;

    sharedRAMbaseAddress = (uint8_t *)((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase;
    /*Reset Port 1 NRR Info*/
    memset(timeSyncHandle->tsNrrInfo[1], 0x0, sizeof(timeSync_NrrInfo_t));
    timeSyncHandle->tsNrrInfo[1]->nrr = 1.0;
    /*reset sequence id on that port*/
    timeSyncHandle->tsRunTimeVar->pDelReqSequenceID[1] = 0;
    /*Reset peer delay params*/
    memset(&(timeSyncHandle->pDelayParams[1]), 0x0, sizeof(peerDelayParams_t));
    /*make peer delay history 0*/
    timeSyncHandle->tsRunTimeVar->pathDelay[1] = 0;

    /*If link loss on port connected to Master then reset the
     *  Syntonization and Sync params*/
    if(timeSyncHandle->tsRunTimeVar->syncPortNum == ICSS_EMAC_PORT_2)
    {
        /*Reset RCF*/
        memset((timeSyncHandle->tsSyntInfo), 0x0, sizeof(timeSync_SyntInfo_t));
        timeSyncHandle->tsSyntInfo->rcf = 1.0;

        bytePtr = (uint8_t *)(sharedRAMbaseAddress + TIMESYNC_TC_RCF_OFFSET);
        memcpy(bytePtr, &word, 4);

        /*reset sync params*/
        memset((timeSyncHandle->syncParam[1]), 0x0, sizeof(syncParam_t));

    }


}

void TimeSync_reset(TimeSync_ParamsHandle_t timeSyncHandle)
{
    uint8_t *bytePtr = NULL;
    uint8_t *sharedRAMbaseAddress = NULL;
    uint32_t word = 0;
    uint8_t ptpEnabled = timeSyncHandle->enabled;

    /*call sync loss callback*/
    if(timeSyncHandle->timeSyncConfig.timeSyncSyncLossCallBackfn != NULL)
    {
        timeSyncHandle->timeSyncConfig.timeSyncSyncLossCallBackfn();
    }

    sharedRAMbaseAddress = (uint8_t *)((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase;

    /*first disable PTP so firmware is not running*/
    TimeSync_drvDisable(timeSyncHandle);

    /*Now reset all values*/

    /*Reset structures*/
    memset((timeSyncHandle->syncParam[0]), 0x0, sizeof(syncParam_t));
    memset((timeSyncHandle->syncParam[1]), 0x0, sizeof(syncParam_t));
    memset((timeSyncHandle->delayParams), 0x0, sizeof(delayReqRespParams_t));
    memset(&(timeSyncHandle->pDelayParams[0]), 0x0, sizeof(peerDelayParams_t));
    memset(&(timeSyncHandle->pDelayParams[1]), 0x0, sizeof(peerDelayParams_t));
    memset((timeSyncHandle->tsSyntInfo), 0x0, sizeof(timeSync_SyntInfo_t));
    memset((timeSyncHandle->tsRunTimeVar), 0x0, sizeof(timeSync_RuntimeVar_t));
    memset(timeSyncHandle->tsNrrInfo[0], 0x0, sizeof(timeSync_NrrInfo_t));
    memset(timeSyncHandle->tsNrrInfo[1], 0x0, sizeof(timeSync_NrrInfo_t));


    memset(sharedRAMbaseAddress + SYNC_MASTER_MAC_OFFSET, 0x0, 6);

    word = 1 * 1024;
    bytePtr = (uint8_t *)(sharedRAMbaseAddress + TIMESYNC_TC_RCF_OFFSET);
    memcpy(bytePtr, &word, 4);

    /*Reset structure values*/
    timeSyncHandle->tsRunTimeVar->stateMachine = 0;
    timeSyncHandle->tsSyntInfo->rcf = 1.0;
    timeSyncHandle->tsNrrInfo[0]->nrr = 1.0;
    timeSyncHandle->tsNrrInfo[1]->nrr = 1.0;

    timeSyncHandle->tsRunTimeVar->clockDrift = 10000;
    timeSyncHandle->tsRunTimeVar->syncTimeoutInterval = 10000;

    timeSyncHandle->offsetAlgo->driftThreshold =
        TIMESYNC_OFFSET_STABLE_ALGO_THRESHOLD;

    /*Reset stack*/
    if((timeSyncHandle->timeSyncConfig).ptpDrvStackReset != NULL)
    {
        (timeSyncHandle->timeSyncConfig).ptpDrvStackReset((void *)timeSyncHandle);
    }

    /*Finally enable PTP, only if it was enabled while calling this function*/
    if(ptpEnabled == TRUE)
        TimeSync_drvEnable(timeSyncHandle);
}

void TimeSync_writeTS_SingleStep_Sync(TimeSync_ParamsHandle_t timeSyncHandle,
                                      uint8_t portNum)
{


    /*The logic of this function works like this
     * Assume three timestamps Y, Y' and Z
     * Y--------------Y'----------------Z
     * Z is the start of the seconds cycle i.e. nanoseconds value of 0
     * Y denotes current time inside this function which is invoked
     * just before transmitting the sync frame. Y' is the actual Tx SOF timestamp
     * and is guaranteed to be within few microsends/miliseconds ahead of Y.
     * Now three possible cases exist
     * Y-----Z-----Y' in which case Y' lies in the next cycle
     * Y-----Y'----Z  in which case Y' is in current cycle
     * corner cases in which Y == Z and Y' == Z also fall under the above 2
     *
     * In firmware the timestamp Y' is compared against Z and it is determined
     * whether it falls in next or previous cycle. The corresponding seconds value
     * of Z is then used to get the seconds value. Nanoseconds value is obtained by
     * subtraction and long division and reminder operation is avoided.
     */

    uint8_t *bytePtr = NULL;
    uint64_t iepCount = 0;
    uint64_t seconds = 0;
    uint32_t iepBaseAddress = (uint32_t)(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->iep0RegBase);
    uint32_t sharedRAMbaseAddress = (uint32_t)((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->sharedDramBase;
    bytePtr = (uint8_t *)((uint32_t)(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0));
    /*Get current time 64 bit counter and time in seconds or Y*/
    memcpy(&iepCount, bytePtr, 8);
    seconds = iepCount / (uint64_t)SEC_TO_NS;

    /*Get next start of cycle TS or Z (see comments)*/
    seconds += 1;
    iepCount = seconds * (uint64_t)SEC_TO_NS;

    if(ICSS_EMAC_PORT_1 == portNum)
    {
        memcpy((uint8_t *)(sharedRAMbaseAddress + SINGLE_STEP_IEP_OFFSET_P1), &iepCount,
               8);
        memcpy((uint8_t *)(sharedRAMbaseAddress + SINGLE_STEP_SECONDS_OFFSET_P1),
               &seconds, 8);
    }

    else
    {
        memcpy((uint8_t *)(sharedRAMbaseAddress + SINGLE_STEP_IEP_OFFSET_P2), &iepCount,
               8);
        memcpy((uint8_t *)(sharedRAMbaseAddress + SINGLE_STEP_SECONDS_OFFSET_P2),
               &seconds, 8);
    }

}

void TimeSync_dummyBMCA(TimeSync_ParamsHandle_t timeSyncHandle,
                        uint8_t *pktBuffer)
{
    /*Extract MAC ID and write parent address*/

    if(!timeSyncHandle->tsRunTimeVar->bmcaDone)
    {
        /*pass the source MAC ID*/
        TimeSync_updateParentAddress(timeSyncHandle, pktBuffer + 6);
        timeSyncHandle->tsRunTimeVar->bmcaDone = 1;
    }
}

void TimeSync_rxPhyDelayCorrection(TimeSync_ParamsHandle_t timeSyncHandle)
{
    if(timeSyncHandle->rxTimestamp_gPTP->nanoseconds >= timeSyncHandle->timeSyncConfig.rxPhyLatency)
    {
        timeSyncHandle->rxTimestamp_gPTP->nanoseconds -= timeSyncHandle->timeSyncConfig.rxPhyLatency;
    }

    else
    {
        timeSyncHandle->rxTimestamp_gPTP->nanoseconds = SEC_TO_NS -
                (timeSyncHandle->timeSyncConfig.rxPhyLatency - timeSyncHandle->rxTimestamp_gPTP->nanoseconds);
        timeSyncHandle->rxTimestamp_gPTP->seconds -= 1;
    }
}

uint64_t getIEPTimestamp(TimeSync_ParamsHandle_t timeSyncHandle)
{

    uint8_t *bytePtr;
    uint64_t iepCount = 0;
    uint32_t iepBaseAddress = (uint32_t)(((PRUICSS_HwAttrs const *)(timeSyncHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    bytePtr = (uint8_t *)((uint32_t)(iepBaseAddress +  CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0));
    memcpy(&iepCount, bytePtr, 8);

    return iepCount;
}

