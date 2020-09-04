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
#include <string.h>
#include "icss_timeSync.h"
#include "icss_timeSync_init.h"
#include "icss_timeSync_memory_map.h"
#include "icss_timeSyncApi.h"
#include "icss_timeSync_utils.h"
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t TimeSync_config(TimeSync_ParamsHandle_t timeSyncParamsHandle)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(timeSyncParamsHandle->pruicssHandle->hwAttrs);
    uintptr_t iepBaseAddress = pruicssHwAttrs->iep0RegBase;
    uint8_t i;
    uint32_t regVal;

    if(timeSyncParamsHandle == NULL)
    {
        return ERROR_HANDLE_INVALID;
    }

    /* Configures domainNumber list */
    for(i = 0; i < TS_NUM_DOMAINS; i++)
    {
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + TIMESYNC_DOMAIN_NUMBER_LIST + i,
            timeSyncParamsHandle->timeSyncConfig.domainNumber[i]);
    }

    /*Update domain number in PTP TX buffers*/
    TimeSync_updateDomainNumberInPTPFrames(timeSyncParamsHandle);

    /*Update log message interval in PTP TX buffers*/
    TimeSync_updateLogMessageIntervalInPTPFrames(timeSyncParamsHandle);

    /* Configures SYNCOUT sync0 signal start time and pulse width */
    regVal = HW_RD_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);
    regVal |= 0x00000004;
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, regVal);
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0, timeSyncParamsHandle->timeSyncConfig.syncOut_sync0Start);
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG, timeSyncParamsHandle->timeSyncConfig.syncOut_sync0PWidth);

    return 0;
}

int32_t TimeSync_getTxTimestamp(TimeSync_ParamsHandle_t timeSyncParamsHandle, \
                                ptpFrameTypes_t txFrameType, uint8_t txPort, \
                                uint32_t *nanoseconds, uint64_t *seconds)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(timeSyncParamsHandle->pruicssHandle->hwAttrs);
    uint32_t timeStampOffsetAddr = 0;
    uint8_t *bytePtr;
    uint64_t Nanoseconds64;

    if(timeSyncParamsHandle->emacHandle == NULL)
    {
        return ERROR_HANDLE_INVALID;
    }

    /* Input error check */
    if((txFrameType != SYNC_FRAME) && (txFrameType != DELAY_REQ_FRAME)
            && (txFrameType != DELAY_RESP_FRAME))
    {
        return ERROR_TX_FRAMETYPE_NOTVALID;    /* not a valid frameType value */
    }

    if((txPort != ICSS_EMAC_PORT_1) && (txPort != ICSS_EMAC_PORT_2))
    {
        return ERROR_TX_PORTNUMBER_NOTVALID;    /* not a valid port number */
    }

    /* Read timestamp */
    if(txFrameType == SYNC_FRAME)
    {
        if(txPort == ICSS_EMAC_PORT_1)
        {
            timeStampOffsetAddr = TX_SYNC_TIMESTAMP_OFFSET_P1;
        }

        else
        {
            timeStampOffsetAddr = TX_SYNC_TIMESTAMP_OFFSET_P2;
        }
    }

    else if(txFrameType == DELAY_REQ_FRAME)
    {
        if(txPort == ICSS_EMAC_PORT_1)
        {
            timeStampOffsetAddr = TX_PDELAY_REQ_TIMESTAMP_OFFSET_P1;
        }

        else
        {
            timeStampOffsetAddr = TX_PDELAY_REQ_TIMESTAMP_OFFSET_P2;
        }
    }

    else /* (rxFrameType == DELAY_RESP_FRAME) */
    {
        if(txPort == ICSS_EMAC_PORT_1)
        {
            timeStampOffsetAddr = TX_PDELAY_RESP_TIMESTAMP_OFFSET_P1;
        }

        else
        {
            timeStampOffsetAddr = TX_PDELAY_RESP_TIMESTAMP_OFFSET_P2;
        }
    }

    bytePtr = (uint8_t *)((uint32_t)(pruicssHwAttrs->sharedDramBase +
                                     timeStampOffsetAddr));

    memcpy(&Nanoseconds64, bytePtr, 8);
    *nanoseconds = (uint32_t)(Nanoseconds64 % (uint64_t)SEC_TO_NS);
    *seconds = Nanoseconds64 / (uint64_t)SEC_TO_NS;

    return 0;
}

int32_t TimeSync_getRxTimestamp(TimeSync_ParamsHandle_t timeSyncParamsHandle,
                                ptpFrameTypes_t rxFrameType, uint8_t rxPort, \
                                uint32_t *nanoseconds, uint64_t *seconds)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(timeSyncParamsHandle->pruicssHandle->hwAttrs);
    uint32_t timeStampOffsetAddr = 0;
    uint8_t *bytePtr;
    uint64_t Nanoseconds64;

    if(timeSyncParamsHandle->emacHandle == NULL)
    {
        return ERROR_HANDLE_INVALID;
    }

    /* Input error check */
    if((rxFrameType != SYNC_FRAME)
            && (rxFrameType != DELAY_REQ_FRAME)
            && (rxFrameType != DELAY_RESP_FRAME))
    {
        return -1;    /* not a valid frameType value */
    }

    if((rxPort != ICSS_EMAC_PORT_1)
            && (rxPort != ICSS_EMAC_PORT_2))
    {
        return -2;    /* not a valid port number */
    }

    if(rxFrameType == SYNC_FRAME)
    {
        if(rxPort == ICSS_EMAC_PORT_1)
        {
            timeStampOffsetAddr = RX_SYNC_TIMESTAMP_OFFSET_P1;
        }

        else
        {
            timeStampOffsetAddr = RX_SYNC_TIMESTAMP_OFFSET_P2;
        }
    }

    else if(rxFrameType == DELAY_REQ_FRAME)
    {
        if(rxPort == ICSS_EMAC_PORT_1)
        {
            timeStampOffsetAddr = RX_PDELAY_REQ_TIMESTAMP_OFFSET_P1;
        }

        else
        {
            timeStampOffsetAddr = RX_PDELAY_REQ_TIMESTAMP_OFFSET_P2;
        }
    }

    else /* (rxFrameType == DELAY_RESP_FRAME) */
    {
        if(rxPort == ICSS_EMAC_PORT_1)
        {
            timeStampOffsetAddr = RX_PDELAY_RESP_TIMESTAMP_OFFSET_P1;
        }

        else
        {
            timeStampOffsetAddr = RX_PDELAY_RESP_TIMESTAMP_OFFSET_P2;
        }
    }

    bytePtr = (uint8_t *)((uint32_t)(pruicssHwAttrs->sharedDramBase +
                                     timeStampOffsetAddr));

    memcpy(&Nanoseconds64, bytePtr, 8);
    *nanoseconds = (uint32_t)(Nanoseconds64 % (uint64_t)SEC_TO_NS);
    *seconds = Nanoseconds64 / (uint64_t)SEC_TO_NS;

    return 0;
}

int32_t TimeSync_getRxTimestampFromFrame(TimeSync_ParamsHandle_t timeSyncParamsHandle,
                                uint8_t rxPort, \
                                uint32_t *nanoseconds, uint64_t *seconds, uint8_t* TimeStampAddress)
{
    uint64_t Nanoseconds64;

    if(timeSyncParamsHandle->emacHandle == NULL)
    {
        return ERROR_HANDLE_INVALID;
    }

    if((rxPort != ICSS_EMAC_PORT_1)
            && (rxPort != ICSS_EMAC_PORT_2))
    {
        return -2;    /* not a valid port number */
    }

    memcpy(&Nanoseconds64, TimeStampAddress, 8);
    *nanoseconds = (uint32_t)(Nanoseconds64 % (uint64_t)SEC_TO_NS);
    *seconds = Nanoseconds64 / (uint64_t)SEC_TO_NS;

    return 0;
}

int8_t TimeSync_adjTimeSlowComp(TimeSync_ParamsHandle_t timeSyncParamsHandle,
                                int32_t adjOffset)
{
    uint32_t compensation_period;
    uintptr_t iepBaseAddress = (((PRUICSS_HwAttrs const *)(timeSyncParamsHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    if(timeSyncParamsHandle->emacHandle == NULL)
    {
        return ERROR_HANDLE_INVALID;
    }

    /* set compensation interval = sync interval/drift.
       Example: Sync interval=30ms, drift=300ns, compensation interval=30ms/300ns = 5*30000000/300 = 5*100000ns. */
    compensation_period  = (uint32_t)((double)(
                                          timeSyncParamsHandle->tsRunTimeVar->ltaSyncInterval) /
                                      (double)(abs(adjOffset)));

    if(adjOffset == 0) /*No compensation required*/
    {
        /* set compensation increment = 5ns (default val)*/
        HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x00000551);
        HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_SLOW_COMPEN_REG, 0);
    }

    else if(adjOffset < 0) /* slave is faster*/
    {
        /* set compensation increment = 10ns (default val)*/
        HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x00000A51);
        HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_SLOW_COMPEN_REG, compensation_period);
    }

    else    /* master is faster*/
    {
        /* set compensation increment = 0ns*/
        HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x00000051);
        HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_SLOW_COMPEN_REG, compensation_period);
    }
    return TIMESYNC_OK;
}

int32_t TimeSync_setClockTime(TimeSync_ParamsHandle_t timeSyncParamsHandle)
{
    uint64_t NewCycleCounter;
    uint32_t iepBaseAddress;
    uint8_t *bytePtr;

    if(timeSyncParamsHandle->emacHandle == NULL)
    {
        return ERROR_HANDLE_INVALID;
    }
    /*Calculate the absolute time in nanoseconds*/
    NewCycleCounter = timeSyncParamsHandle->clockTime.seconds *
                      (uint64_t)SEC_TO_NS + \
                      (uint64_t)timeSyncParamsHandle->clockTime.nanoseconds;
    /*Write it to IEP register*/
    iepBaseAddress = (uint32_t)(((PRUICSS_HwAttrs const *)(timeSyncParamsHandle->pruicssHandle->hwAttrs))->iep0RegBase);
    bytePtr = (uint8_t *)((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0));
    memcpy(bytePtr, &NewCycleCounter, 8);

    return 0;
}

void TimeSync_getCurrentTime(TimeSync_ParamsHandle_t timeSyncParamsHandle,
                             uint32_t *nanoseconds, uint64_t *seconds)
{
    uint32_t iepBaseAddress;
    uint8_t *bytePtr;
    uint64_t seconds_and_nanosec;

    iepBaseAddress = (uint32_t)(((PRUICSS_HwAttrs const *)(timeSyncParamsHandle->pruicssHandle->hwAttrs))->iep0RegBase);

    bytePtr = (uint8_t *)((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0));
    memcpy(&seconds_and_nanosec, bytePtr, 8);
    *nanoseconds = (uint32_t)(seconds_and_nanosec % (uint64_t)SEC_TO_NS);
    *seconds = seconds_and_nanosec / (uint64_t)SEC_TO_NS;
}

void TimeSync_updateDomainNumberInPTPFrames(TimeSync_ParamsHandle_t timeSyncHandle)
{
    uint8_t *bytePtr = NULL;
    uint8_t offset = timeSyncHandle->timeSyncConfig.frame_offset;

    if(P2P == timeSyncHandle->timeSyncConfig.type)
    {
        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.pdelayReq_TxBuf[0] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.pdelayReq_TxBuf[1] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.pdelayRes_TxBuf[0] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.pdelayRes_TxBuf[1] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.pdelayResFlwUp_TxBuf[0] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.pdelayResFlwUp_TxBuf[1] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.sync_TxBuf + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.announce_TxBuf + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.followUp_TxBuf[0] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.followUp_TxBuf[1] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];
    }
    else
    {
        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.sync_TxBuf + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.announce_TxBuf + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.followUp_TxBuf[0] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.followUp_TxBuf[1] + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];

        bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.delayReq_TxBuf + PTP_DOMAIN_NUM_OFFSET - offset);
        *bytePtr = timeSyncHandle->timeSyncConfig.domainNumber[0];
    }
}


void TimeSync_updateLogMessageIntervalInPTPFrames(TimeSync_ParamsHandle_t timeSyncHandle)
{
    uint8_t *bytePtr = NULL;
    uint8_t offset = timeSyncHandle->timeSyncConfig.frame_offset;

    /*Add log message interval to sync frame*/
    bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.sync_TxBuf +
                          PTP_LOG_MSG_PERIOD -
                          offset);
    *bytePtr = timeSyncHandle->timeSyncConfig.logSyncInterval;

    /*add announce send interval to announce frame*/
    bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.announce_TxBuf +
                          PTP_LOG_MSG_PERIOD -
                          offset);
    *bytePtr = timeSyncHandle->timeSyncConfig.logAnnounceSendInterval;
}

void TimeSync_updateDscpValueInDelayRequestFrame(TimeSync_ParamsHandle_t timeSyncHandle,
                                                 uint8_t dscpValue)
{
    uint8_t *bytePtr = NULL;
    uint8_t offset = timeSyncHandle->timeSyncConfig.frame_offset;

    /*DSCP value should be set in the upper 6 bits of the byte*/
    dscpValue = (dscpValue & 0x3F) << 2;

    bytePtr = (uint8_t *)(timeSyncHandle->timeSyncBuff.delayReq_TxBuf + IP_DSCP_OFFSET - offset);

    /*Clearing the old value and updating it with new value*/
    *bytePtr = *bytePtr & 0x3;
    *bytePtr = *bytePtr | dscpValue;

    TimeSync_calcIPChecksum(timeSyncHandle->timeSyncBuff.delayReq_TxBuf);
}
