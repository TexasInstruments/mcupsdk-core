/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
/*                              Include Files                                 */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <tsn_unibase/unibase.h>
#include <tsn_combase/combase.h>
#include <tsn_combase/cb_tmevent.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_unibase/unibase_binding.h>
#include <tsn_uniconf/yangs/yang_db_runtime.h>
#include <tsn_uniconf/yangs/yang_modules.h>
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>

#ifdef GPTP_ENABLED
#include <tsn_gptp/gptpmasterclock.h>
#endif

#include "debug_log.h"
#include "tsninit.h"
#include "common.h"
#include "qosapp_misc.h"

/*============================================================================*/
/*                          Macros and Constants                              */
/*============================================================================*/
#define DISPLAY_BITRATE_INTERVAL_SEC (20)
static const uint8_t STREAMID_PREFIX[7]= {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
static uint8_t MCAST_MAC_ADDR[ENET_MAC_ADDR_LEN] = {0x91, 0xE0, 0xF0, 0x00, 0xFE, 0x00};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
uint64_t EnetQoSApp_getCurrentTimeUs(void)
{
    return (cb_lld_gettime64(UB_CLOCK_REALTIME)/1000U);
}

void EnetQoSApp_initBitrateCtrl(BitrateCtrl_t *bc,
                                uint32_t maxCapacity,
                                uint64_t bitRate)
{
    bc->maxCapacity = maxCapacity;
    bc->tokens = bc->maxCapacity;
    bc->bitRate = bitRate;
    bc->lastTs = 0;
}

void EnetQoSApp_deinitBitrateCtrl(BitrateCtrl_t *bc)
{
    memset(bc, 0, sizeof(BitrateCtrl_t));
}

static void EnetQoSApp_updateTokensForBitrateCtrl(BitrateCtrl_t *bc)
{
    if (bc->lastTs == 0)
    {
        bc->lastTs = EnetQoSApp_getCurrentTimeUs();
    }
    else
    {
        uint64_t curTs = EnetQoSApp_getCurrentTimeUs();
        uint64_t delta = curTs > bc->lastTs? curTs - bc->lastTs: 0;
        if (delta > 0)
        {
            uint64_t tokens = ((bc->bitRate)*delta)/1000000U;
            bc->tokens += (uint32_t)(tokens/8);
            if (bc->tokens > bc->maxCapacity)
            {
                bc->tokens = bc->maxCapacity;
            }
        }
        bc->lastTs = curTs;
    }
}

bool EnetQoSApp_checkTransmitReady(BitrateCtrl_t *bc,
                                   uint32_t bytes,
                                   uint64_t *sleepTimeUs)
{
    bool res = false;
    EnetQoSApp_updateTokensForBitrateCtrl(bc);
    if (bytes <= bc->tokens)
    {
        bc->tokens -= bytes;
        res = true;
        *sleepTimeUs = 0;
    }
    else
    {
        uint64_t tokens = bytes-bc->tokens;
        *sleepTimeUs = (tokens*8*1000000U)/bc->bitRate;
        if (*sleepTimeUs == 0)
        {
            res = true;
            bc->tokens = 0;
        }
    }
    return res;
}

/* mode could be "w" for writing  or "r" for reading */
int EnetQoSApp_openDB(EnetApp_dbArgs *dbarg, char *dbName, const char *mode)
{
    int res = 0;
    int timeout_ms = 500;
    do {
        res = uniconf_ready(dbName, UC_CALLMODE_THREAD, timeout_ms);
        if (res != 0)
        {
            DPRINT("The uniconf must be run first!");
            break;
        }
        res = -1;
        dbarg->dbald = uc_dbal_open(dbName, mode, UC_CALLMODE_THREAD);
        if (!dbarg->dbald)
        {
            DPRINT("Failed to open DB for EstApp!");
            break;
        }
        dbarg->ucntd = uc_notice_init(UC_CALLMODE_THREAD, dbName);
        if (!dbarg->ucntd)
        {
            DPRINT("Failed to open uc notice!");
            break;
        }
        dbarg->ydrd = yang_db_runtime_init(dbarg->dbald, NULL);
        if (!dbarg->ydrd)
        {
            DPRINT("Failed to init DB runtime!");
            break;
        }
        res = 0;
    } while (0);
    return res;
}

void EnetQoSApp_closeDB(EnetApp_dbArgs *dbarg)
{
    uc_notice_close(dbarg->ucntd, 0);
    yang_db_runtime_close(dbarg->ydrd);
    uc_dbal_close(dbarg->dbald, UC_CALLMODE_THREAD);
}

int EnetQoSApp_setCommonParam(QoSAppCommonParam_t *prm,
                              EnetApp_dbArgs *dbarg)
{
    int err = 0, i;
    char buffer[MAX_KEY_SIZE];
    char val[MAX_VAL_SIZE];
    yang_db_runtime_dataq_t *ydrd = dbarg->ydrd;

    /* Write the num of traffic classes and value of each TC to DB */
    snprintf(buffer, sizeof(buffer),
             TRAFFIC_CLASS_TABLE_NODE"/number-of-traffic-classes",
             prm->netdev);
    snprintf(val, sizeof(val), "%d", prm->nTCs);
    YANGDB_RUNTIME_WRITE(buffer, val);

    /* Use one-to-one mapping of priority to logical queue */
    for (i = 0; i < prm->nTCs; i++)
    {
        snprintf(buffer, sizeof(buffer),
                 TRAFFIC_CLASS_TABLE_NODE"/priority%d",
                 prm->netdev, i);
        snprintf(val, sizeof(val), "%d",
                 prm->priority2TcMapping[i]);
        YANGDB_RUNTIME_WRITE(buffer, val);

        /* Map same number of priority to logical queue */
        snprintf(buffer, sizeof(buffer),
                 TRAFFIC_CLASS_DATA_NODE"|tc:%d|/lqueue",
                 prm->netdev, prm->priority2TcMapping[i]);
        snprintf(val, sizeof(val), "%d", i);
        YANGDB_RUNTIME_WRITE(buffer, val);
    }

    snprintf(buffer, sizeof(buffer),
             TRAFFIC_CLASS_NODE"/number-of-pqueues", prm->netdev);
    snprintf(val, sizeof(val), "%d", prm->nQueues);
    YANGDB_RUNTIME_WRITE(buffer, val);

    /* Use one-to-one mapping of logical queue to HW queue */
    for (i = 0; i < prm->nQueues; i++)
    {
        snprintf(buffer, sizeof(buffer),
                 PHYSICAL_QUEUE_MAP_NODE"|pqueue:%d|/lqueue",
                 prm->netdev, i);
        snprintf(val, sizeof(val), "%d", i);
        YANGDB_RUNTIME_WRITE(buffer, val);
    }

    return err;
}

static void EnetQoSApp_rxNotifyCb(void* arg)
{
    EnetQoSApp_AppCtx_t *ctx = (EnetQoSApp_AppCtx_t *)arg;
    if (ctx->listener.rxPacketSem)
    {
        CB_SEM_POST(&ctx->listener.rxPacketSem);
    }
}

int EnetQoSApp_initialize(EnetQoSApp_AppCtx_t *ctx,
                          EnetApp_ModuleCtx_t *modCtx,
                          PacketHandlerCb cb)
{
    int err, i;
    cb_rawsock_paras_t param;
    EnetApp_Ctx_t *ectx = (EnetApp_Ctx_t *)modCtx->appCtx;

    ctx->ifidx = DEFAULT_INTERFACE_INDEX;
    DebugP_assert(ectx->netdevSize > 0);
    DebugP_assert(ctx->ifidx < ectx->netdevSize);
    for (i = 0; i < ectx->netdevSize; i++)
    {
        ctx->netdev[i] = ectx->netdev[i];
    }
    ctx->netdevSize = ectx->netdevSize;

    ctx->ectx = ectx;

    ctx->packetHandlerCb = cb;
    ctx->talker.vid = DEFAULT_VLAN_ID;
    ctx->talker.nStreams = QOSAPP_NUM_OF_STREAMS;
    ctx->talker.nTCs = QOSAPP_NUM_OF_STREAMS;

    memset(&param, 0, sizeof(param));
    param.dev = ctx->netdev[ctx->ifidx];
    param.proto = ETH_P_TSN;
    param.vlan_proto = param.proto;
    param.rw_type = CB_RAWSOCK_RDWR;
    param.sock_mode = CB_SOCK_MODE_NORMAL;
    param.vlanid = DEFAULT_VLAN_ID;
    int mtuSize = 0;
    err = cb_rawsock_open(&param, &ctx->est_sock,
                          &ctx->sockAddress, &mtuSize,
                          ctx->source_mac);
    if (err)
    {
        DPRINT("Failed to open socket for EST app!, err: %d", err);
    }
    else
    {
        DebugP_assert(mtuSize >= sizeof(ctx->talker.buffer));
        err = cb_lld_set_rxnotify_cb(ctx->est_sock,
                                     EnetQoSApp_rxNotifyCb,
                                     ctx);
    }

    if (err)
    {
        DPRINT("Failed to set rx notify callback!, err: %d", err);
    }
    else
    {
#ifdef GPTP_ENABLED
        /* Initialize gptpmaster clock  to get PTP time */
        while (gptpmasterclock_init(NULL))
        {
            /* waiting for the gPTP running first  */
            CB_USLEEP(2000000ULL);
        }
#endif
    }

    return err;
}

int EnetQoSApp_deinitialize(EnetQoSApp_AppCtx_t *ctx)
{
    cb_rawsock_close(ctx->est_sock);
#ifdef GPTP_ENABLED
    gptpmasterclock_close();
#endif
    return 0;
}

static void EnetQoSApp_createAVTPHeader(StreamParam_t *sprm,
                                        AVTPCommonStreamHdr_t *avtphdr,
                                        uint16_t dataLen)
{
    int64_t pts = 0;
    avtphdr->hh.subtype = AVTP_AAF_SUBTYPE;
    avtphdr->pdLength = Enet_htons(dataLen);
    memcpy(avtphdr->streamId, sprm->sid, sizeof(ub_streamid_t));
    avtphdr->hh.seqn = sprm->seqn;
    sprm->seqn++;
#ifdef GPTP_ENABLED
    pts = gptpmasterclock_getts64();
#endif
    /* Get lower 32 bits of the timestamp for avtp timestamp */
    avtphdr->headerTimestamp = Enet_htonl(pts);
    avtphdr->bf=cmsh_sv_set_bit_field(avtphdr->bf, 1);
    avtphdr->hh.bf0 |= 0x1;
}

static void EnetQoSApp_initStreamParams(EnetQoSApp_AppCtx_t *ctx)
{
    int i;
    EnetQoSApp_TaskCtx_t *talker = &ctx->talker;

    uint8_t streamId[8];

    EthVlanFrame *txFrame;
    txFrame = (EthVlanFrame *)talker->buffer;
    memcpy(streamId, STREAMID_PREFIX, sizeof(STREAMID_PREFIX));

    /* Initialize constant data for ethernet header which are used for all streams */
    memcpy(txFrame->hdr.dstMac, MCAST_MAC_ADDR, ENET_MAC_ADDR_LEN);
    memcpy(txFrame->hdr.srcMac, ctx->source_mac, ENET_MAC_ADDR_LEN);
    txFrame->hdr.tpid = Enet_htons(ENETAPP_VLAN_TPID);
    txFrame->hdr.etherType = Enet_htons(ETH_P_TSN);

    for (i = 0; i < talker->nStreams; i++)
    {
        talker->streams[i].priority = i;
        talker->streams[i].tc = i; /* Default setting for TCs */
        streamId[7] = talker->streams[i].tc;
        memcpy(talker->streams[i].sid, streamId, sizeof(streamId));
        talker->streams[i].seqn = 0;
    }
}

static void EnetQosApp_initTalker(EnetQoSApp_TaskCtx_t *talker)
{
    int i;
    uint32_t payloadLen;
    uint64_t bitRate;

    for (i = 0; i < talker->nStreams; i++)
    {
        bitRate = talker->streams[i].bitRate;
        EnetQoSApp_initBitrateCtrl(&talker->bitrateCtrl[i],
                                   2*sizeof(talker->buffer), bitRate);
        payloadLen = talker->streams[i].payloadLen;
        char buffer[MAX_KEY_SIZE];
        snprintf(buffer, sizeof(buffer),
                 "Talker[%d], payloadLen: %d bytes, bitRate: %lld kbps",
                i, payloadLen, bitRate/1000);
        DPRINT("%s ", buffer);
    }
}

static void *EnetQoSApp_talkerHandler(void *arg)
{
    int i;
    EnetQoSApp_AppCtx_t *ctx = (EnetQoSApp_AppCtx_t *)arg;
    EnetQoSApp_TaskCtx_t *talker = &ctx->talker;
    uint64_t minSleepTime = UINT64_MAX;
    EthVlanFrame *txFrame;
    AVTPCommonStreamHdr_t *avtphdr;
    uint32_t payloadLen, frameLen;
    uint32_t nShortSleep = 0;
#define HIGH_CPU_LOAD_THRESHOLD (15)

    while (talker->enable)
    {
        for (i = 0; i < talker->nStreams; i++)
        {
            payloadLen = talker->streams[i].payloadLen;
            frameLen = sizeof(EthVlanFrameHeader) + payloadLen;
            uint64_t sleepTimeUs = 0;
            if (EnetQoSApp_checkTransmitReady(&talker->bitrateCtrl[i],
                                              frameLen,
                                              &sleepTimeUs))
            {
                txFrame = (EthVlanFrame *)talker->buffer;
                int tci = ENETAPP_VLAN_TCI(talker->streams[i].priority, 0, talker->vid);
                txFrame->hdr.tci  = Enet_htons(tci);
                avtphdr = (AVTPCommonStreamHdr_t *)txFrame->payload;
                uint16_t dataLen = payloadLen - sizeof(AVTPCommonStreamHdr_t);
                EnetQoSApp_createAVTPHeader(&talker->streams[i], avtphdr, dataLen);
                memset(&txFrame->payload[sizeof(AVTPCommonStreamHdr_t)],
                       talker->streams[i].tc, payloadLen - sizeof(AVTPCommonStreamHdr_t));
                int ret = CB_SOCK_SENDTO(ctx->est_sock, talker->buffer,
                                         frameLen, 0, &ctx->sockAddress,
                                         sizeof(ctx->sockAddress));
                if (ret == -1)
                {
                    DPRINT("Failed to send %d bytes on stream %d", frameLen, i);
                }
                minSleepTime = 0;
            }
            else
            {
                if (minSleepTime > sleepTimeUs)
                {
                    minSleepTime = sleepTimeUs;
                }
            }
        }

        /* sleep less than 1ms doesn't help to reduce cpuload. Therefore, to reduce 
         * the cpu load in case of high bitrate, it need to take a sufficient sleep (1ms)
         */
        nShortSleep = minSleepTime < UB_MSEC_US? (nShortSleep+1): 0;

        if (nShortSleep >= HIGH_CPU_LOAD_THRESHOLD && minSleepTime > 0)
        {
            minSleepTime = UB_MSEC_US;
            TaskP_yield();
            nShortSleep = 0;
        }
        if (minSleepTime != UINT64_MAX && minSleepTime > 0)
        {
            CB_USLEEP(minSleepTime);
        }
        minSleepTime = UINT64_MAX;
    }

    CB_SEM_POST(&talker->terminatedSem);
    DPRINT("EST App talker is terminating ...");

    return NULL;
}

static int EnetQoSApp_setInputParam(EnetQoSApp_AppCtx_t *ctx,
                                    QoSAppStreamConfigParam_t *stParams)
{
    int i, err = 0;
    EnetQoSApp_TaskCtx_t *talker = &ctx->talker;
    if (stParams->nStreams == 0)
    {
        DPRINT("Invalid number of stream: %d", stParams->nStreams);
        err = -1;
    }
    else
    {
        talker->nStreams = stParams->nStreams;
        for (i = 0; i < stParams->nStreams; i++)
        {
            talker->streams[i].bitRate = stParams->streamParams[i].bitRateKbps*1000U;
            talker->streams[i].payloadLen = stParams->streamParams[i].payloadLen;
            talker->streams[i].priority = stParams->streamParams[i].priority;
            talker->streams[i].tc = stParams->streamParams[i].tc;
            /* Set the last byte of stream if to priority */
            talker->streams[i].sid[7u] = talker->streams[i].priority;
        }
    }

    return err;
}

static void EnetQoSApp_initTaskCtx(cb_tsn_thread_attr_t *attr,
                                   EnetQoSApp_TaskCfg_t *cfg)
{
    cb_tsn_thread_attr_init(attr, cfg->priority,
                            cfg->stackBufferSize,
                            cfg->name);
    cb_tsn_thread_attr_set_stackaddr(attr, cfg->stackBuffer);
}

void EnetQoSApp_startTalker(EnetQoSApp_AppCtx_t *ctx,
                            EnetQoSApp_TaskCfg_t *cfg,
                            QoSAppStreamConfigParam_t *stParams)
{
    int err = 0;
    EnetQoSApp_TaskCtx_t *talker = &ctx->talker;

    DebugP_assert(stParams != NULL);
    if (!talker->enable)
    {
        if (CB_SEM_INIT(&talker->terminatedSem, 0, 0) < 0)
        {
            DPRINT("Failed to create terminatedSem");
            err = -1;
        }
        talker->enable = true;
        if (!talker->hTaskHandle && err == 0)
        {
            cb_tsn_thread_attr_t attr;

            /* Init default parameters for all streams */
            EnetQoSApp_initStreamParams(ctx);

            EnetQoSApp_initTaskCtx(&attr, cfg);

            err = EnetQoSApp_setInputParam(ctx, stParams);

            if (err == 0)
            {
                EnetQosApp_initTalker(&ctx->talker);
                err = CB_THREAD_CREATE(&talker->hTaskHandle,
                                       &attr, EnetQoSApp_talkerHandler,
                                       ctx);
            }
            else
            {
                talker->enable = false;
                DPRINT("User request to terminate the talker");
            }
        }
        if (err != 0)
        {
            DPRINT("Failed to start the talker task!");
            CB_SEM_DESTROY(&talker->terminatedSem);
            talker->enable = false;
        }
        else
        {
            DPRINT("Start the talker successfully!");
        }
    }
    else
    {
        DPRINT("The talker has been started, no need to start again!");
    }
}

void EnetQoSApp_stopTalker(EnetQoSApp_AppCtx_t *ctx)
{
    if (ctx->talker.enable)
    {
        ctx->talker.enable = false;
        CB_SEM_WAIT(&ctx->talker.terminatedSem);
        CB_THREAD_JOIN(ctx->talker.hTaskHandle, NULL);

        ctx->talker.hTaskHandle = NULL;
        CB_SEM_DESTROY(&ctx->talker.terminatedSem);
        DPRINT("Terminate the talker succesfully!");
    }
}

void EnetQoSApp_stopListener(EnetQoSApp_AppCtx_t *ctx)
{
    if (ctx->listener.enable)
    {
        ctx->listener.enable = false;
        CB_SEM_WAIT(&ctx->listener.terminatedSem);
        ctx->listener.hTaskHandle = NULL;
        CB_SEM_DESTROY(&ctx->listener.terminatedSem);
        CB_SEM_DESTROY(&ctx->listener.rxPacketSem);
    }
}

static inline  void EnetQoSApp_showRecvBitrate(StreamParam_t *stream,
                                               EnetQoSApp_Packet_t *pktinfo)
{
    int64_t now = EnetQoSApp_getCurrentTimeUs();
    uint64_t duration  = now > stream->prevTs? now - stream->prevTs: 0ULL;

    if (duration >= DISPLAY_BITRATE_INTERVAL_SEC*UB_SEC_US)
    {
        char buffer[MAX_LOG_LEN];
        int n;
        n = snprintf(buffer, sizeof(buffer),
                     "PacketPriority[%d], frameLength: %dB, receivedBitrate: %lluKbps, nBrokenPackets: %d",
                     pktinfo->recvAddr.tcid, pktinfo->bufferSize, (stream->rxBytes*8*1000)/duration,
                     stream->nBrokenPkt);
        DebugP_assert(n < sizeof(buffer));
        DPRINT("%s ", buffer);

        stream->prevTs = now;
        stream->rxBytes = 0ULL;
        stream->nBrokenPkt = 0U;
    }
}

static void EnetQoSApp_handleRxPacket(EnetQoSApp_AppCtx_t *ctx,
                                      uint8_t *buffer, int size,
                                      CB_SOCKADDR_LL_T *addr)
{
    EnetQoSApp_TaskCtx_t *listener = &ctx->listener;
    EthVlanFrame *txFrame = (EthVlanFrame *)buffer;
    int tci = Enet_ntohs(txFrame->hdr.tci);
    uint8_t pcp = (tci >> ENETAPP_VLAN_PCP_OFFSET)&ENETAPP_VLAN_PCP_MASK;
    if (pcp >= QOSAPP_MAX_STREAMS)
    {
        DPRINT("Received packets with unexpected PCP: %d", pcp);
    }
    else
    {
        AVTPCommonStreamHdr_t *avtphdr;
        uint64_t timestamp = 0;
        avtphdr = (AVTPCommonStreamHdr_t *)txFrame->payload;
        uint8_t expectedSeqn = listener->streams[pcp].seqn+1;
        if (expectedSeqn != avtphdr->hh.seqn)
        {
            listener->streams[pcp].nBrokenPkt++;
        }
        listener->streams[pcp].seqn = avtphdr->hh.seqn;
        listener->streams[pcp].rxBytes += size;
#ifdef GPTP_ENABLED
        timestamp = gptpmasterclock_expand_timestamp(Enet_ntohl(avtphdr->headerTimestamp));
#endif
        listener->streams[pcp].txts = timestamp;
        listener->streams[pcp].tc = pcp;

        EnetQoSApp_Packet_t pktinfo;
        pktinfo.buffer = buffer;
        pktinfo.bufferSize = size;
        memcpy(&pktinfo.recvAddr, addr, sizeof(CB_SOCKADDR_LL_T));
        pktinfo.recvAddr.tcid = pcp;

        EnetQoSApp_showRecvBitrate(&listener->streams[pcp], &pktinfo);
        listener->streams[pcp].rxts = addr->rxts;

        ctx->packetHandlerCb(ctx, &pktinfo);
    }
}
#if LISTNER_VERIFY_ENABLE
static void *EnetQoSApp_listenerHandler(void *arg)
{
    int ret;
    EnetQoSApp_AppCtx_t *ctx = (EnetQoSApp_AppCtx_t *)arg;
    EnetQoSApp_TaskCtx_t *listener = &ctx->listener;
    CB_SOCKADDR_LL_T addr;

    ret = cb_reg_multicast_address(ctx->est_sock, ctx->netdev[ctx->ifidx],
                                   MCAST_MAC_ADDR, 0);
    DebugP_assert(ret == 0);
    while (listener->enable)
    {
        ret = cb_lld_recv(ctx->est_sock, listener->buffer,
                          sizeof(listener->buffer),
                          &addr, sizeof(CB_SOCKADDR_LL_T));
        if (ret <= 0)
        {
            TaskP_yield();
            ret = CB_SEM_WAIT(&listener->rxPacketSem);
            if (ret != 0)
            {
                DPRINT("%s, Failed to wait rxPacketSem", __func__);
                break;
            }
        }
        else if (ret == 0xFFFF)
        {
            /* When it return 0xFFFF, this packet belongs to another app,
             * this application just ignore the packet
             */
            DPRINT("Received an unexpected packet !");
        }
        else
        {
            EnetQoSApp_handleRxPacket(ctx, listener->buffer,
                                      ret, &addr);
        }
    }

    DPRINT("The listener thread is terminating");

    return NULL;
}

void EnetQoSApp_startListener(EnetQoSApp_AppCtx_t *ctx,
                              EnetQoSApp_TaskCfg_t *cfg)
{
    int err = 0;
    EnetQoSApp_TaskCtx_t *listener = &ctx->listener;

    listener->enable = true;
    err = CB_SEM_INIT(&listener->rxPacketSem, 0, 0);
    DebugP_assert(err == 0);
    err = CB_SEM_INIT(&listener->terminatedSem, 0, 0);
    DebugP_assert(err == 0);
    if (!listener->hTaskHandle)
    {
        cb_tsn_thread_attr_t attr;
        EnetQoSApp_initTaskCtx(&attr, cfg);
        err = CB_THREAD_CREATE(&listener->hTaskHandle,
                               &attr, EnetQoSApp_listenerHandler,
                               ctx);
        if (err)
        {
            ctx->listener.enable = false;
            ctx->listener.hTaskHandle = NULL;
            CB_SEM_DESTROY(&ctx->listener.terminatedSem);
            CB_SEM_DESTROY(&ctx->listener.rxPacketSem);
            DPRINT("Failed to create listener task!");
        }
        else
        {
            DPRINT("Create listener task succesfully!");
        }
    }
}
#endif

void EnetQoSApp_printStats(EnetQoSApp_AppCtx_t *ctx)
{
    cb_tilld_port_stats_t stats;
    int err, port, i;

    err = cb_lld_get_port_stats(ctx->est_sock, 0xFFU, &stats);
    if (err)
    {
        DPRINT("Failed to get host port stats: %d", err);
    }
    else
    {
        DPRINT("----------------Host port stats info --------------------");
        EnetAppUtils_printHostPortStats9G((CpswStats_HostPort_Ng *)&stats);

        for (i = 0; i < ctx->netdevSize; i++)
        {
            port = cb_lld_netdev_to_macport(ctx->netdev[i]);
            if (port < 0)
            {
                DPRINT("Invalid mac port translated from netdev %s",
                       ctx->netdev[i]);
            }
            else
            {
                memset(&stats, 0, sizeof(cb_tilld_port_stats_t));

                err = cb_lld_get_port_stats(ctx->est_sock, port, &stats);
                if (err == 0)
                {
                    DPRINT("---------MAC port[%d] stats info -----------", i);
                    EnetAppUtils_printMacPortStats9G((CpswStats_MacPort_Ng *)&stats);
                }
                else
                {
                    DPRINT("Failed to get MAC port stats info");
                }
            }
        }
    }
}

void EnetQoSApp_resetStats(EnetQoSApp_AppCtx_t *ctx)
{
    int32_t i, port;
    cb_lld_reset_port_stats(ctx->est_sock, 0xFFU);
    for (i = 0; i < ctx->netdevSize; i++)
    {
        port = cb_lld_netdev_to_macport(ctx->netdev[i]);
        cb_lld_reset_port_stats(ctx->est_sock, port);
    }
}

int8_t EnetQoSApp_getPortIdx(EnetQoSApp_AppCtx_t *ctx, char *netdev)
{
    int i;

    for (i = 0; i < ctx->netdevSize; i++)
    {
        if (strncmp(netdev, ctx->netdev[i], strlen(netdev)) == 0)
        {
            return i;
        }
    }

    return -1;
}
