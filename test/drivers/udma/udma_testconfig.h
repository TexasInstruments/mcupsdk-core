/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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

/**
 *  \file udma_testconfig.h
 *
 *  \brief This file defines the common configurations like driver config etc...
 */

#ifndef UDMA_TEST_CONFIG_H_
#define UDMA_TEST_CONFIG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <udma_test.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Defines the various TX channel parameters. */
static const UdmaTestTxChPrm gUdmaTestTxChPrm[] =
{
    {
        .txChPrmId      = UDMA_TEST_TXCH_PRMID_DEF,
        .txPrms         =
        {
            .pauseOnError   = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED,
            .filterEinfo    = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED,
            .filterPsWords  = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_DISABLED,
            .addrType       = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS,
            .chanType       = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_BLOCK_REF,
            .fetchWordSize  = 16U,
            .busPriority    = UDMA_DEFAULT_TX_CH_BUS_PRIORITY,
            .busQos         = UDMA_DEFAULT_TX_CH_BUS_QOS,
            .busOrderId     = UDMA_DEFAULT_TX_CH_BUS_ORDERID,
            .dmaPriority    = UDMA_DEFAULT_TX_CH_DMA_PRIORITY,
            .txCredit       = 0U,
            .fifoDepth      = 128U,
            .burstSize      = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_64_BYTES,
            .supressTdCqPkt = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_SUPPRESS_TD_DISABLED,
        }
    },
    {
        .txChPrmId      = UDMA_TEST_TXCH_PRMID_DMA_PRIORITY_HIGH,
        .txPrms         =
        {
            .pauseOnError   = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED,
            .filterEinfo    = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED,
            .filterPsWords  = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_DISABLED,
            .addrType       = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS,
            .chanType       = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_BLOCK_REF,
            .fetchWordSize  = 16U,
            .busPriority    = UDMA_DEFAULT_TX_CH_BUS_PRIORITY,
            .busQos         = UDMA_DEFAULT_TX_CH_BUS_QOS,
            .busOrderId     = UDMA_DEFAULT_TX_CH_BUS_ORDERID,
            .dmaPriority    = TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_HIGH,
            .txCredit       = 0U,
            .fifoDepth      = 128U,
            .burstSize      = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_64_BYTES,
            .supressTdCqPkt = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_SUPPRESS_TD_DISABLED,
        }
    },
};
#define UDMA_TEST_NUM_TX_CH_PRM         (sizeof(gUdmaTestTxChPrm) / \
                                         sizeof(gUdmaTestTxChPrm[0U]))

/** \brief Defines the various RX channel parameters. */
static const UdmaTestRxChPrm gUdmaTestRxChPrm[] =
{
    {

        .rxChPrmId      = UDMA_TEST_RXCH_PRMID_DEF,
        .rxPrms         =
        {
            .pauseOnError       = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED,
            .addrType           = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS,
            .chanType           = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_BLOCK_REF,
            .fetchWordSize      = 16U,
            .busPriority        = UDMA_DEFAULT_RX_CH_BUS_PRIORITY,
            .busQos             = UDMA_DEFAULT_RX_CH_BUS_QOS,
            .busOrderId         = UDMA_DEFAULT_RX_CH_BUS_ORDERID,
            .dmaPriority        = UDMA_DEFAULT_RX_CH_DMA_PRIORITY,
            .flowIdFwRangeStart = 0U,  /* Flow ID not used */
            .flowIdFwRangeCnt   = 0U,  /* Flow ID not used */
            .ignoreShortPkts    = TISCI_MSG_VALUE_RM_UDMAP_RX_CH_PACKET_EXCEPTION,
            .ignoreLongPkts     = TISCI_MSG_VALUE_RM_UDMAP_RX_CH_PACKET_EXCEPTION,
            .configDefaultFlow  = TRUE,
            .burstSize          = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_64_BYTES,
        }
    },
    {

        .rxChPrmId      = UDMA_TEST_RXCH_PRMID_DMA_PRIORITY_HIGH,
        .rxPrms         =
        {
            .pauseOnError       = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED,
            .addrType           = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS,
            .chanType           = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_BLOCK_REF,
            .fetchWordSize      = 16U,
            .busPriority        = UDMA_DEFAULT_RX_CH_BUS_PRIORITY,
            .busQos             = UDMA_DEFAULT_RX_CH_BUS_QOS,
            .busOrderId         = UDMA_DEFAULT_RX_CH_BUS_ORDERID,
            .dmaPriority        = TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIOR_HIGH,
            .flowIdFwRangeStart = 0U,  /* Flow ID not used */
            .flowIdFwRangeCnt   = 0U,  /* Flow ID not used */
            .ignoreShortPkts    = TISCI_MSG_VALUE_RM_UDMAP_RX_CH_PACKET_EXCEPTION,
            .ignoreLongPkts     = TISCI_MSG_VALUE_RM_UDMAP_RX_CH_PACKET_EXCEPTION,
            .configDefaultFlow  = TRUE,
            .burstSize          = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_64_BYTES,
        }
    },
};
#define UDMA_TEST_NUM_RX_CH_PRM         (sizeof(gUdmaTestRxChPrm) / \
                                         sizeof(gUdmaTestRxChPrm[0U]))

/** \brief Defines the various PDMA channel parameters. */
static const UdmaTestPdmaChPrm gUdmaTestPdmaChPrm[] =
{
    {
        .pdmaChPrmId    = UDMA_TEST_PDMACH_PRMID_DEF,
        .pdmaPrms       =
        {
            .elemSize  = UDMA_PDMA_ES_8BITS,
            .elemCnt   = 0U,
            .fifoCnt   = 0U,
        }
    },
    {
        .pdmaChPrmId    = UDMA_TEST_PDMACH_PRMID_ES_16BITS,
        .pdmaPrms       =
        {
            .elemSize  = UDMA_PDMA_ES_16BITS,
            .elemCnt   = 0U,
            .fifoCnt   = 0U,
        }
    },
};
#define UDMA_TEST_NUM_PDMA_CH_PRM       (sizeof(gUdmaTestPdmaChPrm) / \
                                         sizeof(gUdmaTestPdmaChPrm[0U]))

/** \brief Defines the various channel parameters. */
static const UdmaTestChPrm gUdmaTestChPrm[] =
{
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_DEF,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY,
        .eventMode      = UDMA_TEST_EVENT_NONE,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_NONE,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_INTR_DEF,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY,
        .eventMode      = UDMA_TEST_EVENT_INTR,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_NONE,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_TRIGGER_GLOBAL0,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY,
        .eventMode      = UDMA_TEST_EVENT_NONE,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_TRIGGER_GLOBAL0_INTR,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY,
        .eventMode      = UDMA_TEST_EVENT_INTR,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_EVENTSIZE_ICNT1,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY,
        .eventMode      = UDMA_TEST_EVENT_NONE,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT1_DEC,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_EVENTSIZE_ICNT2,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY,
        .eventMode      = UDMA_TEST_EVENT_NONE,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT2_DEC,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_EVENTSIZE_ICNT3,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY,
        .eventMode      = UDMA_TEST_EVENT_NONE,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT3_DEC,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT3_DEC,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_BLKCPY_HC_DEF,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY_HC,
        .eventMode      = UDMA_TEST_EVENT_NONE,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_NONE,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_BLKCPY_HC_INTR_DEF,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY_HC,
        .eventMode      = UDMA_TEST_EVENT_INTR,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_NONE,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_BLKCPY_UHC_DEF,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY_UHC,
        .eventMode      = UDMA_TEST_EVENT_NONE,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_NONE,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
    {
        .chPrmId        = UDMA_TEST_CH_PRMID_BLKCPY_UHC_INTR_DEF,
        .chType         = UDMA_CH_TYPE_TR_BLK_COPY_UHC,
        .eventMode      = UDMA_TEST_EVENT_INTR,
        .trigger        = CSL_UDMAP_TR_FLAGS_TRIGGER_NONE,
        .eventSize      = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION,
        .triggerType    = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL,
        .txPrmId        = UDMA_TEST_TXCH_PRMID_DEF,
        .rxPrmId        = UDMA_TEST_RXCH_PRMID_DEF,
        .utcPrmId       = UDMA_TEST_UTCCH_PRMID_INVALID,
        .pdmaPrmId      = UDMA_TEST_PDMACH_PRMID_INVALID,
    },
};
#define UDMA_TEST_NUM_CH_PRM            (sizeof(gUdmaTestChPrm) / \
                                         sizeof(gUdmaTestChPrm[0U]))

/** \brief Defines the various ring parameters. */
static const UdmaTestRingPrm gUdmaTestRingPrm[] =
{
    {
        .ringPrmId      = UDMA_TEST_RING_PRMID_EVENT_NONE,
        .eventMode      = UDMA_TEST_EVENT_NONE,
    },
    {
        .ringPrmId      = UDMA_TEST_RING_PRMID_EVENT_INTR,
        .eventMode      = UDMA_TEST_EVENT_INTR,
    },
    {
        .ringPrmId      = UDMA_TEST_RING_PRMID_EVENT_POLLED,
        .eventMode      = UDMA_TEST_EVENT_POLLED,
    },
};
#define UDMA_TEST_NUM_RING_PRM          (sizeof(gUdmaTestRingPrm) / \
                                         sizeof(gUdmaTestRingPrm[0U]))

/** \brief Defines the various PKTDMA channel parameters. */
static const UdmaTestPktdmaChPrm gUdmaTestPktdmaChPrm[] =
{
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_UNMAPPED_TX,
        .chType         = UDMA_CH_TYPE_TX,
        .mappedChGrp    = UDMA_MAPPED_GROUP_INVALID,
        .peerChNum      = UDMA_TEST_PKTDMA_UNMAPPED_TX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_CPSW_TX,
        .chType         = UDMA_CH_TYPE_TX_MAPPED,
        .mappedChGrp    = UDMA_MAPPED_TX_GROUP_CPSW,
        .peerChNum      = UDMA_TEST_PKTDMA_CPSW_TX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_SAUL_TX,
        .chType         = UDMA_CH_TYPE_TX_MAPPED,
        .mappedChGrp    = UDMA_MAPPED_TX_GROUP_SAUL,
        .peerChNum      = UDMA_TEST_PKTDMA_SAUL_TX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_ICSSG_0_TX,
        .chType         = UDMA_CH_TYPE_TX_MAPPED,
        .mappedChGrp    = UDMA_MAPPED_TX_GROUP_ICSSG_0,
        .peerChNum      = UDMA_TEST_PKTDMA_ICSSG_0_TX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_ICSSG_1_TX,
        .chType         = UDMA_CH_TYPE_TX_MAPPED,
        .mappedChGrp    = UDMA_MAPPED_TX_GROUP_ICSSG_1,
        .peerChNum      = UDMA_TEST_PKTDMA_ICSSG_1_TX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_UNMAPPED_RX,
        .chType         = UDMA_CH_TYPE_RX,
        .mappedChGrp    = UDMA_MAPPED_GROUP_INVALID,
        .peerChNum      = UDMA_TEST_PKTDMA_UNMAPPED_RX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_CPSW_RX,
        .chType         = UDMA_CH_TYPE_RX_MAPPED,
        .mappedChGrp    = UDMA_MAPPED_RX_GROUP_CPSW,
        .peerChNum      = UDMA_TEST_PKTDMA_CPSW_RX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_SAUL_RX,
        .chType         = UDMA_CH_TYPE_RX_MAPPED,
        .mappedChGrp    = UDMA_MAPPED_RX_GROUP_SAUL,
        .peerChNum      = UDMA_TEST_PKTDMA_SAUL_RX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_ICSSG_0_RX,
        .chType         = UDMA_CH_TYPE_RX_MAPPED,
        .mappedChGrp    = UDMA_MAPPED_RX_GROUP_ICSSG_0,
        .peerChNum      = UDMA_TEST_PKTDMA_ICSSG_0_RX_PEER_CH,
    },
    {
        .pktdmachPrmId  = UDMA_TEST_PKTDMA_CH_PRMID_ICSSG_1_RX,
        .chType         = UDMA_CH_TYPE_RX_MAPPED,
        .mappedChGrp    = UDMA_MAPPED_RX_GROUP_ICSSG_1,
        .peerChNum      = UDMA_TEST_PKTDMA_ICSSG_1_RX_PEER_CH,
    },
};
#define UDMA_TEST_NUM_PKTDMA_CH_PRM          (sizeof(gUdmaTestPktdmaChPrm) / \
                                              sizeof(gUdmaTestPktdmaChPrm[0U]))

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_TEST_CONFIG_H_ */
