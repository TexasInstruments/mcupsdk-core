/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 *  \file     hostapp_utils.h
 *
 *  \brief    This file contains the function prototypes of miscellaneous
 *            Ethernet utilities.
 */

#ifndef ENETAPPETHUTILS_H_
#define ENETAPPETHUTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/** \brief Print wrapper */

#if defined(__ti__)
#define HostappUtils_print(args ...)        EnetAppUtils_print(args)
#else
#define HostappUtils_print(args ...)        printf(args)
#endif

/** \brief Pass status */
#define PASS                            true

/** \brief Fail status */
#define FAIL                            false

/** \brief Pass status */
#define CPSWETHUTILS_SOK                (0)

/** \brief Fail status */
#define CPSWETHUTILS_EFAIL              (-1)

/** \brief Host to network byte order conversion for short integer */
#define Enet_htons(a)                   ((((a) & 0x00FFU) << 8) | \
                                         (((a) & 0xFF00U) >> 8))

/** \brief Host to network byte order conversion for long integer */
#define Enet_htonl(a)                   ((((a) & 0xFF000000U) >> 24) | \
                                         (((a) & 0x00FF0000U) >> 8) |  \
                                         (((a) & 0x0000FF00U) << 8) |  \
                                         (((a) & 0x000000FFU) << 24))

/** \brief Network to host byte order conversion for short integer */
#define Enet_ntohl(a)                   Enet_htonl(a)

/** \brief Network to host byte order conversion for long integer */
#define Enet_ntohs(a)                   Enet_htons(a)

/** \brief Experimental EtherType used in test packets */
#define ETHERTYPE_EXPERIMENTAL1         (0x88b5U)

/** \brief Experimental EtherType used in test packets */
#define ETHERTYPE_EXPERIMENTAL2         (0x88b6U)

/** \brief Experimental EtherType used in test control packets */
#define ETHERTYPE_EXP_CONTROL           (0x86aaU)

/** \brief VLAN tag's Tag Protocol Identifier (TPID) */
#define ETHERTYPE_VLAN_TAG              (0x8100U)

/** \brief PTP V2 Protocol Identifier */
#define ETHERTYPE_PTP_V2_FRAME_TYPE     (0x88F7U)

/** \brief MAC address length in bytes */
#define ETH_MAC_ADDR_LEN              (6U)

/** \brief Total bytes in header */
#define ETH_HDR_LEN                     (14U)

/** \brief Max octets in payload */
#define ETH_PAYLOAD_LEN                 (1500U)

/** \brief VLAN tag length in bytes */
#define ETH_VLAN_TAG_LEN                (4U)

/** \brief Test frame's header length in bytes */
#define ETH_TEST_DATA_HDR_LEN           (4U)

/** \brief Test pattern 1 */
#define ETH_TEST_TYPE_PATTERN_1         (0U)

/** \brief Test pattern 2 */
#define ETH_TEST_TYPE_PATTERN_2         (1U)

/** \brief Test pattern 3 */
#define ETH_TEST_TYPE_PATTERN_3         (2U)

/** \brief Test pattern 4 */
#define ETH_TEST_TYPE_PATTERN_4         (3U)

/** \brief Total number of pattern types */
#define ETH_TEST_NUM_TYPES              (4U)

/** \brief Success */
#define ETH_TEST_PKT_SOK                (0)

/** \brief Invalid pattern type */
#define ETH_TEST_PKT_ETYPE              (1)

/** \brief Invalid frame size */
#define ETH_TEST_PKT_ESIZE              (2)

/** \brief Invalid frame content */
#define ETH_TEST_PKT_ECONTENT           (3)

/** \brief Ready command which indicates that the DUT is up and running */
#define CTRL_FRAME_CMD_DUT_READY        (0x01U)

/** \brief Ready command which indicates that the PC client is up and running */
#define CTRL_FRAME_CMD_PC_READY         (0x02U)

/** \brief Start command which indicates that the test can begin */
#define CTRL_FRAME_CMD_START            (0x03U)

/** \brief Stop command which indicates when the host or DUT want to end the test */
#define CTRL_FRAME_CMD_STOP             (0x04U)

/** \brief Large number of test iterations */
#define ETH_TEST_ITER_L_COUNT           (150000U)

/** \brief Med number of test iterations */
#define ETH_TEST_ITER_M_COUNT           (5000U)

/** \brief Small number of test iterations */
#define ETH_TEST_ITER_S_COUNT           (10U)

/** \brief Smallest length when generating frames with random length */
#define ETH_TEST_BUF_LEN_MIN            (10U)

/** \brief Largest length when generating frames with random length */
#define ETH_TEST_BUF_LEN_MAX            (1500U)

/** \brief VLAN PCP */
#define ETH_TEST_VLAN_PCP               (5U)

/** \brief VLAN VID */
#define ETH_TEST_VLAN_VID               (1024U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct
{
    uint8_t dstMac[ETH_MAC_ADDR_LEN];
    uint8_t srcMac[ETH_MAC_ADDR_LEN];
    uint16_t etherType;
} __attribute__ ((packed)) EthFrameHeader;

typedef struct
{
    EthFrameHeader hdr;
    uint8_t payload[ETH_PAYLOAD_LEN + ETH_VLAN_TAG_LEN];
} __attribute__ ((packed)) EthFrame;

typedef struct
{
    uint8_t dstMac[ETH_MAC_ADDR_LEN];
    uint8_t srcMac[ETH_MAC_ADDR_LEN];
    uint16_t tpid;
    uint16_t tci;
    uint16_t etherType;
} __attribute__ ((packed)) EthVlanFrameHeader;

typedef struct
{
    EthVlanFrameHeader hdr;
    uint8_t payload[ETH_PAYLOAD_LEN];
} __attribute__ ((packed)) EthVlanFrame;

typedef struct
{
    uint8_t cmd;
    union
    {
        uint8_t status;
        uint8_t payload[45];
    } data;
} __attribute__ ((packed)) CtrlFramePayload;

typedef struct
{
    EthFrameHeader hdr;
    CtrlFramePayload payload;
} __attribute__ ((packed)) CtrlFrame;

typedef struct
{
    uint8_t type;
    uint8_t rsvd;
    uint16_t len;
    uint8_t data[0];
} __attribute__ ((packed)) DataFramePayload;

typedef struct
{
    EthFrameHeader hdr;
    DataFramePayload payload;
} __attribute__ ((packed)) DataFrame;

typedef struct
{
    uint16_t tci;
    uint16_t etherType;
    DataFramePayload payload;
} __attribute__ ((packed)) VlanDataFramePayload;

typedef struct
{
    EthFrameHeader hdr;
    VlanDataFramePayload payload;
} __attribute__ ((packed)) VlanDataFrame;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static inline bool EthFrame_isVlanTagged(EthFrame *frame)
{
    return frame->hdr.etherType == Enet_htons(ETHERTYPE_VLAN_TAG);
}

static inline bool EthFrame_isCtrl(EthFrame *frame)
{
    return frame->hdr.etherType == Enet_htons(ETHERTYPE_EXP_CONTROL);
}

static inline bool EthFrame_isStopCmd(EthFrame *frame)
{
    CtrlFrame *ctrlFrame = (CtrlFrame *)frame;

    return EthFrame_isCtrl(frame) && (ctrlFrame->payload.cmd == CTRL_FRAME_CMD_STOP);
}

static inline int32_t EthFrame_changeVlanId(EthVlanFrame *frame,
                                            uint16_t vlanId)
{
    int32_t status = CPSWETHUTILS_SOK;

    if (true == EthFrame_isVlanTagged((EthFrame *)frame))
    {
        uint16_t tci;

        tci            = Enet_ntohs(frame->hdr.tci);
        tci            = ((tci & ~0xFFF) | vlanId);
        frame->hdr.tci = Enet_ntohs(tci);
    }
    else
    {
        status = CPSWETHUTILS_EFAIL;
    }

    return status;
}

static inline void EthFrame_decrementTTL(EthVlanFrame *frame)
{
    uint8_t *IpFrameTTL;

    IpFrameTTL  = &frame->payload[8U];
    *IpFrameTTL = *IpFrameTTL - 1U;
}

int32_t HostappUtils_checkPayload(DataFramePayload *payload);

int32_t HostappUtils_checkVlanPayload(VlanDataFramePayload *payload);

int32_t HostappUtils_checkVlanTagAndPayload(VlanDataFramePayload *payload,
                                            uint8_t pcp,
                                            uint16_t vid,
                                            uint16_t etherType);

int32_t HostappUtils_fillPayload(DataFramePayload *payload,
                                 uint16_t type,
                                 uint16_t len);

int32_t HostappUtils_fillVlanPayload(VlanDataFramePayload *payload,
                                     uint16_t type,
                                     uint16_t len,
                                     uint8_t pcp,
                                     uint16_t vid,
                                     uint16_t etherType);

void HostappUtils_printFrame(EthFrame *frame,
                             int len);

static inline uint32_t HostappUtils_rand(uint32_t min,
                                         uint32_t max)
{
    return (rand() % (max - min + 1)) + min;
}

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENETAPPETHUTILS_H_ */
