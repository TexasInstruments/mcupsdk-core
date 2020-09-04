/**
 * \file hsrPrp_red_common.h
 * \brief Commonly used Macros, structures and Offsets
 *
 * \par
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
 * \par
 */

#ifndef RED_COMMON_H_
#define RED_COMMON_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <networking/icss_emac/icss_emac.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Endianness */
#if __BYTE_ORDER == __LITTLE_ENDIAN

#define OS_HostToNet16(x)   ((((x) & 0x00ff) << 8) | \
                             (((x) & 0xff00) >> 8))

#define OS_NetToHost16(x)   ((((x) & 0x00ff) << 8) | \
                             (((x) & 0xff00) >> 8))

#define OS_HostToNet32(x)   ((((x) & 0x000000ff) << 24) | \
                             (((x) & 0x0000ff00) <<  8) | \
                             (((x) & 0x00ff0000) >>  8) | \
                             (((x) & 0xff000000) >> 24))


#define OS_NetToHost32(x)   ((((x) & 0x000000ff) << 24) | \
                             (((x) & 0x0000ff00) <<  8) | \
                             (((x) & 0x00ff0000) >>  8) | \
                             (((x) & 0xff000000) >> 24))
#endif

#define RED_SUP_SIZE                 ( 52 )  /**< LSDU size of a RED Supervision frame */
#define RED_SUP_PATH                  ( 0 )  /**< Unused, set to 0 */
#define RED_SUP_VER                   ( 1 )  /**< Protocol version */
#define RED_SUP_PAD                  ( 24 )  /**< Number of padding bytes in RED Supervision frame */

#define ETHER_ADDR_LEN                ( 6 )  /**< Ethernet address length is 6 bytes */

#define ETHER_TYPE_ARP           ( 0x0806 )  /**< ARP ether type */
#define ETHER_TYPE_VLAN          ( 0x8100 )  /**< VLAN ether type */
#define ETHER_TYPE_RED_SUP       ( 0x88FB )  /**< Ether type for supervision frames */
#define ETHER_TYPE_RED_SUP_BIG_ENDIEN ( 0xFB88 )  /**< Supervision frame identifier in Big Endian format*/
#define ETHER_TYPE_SIZE               ( 2 )  /**< Ether type size */
#define ETHER_HEADER_LEN             ( 14 )  /**< Default ethernet header is 14 bytes */
#define ETHER_VLAN_PT_LEN             ( 4 )  /**< VLAN Protocol Type length is 4 bytes */

#define ETHER_TYPE_OFFSET            ( 12 )  /**< Offset from the beginning of a frame to the ether type */

#define ARP_IP_SRC_OFFSET            ( 28 )  /**< Offset from the beginning of a frame to the ARP IP Source */
#define ARP_IP_DST_OFFSET            ( 38 )  /**< Offset from the beginning of a frame to the ARP IP Destination */

#define RED_FRAME_MIN_SIZE           ( 66 )  /**< The minimum RED frame size with no VLAN tag - no FCS! */
#define RED_FRAME_VLAN_MIN_SIZE      ( 70 )  /**< The minimum RED frame size with a VLAN tag - no FCS! */

#define RED_TLV2_TYPE                ( 30 )  /**< Indicates that the source node is a RedBox */

#define ICSS_EMAC_MAXMTU_HSR         1528

/** SUP tag */
typedef struct _SUP_TAG
{

    uint16_t ether_type;                /**< Ethertype for supervision frames, shall be 0x88FB */
    uint16_t path_and_ver;              /**< Protocol version, set to "1" (one) for this version of HSR */
    uint16_t seq_nr;                    /**< Sequence number of the supervision frame */

} SUP_TAG;

/** TLV1 tag */
typedef struct _TLV1_TAG
{

    uint8_t type;                       /**< Indicates the HSR node in normal operation or PRP node in Duplicate Discard or Accept */
    uint8_t mac_length;                 /**< Length of the following MAC address in octets */
    uint8_t dan_mac[ETHER_ADDR_LEN];    /**< MAC address used by the sender */

} TLV1_TAG;

/** TLV2 tag */
typedef struct _TLV2_TAG
{

    uint8_t type;                       /**< Source node identifier set to 30 if the source node is a RedBox */
    uint8_t mac_length;                 /**< Length of the following MAC address in octets */
    uint8_t red_mac[ETHER_ADDR_LEN];    /**< MAC addresses used by the RedBox on its interlink */

} TLV2_TAG;

/** TLV0 tag */
typedef struct _TLV0_TAG
{

    uint8_t type;                       /**< Closing TLV, set to 0 */
    uint8_t length;                     /**< Closing TLV, set to 0 */

} TLV0_TAG;

/** RED frame */
typedef struct _RED_FRAME
{

    uint8_t *pDataBuffer;
    uint16_t bufferLen;
    uint16_t vlanTagSize;
    uint16_t paddingSize;

} RED_FRAME;

/**
 *  \brief  LRE Row Status
 */
typedef enum
{

    LRE_ROW_STATUS_ACTIVE = 1,
    LRE_ROW_STATUS_NOT_IN_SERVICE,
    LRE_ROW_STATUS_NOT_READY,
    LRE_ROW_STATUS_CREATE_AND_GO,
    LRE_ROW_STATUS_CREATE_AND_WAIT,
    LRE_ROW_STATUS_DESTROY

} LreRowStatus_t;

/**
 *  \brief  LRE Node Type
 */
typedef enum
{

    LRE_NODE_TYPE_PRP_MODE_1 = 1,
    LRE_NODE_TYPE_HSR

} LreNodeType_t;

/**
 *  \brief  MAC Address
 */
typedef struct
{

    uint8_t octet[ETHER_ADDR_LEN];

} Mac_t;

/**
 *  \brief  RED Port
 */
typedef enum
{

    RED_PORT_A = ICSS_EMAC_PORT_1,
    RED_PORT_B = ICSS_EMAC_PORT_2

} RedPort_t;

/**
 *  \brief  LRE Port Admin State
 */
typedef enum
{

    LRE_PORT_NOT_ACTIVE = 1,
    LRE_PORT_ACTIVE     = 2

} LrePortAdminState_t;

/**
 *  \brief  LRE Link Status
 */
typedef enum
{

    LRE_LINK_UP   = 1,
    LRE_LINK_DOWN = 2

} LreLinkStatus_t;

/**
 *  \brief  LRE Duplicate Discard
 */
typedef enum
{

    LRE_DD_DO_NOT_DISCARD = 1,
    LRE_DD_DISCARD

} LreDuplicateDiscard_t;

/**
 *  \brief  LRE Transparent Reception
 */
typedef enum
{

    LRE_TR_REMOVE_RCT = 1,
    LRE_TR_PASS_RCT

} LreTransparentReception_t;

/**
 *  \brief  LRE HSR Mode
 */
typedef enum
{

    HSR_COMMON_MODE_H = 1,
    HSR_COMMON_MODE_N,
    HSR_COMMON_MODE_T,
    HSR_COMMON_MODE_U,
    HSR_COMMON_MODE_M

} HSRMode_t;

/**
 *  \brief  The switching end node functionality
 */
typedef enum
{

    SN_NON_BRIDGING_NODE = 1,
    SN_BRIDGING_UNSPECIFIED,
    SN_PRP_NODE,
    SN_HSR_REDBOX_SAN,
    SN_HSR_NODE,
    SN_HSR_REDBOX_HSR,
    SN_HSR_REDBOX_PRP_A,
    SN_HSR_REDBOX_PRP_B

} SwitchingEndNode_t;

/**
 *  \brief  LRE RedBox Identity
 */
typedef enum
{

    REDBOX_ID1A = 2,
    REDBOX_ID1B,
    REDBOX_ID2A,
    REDBOX_ID2B,
    REDBOX_ID3A,
    REDBOX_ID3B,
    REDBOX_ID4A,
    REDBOX_ID4B,
    REDBOX_ID5A,
    REDBOX_ID5B,
    REDBOX_ID6A,
    REDBOX_ID6B,
    REDBOX_ID7A,
    REDBOX_ID7B

} LreRedBoxIdentity_t;

/**
 *  \brief  LRE Truth Value
 */
typedef enum
{

    LRE_FALSE = 0,
    LRE_TRUE  = 1

} LreTruthValue_t;

/**
 *  \brief  LRE Table Operation
 */
typedef enum
{

    LRE_TABLE_NO_OP = 0,
    LRE_TABLE_CLEAR

} LreTableOperation_t;

/**
 *  \brief  LRE Interface Statistics Entry
 */
typedef enum
{
    LRE_IF_STATS_CNT_TX_A = 0,
    LRE_IF_STATS_CNT_TX_B,
    LRE_IF_STATS_CNT_TX_C,
    LRE_IF_STATS_CNT_ERR_WRONG_LAN_A,
    LRE_IF_STATS_CNT_ERR_WRONG_LAN_B,
    LRE_IF_STATS_CNT_ERR_WRONG_LAN_C,
    LRE_IF_STATS_CNT_RX_A,
    LRE_IF_STATS_CNT_RX_B,
    LRE_IF_STATS_CNT_RX_C,
    LRE_IF_STATS_CNT_ERRORS_A,
    LRE_IF_STATS_CNT_ERRORS_B,
    LRE_IF_STATS_CNT_ERRORS_C,
    LRE_IF_STATS_CNT_NODES,
    LRE_IF_STATS_CNT_PROXY_NODES,
    LRE_IF_STATS_CNT_UNIQUE_A,
    LRE_IF_STATS_CNT_UNIQUE_B,
    LRE_IF_STATS_CNT_UNIQUE_C,
    LRE_IF_STATS_CNT_DUPLICATE_A,
    LRE_IF_STATS_CNT_DUPLICATE_B,
    LRE_IF_STATS_CNT_DUPLICATE_C,
    LRE_IF_STATS_CNT_MULTI_A,
    LRE_IF_STATS_CNT_MULTI_B,
    LRE_IF_STATS_CNT_MULTI_C,
    LRE_IF_STATS_CNT_OWN_RX_A,
    LRE_IF_STATS_CNT_OWN_RX_B


} LreIfStat_t;

/**
 *  \brief  Remote Node Type
 */
typedef enum
{

    RED_REM_NODE_TYPE_DANP = 0,
    RED_REM_NODE_TYPE_REDBOXP,
    RED_REM_NODE_TYPE_VDANP,
    RED_REM_NODE_TYPE_DANH,
    RED_REM_NODE_TYPE_REDBOXH,
    RED_REM_NODE_TYPE_VDANH,
    RED_REM_NODE_TYPE_SAN,
    RED_REM_NODE_TYPE_UNKNOWN

} RemNodeType_t;

/**
 *  \brief  LRE Interface Statistics
 */
typedef struct _RED_STATISTICS
{

    /**< Number of frames */
    uint32_t cntTxA;               /**< - sent over port A that are HSR tagged */
    uint32_t cntTxB;               /**< - sent over port B that are HSR tagged */
    uint32_t cntTxC;               /**< - sent towards the application interface of the DAN */

    /**< Number of frames with the wrong LAN identifier */
    uint32_t cntErrWrongLanA;      /**< - received on the LRE port A */
    uint32_t cntErrWrongLanB;      /**< - received on the LRE port B */
    uint32_t cntErrWrongLanC;      /**< - received on the interlink */

    /**< Number of frames */
    uint32_t cntRxA;               /**< - received on the LRE port A */
    uint32_t cntRxB;               /**< - received on the LRE port B */
    uint32_t cntRxC;               /**< - received from the application interface of the DAN */

    /**< Number of frames with errors */
    uint32_t cntErrorsA;           /**< - received on the LRE port A */
    uint32_t cntErrorsB;           /**< - received on the LRE port B */
    uint32_t cntErrorsC;           /**< - received on the application interface of the DAN */

    uint32_t cntNodes;             /**< Number of entries in the node table */
    uint32_t cntProxyNodes;        /**< Number of entries in the proxy table */

    /**< Number of entries in the duplicate detection mechanism */
    uint32_t cntUniqueA;           /**< - on port A for which no duplicate was received */
    uint32_t cntUniqueB;           /**< - on port B for which no duplicate was received */
    uint32_t cntUniqueC;           /**< - on the application interface of the DAN for which no duplicate was received */

    /**< Number of entries in the duplicate detection mechanism */
    uint32_t cntDuplicateA;        /**< - on port A for which one single duplicate was received */
    uint32_t cntDuplicateB;        /**< - on port B for which one single duplicate was received */
    uint32_t cntDuplicateC;        /**< - on the application interface of the DAN for which one single duplicate was received */

    /**< Number of entries in the duplicate detection mechanism */
    uint32_t cntMultiA;            /**< - on port A for which more than one duplicate was received */
    uint32_t cntMultiB;            /**< - on port B for which more than one duplicate was received */
    uint32_t cntMultiC;            /**< - on the application interface of the DAN for which more than one duplicate was received */

    /**< Number of HSR tagged frames */
    uint32_t cntOwnRxA;            /**< - received on port A that originated from this device */
    uint32_t cntOwnRxB;            /**< - received on port B that originated from this device */

    uint32_t hostCntRx;            /**Numeber of frame retreive by the host */

    /* debug values */

    uint32_t tx_bc_frames;
    uint32_t TX_MC_FRAMES;
    uint32_t TX_UC_FRAMES;

    uint32_t TX_BYTE_CNT;

    uint32_t RX_BC_FRAMES;
    uint32_t RX_MC_FRAMES;
    uint32_t RX_UC_FRAMES;

    uint32_t TXB_BC_FRAMES;
    uint32_t TXB_MC_FRAMES;
    uint32_t TXB_UC_FRAMES;

    uint32_t TXB_BYTE_CNT;

    uint32_t RXB_BC_FRAMES;
    uint32_t RXB_MC_FRAMES;
    uint32_t RXB_UC_FRAMES;

    uint32_t NODE_TABLE_INSERTION_ERROR;
    uint32_t RXA_OVERFLOW;
    uint32_t RXB_OVERFLOW;
    uint32_t RXA_FWD_OVERFLOW;
    uint32_t RXB_FWD_OVERFLOW;
    uint32_t RXA_FAILACQU_QUEUE;
    uint32_t RXB_FAILACQU_QUEUE;
    uint32_t RXA_FWD_FAILACQU_QUEUE ;
    uint32_t RXB_FWD_FAILACQU_QUEUE;

    uint32_t DEBUG_1 ;
    uint32_t DEBUG_2;

    uint32_t DEBUG_3 ;
    uint32_t DEBUG_4;

} RED_STATISTICS;

/**
 *  \brief  Index array Entry
 */
typedef struct _RED_INDEX_ARRAY_ENTRY
{

    uint16_t bin_offset
    ;       /* offset of the bin in BIN ARRAY as pointed by hash */
    uint16_t binNoEntries
    ;     /* no of entries in the bin | increments or decrements by 1 */
    uint8_t   bitLinBin ;         /* 0x0 : Linear search | 0x1 : Binary search */

} RED_INDEX_ARRAY_ENTRY;        /* 2 + 2 + 1 = 5 bytes */

/**
 *  \brief  Bin array Entry
 */
typedef struct _RED_BIN_ARRAY_ENTRY
{

    uint8_t  MacId[ETHER_ADDR_LEN];   /* MAC ID of the source */
    uint16_t nodetable_offset;      /* offset of the nodetable entry in NODETABLE corresponding to the attached MAC ID */

} RED_BIN_ARRAY_ENTRY;              /*2 + 6 = 8 bytes */

/**
 *  \brief  Node Table Entry
 */

typedef struct _RED_NODE_TABLE_ENTRY
{

    uint8_t  src[ETHER_ADDR_LEN];  /**< Source Address (MAC address of the node) */
    uint8_t  state;                /**< Valid (1) / Invalid (0) */
    uint8_t  status;               /**< Duplicate Discard [8:7] and Node Type [5:0] */
    uint32_t cntRxA;               /**< Frames received on port A */
    uint32_t cntRxB;               /**< Frames received on port B */
    uint32_t errRxA;               /**< Erroneous frames received on port A */
    uint32_t errRxB;               /**< Erroneous frames received on port B */
    uint8_t  cntRxSupA;            /**< Number of supervision frames received on port A */
    uint8_t  cntRxSupB;            /**< Number of supervision frames received on port B */
    uint16_t timeLasSeenS;         /**< Time last seen of a supervision frame */
    uint16_t timeLasSeenA;         /**< Time last seen of a frame received on port A */
    uint16_t timeLasSeenB;         /**< Time last seen of a frame received on port B */

} RED_NODE_TABLE_ENTRY;          /*6 + 1 + 1 + 4 + 4 + 4 + 4 + 1 + 1 + 2 + 2 + 2 = 32 bytes*/

/**
 *  \brief  Node Table
 */
typedef struct _RED_NODE_TABLE
{

    RED_NODE_TABLE_ENTRY *entries;  /**< Node entries array */
    uint32_t                cnt;      /**< Number of entries in the array */
    uint32_t                max;      /**< Maximal number of entries */

} RED_NODE_TABLE;

/**
 *  \brief  RED Status
 */
typedef enum
{

    RED_OK = 0,
    RED_ERR

} RED_STATUS;

#ifdef RED_DEBUG
#define RED_DEBUG_MSG(...) printf(__VA_ARGS__)
#else /* RED_DEBUG */
#define RED_DEBUG_MSG(...)
#endif /* RED_DEBUG */


#ifdef __cplusplus
}
#endif

#endif /* RED_COMMON_H_ */
