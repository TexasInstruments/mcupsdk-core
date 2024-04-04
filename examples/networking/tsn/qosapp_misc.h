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

#ifndef __QOSAPP_MISC_H__
#define __QOSAPP_MISC_H__

#define DEFAULT_VLAN_ID     (110)
#define AVTP_AAF_SUBTYPE    (2)
/* Default interface configured for the application, please change it
 * if you want to test it for the port other than mac port 1 (tilld0)
 * 0: index of the interface tilld0
*/
#define DEFAULT_INTERFACE_INDEX (0)

#define MAX_KEY_SIZE            (256)
#define MAX_LOG_LEN             (256)
#define MAX_VAL_SIZE            (64)

#define QOSAPP_MAX_STREAMS      (8)
#define QOSAPP_NUM_OF_STREAMS   (7)
#define QOSAPP_TASK_PRIORITY    (1)
#define QOSAPP_PRIORITY_MAX     (8)

#define ENETAPP_VLAN_TPID       (0x8100U)
#define ENETAPP_VLAN_PCP_OFFSET (13U)
#define ENETAPP_VLAN_PCP_MASK   (0x7U)
#define ENETAPP_VLAN_DEI_OFFSET (12U)
#define ENETAPP_VLAN_DEI_MASK   (0x1U)
#define ENETAPP_VLAN_VID_MASK   (0xFFFU)
#define ENETAPP_VLAN_TCI(pcp, dei, vid)  ( (((pcp) & ENETAPP_VLAN_PCP_MASK) << ENETAPP_VLAN_PCP_OFFSET)| \
                                           (((dei) & ENETAPP_VLAN_DEI_MASK)<< ENETAPP_VLAN_DEI_OFFSET) | \
                                           ((vid) & ENETAPP_VLAN_VID_MASK) )

#define TRAFFIC_CLASS_NODE      "/ietf-interfaces/interfaces/interface|name:%s|" \
                                "/bridge-port/traffic-class"
#define TRAFFIC_CLASS_TABLE_NODE TRAFFIC_CLASS_NODE"/traffic-class-table"
#define TRAFFIC_CLASS_DATA_NODE  TRAFFIC_CLASS_NODE"/tc-data"
#define PHYSICAL_QUEUE_MAP_NODE  TRAFFIC_CLASS_NODE"/pqueue-map"

#define YANGDB_RUNTIME_WRITE(key,val) do {                              \
        err = yang_db_runtime_put_oneline(ydrd, key, val, YANG_DB_ONHW_NOACTION); \
        DebugP_assert(err == 0);                                        \
    } while (0)

UB_ABIT32_FIELD(cmsh_sv, 23, 0x1) // cmsh_sv_bit_field, cmsh_sv_set_bit_field

typedef struct QoSAppCommonParam
{
    /*! Name of network interface */
    char *netdev;
    /*! index is priority, value of each index is TC, -1: not used. */
    int8_t priority2TcMapping[QOSAPP_PRIORITY_MAX];
    uint8_t nTCs;                 /*! Num of traffic classes */
    uint8_t nQueues;              /*! Num of HW queue */
} QoSAppCommonParam_t;

typedef struct BitrateCtrl
{
    uint32_t maxCapacity; /*!  Maximum bytes of the bucket */
    uint64_t bitRate;     /*!  bit per second. */
    uint32_t tokens;      /*!  Number of available tokens in bytes */
    uint64_t lastTs;      /*!  Timestamp of previous sent packet */
} BitrateCtrl_t;

struct AvtpCommonStreamParam
{
    uint8_t subtype; /*! Subtype of AVTP header */
    uint8_t bf0;
    uint8_t seqn;    /*! Sequence number of AVTP header */
    uint8_t bf1;
} __attribute__ ((packed));

typedef struct AVTPCommonStreamHdr
{
    union
    {
        struct AvtpCommonStreamParam hh;
        uint32_t bf;
    };
    ub_streamid_t streamId;   /*! Stream ID  */
    uint32_t headerTimestamp; /*! Timestamp of AVTP packet */
    uint32_t fsd2;
    uint16_t pdLength;        /*! packet data length */
    uint16_t fsd3;
} __attribute__ ((packed)) AVTPCommonStreamHdr_t;

typedef struct StreamParam
{
    ub_streamid_t sid; /*!  8 bytes stream id */
    /*! priority of traffic has on-to-one mapping with PCP(3 bits) */
    uint8_t priority;
    /*! traffic class of the stream */
    uint8_t tc;
    /*!  Sequence number of AVTP packet */
    uint8_t seqn;
    /*! Seceived timestamp of the received packet */
    uint64_t rxts;
    /*! Present timestamp in avtp header which
     * is captured when sending by application.
     */
    int64_t txts;
    /*! numof bytes received */
    uint64_t rxBytes;
    /*! The previous time it shows the bitrate */
    uint64_t prevTs;
    /*! bitRate of a stream. */
    uint64_t bitRate;
    /*!  Length of payload data in the Ethernet
     * packet for each stream.
     */
    uint32_t payloadLen;
    /*!  Num of broken packets. */
    uint32_t nBrokenPkt;
} StreamParam_t;

typedef struct EnetQoSApp_TaskCfg
{
    /*! A string which indicate task name */
    char *name;
    /*! Priority of task */
    int priority;
    /*! Start address of a buffer allocaing for task stack  */
    void *stackBuffer;
    /*! Size of buffer allocated for the stack */
    int stackBufferSize;
} EnetQoSApp_TaskCfg_t;

typedef struct EnetQoSApp_TaskCtx
{
    /*!  Handle of the task.*/
    CB_THREAD_T hTaskHandle;
    /*! true: Enable task running, false: disable task running. */
    bool enable;
    /*! Num of traffic classes */
    uint8_t nTCs;
    /*!  VLAN ID of the stream. */
    uint16_t vid;
    /*!  Num of streams for talker or listener. */
    uint16_t nStreams;
    /*! Buffer for sending or receiving ethernet packet */
    uint8_t buffer[sizeof(EthVlanFrameHeader)+ETH_PAYLOAD_LEN];
    /*! Info associated for each stream of this task */
    StreamParam_t streams[QOSAPP_MAX_STREAMS];
    /*! Handle of an object to control bitrate for the talker */
    BitrateCtrl_t bitrateCtrl[QOSAPP_MAX_STREAMS];
    /*! A semaphore to signal receiver thread for availability of packets */
    CB_SEM_T rxPacketSem;
    /*!  A semaphore to signal the parent thread when
     * child thread is terminated.
     */
    CB_SEM_T terminatedSem;
} EnetQoSApp_TaskCtx_t;

typedef struct EnetQoSApp_Packet
{
    /*! Start address of buffer for containing an ethernet packet */
    void *buffer;
    /*! Size of buffer */
    int bufferSize;
    /*! Meta info of the buffer */
    CB_SOCKADDR_LL_T recvAddr;
} EnetQoSApp_Packet_t;

/*! Callback for notifying packet receiption to higher layer application */
typedef void (*PacketHandlerCb)(void *ctx, EnetQoSApp_Packet_t *pkt);

typedef struct EnetQoSApp_AppCtx
{
    /*! Socket handle for entire application */
    CB_SOCKET_T est_sock;
    /*! Source address associated with the socket above. */
    ub_macaddr_t source_mac;
    /*! Address associated with the socket above. */
    CB_SOCKADDR_LL_T sockAddress;
    /*! A handle of talker of the app */
    EnetQoSApp_TaskCtx_t talker;
    /*! A handle of listener of the app */
    EnetQoSApp_TaskCtx_t listener;
    /*! An active network interface used for this app */
    char *netdev[MAX_NUMBER_ENET_DEVS];
    /*! How many network interfaces */
    int32_t netdevSize;
    /*! A delay offset for applying a schedule in microsecond unit */
    int64_t adminDelayOffset;
    /*! Default interface index of the network interface for the application */
    int32_t ifidx;
    /*! A pointer to a context object. */
    void *ectx;
    /*! Callback to notify application on packet receiption */
    PacketHandlerCb packetHandlerCb;
} EnetQoSApp_AppCtx_t;

typedef struct StreamConfigParam
{
    /*! Bitrate of the stream in Kbps */
    uint32_t bitRateKbps;
    /*! Length of payload */
    uint32_t payloadLen;
    /*! Traffic class ID */
    uint8_t tc;
    /*! Priority of traffic (PCP) */
    uint8_t priority;
} StreamConfigParam_t;

typedef struct QoSAppStreamConfigParam
{
    /*! Config parameters of the streams to be run */
    StreamConfigParam_t streamParams[QOSAPP_MAX_STREAMS];
    /*! How many streams requested by user */
    int nStreams;
} QoSAppStreamConfigParam_t;

/*!
 * \brief Initialize a QoS application.
 *
 * \param ctx    [OUT] a context object to be initialized
 * \param modCtx [IN]  specify config parameters on network interfaces.
 * \param cb     [IN]  callback to be called when packet received.
 *
 * \return 0: On Sucess; -1: on Failure
 */
int  EnetQoSApp_initialize(EnetQoSApp_AppCtx_t *ctx,
                           EnetApp_ModuleCtx_t *modCtx,
                           PacketHandlerCb cb);

/*!
 * \brief Deinitialize a QoS application.
 *
 * \param ctx [IN]  A context object to be deinitialized
 *
 * \return 0: On Sucess; -1: on Failure
 */
int  EnetQoSApp_deinitialize(EnetQoSApp_AppCtx_t *ctx);

/*!
 * \brief Open a Yang DB for reading or writing.
 *
 * \param dbarg [OUT]  Keep necessary handles after openning a DB.
 * \param dbName[IN] Name of the DB. Could be null to use default DB.
 * \param mode  [IN] r: reading ; w: writing mode
 *
 * \return 0: On Sucess; -1: on Failure
 */
int  EnetQoSApp_openDB(EnetApp_dbArgs *dbarg,
                       char *dbName,
                       const char *mode);

/*!
 * \brief Close a Yang DB after use.
 *
 * \param dbarg [IN]  Hold necessary handles for closing a DB.
 */
void EnetQoSApp_closeDB(EnetApp_dbArgs *dbarg);

/*!
 * \brief Start a talker.
 *  Talker will be run in a saperate task.

 * \param ctx   [IN] Point to context object of the application
 * \param cfg   [IN] Config parameter of task which run talker.
 * \param stParams  [IN] Config parameters for all streams.
 */
void EnetQoSApp_startTalker(EnetQoSApp_AppCtx_t *ctx,
                            EnetQoSApp_TaskCfg_t *cfg,
                            QoSAppStreamConfigParam_t *stParams);

/*!
 * \brief Stop a talker.

 * \param ctx   [IN] Point to context object of the application
 */
void EnetQoSApp_stopTalker(EnetQoSApp_AppCtx_t *ctx);

/*!
 * \brief Start a listener.
 *  Listener will be run in a saperate task.

 * \param ctx   [IN] Point to context object of the application
 * \param cfg   [IN] Config parameter of task which run talker.
 */
void EnetQoSApp_startListener(EnetQoSApp_AppCtx_t *ctx,
                              EnetQoSApp_TaskCfg_t *cfg);

/*!
 * \brief Stop a listener.

 * \param ctx   [IN] Point to context object of the application
 */
void EnetQoSApp_stopListener(EnetQoSApp_AppCtx_t *ctx);

/*!
 * \brief print stats info of host and all mac ports.

 * \param ctx   [IN] Point to context object of the application
 */
void EnetQoSApp_printStats(EnetQoSApp_AppCtx_t *ctx);


/*!
 * \brief Reset stats info of host and all mac ports.

 * \param ctx   [IN] Point to context object of the application
 */
void EnetQoSApp_resetStats(EnetQoSApp_AppCtx_t *ctx);

/*!
 * \brief Convert network interface name to portIndex.

 * \param ctx   [IN] Point to context object of the application
 * \param netdev[IN] Network interface name.
 *
 * \return -1: on error: Other than -1: port index of the interface
 */
int8_t EnetQoSApp_getPortIdx(EnetQoSApp_AppCtx_t *ctx, char *netdev);

/*!
 * \brief Set common parameters for all streams in the yang DB.

 * \param dbarg [IN] necessary handles to access a DB.
 * \param ctx   [IN] Point to context object of the application
 *
 * \return 0: On Sucess; -1: on Failure
 */
int EnetQoSApp_setCommonParam(QoSAppCommonParam_t *prm,
                              EnetApp_dbArgs *dbarg);

/*!
 * \brief Get current time.

* \return 0: On Failure; != 0: Current time in microsecond unit.
 */
uint64_t EnetQoSApp_getCurrentTimeUs(void);

#endif
