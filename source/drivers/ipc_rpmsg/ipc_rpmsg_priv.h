/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#ifndef IPC_RPMSG_PRIV_H_
#define IPC_RPMSG_PRIV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/ipc_rpmsg.h>
#include <drivers/ipc_notify.h>
#include <drivers/ipc_rpmsg/ipc_rpmsg_queue.h>

/* max number of recevied messages that can be outstanding at various end points
 * This is a per remote core limit
 * Ideally this needs to be equal to number of buffers in vring since that
 * is the theortical max limit on outstanding recv messages.
 */
#define RPMESSAGE_MAX_LOCAL_MSG_OBJ     (16U)

/*
 * End Point used to communicate control messages.
 * Primarily used to communicate available end point on a core ID
 */
#define RPMESSAGE_CTRL_ENDPOINT_ID      (53U)

/* Message via IPC Notify to indicate is new message is received */
#define RPMESSAGE_MSG_VRING_NEW_FULL    (0U)
/* Message via IPC Notify to indicate if free buffers are available for transmit */
#define RPMESSAGE_MSG_VRING_NEW_EMPTY   (1U)

/* This is fixed in Linux side and MUST match LInux side */
#define RPMESSAGE_LINUX_MSG_SIZE        (512U)

/* VRING ID for TX as used with Linux */
#define RPMESSAGE_LINUX_TX_VRING_ID     (0U)

/* VRING ID for RX as used with Linux */
#define RPMESSAGE_LINUX_RX_VRING_ID     (1U)

/* VRING structure in shared memory is as below for a VRING
   with N buffers and max msg size of X

   mesage desc [0]          (16 bytes each)
   mesage desc [1]
   ...
   mesage desc [N-1]
   avail Q flags            (2 bytes)
   avail Q current index    (2 bytes)
   avail Q element [0]      (2 bytes each)
   avail Q element [1]
   ...
   avail Q element [N-1]
   alignment padding, if any, needs 4 byte alignment atleast
   used Q flags              (2 bytes)
   used Q current index      (2 bytes)
   used Q element [0] ID     (4 bytes each)
   used Q element [0] Length (4 bytes each)
   used Q element [1] ID
   used Q element [1] Length
   ...
   used Q element [N-1] ID
   used Q element [N-1] Length
   alignment padding, if any, needs 4 byte alignment atleast
   message buffer [0]       (X bytes each)
   message buffer [1]
   ...
   message buffer [N-1]

   Total size of a VRING is
     16 * N
   + align(4 + 2 * N) to 4 bytes
   + align(4 + 8 * N) to 4 bytes
   + X * N

   When there are K CPUs participating in IPC RPMessage,
   there are total ( K * (K-1) ) VRINGs to allow TX and RX of messages
   from any CPU to any CPU

   It is recommended to keep X as a power of 2, like 64, 128, 256
   It is recommended to keep N as multiple of 2, like 2, 4, 6, 8

   In memory constrained systems, recommended values are
   X = 128, N = 8 and alignment of 4 bytes

   In this case total memory needed for VRING size when 4 CPUs are involved is,

   ( 16 * N + 4 + 2 * N + 4 + 8 * N + N * X   ) * (K * (K-1))
   ( 16 * 8 + 4 + 2 * 8 + 4 + 8 * 8 + 8 * 128 ) * (4 * (4-1))
   = 14880 bytes ~ 14.6 KB

   A simpler formula which will always be valid is below
   number of buffers * (size of buffer + 32) + number of CPUs * ( number of CPUs - 1 )

   For above example, this is
   ( 8 * ( 128 + 32 ) * 4 * (4-1) )
   = 15360 bytes = 15 KB
*/

/* buffer descriptor within vring
 * descriptor value are set once during vring init and dont change after that
 */
struct vring_desc
{
    uint32_t addr;     /* Physical address of message buffer */
    uint32_t padding;  /* NOT USED */
    uint32_t len;      /* max length of message buffer */
    uint16_t flags;    /* NOT USED */
    uint16_t next;     /* NOT USED */
};

/* avail Q */
struct vring_avail
{
    uint16_t flags;   /* NOT USED */
    uint16_t idx;     /* next write index within vring_avail.ring */
    uint16_t ring[1]; /* buffer ID, valid values are 0 .. number of buffers - 1 */
};

/* used Q element */
struct vring_used_elem
{
    uint32_t id;    /* buffer ID, valid values are 0 .. number of buffers - 1 */
    uint32_t len;   /* NOT USED */
};

/* used Q  */
struct vring_used
{
    uint16_t flags; /* NOT USED */
    uint16_t idx;   /* next write index within vring_used.ring */
    struct vring_used_elem ring[1]; /* used Q elements */
};

/* header that is present in every message buffer, this space is not available for
 * users, that max message size from user point of view is
 * message buffer size - sizeof(RPMessage_Header)
 *
 * e.g, if message buffer size is 128 bytes, max message size for user is
 *      128 - 16 = 112 bytes
 */
typedef struct
{
    uint32_t  srcEndPt;  /* source/sender endpoint        */
    uint32_t  dstEndPt;  /* destination/receiver endpoint */
    uint32_t  srcCoreId; /* NOT USED                      */
    uint16_t  dataLen;   /* length of valid user data in the message buffer */
    uint16_t  flags;     /* NOT USED                      */
} RPMessage_Header;

/* local structure to maintain state of a given VRING,
 * holds pointers to shared memory VRING data structures
 */
typedef struct
{
    uint16_t lastUsedIdx;            /* last read index into used Q */
    uint16_t lastAvailIdx;           /* last read index into avail Q */
    uint16_t vringNumBuf;            /* number of buffer in the vring */
    struct vring_desc  *desc;        /* pointer to buffer descriptors in VRING shared memory */
    struct vring_avail *avail;       /* pointer to avail Q in VRING shared memory */
    struct vring_used  *used;        /* pointer to used Q in VRING shared memory */
    uint8_t            *bufBaseAddr; /* pointer to message buffer 0 in VRING shared memory */
} RPMessage_Vring;

/* structure to hold received buffer ID and sender core ID
 *
 * An instance of this structure is put into the end point specific queue.
 * On calling rpmsg recv, an element from end point queue is extracted,
 * the vring buffer processed and the vring buffer is freed.
 *
 * This prevents a copy from vring to local end point queue
 * and also reduces the memory needed for local queing
 */
typedef struct {
    struct RPMessage_QueueElem_s elem;  /* queue element header */
    uint16_t remoteCoreId;  /* remote core that sent a message */
    uint16_t vringBufId;    /* buffer ID within VRING which holds the message */
} RPMessage_LocalMsg;

/* structure to hold state of IPC rpmsg with a remote core
 */
typedef struct
{
    SemaphoreP_Object newEmptyVringBufSem;  /* semaphore to indicate if a empty buffer
                                             *  is available to transmit
                                             *
                                             * ONLY used during transmit
                                             */
    uint32_t freeQAllocPending; /* flag to indicate if a freeQ alloc failed
                                 * and therefore there is a pending recvied message in VRING
                                 * if this flag is set then recv message handler is invoked without
                                 * waiting for a new interrupt
                                 *
                                 * ONLY used during receive.
                                 */
    RPMessage_Queue freeQ;      /* queue of RPMessage_LocalMsg to hold received messages until
                                 * they are processed by users
                                 *
                                 * ONLY used during receive.
                                 */
    RPMessage_LocalMsg  localMsgObj[RPMESSAGE_MAX_LOCAL_MSG_OBJ]; /* RPMessage_LocalMsg messages are put
                                                                   * in the freeQ initially
                                                                   *
                                                                   * ONLY used during receive.
                                                                   */
    RPMessage_Vring vringTxObj; /* VRING used to transmit messages to this remote core */
    RPMessage_Vring vringRxObj; /* VRING used to receive messages from this remote core */
} RPMessage_Core;

/* structure to hold state of RPMessage end point */
typedef struct
{
    uint16_t localEndPt;    /* local end point number, MUST be < RPMESSAGE_MAX_LOCAL_ENDPT */
    RPMessage_RecvCallback recvCallback;    /* when not NULL, received messages are handled in callback that via RPMessage_recv */
    void *recvCallbackArgs;     /* arguments passed to the recvCallback callback */
    uint32_t doRecvUnblock;     /* flag to unblock RPMessage_recv, if its blocked for every waiting for messages and user wants to shutdown or exit */
    RPMessage_Queue endPtQ;     /* end point specific queue to hold received messages pending for processing at this end point */
    SemaphoreP_Object newEndPtMsgSem; /* semaphore to indicate that there messages pending endPtQ */
    RPMessage_RecvNotifyCallback recvNotifyCallback;    /* when not NULL, this callback is whenever a message is received */
    void *recvNotifyCallbackArgs;     /* arguments passed to the recvNotifyCallback callback */
} RPMessage_Struct;

/* message that is sent during annouce and received by a control end point */
#define RPMESSAGE_ANNOUNCE_SERVICENAME_LEN  (32u)
typedef struct
{
    char      name[RPMESSAGE_ANNOUNCE_SERVICENAME_LEN]; /* service name that has annouced */
    uint32_t  remoteEndPt; /* end point at which the service is read */
    uint32_t  type; /* NOT USED */
} RPMessage_AnnounceMsg;

/* structure to hold over RP Message state including end point and vring states */
typedef struct
{
    uint16_t selfCoreId;    /* self core ID */
    uint8_t isCoreEnable[CSL_CORE_ID_MAX]; /* 1: core is enabled for IPC RPMessage, else disabled */
    uint8_t isCoreInitialized[CSL_CORE_ID_MAX]; /* 1: core is initialized for IPC RPMessage, else not yet initialized */
    RPMessage_Core  coreObj[CSL_CORE_ID_MAX];   /* remote core objects, indexed by remote core ID */
    RPMessage_Struct *localEndPtObj[RPMESSAGE_MAX_LOCAL_ENDPT]; /* end point objects, indexed by endpoint ID */
    RPMessage_Object controlEndPtObj; /* object/handle of end point that receives accouncement messages */
    RPMessage_ControlEndPtCallback controlEndPtCallback; /* user callback to invoke when a control message is received */
    void  *controlEndPtCallbackArgs; /* user callback args for control message */
    const RPMessage_ResourceTable *linuxResourceTable; /* resource table used with linux */
    uint16_t linuxCoreId; /* Core ID of core running linux */
} IpcRpmsg_Ctrl;

/* global varaible that holds the state of this module, this is the only global within this module */
extern IpcRpmsg_Ctrl gIpcRpmsgCtrl;

/* utility function to align a value, `align` MUST be power of 2 */
static inline uint32_t RPMessage_align(uint32_t value, uint32_t align)
{
    return (value + align - 1) & ~(align-1);
}

/* utility function to find if core ID runs linux */
uint32_t RPMessage_isLinuxCore(uint16_t coreId);

/* functions for VRING TX handling and initialization */
void     RPMessage_vringCheckEmptyTxBuf(uint16_t remoteCoreId);
int32_t  RPMessage_vringGetEmptyTxBuf(uint16_t remoteCoreId, uint16_t *vringBufId, uint32_t timeout);
uint8_t *RPMessage_vringGetTxBufAddr(uint16_t remoteCoreId, uint16_t vringBufId);
uint32_t RPMessage_vringGetTxBufLen(uint16_t remoteCoreId, uint16_t vringBufId);
void     RPMessage_vringPutFullTxBuf(uint16_t remoteCoreId, uint16_t vringBufId, uint16_t dataLen);
/* functions for VRING RX handling and initialization */
uint32_t RPMessage_vringIsFullRxBuf(uint16_t remoteCoreId);
int32_t  RPMessage_vringGetFullRxBuf(uint16_t remoteCoreId, uint16_t *vringBufId);
uint8_t *RPMessage_vringGetRxBufAddr(uint16_t remoteCoreId, uint16_t vringBufId);
void     RPMessage_vringPutEmptyRxBuf(uint16_t remoteCoreId, uint16_t vringBufId);
/* functions for VRING initialization and other utility functions */
uint32_t RPMessage_vringGetSize(uint16_t numBuf, uint16_t msgSize, uint32_t align);
void     RPMessage_vringReset(uint16_t remoteCoreId, uint16_t isTx, const RPMessage_Params *params);
void     RPMessage_vringResetLinux(uint16_t remoteCoreId, uint16_t isTx, const RPMessage_ResourceTable *rscTable);

#ifdef __cplusplus
}
#endif

#endif /* IPC_RPMSG_PRIV_H_ */
