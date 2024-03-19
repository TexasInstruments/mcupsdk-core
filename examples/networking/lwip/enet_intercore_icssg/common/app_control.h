/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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
 * \file  app_control.h
 *
 * \brief This file contains the implementation of the Enet Remote Connect control path for remote core.
 */
#ifndef _APP_CONTROL_H_
#define _APP_CONTROL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

enum icve_msg_type
{
    ICVE_REQUEST_MSG = 0,
    ICVE_RESPONSE_MSG,
    ICVE_NOTIFY_MSG,
};

enum icve_rpmsg_type
{
    /* Request types */
    ICVE_REQ_SHM_INFO = 0,
    ICVE_REQ_SET_MAC_ADDR,
    /* Req for Multicast handling*/
    ICVE_REQ_ADD_MC_ADDR,
    ICVE_REQ_DEL_MC_ADDR,

    /* Response types */
    ICVE_RESP_SHM_INFO,
    ICVE_RESP_SET_MAC_ADDR,
    /* Resp for Multicast handling*/
    ICVE_RESP_ADD_MC_ADDR,
    ICVE_RESP_DEL_MC_ADDR,

    /* Notification types */
    ICVE_NOTIFY_PORT_UP,
    ICVE_NOTIFY_PORT_DOWN,
    ICVE_NOTIFY_PORT_READY,
    ICVE_NOTIFY_REMOTE_READY,
    ICVE_NOTIFY_REMOTE_IP,
};

enum icve_status
{
    ICVE_OK = 0,
    ICVE_FAIL = -1,
    ICVE_BADARGS = -2,
    ICVE_INVALID_PARAMS = -3,
    ICVE_TIMEOUT = -4,
    ICVE_UNSUPPORTED_CMD = -5,
};

#define MAX_IP_LEN (20U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct Icve_macAddr_s
{
    uint8_t macAddr[6U];
} Icve_macAddr;

typedef struct Icve_ShrMemQInfo_s
{
    /* Total shared memory size */
        uint32_t total_shm_size;
    /* Total number of buffers */
        uint32_t num_pkt_bufs;
    /* Per buff slot size i.e MTU Size + 4 bytes for magic number + 4 bytes
     * for Pkt len */
        uint32_t buff_slot_size;
    /* Base Address for Tx or Rx shared memory */
        uint32_t base_addr;
} Icve_ShrMemQInfo;

typedef struct Icve_ShrMemInfo_s
{
    /* Total shared memory size */
    Icve_ShrMemQInfo tx_shm_info;
    /* Total number of buffers */
    Icve_ShrMemQInfo rx_shm_info;
} Icve_ShrMemInfo;

typedef struct Icve_reqMsg_s
{
    uint32_t type; /* Request Type */
    uint32_t id;   /* Request ID */
    union {
        Icve_macAddr mac_addr;
    };
} Icve_reqMsg;

typedef struct Icve_respMsg_s
{
    uint32_t type;   /* Response Type */
    uint32_t id;     /* Response ID */
    union {
        Icve_ShrMemInfo shm_info;
    };
} Icve_respMsg;

typedef struct Icve_notifyMsg_s
{
    uint32_t type;   /* Notify Type */
    uint32_t id;     /* Notify ID */
    union {
        ip_addr_t ipAddr;
    };
} Icve_notifyMsg;

typedef struct Icve_msgHeader_s
{
    uint32_t src_id;
    uint32_t msg_type; /* Do not use enum type, as enum size is compiler dependent */
} Icve_msgHeader;

typedef struct Icve_message_s
{
    Icve_msgHeader msg_hdr;
    union {
        Icve_reqMsg req_msg;
        Icve_respMsg resp_msg;
        Icve_notifyMsg notify_msg;
    };
} Icve_message;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void AppCtrl_createSendTask();

void AppCtrl_createRecvTask();

void AppCtrl_sendAddMacAddrReq(Icve_macAddr args, uint32_t type);

void AppCtrl_sendIPNotify();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void App_handleControlMsg(void* msgBuf);

#endif /* _APP_CONTROL_H_ */
