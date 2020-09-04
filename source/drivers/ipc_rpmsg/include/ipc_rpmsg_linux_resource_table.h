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

#ifndef IPC_RPMSG_LINUX_RESOURCE_TABLE_H_
#define IPC_RPMSG_LINUX_RESOURCE_TABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/*
 * Keep structure and defines in the file in sync with the linux kernel 
 */

/** \brief Virtio type for Remote Proc Messaging 
 */
#define RPMESSAGE_RSC_VIRTIO_ID_RPMSG  (7U)

/**
 * \name Resource Entry type
 * 
 * @{
 */ 

/** \brief trace type */
#define RPMESSAGE_RSC_TYPE_TRACE       (2U)
/** \brief VDEV type */
#define RPMESSAGE_RSC_TYPE_VDEV        (3U)

/**@}*/

/** \brief Macro to specify VRING memory needs to be dynamically allocated by linux */
#define RPMESSAGE_RSC_VRING_ADDR_ANY (0xFFFFFFFFU)

/** \brief Macro to specify linux trace version */
#define RPMESSAGE_RSC_TRACE_INTS_VER0        (0 << 16)
/** \brief Macro to specify linux trace version */
#define RPMESSAGE_RSC_TRACE_INTS_VER1        (1 << 16)
/** \brief Max length of trace name */
#define RPMESSAGE_RSC_TRACE_NAME_LEN         (32u) 

/**
 * \brief Resource Table Header
 */
typedef struct
{
    uint32_t ver;
    /**< Version Number, set to 1 */
    uint32_t num;
    /**< Number of Entries, MUST be 2 */
    uint32_t reserved[2];
    /**< Reserved for future use, set to 0 */
} RPMessage_RscHdr;

/**
 * \brief Structure used for remoteproc trace
 */
typedef struct
{
    uint32_t  type;
    /** Type of trace, MUST be set to TYPE_TRACE | TRACE_INTS_VER0 */
    uint32_t  da;
    /**< Device Address, physical address of location of trace buffer in remote side */
    uint32_t  len;
    /**< Length of trace buffer  */
    uint32_t  reserved;
    /**< Reserved for future use, set to 0  */
    uint8_t   name[RPMESSAGE_RSC_TRACE_NAME_LEN];
    /**<  Name of the trace */
} RPMessage_RscTrace;

/**
 * \brief Resource Table Device VRing Structure
 */
typedef struct
{
    uint32_t  da;
    /**< device address, physical address of VRING, 
     *   set to RPMSG_VRING_ADDR_ANY, updated by linux, with actual address  
     */
    uint32_t  align;
    /**< Alignment used between AVAIL and USED structures, updated by linux */
    uint32_t  num;
    /**< Number of message buffers, MUST be 256 */
    uint32_t  notifyid;
    /**< NotifyId for receive channel, set 1 for TX VRING and 2 for RX VRING */
    uint32_t  reserved;
    /**< Reserved for future use, set to 0 */
} RPMessage_RscVring;

/**
 *  \brief VDEV structure
 */
typedef struct
{
    uint32_t  type;
    /**< type of VDEV, set to TYPE_VDEV */
    uint32_t  id;
    /**< ID of VDEV, set to VIRTIO_ID_RPMSG */
    uint32_t  notifyid;
    /**< Not used, set to 0  */
    uint32_t  dfeatures;
    /**< Not used, set to 1 */
    uint32_t  gfeatures;
    /**< not used, set to 0 */
    uint32_t  config_len;
    /**< not used, set to 0 */
    uint8_t   status;
    /**< updated by linux, after linux init, this should be 0x7 */
    uint8_t   num_of_vrings;
    /**< number of vrings, set to 2 */
    uint8_t   reserved[2];
    /**< Reserved for future use, set to 0 */
} RPMessage_RscVdev;

/**
 *  \brief IPC Resource Table used by IPC app
 */
typedef struct
{
    RPMessage_RscHdr base;
    /**< Header Information */
    uint32_t offset[2];  
    /**< offset to VDEV and TRACE entries */
    RPMessage_RscVdev vdev;
    /**< VDEV entry */
    RPMessage_RscVring  vring0;
    /**< TX VRING  */
    RPMessage_RscVring  vring1;
    /**< RX VRING */
    RPMessage_RscTrace trace;
    /**< Trace entry  */

} RPMessage_ResourceTable;

#ifdef __cplusplus
}
#endif

#endif /* IPC_RPMSG_LINUX_RESOURCE_TABLE_H_ */
