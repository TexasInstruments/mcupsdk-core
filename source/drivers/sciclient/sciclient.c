/*
 * Copyright (c) 2017-2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  \file sciclient.c
 *
 *  \brief File containing the SCICLIENT driver APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/sciclient/sciclient_priv.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <string.h> /*For memcpy*/
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*Header size in words*/
#define SCICLIENT_HEADER_SIZE_IN_WORDS (sizeof (struct tisci_header) \
                                        / sizeof (uint32_t))

/** Indicate that this message is marked secure */
#define TISCI_MSG_FLAG_MASK    (TISCI_BIT(0) | TISCI_BIT(1))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief   Gives the address for status register for a particular thread.
 *
 *  \param   thread    Index of the thread.
 *
 *  \return  address   address of the thread status
 */
static inline uint32_t Sciclient_secProxyThreadStatusReg(uint32_t thread);

/**
 *  \brief   Read a 32 bit word from the thread.
 *
 *  \param   thread    Index of the thread to be read from.
 *  \param   idx       Index of the word to be read from the thread.
 *
 *  \return  word      Value read back.
 */
static inline uint32_t Sciclient_secProxyReadThread32(uint32_t thread, uint8_t idx);

/**
 *  \brief   Read the current thread count.
 *
 *  \param   thread    Index of the thread to be read from.
 *
 *  \return  word      Count read back.
 */
static inline uint32_t Sciclient_secProxyReadThreadCount(uint32_t thread);

/**
 *  \brief   Validate thread has no errors and has space to accept the next
 *           message.
 *
 *  \param   thread    Index of the thread.
 *
 *  \return  status    Status of the message.
 */
static int32_t Sciclient_secProxyVerifyThread(uint32_t thread);

/**
 *  \brief   Check if there are credits to write to the thread.
 *
 *  \param   thread    Index of the thread.
 *  \param   timeout   Wait for timeout if operation is complete.
 *
 *  \return  status    Status of the message.
 */
static int32_t Sciclient_secProxyWaitThread(uint32_t thread, uint32_t timeout);

/**
 *  \brief   API to send the message to the thread.
 *
 *  \param   thread         Index of the thread.
 *  \param   pSecHeader     Pointer to the security header extension.
 *  \param   pHeader        Pointer to the header structure.
 *  \param   pPayload       Pointer to the payload structure.
 *  \param   payloadSize    Size of the payload.
 *
 *  \return  None
 */
static void Sciclient_sendMessage(uint32_t        thread,
                                  const uint8_t  *pSecHeader,
                                  const uint8_t  *pHeader,
                                  const uint8_t  *pPayload,
                                  uint32_t        payloadSize);

/**
 *  \brief   API to wait for message from the thread.
 *
 *  \param   rxThread          Index of the response thread.
 *  \param   timeout           Timeout value to wait until
 *  \param   initialCount      Thread count before sending message
 *  \param   localSeqId        Sequence Id of the message just sent
 *
 *  \return  status            Status of the wait (Timeout or Pass)
 */

static int32_t Sciclient_waitForMessage(uint32_t rxThread,
                                         uint32_t timeout,
                                         uint32_t initialCount,
                                         uint8_t localSeqId);

/**
 *  \brief   API to receive the message from the thread.
 *
 *  \param   rxThread          Index of the response thread.
 *  \param   pLocalRespPayload Pointer to the response payload.
 *  \param   rxPayloadSize     Size of the response payload.
 *
 *  \return  None
 */

static void Sciclient_recvMessage(uint32_t rxThread,
                                  uint8_t *pLocalRespPayload,
                                  uint32_t rxPayloadSize);
/**
 *  \brief   API to flush/remove all outstanding messages on a thread .
 *
 *  \param   thread    Index of the thread.
 *
 *  \return None
 */
static void Sciclient_secProxyFlush(uint32_t thread);

/**
 *  \brief   API to get the transmission thread ID.
 *
 *  \param   contextId  Context of the CPU.
 *
 *  \return None
 */
int32_t Sciclient_getTxThreadId(uint32_t contextId);

/**
 *  \brief   API to get the receive thread ID.
 *
 *  \param   contextId   Context of the CPU.
 *
 *  \return None
 */
int32_t Sciclient_getRxThreadId(uint32_t contextId);

/* ========================================================================== */
/*                            Extern Functions                                */
/* ========================================================================== */

extern uint32_t Sciclient_getContext(uint32_t contextType, uint32_t coreId);
extern uint32_t Sciclient_getDevId(uint32_t coreId);

/* ========================================================================== */
/*                            Extern Variables                                */
/* ========================================================================== */

extern CSL_SecProxyCfg gSciclientSecProxyCfg;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/**
 *   \brief Handle used by #Sciclient_service function
 */
static Sciclient_ServiceHandle_t gSciclientHandle = {0};

/**
 *   \brief Size of secure header.This is initialized when the context is
 *          SECURE.
 */
static uint8_t gSecHeaderSizeWords = 0;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Sciclient_init(uint32_t coreId)
{
    int32_t   status = SystemP_SUCCESS;

    /* convert system address to CPU local address */
    gSciclientSecProxyCfg.pSecProxyRegs
        = (CSL_sec_proxyRegs*)AddrTranslateP_getLocalAddr( (uint64_t)gSciclientSecProxyCfg.pSecProxyRegs);
    gSciclientSecProxyCfg.pSecProxyScfgRegs
        = (CSL_sec_proxy_scfgRegs*)AddrTranslateP_getLocalAddr( (uint64_t)gSciclientSecProxyCfg.pSecProxyScfgRegs);
    gSciclientSecProxyCfg.pSecProxyRtRegs
        = (CSL_sec_proxy_rtRegs*)AddrTranslateP_getLocalAddr( (uint64_t)gSciclientSecProxyCfg.pSecProxyRtRegs);
    gSciclientSecProxyCfg.proxyTargetAddr
        = (uint64_t)AddrTranslateP_getLocalAddr( (uint64_t)gSciclientSecProxyCfg.proxyTargetAddr);

    gSciclientHandle.currSeqId = 0;
    gSciclientHandle.coreId = coreId;
    gSciclientHandle.devIdCore = Sciclient_getDevId(coreId);
    gSciclientHandle.secureContextId = Sciclient_getContext(SCICLIENT_SECURE_CONTEXT, coreId);
    gSciclientHandle.nonSecureContextId = Sciclient_getContext(SCICLIENT_NON_SECURE_CONTEXT, coreId);
    gSciclientHandle.maxMsgSizeBytes = CSL_secProxyGetMaxMsgSize(&gSciclientSecProxyCfg) -
                                CSL_SEC_PROXY_RSVD_MSG_BYTES;

    return status;
}

int32_t Sciclient_deinit(void)
{
    int32_t   status = SystemP_SUCCESS;
    return status;
}

int32_t Sciclient_abiCheck(void)
{
    int32_t status = SystemP_SUCCESS;
    /* Send getRevision message for ABI Revision Check */
    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) &request,
        sizeof (request),
        SystemP_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    /* Explicitly initialize the value to something other than SCICLIENT_FIRMWARE_ABI_MAJOR
     * so that the function doesn't accidentally pass.
     */
    response.abi_major = 0xFFU;

    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        (uint32_t) sizeof (response)
    };

    status = Sciclient_service(&reqPrm, &respPrm);
    if ((status != SystemP_SUCCESS) ||
        (respPrm.flags != TISCI_MSG_FLAG_ACK) ||
        ((uint32_t)(response.abi_major) != SCICLIENT_FIRMWARE_ABI_MAJOR) )
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t Sciclient_getVersionCheck(uint32_t doLog)
{
    int32_t status;
    struct tisci_msg_version_req req = {0};
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (const uint8_t *)&req,
        sizeof(req),
        SystemP_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    /* Explicitly initialize the value to something other than SCICLIENT_FIRMWARE_ABI_MAJOR
     * so that we would know the getVersion failed at least from the prints.
     */
    response.version = 0xFFFFU;
    response.abi_major = 0xFFU;
    response.abi_minor = 0xFFU;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    status = Sciclient_service(&reqPrm, &respPrm);
    if ( (SystemP_SUCCESS == status) && (respPrm.flags == TISCI_MSG_FLAG_ACK))
    {
        if(doLog)
        {
            DebugP_log("\r\n");
            DebugP_log("DMSC Firmware Version %s\r\n",
                                (char *) response.str);
            DebugP_log("DMSC Firmware revision 0x%x\r\n", response.version);
            DebugP_log("DMSC ABI revision %d.%d\r\n", response.abi_major,
                                response.abi_minor);
            DebugP_log("\r\n");
        }
    }
    else
    {
        status = SystemP_FAILURE;
        if(doLog)
        {
            DebugP_log("\r\n");
            DebugP_logError("[ERROR] Sciclient get version failed !!!\r\n");
        }
    }
    return status;
}

int32_t Sciclient_triggerSecHandover(void)
{
    int32_t status;
    struct tisci_msg_security_handover_req req = {0};
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_SEC_HANDOVER,
        TISCI_MSG_FLAG_AOP,
        (const uint8_t *)&req,
        sizeof(req),
        SystemP_WAIT_FOREVER
    };

    struct tisci_msg_security_handover_resp response = {0};
    Sciclient_RespPrm_t respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    status = Sciclient_service(&reqPrm, &respPrm);
    if ( (SystemP_SUCCESS == status) && (respPrm.flags == TISCI_MSG_FLAG_ACK))
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }


    return status;
}

/* HWI_disable instead of semaphores for MCAL polling based. define in separate files*/
int32_t Sciclient_service(const Sciclient_ReqPrm_t *pReqPrm,
                          Sciclient_RespPrm_t      *pRespPrm)
{
    int32_t   status        = SystemP_SUCCESS;
    uint32_t  contextId     = SCICLIENT_CONTEXT_MAX_NUM;
    uint32_t  initialCount  = 0U;
    uint8_t   localSeqId    = gSciclientHandle.currSeqId;
    /* size of request payload in bytes  */
    uint32_t  txPayloadSize = 0U;
    /* size of response payload in bytes */
    uint32_t  rxPayloadSize = 0U;
    uint8_t  *pLocalRespPayload = NULL;
    uint32_t  txThread;
    uint32_t  rxThread;
    uintptr_t key;
    uint8_t  *pSecHeader = NULL;
    struct tisci_header *header;
    struct tisci_sec_header secHeader;

    /* Run all error checks */
    if((pReqPrm == NULL) || (pRespPrm == NULL) || (pReqPrm->pReqPayload == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        contextId = Sciclient_getCurrentContext(pReqPrm->messageType);
        if(contextId < SCICLIENT_CONTEXT_MAX_NUM)
        {
            txThread = Sciclient_getTxThreadId(contextId);
            rxThread = Sciclient_getRxThreadId(contextId);

            if(gSciclientMap[contextId].context == SCICLIENT_SECURE_CONTEXT)
            {
                gSecHeaderSizeWords = sizeof(struct tisci_sec_header)/sizeof(uint32_t);
            }
            else
            {
                gSecHeaderSizeWords = 0;
            }
            gSciclientHandle.maxMsgSizeBytes = CSL_secProxyGetMaxMsgSize(&gSciclientSecProxyCfg) -
                                        CSL_SEC_PROXY_RSVD_MSG_BYTES;

            if(gSciclientMap[contextId].context == SCICLIENT_SECURE_CONTEXT)
            {
                secHeader.integ_check = (uint16_t)0;
                secHeader.rsvd = (uint16_t)0;
                pSecHeader = (uint8_t * )(&secHeader);
            }
            if (pReqPrm->reqPayloadSize > 0U)
            {
                txPayloadSize = pReqPrm->reqPayloadSize - sizeof(struct tisci_header);
            }
            else
            {
                txPayloadSize = 0U;
            }
            if (txPayloadSize > (gSciclientHandle.maxMsgSizeBytes - sizeof(struct tisci_header)))
            {
                status = SystemP_FAILURE;
            }
            if ((txPayloadSize > 0U) && (pReqPrm->pReqPayload == NULL))
            {
                status = SystemP_FAILURE;
            }
            if (pRespPrm->respPayloadSize > 0U)
            {
                rxPayloadSize = pRespPrm->respPayloadSize - sizeof(struct tisci_header);
            }
            else
            {
                rxPayloadSize = 0U;
            }
            if (rxPayloadSize > (gSciclientHandle.maxMsgSizeBytes - sizeof(struct tisci_header)))
            {
                status = SystemP_FAILURE;
            }
            if ((rxPayloadSize > 0U) && (pRespPrm->pRespPayload == NULL))
            {
                status = SystemP_FAILURE;
            }
            else
            {
                pLocalRespPayload = (uint8_t *)(pRespPrm->pRespPayload + sizeof(struct tisci_header));
            }
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    /* CRITICAL Section */
    key = HwiP_disable();

    if (SystemP_SUCCESS == status)
    {
        uint8_t * pFlags;
        uint32_t numBytes;

        /* Construct header */

        /* This is done to remove stray messages(due to timeout) in a thread
         * in case of "polling". */
        Sciclient_secProxyFlush(rxThread);

        header = (struct tisci_header*)pReqPrm->pReqPayload;
        header->type = pReqPrm->messageType;
        header->host = (uint8_t) gSciclientMap[contextId].hostId;
        header->seq = localSeqId;
        pFlags = (uint8_t*)&pReqPrm->flags;
        /* This is done in such a fashion for CPUs which do not honor a non word aligned
         * write.
         */
        for (numBytes = 0; numBytes < sizeof(pReqPrm->flags); numBytes++)
        {
            uint8_t *pDestFlags = ((uint8_t*)&header->flags) + numBytes;
            *pDestFlags = *pFlags;
            pFlags++;
        }

        gSciclientHandle.currSeqId = (gSciclientHandle.currSeqId + 1U) %
                                    SCICLIENT_MAX_QUEUE_SIZE;

        /* Verify thread status before reading/writing */
        status = Sciclient_secProxyVerifyThread(txThread);
        if (SystemP_SUCCESS == status)
        {
            status = Sciclient_secProxyWaitThread(txThread, pReqPrm->timeout);
        }
    }
    if (SystemP_SUCCESS == status)
    {
        /* Send Message */
        initialCount = Sciclient_secProxyReadThreadCount(rxThread);
        Sciclient_sendMessage(txThread, pSecHeader ,(uint8_t *) header,
                              (pReqPrm->pReqPayload + sizeof(struct tisci_header)),
                              txPayloadSize);

        /* Verify thread status before reading/writing */
        status = Sciclient_secProxyVerifyThread(rxThread);
    }
    /* Wait for response: Polling based waiting */
    if ((status == SystemP_SUCCESS) &&
        ((pReqPrm->flags & TISCI_MSG_FLAG_MASK) != 0U))
    {
        status = Sciclient_waitForMessage(rxThread, pReqPrm->timeout, initialCount, localSeqId);
    }
    if(status == SystemP_SUCCESS)
    {
        pRespPrm->flags = Sciclient_secProxyReadThread32(rxThread, 1U+gSecHeaderSizeWords);
        Sciclient_recvMessage(rxThread, pLocalRespPayload, rxPayloadSize);
    }
    HwiP_restore(key);

    return status;
}

int32_t Sciclient_loadFirmware(const uint32_t *pSciclient_firmware)
{
    int32_t  status   = SystemP_SUCCESS;
    uint32_t txThread = SCICLIENT_ROM_R5_TX_NORMAL_THREAD;
    uint32_t rxThread = SCICLIENT_ROM_R5_RX_NORMAL_THREAD;
    Sciclient_RomFirmwareLoadHdr_t header      = {0};
    Sciclient_RomFirmwareLoadPayload_t payload = {0};
    uint32_t secHeaderSizeWords = sizeof(struct tisci_sec_header)/sizeof(uint32_t);

    volatile Sciclient_RomFirmwareLoadHdr_t *pLocalRespHdr =
        (Sciclient_RomFirmwareLoadHdr_t *)CSL_secProxyGetDataAddr
                                        (&gSciclientSecProxyCfg, rxThread, 0U);
    uint8_t  payloadSize = sizeof (Sciclient_RomFirmwareLoadPayload_t) /
                           sizeof (uint8_t);

    /* Construct header */
    header.type = SCICLIENT_ROM_MSG_R5_TO_M3_M3FW;

    header.host = SCICLIENT_HOST_ID;
    /* ROM expects a sequence number of 0 */
    header.seq  = 0U;
    /* ROM doesn't check for flags */
    header.flags = 0U;

    if (pSciclient_firmware != NULL)
    {
        payload.bufferAddress = (uint32_t)(uintptr_t)pSciclient_firmware;

        /*Size is not needed actually.It is taken from x509 certificate*/
        payload.bufferSizeBytes = 0xffffffffU;

        /* Verify thread status before reading/writing */
        status = Sciclient_secProxyVerifyThread(txThread);
        if (SystemP_SUCCESS == status)
        {
            status = Sciclient_secProxyWaitThread(txThread, SystemP_WAIT_FOREVER);
        }
        if (SystemP_SUCCESS == status)
        {
            /* Writing header and payload */
            Sciclient_sendMessage(txThread,NULL, (uint8_t *) &header,
                                  (uint8_t *)&payload, payloadSize);

            /* CHECKING FOR FIRMWARE LOAD ACK */
            /* Verify thread status before reading/writing */
            status = Sciclient_secProxyVerifyThread(rxThread);
        }
        if (SystemP_SUCCESS == status)
        {
            while ((CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(rxThread)) &
                 CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) == 0U) {;}
            /* Check the message type and flag of the response */
            if ((pLocalRespHdr->type ==
                SCICLIENT_ROM_MSG_M3_TO_R5_M3FW_RESULT)
                && (pLocalRespHdr->flags == SCICLIENT_ROM_MSG_CERT_AUTH_PASS))
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
            /* Reading from the last register of rxThread. This flushes the thread*/
            (void) Sciclient_secProxyReadThread32(rxThread,
                            (uint8_t)((gSciclientHandle.maxMsgSizeBytes/4U)-1U));
        }

        /* CHECKING FOR TISCI_MSG_BOOT_NOTIFICATION from DMSC*/
        pLocalRespHdr =
        (Sciclient_RomFirmwareLoadHdr_t *)(CSL_secProxyGetDataAddr(
                                            &gSciclientSecProxyCfg, rxThread, 0U)
                                            + ((uintptr_t) secHeaderSizeWords * (uintptr_t) 4U));
        if (status == SystemP_SUCCESS)
        {
            status = Sciclient_secProxyVerifyThread(rxThread);
        }
        if (status == SystemP_SUCCESS)
        {
            while ((CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(rxThread)) &
                 CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) == 0U) {;}
            /* Check the message type and flag of the response */
            if (pLocalRespHdr->type ==
                TISCI_MSG_BOOT_NOTIFICATION)
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
            /* Reading from the last register of rxThread*/
            (void) Sciclient_secProxyReadThread32(rxThread,
                            (uint8_t)((gSciclientHandle.maxMsgSizeBytes/4U)-1U));
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t Sciclient_waitForBootNotification(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t rxThread = SCICLIENT_ROM_R5_RX_NORMAL_THREAD;
    uint32_t secHeaderSizeWords = sizeof(struct tisci_sec_header)/sizeof(uint32_t);
    volatile Sciclient_RomFirmwareLoadHdr_t *pLocalRespHdr;
    uint32_t maxMsgSizeBytes;

    status = Sciclient_secProxyVerifyThread(rxThread);

    if (status == SystemP_SUCCESS)
    {
        while ((CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(rxThread)) &
             CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) == 0U) {;}

        maxMsgSizeBytes = CSL_secProxyGetMaxMsgSize(&gSciclientSecProxyCfg) -
                                        CSL_SEC_PROXY_RSVD_MSG_BYTES;

        pLocalRespHdr = (Sciclient_RomFirmwareLoadHdr_t *)(CSL_secProxyGetDataAddr(
                                            &gSciclientSecProxyCfg, rxThread, 0U)
                                            + ((uintptr_t) secHeaderSizeWords * (uintptr_t) 4U));

        /* Check the message type and flag of the response */
        if (pLocalRespHdr->type ==
            TISCI_MSG_BOOT_NOTIFICATION)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
        /* Reading from the last register of rxThread*/
        (void) Sciclient_secProxyReadThread32(rxThread,
                        (uint8_t)((maxMsgSizeBytes/4U)-1U));
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

uint32_t Sciclient_getCurrentContext(uint16_t messageType)
{
    uint32_t retVal = SCICLIENT_CONTEXT_MAX_NUM;

    if((TISCI_MSG_BOOT_NOTIFICATION == messageType) ||
       (TISCI_MSG_SEC_HANDOVER == messageType) ||
       (TISCI_MSG_BOARD_CONFIG == messageType) ||
       (TISCI_MSG_BOARD_CONFIG_RM == messageType) ||
       (TISCI_MSG_BOARD_CONFIG_SECURITY == messageType) ||
       (TISCI_MSG_KEY_WRITER == messageType) ||
       (TISCI_MSG_READ_OTP_MMR == messageType) ||
       (TISCI_MSG_WRITE_OTP_ROW == messageType) ||
       (TISCI_MSG_READ_SWREV == messageType) ||
       (TISCI_MSG_WRITE_SWREV == messageType) ||
       (TISCI_MSG_BOARD_CONFIG_PM == messageType))
    {
        retVal = gSciclientHandle.secureContextId;
    }
    else
    {
        /* For all other message type use non-secure context */
        retVal = gSciclientHandle.nonSecureContextId;
    }

    return retVal;
}

uint32_t Sciclient_getSelfDevIdCore(void)
{
    return gSciclientHandle.devIdCore;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

#if defined (__C7100__)
#ifdef __cplusplus
#pragma FUNCTION_OPTIONS("--opt_level=off")
#else
#pragma FUNCTION_OPTIONS(Sciclient_sendMessage, "--opt_level=off")
#endif
#endif
static void Sciclient_sendMessage(uint32_t        thread,
                                  const uint8_t  *pSecHeader,
                                  const uint8_t  *pHeader,
                                  const uint8_t  *pPayload,
                                  uint32_t        payloadSize)
{
    uint32_t        i   = 0U;
    const uint8_t *msg = pSecHeader;
    uint32_t numWords   = 0U;
    uint32_t test = 0U;
    uintptr_t threadAddr = CSL_secProxyGetDataAddr(&gSciclientSecProxyCfg, thread, 0U);

    if(pSecHeader != NULL)
    {
        /* Write secure header */
        for (i = 0U; i < gSecHeaderSizeWords; i++)
        {
            /*Change this when unaligned access is supported*/
            (void) memcpy((void *)&test, (const void *)msg, 4);
            CSL_REG32_WR(threadAddr, test);
            msg += 4;
            threadAddr+=sizeof(uint32_t);
        }
    }
    /* Write header */
    msg = pHeader;
    for (i = 0U; i < SCICLIENT_HEADER_SIZE_IN_WORDS; i++)
    {
        /*Change this when unaligned access is supported*/
        (void) memcpy((void *)&test, (const void *)msg, 4);
        CSL_REG32_WR(threadAddr, test);
        msg += 4;
        threadAddr+=sizeof(uint32_t);
    }
    /* Writing payload */
    if (payloadSize > 0U)
    {
        numWords   = (payloadSize+3U)/4U;
        msg = pPayload;
        for (; i < (SCICLIENT_HEADER_SIZE_IN_WORDS + numWords); i++)
        {
            /*Change this when unaligned access is supported*/
            (void) memcpy((void *)&test, (const void *)msg, 4);
            CSL_REG32_WR(threadAddr, test);
            msg += 4;
            threadAddr+=sizeof(uint32_t);
        }
    }
    /* Write to the last register of the TX thread */
    if ((((uint32_t) gSecHeaderSizeWords*4U)+(SCICLIENT_HEADER_SIZE_IN_WORDS*4U)+payloadSize) <=
        (gSciclientHandle.maxMsgSizeBytes - 4U))
    {
        threadAddr = CSL_secProxyGetDataAddr(&gSciclientSecProxyCfg, thread, 0U) +
        ((uintptr_t) gSciclientHandle.maxMsgSizeBytes  - (uintptr_t) 4U) ;
        CSL_REG32_WR(threadAddr,0U);
    }
}

static int32_t Sciclient_waitForMessage(uint32_t rxThread, uint32_t timeout, uint32_t initialCount, uint8_t localSeqId)
{
    volatile struct tisci_header *pLocalRespHdr;
    uint32_t timeToWait = timeout;
    int32_t status = SystemP_SUCCESS;

    pLocalRespHdr =
        (struct tisci_header *)(CSL_secProxyGetDataAddr(
                                        &gSciclientSecProxyCfg, rxThread, 0U)
                                + ((uintptr_t) gSecHeaderSizeWords * (uintptr_t) 4U));

    /* Check if some message is received*/
    while (((CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(rxThread)) &
            CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) - initialCount) <= 0U)
    {
        if (timeToWait != 0U)
        {
            timeToWait--;
        }
        else
        {
            status = SystemP_TIMEOUT;
            break;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        /* Check the seqId of response*/
        status = SystemP_TIMEOUT;
        timeToWait =  timeout;
        while(1)
        {
            uint32_t numCurrentMsgs = (CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(rxThread)) &
                    CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) - initialCount;
            if (pLocalRespHdr->seq == (uint32_t)localSeqId)
            {
                status = SystemP_SUCCESS;
                break;
            }
            if (numCurrentMsgs > 1U)
            {
                (void) Sciclient_secProxyReadThread32(rxThread,
                                            (uint8_t)((gSciclientHandle.maxMsgSizeBytes/4U) - 1U));
            }
            if (timeToWait != 0U)
            {
                timeToWait--;
            }
            else
            {
                status = SystemP_TIMEOUT;
                break;
            }
        }
    }

    return status;
}

static void Sciclient_recvMessage(uint32_t rxThread, uint8_t *pLocalRespPayload, uint32_t rxPayloadSize)
{
    uint32_t numWords, i;
    uint8_t  trailBytes = 0U;

    numWords   = (uint32_t) (rxPayloadSize / 4U);
    trailBytes = (uint8_t) (rxPayloadSize - (numWords * 4U));
    /* Read the full message */
    /* We do not need to read the header*/
    for (i = 0; i < numWords; i++)
    {
        uint32_t tempWord = Sciclient_secProxyReadThread32(
            rxThread,
            ((uint8_t) i +
                SCICLIENT_HEADER_SIZE_IN_WORDS+gSecHeaderSizeWords));
        uint8_t * tempWordPtr = (uint8_t*) & tempWord;
        uint32_t j = 0U;
        for (j = 0U; j < 4U; j++)
        {
            *(pLocalRespPayload + i * 4 + j) = *tempWordPtr;
            tempWordPtr++;
        }
    }
    if (trailBytes > 0U)
    {
        uint32_t tempWord = Sciclient_secProxyReadThread32(
                rxThread,
                ((uint8_t)i +
                    SCICLIENT_HEADER_SIZE_IN_WORDS+gSecHeaderSizeWords));
        uint8_t * pTempWord = (uint8_t*) &tempWord;
        uint32_t bytes;
        for (bytes = 0U; bytes < trailBytes; bytes++)
        {
            uint8_t * address = (uint8_t*)pLocalRespPayload;
            uint8_t value = *(uint8_t*)(pTempWord + bytes);
            *(uint8_t*)(address + i*4 + bytes) = value;
        }
    }
    /* Read the last register of the rxThread */
    if ((((uint32_t) gSecHeaderSizeWords*4U) +
        (SCICLIENT_HEADER_SIZE_IN_WORDS*4U) +
        rxPayloadSize) <=
        (gSciclientHandle.maxMsgSizeBytes - 4U))
    {
        (void) Sciclient_secProxyReadThread32(rxThread,
                        (uint8_t)((gSciclientHandle.maxMsgSizeBytes/4U) - 1U));
    }
}

static inline uint32_t Sciclient_secProxyThreadStatusReg(uint32_t thread)
{
    return ((uint32_t)(uintptr_t)(gSciclientSecProxyCfg.pSecProxyRtRegs) +
        CSL_SEC_PROXY_RT_THREAD_STATUS(thread));
}

static inline uint32_t Sciclient_secProxyReadThread32(uint32_t thread, uint8_t idx)
{
    uint32_t ret;
    ret = CSL_REG32_RD(CSL_secProxyGetDataAddr(&gSciclientSecProxyCfg,thread,0U) +
        ((uintptr_t) (0x4U) * (uintptr_t) idx));
    return ret;
}

static inline uint32_t Sciclient_secProxyReadThreadCount(uint32_t thread)
{
    return (CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(thread)) &
        CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK);
}

static int32_t Sciclient_secProxyVerifyThread(uint32_t thread)
{
    int32_t status = SystemP_SUCCESS;
    /* Verify thread status before reading/writing */
    if ((CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(thread)) &
        CSL_SEC_PROXY_RT_THREAD_STATUS_ERROR_MASK) != 0U)
    {
        status = SystemP_FAILURE;
    }
    return status;
}

static int32_t Sciclient_secProxyWaitThread(uint32_t thread, uint32_t timeout)
{
    int32_t  status     = CSL_ETIMEOUT;
    uint32_t timeToWait = timeout;
    /* Checks the thread count is > 0 */
    while (timeToWait > 0U)
    {
        if ((CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(thread)) &
            CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) > 0U)
        {
            status = SystemP_SUCCESS;
            break;
        }
        timeToWait--;
    }
    return status;
}

static void Sciclient_secProxyFlush(uint32_t thread)
{
    while ((CSL_REG32_RD(Sciclient_secProxyThreadStatusReg(thread)) &
        CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) > 0U)
    {
        /* Reading from the last register of rxThread*/
        (void) Sciclient_secProxyReadThread32(thread,
                        (uint8_t)((gSciclientHandle.maxMsgSizeBytes/4U)-1U));
    }

    return ;
}
