/*
 *  Copyright (c) 2024 Texas Instruments Incorporated
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

#include <drivers/ipc_notify/v0/ipc_notify_v0.h>
#include <drivers/ipc_notify/v0/ipc_notify_v0_mailbox.h>
#include <drivers/ipc_notify/v0/soc/am65x/ipc_notify_soc.h>
#include <drivers/sciclient.h>

#define NAVSS_INTRTR_INPUT_MAILBOX0_USER0   (436U)
#define NAVSS_INTRTR_INPUT_MAILBOX1_USER0   (432U)
#define NAVSS_INTRTR_INPUT_MAILBOX2_USER0   (428U)
#define NAVSS_INTRTR_INPUT_MAILBOX3_USER0   (424U)
#define NAVSS_INTRTR_INPUT_MAILBOX4_USER0   (420U)
#define NAVSS_INTRTR_INPUT_MAILBOX5_USER0   (416U)
#define NAVSS_INTRTR_INPUT_MAILBOX6_USER0   (412U)
#define NAVSS_INTRTR_INPUT_MAILBOX7_USER0   (408U)
#define NAVSS_INTRTR_INPUT_MAILBOX8_USER0   (404U)
#define NAVSS_INTRTR_INPUT_MAILBOX9_USER0   (400U)
#define NAVSS_INTRTR_INPUT_MAILBOX10_USER0  (396U)
#define NAVSS_INTRTR_INPUT_MAILBOX11_USER0  (392U)

/* core interrupt router offset */
#define NAVSS512_MPU1_0_OUTPUT_OFFSET        (112)
#define NAVSS512_MCU1R5F0_OUTPUT_OFFSET      (120)
#define NAVSS512_MCU1R5F1_OUTPUT_OFFSET      (121)

/**
 * \brief Main NavSS512 - Mailbox input line
 */
static uint32_t gNavss512MbInput[IPC_NOTIFY_MAILBOX_MAX_INSTANCES] =
{
    NAVSS_INTRTR_INPUT_MAILBOX0_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX1_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX2_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX3_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX4_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX5_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX6_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX7_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX8_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX9_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX10_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX11_USER0
};

/* Indexed list of req type */
static const uint16_t gReqType[] =
{
    /* NOTE: This list should match the Core index */
    TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    TISCI_DEV_NAVSS0_INTR_ROUTER_0
};

/* Indexed list of req subtype */
static const uint16_t gReqSubtype[] =
{
    /* NOTE: This list should match the Core index */
    TISCI_RESASG_SUBTYPE_IR_OUTPUT,
    TISCI_RESASG_SUBTYPE_IR_OUTPUT,
    TISCI_RESASG_SUBTYPE_IR_OUTPUT
};

/* Indexed list of dst ids */
static const int32_t gMapDstId[] =
{
    TISCI_DEV_MCU_ARMSS0_CPU0,
    TISCI_DEV_MCU_ARMSS0_CPU1,
    TISCI_DEV_GIC0
};

/* Indexed list of src ids */
static const uint16_t gMapSrcId[] =
{
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER0,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER1,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER2,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER3,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER4,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER5,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER6,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER7,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER8,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER9,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER10,
    TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER11
};

int32_t IpcNotify_sciclientIrqTranslate(uint16_t coreId, uint32_t eventId,
        uint16_t *procIrq);

uintptr_t IpcNotify_getMailboxBaseAddr(uint32_t clusterId)
{
    uintptr_t baseAddr = 0x00000000U;

    if( clusterId < IPC_NOTIFY_MAILBOX_MAX_INSTANCES)
    {
        baseAddr = gIpcNotifyMailboxBaseAddr[clusterId];
    }

    return baseAddr;
}

uint32_t IpcNotify_getNavss512MailboxInputIntr(int32_t clusterId, int32_t userId)
{
    uint32_t   mailboxIntrNum = 0;

    if( (clusterId != MAILBOX_CLUSTER_INVALID)  &&
        (clusterId < IPC_NOTIFY_MAILBOX_MAX_INSTANCES)   &&
        (userId != MAILBOX_USER_INVALID)        &&
        (userId < IPC_NOTIFY_MAILBOX_USER_CNT))
    {
        mailboxIntrNum = gNavss512MbInput[clusterId] + userId;
    }
    return mailboxIntrNum;
}

int32_t IpcNotify_setCoreEventId(uint32_t selfId, IpcNotify_MbIntrConfig* cfg, uint32_t intrCnt)
{
    int32_t    retVal          = SystemP_SUCCESS;
    uint32_t   outIntrBaseNum  = 0U;
    uint32_t   vimEventBaseNum = 0U;
    uint16_t   proc_irq        = 0U;

    /*
     * static variable to used to store the base for first
     * mailbox interrupt register. In subsequent call, it uses
     * the offset of intrCnt from base
     */
    static uint16_t   start    = 0U;
    static uint16_t   range    = 0U;
    uint16_t   offset   = 0;

    /* Get available CorePack IRQ number from DMSC */
    if( (start == 0U) && (range == 0U))
    {
        retVal = IpcNotify_getIntNumRange(selfId, &start, &range);
    }

    if((retVal == SystemP_SUCCESS) && (range >= 1U))
    {
        /* Allocate the last 5 interrupts for IPC. Note that the IR allocation is
         * static so this needs to be carefully set. Currently first interrupt is
         * used by UDMA and middle one's are used by other modules like CPSW9G so
         * we are using last 5 as a safe option.
         */
        if(range >= 5)
        {
            offset = 5;
        }
        else
        {
            offset = range;
        }
        vimEventBaseNum = (start + range) - offset;

        /* Translation must happen after this offset */
        retVal = IpcNotify_sciclientIrqTranslate((uint16_t)selfId, vimEventBaseNum,
                                           &proc_irq);
        if (SystemP_SUCCESS == retVal)
        {
            vimEventBaseNum = proc_irq;
        }

    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    switch(selfId)
    {
        case CSL_CORE_ID_A53SS0_0:
            outIntrBaseNum = NAVSS512_MPU1_0_OUTPUT_OFFSET;
            cfg->outputIntrNum = outIntrBaseNum + intrCnt;
            cfg->eventId       = vimEventBaseNum + intrCnt;
            break;
        case CSL_CORE_ID_R5FSS0_0:
            outIntrBaseNum = NAVSS512_MCU1R5F0_OUTPUT_OFFSET;
            cfg->outputIntrNum = outIntrBaseNum + intrCnt*2;
            cfg->eventId       = vimEventBaseNum + intrCnt;
            break;
        case CSL_CORE_ID_R5FSS0_1:
            outIntrBaseNum = NAVSS512_MCU1R5F1_OUTPUT_OFFSET;
            cfg->outputIntrNum = outIntrBaseNum + intrCnt*2;
            cfg->eventId       = vimEventBaseNum + intrCnt;
            break;
    }

    return retVal;
}


int32_t IpcNotify_getMailboxIntrRouterCfg(uint32_t selfId, uint32_t clusterId, uint32_t userId,
            IpcNotify_MbIntrConfig* cfg, uint32_t cnt)
{
    int32_t    retVal         = SystemP_SUCCESS;
    uint32_t   mailboxIntrNum = 0;

    /* Get Navss512 input interrupt number for mailbox */
    mailboxIntrNum = IpcNotify_getNavss512MailboxInputIntr(clusterId, userId);

    cfg->inputIntrNum  = mailboxIntrNum;
    cfg->priority      = 1U;
    retVal = IpcNotify_setCoreEventId(selfId, cfg, cnt);

    return retVal;
}


int32_t IpcNotify_sciclientIrqTranslate(uint16_t coreId, uint32_t eventId,
        uint16_t *procIrq)
{
    return Sciclient_rmIrqTranslateIrOutput(gReqType[coreId],
                                            (uint16_t)eventId,
                                            (uint16_t)gMapDstId[coreId],
                                            procIrq);
}

int32_t IpcNotify_sciclientIrqRelease(uint16_t coreId, uint32_t clusterId,
        uint32_t userId, uint32_t intNumber)
{
	int32_t                               retVal = SystemP_SUCCESS;
    struct tisci_msg_rm_irq_release_req   rmIrqRel;

    rmIrqRel.ia_id                  = 0U;
    rmIrqRel.vint                   = 0U;
    rmIrqRel.global_event           = 0U;
    rmIrqRel.vint_status_bit_index  = 0U;

    rmIrqRel.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                              TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqRel.src_id         = gMapSrcId[clusterId];
    rmIrqRel.src_index      = userId;
    rmIrqRel.dst_id         = (uint16_t)gMapDstId[coreId];
    rmIrqRel.dst_host_irq   = (uint16_t)intNumber;
    rmIrqRel.secondary_host = (uint8_t)TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    retVal = Sciclient_rmIrqRelease(
                 &rmIrqRel, SystemP_WAIT_FOREVER);

    return retVal;
}


int32_t IpcNotify_sciclientIrqSet(uint16_t coreId, uint32_t clusterId,
        uint32_t userId, uint32_t intNumber)
{
	int32_t                           retVal = SystemP_SUCCESS;
    struct tisci_msg_rm_irq_set_req   rmIrqReq;
    struct tisci_msg_rm_irq_set_resp  rmIrqResp;

    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;

    rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                              TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id         = gMapSrcId[clusterId];
    rmIrqReq.src_index      = (uint16_t)userId;
    rmIrqReq.dst_id         = (uint16_t)gMapDstId[coreId];
    rmIrqReq.dst_host_irq   = (uint16_t)intNumber;
    rmIrqReq.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    /* Config event */
    retVal = Sciclient_rmIrqSet(
                 &rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);

    return retVal;
}

int32_t IpcNotify_getIntNumRange(uint32_t coreIndex,
        uint16_t *rangeStartP, uint16_t *rangeNumP)
{
    int32_t                                     retVal = SystemP_SUCCESS;
    struct tisci_msg_rm_get_resource_range_resp res = {{0, 0, 0, 0}, 0, 0, 0, 0};
    struct tisci_msg_rm_get_resource_range_req  req = {{0, 0, 0, 0}, 0, 0, 0};

    req.type           = gReqType[coreIndex];
    req.subtype        = (uint8_t)gReqSubtype[coreIndex];
    req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    res.range_num = 0;
    res.range_start = 0;

    /* Get interrupt number range */
    retVal =  Sciclient_rmGetResourceRange(
                &req,
                &res,
                SystemP_WAIT_FOREVER);
    if (CSL_PASS != retVal || res.range_num == 0) {
        /* Try with HOST_ID_ALL */
        req.type           = gReqType[coreIndex];
        req.subtype        = (uint8_t)gReqSubtype[coreIndex];
        req.secondary_host = TISCI_HOST_ID_ALL;

        retVal = Sciclient_rmGetResourceRange(
                &req,
                &res,
                SystemP_WAIT_FOREVER);
    }
    if (CSL_PASS == retVal)
    {
        *rangeStartP = res.range_start;
        *rangeNumP = res.range_num;
    }

    return retVal;
}

int32_t IpcNotify_setIntrRtr(uint32_t selfCoreId, uint32_t remoteCoreId, uint32_t mailboxBaseAddr, uint32_t hwFifoId, uint32_t userId)
{
    int32_t  retVal = SystemP_SUCCESS;
    uint32_t mailboxId;
    IpcNotify_MbIntrConfig cfg;
    IpcNotify_MailboxConfig *pMailboxConfig;
    static uint32_t ipc_mBoxCnt = 0U;

    pMailboxConfig = &gIpcNotifyMailboxConfig[remoteCoreId][selfCoreId];
    mailboxId = pMailboxConfig->mailboxId;

    /* Get the Interrupt Configuration */
    retVal = IpcNotify_getMailboxIntrRouterCfg(selfCoreId, mailboxId, userId, &cfg, ipc_mBoxCnt);
    if(retVal == SystemP_SUCCESS){
        uint32_t timeout_cnt = 10;

        /* Release the resource first */
        IpcNotify_sciclientIrqRelease(selfCoreId, mailboxId, userId, cfg.eventId);
        do
        {
            retVal = IpcNotify_sciclientIrqSet(selfCoreId, mailboxId, userId, cfg.eventId);
            timeout_cnt--;
        }while((retVal != 0) && (timeout_cnt > 0));

        if(timeout_cnt == 0)
        {
            retVal = SystemP_FAILURE;
        }else{
            ipc_mBoxCnt++;
        }
    }
    if(retVal == SystemP_SUCCESS){
        IpcNotify_mailboxClearInt(mailboxBaseAddr, hwFifoId, userId);
        IpcNotify_mailboxEnableInt(mailboxBaseAddr, hwFifoId, userId);
    }
    return retVal;
}
