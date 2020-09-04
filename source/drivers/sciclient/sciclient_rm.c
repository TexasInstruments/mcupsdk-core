/*
 * Copyright (c) 2018-2020, Texas Instruments Incorporated
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
 * *  Neither the name of Texas Instruments Incorporated nor the names of
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
 *  \file sciclient_rm.c
 *
 *  \brief File containing the SCICLIENT API interfaces to the resource
 *         management DMSC services.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/sciclient.h>
#include <drivers/sciclient/sciclient_rm_priv.h>

#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Sciclient_rmGetResourceRange(
                const struct tisci_msg_rm_get_resource_range_req *req,
                struct tisci_msg_rm_get_resource_range_resp *resp,
                uint32_t timeout)
{
    int32_t r = SystemP_SUCCESS;
    Sciclient_ReqPrm_t sciReq ;
    struct tisci_msg_rm_get_resource_range_req req_copy;
    sciReq.messageType    = TISCI_MSG_RM_GET_RESOURCE_RANGE;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) &req_copy;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);
    memcpy(&req_copy, req, sizeof(struct tisci_msg_rm_get_resource_range_req));

    if (SystemP_SUCCESS == r)
    {
        r = Sciclient_service(&sciReq, &sciResp);
    }
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmIrqSet(const struct tisci_msg_rm_irq_set_req *req,
                           const struct tisci_msg_rm_irq_set_resp *resp,
                           uint32_t timeout)
{
    return Sciclient_rmProgramInterruptRoute(req, resp, timeout);
}

int32_t Sciclient_rmIrqRelease(const struct tisci_msg_rm_irq_release_req *req,
                               uint32_t timeout)
{
    struct tisci_msg_rm_irq_release_resp resp;

    return Sciclient_rmClearInterruptRoute(req, &resp, timeout);
}

int32_t Sciclient_rmIrqTranslateIrOutput(uint16_t	ir_dev_id,
                                         uint16_t	ir_output,
					 uint16_t	dst_dev_id,
					 uint16_t	*dst_input)
{
    return Sciclient_rmTranslateIntOutput(ir_dev_id,
                                          ir_output,
                                          dst_dev_id,
                                          dst_input);
}

int32_t Sciclient_rmIrqTranslateIaOutput(uint16_t	ia_dev_id,
                                         uint16_t	ia_output,
					 uint16_t	dst_dev_id,
					 uint16_t	*dst_input)
{
    return Sciclient_rmTranslateIntOutput(ia_dev_id,
                                          ia_output,
                                          dst_dev_id,
                                          dst_input);
}

int32_t Sciclient_rmIrqTranslateIrqInput(uint16_t	dst_dev_id,
                                         uint16_t	dst_input,
					 uint16_t	src_dev_id,
					 uint16_t	*src_output)
{
    return Sciclient_rmTranslateIrqInput(dst_dev_id,
                                         dst_input,
                                         src_dev_id,
                                         src_output);
}

int32_t Sciclient_rmIrqSetRaw(const struct tisci_msg_rm_irq_set_req *req,
                              const struct tisci_msg_rm_irq_set_resp *resp,
                              uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_IRQ_SET;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);


    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmIrqReleaseRaw(const struct tisci_msg_rm_irq_release_req *req,
                                  uint32_t timeout)
{
    int32_t r;

    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_IRQ_RELEASE;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    struct tisci_msg_rm_irq_release_resp resp;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *)&resp;
    sciResp.respPayloadSize = sizeof(resp);

    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmRingCfg(const struct tisci_msg_rm_ring_cfg_req *req,
                            const struct tisci_msg_rm_ring_cfg_resp *resp,
                            uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_RING_CFG;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);

    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmRingMonCfg(const struct tisci_msg_rm_ring_mon_cfg_req *req,
                               const struct tisci_msg_rm_ring_mon_cfg_resp *resp,
                               uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_RING_MON_CFG;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);

    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmUdmapGcfgCfg(
            const struct tisci_msg_rm_udmap_gcfg_cfg_req *req,
            const struct tisci_msg_rm_udmap_gcfg_cfg_resp *resp,
            uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_UDMAP_GCFG_CFG;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);



    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmUdmapTxChCfg(
            const struct tisci_msg_rm_udmap_tx_ch_cfg_req *req,
            const struct tisci_msg_rm_udmap_tx_ch_cfg_resp *resp,
            uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_UDMAP_TX_CH_CFG;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);


    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmUdmapRxChCfg(
            const struct tisci_msg_rm_udmap_rx_ch_cfg_req *req,
            const struct tisci_msg_rm_udmap_rx_ch_cfg_resp *resp,
            uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_UDMAP_RX_CH_CFG;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);



    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmUdmapFlowCfg(
            const struct tisci_msg_rm_udmap_flow_cfg_req *req,
            const struct tisci_msg_rm_udmap_flow_cfg_resp *resp,
            uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_UDMAP_FLOW_CFG;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);



    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmUdmapFlowSizeThreshCfg(
            const struct tisci_msg_rm_udmap_flow_size_thresh_cfg_req *req,
            const struct tisci_msg_rm_udmap_flow_size_thresh_cfg_resp *resp,
            uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_UDMAP_FLOW_SIZE_THRESH_CFG;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);



    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmPsilPair(const struct tisci_msg_rm_psil_pair_req *req,
                             uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_PSIL_PAIR;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    struct tisci_msg_rm_psil_pair_resp resp;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t*)&resp;
    sciResp.respPayloadSize = sizeof(resp);

    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmPsilUnpair(const struct tisci_msg_rm_psil_unpair_req *req,
                               uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_PSIL_UNPAIR;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    struct tisci_msg_rm_psil_unpair_resp resp;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *)&resp;
    sciResp.respPayloadSize = sizeof(resp);

    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmPsilRead(const struct tisci_msg_rm_psil_read_req *req,
                             struct tisci_msg_rm_psil_read_resp *resp,
                             uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_PSIL_READ;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t *) resp;
    sciResp.respPayloadSize = (uint32_t) sizeof(*resp);

    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmPsilWrite(const struct tisci_msg_rm_psil_write_req *req,
                              uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_RM_PSIL_WRITE;
    sciReq.flags          = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    struct tisci_msg_rm_psil_write_resp resp;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t*)&resp;
    sciResp.respPayloadSize = sizeof(resp);

    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}

int32_t Sciclient_rmSetProxyCfg(const struct tisci_msg_rm_proxy_cfg_req *req, uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType = TISCI_MSG_RM_PROXY_CFG;
    sciReq.flags       = TISCI_MSG_FLAG_AOP;
    sciReq.pReqPayload    = (const uint8_t *) req;
    sciReq.reqPayloadSize = (uint32_t) sizeof(*req);
    sciReq.timeout        = timeout;

    Sciclient_RespPrm_t sciResp ;
    struct tisci_msg_rm_proxy_cfg_resp resp;
    sciResp.flags           = 0;
    sciResp.pRespPayload    = (uint8_t*)&resp;
    sciResp.respPayloadSize = sizeof(resp);

    r = Sciclient_service(&sciReq, &sciResp);
    if ((r != SystemP_SUCCESS) ||
        ((sciResp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)) {
        r = SystemP_FAILURE;
    }

    return r;
}


/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

/* None */

