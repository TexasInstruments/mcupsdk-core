/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
 *  \file sciclient_firewall.c
 *
 *  \brief File containing the SCICLIENT API interfaces to the firewall
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

int32_t Sciclient_firewallChangeOwnerInfo(
    const struct tisci_msg_fwl_change_owner_info_req *req,
    struct tisci_msg_fwl_change_owner_info_resp *resp,
    uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_CHANGE_FWL_OWNER;
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

int32_t Sciclient_firewallSetRegion(
    const struct tisci_msg_fwl_set_firewall_region_req *req,
    struct tisci_msg_fwl_set_firewall_region_resp *resp,
    uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_SET_FWL_REGION;
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

int32_t Sciclient_firewallGetRegion(
    const struct tisci_msg_fwl_get_firewall_region_req *req,
    struct tisci_msg_fwl_get_firewall_region_resp *resp,
    uint32_t timeout)
{
    int32_t r;
    Sciclient_ReqPrm_t sciReq ;
    sciReq.messageType    = TISCI_MSG_GET_FWL_REGION;
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


/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

/* None */

