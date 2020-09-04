/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
 *  \file sciclient_procboot.c
 *
 *  \brief File containing the SCICLIENT API interfaces to the Processor boot
 *         DMSC services.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/sciclient.h>

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

int32_t Sciclient_procBootRequestProcessor(uint8_t processorId,
                                           uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_proc_request_req request ;
    request.processor_id       = (uint8_t) processorId;

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_PROC_REQUEST;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) 0;
    respParam.respPayloadSize = (uint32_t) 0;

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}


int32_t Sciclient_procBootReleaseProcessor(uint8_t  processorId,
                                           uint32_t reqFlag,
                                           uint32_t  timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_proc_release_req request ;
    request.processor_id       = (uint8_t) processorId;

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_PROC_RELEASE;
    reqParam.flags          = (uint32_t) reqFlag;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) 0;
    respParam.respPayloadSize = (uint32_t) 0;

    if (((reqFlag & TISCI_MSG_FLAG_AOP) != TISCI_MSG_FLAG_AOP) &&
        (reqFlag != 0U))
    {
        retVal = SystemP_FAILURE;
    }
    if(retVal == SystemP_SUCCESS)
    {
        retVal = Sciclient_service(&reqParam, &respParam);
    }
    if((retVal != SystemP_SUCCESS) ||
        ((reqFlag != 0U) &&
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_procBootHandoverProcessor(uint8_t  processorId,
                                            uint8_t  hostId,
                                            uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_proc_handover_req request ;
    request.processor_id     = (uint8_t) processorId;
    request.host_id          = (uint8_t) hostId;

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_PROC_HANDOVER;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) 0;
    respParam.respPayloadSize = (uint32_t) 0;

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_procBootSetProcessorCfg (
            const struct tisci_msg_proc_set_config_req * configReq,
            uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_PROC_SET_CONFIG;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) configReq;
    reqParam.reqPayloadSize = (uint32_t) sizeof (struct tisci_msg_proc_set_config_req);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) 0;
    respParam.respPayloadSize = (uint32_t) 0;

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_procBootSetSequenceCtrl(uint8_t  processorId,
                                          uint32_t control_flags_1_set,
                                          uint32_t control_flags_1_clear,
                                          uint32_t reqFlag,
                                          uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_proc_set_control_req request ;
    request.processor_id         = (uint8_t ) processorId;
    request.control_flags_1_set  = (uint32_t) control_flags_1_set;
    request.control_flags_1_clear= (uint32_t) control_flags_1_clear;

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_PROC_SET_CONTROL;
    reqParam.flags          = (uint32_t) reqFlag;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) 0;
    respParam.respPayloadSize = (uint32_t) 0;

    if (((reqFlag & TISCI_MSG_FLAG_AOP) != TISCI_MSG_FLAG_AOP)&&
        (reqFlag != 0U))
    {
        retVal = SystemP_FAILURE;
    }
    if(retVal == SystemP_SUCCESS)
    {
        retVal = Sciclient_service(&reqParam, &respParam);
    }
    if((retVal != SystemP_SUCCESS) ||
        ((reqFlag != 0U) &&
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_procBootAuthAndStart(
            const struct tisci_msg_proc_auth_boot_req * authBootCfg,
            uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_proc_auth_boot_resp response;

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_PROC_AUTH_BOOT;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) authBootCfg;
    reqParam.reqPayloadSize = (uint32_t) sizeof (struct tisci_msg_proc_auth_boot_req);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) &response;
    respParam.respPayloadSize = (uint32_t) sizeof (response);

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_procBootGetProcessorState(
            uint8_t processorId,
            struct tisci_msg_proc_get_status_resp * procStatus,
            uint32_t  timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_proc_get_status_req request ;
    request.processor_id         = (uint8_t ) processorId;
    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_PROC_GET_STATUS;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;
    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) procStatus;
    respParam.respPayloadSize = (uint32_t) sizeof (struct tisci_msg_proc_get_status_resp);

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_procBootWaitProcessorState(uint8_t  processorId,
                                        uint8_t  num_match_iterations,
                                        uint8_t  delay_per_iteration_us,
                                        uint32_t status_flags_1_set_all_wait,
                                        uint32_t status_flags_1_set_any_wait,
                                        uint32_t status_flags_1_clr_all_wait,
                                        uint32_t status_flags_1_clr_any_wait,
                                        uint32_t reqFlag,
                                        uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;
    struct tisci_msg_proc_status_wait_req request ;
    request.processor_id         = (uint8_t ) processorId;
    request.num_wait_iterations = (uint8_t)255;
    request.num_match_iterations = (uint8_t)num_match_iterations;
    request.delay_per_iteration_us = (uint8_t)delay_per_iteration_us;
    request.delay_before_iteration_loop_start_us = (uint8_t)0;
    request.status_flags_1_set_all_wait = (uint32_t)status_flags_1_set_all_wait;
    request.status_flags_1_set_any_wait = (uint32_t)status_flags_1_set_any_wait;
    request.status_flags_1_clr_all_wait = (uint32_t)status_flags_1_clr_all_wait;
    request.status_flags_1_clr_any_wait = (uint32_t)status_flags_1_clr_any_wait;
    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_PROC_WAIT_STATUS;
    reqParam.flags          = (uint32_t) reqFlag;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;
    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) 0;
    respParam.respPayloadSize = (uint32_t) 0;

    if (((reqFlag & TISCI_MSG_FLAG_AOP) != TISCI_MSG_FLAG_AOP) &&
        (reqFlag != 0U))
    {
        retVal = SystemP_FAILURE;
    }
    if(retVal == SystemP_SUCCESS)
    {
        retVal = Sciclient_service(&reqParam, &respParam);
    }
    if((retVal != SystemP_SUCCESS) ||
        ((reqFlag != 0U) &&
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK)))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

/* None */

