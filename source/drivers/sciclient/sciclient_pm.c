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
 *  \file sciclient_pm.c
 *
 *  \brief File containing the SCICLIENT API interfaces to the Power management
 *         DMSC services.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/sciclient.h>
#include <kernel/dpl/SystemP.h>

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

int32_t Sciclient_pmSetModuleState(uint32_t moduleId,
                                   uint32_t state,
                                   uint32_t reqFlag,
                                   uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_set_device_req request;
    Sciclient_ReqPrm_t reqParam;

    Sciclient_RespPrm_t respParam;

    request.id       = (uint32_t) moduleId;
    request.reserved = (uint32_t) 0;
    request.state    = (uint8_t) state;

    reqParam.messageType    = (uint16_t) TISCI_MSG_SET_DEVICE;
    reqParam.flags          = (uint32_t) reqFlag;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

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

int32_t Sciclient_pmGetModuleState(uint32_t  moduleId,
                                   uint32_t *moduleState,
                                   uint32_t *resetState,
                                   uint32_t *contextLossState,
                                   uint32_t  timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_get_device_req request;
    struct tisci_msg_get_device_resp response = {{0}};
    Sciclient_ReqPrm_t reqParam;

    Sciclient_RespPrm_t respParam;

    request.id = (uint32_t) moduleId;

    reqParam.messageType    = (uint16_t) TISCI_MSG_GET_DEVICE;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) &response;
    respParam.respPayloadSize = (uint32_t) sizeof (response);

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    if (retVal == SystemP_SUCCESS)
    {
        *moduleState = (uint32_t) response.current_state;
        *resetState = (uint32_t) response.resets;
        *contextLossState = (uint32_t) response.context_loss_count;
    }
    return retVal;
}

int32_t Sciclient_pmSetModuleRst(uint32_t moduleId,
                                 uint32_t resetBit,
                                 uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;
    struct tisci_msg_set_device_resets_req request;
    request.id     = (uint32_t) moduleId;
    request.resets = (uint32_t) resetBit;

    Sciclient_ReqPrm_t reqParam;
    reqParam.messageType    = (uint16_t) TISCI_MSG_SET_DEVICE_RESETS;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam;
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

int32_t Sciclient_pmModuleClkRequest(uint32_t moduleId,
                                     uint32_t clockId,
                                     uint32_t state,
                                     uint32_t additionalFlag,
                                     uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;
    struct tisci_msg_set_clock_req request ;
    request.device = (uint32_t) moduleId;

    if(clockId > (uint32_t)(255U))
    {
        request.clk = (uint8_t)(255U);
        request.clk32 = clockId;
    }
    else
    {
        request.clk    = (uint8_t)clockId;
    }
    request.state  = (uint8_t) state;

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_SET_CLOCK;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP | additionalFlag;
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

int32_t Sciclient_pmModuleGetClkStatus(uint32_t  moduleId,
                                       uint32_t  clockId,
                                       uint32_t *state,
                                       uint32_t  timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_get_clock_req request ;
    request.device = (uint32_t) moduleId;

    /* If clockId is larger than 8-bit max, we use clk32 to specify the
     * extended clock id for the device */
    if (clockId >= 255U)
    {
        request.clk32 = clockId;
        request.clk   = (uint8_t) 255U;
    }
    else
    {
        request.clk    = (uint8_t) clockId;
    }

    struct tisci_msg_get_clock_resp response = {{0}};
    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_GET_CLOCK;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
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
    if (retVal == SystemP_SUCCESS)
    {
        *state = (uint32_t) response.current_state;
    }
    return retVal;
}

int32_t Sciclient_pmSetModuleClkParent(uint32_t moduleId,
                                       uint32_t clockId,
                                       uint32_t parent,
                                       uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_set_clock_parent_req request ;
    request.device = (uint32_t) moduleId;

    /* If clockId is larger than 8-bit max, we use clk32 to specify the
     * extended clock id for the device */
    if (clockId >= 255U)
    {
        request.clk32 = clockId;
        request.clk   = (uint8_t) 255U;
    }
    else
    {
        request.clk    = (uint8_t) clockId;
    }

    /* If parent is larger than 8-bit max, we use parent32 to specify the
     * extended parent id for the clock */
    if (parent >= 255U)
    {
        request.parent32 = parent;
        request.parent   = (uint8_t) 255U;
    }
    else
    {
        request.parent = (uint8_t) parent;
    }

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_SET_CLOCK_PARENT;
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

int32_t Sciclient_pmGetModuleClkParent(uint32_t  moduleId,
                                       uint32_t  clockId,
                                       uint32_t *parent,
                                       uint32_t  timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_get_clock_parent_req request;
    request.device = (uint32_t) moduleId;

    /* If clockId is larger than 8-bit max, we use clk32 to specify the
     * extended clock id for the device */
    if (clockId >= 255U)
    {
        request.clk32 = clockId;
        request.clk   = (uint8_t) 255U;
    }
    else
    {
        request.clk    = (uint8_t) clockId;
    }

    struct tisci_msg_get_clock_parent_resp response = {{0}};
    Sciclient_ReqPrm_t reqParam;
    reqParam.messageType    = (uint16_t) TISCI_MSG_GET_CLOCK_PARENT;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) &response;
    respParam.respPayloadSize = (uint32_t) sizeof (response);


    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    if (retVal == SystemP_SUCCESS)
    {
        *parent = (uint32_t) response.parent;
    }
    return retVal;
}

int32_t Sciclient_pmGetModuleClkNumParent(uint32_t  moduleId,
                                          uint32_t  clockId,
                                          uint32_t *numParent,
                                          uint32_t  timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_get_num_clock_parents_req request = {{0}};
    request.device = (uint32_t) moduleId;

    /* If clockId is larger than 8-bit max, we use clk32 to specify the
     * extended clock id for the device */
    if (clockId >= 255U)
    {
        request.clk32 = clockId;
        request.clk   = (uint8_t) 255U;
    }
    else
    {
        request.clk    = (uint8_t) clockId;
    }

    struct tisci_msg_get_num_clock_parents_resp response = {{0}};
    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_GET_NUM_CLOCK_PARENTS;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
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
    if (retVal == SystemP_SUCCESS)
    {
        *numParent = (uint32_t) response.num_parents;
    }
    return retVal;
}

int32_t Sciclient_pmSetModuleClkFreq(uint32_t moduleId,
                                     uint32_t clockId,
                                     uint64_t freqHz,
                                     uint32_t additionalFlag,
                                     uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_set_freq_req request ;
    request.device         = (uint32_t) moduleId;
    request.min_freq_hz    = (uint64_t) freqHz;
    request.target_freq_hz = (uint64_t) freqHz;
    request.max_freq_hz    = (uint64_t) freqHz;
    /* This is required in cases of fractional clocks with a recurring decimal */
    if((freqHz%10U) != 0U)
    {
        request.min_freq_hz = (uint64_t)(freqHz-(freqHz%10U));
        request.max_freq_hz = (uint64_t)(request.min_freq_hz + 10UL);
    }

    /* If clockId is larger than 8-bit max, we use clk32 to specify the
     * extended clock id for the device */
    if (clockId >= 255U)
    {
        request.clk32 = clockId;
        request.clk   = (uint8_t) 255U;
    }
    else
    {
        request.clk    = (uint8_t) clockId;
    }

    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_SET_FREQ;
    reqParam.flags          = (uint32_t)(TISCI_MSG_FLAG_AOP | additionalFlag);
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

int32_t Sciclient_pmQueryModuleClkFreq(uint32_t moduleId,
                                       uint32_t clockId,
                                       uint64_t reqFreqHz,
                                       uint64_t *respFreqHz,
                                       uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_query_freq_req request ;
    request.device         = (uint32_t) moduleId;
    request.min_freq_hz    = (uint64_t) reqFreqHz;
    request.target_freq_hz = (uint64_t) reqFreqHz;
    request.max_freq_hz    = (uint64_t) reqFreqHz;
    /* This is required in cases of fractional clocks with a recurring decimal */
    if((reqFreqHz%10U) != 0U)
    {
        request.min_freq_hz = (uint64_t)(reqFreqHz-(reqFreqHz%10U));
        request.max_freq_hz = (uint64_t)(request.min_freq_hz + 10UL);
    }

    /* If clockId is larger than 8-bit max, we use clk32 to specify the
     * extended clock id for the device */
    if (clockId >= 255U)
    {
        request.clk32 = clockId;
        request.clk   = (uint8_t) 255U;
    }
    else
    {
        request.clk    = (uint8_t) clockId;
    }

    struct tisci_msg_query_freq_resp response = {{0}};
    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_QUERY_FREQ;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
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
    if (retVal == SystemP_SUCCESS)
    {
        *respFreqHz = (uint64_t) response.freq_hz;
    }
    return retVal;
}

int32_t Sciclient_pmGetModuleClkFreq(uint32_t  moduleId,
                                     uint32_t  clockId,
                                     uint64_t *freqHz,
                                     uint32_t  timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_get_freq_req request ;
    request.device = (uint32_t) moduleId;

    /* If clockId is larger than 8-bit max, we use clk32 to specify the
     * extended clock id for the device */
    if (clockId >= 255U)
    {
        request.clk32 = clockId;
        request.clk   = (uint8_t) 255U;
    }
    else
    {
        request.clk    = (uint8_t) clockId;
    }

    struct tisci_msg_get_freq_resp response = {{0}};
    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_GET_FREQ;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
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
    if (retVal == SystemP_SUCCESS)
    {
        *freqHz = (uint64_t) response.freq_hz;
    }
    return retVal;
}

int32_t Sciclient_pmEnableWdt(uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;
    Sciclient_ReqPrm_t reqParam ;
    struct tisci_msg_enable_wdt_req request = {0};
    struct tisci_msg_enable_wdt_resp response = {0};
    reqParam.messageType    = (uint16_t) TISCI_MSG_ENABLE_WDT;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof(request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) &response;
    respParam.respPayloadSize = (uint32_t) sizeof(response);

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_pmDisableWakeup(uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    Sciclient_ReqPrm_t reqParam;
    struct tisci_msg_wake_reset_req request = {0};
    struct tisci_msg_wake_reset_resp response = {0};
    reqParam.messageType    = (uint16_t) TISCI_MSG_WAKE_RESET;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof(request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) &response;
    respParam.respPayloadSize = (uint32_t) sizeof(response);

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_pmGetWakeupReason(uint8_t   mode[32],
                                    uint8_t   reason[32],
                                    uint32_t *time_ms,
                                    uint32_t  timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_wake_reason_resp response = {{0}};
    struct tisci_msg_wake_reason_req request = {0};
    Sciclient_ReqPrm_t reqParam ;
    reqParam.messageType    = (uint16_t) TISCI_MSG_WAKE_REASON;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof(request);
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
    if (retVal == SystemP_SUCCESS)
    {
        (void) memcpy((void *)mode, (void *)response.mode, sizeof (response.mode));
        (void) memcpy((void *)reason, (void *)response.reason, sizeof (response.reason));
        *time_ms = (uint32_t)(uintptr_t)time_ms;
    }
    return retVal;
}

int32_t Sciclient_pmDevicePowerOff(uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    Sciclient_ReqPrm_t reqParam;
    struct tisci_msg_goodbye_req request = {0};
    struct tisci_msg_goodbye_resp response = {0};
    reqParam.messageType    = (uint16_t) TISCI_MSG_GOODBYE;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof(request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) &response;
    respParam.respPayloadSize = (uint32_t) sizeof(response);


    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_pmDeviceReset(uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;

    Sciclient_ReqPrm_t reqParam ;
    struct tisci_msg_sys_reset_req request = {{0}};
    struct tisci_msg_sys_reset_resp response = {0};
    reqParam.messageType    = (uint16_t) TISCI_MSG_SYS_RESET;
    reqParam.flags          = (uint32_t) TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof(request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam ;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) &response;
    respParam.respPayloadSize = (uint32_t) sizeof(response);

    retVal = Sciclient_service(&reqParam, &respParam);
    if((retVal != SystemP_SUCCESS) ||
        ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

int32_t Sciclient_pmIsModuleValid(uint32_t modId)
{
   int32_t retVal = SystemP_SUCCESS;
   return retVal;
}

int32_t Sciclient_pmSetModuleRst_flags(uint32_t moduleId,
                                 uint32_t resetBit,
                                 uint32_t reqFlag,
                                 uint32_t timeout)
{
    int32_t retVal = SystemP_SUCCESS;
    struct tisci_msg_set_device_resets_req request;
    request.id     = (uint32_t) moduleId;
    request.resets = (uint32_t) resetBit;

    Sciclient_ReqPrm_t reqParam;
    reqParam.messageType    = (uint16_t) TISCI_MSG_SET_DEVICE_RESETS;
    reqParam.flags          = (uint32_t) reqFlag;
    reqParam.pReqPayload    = (const uint8_t *) &request;
    reqParam.reqPayloadSize = (uint32_t) sizeof (request);
    reqParam.timeout        = (uint32_t) timeout;

    Sciclient_RespPrm_t respParam;
    respParam.flags           = (uint32_t) 0;   /* Populated by the API */
    respParam.pRespPayload    = (uint8_t *) 0;
    respParam.respPayloadSize = (uint32_t) 0;

    retVal = Sciclient_service(&reqParam, &respParam);

    if((retVal != SystemP_SUCCESS) ||
        ( reqFlag != 0 && (respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

/* None */

