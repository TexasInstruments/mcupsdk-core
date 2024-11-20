/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <board/ioexp/ioexp_tca6424.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/sciclient.h>
#include <drivers/sciclient/include/tisci/security/tisci_otp_revision.h>
#include "runtime_keyrev.h"
#include "dual_cert_keyrev.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

const uint8_t gKeyCert[KEYREV_CHANGE_CERT_SIZE_IN_BYTES] __attribute__((aligned(128)))
    = KEYREV_CHANGE_CERT;

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

int32_t runtime_keyrev_readKeyrevKeycnt(uint32_t *keyCountVal, uint32_t *keyRevVal)
{
    int32_t status = SystemP_SUCCESS;
    Sciclient_ReqPrm_t  reqParam;
    Sciclient_RespPrm_t respParam;

    struct tisci_msg_get_keycnt_keyrev_req request;
    struct tisci_msg_get_keycnt_keyrev_resp response;

    reqParam.messageType      = (uint16_t) TISCI_MSG_READ_KEYCNT_KEYREV;
    reqParam.flags            = (uint32_t)TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload      = (const uint8_t *)&request;
    reqParam.reqPayloadSize   = (uint32_t) sizeof (request);
    reqParam.timeout          = (uint32_t) SystemP_WAIT_FOREVER;

    respParam.flags           = 0U; /* API will populate */
    respParam.pRespPayload    = (uint8_t *)&response;
    respParam.respPayloadSize = (uint32_t) sizeof (response);

    status = Sciclient_service(&reqParam, &respParam);

    if((SystemP_SUCCESS == status) && ((respParam.flags & TISCI_MSG_FLAG_ACK) == TISCI_MSG_FLAG_ACK))
    {
        *keyCountVal = response.keycnt;
        *keyRevVal = response.keyrev;
        DebugP_logInfo("Success reading KEYREV and KEY Count value\r\n");
        DebugP_logInfo("KEYREV value reported : 0x%x \r\n", (uint32_t) *keyRevVal);
        DebugP_logInfo("KEY Count value reported : 0x%x \r\n", (uint32_t) *keyCountVal);
    }
    else
    {
        DebugP_logError("Error reading KEYREV and KEY Count !!! \r\n");
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t runtime_keyrev_writeKeyrev()
{
    int32_t status = SystemP_SUCCESS;
    Sciclient_ReqPrm_t  reqParam;
    Sciclient_RespPrm_t respParam;

    struct tisci_msg_set_keyrev_req  request;
    struct tisci_msg_set_keyrev_resp response;

    request.value              = 2;
    request.cert_addr_lo       = (uint32_t) ((uint64_t) gKeyCert & (uint64_t) 0xFFFFFFFF);
    request.cert_addr_hi       = 0;

    reqParam.messageType      = (uint16_t)TISCI_MSG_WRITE_KEYREV;
    reqParam.flags            = (uint32_t)TISCI_MSG_FLAG_AOP;
    reqParam.pReqPayload      = (const uint8_t *)&request;
    reqParam.reqPayloadSize   = (uint32_t) sizeof (request);
    reqParam.timeout          = (uint32_t) SystemP_WAIT_FOREVER;

    respParam.flags           = 0U; /* API will populate */
    respParam.pRespPayload    = (uint8_t *)&response;
    respParam.respPayloadSize = (uint32_t) sizeof (response);

    DebugP_log("Dual Certificate found: 0x%x \r\n", (uint32_t) gKeyCert);
    DebugP_log("Updating KEYREV value to 2 ...\r\n");

    status = Sciclient_service(&reqParam, &respParam);

    DebugP_log("KEYREV Writer Response : 0x%x \r\n", (uint32_t) respParam.flags);

    if((SystemP_SUCCESS == status) && ((respParam.flags & TISCI_MSG_FLAG_ACK) == TISCI_MSG_FLAG_ACK))
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void runtime_keyrev_setVpp(uint32_t ioIndex, uint32_t LED_ioIndex)
{
	int32_t status;

	TCA6424_Params TCA6424_IOexp_params =
	{
		.i2cInstance = 0,
		.i2cAddress = 0x22
	};

	TCA6424_Config TCA6424_IOexp_config;

    status = TCA6424_open(&TCA6424_IOexp_config, &TCA6424_IOexp_params);

	/* set VPP core */
    if (status == SystemP_SUCCESS)
	{
		status = TCA6424_config(&TCA6424_IOexp_config, ioIndex, TCA6424_MODE_OUTPUT);
	}

    if (status == SystemP_SUCCESS)
	{
    	status = TCA6424_setOutput(&TCA6424_IOexp_config, ioIndex, TCA6424_OUT_STATE_HIGH);
	}

	/* make onboard RED LED on indicating VPP is set. */
    if (status == SystemP_SUCCESS)
	{
		status = TCA6424_config(&TCA6424_IOexp_config, LED_ioIndex, TCA6424_MODE_OUTPUT);
	}

    if (status == SystemP_SUCCESS)
	{
    	status = TCA6424_setOutput(&TCA6424_IOexp_config, LED_ioIndex, TCA6424_OUT_STATE_HIGH);
	}

    TCA6424_close(&TCA6424_IOexp_config);

	DebugP_assertNoLog(status==SystemP_SUCCESS);
}
