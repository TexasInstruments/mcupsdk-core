/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
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
 *
 */
/**
 * \ingroup DRV_SCICLIENT_MODULE
 * \defgroup SCICLIENT_FMW_DKEK_IF Sciclient DKEK API Interface
 *
 * K3 HS devices have a randomly generated 256 bit key written into the efuses
 * in TI Factory. This key is called a Key Encryption Key (KEK) and is unique to
 * each device. The key is only accessible via an AES engine which is controlled
 * by the DMSC. System controller firmware uses CMAC as the Pseudo Random
 * Function(PRF) in counter mode to derive a new KEK, called DKEK. The DKEK can
 * be accessed in the following ways:
 *
 * - Indirectly by having DMSC program the DKEK into the SA2UL registers and
 *   accessing through the USE_DKEK flag in the security context.
 *
 * - Directly by having DMSC derive the key and return the key value over the
 *   secure queue for CPU-based encryption routines.
 *
 * @{
 */
/**
 *  \file   sciclient_dkek.h
 *
 *  \brief  This file contains the definition of all the message IDs, message
 *          formats to be able to interact with the System Controller firmware
 *          for DKEK management.
 */

#ifndef SCICLIENT_DKEK_H_
#define SCICLIENT_DKEK_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/**
 *  \brief Request to derive a KEK and set SA2UL DKEK register
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SA2UL_SET_DKEK
 *  \n<b>Request</b>:    #tisci_msg_sa2ul_set_dkek_req
 *  \n<b>Response</b>:   #tisci_msg_sa2ul_set_dkek_resp
 *
 *  \param  req             Pointer to DKEK set request payload
 *
 *  \param  resp            Pointer to DKEK set response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_setDKEK(
    const struct tisci_msg_sa2ul_set_dkek_req *req,
    struct tisci_msg_sa2ul_set_dkek_resp *resp,
    uint32_t timeout);

/**
 *  \brief Request to erase the DKEK register
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SA2UL_RELEASE_DKEK
 *  \n<b>Request</b>:    #tisci_msg_sa2ul_release_dkek_req
 *  \n<b>Response</b>:   #tisci_msg_sa2ul_release_dkek_resp
 *
 *  \param  req             Pointer to DKEK release request payload
 *
 *  \param  resp            Pointer to DKEK release response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_releaseDKEK(
    const struct tisci_msg_sa2ul_release_dkek_req *req,
    struct tisci_msg_sa2ul_release_dkek_resp *resp,
    uint32_t timeout);

/**
 *  \brief Request for getting the firewall permissions.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SA2UL_GET_DKEK
 *  \n<b>Request</b>:    #tisci_msg_sa2ul_release_dkek_req
 *  \n<b>Response</b>:   #tisci_msg_sa2ul_release_dkek_resp
 *
 *  \param  req             Pointer to DKEK get request payload
 *
 *  \param  resp            Pointer to DKEK get response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_getDKEK(
    const struct tisci_msg_sa2ul_get_dkek_req *req,
    struct tisci_msg_sa2ul_get_dkek_resp *resp,
    uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_DKEK_H_ */

/** @} */
