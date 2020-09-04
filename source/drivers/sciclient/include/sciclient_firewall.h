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
 * \defgroup SCICLIENT_FMW_FIREWALL_IF Sciclient Firewall API Interface
 *
 * The DMSC firmware Firewall Management system manages SoC Firewall resources.
 * Each firewall in the system is assigned an owner. Ownership is assigned
 * using Host ID. Only a owner of a firewall can:
 * - Configure the firewall permissions
 * - Query the firewall permissions
 * - Transfer the firewall ownership to another owner(host)
 *
 * @{
 */
/**
 *  \file   sciclient_firewall.h
 *
 *  \brief  This file contains the definition of all the message IDs, message
 *          formats to be able to interact with the System Controller firmware
 *          for firewall management.
 */

#ifndef SCICLIENT_FWL_H_
#define SCICLIENT_FWL_H_

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
 *  \brief Request for configuring the firewall permissions.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_CHANGE_FWL_OWNER
 *  \n<b>Request</b>:    #tisci_msg_fwl_change_owner_info_req
 *  \n<b>Response</b>:   #tisci_msg_fwl_change_owner_info_resp
 *
 *  \param  req             Pointer to firewall change owner payload
 *
 *  \param  resp            Pointer to firewall change owner response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_firewallChangeOwnerInfo(
    const struct tisci_msg_fwl_change_owner_info_req *req,
    struct tisci_msg_fwl_change_owner_info_resp *resp,
    uint32_t timeout);

/**
 *  \brief Request for configuring the firewall permissions.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SET_FWL_REGION
 *  \n<b>Request</b>:    #tisci_msg_fwl_set_firewall_region_req
 *  \n<b>Response</b>:   #tisci_msg_fwl_set_firewall_region_resp
 *
 *  \param  req             Pointer to firewall region set payload
 *
 *  \param  resp            Pointer to firewall region set response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_firewallSetRegion(
    const struct tisci_msg_fwl_set_firewall_region_req *req,
    struct tisci_msg_fwl_set_firewall_region_resp *resp,
    uint32_t timeout);

/**
 *  \brief Request for getting the firewall permissions.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_GET_FWL_REGION
 *  \n<b>Request</b>:    #tisci_msg_fwl_get_firewall_region_req
 *  \n<b>Response</b>:   #tisci_msg_fwl_get_firewall_region_resp
 *
 *  \param  req             Pointer to firewall region get payload
 *
 *  \param  resp            Pointer to firewall region get response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_firewallGetRegion(
    const struct tisci_msg_fwl_get_firewall_region_req *req,
    struct tisci_msg_fwl_get_firewall_region_resp *resp,
    uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_FWL_H_ */

/** @} */
