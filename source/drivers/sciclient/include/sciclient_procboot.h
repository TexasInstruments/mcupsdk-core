/*
 *  Copyright (C) 2018 Texas Instruments Incorporated
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
 * \defgroup SCICLIENT_FMW_PROCBOOT_IF Sciclient Processor Boot API Interface
 *
 * DMSC controls the power management of the device, hence is responsible for
 * bringing the device out of reset, enforce clock and reset rules. DMSC power
 * management functions are critical to bring device to low power modes, for
 * example DeepSleep, and sense wake-up events to bring device back online to
 * active state.
 *
 *
 * @{
 */
/**
 *  \file   sciclient_procboot.h
 *
 *  \brief  This file contains the definition of all the message IDs, message
 *          formats to be able to interact with the System Controller firmware
 *          for processor boot and control.
 *          APIs are divided into the following two sets:
 *          * Book keeping APIs - Meant to control access to allow a
 *            reasonable usage scenario of processors.
 *          * Processor Control APIs - Meant to be the actual Processor Core
 *           controls.
 *
 */

#ifndef SCICLIENT_PROCBOOT_H_
#define SCICLIENT_PROCBOOT_H_

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
 *  \brief    Message to request a processor
 *  Provides a means for either the “recovery master” host or another host in
 *  the permitted access list to request for a physical processor control.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_PROC_REQUEST
 *  \n<b>Request</b>:    #tisci_msg_proc_request_req
 *  \n<b>Response</b>:   #tisci_msg_proc_request_resp
 *
 *  \param  processorId     Index of the processor
 *                          Refer \ref Sciclient_ProcessorIds.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_procBootRequestProcessor(uint8_t processorId,
                                           uint32_t timeout);
/**
 *  \brief    Message to release a processor
 *  Provides a means for the host with current control to relinquish a
 *  physical processor control.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_PROC_RELEASE
 *  \n<b>Request</b>:    #tisci_msg_proc_release_req
 *  \n<b>Response</b>:   #tisci_msg_proc_release_resp
 *
 *  \param  processorId     Index of the processor
 *                          Refer \ref Sciclient_ProcessorIds.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \param  reqFlag         Can be TISCI_MSG_FLAG_AOR/TISCI_MSG_FLAG_AOP
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_procBootReleaseProcessor(uint8_t  processorId,
                                           uint32_t reqFlag,
                                           uint32_t  timeout);

/**
 *  \brief    Message to handover a processor
 *  Provides a means for the host with current control to relinquish a
 *  physical processor control to another host in the permitted list.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_PROC_HANDOVER
 *  \n<b>Request</b>:    #tisci_msg_proc_handover_req
 *  \n<b>Response</b>:   #tisci_msg_proc_handover_resp
 *
 *  \param  processorId     Index of the processor
 *                          Refer \ref Sciclient_ProcessorIds.
 *  \param  hostId          Index of the Host
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_procBootHandoverProcessor(uint8_t  processorId,
                                            uint8_t  hostId,
                                            uint32_t timeout);
/**
 *  \brief    Message to set the processor configuration
 *  Provides a means for the host with current control to do the base
 *  configuration of the processor.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_PROC_SET_CONFIG
 *  \n<b>Request</b>:    #tisci_msg_proc_set_config_req
 *  \n<b>Response</b>:   #tisci_msg_proc_set_config_resp
 *
 *  \param  configReq       Configuration parameters for the processor.
 *                          Refer #tisci_msg_proc_set_config_req
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_procBootSetProcessorCfg (
            const struct tisci_msg_proc_set_config_req * configReq,
            uint32_t timeout);

/**
 *  \brief    Message to setup the processor sequence control.
 *  Provides a means for the host with current control to setup limited control
 *  flags in specific cases.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_PROC_SET_CONTROL
 *  \n<b>Request</b>:    #tisci_msg_proc_set_control_req
 *  \n<b>Response</b>:   #tisci_msg_proc_set_control_resp
 *
 *  \param  processorId         Index of the processor
 *                              Refer \ref Sciclient_ProcessorIds.
 *  \param  control_flags_1_set Optional Processor specific Control Flags to set.
 *                              Setting a bit here implies required bit has to be
 *                              set to 1.
 *  \param  control_flags_1_clear Optional Processor specific Control Flags to clear.
 *                                Setting a bit here implies required bit has to be
 *                                cleared to 0.
 *  \param  reqFlag             Can be TISCI_MSG_FLAG_AOR/TISCI_MSG_FLAG_AOP ORRed
 *                              with additional flag that can be set to alter the
 *                              device state.
 *  \param  timeout             Gives a sense of how long to wait for the operation.
 *                              Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_procBootSetSequenceCtrl(uint8_t  processorId,
                                          uint32_t control_flags_1_set,
                                          uint32_t control_flags_1_clear,
                                          uint32_t reqFlag,
                                          uint32_t timeout);

/**
 *  \brief    Message to Authorize and Start Processor Boot:
 *  Provides a means for the host with current control to do the following:
 *  - Authenticate and load a binary using the certificate provided information
 *  - Use certificate information also to setup critical processor specific flags
 *
 *  \n<b>Message</b>:    #TISCI_MSG_PROC_AUTH_BOOT
 *  \n<b>Request</b>:    #tisci_msg_proc_auth_boot_req
 *  \n<b>Response</b>:   #tisci_msg_proc_auth_boot_resp
 *
 *  \param  authBootCfg     Configuration Structure for the Authentication and boot
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_procBootAuthAndStart(
            const struct tisci_msg_proc_auth_boot_req * authBootCfg,
            uint32_t timeout);

/**
 *  \brief    Message to Get Processor Status
 *  Provides a means for hosts in the permitted list to get the status of a
 *  physical processor. This is required for the hosts to sequence events in
 *  the correct order
 *
 *  \n<b>Message</b>:    #TISCI_MSG_PROC_GET_STATUS
 *  \n<b>Request</b>:    #tisci_msg_proc_get_status_req
 *  \n<b>Response</b>:   #tisci_msg_proc_get_status_resp
 *
 *  \param  processorId     Index of the processor
 *                          Refer \ref Sciclient_ProcessorIds.
 *  \param  procStatus      Processor Status returned as given by
 *                          #tisci_msg_proc_get_status_resp
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_procBootGetProcessorState(
            uint8_t processorId,
            struct tisci_msg_proc_get_status_resp * procStatus,
            uint32_t  timeout);

/**
 *  \brief    Message to Wait for Processor Status
 *  Provides a means for hosts to request the SYSFW to wait till a
 *  a processor status matches a user defined value.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_PROC_WAIT_STATUS
 *  \n<b>Request</b>:    #tisci_msg_proc_status_wait_req
 *  \n<b>Response</b>:   #tisci_msg_proc_status_wait_resp
 *
 *  \param  processorId     Index of the processor
 *                          Refer \ref Sciclient_ProcessorIds.
 *  \param  num_match_iterations Must be between 1 - 255. Specifies number of
 *                               interations the status bit must be stable,
 *                               before it is used by API. This is to account
 *                               for glitches. For eg. specifying a value of
 *                               10 would mean that the bit must be stable for
 *                               10 iterations before the SYSFW API would
 *                               report to the host  that the bit has changed.
 *  \param  delay_per_iteration_us      Must be between 0 - 255.Specifies how
 *                                      long to wait (in micro seconds) between
 *                                      each status check.
 *  \param  status_flags_1_set_all_wait Optional Processor specific status
 *                                      Flags to wait on. Setting a bit implies
 *                                      that the SYFW will wait until all bits
 *                                      of the status matching this field
 *                                      become '1'.
 *  \param  status_flags_1_set_any_wait Optional Processor specific status
 *                                      Flags to wait on. Setting a bit implies
 *                                      that the SYFW will wait until any bit
 *                                      of the status matching this field
 *                                      becomes '1'.
 *  \param  status_flags_1_clr_all_wait Optional Processor specific status
 *                                      Flags to wait on. Setting a bit implies
 *                                      that the SYFW will wait until all bits
 *                                      of the status matching this field
 *                                      become '0'.
 *  \param  status_flags_1_clr_any_wait Optional Processor specific status
 *                                      Flags to wait on. Setting a bit implies
 *                                      that the SYFW will wait until any bit
 *                                      of the status matching this field
 *                                      becomes '0'.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \param  reqFlag         Can be TISCI_MSG_FLAG_AOR/TISCI_MSG_FLAG_AOP
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_procBootWaitProcessorState(
            uint8_t  processorId,
            uint8_t  num_match_iterations,
            uint8_t  delay_per_iteration_us,
            uint32_t status_flags_1_set_all_wait,
            uint32_t status_flags_1_set_any_wait,
            uint32_t status_flags_1_clr_all_wait,
            uint32_t status_flags_1_clr_any_wait,
            uint32_t reqFlag,
            uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_PROCBOOT_H_ */

/** @} */
