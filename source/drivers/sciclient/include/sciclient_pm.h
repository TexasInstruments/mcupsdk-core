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
 * \defgroup SCICLIENT_FMW_PM_IF Sciclient PM API Interface
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
 *  \file   sciclient_pm.h
 *
 *  \brief  This file contains the definition of all the message IDs, message
 *          formats to be able to interact with the System Controller firmware
 *          for power management.
 */

#ifndef SCICLIENT_PM_H_
#define SCICLIENT_PM_H_

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
 *  \brief    Message to set the hardware block/module state
 *  This is used to request or release a device. For example: When the device
 *  is requested for operation, state is set to MSG_DEVICE_SW_STATE_ON. When
 *  the usage of the device is complete and released, the same request with
 *  state set as MSG_DEVICE_SW_STATE_AUTO_OFF is invoked. Based on exclusive
 *  access request, multiple processing entities can share a specific
 *  hardware block, however, this must be carefully used keeping the full
 *  system view in mind.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SET_DEVICE
 *  \n<b>Request</b>:    #tisci_msg_set_device_req
 *  \n<b>Response</b>:   #tisci_msg_set_device_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer @ref Sciclient_PmDeviceIds.
 *  \param  state           Module State requested.
 *  \param  reqFlag         Can be TISCI_MSG_FLAG_AOR/TISCI_MSG_FLAG_AOP ORRed
 *                          with additional flag that can be set to alter the
 *                          device state.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmSetModuleState(uint32_t moduleId,
                                   uint32_t state,
                                   uint32_t reqFlag,
                                   uint32_t timeout);
/**
 *  \brief    Message to get the hardware block/Module state.
 *  This request does not require the processing entity to have control of the
 *  device via a set device state request.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_GET_DEVICE
 *  \n<b>Request</b>:    #tisci_msg_get_device_req
 *  \n<b>Response</b>:   #tisci_msg_get_device_resp
 *
 *  \param  moduleId         Module for which the state should be set.
 *                           Refer \ref Sciclient_PmDeviceIds.
 *  \param  moduleState      Module State returned.
 *  \param  resetState       Programmed state of the reset lines.
 *  \param  contextLossState Indicates how many times the device has lost
 *                           context. A driver can use this monotonic counter
 *                           to determine if the device has lost context since
 *                           the last time this message was exchanged.
 *  \param  timeout          Gives a sense of how long to wait for the
 *                           operation. Refer
 *                           \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmGetModuleState(uint32_t  moduleId,
                                   uint32_t *moduleState,
                                   uint32_t *resetState,
                                   uint32_t *contextLossState,
                                   uint32_t  timeout);
/**
 *  \brief    Set the device reset state.
 *  This is used to set or release various resets of the hardware block/module
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SET_DEVICE_RESETS
 *  \n<b>Request</b>:    #tisci_msg_set_device_resets_req
 *  \n<b>Response</b>:   #tisci_msg_set_device_resets_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *  \param  resetBit        Module Reset Bit to be set.
 *                          1 - Assert the reset
 *                          0 - Deassert the reset
 *                          Note this convention is opposite of PSC MDCTL
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmSetModuleRst(uint32_t moduleId,
                                 uint32_t resetBit,
                                 uint32_t timeout);

/**
 *  \brief    Set the device reset state, allowing for user to set reqFlags
 *  This is used to set or release various resets of the hardware block/module
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SET_DEVICE_RESETS
 *  \n<b>Request</b>:    #tisci_msg_set_device_resets_req
 *  \n<b>Response</b>:   #tisci_msg_set_device_resets_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *  \param  resetBit        Module Reset Bit to be set.
 *                          1 - Assert the reset
 *                          0 - Deassert the reset
 *                          Note this convention is opposite of PSC MDCTL
 *  \param  reqFlag         Can be TISCI_MSG_FLAG_AOR/TISCI_MSG_FLAG_AOP ORRed
 *                          with additional flag that can be set to alter the
 *                          device state.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmSetModuleRst_flags(uint32_t moduleId,
                                       uint32_t resetBit,
                                       uint32_t reqFlag,
                                       uint32_t timeout);
/**
 *  \brief    Message to set the clock state:
 *  This requests for finer control of hardware device's clocks. This allows
 *  for configuration for hardware blocks that require customization of the
 *  specific input clocks. NOTE: each of the clock IDs are relative to the
 *  hardware block.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SET_CLOCK
 *  \n<b>Request</b>:    #tisci_msg_set_clock_req
 *  \n<b>Response</b>:   #tisci_msg_set_clock_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *  \param  clockId         Clock Id for the module.
 *  \param  state           Clock State requested.
 *  \param  additionalFlag  Certain flags can also be set to alter the clock
 *                          state.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmModuleClkRequest(uint32_t moduleId,
                                     uint32_t clockId,
                                     uint32_t state,
                                     uint32_t additionalFlag,
                                     uint32_t timeout);
/**
 *  \brief    Message to get the clock state to or from a hardware block
 *
 *  \n<b>Message</b>:    #TISCI_MSG_GET_CLOCK
 *  \n<b>Request</b>:    #tisci_msg_get_clock_req
 *  \n<b>Response</b>:   #tisci_msg_get_clock_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  state           Clock State returned.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmModuleGetClkStatus(uint32_t  moduleId,
                                       uint32_t  clockId,
                                       uint32_t *state,
                                       uint32_t  timeout);

/**
 *  \brief    Message to Set Clock Parent:
 *  This message allows SoC specific customization for setting up a specific
 *  clock parent ID for the various clock input options for a hardware
 *  block's clock.
 *  This is rarely used customization that may be required based on the usecase
 *  of the system where the reset input clock option may not suffice for the
 *  usecase attempted.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SET_CLOCK_PARENT
 *  \n<b>Request</b>:    #tisci_msg_set_clock_parent_req
 *  \n<b>Response</b>:   #tisci_msg_set_clock_parent_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  parent          Parent Id for the clock.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmSetModuleClkParent(uint32_t moduleId,
                                       uint32_t clockId,
                                       uint32_t parent,
                                       uint32_t timeout);
/**
 *  \brief    Message to Get Clock Parent:
 *  Query the clock parent currently configured for a specific clock source of
 *  a hardware block
 *  This is typically used to confirm the current clock parent to ensure that
 *  the requisite usecase for the hardware block can be satisfied.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_GET_CLOCK_PARENT
 *  \n<b>Request</b>:    #tisci_msg_get_clock_parent_req
 *  \n<b>Response</b>:   #tisci_msg_get_clock_parent_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  parent          Returned Parent Id for the clock.s
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmGetModuleClkParent(uint32_t  moduleId,
                                       uint32_t  clockId,
                                       uint32_t *parent,
                                       uint32_t  timeout);
/**
 *  \brief    Message to get the number of clock parents for a given module.
 *  This is typically used to get the max number of clock parent options
 *  available for a specific hardware block's clock.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_GET_NUM_CLOCK_PARENTS
 *  \n<b>Request</b>:    #tisci_msg_get_num_clock_parents_req
 *  \n<b>Response</b>:   #tisci_msg_get_num_clock_parents_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  numParent       Returned number of parents.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmGetModuleClkNumParent(uint32_t  moduleId,
                                          uint32_t  clockId,
                                          uint32_t *numParent,
                                          uint32_t  timeout);
/**
 *  \brief    Message to set the clock frequency.
 *  This is typically desired when the default frequency of the hardware
 *  block's clock is not appropriate for the usecase desired.
 *  NOTE: Normally clock frequency management is automatically done by TISCI
 *  entity. In case of specific requests, TISCI evaluates capability to achieve
 *  requested range and responds with success/failure message.
 *  This sets the desired frequency for a clock within an allowable range.
 *  This message will fail on an enabled clock unless
 *  MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE is set for the clock. Additionally,
 *  if other clocks have their frequency modified due to this message,
 *  they also must have the MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE or be disabled.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SET_FREQ
 *  \n<b>Request</b>:    #tisci_msg_set_freq_req
 *  \n<b>Response</b>:   #tisci_msg_set_freq_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  freqHz          Frequency of the clock in Hertz.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \param  additionalFlag  Additional flags for the request.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmSetModuleClkFreq(uint32_t moduleId,
                                     uint32_t clockId,
                                     uint64_t freqHz,
                                     uint32_t additionalFlag,
                                     uint32_t timeout);
/**
 *  \brief    Message to set the clock frequency.
 *  This is typically desired when the default frequency of the hardware
 *  block's clock is not appropriate for the usecase desired.
 *  NOTE: Normally clock frequency management is automatically done by TISCI
 *  entity. In case of specific requests, TISCI evaluates capability to achieve
 *  requested range and responds with success/failure message.
 *  This sets the desired frequency for a clock within an allowable range.
 *  This message will fail on an enabled clock unless
 *  MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE is set for the clock. Additionally,
 *  if other clocks have their frequency modified due to this message,
 *  they also must have the MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE or be disabled.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SET_FREQ
 *  \n<b>Request</b>:    #tisci_msg_set_freq_req
 *  \n<b>Response</b>:   #tisci_msg_set_freq_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  freqHz          Frequency of the clock in Hertz.
 *  \param  minFreqHz       Frequency of the clock in Hertz(Min Limit).
 *  \param  maxFreqHz       Frequency of the clock in Hertz(Max Limit).
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \param  additionalFlag  Additional flags for the request.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmSetModuleClkFreqRange(uint32_t moduleId,
                                          uint32_t clockId,
                                          uint64_t freqHz,
                                          uint64_t minFreqHz,
                                          uint64_t maxFreqHz,
                                          uint32_t additionalFlag,
                                          uint32_t timeout);
/**
 *  \brief    Message to query the best clock frequency in the given range.
 *  This message does no real operation, instead, it requests the system
 *  control entity to respond with the best frequency that can match a
 *  frequency range provided. NOTE: This is a snapshot view. In a multi
 *  processing system, it is very well possible that another processing
 *  entity might change the configuration after one entity has queried for
 *  best match capability. Only a SET_CLOCK_FREQ will guarantee the frequency
 *  is configured.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_QUERY_FREQ
 *  \n<b>Request</b>:    #tisci_msg_query_freq_req
 *  \n<b>Response</b>:   #tisci_msg_query_freq_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  reqFreqHz       The target clock frequency.
 *  \param  respFreqHz      The frequency that the hardware would set for the
 *                          given parameters.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmQueryModuleClkFreq(uint32_t moduleId,
                                       uint32_t clockId,
                                       uint64_t reqFreqHz,
                                       uint64_t *respFreqHz,
                                       uint32_t timeout);
/**
 *  \brief    Message to query the best clock frequency in the given range.
 *  This message does no real operation, instead, it requests the system
 *  control entity to respond with the best frequency that can match a
 *  frequency range provided. NOTE: This is a snapshot view. In a multi
 *  processing system, it is very well possible that another processing
 *  entity might change the configuration after one entity has queried for
 *  best match capability. Only a SET_CLOCK_FREQ will guarantee the frequency
 *  is configured.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_QUERY_FREQ
 *  \n<b>Request</b>:    #tisci_msg_query_freq_req
 *  \n<b>Response</b>:   #tisci_msg_query_freq_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  reqFreqHz       The target clock frequency.
 *  \param  minFreqHz       The target clock frequency(Min Limit)
 *  \param  maxFreqHz       The target clock frequency(Max Limit)
 *  \param  respFreqHz      The frequency that the hardware would set for the
 *                          given parameters.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmQueryModuleClkFreqRange(uint32_t moduleId,
                                           uint32_t clockId,
                                           uint64_t reqFreqHz,
                                           uint64_t minFreqHz,
                                           uint64_t maxFreqHz,
                                           uint64_t *respFreqHz,
                                           uint32_t timeout);
/**
 *  \brief    Message to Get Clock Frequency
 *  This is most used functionality and is meant for usage when the driver
 *  controlling the hardware block requires to know the input clock frequency
 *  for configuring internal dividers / multipliers as required.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_GET_FREQ
 *  \n<b>Request</b>:    #tisci_msg_get_freq_req
 *  \n<b>Response</b>:   #tisci_msg_get_freq_resp
 *
 *  \param  moduleId        Module for which the state should be set.
 *                          Refer \ref Sciclient_PmDeviceIds.
 *  \param  clockId         Clock Id for the module.
 *                          Refer \ref Sciclient_PmModuleClockIds.
 *  \param  freqHz          Frequency of the clock returned in Hertz.
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmGetModuleClkFreq(uint32_t  moduleId,
                                     uint32_t  clockId,
                                     uint64_t *freqHz,
                                     uint32_t  timeout);
/**
 *  \brief    Enables the WDT controllers within the DMSC.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_ENABLE_WDT
 *  \n<b>Request</b>:    #tisci_msg_enable_wdt_req
 *  \n<b>Response</b>:   #tisci_msg_enable_wdt_resp
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmEnableWdt(uint32_t timeout);

/**
 *  \brief    This message is part of the CPU Off sequence. The sequence is:
 *  - Mask interrupts
 *  - Send wake reset message to PMMC
 *  - Wait for wake reset ACK
 *  - Abort if any interrupts are pending
 *  - Disable all interrupts
 *  - Send goodbye to PMMC
 *  - Wait for goodbye ACK
 *  - Execute WFI
 *
 *  \n<b>Message</b>:    #TISCI_MSG_WAKE_RESET
 *  \n<b>Request</b>:    #tisci_msg_wake_reset_req
 *  \n<b>Response</b>:   #tisci_msg_wake_reset_resp
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmDisableWakeup(uint32_t timeout);

/**
 *  \brief    Request wakeup reason
 *  After a wakeup, the host can request the deepest sleep/idle mode reached
 *  and the reason for the wakeup. The API also returns the time spent in idle
 *  state.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_WAKE_REASON
 *  \n<b>Request</b>:    #tisci_msg_wake_reason_req
 *  \n<b>Response</b>:   #tisci_msg_wake_reason_resp
 *
 *  \param  mode            Deepest sleep/idle mode 0x000C reached (ASCII)
 *  \param  reason          Wakeup reason (ASCII)
 *  \param  time_ms         Time spent in idle state (ms)
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmGetWakeupReason(uint8_t mode[32],
                                    uint8_t reason[32],
                                    uint32_t * time_ms,
                                    uint32_t timeout);
/**
 *  \brief    Some processors have a special sequence for powering off the core
 *  that provides notification to the PMMC when that sequence has completed.
 *  For processors without such a sequence, the goodbye message exists.
 *  The exact sequence involved in the goodbye message depends on the SoC.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_GOODBYE
 *  \n<b>Request</b>:    #tisci_msg_goodbye_req
 *  \n<b>Response</b>:   #tisci_msg_goodbye_resp
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmDevicePowerOff(uint32_t timeout);

/**
 *  \brief   Objective: Trigger a SoC level reset
 *  Usage: Used to trigger a system level reset.
 *  NOTE: Depending on permissions configured for the SoC, not all processing
 *  entities may be permitted to request a SoC reset. When permitted, the
 *  request once processed will not return back to caller.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_SYS_RESET
 *  \n<b>Request</b>:    #tisci_msg_sys_reset_req
 *  \n<b>Response</b>:   #tisci_msg_sys_reset_resp
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmDeviceReset(uint32_t timeout);

/**
 *  \brief   This API would check if the given module Id is valid for the
 *           device.
 *
 *  \param  modId       Module Index to be checked.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_pmIsModuleValid(uint32_t modId);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_FMWPMMESSAGES_H_ */

/** @} */
