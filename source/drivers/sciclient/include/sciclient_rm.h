/*
 *  Copyright (C) 2018-2020 Texas Instruments Incorporated
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
 * \defgroup SCICLIENT_FMW_RM_IF Sciclient RM API Interface
 *
 * The DMSC firmware Resource Management (RM) (sub) system manages SoC shared
 * resources.  RM manages access and configuration of shared resources amongst
 * SoC processing entities.  RM provides a set of interfaces over which SoC
 * processing entities can allocate, configure, and free shared resources.
 *
 *
 * @{
 */
/**
 *  \file   sciclient_rm.h
 *
 *  \brief  This file contains the definition of all the message IDs, message
 *          formats to be able to interact with the System Controller firmware
 *          for resource management.
 */

#ifndef SCICLIENT_RM_H_
#define SCICLIENT_RM_H_

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
 *  \brief Retrieves a host's assigned range for a resource
 *
 *  Returns the range for a unique resource type assigned to the specified host,
 *  or secondary host.  The unique resource type is formed by combining the
 *  10 LSB of type and the 6 LSB of subtype.
 *
 *  Unique types which do not map to an SoC resource will not be NACK'd.  Instead
 *  the tisci_msg_rm_get_resource_range_resp range_start and range_num values are
 *  zeroed.  This provides a known response mechanism across varied SoCs.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_GET_RESOURCE_RANGE
 *  \n<b>Request</b>:    #tisci_msg_rm_get_resource_range_req
 *  \n<b>Response</b>:   #tisci_msg_rm_get_resource_range_resp
 *
 *  \param  req             Pointer to resource range get payload
 *
 *  \param  resp            Pointer to resource range response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmGetResourceRange(
                const struct tisci_msg_rm_get_resource_range_req *req,
                struct tisci_msg_rm_get_resource_range_resp *resp,
                uint32_t timeout);

/**
 *  \brief Configures a peripheral to processor IRQ
 *
 *  Configures an interrupt route between the peripheral and host processor
 *  specified within the #tisci_msg_rm_irq_set_req payload.  The interrupt
 *  destination is either the processor sending the request, or the secondary
 *  host if it's defined as a valid host.  The following message valid_params
 *  bit combinations are allowed:
 *
 *  Non-Event Sourced Direct Interrupt - Non-event peripheral interrupt direct
 *                                       to destination processor.  One thing
 *                                       to note is an IA unmapped VINT route
 *                                       can be configured via this combination
 *                                       by passing the IA ID and VINT values
 *                                       as the src_id and src_index parameters.
 *                                       An IA unmapped VINT route is considered
 *                                       a non-event sourced direct interrupt
 *                                       route until a global event is mapped
 *                                       to the IA VINT:
 *    dst_id valid bit == true
 *    dst_host_irq valid bit == true
 *    ia_id valid bit == false
 *    vint valid bit == false
 *    global_event valid bit == false
 *    vint_status_bit_index valid bit == false
 *
 *  Event Sourced Direct Interrupt - Event-based peripheral interrupt direct
 *                                   to destination processor:
 *    dst_id valid bit == true
 *    dst_host_irq valid bit == true
 *    ia_id valid bit == true
 *    vint valid bit == true
 *    global_event valid bit == true
 *    vint_status_bit_index valid bit == true
 *
 *  Unmapped VINT Direct Interrupt - Event-based peripheral interrupt direct to
 *                                   processor with no global event to VINT
 *                                   status bit mapping configured on allocation
 *                                   of the VINT.  Allows all event to VINT
 *                                   status bit mappings to take place at a
 *                                   later time:
 *    dst_id valid bit == true
 *    dst_host_irq valid bit == true
 *    ia_id valid bit == true
 *    vint valid bit == true
 *    global_event valid bit == false
 *    vint_status_bit_index valid bit == false
 *
 *  Event to VINT Mapping Only - Configure, or add a mapping to, an event-based
 *                               peripheral interrupt polled from IA VINT
 *                               real-time registers.  Can also be used to add
 *                               an event to VINT status bit mapping to an
 *                               event-based direct interrupt route:
 *    dst_id valid bit == false
 *    dst_host_irq valid bit == false
 *    ia_id valid bit == true
 *    vint valid bit == true
 *    global_event valid bit == true
 *    vint_status_bit_index valid bit == true
 *
 *  OES Register Programming Only - Only programs the OES register of the
 *                                  source.  Useful for setting UDMAP trigger
 *                                  events and any other events that are not
 *                                  translated to the interrupt domain:
 *    dst_id valid bit == false
 *    dst_host_irq valid bit == false
 *    ia_id valid bit == false
 *    vint valid bit == false
 *    global_event valid bit == true
 *    vint_status_bit_index valid bit == false
 *  The shortest route between the peripheral and the host processor is
 *  programmed.  Interrupts are not configured on the host processor.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_IRQ_SET
 *  \n<b>Request</b>:    #tisci_msg_rm_irq_set_req
 *  \n<b>Response</b>:   #tisci_msg_rm_irq_set_resp
 *
 *  \param  req             Pointer to interrupt route set payload
 *
 *  \param  resp            Pointer to interrupt route set response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmIrqSet(const struct tisci_msg_rm_irq_set_req *req,
                           const struct tisci_msg_rm_irq_set_resp *resp,
                           uint32_t timeout);

/**
 *  \brief Releases a peripheral to processor IRQ
 *
 *  Releases a previously configured interrupt route between a peripheral and
 *  host processor.  The interrupt destination is either the processor sending
 *  the request, or the secondary host if it's defined as a valid host.  The
 *  following valid_params valid bit combinations are allowed:
 *
 *  Non-Event Sourced Direct Interrupt - Non-event peripheral interrupt direct
 *                                       to destination processor.  One thing
 *                                       to note is an IA unmapped VINT route
 *                                       can be released via this combination
 *                                       by passing the IA ID and VINT values
 *                                       as the src_id and src_index parameters.
 *                                       An IA unmapped VINT route is considered
 *                                       a non-event sourced direct interrupt
 *                                       route until a global event is mapped
 *                                       to the IA VINT:
 *    dst_id valid bit == true
 *    dst_host_irq valid bit == true
 *    ia_id valid bit == false
 *    vint valid bit == false
 *    global_event valid bit == false
 *    vint_status_bit_index valid bit == false
 *
 *  Event Sourced Direct Interrupt - Event-based peripheral interrupt direct
 *                                   to destination processor:
 *    dst_id valid bit == true
 *    dst_host_irq valid bit == true
 *    ia_id valid bit == true
 *    vint valid bit == true
 *    global_event valid bit == true
 *    vint_status_bit_index valid bit == true
 *
 *  Unmapped VINT Direct Interrupt - Clear event-based interrupt direct to
 *                                   destination processor which does not have
 *                                   any existing event to VINT status bit
 *                                   mappings:
 *    dst_id valid bit == true
 *    dst_host_irq valid bit == true
 *    ia_id valid bit == true
 *    vint valid bit == true
 *    global_event valid bit == false
 *    vint_status_bit_index valid bit == false
 *
 *  Event to VINT Mapping Only - Clear only peripheral OES register and event to
 *                               VINT status bit mapping from direct to processor
 *                               and polled routes.  Event-based peripheral
 *                               interrupt polled routes are polled from the IA
 *                               VINT real-time registers.  For direct to
 *                               processor routes the entire route is NOT
 *                               released when the last event to VINT status bit
 *                               is unmapped using this valid bit combination.
 *                               This differs from using the Event Source Direct
 *                               Interrupt valid bit combination where the entire
 *                               route is released when the last event to VINT
 *                               status bit mapping is cleared.  The Unmapped
 *                               VINT Direct Interrupt valid bit combination is
 *                               used to clear an event sourced direct interrupt
 *                               with no existing event to VINT status bit
 *                               mappings:
 *    dst_id valid bit == false
 *    dst_host_irq valid bit == false
 *    ia_id valid bit == true
 *    vint valid bit == true
 *    global_event valid bit == true
 *    vint_status_bit_index valid bit == true
 *
 *  OES Register Programming Only - Only clears the OES register of the
 *                                  source.  Useful for clearing UDMAP trigger
 *                                  events and any other events that are not
 *                                  translated to the interrupt domain:
 *    dst_id valid bit == false
 *    dst_host_irq valid bit == false
 *    ia_id valid bit == false
 *    vint valid bit == false
 *    global_event valid bit == true
 *    vint_status_bit_index valid bit == false
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_IRQ_RELEASE
 *  \n<b>Request</b>:    #tisci_msg_rm_irq_release_req
 *
 *  \param  req             Pointer to interrupt route release payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmIrqRelease(const struct tisci_msg_rm_irq_release_req *req,
                               uint32_t timeout);

/**
 *  \brief Translates an interrupt router output to the peripheral input it's
 *         connected to.  The primary use of the function is to retrieve the
 *         processor input IRQ an interrupt router output is connected to.
 *
 *  \param  ir_dev_id       Interrupt router device ID
 *
 *  \param  ir_output       Interrupt router output index
 *
 *  \param  dst_dev_id      Device ID of entity connected to interrupt router
 *                          output
 *
 *  \param  dst_input       Pointer to returned input index of entity connected
 *                          to interrupt router output
 *
 *  \return SystemP_SUCCESS on successful translation, else failure
 */
int32_t Sciclient_rmIrqTranslateIrOutput(uint16_t   ir_dev_id,
                                         uint16_t   ir_output,
                                         uint16_t   dst_dev_id,
                                         uint16_t   *dst_input);

/**
 *  \brief Translates an interrupt aggregator output to the peripheral input
 *         it's connected to.
 *         The primary use of the function is to retrieve the processor input
 *         IRQ or IR input an interrupt aggregator output is connected to.
 *
 *  \param  ia_dev_id       Interrupt aggregator device ID
 *
 *  \param  ia_output       Interrupt aggregator output index
 *
 *  \param  dst_dev_id      Device ID of entity connected to interrupt router
 *                          output
 *
 *  \param  dst_input       Pointer to returned input index of entity
 *                          connected to interrupt router output
 *
 *  \return SystemP_SUCCESS on successful translation, else failure
 */
int32_t Sciclient_rmIrqTranslateIaOutput(uint16_t   ia_dev_id,
                                         uint16_t   ia_output,
                                         uint16_t   dst_dev_id,
                                         uint16_t   *dst_input);

/**
 *  \brief Translates a peripheral input to the connected interrupt router
 *         or aggregator output.
 *         The primary use of the function is to retrieve the interrupt router
 *         or aggregator output connected to a processor input IRQ.
 *
 *  \param  dst_dev_id      Device ID of entity connected to interrupt router
 *                          or aggregator output
 *
 *  \param  dst_input       Input index of entity connected to interrupt router
 *                          or aggregator output
 *
 *  \param  src_dev_id      Interrupt router or aggregator device ID
 *
 *  \param  src_output      Pointer to returned Interrupt router or
 *                          aggregator output index
 *
 *  \return SystemP_SUCCESS on successful translation, else failure
 */
int32_t Sciclient_rmIrqTranslateIrqInput(uint16_t   dst_dev_id,
                                         uint16_t   dst_input,
                                         uint16_t   src_dev_id,
                                         uint16_t   *src_output);

/**
 *  \brief Configures individual peripherals within the interrupt subsystem
 *         (interrupt routers, interrupt aggregators, etc.) according to the
 *         configuration provided.  Each call of the API only configures a single
 *         peripheral within the interrupt route.  Multiple calls of the API are
 *         required to setup a complete interrupt connection between source and
 *         destination which contains multiple hops.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_IRQ_SET
 *  \n<b>Request</b>:    #tisci_msg_rm_irq_set_req
 *  \n<b>Response</b>:   #tisci_msg_rm_irq_set_resp
 *
 *  \param  req             Pointer to interrupt peripheral set payload
 *
 *  \param  resp            Pointer to interrupt peripheral set response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmIrqSetRaw(const struct tisci_msg_rm_irq_set_req *req,
                              const struct tisci_msg_rm_irq_set_resp *resp,
                              uint32_t timeout);

/**
 *  \brief Releases configurations within individual peripherals within the
 *         interrupt subsystem (interrupt routers, interrupt aggregators, etc.)
 *         according to the configuration provided.  Each call of the API only
 *         releases a configuration within a single peripheral within the
 *         interrupt route.  Multiple calls of the API are required to teardown
 *         a complete interrupt connection between source and destination which
 *         contains multiple hops.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_IRQ_RELEASE
 *  \n<b>Request</b>:    #tisci_msg_rm_irq_release_req
 *
 *  \param  req             Pointer to interrupt peripheral release payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmIrqReleaseRaw(const struct tisci_msg_rm_irq_release_req *req,
                                  uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem ring
 *
 *  Configures the non-real-time registers of a Navigator Subsystem ring.
 *  The ring index must be assigned to the host defined in the TISCI header via
 *  the RM board configuration resource assignment range list.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_RING_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_ring_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_ring_cfg_resp
 *
 *  \param  req             Pointer to Ring Accelerator configure payload
 *
 *  \param  resp            Pointer to Ring Accelerator configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmRingCfg(const struct tisci_msg_rm_ring_cfg_req *req,
                            const struct tisci_msg_rm_ring_cfg_resp *resp,
                            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem ring monitor
 *
 *  Configures the non-real-time registers of a Navigator Subsystem ring
 *  monitor.
 *  The ring monitor index must be assigned to the host defined in the
 *  TISCI header via the RM board configuration resource assignment range list.
 *  Also, the ring being monitored must be assigned to the same host as
 *  the ring monitor
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_RING_MON_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_ring_mon_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_ring_mon_cfg_resp
 *
 *  \param  req             Pointer to Ring monitor configure payload
 *
 *  \param  resp            Pointer to Ring monitor configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmRingMonCfg(const struct tisci_msg_rm_ring_mon_cfg_req *req,
                               const struct tisci_msg_rm_ring_mon_cfg_resp *resp,
                               uint32_t timeout);

/**
 *  \brief Configures Navigator Subsystem UDMAP GCFG region
 *
 *  Configures a Navigator Subsystem UDMAP global configuration region.
 *  Configures the non-real-time registers of a Navigator Subsystem UDMAP
 *  global configuration region.  The register fields specified as valid for
 *  programming must be assigned to the host defined in the TISCI header via the
 *  RM board configuration resource assignment array.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_GCFG_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_gcfg_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_gcfg_cfg_resp
 *
 *  \param  req             Pointer to UDMAP GCFG configure payload
 *
 *  \param  resp            Pointer to UDMAP GCFG configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmUdmapGcfgCfg(
            const struct tisci_msg_rm_udmap_gcfg_cfg_req *req,
            const struct tisci_msg_rm_udmap_gcfg_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem UDMAP transmit channel
 *
 *  Configures the non-real-time registers of a Navigator Subsystem UDMAP
 *  transmit channel.  The channel index must be assigned to the host defined
 *  in the TISCI header via the RM board configuration resource assignment
 *  range list.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_TX_CH_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_tx_ch_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_tx_ch_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Tx channel configure payload
 *
 *  \param  resp            Pointer to UDMAP Tx channel configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmUdmapTxChCfg(
            const struct tisci_msg_rm_udmap_tx_ch_cfg_req *req,
            const struct tisci_msg_rm_udmap_tx_ch_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem UDMAP receive channel
 *
 *  Configures the non-real-time registers of a Navigator Subsystem UDMAP
 *  receive channel.  The channel index must be assigned to the host defined
 *  in the TISCI header via the RM board configuration resource assignment
 *  range list.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_RX_CH_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_rx_ch_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_rx_ch_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx channel configure payload
 *
 *  \param  resp            Pointer to UDMAP Rx channel configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmUdmapRxChCfg(
            const struct tisci_msg_rm_udmap_rx_ch_cfg_req *req,
            const struct tisci_msg_rm_udmap_rx_ch_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem UDMAP receive flow
 *
 *  Configures a Navigator Subsystem UDMAP receive flow's registers.
 *  Configuration does not include the flow registers which handle size-based
 *  free descriptor queue routing.  The
 *  #tisci_msg_rm_udmap_flow_size_thresh_cfg_req message is used to
 *  configure register fields related to size based free descriptor queues.
 *
 *  The flow index must be assigned to the host defined in the TISCI header via
 *  the RM board configuration resource assignment range list.
 *
 *  It's the user's responsibility to make sure any receive channels using the
 *  flow are disabled when changing the receive flow configuration.  Otherwise,
 *  unknown operation may occur.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_FLOW_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_flow_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_flow_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx flow configure payload
 *
 *  \param  resp            Pointer to UDMAP Rx flow configure response
 *                          payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmUdmapFlowCfg(
            const struct tisci_msg_rm_udmap_flow_cfg_req *req,
            const struct tisci_msg_rm_udmap_flow_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Configures a Navigator Subsystem UDMAP receive flow's size threshold
 *         fields.
 *
 *  Configures a Navigator Subsystem UDMAP receive flow's size threshold fields
 *
 *  The flow index must be assigned to the host defined in the TISCI header via
 *  the RM board configuration resource assignment range list.
 *
 *  It's the user's responsibility to make sure any receive channels using the
 *  flow are disabled when changing the receive flow configuration.  Otherwise,
 *  unknown operation may occur.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_UDMAP_FLOW_SIZE_THRESH_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_udmap_flow_size_thresh_cfg_req
 *  \n<b>Response</b>:   #tisci_msg_rm_udmap_flow_size_thresh_cfg_resp
 *
 *  \param  req             Pointer to UDMAP Rx flow size threshold based free
 *                          queue routing configure payload
 *
 *  \param  resp            Pointer to UDMAP Rx flow size threshold based free
 *                          queue routing configure response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmUdmapFlowSizeThreshCfg(
            const struct tisci_msg_rm_udmap_flow_size_thresh_cfg_req *req,
            const struct tisci_msg_rm_udmap_flow_size_thresh_cfg_resp *resp,
            uint32_t timeout);

/**
 *  \brief Pairs a PSI-L source thread and destination threads
 *
 *  Pairs a PSI-L source thread to a PSI-L destination thread.  The pairing
 *  occurs only if both threads are unpaired at the time of the pairing request.
 *  The source thread's width and credit count parameters are set to the
 *  destination thread's capabilities.  Both the source and destination threads
 *  are non-real-time enabled on successful pairing.
 *
 *  The PSI-L configuration proxy used to pair the source and destination
 *  threads is based on the Navigator Subsystem specified by #tisci_msg_rm_psil_pair_req::nav_id
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PSIL_PAIR
 *  \n<b>Request</b>:    #tisci_msg_rm_psil_pair_req
 *
 *  \param  req             Pointer to PSI-L thread pair payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmPsilPair(const struct tisci_msg_rm_psil_pair_req *req,
                             uint32_t timeout);

/**
 *  \brief Unpairs a PSI-L source thread and destination thread
 *
 *  Unpairs a PSI-L source thread from a PSI-L destination thread.  The
 *  source thread's width and credit count parameters are cleared.  Both the
 *  source and destination threads are non-real-time disabled on successful
 *  unpairing.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PSIL_UNPAIR
 *  \n<b>Request</b>:    #tisci_msg_rm_psil_unpair_req
 *
 *  \param  req             Pointer to PSI-L thread unpair payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmPsilUnpair(const struct tisci_msg_rm_psil_unpair_req *req,
                               uint32_t timeout);

/**
 *  \brief Reads a PSI-L thread real-time register
 *
 *  Reads the specified thread real-time configuration register from a
 *  specified PSI-L thread using the PSI-L configuration proxy.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PSIL_READ
 *  \n<b>Request</b>:    #tisci_msg_rm_psil_read_req
 *
 *  \param  req             Pointer to PSI-L thread read payload
 *
 *  \param  resp            Pointer to PSI-L thread read response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmPsilRead(const struct tisci_msg_rm_psil_read_req *req,
                             struct tisci_msg_rm_psil_read_resp *resp,
                             uint32_t timeout);

/**
 *  \brief Writes a PSI-L thread real-time register
 *
 *  Writes the specified thread real-time configuration register to a
 *  specified PSI-L thread using the PSI-L configuration proxy.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PSIL_WRITE
 *  \n<b>Request</b>:    #tisci_msg_rm_psil_write_req
 *
 *  \param  req             Pointer to PSI-L thread write payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmPsilWrite(const struct tisci_msg_rm_psil_write_req *req,
                              uint32_t timeout);

/**
 *  \brief Proxy Configuration Request
 *
 * The proxy_cfg TISCI message API is used to configure the channelized
 * firewalls of a Navigator Subsystem proxy. The proxy index must be assigned
 * to the host defined in the TISCI header via the RM board configuration resource
 * assignment range list. The channelized firewalls covering the proxy error events
 * are configured to only give DMSC write access. The proxy target data control
 * region channelized firewalls are configured to give the assigned host read and
 * write access.
 *
 *  \n<b>Message</b>:    #TISCI_MSG_RM_PROXY_CFG
 *  \n<b>Request</b>:    #tisci_msg_rm_proxy_cfg_req
 *
 *  \param  req             Pointer to proxy config payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmSetProxyCfg(const struct tisci_msg_rm_proxy_cfg_req *req,
                                uint32_t timeout);

/**
 *  \brief Checks the output's control register to see if the output line has an
 *        input line mapped to it.
 *
 *
 *  \param  id             TISCI Device ID of the interrupt router. Some possible values are :
 * 
 *                         TISCI_DEV_CMP_EVENT_INTROUTER0
 *                         TISCI_DEV_MAIN_GPIOMUX_INTROUTER0
 *                         TISCI_DEV_WKUP_MCU_GPIOMUX_INTROUTER0
 *                         TISCI_DEV_TIMESYNC_EVENT_ROUTER0
 *
 *  \param  outp           Output line of the interrupt router. Check TRM for number of output pins
 *                         of the interrupt router
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t Sciclient_rmIrOutpIsFree(uint16_t id, uint16_t outp);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_RM_H_ */

/** @} */
