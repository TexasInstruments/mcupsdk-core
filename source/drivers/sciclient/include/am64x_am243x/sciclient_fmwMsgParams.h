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
 *  \file   am64x_am243x/sciclient_fmwMsgParams.h
 *
 *  \brief  This file contains the definition of all the parameter IDs for
 *          PM, RM, Security.
 */

#ifndef SCICLIENT_FMWMSGPARAMS_H_
#define SCICLIENT_FMWMSGPARAMS_H_

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

/** Undefined Param Undefined */
#define TISCI_PARAM_UNDEF                                        (0xFFFFFFFFU)

/**
 *  \anchor Sciclient_FirmwareABI
 *  \name Sciclient Firmware ABI revisions
 *  @{
 *  ABI revisions for compatibility check.
 */
/* ABI Major revision - Major revision changes
*       indicate backward compatibility breakage */
#define SCICLIENT_FIRMWARE_ABI_MAJOR                     (3U)
/* ABI Minor revision - Minor revision changes
*       indicate backward compatibility is maintained,
*       however, new messages OR extensions to existing
*       messages might have been adde */
#define SCICLIENT_FIRMWARE_ABI_MINOR                     (1U)
/** @} */

/**
 *  \anchor Sciclient_ContextIds
 *  \name Sciclient Context Ids
 *  @{
 *  Context IDs for Sciclient_ConfigPrms_t .
 */
/** r5(Secure): Cortex R5 Context 0 */
#define SCICLIENT_CONTEXT_R5_0_SEC_0                   (0U)
/** r5(Non Secure): Cortex R5 Context 1 */
#define SCICLIENT_CONTEXT_R5_0_NONSEC_0                (1U)
/** r5(Secure): Cortex R5 Context 2 */
#define SCICLIENT_CONTEXT_R5_0_SEC_1                   (2U)
/** r5(Non Secure): Cortex R5 Context 3 */
#define SCICLIENT_CONTEXT_R5_0_NONSEC_1                (3U)
/** a53(Secure): Cortex A53 context 0 */
#define SCICLIENT_CONTEXT_A53_SEC_0                    (4U)
/** a53(Non Secure): Cortex A53 context 1 */
#define SCICLIENT_CONTEXT_A53_NONSEC_0                 (5U)
/** a53(Non Secure): Cortex A53 context 2 */
#define SCICLIENT_CONTEXT_A53_NONSEC_1                 (6U)
/** a53(Non Secure): Cortex A53 context 3 */
#define SCICLIENT_CONTEXT_A53_NONSEC_2                 (7U)
/** M4 (Non Secure): Cortex M4 context 1 */
#define SCICLIENT_CONTEXT_M4_NONSEC_0                  (8U)
/** r5(Secure): Cortex R5 Context 0 */
#define SCICLIENT_CONTEXT_R5_1_SEC_0                   (9U)
/** r5(Non Secure): Cortex R5 Context 1 */
#define SCICLIENT_CONTEXT_R5_1_NONSEC_0                (10U)
/** r5(Secure): Cortex R5 Context 2 */
#define SCICLIENT_CONTEXT_R5_1_SEC_1                   (11U)
/** r5(Non Secure): Cortex R5 Context 3 */
#define SCICLIENT_CONTEXT_R5_1_NONSEC_1                (12U)
/** ICSSG (Non Secure) : ICSSG Non Secure Context */
#define SCICLIENT_CONTEXT_ICSSG_NONSEC_0               (13U)
/** M4 (Secure): Cortex M4 context 1 */
#define SCICLIENT_CONTEXT_M4_SEC_0                     (14U) /* TBD */

/** Total number of possible contexts for application. */
#define SCICLIENT_CONTEXT_MAX_NUM                      (15U)
/** @} */

/**
 *  \anchor Sciclient_ProcessorIds
 *  \name Sciclient Processor Ids
 *  @{
 *  Processor IDs for the Processor Boot Configuration APIs.
 */

/** AM64_MAIN_SEC_MMR_MAIN_0: (Cluster 9 Processor 0) */
#define SCICLIENT_PROCID_A53_CL0_C0 (0x20U)
/** AM64_MAIN_SEC_MMR_MAIN_0: (Cluster 9 Processor 1) */
#define SCICLIENT_PROCID_A53_CL0_C1 (0x21U)
/** AM64_MAIN_SEC_MMR_MAIN_0: (Cluster 0 Processor 0) */
#define SCICLIENT_PROCID_R5_CL0_C0 (0x01U)
/** AM64_MAIN_SEC_MMR_MAIN_0: (Cluster 0 Processor 1) */
#define SCICLIENT_PROCID_R5_CL0_C1 (0x02U)
/** AM64_MAIN_SEC_MMR_MAIN_0: (Cluster 1 Processor 0) */
#define SCICLIENT_PROCID_R5_CL1_C0 (0x06U)
/** AM64_MAIN_SEC_MMR_MAIN_0: (Cluster 1 Processor 1) */
#define SCICLIENT_PROCID_R5_CL1_C1 (0x07U)
/*** AM64_MAIN_SEC_MMR_MAIN_0: (Cluster 16 Processor 0) */
#define SCICLIENT_PROCID_MCU_M4FSS0_C0 (0x18U)


/** @} */

/** -------------------- Resource Management Parameters ---------------------*/
#define TISCI_MSG_VALUE_RM_NULL_RING_TYPE     (0xFFFFu)
#define TISCI_MSG_VALUE_RM_NULL_RING_INDEX    (0xFFFFFFFFu)
#define TISCI_MSG_VALUE_RM_NULL_RING_ADDR     (0xFFFFFFFFu)
#define TISCI_MSG_VALUE_RM_NULL_RING_COUNT    (0xFFFFFFFFu)
/**
 * The ring mode field of the RING_SIZE register is not modified if this value
 * is used for:
 * @ref tisci_msg_rm_ring_cfg_req::mode
 */
#define TISCI_MSG_VALUE_RM_NULL_RING_MODE      (0xFFu)
#define TISCI_MSG_VALUE_RM_NULL_RING_SIZE      (0xFFu)
#define TISCI_MSG_VALUE_RM_NULL_ORDER_ID       (0xFFu)
#define TISCI_MSG_VALUE_RM_UDMAP_NULL_CH_TYPE  (0xFFu)
#define TISCI_MSG_VALUE_RM_UDMAP_NULL_CH_INDEX (0xFFFFFFFFu)

/** ------------ Power Management Messages Parameters -----------------------*/
/**
 *  \anchor Sciclient_PmDeviceIds
 *  \name Power Management Device IDs
 *  @{
 *  Power Management Module Device IDs
 */
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_devices.h>
/** @} */

/**
 *  \anchor Sciclient_PmModuleClockIds
 *  \name Power Management Clock IDs Module Wise
 *  @{
 *  Power Management Module Clock IDs for individual modules.
 */
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_clocks.h>
/** @} */

/**
 *  \anchor Sciclient_IrqSrcIdxStart
 *  \name IRQ source index start
 *  @{
 *  Start offset of IRQ source index.
 */

#define TISCI_RINGACC0_OES_IRQ_SRC_IDX_START         (20U)
#define TISCI_RINGACC0_EOES_IRQ_SRC_IDX_START        (32U)
#define TISCI_TIMERMGR_OES_IRQ_SRC_IDX_START         (0U)
#define TISCI_PKTDMA0_TX_EOES_IRQ_SRC_IDX_START      (4096U)
#define TISCI_PKTDMA0_TX_FLOW_OES_IRQ_SRC_IDX_START  (4608U)
#define TISCI_PKTDMA0_RX_EOES_IRQ_SRC_IDX_START      (5120U)
#define TISCI_PKTDMA0_RX_FLOW_OES_IRQ_SRC_IDX_START  (5632U)
#define TISCI_PKTDMA0_RX_FLOW_SOES_IRQ_SRC_IDX_START (6144U)
#define TISCI_PKTDMA0_RX_FLOW_FOES_IRQ_SRC_IDX_START (6656U)
#define TISCI_BCDMA0_BC_EOES_IRQ_SRC_IDX_START       (8192U)
#define TISCI_BCDMA0_BC_DC_OES_IRQ_SRC_IDX_START     (8704U)
#define TISCI_BCDMA0_BC_RC_OES_IRQ_SRC_IDX_START     (9216U)
#define TISCI_BCDMA0_TX_EOES_IRQ_SRC_IDX_START       (9728U)
#define TISCI_BCDMA0_TX_DC_OES_IRQ_SRC_IDX_START     (10240U)
#define TISCI_BCDMA0_TX_RC_OES_IRQ_SRC_IDX_START     (10752U)
#define TISCI_BCDMA0_RX_EOES_IRQ_SRC_IDX_START       (11264U)
#define TISCI_BCDMA0_RX_DC_OES_IRQ_SRC_IDX_START     (11776U)
#define TISCI_BCDMA0_RX_RC_OES_IRQ_SRC_IDX_START     (12288U)

/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_FMWMSGPARAMS_H_ */
