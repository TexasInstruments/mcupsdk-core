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
 *
 */

/**
 *  \file   am65x/sciclient_fmwMsgParams.h
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
#include <drivers/soc.h>

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
/** r5(Non Secure): Cortex R5 Context 0 on MCU island */
#define SCICLIENT_CONTEXT_R5_NONSEC_0                     (0U)
/** r5(Secure): Cortex R5 Context 1 on MCU island(Boot) */
#define SCICLIENT_CONTEXT_R5_SEC_0                        (1U)
/** r5(Non Secure): Cortex R5 Context 2 on MCU island */
#define SCICLIENT_CONTEXT_R5_NONSEC_1                     (2U)
/** r5(Secure): Cortex R5 Context 3 on MCU island */
#define SCICLIENT_CONTEXT_R5_SEC_1                        (3U)
/** a53(Secure): Cortex A53 context 0 on Main island */
#define SCICLIENT_CONTEXT_A53_SEC_0                       (4U)
/** a53(Secure): Cortex A53 context 1 on Main island */
#define SCICLIENT_CONTEXT_A53_SEC_1                       (5U)
/** a53(Non Secure): Cortex A53 context 2 on Main island */
#define SCICLIENT_CONTEXT_A53_NONSEC_0                    (6U)
/** a53(Non Secure): Cortex A53 context 3 on Main island */
#define SCICLIENT_CONTEXT_A53_NONSEC_1                    (7U)
/** a53(Non Secure): Cortex A53 context 4 on Main island */
#define SCICLIENT_CONTEXT_A53_NONSEC_2                    (8U)
/** a53(Non Secure): Cortex A53 context 5 on Main island */
#define SCICLIENT_CONTEXT_A53_NONSEC_3                    (9U)
/** a53(Non Secure): Cortex A53 context 6 on Main island */
#define SCICLIENT_CONTEXT_A53_NONSEC_4                    (10U)
/** a53(Non Secure): Cortex A53 context 7 on Main island */
#define SCICLIENT_CONTEXT_A53_NONSEC_5                    (11U)
/** gpu(Non Secure): SGX544 Context 0 on Main island */
#define SCICLIENT_CONTEXT_GPU_NONSEC_0                    (12U)
/** gpu(Non Secure): SGX544 Context 1 on Main island */
#define SCICLIENT_CONTEXT_GPU_NONSEC_1                    (13U)
/** icssg(Non Secure): ICSS Context 0 on Main island */
#define SCICLIENT_CONTEXT_ICSSG_NONSEC_0                  (14U)
/** icssg(Non Secure): ICSS Context 1 on Main island */
#define SCICLIENT_CONTEXT_ICSSG_NONSEC_1                  (15U)
/** icssg(Non Secure): ICSS Context 2 on Main island */
#define SCICLIENT_CONTEXT_ICSSG_NONSEC_2                  (16U)
/** Total number of possible contexts for application. */
#define SCICLIENT_CONTEXT_MAX_NUM                        (17U)
/** @} */

/**
 *  \anchor Sciclient_ProcessorIds
 *  \name Sciclient Processor Ids
 *  @{
 *  Processor IDs for the Processor Boot Configuration APIs.
 */
/** COMPUTE_CLUSTER_MSMC0: (Cluster 0 Processor 0) */
#define SCICLIENT_PROCID_A53_CL0_C0                       (0x20U)
/** COMPUTE_CLUSTER_MSMC0: (Cluster 0 Processor 1) */
#define SCICLIENT_PROCID_A53_CL0_C1                       (0x21U)
/** COMPUTE_CLUSTER_MSMC0: (Cluster 1 Processor 0) */
#define SCICLIENT_PROCID_A53_CL1_C0                       (0x22U)
/** COMPUTE_CLUSTER_MSMC0: (Cluster 1 Processor 1) */
#define SCICLIENT_PROCID_A53_CL1_C1                       (0x23U)
/** MCU_SEC_MMR0: (Cluster 0 Processor 0) */
#define SCICLIENT_PROCID_R5_CL0_C0                        (0x01U)
/** MCU_SEC_MMR0: (Cluster 0 Processor 1) */
#define SCICLIENT_PROCID_R5_CL0_C1                        (0x02U)
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
#include <drivers/sciclient/include/tisci/am65x/tisci_devices.h>
#include <drivers/sciclient/include/tisci/am65x_sr2/tisci_devices.h>
/** @} */

/**
 *  \anchor Sciclient_PmModuleClockIds
 *  \name Power Management Clock IDs Module Wise
 *  @{
 *  Power Management Module Clock IDs for individual modules.
 */
#include <drivers/sciclient/include/tisci/am65x/tisci_clocks.h>
#include <drivers/sciclient/include/tisci/am65x_sr2/tisci_clocks.h>
/** @} */

/**
 * \brief Special ISC ID to refer to compute cluster privid registers
 */
#define TISCI_ISC_CC_ID                (160U)

/**
 *  \anchor Sciclient_IrqSrcIdxStart
 *  \name IRQ source index start
 *  @{
 *  Start offset of IRQ source index.
 */
#define TISCI_RINGACC0_OES_IRQ_SRC_IDX_START        (0U)
#define TISCI_RINGACC0_MON_IRQ_SRC_IDX_START        (1024U)
#define TISCI_RINGACC0_EOES_IRQ_SRC_IDX_START       (2048U)
#define TISCI_UDMAP0_TX_OES_IRQ_SRC_IDX_START       (0U)
#define TISCI_UDMAP0_TX_EOES_IRQ_SRC_IDX_START      (256U)
#define TISCI_UDMAP0_RX_OES_IRQ_SRC_IDX_START       (512U)
#define TISCI_UDMAP0_RX_EOES_IRQ_SRC_IDX_START      (768U)
#define TISCI_UDMAP0_RX_FLOW_EOES_IRQ_SRC_IDX_START (1024U)
/** @} */

/**
 *  \anchor Sciclient_McuR5fIds
 *  \name MCU Pulsar IDs
 *  @{
 *  MCU Device CPU IDs.
 */
#define SCICLIENT_DEV_MCU_R5FSS0_CORE0  (TISCI_DEV_MCU_ARMSS0_CPU0)
#define SCICLIENT_DEV_MCU_R5FSS0_CORE1  (TISCI_DEV_MCU_ARMSS0_CPU1)
/** @} */

/**
 *  \anchor Sciclient_McuR5fProcIds
 *  \name MCU Pulsar Processor IDs
 *  @{
 *  MCU Device Processor IDs.
 */
#define SCICLIENT_DEV_MCU_R5FSS0_CORE0_PROCID  (SCICLIENT_PROCID_R5_CL0_C0)
#define SCICLIENT_DEV_MCU_R5FSS0_CORE1_PROCID  (SCICLIENT_PROCID_R5_CL0_C1)
/** @} */

/** Board config Base start address */
#define SCICLIENT_ALLOWED_BOARDCFG_BASE_START (CSL_MCU_MSRAM0_RAM_BASE)
/** Board config Base end address */
#define SCICLIENT_ALLOWED_BOARDCFG_BASE_END   (CSL_MCU_MSRAM0_RAM_BASE + CSL_MCU_MSRAM0_RAM_SIZE)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_FMWMSGPARAMS_H_ */
