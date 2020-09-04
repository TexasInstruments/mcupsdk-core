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
 *  \file sciclient_rm_priv.h
 *
 *  \brief This file contains macros and structures used internally by sciclient_rm.
 */
#ifndef SCICLIENT_PRIV_RM_H_
#define SCICLIENT_PRIV_RM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/hw_include/hw_types.h>

#include <drivers/sciclient.h>
#include <drivers/sciclient/sciclient_priv.h>
#include <drivers/sciclient/soc/sciclient_soc_priv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* NULL device */
#define SCICLIENT_RM_DEV_NONE 255

#define SCICLIENT_RM_IA_GENERIC_EVT_RESETVAL 0xFFFF

#define SCICLIENT_RM_IR_MAPPING_FREE 0xFFFF

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief
 * Describes the IA events used by ROM and if they've been cleared back to
 * hardware reset values
 *
 * \param event
 * IA event used by ROM
 *
 * \param cleared
 * Boolean storing whether or not the event and the IA VINT status bit its
 * been mapped to has been cleared
 */
struct Sciclient_rmIaUsedMapping {
    uint16_t    event;
    bool        cleared;
};

/**
 * \brief Interrupt aggregator instance containing data required to manage interrupt
 *        route discovery using an IA.
 *
 * \param dev_id
 * Device ID of NavSS IA
 *
 * \param imap
 * IA subsystem's event interrupt mapping MMR region
 *
 * \param sevt_offset
 * SEVI global event offset for IA
 *
 * \param n_sevt
 * Number of IA SEVIs
 *
 * \param n_vint
 * Number of IA virtual interrupts
 *
 * \param vint_usage_count
 * Number of VINT status bits used for event mapping.  The usage count for
 * each VINT cannot exceed the number of maximum supported event mappings.
 *
 * \param v0_b0_evt
 * A standalone element is needed to track usage of VINT 0 bit 0.  The INTMAP
 * registers are read to determine VINT bit usage.  The default state for
 * INTMAP registers is 0x0000 which translates to the INTMAP using VINT 0 bit
 * 0.  This field allows the IA driver to determine if VINT 0 bit 0 is truly
 * mapped to an IA event.
 *
 * \param rom_usage
 * IA event to VINT mappings used by ROM during boot that need to be reset
 * to hardware reset values
 *
 * \param n_rom_usage
 * Number of entries in the rom_usage array
 */
struct Sciclient_rmIaInst {
    const uint16_t                          dev_id;
    const uint32_t                          imap;
    const uint16_t                          sevt_offset;
    const uint16_t                          n_sevt;
    const uint16_t                          n_vint;
    uint8_t                                 *vint_usage_count;
    uint16_t                                v0_b0_evt;
    struct Sciclient_rmIaUsedMapping *const rom_usage;
    const uint8_t                           n_rom_usage;
};

/**
 * \brief
 * Describes a range of IR input to output mappings used by ROM and if they've
 * been cleared back to hardware reset values
 *
 * \param inp_start
 * Start of IR input range
 *
 * \param outp_start
 * Start of IR output range
 *
 * \param length
 * Length of the mapping range
 *
 * \param cleared
 * Boolean storing whether or not the mapped range has been cleared
 */
struct Sciclient_rmIrUsedMapping {
    uint16_t    inp_start;
    uint16_t    outp_start;
    uint16_t    length;
    bool        cleared;
};

/**
 * \brief Interrupt router instance containing data required to manage interrupt
 *        route discovery using an IR.
 *
 * \param dev_id
 * Device ID of IR
 *
 * \param cfg
 * IR subsystem's config MMR region
 *
 * \param n_inp
 * Number of IR inputs
 *
 * \param n_outp
 * Number of IR outputs
 *
 * \param inp0_mapping
 * A standalone element is needed to track usage of IR input 0.  The CONTROL
 * registers are read to determine IR input and output usage.  The default
 * state for the output CONTROL registers is 0x0000 which translates to use of
 * IR input zero.  This field allows the IR driver to determine which IR
 * output input zero has been mapped to.
 *
 * \param rom_usage
 * IR inputs and outputs used by ROM during boot that need to be reset to defaults
 *
 * \param n_rom_usage
 * Number of entries in the rom_usage array
 */
struct Sciclient_rmIrInst {
    const uint16_t                          dev_id;
    const uint32_t                          cfg;
    const uint16_t                          n_inp;
    const uint16_t                          n_outp;
    uint16_t                                inp0_mapping;
    struct Sciclient_rmIrUsedMapping *const rom_usage;
    const uint8_t                           n_rom_usage;
};

/**
 * \brief IRQ connection interface
 *
 * Defines a single or contiguous block of outgoing interrupt connections
 * destined for a SoC subsystem within the interrupt interconnection tree.
 *
 * \param lbase
 * Local subsystem's outgoing interrupt interface base
 *
 * \param rbase
 * Remote subsystem's incoming interrupt interface base
 *
 * \param len
 * Number of contiguous interrupt interface connections between the two subsystems
 *
 * \param rid
 * Remote subsystem ID
 */
struct Sciclient_rmIrqIf {
    uint16_t    lbase;
    uint16_t    rbase;
    uint16_t    len;
    uint16_t    rid;
};

/**
 * \struct irq_node
 *
 * \brief IRQ interconnection tree node
 *
 * Defines a node in the SoC IRQ interconnection tree.  The node documents the
 * subsystem ID and all outgoing interrupt interfaces from the subsystem.
 *
 * \param id
 * subsystem ID
 *
 * \param n_if
 * Number of outgoing interrupt interfaces from the subsystem
 *
 * \param p_if
 * Pointer to array of outgoing interrupt interface data structures
 */
struct Sciclient_rmIrqNode {
    uint16_t                                id;
    uint16_t                                n_if;
    const struct Sciclient_rmIrqIf *const   *p_if;
};


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Finds and programs an available interrupt route via System Firmware
 *         based on the input parameters.
 *
 *  \param  req             Pointer to interrupt route set payload
 *
 *  \param  resp            Pointer to interrupt route set response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return r       SystemP_FAILURE: If the route is not programmed.
 *                  SystemP_SUCCESS:  If the route is programmed.
 */
int32_t Sciclient_rmProgramInterruptRoute (const struct tisci_msg_rm_irq_set_req *req,
                                           const struct tisci_msg_rm_irq_set_resp *resp,
                                           uint32_t timeout);

/**
 *  \brief Mapping resources from AM65xx PG2.0 to AM65xx PG1.0.
 *         NOTE: Not all the PG2.0 resources are mapped to PG1.0. Due to this
 *               the API is programmed to return failure if a match is not
 *               found.
 *
 *  \param  req             Pointer to interrupt route release payload
 *
 *  \param  resp            Pointer to interrupt route release response payload
 *
 *  \param  timeout         Gives a sense of how long to wait for the operation.
 *                          Refer \ref SystemP_Timeout.
 *
 *  \return r       SystemP_FAILURE: If the route is not cleared.
 *                  SystemP_SUCCESS:  If the route is cleared.
 */
int32_t Sciclient_rmClearInterruptRoute (const struct tisci_msg_rm_irq_release_req *req,
                                         const struct tisci_msg_rm_irq_release_resp *resp,
                                         uint32_t timeout);

/**
 *  \brief Translates an interrupt router or aggregator output to the 
 *         peripheral input or IR input it's connected to.  
 *         The primary use of the function is to retrieve the processor 
 *         input IRQ or IR input an interrupt router or aggregator 
 *         output is connected to.
 *
 *  \param  src_dev_id      Interrupt router or aggregator device ID
 *
 *  \param  src_output      Interrupt router or aggregator output index
 *
 *  \param  dst_dev_id      Device ID of entity connected to interrupt router
 *                          or aggregator output
 *
 *  \param  dst_input       Pointer to returned input index of entity connected
 *                          to interrupt router or aggregator output
 *
 *  \return SystemP_SUCCESS on successful translation, else failure
 */
int32_t Sciclient_rmTranslateIntOutput(uint16_t  src_dev_id,
                                       uint16_t  src_output,
                                       uint16_t  dst_dev_id,
                                       uint16_t  *dst_input);

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
int32_t Sciclient_rmTranslateIrqInput(uint16_t   dst_dev_id,
                                      uint16_t   dst_input,
                                      uint16_t   src_dev_id,
                                      uint16_t   *src_output);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/**
 *  \anchor Sciclient_proxyMap
 *  \name Sciclient map structure
 *  @{
 *  Map structure for R5F,A53,GPU and ICSSG \n
 *  in different contexts.
 */
extern const Sciclient_MapStruct_t gSciclientMap[SCICLIENT_CONTEXT_MAX_NUM];
/* @} */

extern struct Sciclient_rmIaInst gRmIaInstances[SCICLIENT_RM_IA_NUM_INST];

extern struct Sciclient_rmIrInst gRmIrInstances[SCICLIENT_RM_IR_NUM_INST];

/**
 * Extern IRQ interconnection tree defined in SoC-specific irq_tree.c.  The
 * IRQ tree can be referenced by including this header.
 */
extern const struct Sciclient_rmIrqNode *const gRmIrqTree[];

/**
 * Extern IRQ interconnection tree array size defined in SoC-specific
 * irq_tree.c.  The IRQ tree size can be referenced by including this header.
 */
extern const uint32_t gRmIrqTreeCount;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_PRIV_RM_H_*/
