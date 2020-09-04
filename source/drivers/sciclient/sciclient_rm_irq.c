/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
 *  \file sciclient_rm_irq.c
 *
 *  \brief File containing the interrupt routing logic front-end for interrupt
 *         resource management DMSC services.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/sciclient.h>
#include <drivers/sciclient/sciclient_rm_priv.h>

#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_intaggr.h>
#include <drivers/hw_include/cslr_intr_router.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * Maximum depth of the path stack.
 */
#define SCICLIENT_PS_MAX_DEPTH (SCICLIENT_IRQ_MAX_ROUTE_DEPTH)

#define SCICLIENT_INP_TO_OUTP(rinp, rbase, lbase) (lbase + (rinp - rbase))
#define SCICLIENT_OUTP_TO_INP(loutp, lbase, rbase) (rbase + (loutp - lbase))

/*
 * IR Register Macros
 */
#define SCICLIENT_IR_INT_CONTROL_REG_OFFSET        0x00000004U
#define SCICLIENT_IR_INT_CONTROL_REG_STEP          0x00000004U

/*
 * IA Register Macros
 */
#define SCICLIENT_IA_ENTRY_INTMAP_LO(e)        ((e) * 0x8u)

#define SCICLIENT_IA_VINT_MAX_BITS             64u

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief Path stack node
 *
 * Stores a pointer to the IRQ tree node and its outgoing interface index
 *
 * \param p_n
 * Pointer to an IRQ tree node
 *
 * \param if_idx
 * Outgoing interface index to the next node (up) in the path stack
 *
 * \param inp
 * Valid incoming line to node from previous node in the stack
 *
 * \param outp
 * Valid outgoing line within interface specified by if_idx
 */
struct Sciclient_rmPsNode {
    const struct Sciclient_rmIrqNode    *p_n;
    uint16_t                            if_idx;
    uint16_t                            inp;
    uint16_t                            outp;
};

/**
 * \brief Path stack instance
 *
 * Instance of a path stack which can be manipulated by passing to the path
 * stack APIs
 *
 * \param ps
 * Path stack sized to maximum depth for the SoC
 *
 * \param psp
 * Path stack pointer
 */
struct Sciclient_rmPsInst {
    struct Sciclient_rmPsNode   ps[SCICLIENT_PS_MAX_DEPTH];
    uint16_t                    psp;
};

/**
 * \brief IRQ route configuration
 *
 * \param valid_params Validity information for optional parameters
 *
 * \param host IRQ route destination host
 *
 * \param s_id Device ID of peripheral sourcing the IRQ
 *
 * \param s_idx Peripheral resource index sourcing the IRQ
 *
 * \param d_id Device ID of IRQ route destination host
 *
 * \param d_irq Destination interrupt controller input IRQ
 *
 * \param global_evt Global event - applicable if source peripheral to
 *                   destination host route requires use of an IA
 *
 * \param s_ia Source IA - applicable if source peripheral to destination host
 *             route requires use of an IA
 *
 * \param vint VINT - applicable if source peripheral to destination
 *             host route requires use of an IA
 *
 * \param vint_sb VINT status bit - applicable if source peripheral to
 *                destination host route requires use of an IA
 *
 * \param set_resp Tracks responses from Sciclient_service for irq set calls
 */
struct Sciclient_rmIrqCfg {
    uint32_t                                    valid_params;
    uint8_t                                     host;
    uint16_t                                    s_id;
    uint16_t                                    s_idx;
    uint16_t                                    d_id;
    uint16_t                                    d_irq;
    uint16_t                                    global_evt;
    uint16_t                                    s_ia;
    uint16_t                                    vint;
    uint8_t                                     vint_sb;
    const struct tisci_msg_rm_irq_set_resp      *set_resp;
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Get the interrupt router from the instance address.
 *
 * \param addr Address of the IR
 *
 * \param i    Iteration number
 *
 * \return Updated address
 */
static uint32_t * Sciclient_getIrAddr (uint32_t addr, uint32_t i);

/**
 * \brief Check if a parameter is valid.  Uses a valid_params field and the bit
 *        mask for a parameter to check if a parameter is valid.
 *
 * \param valid_params Valid parameter bit field
 *
 * \param param_mask Bit mask for the parameter to validate
 *
 * \return true if parameter is valid, else false
 */
static bool Sciclient_rmParamIsValid(uint32_t valid_params, uint32_t param_mask);

/**
 * \brief Initializes the path stack so all node pointers are NULL and if_idx
 *        values are zero.  Sets the path stack pointer to zero.
 */
static void Sciclient_rmPsInit(void);

/**
 * \brief Pushes a node and its outgoing interface onto the path stack then
 *        increments the path stack pointer
 *
 * \param n
 * Pointer to the IRQ tree node to be pushed onto the stack
 *
 * \param if_idx
 * Outgoing interface from IRQ tree node to be pushed onto the stack
 *
 * \return
 * SystemP_SUCCESS - Push operation succeeded
 * SystemP_FAILURE - Push operation failed because stack is full
 */
static int32_t Sciclient_rmPsPush(const struct Sciclient_rmIrqNode *n,
                                  uint16_t                         if_idx);

/**
 * \brief Decrements the path stack pointer then pops a node off the path stack,
 *        updating the provided pointer to point to the popped node and the
 *        if_idx to the popped index.  The path stack entry that was popped is
 *        then cleared.
 *
 * \param n
 * Pointer to a node pointer
 *
 * \param if_idx
 * Pointer to an outgoing interface variable
 *
 * \return
 * SystemP_SUCCESS - Push operation succeeded
 * SystemP_FAILURE - Pop operation failed because stack is empty
 */
static int32_t Sciclient_rmPsPop(const struct Sciclient_rmIrqNode  **n,
                                 uint16_t                          *if_idx);

/**
 * \brief Checks if the path stack is empty by looking at the path stack pointer
 *
 * \return
 * true - Path stack is empty
 * false - Path stack has at least one entry
 */
static bool Sciclient_rmPsIsEmpty(void);

/**
 * \brief Returns the current index of the path stack pointer.  Will be zero
 *        if no nodes are in the stack at the time of calling this function.
 *
 * \return
 * Path stack pointer value
 */
static uint16_t Sciclient_rmPsGetPsp(void);

/**
 * \brief Returns the potential value of the path stack pointer when the path
 *        stack is full.
 *
 * \return
 * Max path stack pointer value
 */
static uint16_t Sciclient_rmPsGetMaxPsp(void);

/**
 * \brief Returns a pointer to the IRQ tree node at the provided path stack
 *        index.  NULL will be returned for the pointer if the index value is
 *        greater than or equal to the current path stack pointer value.
 *
 * \param index
 * Path stack index
 *
 * \return
 * Pointer to IRQ tree node at the provided index
 */
static const struct Sciclient_rmIrqNode *Sciclient_rmPsGetIrqNode(
                uint16_t    index);

/**
 * \brief Returns the interface index at the provided path stack index.  An
 *        error will be returned for the interface index if the index value
 *        is greater than or equal to the current path stack pointer value.
 *
 * \param index
 * Path stack index
 *
 * \return
 * >= 0 - Interface index at the provided index
 * CSL_EBADARGS - Specified index is invalid
 */
static int32_t Sciclient_rmPsGetIfIdx(uint16_t  index);

/**
 * \brief The inp value is updated for an existing path stack node.  Setting of
 *        the inp value is out of band from Sciclient_rmPsPush since the inp
 *        value cannot be set until the entire path in the path stack is
 *        validated.  An error is returned if the index is greater than or
 *        equal to the current path stack index.
 *
 * \param index
 * Path stack index
 *
 * \param inp
 * Validate input line for node at specified index
 *
 * \return
 * SystemP_SUCCESS - Set successful
 * CSL_EBADARGS - Specified index is invalid
 */
static int32_t Sciclient_rmPsSetInp(uint16_t    index,
                                    uint16_t    inp);

/**
 * \brief This function gets the specified pushed node's inp line.  An error
 *        will be returned if the index is greater than or equal to the current
 *        path stack index.
 *
 * \param index
 * Path stack index
 *
 * \return
 * >= 0 - Path stack node's inp value
 * CSL_EBADARGS - Specified index is invalid
 */
static int32_t Sciclient_rmPsGetInp(uint16_t index);

/**
 * \brief The outp value is updated for an existing path stack node.  Setting of
 *        the outp value is out of band from Sciclient_rmPsPush since the outp
 *        value cannot be set until the entire path in the path stack is
 *        validated.  An error will be returned if the index is greater than or
 *        equal to the current path stack index.
 *
 * \param index
 * Path stack index
 *
 * \param outp
 * Validate output line for node at specified index
 *
 * \return
 * SystemP_SUCCESS - Set successful
 * CSL_EBADARGS - Specified index is invalid
 */
static int32_t Sciclient_rmPsSetOutp(uint16_t   index,
                                     uint16_t   outp);

/**
 * \brief This function gets the specified pushed node's outp line.  An error
 *        will be returned if the index is greater than or equal to the current
 *        path stack index.
 *
 * \param index
 * Path stack index
 *
 * \return
 * >= 0 - Path stack node's outp value
 * CSL_EBADARGS - Specified index is invalid
 */
static int32_t Sciclient_rmPsGetOutp(uint16_t   index);

/**
 * \brief Get IRQ tree node
 *
 * Uses a binary search of the IRQ tree node list to get the node for the
 * provided device ID
 *
 * \param id Device ID
 *
 * \param found_n Returns the node if found, else NULL
 *
 * \return SystemP_SUCCESS if node found, else error
 */
static int32_t Sciclient_rmIrqGetNode(uint16_t                          id,
                                      const struct Sciclient_rmIrqNode  **found_n);

/**
 * \brief Get IRQ tree node interface
 *
 * \param n IRQ node
 *
 * \param n_itf Node interface index
 *
 * \param itf Returns the node interface if found, else NULL
 *
 * \return SystemP_SUCCESS if node interface found, else error
 */
static int32_t Sciclient_rmIrqGetNodeItf(const struct Sciclient_rmIrqNode   *n,
                                         uint16_t                           n_itf,
                                         const struct Sciclient_rmIrqIf     **itf);

/**
 * \brief Checks whether the path stack has a node circular reference
 *
 * \param cfg IRQ configuration containing path stack
 *
 * \return true if loop occurs in path stack, else false
 */
static bool Sciclient_rmIrqCheckLoop(struct Sciclient_rmIrqCfg *cfg);

/**
 * \brief Check if valid parameter configuration is for a non-event sourced
 *        direct interrupt route
 *
 * \param valid_params Valid params field from message
 *
 * \return true if cfg is for non-event direct interrupt, else false
 */
static bool Sciclient_rmIrqCfgIsDirectNonEvent(uint32_t valid_params);

/**
 * \brief Check if valid parameter configuration is for a event sourced
 *        direct interrupt route
 *
 * \param valid_params Valid params field from message
 *
 * \return true if cfg is for event direct interrupt, else false
 */
static bool Sciclient_rmIrqCfgIsDirectEvent(uint32_t valid_params);

/**
 * \brief Check if valid parameter configuration is for an event sourced
 *        direct interrupt route with an unmapped VINT within it.
 *
 * \param valid_params Valid params field from message
 *
 * \return true if cfg is for event direct interrupt, else false
 */
static bool Sciclient_rmIrqCfgIsUnmappedVintDirectEvent(uint32_t valid_params);

/**
 * \brief Check if valid parameter configuration is for an event mapping to
 *        VINT only.
 *
 * \param valid_params Valid params field from message
 *
 * \return true if cfg is for event mapping only, else false
 */
static bool Sciclient_rmIrqCfgIsEventToVintMappingOnly(uint32_t valid_params);

/**
 * \brief Check if valid parameter configuration is for OES register
 *        programming only
 *
 * \param valid_params Valid params field from message
 *
 * \return true if cfg is for OES programming only, else false
 */
static bool Sciclient_rmIrqCfgIsOesOnly(uint32_t valid_params);

/**
 * \brief Finds if a VINT is in use by checking if the IR input tied to the
 *        VINT is in use.  This method for finding VINT usage is needed when
 *        the VINT has no existing event to status bit mappings.
 *
 * \param cfg IRQ route configuration
 *
 * \param vint_used Returns true if VINT is in use, false otherwise
 *
 * \return SystemP_SUCCESS if VINT usage check was successful, error otherwise
 */
static int32_t Sciclient_rmIrqIsVintRouteSet(struct Sciclient_rmIrqCfg  *cfg,
                                             bool                       *vint_used);

/**
 * \brief Checks if the path stack nodes form a valid IRQ route
 *
 * \param cfg IRQ route configuration
 *
 * \return true if route is valid, else false
 */
static bool Sciclient_rmIrqRouteValidate(struct Sciclient_rmIrqCfg  *cfg);

/**
 * \brief Finds a valid IRQ route based on the IRQ route configuration
 *
 * \param cfg IRQ route configuration
 *
 * \return SystemP_SUCCESS if valid route found, else error
 */
static int32_t Sciclient_rmIrqFindRoute(struct Sciclient_rmIrqCfg *cfg);

/**
 * \brief Programs a valid IRQ route using path stack information
 *
 * \param cfg IRQ route configuration
 *
 * \param map_vint Configures peripheral OES and maps event to VINT status bit
 *                 if set to TRUE.  Just configures IRs in route to
 *                 destination if set to FALSE.
 *
 * \return SystemP_SUCCESS if route created, else error
 */
static int32_t Sciclient_rmIrqProgramRoute(struct Sciclient_rmIrqCfg   *cfg,
                                           bool                        map_vint);

/**
 * \brief Add event to existing VINT
 *
 * Adds an event to the specified virtual interrupt and programs the event
 * into the source peripheral IRQ's OES register
 *
 * \param cfg Pointer to the IRQ route configuration
 *
 * \return SystemP_SUCCESS if addition to VINT is successful, else error
 */
static int32_t Sciclient_rmIrqVintAdd(struct Sciclient_rmIrqCfg *cfg);

/**
 * \brief Allocate event-based route without any event to VINT status bit
 *        mappings.  The event to VINT status bit mappings are allocated at
 *        a later time based on the discretion of the application.
 *
 * \param cfg Pointer to the IRQ route configuration
 *
 * \return SystemP_SUCCESS if route allocation is successful, else error
 */
static int32_t Sciclient_rmUnmappedVintRouteCreate(
                struct Sciclient_rmIrqCfg    *cfg);

/**
 * \brief Program a global event value into a peripheral's OES register
 *
 * \param cfg Pointer to the IRQ route configuration.  In this case, just the
 *            OES configuration.
 *
 * \return SystemP_SUCCESS if OES register programming is successful, else error
 */
static int32_t Sciclient_rmIrqProgramOesRegister(
                struct Sciclient_rmIrqCfg  *cfg);

/**
 * \brief Retrieve the programmed IRQ route described by the provided
 *        configuration
 *
 * \param cfg Pointer to the IRQ route configuration
 *
 * \return SystemP_SUCCESS if route discovered successfully, else error
 */
static int32_t Sciclient_rmIrqGetRoute(struct Sciclient_rmIrqCfg    *cfg);

/**
 * \brief Release the programmed IRQ route described by the provided
 *        configuration
 *
 * \param cfg Pointer to the IRQ route configuration
 *
 * \param unmap_vint Deletes peripheral OES and event to VINT status bit
 *                   mapping if set to TRUE.  Just deletes IR mappings from
 *                   route to destination if set to FALSE.
 *
 * \return SystemP_SUCCESS if route deleted, else error
 */
static int32_t Sciclient_rmIrqDeleteRoute(struct Sciclient_rmIrqCfg    *cfg,
                                          bool                         unmap_vint);

/**
 * \brief Delete event from existing VINT
 *
 * Deletes an event from the specified virtual interrupt and removes the event
 * from the source peripheral IRQ's OES register
 *
 * \param cfg Pointer to the IRQ route configuration
 *
 * \return SystemP_SUCCESS if removal of VINT is successful, else error
 */
static int32_t Sciclient_rmIrqVintDelete(struct Sciclient_rmIrqCfg  *cfg);

/**
 * \brief Delete an event-based, direct to processor interrupt route which
 *        has no existing event to VINT status bit mappings
 *
 * \param cfg Pointer to the IRQ route configuration
 *
 * \return SystemP_SUCCESS if route deletion is successful, else error
 */
static int32_t Sciclient_rmIrqUnmappedVintRouteDelete(
                struct Sciclient_rmIrqCfg   *cfg);

/**
 * \brief Clear global event value from a peripheral's OES register
 *
 * \param cfg Pointer to the IRQ route configuration.  In this case, just the
 *            OES configuration.
 *
 * \return SystemP_SUCCESS if OES register clear is successful, else error
 */
static int32_t Sciclient_rmIrqClearOesRegister(
                struct Sciclient_rmIrqCfg    *cfg);

/**
 * \brief Get IA instance mapped to provided device ID
 *
 * \param id IA device ID
 *
 * \return Pointer to IA instance, NULL if no instance mapped to ID
 */
static struct Sciclient_rmIaInst *Sciclient_rmIaGetInst(uint16_t  id);

/**
 * \brief Check if IA event is still ROM mapped after ROM handover
 *
 * \param inst Pointer to IA instance
 *
 * \param evt IA input event
 *
 * \return true if IA event is still ROM mapped, else false
 */
static bool Sciclient_rmIaEvtRomMapped(const struct Sciclient_rmIaInst  *inst,
                                       uint16_t                         evt);

/**
 * \brief Validate IA event for in use or free cases
 *
 * \param inst Pointer to IA instance
 *
 * \param evt IA input event
 *
 * \param vint Virtual interrupt.  Only applicable for "in_use" check
 *
 * \param vint_sb_index VINT status bit.  Only applicable for "in_use" check
 *
 * \param in_use Validate for in use case if true, else validate for free case
 *
 * \return SystemP_SUCCESS if validation succeeds, else CSL_EBADARGS
 */
static int32_t Sciclient_rmIaValidateEvt(const struct Sciclient_rmIaInst    *inst,
                                         uint16_t                           evt,
                                         uint16_t                           vint,
                                         uint8_t                            vint_sb_index,
                                         bool                               in_use);

/**
 * \brief Verifies the global event is not already mapped to a VINT status bit
 *
 * \param id
 * IA SoC device ID.
 *
 * \param global_evt
 * Global event
 *
 * \return SystemP_SUCCESS if global event is free, else CSL_EBADARGS
 */
static int32_t Sciclient_rmIaValidateGlobalEvt(uint16_t id,
                                               uint16_t global_evt);

/**
 * \brief Checks if the providing virtual interrupt, global event, and virtual
 *        interrupt status bit index are mapped.  Also, validates the VINT
 *        against the board configuration resasg data.
 *
 * \param host
 * Host ID
 *
 * \param id
 * IA SoC device ID.
 *
 * \param vint
 * IA virtual interrupt
 *
 * \param global_evt
 * Global event
 *
 * \param vint_sb_index
 * Virtual interrupt status bit to map to global event
 *
 * \return SystemP_SUCCESS if mapping is valid, else CSL_EBADARGS
 */
static int32_t Sciclient_rmIaValidateMapping(uint8_t   host,
                                             uint16_t  id,
                                             uint16_t  vint,
                                             uint16_t  global_evt,
                                             uint8_t   vint_sb_index);

/**
 * \brief Returns a virtual interrupt's number of events currently mapped to
 *        status bits.
 *
 * \param id
 * IA SoC device ID
 *
 * \param vint
 * IA virtual interrupt
 *
 * \param n_evts
 * Pointer to current number of events mapped by the virtual interrupt
 *
 * \return
 *      SystemP_SUCCESS - Get info was successful, else error
 */
static int32_t Sciclient_rmIaVintGetInfo(uint16_t   id,
                                         uint16_t   vint,
                                         uint8_t    *n_evts);

/**
 * \brief Checks whether the provided ID is mapped to an interrupt aggregator
 *        subsystem
 *
 * \param id
 * IA SoC device ID.
 *
 * \return
 *      true - ID is mapped to an IA
 *      false - ID is not mapped to an IA
 */
static bool Sciclient_rmIaIsIa(uint16_t id);

/**
 * \brief Returns the control register off for an IR output
 *
 * \param IR output index
 *
 * \return register offset for the output
 */
static uint32_t Sciclient_rmIrIntControlReg(uint16_t    outp);

/**
 * \brief Get IR instance mapped to provided device ID
 *
 * \param id IR device ID
 *
 * \return Pointer to IR instance, NULL if no instance mapped to ID
 */
static struct Sciclient_rmIrInst *Sciclient_rmIrGetInst(uint16_t    id);

/**
 * \brief Check if IR input is still ROM mapped after ROM handover
 *
 * \param inst Pointer to IR instance
 *
 * \param inp Input line to IR
 *
 * \return true if IR input is still ROM mapped, else false
 */
static bool Sciclient_rmIrInpRomMapped(const struct Sciclient_rmIrInst  *inst,
                                       uint16_t                         inp);

/**
 * \brief Checks output control registers to see if the input has been mapped to
 *        an output.
 *
 * \param dev_id
 * IR SoC device ID.
 *
 * \param inp
 * Input line to IR
 *
 * \return
 *      SystemP_SUCCESS - Input line is free
 *      SystemP_FAILURE - Input line is already allocated
 */
static int32_t Sciclient_rmIrInpIsFree(uint16_t id,
                                       uint16_t inp);

/**
 * \brief Check if IR output is still ROM mapped after ROM handover
 *
 * \param inst Pointer to IR instance
 *
 * \param outp Output line to IR
 *
 * \return true if IR output is still ROM mapped, else false
 */
static bool Sciclient_rmIrOutpRomMapped(const struct Sciclient_rmIrInst  *inst,
                                        uint16_t                         outp);

/**
 * \brief Returns the output line to which the specified input line has been
 *        mapped.  An error is returned if the inp line does not map to an
 *        output line.
 *
 * \param id
 * IR SoC device ID.
 *
 * \param inp
 * Input line to specified IR
 *
 * \param outp
 * Pointer to returned output line from specified IR
 *
 * \return
 *      SystemP_SUCCESS - Retrieval of output successful
 *      < 0 - Error occurred while attempting to retrieve output line
 */
static int32_t Sciclient_rmIrGetOutp(uint16_t   id,
                                     uint16_t   inp,
                                     uint16_t   *outp);

/**
 * \brief Checks whether the provided ID is mapped to an interrupt router
 *        subsystem
 *
 * \param id
 * IR SoC device ID.
 *
 * \return
 *      true - ID is mapped to an IR
 *      false - ID is not mapped to an IR
 */
static bool Sciclient_rmIrIsIr(uint16_t id);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/**
 * Interrupt route retrieval path stack instance.  Used to store the nodes in
 * the interrupt tree as a route is found.
 */
static struct Sciclient_rmPsInst gPstack;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Sciclient_rmProgramInterruptRoute (const struct tisci_msg_rm_irq_set_req *req,
                                           const struct tisci_msg_rm_irq_set_resp *resp,
                                           uint32_t timeout)
{
    int32_t r = SystemP_SUCCESS;
    uint16_t messageType = TISCI_MSG_RM_IRQ_SET;
    uint8_t dst_host;
    struct Sciclient_rmIrqCfg cfg;

    memset(&cfg, 0, sizeof(cfg));

    /* Run all error checks */
    if((req == NULL) || (resp == NULL)) {
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS &&
        Sciclient_rmParamIsValid(req->valid_params,
                                 TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID) == TRUE) {
        dst_host = req->secondary_host;
    } else {
        dst_host = (uint8_t) gSciclientMap[Sciclient_getCurrentContext(messageType)].hostId;
    }

    if (r == SystemP_SUCCESS &&
        Sciclient_rmParamIsValid(req->valid_params,
                                 TISCI_MSG_VALUE_RM_IA_ID_VALID) == TRUE) {
        cfg.s_ia = req->ia_id;
    } else {
        cfg.s_ia = SCICLIENT_RM_DEV_NONE;
    }

    if (r == SystemP_SUCCESS) {
        cfg.valid_params = req->valid_params;
        cfg.host = dst_host;
        cfg.s_id = req->src_id;
        cfg.s_idx = req->src_index;
        cfg.d_id = req->dst_id;
        cfg.d_irq = req->dst_host_irq;
        cfg.global_evt = req->global_event;
        cfg.vint = req->vint;
        cfg.vint_sb = req->vint_status_bit_index;
        cfg.set_resp = resp;

        if (Sciclient_rmIrqCfgIsDirectNonEvent(cfg.valid_params) == true) {
            if (Sciclient_rmIaIsIa(cfg.s_id) == true) {
                /*
                 * Users can configure unmapped vint direct
                 * events this way as well.  For these cases
                 * populate vint since it was passed as the
                 * source index
                 */
                cfg.vint = cfg.s_idx;
            }

            /* Route search for non event-source routes */
            r = Sciclient_rmIrqFindRoute(&cfg);
            if (r == SystemP_SUCCESS) {
                r = Sciclient_rmIrqProgramRoute(&cfg, false);
            }
        } else if ((Sciclient_rmIrqCfgIsDirectEvent(cfg.valid_params) ==
                true) ||
               (Sciclient_rmIrqCfgIsEventToVintMappingOnly(cfg.valid_params) ==
                true)) {
            /*
             * Route creation for event-sourced routes, direct
             * to processor or polled (where polling occurs at
             * VINT)
             * */
            r = Sciclient_rmIrqVintAdd(&cfg);
        } else if (Sciclient_rmIrqCfgIsUnmappedVintDirectEvent(cfg.valid_params) ==
               true) {
            /*
             * Route search for event-source routes but do not
             * configure peripheral OES or map event to VINT
             * status bit
             */
            r = Sciclient_rmUnmappedVintRouteCreate(&cfg);
        } else if (Sciclient_rmIrqCfgIsOesOnly(cfg.valid_params) == true) {
            /*
             * Just program OES register.  No interrupt route
             * associated with event.  For example, a UDMAP
             * trigger event
             */
            r = Sciclient_rmIrqProgramOesRegister(&cfg);
        } else {
            r = CSL_EBADARGS;
        }
    }

    return r;
}

int32_t Sciclient_rmClearInterruptRoute (const struct tisci_msg_rm_irq_release_req *req,
                                         const struct tisci_msg_rm_irq_release_resp *resp,
                                         uint32_t timeout)
{
    int32_t r = SystemP_SUCCESS;
    uint16_t messageType = TISCI_MSG_RM_IRQ_RELEASE;
    uint8_t dst_host;
    struct Sciclient_rmIrqCfg cfg;

    memset(&cfg, 0, sizeof(cfg));

    /* Run all error checks */
    if((req == NULL) || (resp == NULL)) {
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS &&
        Sciclient_rmParamIsValid(req->valid_params,
                                 TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID) == TRUE) {
        dst_host = req->secondary_host;
    } else {
        dst_host = (uint8_t) gSciclientMap[Sciclient_getCurrentContext(messageType)].hostId;
    }

    if (r == SystemP_SUCCESS &&
        Sciclient_rmParamIsValid(req->valid_params,
                                 TISCI_MSG_VALUE_RM_IA_ID_VALID) == TRUE) {
        cfg.s_ia = req->ia_id;
    } else {
        cfg.s_ia = SCICLIENT_RM_DEV_NONE;
    }

    if (r == SystemP_SUCCESS) {
        cfg.valid_params = req->valid_params;
        cfg.host = dst_host;
        cfg.s_id = req->src_id;
        cfg.s_idx = req->src_index;
        cfg.d_id = req->dst_id;
        cfg.d_irq = req->dst_host_irq;
        cfg.global_evt = req->global_event;
        cfg.vint = req->vint;
        cfg.vint_sb = req->vint_status_bit_index;

        if (Sciclient_rmIrqCfgIsDirectNonEvent(cfg.valid_params) == true) {
            if (Sciclient_rmIaIsIa(cfg.s_id) == true) {
                /*
                 * Users can release unmapped vint direct
                 * events this way as well.  For these cases
                 * populate s_ia and vint since they are passed
                 * as source id and source index.  Then call
                 * the unmapped VINT route delete.
                 */
                cfg.s_ia = cfg.s_id;
                cfg.vint = cfg.s_idx;
                r = Sciclient_rmIrqUnmappedVintRouteDelete(&cfg);
            } else {
                /* Route removal for non event-source routes */
                r = Sciclient_rmIrqGetRoute(&cfg);
                if (r == SystemP_SUCCESS) {
                    r = Sciclient_rmIrqDeleteRoute(&cfg, false);
                }
            }
        } else if ((Sciclient_rmIrqCfgIsDirectEvent(cfg.valid_params) ==
                true) ||
               (Sciclient_rmIrqCfgIsEventToVintMappingOnly(cfg.valid_params) ==
                true)) {
            /* Route removal for event-source routes */
            r = Sciclient_rmIrqVintDelete(&cfg);
        } else if (Sciclient_rmIrqCfgIsUnmappedVintDirectEvent(cfg.valid_params) ==
               true) {
            /*
             * Route removal for event-source route but no events
             * are mapped to VINT status bits
             */
            r = Sciclient_rmIrqUnmappedVintRouteDelete(&cfg);
        } else if (Sciclient_rmIrqCfgIsOesOnly(cfg.valid_params) == true) {
            /*
             * Just reset OES register.  No interrupt route
             * associated with event.  For example, a UDMAP
             * trigger event
             */
            r = Sciclient_rmIrqClearOesRegister(&cfg);
        } else {
            r = CSL_EBADARGS;
        }
    }

    return r;
}

int32_t Sciclient_rmTranslateIntOutput(uint16_t  src_dev_id,
                                      uint16_t  src_output,
                                      uint16_t  dst_dev_id,
                                      uint16_t  *dst_input)
{
    int32_t r = SystemP_SUCCESS;
    const struct Sciclient_rmIrqNode *cur_n;
    const struct Sciclient_rmIrqIf *cur_if;
    uint16_t i;
    bool translated = false;

    /* Only attempt to translate to destination input if an IR/IA is passed */
    if ((Sciclient_rmIrIsIr(src_dev_id) == true)||
        (Sciclient_rmIaIsIa(src_dev_id) == true)) {
        /*
         * Translate the specified IR/IA output to the destination processor
         * IRQ input
         */
        r = Sciclient_rmIrqGetNode(src_dev_id, &cur_n);
        if (r == SystemP_SUCCESS) {
            for (i = 0; i < cur_n->n_if; i++) {
                r = Sciclient_rmIrqGetNodeItf(cur_n, i, &cur_if);
                if (r != SystemP_SUCCESS) {
                    break;
                }

                if ((src_output >= cur_if->lbase) &&
                    (src_output < (cur_if->lbase + cur_if->len)) &&
                    (dst_dev_id == cur_if->rid)) {
                    *dst_input = SCICLIENT_OUTP_TO_INP(src_output,
                                                       cur_if->lbase,
                                                       cur_if->rbase);
                    translated = true;
                    break;
                }
            }

            if ((r == SystemP_SUCCESS) &&
                (translated == false)) {
                /* No translatable destination found. */
                r = CSL_EBADARGS;
            }
        }
    } else {
        r = CSL_EBADARGS;
    }

    return r;
}

int32_t Sciclient_rmTranslateIrqInput(uint16_t  dst_dev_id,
                                      uint16_t  dst_input,
                                      uint16_t  src_dev_id,
                                      uint16_t  *src_output)
{
    int32_t r = SystemP_SUCCESS;
    const struct Sciclient_rmIrqNode *cur_n;
    const struct Sciclient_rmIrqIf *cur_if;
    uint16_t i;
    bool translated = false;

    /* Only attempt to translate to IR/IA output if an IR/IA is passed */
    if ((Sciclient_rmIrIsIr(src_dev_id) == true) ||
        (Sciclient_rmIaIsIa(src_dev_id) == true)) {
        /*
         * Translate the specified destination processor IRQ input to the
         * IR/IA output
         */
        r = Sciclient_rmIrqGetNode(src_dev_id, &cur_n);
        if (r == SystemP_SUCCESS) {
            for (i = 0; i < cur_n->n_if; i++) {
                r = Sciclient_rmIrqGetNodeItf(cur_n, i, &cur_if);
                if (r != SystemP_SUCCESS) {
                    break;
                }

                if ((dst_dev_id == cur_if->rid) &&
                    (dst_input >= cur_if->rbase) &&
                    (dst_input < (cur_if->rbase + cur_if->len))) {
                    *src_output = SCICLIENT_INP_TO_OUTP(dst_input,
                                                        cur_if->rbase,
                                                        cur_if->lbase);
                    translated = true;
                    break;
                }
            }

            if ((r == SystemP_SUCCESS) &&
                (translated == false)) {
                /* No translatable IR/IA input found. */
                r = CSL_EBADARGS;
            }
        }
    } else {
        r = CSL_EBADARGS;
    }

    return r;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

static uint32_t * Sciclient_getIrAddr (uint32_t addr, uint32_t i)
{
    uint32_t * int_ctrl_reg;
#if defined (BUILD_MPU1_0) || defined(BUILD_MPU1_1)
    int_ctrl_reg = (uint32_t *)((uint64_t)addr + (uint64_t)Sciclient_rmIrIntControlReg(i));
#else
    int_ctrl_reg = (uint32_t *)(addr + Sciclient_rmIrIntControlReg(i));
    int_ctrl_reg = (uint32_t *)AddrTranslateP_getLocalAddr( (uint64_t)int_ctrl_reg);
#endif
#if defined (BUILD_C66X_1) || defined (BUILD_C66X_2)
    /* This corresponds to the IR registers for which the RAT is configured */
    if (((uint32_t)int_ctrl_reg >= CSL_C66SS0_INTROUTER0_INTR_ROUTER_CFG_BASE) &&
        ((uint32_t)int_ctrl_reg < (CSL_C66SS1_INTROUTER0_INTR_ROUTER_CFG_BASE +
                         CSL_C66SS1_INTROUTER0_INTR_ROUTER_CFG_SIZE)))
    {
        int_ctrl_reg = (uint32_t *)((uint32_t)int_ctrl_reg + CSL_C66_COREPAC_RAT_REGION_BASE);
    }
#endif
    return int_ctrl_reg;
}

static bool Sciclient_rmParamIsValid(uint32_t valid_params, uint32_t param_mask)
{
    bool r = false;

    if ((valid_params & param_mask) != 0) {
        r = true;
    }

    return r;
}

static void Sciclient_rmPsInit(void)
{
    /* Init path stack variables */
    memset(&gPstack.ps[0], 0, sizeof(gPstack.ps));
    gPstack.psp = 0;
}

static int32_t Sciclient_rmPsPush(const struct Sciclient_rmIrqNode *n,
                                  uint16_t                         if_idx)
{
    int32_t r = SystemP_SUCCESS;

    if ((gPstack.psp < SCICLIENT_PS_MAX_DEPTH) && (n != NULL)) {
        gPstack.ps[gPstack.psp].p_n = n;
        gPstack.ps[gPstack.psp].if_idx = if_idx;
        gPstack.psp++;
    } else {
        r = SystemP_FAILURE;
    }

    return r;
}

static int32_t Sciclient_rmPsPop(const struct Sciclient_rmIrqNode  **n,
                                 uint16_t                          *if_idx)
{
    int32_t r = SystemP_SUCCESS;

    if ((gPstack.psp > (0u)) && (n != NULL) && (if_idx != NULL)) {
        gPstack.psp--;
        *n = gPstack.ps[gPstack.psp].p_n;
        *if_idx = gPstack.ps[gPstack.psp].if_idx;

        /* Clear the just popped node */
        gPstack.ps[gPstack.psp].p_n = NULL;
        gPstack.ps[gPstack.psp].if_idx = 0;
        gPstack.ps[gPstack.psp].inp = 0;
        gPstack.ps[gPstack.psp].outp = 0;
    } else {
        r = SystemP_FAILURE;
    }

    return r;
}

static bool Sciclient_rmPsIsEmpty(void)
{
    bool e = true;

    if (gPstack.psp != 0) {
        e = false;
    }

    return e;
}

static uint16_t Sciclient_rmPsGetPsp(void)
{
    return gPstack.psp;
}

static uint16_t Sciclient_rmPsGetMaxPsp(void)
{
    return SCICLIENT_PS_MAX_DEPTH;
}

static const struct Sciclient_rmIrqNode *Sciclient_rmPsGetIrqNode(
                uint16_t    index)
{
    return gPstack.ps[index].p_n;
}

static int32_t Sciclient_rmPsGetIfIdx(uint16_t  index)
{
    int32_t r = CSL_EBADARGS;

    if (index < gPstack.psp) {
        r = gPstack.ps[index].if_idx;
    }

    return r;
}

static int32_t Sciclient_rmPsSetInp(uint16_t    index,
                                    uint16_t    inp)
{
    int32_t r = SystemP_SUCCESS;

    if (index < gPstack.psp) {
        gPstack.ps[index].inp = inp;
    } else {
        r = CSL_EBADARGS;
    }

    return r;
}

static int32_t Sciclient_rmPsGetInp(uint16_t    index)
{
    int32_t r = CSL_EBADARGS;

    if (index < gPstack.psp) {
        r = gPstack.ps[index].inp;
    }

    return r;
}

static int32_t Sciclient_rmPsSetOutp(uint16_t   index,
                                     uint16_t   outp)
{
    int32_t r = SystemP_SUCCESS;

    if (index < gPstack.psp) {
        gPstack.ps[index].outp = outp;
    } else {
        r = CSL_EBADARGS;
    }

    return r;
}

static int32_t Sciclient_rmPsGetOutp(uint16_t   index)
{
    int32_t r = CSL_EBADARGS;

    if (index < gPstack.psp) {
        r = gPstack.ps[index].outp;
    }

    return r;
}

static int32_t Sciclient_rmIrqGetNode(uint16_t                          id,
                                      const struct Sciclient_rmIrqNode  **found_n)
{
    int32_t r = CSL_EBADARGS;
    const struct Sciclient_rmIrqNode *cur_n;
    uint32_t lower, upper, current;
    uint16_t count;

    lower = 0u;
    upper = gRmIrqTreeCount - 1u;
    count = gRmIrqTreeCount;
    *found_n = NULL;

    /* Bounds check the search for failure robustness.  Make sure lower (l)
     * and upper (u) do not go outside searchable range.  Also, search
     * should be log(gRmIrqTreeCount) but fail out after gRmIrqTreeCount
     * iterations to avoid infinite loop in case of data corruption */
    while ((lower <= upper) &&
           (lower < gRmIrqTreeCount) &&
           (upper < gRmIrqTreeCount) &&
           (count > 0u)) {
        current = (lower + upper) / (2u);
        cur_n = (const struct Sciclient_rmIrqNode *) gRmIrqTree[current];
        if (cur_n->id == id) {
            *found_n = cur_n;
            r = SystemP_SUCCESS;
            break;
        } else {
            if (cur_n->id < id) {
                lower = current + (1u);
            } else {
                upper = current - (1u);
            }
        }
        count--;
    }

    return r;
}

static int32_t Sciclient_rmIrqGetNodeItf(const struct Sciclient_rmIrqNode   *n,
                                         uint16_t                           n_itf,
                                         const struct Sciclient_rmIrqIf     **itf)
{
    int32_t r = SystemP_SUCCESS;

    if (n_itf < n->n_if) {
        *itf = n->p_if[n_itf];
    } else {
        *itf = NULL;
        r = CSL_EBADARGS;
    }

    return r;
}

static bool Sciclient_rmIrqCheckLoop(struct Sciclient_rmIrqCfg *cfg)
{
    bool loop = false;
    uint16_t i, j;
    const struct Sciclient_rmIrqNode *cur_n, *future_n;

    /* A route cannot contain a single node more than once. Implies
     * a loop back which is not supported */
    for (i = 0u; i < Sciclient_rmPsGetPsp(); i++) {
        cur_n = Sciclient_rmPsGetIrqNode(i);
        for (j = i + 1u; j < Sciclient_rmPsGetPsp(); j++) {
            future_n = Sciclient_rmPsGetIrqNode(j);
            if (cur_n->id == future_n->id) {
                loop = true;
                break;
            }
        }
        if (loop == true) {
            break;
        }
    }

    return loop;
}

static bool Sciclient_rmIrqCfgIsDirectNonEvent(uint32_t valid_params)
{
    bool r = false;

    /*
     * The interrupt configuration is for a non-event sourced interrupt
     * direct to the destination processor.
     */
    if ((Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_ID_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_IA_ID_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID) == false)) {
        r = true;
    }

    return r;
}

static bool Sciclient_rmIrqCfgIsDirectEvent(uint32_t valid_params)
{
    bool r = false;

    /*
     * The interrupt configuration is for an event sourced interrupt
     * direct to the destination processor.
     */
    if ((Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_ID_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_IA_ID_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID) == true)) {
        r = true;
    }

    return r;
}

static bool Sciclient_rmIrqCfgIsUnmappedVintDirectEvent(uint32_t valid_params)
{
    bool r = false;

    /*
     * The interrupt configuration is for an event sourced interrupt
     * direct to the destination processor but the peripheral OES register
     * and event to VINT status bit mappings are not programmed.
     */
    if ((Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_ID_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_IA_ID_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID) == false)) {
        r = true;
    }

    return r;
}

static bool Sciclient_rmIrqCfgIsEventToVintMappingOnly(uint32_t valid_params)
{
    bool r = false;

    /*
     * The interrupt configuration is for an event sourced route but
     * configuration ends at the IA VINT.
     */
    if ((Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_ID_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_IA_ID_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID) == true)) {
        r = true;
    }

    return r;
}

static bool Sciclient_rmIrqCfgIsOesOnly(uint32_t valid_params)
{
    bool r = false;

    /*
     * The interrupt configuration provided to only program the OES
     * register of a peripheral capable of generating events.
     */
    if ((Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_ID_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_IA_ID_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_VALID) == false) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID) == true) &&
        (Sciclient_rmParamIsValid(valid_params,
                    TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID) == false)) {
        r = true;
    }

    return r;
}

static int32_t Sciclient_rmIrqIsVintRouteSet(struct Sciclient_rmIrqCfg  *cfg,
                                             bool                       *vint_used)
{
    int32_t r;
    uint16_t i;
    const struct Sciclient_rmIrqNode *ia_node;
    const struct Sciclient_rmIrqIf *iface;
    bool found_iface = false;
    uint16_t ir_inp;

    *vint_used = false;

    r = Sciclient_rmIrqGetNode(cfg->s_ia, &ia_node);
    if (r == SystemP_SUCCESS) {
        for (i = 0U; i < ia_node->n_if; i++) {
            r = Sciclient_rmIrqGetNodeItf(ia_node, i, &iface);
            if (r != SystemP_SUCCESS) {
                break;
            }

            if ((cfg->vint >= iface->lbase) &&
                (cfg->vint < (iface->lbase + iface->len))) {
                found_iface = true;
                break;
            }
        }
    }

    if (found_iface) {
        /* Check if the IR input tied to the IA VINT is in use. */
        ir_inp = SCICLIENT_OUTP_TO_INP(cfg->vint, iface->lbase, iface->rbase);
        if (Sciclient_rmIrInpIsFree(iface->rid, ir_inp) != SystemP_SUCCESS) {
            *vint_used = true;
        }
    }

    return r;
}

static bool Sciclient_rmIrqRouteValidate(struct Sciclient_rmIrqCfg  *cfg)
{
    bool valid = true;
    uint16_t i, j;
    const struct Sciclient_rmIrqNode *cur_n, *next_n = NULL;
    const struct Sciclient_rmIrqIf *cur_if;
    bool cur_outp_valid = false, next_inp_valid = false;
    uint32_t cur_inp;
    uint16_t cur_outp = 0, next_inp = 0;
    struct tisci_msg_rm_get_resource_range_req req = {{0}};
    struct tisci_msg_rm_get_resource_range_resp host_resp = {{0}};
    struct tisci_msg_rm_get_resource_range_resp all_resp = {{0}};

    if (cfg->s_ia == SCICLIENT_RM_DEV_NONE) {
        /* First node's interface must contain the source IRQ */
        cur_n = Sciclient_rmPsGetIrqNode(0u);
        if (cur_n == NULL) {
            valid = false;
        } else {
            cur_if = cur_n->p_if[Sciclient_rmPsGetIfIdx(0u)];
            if ((cfg->s_idx < cur_if->lbase) ||
                (cfg->s_idx >= (cur_if->lbase + cur_if->len))) {
                valid = false;
            }
        }
    }

    if ((valid == true) &&
        (Sciclient_rmPsGetPsp() <= 1U)) {
        /*
         * Route containing zero nodes is obviously invalid.  Route
         * containing one node is a direct route between peripheral
         * and destination processor which is an unmanaged route
         */
        valid = false;
    }

    if (valid == true) {
        if (Sciclient_rmIrqCheckLoop(cfg) == true) {
            /* There cannot be a route loop */
            valid = false;
        }
    }

    /* Validate input and output line usage of each node */
    for (i = 0u; (i < Sciclient_rmPsGetPsp()) && (valid == true); i++) {
        cur_n = Sciclient_rmPsGetIrqNode(i);
        cur_if = cur_n->p_if[Sciclient_rmPsGetIfIdx(i)];
        if (i < (Sciclient_rmPsGetPsp() - 1u)) {
            next_n = Sciclient_rmPsGetIrqNode(i + 1u);
            if (next_n == NULL) {
                valid = false;
                break;
            }
        }

        if (i > 0u) {
            /* All intermediate nodes between source and
             * destination must be interrupt routers, which are the
             * only programmable routing subsystems.  IAs are
             * programmable but are considered endpoints since
             * they're the entry point for subsystems which output
             * events onto the ETL bus */
            if (Sciclient_rmIrIsIr(cur_n->id) != true) {
                valid = false;
                break;
            }
        }

        if ((i == 0u) && (i < (Sciclient_rmPsGetPsp() - 1u))) {
            if (Sciclient_rmIaIsIa(cur_n->id) == true) {
                cur_outp_valid = false;
                next_inp_valid = false;
                next_inp = SCICLIENT_OUTP_TO_INP(cfg->vint,
                               cur_if->lbase,
                               cur_if->rbase);

                cur_outp = cfg->vint;
                cur_outp_valid = true;

                if ((next_n != NULL) && (Sciclient_rmIrInpIsFree(next_n->id, next_inp) ==
                    SystemP_SUCCESS)) {
                    next_inp_valid = true;
                }

                if (Sciclient_rmParamIsValid(cfg->valid_params,
                               TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID) ==
                    true) {
                    /*  cur_inp = cfg global event */
                    cur_inp = cfg->global_evt;
                    if (Sciclient_rmIaValidateGlobalEvt(cur_n->id,
                                      cur_inp) ==
                        SystemP_SUCCESS) {
                        if (Sciclient_rmPsSetInp(i, cur_inp) != SystemP_SUCCESS) {
                            valid = false;
                            break;
                        }
                    } else {
                        valid = false;
                        break;
                    }
                }
            } else {
                /* outp is from source peripheral and specified
                 * by configuration so always valid */
                cur_outp_valid = true;
                next_inp_valid = false;

                cur_outp = cfg->s_idx;
                next_inp = SCICLIENT_OUTP_TO_INP(cur_outp, cur_if->lbase,
                               cur_if->rbase);

                if ((next_n != NULL) && (Sciclient_rmIrInpIsFree(next_n->id, next_inp) ==
                    SystemP_SUCCESS)) {
                    next_inp_valid = true;
                }
            }
        }

        if ((i > 0u) && (i < (Sciclient_rmPsGetPsp() - 1u))) {
            /* Get the IR output resource range host assignments */
            req.secondary_host = cfg->host;
            req.type = cur_n->id;
            req.subtype = TISCI_RESASG_SUBTYPE_IR_OUTPUT;
            if (Sciclient_rmGetResourceRange(&req, &host_resp,
                    SystemP_WAIT_FOREVER) != SystemP_SUCCESS) {
                valid = false;
                break;
            }
            req.secondary_host = TISCI_HOST_ID_ALL;
            if (Sciclient_rmGetResourceRange(&req, &all_resp,
                    SystemP_WAIT_FOREVER) != SystemP_SUCCESS) {
                valid = false;
                break;
            }

            for (j = cur_if->lbase; j < cur_if->lbase + cur_if->len;
                 j++) {
                cur_outp_valid = false;
                next_inp_valid = false;

                cur_outp = j;
                next_inp = SCICLIENT_OUTP_TO_INP(cur_outp, cur_if->lbase,
                               cur_if->rbase);

                /* Check IR output against boardcfg ranges. First against
                 * the passed host then against HOST_ID_ALL if the passed
                 * host does not match.  Validate the input and output by
                 * checking the hardware if the output validates against
                 * the board configuration range. */
                if ((((cur_outp >= host_resp.range_start) &&
                      (cur_outp < host_resp.range_start + host_resp.range_num)) ||
                     ((cur_outp >= host_resp.range_start_sec) &&
                      (cur_outp < host_resp.range_start_sec +
                                  host_resp.range_num_sec))) ||
                    (((cur_outp >= all_resp.range_start) &&
                      (cur_outp < all_resp.range_start + all_resp.range_num)) ||
                     ((cur_outp >= all_resp.range_start_sec) &&
                      (cur_outp < all_resp.range_start_sec +
                                  all_resp.range_num_sec)))) {
                    if (Sciclient_rmIrOutpIsFree(cur_n->id, cur_outp) ==
                        SystemP_SUCCESS) {
                        cur_outp_valid = true;
                    }
                    if (Sciclient_rmIrInpIsFree(next_n->id, next_inp) ==
                        SystemP_SUCCESS) {
                        next_inp_valid = true;
                    }

                    if ((cur_outp_valid == true) &&
                        (next_inp_valid == true)) {
                        break;
                    }
                }
            }
        }

        if (i == (Sciclient_rmPsGetPsp() - 1u)) {
            /* inp always valid since it's the input to the
             * destination host processor which is assumed to be
             * valid as long as the node output is valid */
            cur_outp_valid = false;
            next_inp_valid = true;

            /* Get the IR output resource range host assignments */
            req.secondary_host = cfg->host;
            req.type = cur_n->id;
            req.subtype = TISCI_RESASG_SUBTYPE_IR_OUTPUT;
            if (Sciclient_rmGetResourceRange(&req, &host_resp,
                    SystemP_WAIT_FOREVER) != SystemP_SUCCESS) {
                valid = false;
                break;
            }
            req.secondary_host = TISCI_HOST_ID_ALL;
            if (Sciclient_rmGetResourceRange(&req, &all_resp,
                    SystemP_WAIT_FOREVER) != SystemP_SUCCESS) {
                valid = false;
                break;
            }

            cur_outp = SCICLIENT_INP_TO_OUTP(cfg->d_irq,
                           cur_if->rbase,
                           cur_if->lbase);

            /* Check IR output against boardcfg ranges. First against
             * the passed host then against HOST_ID_ALL if the passed
             * host does not match.  Validate the output by
             * checking the hardware if the output validates against
             * the board configuration range. */
            if ((((cur_outp >= host_resp.range_start) &&
                  (cur_outp < host_resp.range_start + host_resp.range_num)) ||
                 ((cur_outp >= host_resp.range_start_sec) &&
                  (cur_outp < host_resp.range_start_sec +
                              host_resp.range_num_sec))) ||
                (((cur_outp >= all_resp.range_start) &&
                  (cur_outp < all_resp.range_start + all_resp.range_num)) ||
                 ((cur_outp >= all_resp.range_start_sec) &&
                  (cur_outp < all_resp.range_start_sec +
                              all_resp.range_num_sec)))) {
                if ((cur_if->rid == cfg->d_id) &&
                    (cfg->d_irq >= (cur_if->rbase)) &&
                    (cfg->d_irq < (cur_if->rbase + cur_if->len))) {

                    if (Sciclient_rmIrOutpIsFree(cur_n->id, cur_outp) ==
                        SystemP_SUCCESS) {
                        cur_outp_valid = true;
                    }
                }
	    }
        }

        if ((cur_outp_valid == true) && (next_inp_valid == true)) {
            if (i < (Sciclient_rmPsGetPsp() - (1u))) {
                if (Sciclient_rmPsSetInp(i + (1u), next_inp) != SystemP_SUCCESS) {
                    valid = false;
                    break;
                }
            }
            if (Sciclient_rmPsSetOutp(i, cur_outp) != SystemP_SUCCESS) {
                valid = false;
                break;
            }
        } else {
            valid = false;
            break;
        }
    }

    return valid;
}

static int32_t Sciclient_rmIrqFindRoute(struct Sciclient_rmIrqCfg *cfg)
{
    int32_t r = SystemP_SUCCESS;
    bool search;
    uint16_t search_depth;
    uint16_t if_idx;
    bool node_clear;
    const struct Sciclient_rmIrqNode *cur_n;
    const struct Sciclient_rmIrqNode *root_n;
    const struct Sciclient_rmIrqIf *cur_if;

    /* Source and destination nodes cannot be IRs */
    if ((Sciclient_rmIrIsIr(cfg->s_id) == true) ||
        (Sciclient_rmIrIsIr(cfg->d_id) == true)) {
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS) {
        /* Retrieve source device's node */
        if ((cfg->s_ia != SCICLIENT_RM_DEV_NONE) &&
            (Sciclient_rmParamIsValid(cfg->valid_params,
                        TISCI_MSG_VALUE_RM_IA_ID_VALID) ==
             true) &&
            (Sciclient_rmParamIsValid(cfg->valid_params,
                        TISCI_MSG_VALUE_RM_VINT_VALID) ==
             true)) {
            r = Sciclient_rmIrqGetNode(cfg->s_ia, &cur_n);
        } else {
            r = Sciclient_rmIrqGetNode(cfg->s_id, &cur_n);
        }
        root_n = cur_n;
    }

    if (r == SystemP_SUCCESS) {
        search = true;
        Sciclient_rmPsInit();
        /* Start depth at 1 since no direct connections tracked by
         * IRQ tree */
        search_depth = 1u;
        if_idx = 0u;
    } else {
        search = false;
    }

    while (search == true) {
        node_clear = false;

        if (if_idx < cur_n->n_if) {
            r = Sciclient_rmIrqGetNodeItf(cur_n, if_idx, &cur_if);
            if (r != SystemP_SUCCESS) {
                break;
            }

            if ((Sciclient_rmIaIsIa(cur_n->id) == false) &&
                (Sciclient_rmPsIsEmpty() == true) &&
                ((cfg->s_idx < cur_if->lbase) ||
                 (cfg->s_idx >= (cur_if->lbase + cur_if->len)))) {
                /*
                 * Non-IA root node interfaces that do not
                 * contain the source IRQ are useless for
                 * search so move to next root node interface
                 */
                if_idx++;
            } else if (Sciclient_rmPsGetPsp() < search_depth) {
                r = Sciclient_rmPsPush(cur_n, if_idx);
                if (r != SystemP_SUCCESS) {
                    break;
                }

                r = Sciclient_rmIrqGetNode(cur_if->rid, &cur_n);
                if (cur_n == NULL) {
                    /* Node not found here means no outgoing
                     * connections from subsystem so not
                     * stored in tree.  Just pop back to
                     * node above */
                    node_clear = true;
                    /* Clear false negative */
                    r = SystemP_SUCCESS;
                } else {
                    if_idx = 0u;
                }
            } else {
                if ((cur_if->rid == cfg->d_id) &&
                    (cfg->d_irq >= (cur_if->rbase)) &&
                    (cfg->d_irq <
                     (cur_if->rbase + cur_if->len))) {
                    /*
                     * Only validate interrupt paths that
                     * end with a node connected to the
                     * destination input IRQ
                     */
                    r = Sciclient_rmPsPush(cur_n, if_idx);
                    if (r != SystemP_SUCCESS) {
                        break;
                    }

                    if (Sciclient_rmIrqRouteValidate(cfg) == true) {
                        break;
                    } else {
                        Sciclient_rmPsPop(&cur_n, &if_idx);
                    }
                }
                if_idx++;
            }
        } else {
            node_clear = true;
        }

        if (node_clear == true) {
            if (Sciclient_rmPsIsEmpty() == true) {
                /* Back at root node, increase search depth and
                 * reset the if_idx to restart the search at
                 * the next depth */
                search_depth++;
                cur_n = root_n;
                if_idx = 0u;

                if (search_depth >= Sciclient_rmPsGetMaxPsp()) {
                    r = SystemP_FAILURE;
                    break;
                }
            } else {
                r = Sciclient_rmPsPop(&cur_n, &if_idx);
                if (r != SystemP_SUCCESS) {
                    break;
                }
                if_idx++;
            }
        }
    }

    return r;
}

static int32_t Sciclient_rmIrqProgramRoute(struct Sciclient_rmIrqCfg   *cfg,
                                           bool                        map_vint)
{
    int32_t r = SystemP_SUCCESS;
    uint16_t i;
    const struct Sciclient_rmIrqNode *cur_n;
    uint32_t cur_inp, cur_outp;
    struct tisci_msg_rm_irq_set_req req;
    struct Sciclient_rmIaInst *ia_inst = NULL;
    struct Sciclient_rmIrInst *ir_inst = NULL;

    /* Program each intermediate node */
    for (i = 0u; i < Sciclient_rmPsGetPsp(); i++) {
        cur_n = Sciclient_rmPsGetIrqNode(i);
        cur_inp = Sciclient_rmPsGetInp(i);
        cur_outp = Sciclient_rmPsGetOutp(i);

        if ((i == 0u) && (Sciclient_rmIaIsIa(cur_n->id) == true) &&
            (map_vint == true)) {
            req.valid_params = (TISCI_MSG_VALUE_RM_IA_ID_VALID |
                                TISCI_MSG_VALUE_RM_VINT_VALID |
                                TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID |
                                TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID |
                                TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID);
            req.secondary_host = cfg->host;
            req.src_id = cfg->s_id;
            req.src_index = cfg->s_idx;
            req.ia_id = cfg->s_ia;
            req.vint = cur_outp;
            req.global_event = cur_inp;
            req.vint_status_bit_index = cfg->vint_sb;
            r = Sciclient_rmIrqSetRaw(&req,
                                      cfg->set_resp,
                                      SystemP_WAIT_FOREVER);
            if (r == SystemP_SUCCESS) {
                ia_inst = Sciclient_rmIaGetInst(cur_n->id);
                if (ia_inst != NULL) {
                    ia_inst->vint_usage_count[cur_outp]++;
                    if ((cur_outp == 0) && (cfg->vint_sb == 0)) {
                        ia_inst->v0_b0_evt = cur_inp - ia_inst->sevt_offset;
                    }
                }
            }
        }

        if (i > 0u) {
            req.valid_params = (TISCI_MSG_VALUE_RM_DST_ID_VALID |
                                TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID |
                                TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID);
            req.secondary_host = cfg->host;
            req.src_id = cur_n->id;
            req.src_index = cur_inp;
            req.dst_id = cur_n->id;
            req.dst_host_irq = cur_outp;
            r = Sciclient_rmIrqSetRaw(&req,
                                      cfg->set_resp,
                                      SystemP_WAIT_FOREVER);
            if ((r == SystemP_SUCCESS) && (cur_outp == 0)) {
                ir_inst = Sciclient_rmIrGetInst(cur_n->id);
                if (ir_inst != NULL) {
                    ir_inst->inp0_mapping = cur_outp;
                }
            }
        }

        if (r != SystemP_SUCCESS) {
            break;
        }
    }

    return r;
}

static int32_t Sciclient_rmIrqVintAdd(struct Sciclient_rmIrqCfg *cfg)
{
    int32_t r = SystemP_SUCCESS;
    bool vint_used;
    bool cfg_new_mapping = false;
    bool cfg_whole_route = false;
    struct Sciclient_rmIaInst *ia_inst = NULL;
    struct tisci_msg_rm_irq_set_req req;

    r = Sciclient_rmIrqIsVintRouteSet(cfg, &vint_used);

    if (r == SystemP_SUCCESS) {
        if ((vint_used == true) ||
            (Sciclient_rmIrqCfgIsEventToVintMappingOnly(cfg->valid_params) ==
             true)) {
            /*
             * VINT already has events mapped to it or
             * configuration is for a polled event.  Add a new
             * event mapping but do not configure any route
             * post the VINT since that's already been set up.
             */
            cfg_new_mapping = true;
        } else {
            /*
             * VINT has no events mapped.  Map the new event
             * within the VINT and then configure the entire
             * interrupt route post the VINT.
             */
            cfg_whole_route = true;
        }
    }

    if (cfg_new_mapping == true) {
        req.valid_params = (TISCI_MSG_VALUE_RM_IA_ID_VALID |
                            TISCI_MSG_VALUE_RM_VINT_VALID |
                            TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID |
                            TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID |
                            TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID);
        req.secondary_host = cfg->host;
        req.src_id = cfg->s_id;
        req.src_index = cfg->s_idx;
        req.ia_id = cfg->s_ia;
        req.vint = cfg->vint;
        req.global_event = cfg->global_evt;
        req.vint_status_bit_index = cfg->vint_sb;
        r = Sciclient_rmIrqSetRaw(&req,
                                  cfg->set_resp,
                                  SystemP_WAIT_FOREVER);
        if (r == SystemP_SUCCESS) {
            ia_inst = Sciclient_rmIaGetInst(cfg->s_ia);
            if (ia_inst != NULL) {
                ia_inst->vint_usage_count[cfg->vint]++;
                if ((cfg->vint == 0) && (cfg->vint_sb == 0)) {
                    ia_inst->v0_b0_evt = cfg->global_evt - ia_inst->sevt_offset;
                }
            }
        }
    }

    if (cfg_whole_route == true) {
        r = Sciclient_rmIrqFindRoute(cfg);
        if (r == SystemP_SUCCESS) {
            r = Sciclient_rmIrqProgramRoute(cfg, true);
        }
    }

    return r;
}

static int32_t Sciclient_rmUnmappedVintRouteCreate(
                struct Sciclient_rmIrqCfg    *cfg)
{
    int32_t r;
    uint8_t num_evts;

    r = Sciclient_rmIaVintGetInfo(cfg->s_ia, cfg->vint, &num_evts);
    if ((r == SystemP_SUCCESS) && (num_evts > 0U)) {
        /*
         * VINT cannot have any event to status bit mappings.
         * This route is allocated with a VINT which will have
         * its mapping setup at a later time.
         */
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS) {
        r = Sciclient_rmIrqFindRoute(cfg);
        if (r == SystemP_SUCCESS) {
            /*
             * Program route but do not configure a peripheral
             * OES register and the event to VINT status bit
             * mapping
             */
            r = Sciclient_rmIrqProgramRoute(cfg, false);
        }
    }

    return r;
}

static int32_t Sciclient_rmIrqProgramOesRegister(
                struct Sciclient_rmIrqCfg  *cfg)
{
    struct tisci_msg_rm_irq_set_req req;

    req.valid_params = (TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID |
                        TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID);
    req.secondary_host = cfg->host;
    req.src_id = cfg->s_id;
    req.src_index = cfg->s_idx;
    req.global_event = cfg->global_evt;

    return Sciclient_rmIrqSetRaw(&req,
                                 cfg->set_resp,
                                 SystemP_WAIT_FOREVER);
}

static int32_t Sciclient_rmIrqGetRoute(struct Sciclient_rmIrqCfg    *cfg)
{
    int32_t r = SystemP_SUCCESS;
    uint16_t search_depth;
    uint16_t if_idx;
    uint16_t cur_psp;
    uint16_t inp, next_inp;
    uint16_t outp, next_outp;
    bool search, rt_complete, push_node;
    const struct Sciclient_rmIrqNode *cur_n, *next_n;
    const struct Sciclient_rmIrqIf *cur_if;

    /* Source and destination nodes cannot be IRs */
    if ((Sciclient_rmIrIsIr(cfg->s_id) == true) ||
        (Sciclient_rmIrIsIr(cfg->d_id) == true)) {
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS) {
        if ((cfg->s_ia != SCICLIENT_RM_DEV_NONE) &&
            (Sciclient_rmParamIsValid(cfg->valid_params,
                        TISCI_MSG_VALUE_RM_IA_ID_VALID) ==
             true) &&
            (Sciclient_rmParamIsValid(cfg->valid_params,
                        TISCI_MSG_VALUE_RM_VINT_VALID) ==
             true)) {
            if ((Sciclient_rmParamIsValid(cfg->valid_params,
                            TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID) ==
                 true) &&
                (Sciclient_rmParamIsValid(cfg->valid_params,
                            TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID) ==
                 true)) {
                r = Sciclient_rmIaValidateMapping(cfg->host, cfg->s_ia,
                               cfg->vint, cfg->global_evt,
                               cfg->vint_sb);
            }

            if (r == SystemP_SUCCESS) {
                inp = cfg->global_evt;
                outp = cfg->vint;
                r = Sciclient_rmIrqGetNode(cfg->s_ia, &cur_n);
            }
        } else {
            inp = 0u;
            outp = cfg->s_idx;
            r = Sciclient_rmIrqGetNode(cfg->s_id, &cur_n);
        }
    }

    if (r == SystemP_SUCCESS) {
        search = true;
        rt_complete = false;
        Sciclient_rmPsInit();
    } else {
        search = false;
    }

    for (search_depth = 0u;
         (search_depth < Sciclient_rmPsGetMaxPsp()) && (search == true);
         search_depth++) {
        push_node = false;

        for (if_idx = 0u; if_idx < cur_n->n_if; if_idx++) {
            r = Sciclient_rmIrqGetNodeItf(cur_n, if_idx, &cur_if);
            if (r != SystemP_SUCCESS) {
                break;
            }

            if ((outp >= cur_if->lbase) &&
                (outp < (cur_if->lbase + cur_if->len))) {
                if ((cur_if->rid == cfg->d_id) &&
                    (SCICLIENT_OUTP_TO_INP(outp, cur_if->lbase,
                         cur_if->rbase) ==
                     cfg->d_irq)) {
                    rt_complete = true;
                    break;
                }

                if (Sciclient_rmIrIsIr(cur_if->rid) == true) {
                    r = Sciclient_rmIrqGetNode(cur_if->rid, &next_n);
                    if (r != SystemP_SUCCESS) {
                        break;
                    }

                    next_inp = SCICLIENT_OUTP_TO_INP(outp,
                                   cur_if->lbase,
                                   cur_if->rbase);
                    r = Sciclient_rmIrGetOutp(next_n->id, next_inp,
                               &next_outp);
                    if (r == SystemP_SUCCESS) {
                        push_node = true;
                        r = SystemP_SUCCESS;
                        break;
                    }

                    if (r != SystemP_FAILURE) {
                        break;
                    }
                }
            }
        }

        if (r != SystemP_SUCCESS) {
            break;
        }

        if ((rt_complete == true) || (push_node == true)) {
            /* Route passes through next
             * node so save current node
             * and index since it's valid */
            r = Sciclient_rmPsPush(cur_n, if_idx);
            if (r != SystemP_SUCCESS) {
                break;
            }

            if ((search_depth > 0u) ||
                ((search_depth == 0u) &&
                 (Sciclient_rmIaIsIa(cur_n->id) == true))) {
                /* Only push inp and outp to stack for
                 * intermediate routing subsystems or
                 * if the first node is an IA */
                cur_psp = Sciclient_rmPsGetPsp();
                r = Sciclient_rmPsSetInp(cur_psp - (1u), inp);
                if (r != SystemP_SUCCESS) {
                    break;
                }
                r = Sciclient_rmPsSetOutp(cur_psp - (1u),
                        outp);
                if (r != SystemP_SUCCESS) {
                    break;
                }
            }

            if (rt_complete == true) {
                break;
            } else {
                cur_n = next_n;
                inp = next_inp;
                outp = next_outp;
            }
        }
    }

    if (search_depth >= Sciclient_rmPsGetMaxPsp()) {
        r = SystemP_FAILURE;
    }

    return r;
}

static int32_t Sciclient_rmIrqDeleteRoute(struct Sciclient_rmIrqCfg    *cfg,
                                          bool                         unmap_vint)
{
    int32_t r = SystemP_SUCCESS;
    uint16_t i;
    const struct Sciclient_rmIrqNode *cur_n;
    uint16_t cur_inp, cur_outp;
    struct tisci_msg_rm_irq_release_req req;
    struct Sciclient_rmIaInst *ia_inst = NULL;
    struct Sciclient_rmIrInst *ir_inst = NULL;

    /* Delete connection through each intermediate node */
    for (i = 0u; i < Sciclient_rmPsGetPsp(); i++) {
        cur_n = Sciclient_rmPsGetIrqNode(i);
        cur_inp = Sciclient_rmPsGetInp(i);
        cur_outp = Sciclient_rmPsGetOutp(i);

        if ((i == 0u) && (Sciclient_rmIaIsIa(cur_n->id) == true) &&
            (unmap_vint == true)) {
            req.valid_params = (TISCI_MSG_VALUE_RM_IA_ID_VALID |
                                TISCI_MSG_VALUE_RM_VINT_VALID |
                                TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID |
                                TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID |
                                TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID);
            req.secondary_host = cfg->host;
            req.src_id = cfg->s_id;
            req.src_index = cfg->s_idx;
            req.ia_id = cfg->s_ia;
            req.vint = cur_outp;
            req.global_event = cur_inp;
            req.vint_status_bit_index = cfg->vint_sb;
            r = Sciclient_rmIrqReleaseRaw(&req,
                                          SystemP_WAIT_FOREVER);
            if (r == SystemP_SUCCESS) {
                ia_inst = Sciclient_rmIaGetInst(cur_n->id);
                if (ia_inst != NULL) {
                    ia_inst->vint_usage_count[cur_outp]--;
                    if (ia_inst->v0_b0_evt == cur_inp - ia_inst->sevt_offset) {
                        ia_inst->v0_b0_evt = SCICLIENT_RM_IA_GENERIC_EVT_RESETVAL;
                    }
                }
            }
        }

        if (i > 0u) {
            req.valid_params = (TISCI_MSG_VALUE_RM_DST_ID_VALID |
                                TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID |
                                TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID);
            req.secondary_host = cfg->host;
            req.src_id = cur_n->id;
            req.src_index = cur_inp;
            req.dst_id = cur_n->id;
            req.dst_host_irq = cur_outp;
            r = Sciclient_rmIrqReleaseRaw(&req,
                                          SystemP_WAIT_FOREVER);
            if ((r == SystemP_SUCCESS) && (cur_outp == 0)) {
                ir_inst = Sciclient_rmIrGetInst(cur_n->id);
                if (ir_inst != NULL) {
                    ir_inst->inp0_mapping = SCICLIENT_RM_IR_MAPPING_FREE;
                }
            }
        }
    }

    return r;
}

static int32_t Sciclient_rmIrqVintDelete(struct Sciclient_rmIrqCfg  *cfg)
{
    int32_t r = SystemP_SUCCESS;
    uint8_t num_evts;
    bool del_mapping = false;
    bool del_whole_route = false;
    struct Sciclient_rmIaInst *ia_inst = NULL;
    struct tisci_msg_rm_irq_release_req req;

    if (r == SystemP_SUCCESS) {
        r = Sciclient_rmIaVintGetInfo(cfg->s_ia, cfg->vint, &num_evts);
    }

    if (r == SystemP_SUCCESS) {
        if (num_evts == 0u) {
            /* No events to unmap */
            r = CSL_EBADARGS;
        } else if ((num_evts > 1u) ||
               (Sciclient_rmIrqCfgIsEventToVintMappingOnly(cfg->valid_params) ==
                true)) {
            /*
             * VINT has multiple events mapped to it or
             * configuration is for a polled event.  Delete
             * the mapping but do not delete any route post the
             * VINT since the remaining mappings need to the
             * entire route to remain configured.
             */
            del_mapping = true;
        } else {
            /*
             * VINT has one event mapped.  Delete the last event
             * within the VINT and then delete the entire
             * interrupt route post the VINT.
             */
            del_whole_route = true;
        }
    }

    if (del_mapping == true) {
        req.valid_params = (TISCI_MSG_VALUE_RM_IA_ID_VALID |
                            TISCI_MSG_VALUE_RM_VINT_VALID |
                            TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID |
                            TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID |
                            TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID);
        req.secondary_host = cfg->host;
        req.src_id = cfg->s_id;
        req.src_index = cfg->s_idx;
        req.ia_id = cfg->s_ia;
        req.vint = cfg->vint;
        req.global_event = cfg->global_evt;
        req.vint_status_bit_index = cfg->vint_sb;
        r = Sciclient_rmIrqReleaseRaw(&req,
                                      SystemP_WAIT_FOREVER);
        if (r == SystemP_SUCCESS) {
            ia_inst = Sciclient_rmIaGetInst(cfg->s_ia);
            if (ia_inst != NULL) {
                ia_inst->vint_usage_count[cfg->vint]--;
                if (ia_inst->v0_b0_evt == cfg->global_evt - ia_inst->sevt_offset) {
                    ia_inst->v0_b0_evt = SCICLIENT_RM_IA_GENERIC_EVT_RESETVAL;
                }
            }
        }
    }

    if (del_whole_route == true) {
        r = Sciclient_rmIrqGetRoute(cfg);
        if (r == SystemP_SUCCESS) {
            r = Sciclient_rmIrqDeleteRoute(cfg, true);
        }
    }

    return r;
}

static int32_t Sciclient_rmIrqUnmappedVintRouteDelete(
                struct Sciclient_rmIrqCfg   *cfg)
{
    int32_t r = SystemP_SUCCESS;
    uint8_t num_evts;

    r = Sciclient_rmIaVintGetInfo(cfg->s_ia, cfg->vint, &num_evts);
    if ((r == SystemP_SUCCESS) && (num_evts > 0U)) {
        /*
         * Can't delete route if there are events maps to
         * VINT status bits
         */
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS) {
        r = Sciclient_rmIrqGetRoute(cfg);
        if (r == SystemP_SUCCESS) {
            r = Sciclient_rmIrqDeleteRoute(cfg, false);
        }
    }

    return r;
}

static int32_t Sciclient_rmIrqClearOesRegister(
                struct Sciclient_rmIrqCfg    *cfg)
{
    struct tisci_msg_rm_irq_release_req req;

    req.valid_params = (TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID |
                        TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID);
    req.secondary_host = cfg->host;
    req.src_id = cfg->s_id;
    req.src_index = cfg->s_idx;
    req.global_event = cfg->global_evt;

    return Sciclient_rmIrqReleaseRaw(&req,
                                     SystemP_WAIT_FOREVER);
}

static struct Sciclient_rmIaInst *Sciclient_rmIaGetInst(uint16_t  id)
{
    struct Sciclient_rmIaInst *inst = NULL;
    uint16_t i;

    for (i = 0; i < SCICLIENT_RM_IA_NUM_INST; i++) {
        if (id == gRmIaInstances[i].dev_id) {
            inst = &gRmIaInstances[i];
            break;
        }
    }

    return inst;
}

static bool Sciclient_rmIaEvtRomMapped(const struct Sciclient_rmIaInst  *inst,
                                       uint16_t                         evt)
{
    bool rom_mapped = false;
    uint8_t i;

    if (inst->rom_usage != NULL) {
        for (i = 0u; i < inst->n_rom_usage; i++) {
            if ((inst->rom_usage[i].cleared == false) &&
                (evt == (inst->rom_usage[i].event - inst->sevt_offset))) {
                rom_mapped = true;
                inst->rom_usage[i].cleared = true;
                break;
            }
        }
    }

    return rom_mapped;
}

static int32_t Sciclient_rmIaValidateEvt(const struct Sciclient_rmIaInst    *inst,
                                         uint16_t                           evt,
                                         uint16_t                           vint,
                                         uint8_t                            vint_sb_index,
                                         bool                               in_use)
{
    int32_t r = SystemP_SUCCESS;
    volatile uint32_t *entry_int_map_lo;
    uint16_t reg_vint, reg_sb;

    if (evt >= inst->n_sevt) {
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS) {
#if defined (BUILD_MPU1_0) || defined(BUILD_MPU1_1)
        entry_int_map_lo = (volatile uint32_t *)((uint64_t)inst->imap + (uint64_t)SCICLIENT_IA_ENTRY_INTMAP_LO(evt));
#else
        entry_int_map_lo = (volatile uint32_t *)(inst->imap + SCICLIENT_IA_ENTRY_INTMAP_LO(evt));
#endif

        if (in_use == true) {
            /* Check if event is in use */
            reg_vint = CSL_REG32_FEXT(entry_int_map_lo,
                                      INTAGGR_IMAP_GEVI_IMAP_REGNUM);
            reg_sb = CSL_REG32_FEXT(entry_int_map_lo,
                                    INTAGGR_IMAP_GEVI_IMAP_BITNUM);
            if ((reg_vint == 0u) && (reg_sb == 0u)) {
                /*
                 * INTMAP register's default value is zero
                 * which signifies VINT 0 bit 0.  Validate
                 * against the instance's saved view of
                 * VINT 0 bit 0.
                 */
                if ((vint != reg_vint) &&
                    (vint_sb_index != reg_sb) &&
                    (evt != inst->v0_b0_evt)) {
                    r = CSL_EBADARGS;
                }
            } else {
                if ((vint != reg_vint) &&
                    (vint_sb_index != reg_sb)) {
                    r = CSL_EBADARGS;
                }
            }
        } else {
            /*
             * Check if event is free.  Do not return as in use if event is
             * mapped by ROM.  SYSFW will clear the mapping when it receives the
             * request to configure the IA.
             */
            if (((CSL_REG32_RD(entry_int_map_lo) != 0u) &&
                 (Sciclient_rmIaEvtRomMapped(inst, evt) == false)) ||
                (evt == inst->v0_b0_evt)) {
                r = CSL_EBADARGS;
            }
        }
    }

    return r;
}

static int32_t Sciclient_rmIaValidateGlobalEvt(uint16_t id,
                                               uint16_t global_evt)
{
    int32_t r = SystemP_SUCCESS;
    struct Sciclient_rmIaInst *inst = NULL;
    uint16_t evt;

    inst = Sciclient_rmIaGetInst(id);
    if (inst == NULL) {
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS) {
        evt = global_evt - inst->sevt_offset;
        /* Check if event is free */
        r = Sciclient_rmIaValidateEvt(inst, evt, 0u, 0u, false);
    }

    return r;
}

static int32_t Sciclient_rmIaValidateMapping(uint8_t   host,
                                             uint16_t  id,
                                             uint16_t  vint,
                                             uint16_t  global_evt,
                                             uint8_t   vint_sb_index)
{
    int32_t r = SystemP_SUCCESS;
    struct Sciclient_rmIaInst *inst = NULL;
    uint16_t evt;
    volatile uint32_t *entry_int_map_lo;
    uint16_t reg_vint, reg_sb;

    inst = Sciclient_rmIaGetInst(id);
    if (inst == NULL) {
        r = CSL_EBADARGS;
    } else {
        if (vint >= inst->n_vint) {
            r = CSL_EBADARGS;
        }
    }

    if ((r == SystemP_SUCCESS) && (vint_sb_index >= SCICLIENT_IA_VINT_MAX_BITS)) {
        r = CSL_EBADARGS;
    }

    if (r == SystemP_SUCCESS) {
        evt = global_evt - inst->sevt_offset;
        if (evt >= inst->n_sevt) {
            r = CSL_EBADARGS;
        }
    }

    if (r == SystemP_SUCCESS) {
#if defined (BUILD_MPU1_0) || defined(BUILD_MPU1_1)
        entry_int_map_lo = (volatile uint32_t *)((uint64_t)inst->imap + (uint64_t)SCICLIENT_IA_ENTRY_INTMAP_LO(evt));
#else
        entry_int_map_lo = (volatile uint32_t *)(inst->imap + SCICLIENT_IA_ENTRY_INTMAP_LO(evt));
#endif

        /* Check if event is in use */
        reg_vint = CSL_REG32_FEXT(entry_int_map_lo,
                                  INTAGGR_IMAP_GEVI_IMAP_REGNUM);
        reg_sb = CSL_REG32_FEXT(entry_int_map_lo,
                                INTAGGR_IMAP_GEVI_IMAP_BITNUM);
        if ((reg_vint == 0u) && (reg_sb == 0u)) {
            /*
             * INTMAP register's default value is zero which signifies VINT 0
             * bit 0.  Validate against the instance's saved view of VINT 0
             * bit 0.
             */
            if ((vint != reg_vint) &&
                (vint_sb_index != reg_sb) &&
                (evt != inst->v0_b0_evt)) {
                r = CSL_EBADARGS;
            }
        } else {
            if ((vint != reg_vint) &&
                (vint_sb_index != reg_sb)) {
                r = CSL_EBADARGS;
            }
        }
    }

    return r;
}

static int32_t Sciclient_rmIaVintGetInfo(uint16_t   id,
                                         uint16_t   vint,
                                         uint8_t    *n_evts)
{
    int32_t r = SystemP_SUCCESS;
    const struct Sciclient_rmIaInst *inst;

    if (n_evts == NULL) {
        r = CSL_EBADARGS;
    } else {
        inst = Sciclient_rmIaGetInst(id);
        if (inst == NULL) {
            r = CSL_EBADARGS;
        } else {
            if (vint >= inst->n_vint) {
                r = CSL_EBADARGS;
            }
        }
    }

    if (r == SystemP_SUCCESS) {
        *n_evts = inst->vint_usage_count[vint];
    }

    return r;
}

static bool Sciclient_rmIaIsIa(uint16_t id)
{
    bool r = false;

    if (Sciclient_rmIaGetInst(id) != NULL) {
        r = true;
    }

    return r;
}

static uint32_t Sciclient_rmIrIntControlReg(uint16_t    outp)
{
    return (outp * SCICLIENT_IR_INT_CONTROL_REG_STEP) +
           SCICLIENT_IR_INT_CONTROL_REG_OFFSET;
}

static struct Sciclient_rmIrInst *Sciclient_rmIrGetInst(uint16_t    id)
{
    struct Sciclient_rmIrInst *inst = NULL;
    uint16_t i;

    for (i = 0; i < SCICLIENT_RM_IR_NUM_INST; i++) {
        if (id == gRmIrInstances[i].dev_id) {
            inst = &gRmIrInstances[i];
            break;
        }
    }

    return inst;
}

static bool Sciclient_rmIrInpRomMapped(const struct Sciclient_rmIrInst  *inst,
                                       uint16_t                         inp)
{
    bool rom_mapped = false;
    uint8_t i;

    if (inst->rom_usage != NULL) {
        for (i = 0u; i < inst->n_rom_usage; i++) {
            if ((inst->rom_usage[i].cleared == false) &&
                (inp >= inst->rom_usage[i].inp_start) &&
                (inp < (inst->rom_usage[i].inp_start +
                        inst->rom_usage[i].length))) {
                rom_mapped = true;
                inst->rom_usage[i].cleared = true;
                break;
            }
        }
    }

    return rom_mapped;
}

static int32_t Sciclient_rmIrInpIsFree(uint16_t id,
                                       uint16_t inp)
{
    int32_t r = SystemP_SUCCESS;
    const struct Sciclient_rmIrInst *inst = NULL;
    uint16_t i;
    volatile uint32_t *int_ctrl_reg;
    uint16_t extracted_inp;

    inst = Sciclient_rmIrGetInst(id);
    if (inst == NULL) {
        r = CSL_EBADARGS;
    } else {
        if (inp >= inst->n_inp) {
            r = CSL_EBADARGS;
        }
    }

    if (r == SystemP_SUCCESS) {
        if (inp == 0u) {
            if (inst->inp0_mapping != SCICLIENT_RM_IR_MAPPING_FREE) {
                r = SystemP_FAILURE;
            }
        } else {
            /*
             * Check IR control registers to see if inp has been
             * mapped to an outp
             */
            for (i = 0u; i < inst->n_outp; i++) {
                int_ctrl_reg = (volatile uint32_t *)Sciclient_getIrAddr(inst->cfg, i);
                extracted_inp = CSL_REG32_FEXT(int_ctrl_reg,
                                               INTR_ROUTER_CFG_MUXCNTL_ENABLE);
                /*
                 * Do not return as in use if input is mapped by ROM.  SYSFW will
                 * clear the mapping when it receives the request to configure
                 * the IR.
                 */
                if ((extracted_inp == inp) &&
                    (Sciclient_rmIrInpRomMapped(inst, inp) == false)) {
                    /* Input in use */
                    r = SystemP_FAILURE;
                    break;
                }
            }
        }
    }

    return r;
}

static bool Sciclient_rmIrOutpRomMapped(const struct Sciclient_rmIrInst  *inst,
                                        uint16_t                         outp)
{
    bool rom_mapped = false;
    uint8_t i;

    if (inst->rom_usage != NULL) {
        for (i = 0u; i < inst->n_rom_usage; i++) {
            if ((inst->rom_usage[i].cleared == false) &&
                (outp >= inst->rom_usage[i].outp_start) &&
                (outp < (inst->rom_usage[i].outp_start +
                        inst->rom_usage[i].length))) {
                rom_mapped = true;
                inst->rom_usage[i].cleared = true;
                break;
            }
        }
    }

    return rom_mapped;
}

int32_t Sciclient_rmIrOutpIsFree(uint16_t    id,
                                 uint16_t    outp)
{
    int32_t r = SystemP_SUCCESS;
    struct Sciclient_rmIrInst *inst = NULL;
    volatile uint32_t *int_ctrl_reg;
    uint16_t extracted_inp;

    inst = Sciclient_rmIrGetInst(id);
    if (inst == NULL) {
        r = CSL_EBADARGS;
    } else {
        if (outp >= inst->n_outp) {
            r = CSL_EBADARGS;
        }
    }

    if (r == SystemP_SUCCESS) {
        int_ctrl_reg = (volatile uint32_t *) Sciclient_getIrAddr (inst->cfg, outp);
        extracted_inp = CSL_REG32_FEXT(int_ctrl_reg,
                                       INTR_ROUTER_CFG_MUXCNTL_ENABLE);
        /*
         * Do not return as in use if output is mapped by ROM.  SYSFW will
         * clear the mapping when it receives the request to configure
         * the IR.
         */
        if (((extracted_inp != 0u) &&
             (Sciclient_rmIrOutpRomMapped(inst, outp) == false)) ||
            (outp == inst->inp0_mapping)) {
            /*
             * MUX CONTROL register's default value is zero which
             * also signifies IR input zero.  Check the instance
             * for the output that has been routed input zero.
             */
            r = SystemP_FAILURE;
        }
    }

    return r;
}

static int32_t Sciclient_rmIrGetOutp(uint16_t   id,
                                     uint16_t   inp,
                                     uint16_t   *outp)
{
    int32_t r = SystemP_SUCCESS;
    const struct Sciclient_rmIrInst *inst = NULL;
    uint16_t i;
    volatile uint32_t *int_ctrl_reg;
    uint16_t extracted_inp;

    if (outp == NULL) {
        r = CSL_EBADARGS;
    } else {
        inst = Sciclient_rmIrGetInst(id);
        if (inst == NULL) {
            r = CSL_EBADARGS;
        } else {
            if (inp >= inst->n_inp) {
                r = CSL_EBADARGS;
            }
        }
    }

    if ((r == SystemP_SUCCESS) && (inp == 0u)) {
        /* Use instance's input zero mapping to find output */
        if (inst->inp0_mapping != SCICLIENT_RM_IR_MAPPING_FREE) {
            /* Input zero mapping is valid.  Return the output. */
            *outp = inst->inp0_mapping;
        } else {
            r = SystemP_FAILURE;
        }
    }

    if ((r == SystemP_SUCCESS) && (inp != 0u)) {
        r = SystemP_FAILURE;

        /*
         * Use IR control registers to find the output mapped to the
         * input
         */
        for (i = 0u; i < inst->n_outp; i++) {
            int_ctrl_reg = (volatile uint32_t *) Sciclient_getIrAddr (inst->cfg, i);
            extracted_inp = CSL_REG32_FEXT(int_ctrl_reg,
                                           INTR_ROUTER_CFG_MUXCNTL_ENABLE);
            if (inp == extracted_inp) {
                *outp = i;
                r = SystemP_SUCCESS;
                break;
            }
        }
    }

    return r;
}

static bool Sciclient_rmIrIsIr(uint16_t id)
{
    bool r = false;

    if (Sciclient_rmIrGetInst(id) != NULL) {
        r = true;
    }

    return r;
}

