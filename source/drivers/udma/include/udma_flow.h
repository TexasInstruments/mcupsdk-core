/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

/**
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_FLOW_MODULE UDMA Flow API
 *            This is UDMA driver RX flow related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file udma_flow.h
 *
 *  \brief UDMA flow related parameters and API.
 */

#ifndef UDMA_FLOW_H_
#define UDMA_FLOW_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Default flow ID */
#define UDMA_DEFAULT_FLOW_ID            (0x3FFFU)

/** \brief Macro used to specify that flow ID is invalid.*/
#define UDMA_FLOW_INVALID               ((uint32_t) 0xFFFF0000U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/**
 *  \brief UDMA RX channel mapped flow alloc parameters.
 */
typedef struct
{
    uint32_t                mappedFlowGrp;
    /**< The Mapped flow group to use when channel type is
     *   #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   The driver will allocate mapped flows from this group.
     *   Refer \ref Udma_MappedRxGrpSoc macro for details about mapped RX flow groups.
     */
    uint32_t                mappedChNum;
    /**< The assigned mapped channel number when channel type is
     *   #UDMA_CH_TYPE_RX_MAPPED.
     *
     *   This is used to allocate the corresponding mapped flow for the particular channel.
     *   The driver will allocate from the mapped flows which are dedicated
     *   for this particular channel.
     */
} Udma_FlowAllocMappedPrms;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA mapped flow allocation API. In devices like AM64x,
 *  flows are tied to channels. This API is used to allocate a single flow
 *  from the mapped free flows which are dedicated for a particular channel.
 *
 *  Note: Allocation of mapped flows have to be done after Udma_chOpen, Since the
 *  mapped flow have to be allocated from the dedicated flows for the particular channel.
 *
 *  \param drvHandle           [IN] UDMA driver handle pointer passed during
 *                                  #Udma_init
 *  \param flowHandle          [IN/OUT] UDMA flow handle. The caller need to
 *                                  allocate memory for this object and pass this
 *                                  pointer to all further APIs. The caller should
 *                                  not change any parameters as this is owned and
 *                                  maintained by the driver.
 *  \param flowAllocMappedPrms [IN] UDMA mapped flow alloc parameters.
 *                                  This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_flowAllocMapped(Udma_DrvHandle drvHandle,
                             Udma_FlowHandle flowHandle,
                             const Udma_FlowAllocMappedPrms *flowAllocMappedPrms);

/**
 *  \brief UDMA free flows.
 *
 *  Freeup the flow resources.
 *
 *  Note: This API can be used to free mapped flow also,
 *  which are allocated using #Udma_flowAllocMapped.
 *
 *  \param flowHandle   [IN] UDMA flow handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_flowFree(Udma_FlowHandle flowHandle);

/**
 *  \brief UDMA flow attach API. This API is used to attach to an already
 *  allocated flow. This API differs from flow alloc API in
 *  this aspect - it doesn't allocate resource from RM. Once the flow is
 *  attached to, #Udma_flowConfig API can be used to configure the flow
 *  through sciclient/DMSC API.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3418)
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *  \param flowHandle   [IN/OUT] UDMA flow handle. The caller need to
 *                           allocate memory for this object and pass this
 *                           pointer to all further APIs. The caller should
 *                           not change any parameters as this is owned and
 *                           maintained by the driver.
 *  \param flowStart    [IN] Flow index to attach to. This paramter should
 *                           be a valid flow number allowed to be used by a
 *                           core. The driver doesn't check the validity of
 *                           this field at the time of attach. But the flow
 *                           config API may fail if wrong flow index is used or
 *                           when the core doesn't own the flow as per DMSC board
 *                           config.
 *  \param flowCnt      [IN] Flow count - to attach to more than 1 flow
 *                           which is contiguous from flow start. This is
 *                           provided to allow a single handle to manage
 *                           multiple contiguous flows
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_flowAttach(Udma_DrvHandle drvHandle,
                        Udma_FlowHandle flowHandle,
                        uint32_t flowStart,
                        uint32_t flowCnt);

/**
 *  \brief UDMA mapped flow attach API. This API is used to attach to an already
 *  allocated mapped flow. This API differs from mapped flow alloc API in
 *  this aspect - it doesn't allocate resource from RM. Once the flow is
 *  attached to, #Udma_flowConfig API can be used to configure the flow
 *  through sciclient/DMSC API.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3418)
 *
 *  \param drvHandle           [IN] UDMA driver handle pointer passed during
 *                                  #Udma_init
 *  \param flowHandle          [IN/OUT] UDMA flow handle. The caller need to
 *                                  allocate memory for this object and pass this
 *                                  pointer to all further APIs. The caller should
 *                                  not change any parameters as this is owned and
 *                                  maintained by the driver.
 *  \param mappepdFlowNum      [IN] Mapped flow index to attach to. This paramter should
  *                                   be a valid mapped flow number allowed to be used by a
  *                                 core and by the mapped channel. The driver doesn't check the
  *                                 validity of this field at the time of attach. But the flow
  *                                 config API may fail if wrong mapped flow index is used or
  *                                 when the core doesn't own the mapped flow as per DMSC board
  *                                 config or if the mapped flow dosen't belong to the deicated
  *                                 flows for the particular channel.
 *  \param flowAllocMappedPrms [IN] UDMA mapped flow alloc parameters.
 *                                  This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_flowAttachMapped(Udma_DrvHandle drvHandle,
                              Udma_FlowHandle flowHandle,
                              uint32_t mappepdFlowNum,
                              const Udma_FlowAllocMappedPrms *flowAllocMappedPrms);

/**
 *  \brief UDMA flow detach API.
 *
 *  Since no allocation is done in attach, this API just clears up the
 *  flow handle.
 *
 *  Note: This API can be used to detach mapped flow also,
 *  which are attached using #Udma_flowAttachMapped.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3418)
 *
 *  \param flowHandle   [IN] UDMA flow handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_flowDetach(Udma_FlowHandle flowHandle);

/**
 *  \brief   This API configures the flow configurations.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2599)
 *
 *  \param flowHandle   [IN] UDMA flow handle pointer
 *  \param flowIdx      [IN] Since multiple flows are allocated and also
 *                           contiguously, all the flows needs to be configured
 *                           one after the other. This is the relative index
 *                           from the start of the flow allocated.
 *                           If the index goes beyond what is allocated, then
 *                           the function returns error.
 *                           Note: In case of configuring the default flow and
 *                           mapped flow(in devices like AM64x),
 *                           this should be set to "zero".
 *  \param flowPrms     [IN] Pointer to flow configuration.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_flowConfig(Udma_FlowHandle flowHandle,
                        uint32_t flowIdx,
                        const Udma_FlowPrms *flowPrms);

/**
 *  \brief Returns the start flow number managed by this flow handle.
 *
 *  Note: In case off mapped flow(in devices like AM64x), this returns the
 *  mapped flow number.
 *
 *  \param flowHandle   [IN] UDMA flow handle.
 *                           This parameter can't be NULL.
 *
 *  \return Start flow number on success or #UDMA_FLOW_INVALID on error
 */
uint32_t Udma_flowGetNum(Udma_FlowHandle flowHandle);

/**
 *  \brief Returns the number of flows managed by this flow handle.
 *
 *  Note: In case of mapped flow(in devices like AM64x), this
 *  always returns 1, since only one mapped flow is managed by a flow handle.
 *
 *  Requirement: DOX_REQ_TAG(PDK-3718)
 *
 *  \param flowHandle   [IN] UDMA flow handle.
 *                           This parameter can't be NULL.
 *
 *  \return Flow count on success or #UDMA_FLOW_INVALID on error
 */
uint32_t Udma_flowGetCount(Udma_FlowHandle flowHandle);

/*
 * Structure Init functions
 */
/**
 *  \brief Udma_FlowPrms structure init function.
 *
 *  \param flowPrms     [IN] Pointer to #Udma_FlowPrms structure.
 *  \param chType       [IN] UDMA channel type. Refer \ref Udma_ChType.
 *
 */
void UdmaFlowPrms_init(Udma_FlowPrms *flowPrms, uint32_t chType);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief Opaque UDMA flow object.
 */
typedef struct Udma_FlowObject_t
{
    uintptr_t rsv[6U];
    /**< reserved, should NOT be modified by end users */
} Udma_FlowObject;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_FLOW_H_ */

/** @} */
