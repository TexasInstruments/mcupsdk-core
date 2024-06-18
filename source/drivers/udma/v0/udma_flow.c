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
 *  \file udma_flow.c
 *
 *  \brief File containing the UDMA driver flow related APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/udma/udma_priv.h>

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

#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
static int32_t Udma_mappedFlowCheckParams(Udma_DrvHandleInt drvHandle,
                                          const Udma_FlowAllocMappedPrms *flowAllocMappedPrms);
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_flowAllocMapped(Udma_DrvHandle drvHandle,
                             Udma_FlowHandle flowHandle,
                             const Udma_FlowAllocMappedPrms *flowAllocMappedPrms)
{
    int32_t             retVal = UDMA_SOK;
#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
    uint32_t            mappedFlowNum = UDMA_FLOW_INVALID;
    Udma_DrvHandleInt   drvHandleInt = (Udma_DrvHandleInt) drvHandle;
    Udma_FlowHandleInt  flowHandleInt = (Udma_FlowHandleInt) flowHandle;

    /* Error check */
    if((NULL_PTR == drvHandleInt) ||
       (NULL_PTR == flowHandleInt) ||
       (NULL_PTR == flowAllocMappedPrms))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(drvHandleInt->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        retVal = Udma_mappedFlowCheckParams(drvHandleInt, flowAllocMappedPrms);
    }
    if(UDMA_SOK == retVal)
    {
        /* Allocate mapped ring */
        mappedFlowNum = Udma_rmAllocMappedRing(drvHandleInt, flowAllocMappedPrms->mappedFlowGrp, flowAllocMappedPrms->mappedChNum);
        if(mappedFlowNum != UDMA_RING_INVALID)
        {
            /* Subtract RX Ring Number Offset */
            mappedFlowNum -= drvHandleInt->rxChOffset;

            /* Assign values to handle object */
            flowHandleInt->drvHandle    = drvHandleInt;
            flowHandleInt->flowStart    = mappedFlowNum;
            flowHandleInt->flowCnt      = 1U;
            flowHandleInt->flowInitDone = UDMA_INIT_DONE;
            flowHandleInt->mappedFlowGrp= flowAllocMappedPrms->mappedFlowGrp;
            flowHandleInt->mappedChNum  = flowAllocMappedPrms->mappedChNum;
        }
        else
        {
            /* Alloc not done */
            retVal = UDMA_EALLOC;
        }
    }
#else
    retVal = UDMA_EFAIL;
#endif

    return (retVal);
}

int32_t Udma_flowFree(Udma_FlowHandle flowHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_FlowHandleInt  flowHandleInt = (Udma_FlowHandleInt) flowHandle;

    /* Error check */
    if(NULL_PTR == flowHandleInt)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(flowHandleInt->flowInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = flowHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_MAPPED_GROUP_INVALID == flowHandleInt->mappedFlowGrp)
        {
            retVal = UDMA_EFAIL;
        }
        else
        {
            /* Free Mapped Ring in devices like AM64x */
#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
            DebugP_assert(flowHandleInt->flowCnt == 1U);
            /* Add RX Ring Number Offset */
            Udma_rmFreeMappedRing(
                flowHandleInt->flowStart + drvHandle->rxChOffset,
                drvHandle,
                flowHandleInt->mappedFlowGrp,
                flowHandleInt->mappedChNum);
#else
            retVal = UDMA_EFAIL;
#endif
        }

        flowHandleInt->drvHandle    = (Udma_DrvHandleInt) NULL_PTR;
        flowHandleInt->flowStart    = UDMA_FLOW_INVALID;
        flowHandleInt->flowCnt      = 0U;
        flowHandleInt->flowInitDone = UDMA_DEINIT_DONE;
        flowHandleInt->mappedFlowGrp= UDMA_MAPPED_GROUP_INVALID;
        flowHandleInt->mappedChNum  = UDMA_DMA_CH_INVALID;
    }

    return (retVal);
}

int32_t Udma_flowAttach(Udma_DrvHandle drvHandle,
                        Udma_FlowHandle flowHandle,
                        uint32_t flowStart,
                        uint32_t flowCnt)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandleInt = (Udma_DrvHandleInt) drvHandle;
    Udma_FlowHandleInt  flowHandleInt = (Udma_FlowHandleInt) flowHandle;

    /* Error check */
    if((NULL_PTR == drvHandleInt) ||
       (NULL_PTR == flowHandleInt) ||
       (0U == flowCnt))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(drvHandleInt->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Assign values to handle object */
        flowHandleInt->drvHandle    = drvHandleInt;
        flowHandleInt->flowStart    = flowStart;
        flowHandleInt->flowCnt      = flowCnt;
        flowHandleInt->flowInitDone = UDMA_INIT_DONE;
        flowHandleInt->mappedFlowGrp= UDMA_MAPPED_GROUP_INVALID;
        flowHandleInt->mappedChNum  = UDMA_DMA_CH_INVALID;
    }

    return (retVal);
}

int32_t Udma_flowAttachMapped(Udma_DrvHandle drvHandle,
                              Udma_FlowHandle flowHandle,
                              uint32_t mappepdFlowNum,
                              const Udma_FlowAllocMappedPrms *flowAllocMappedPrms)
{
    int32_t             retVal = UDMA_SOK;
#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
    Udma_DrvHandleInt   drvHandleInt = (Udma_DrvHandleInt) drvHandle;
    Udma_FlowHandleInt  flowHandleInt = (Udma_FlowHandleInt) flowHandle;

    /* Error check */
    if((NULL_PTR == drvHandleInt) ||
       (NULL_PTR == flowHandleInt) ||
       (NULL_PTR == flowAllocMappedPrms))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(drvHandleInt->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        retVal = Udma_mappedFlowCheckParams(drvHandleInt, flowAllocMappedPrms);
    }


    if(UDMA_SOK == retVal)
    {
        /* Assign values to handle object */
        flowHandleInt->drvHandle    = drvHandleInt;
        flowHandleInt->flowStart    = mappepdFlowNum;
        flowHandleInt->flowCnt      = 1U;
        flowHandleInt->flowInitDone = UDMA_INIT_DONE;
        flowHandleInt->mappedFlowGrp= flowAllocMappedPrms->mappedFlowGrp;
        flowHandleInt->mappedChNum  = flowAllocMappedPrms->mappedChNum;
    }
#else
    retVal = UDMA_EFAIL;
#endif
    return (retVal);
}

int32_t Udma_flowDetach(Udma_FlowHandle flowHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_FlowHandleInt  flowHandleInt = (Udma_FlowHandleInt) flowHandle;

    /* Error check */
    if(NULL_PTR == flowHandleInt)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(flowHandleInt->flowInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        flowHandleInt->drvHandle    = (Udma_DrvHandleInt) NULL_PTR;
        flowHandleInt->flowStart    = UDMA_FLOW_INVALID;
        flowHandleInt->flowCnt      = 0U;
        flowHandleInt->flowInitDone = UDMA_DEINIT_DONE;
        flowHandleInt->mappedFlowGrp= UDMA_MAPPED_GROUP_INVALID;
        flowHandleInt->mappedChNum  = UDMA_DMA_CH_INVALID;
    }

    return (retVal);
}

int32_t Udma_flowConfig(Udma_FlowHandle flowHandle,
                        uint32_t flowIdx,
                        const Udma_FlowPrms *flowPrms)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_FlowHandleInt  flowHandleInt = (Udma_FlowHandleInt) flowHandle;
    struct tisci_msg_rm_udmap_flow_cfg_req              rmFlowReq;
    struct tisci_msg_rm_udmap_flow_cfg_resp             rmFlowResp;
    struct tisci_msg_rm_udmap_flow_size_thresh_cfg_req  rmOptFlowReq;
    struct tisci_msg_rm_udmap_flow_size_thresh_cfg_resp rmOptFlowResp;

    /* Error check */
    if((NULL_PTR == flowHandleInt) ||
       (flowHandleInt->flowInitDone != UDMA_INIT_DONE) ||
       (NULL_PTR == flowPrms))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = flowHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        /* Flow Idx is relative to flowStart and must be less than flowCnt */
        if(flowIdx >= flowHandleInt->flowCnt)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Invalid flow index!!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        rmFlowReq.valid_params          = TISCI_MSG_VALUE_RM_UDMAP_FLOW_EINFO_PRESENT_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_PSINFO_PRESENT_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_ERROR_HANDLING_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_DESC_TYPE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_SOP_OFFSET_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_HI_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_LO_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_HI_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_LO_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_HI_SEL_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_LO_SEL_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_HI_SEL_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_LO_SEL_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ0_SZ0_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ1_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ2_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ3_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_PS_LOCATION_VALID;
        rmFlowReq.nav_id                = drvHandle->devIdUdma;
        rmFlowReq.flow_index            = (uint16_t)(flowHandleInt->flowStart + flowIdx);
        rmFlowReq.rx_einfo_present      = flowPrms->einfoPresent;
        rmFlowReq.rx_psinfo_present     = flowPrms->psInfoPresent;
        rmFlowReq.rx_error_handling     = flowPrms->errorHandling;
        rmFlowReq.rx_desc_type          = flowPrms->descType;
        rmFlowReq.rx_ps_location        = flowPrms->psLocation;
        rmFlowReq.rx_sop_offset         = flowPrms->sopOffset;
        rmFlowReq.rx_dest_qnum          = flowPrms->defaultRxCQ;
        rmFlowReq.rx_src_tag_hi         = flowPrms->srcTagHi;
        rmFlowReq.rx_src_tag_lo         = flowPrms->srcTagLo;
        rmFlowReq.rx_src_tag_hi_sel     = flowPrms->srcTagHiSel;
        rmFlowReq.rx_src_tag_lo_sel     = flowPrms->srcTagLoSel;
        rmFlowReq.rx_dest_tag_hi        = flowPrms->destTagHi;
        rmFlowReq.rx_dest_tag_lo        = flowPrms->destTagLo;
        rmFlowReq.rx_dest_tag_hi_sel    = flowPrms->destTagHiSel;
        rmFlowReq.rx_dest_tag_lo_sel    = flowPrms->destTagLoSel;
        rmFlowReq.rx_fdq0_sz0_qnum      = flowPrms->fdq0Sz0Qnum;
        rmFlowReq.rx_fdq1_qnum          = flowPrms->fdq1Qnum;
        rmFlowReq.rx_fdq2_qnum          = flowPrms->fdq2Qnum;
        rmFlowReq.rx_fdq3_qnum          = flowPrms->fdq3Qnum;

        /* Config RX flow */
        retVal = Sciclient_rmUdmapFlowCfg(
                     &rmFlowReq, &rmFlowResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] RX flow config failed!!!\r\n");
        }

        rmOptFlowReq.valid_params       = TISCI_MSG_VALUE_RM_UDMAP_FLOW_SIZE_THRESH0_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_SIZE_THRESH1_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_SIZE_THRESH2_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ0_SZ1_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ0_SZ2_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ0_SZ3_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_FLOW_SIZE_THRESH_EN_VALID;
        rmOptFlowReq.nav_id             = drvHandle->devIdUdma;
        rmOptFlowReq.flow_index         = (uint16_t)(flowHandleInt->flowStart + flowIdx);
        /*
         * The rx_size_thresh0, rx_size_thresh1 and rx_size_thresh2 are shifted right when writing
         * to align with the spec which says the value is shifted left when being
         * compared to the packet size.
         */
        rmOptFlowReq.rx_size_thresh0    = (flowPrms->sizeThresh0 >> UDMA_RFLOW_RX_SIZE_THRESH_VAL_SHIFT);
        rmOptFlowReq.rx_size_thresh1    = (flowPrms->sizeThresh1 >> UDMA_RFLOW_RX_SIZE_THRESH_VAL_SHIFT);
        rmOptFlowReq.rx_size_thresh2    = (flowPrms->sizeThresh2 >> UDMA_RFLOW_RX_SIZE_THRESH_VAL_SHIFT);
        rmOptFlowReq.rx_fdq0_sz1_qnum   = flowPrms->fdq0Sz1Qnum;
        rmOptFlowReq.rx_fdq0_sz2_qnum   = flowPrms->fdq0Sz2Qnum;
        rmOptFlowReq.rx_fdq0_sz3_qnum   = flowPrms->fdq0Sz3Qnum;
        rmOptFlowReq.rx_size_thresh_en  = flowPrms->sizeThreshEn;

        /* Config optional RX flow params */
        retVal += Sciclient_rmUdmapFlowSizeThreshCfg(
                      &rmOptFlowReq, &rmOptFlowResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            DebugP_logError("[UDMA] RX flow threshold config failed!!!\r\n");
        }
    }

    return (retVal);
}

uint32_t Udma_flowGetNum(Udma_FlowHandle flowHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            flowNum = UDMA_FLOW_INVALID;
    Udma_FlowHandleInt  flowHandleInt = (Udma_FlowHandleInt) flowHandle;

    /* Error check */
    if((NULL_PTR == flowHandleInt) ||
       (flowHandleInt->flowInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }

    if(UDMA_SOK == retVal)
    {
        flowNum = flowHandleInt->flowStart;
    }

    return (flowNum);
}

uint32_t Udma_flowGetCount(Udma_FlowHandle flowHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            flowCnt = UDMA_FLOW_INVALID;
    Udma_FlowHandleInt  flowHandleInt = (Udma_FlowHandleInt) flowHandle;

    /* Error check */
    if((NULL_PTR == flowHandleInt) ||
       (flowHandleInt->flowInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }

    if(UDMA_SOK == retVal)
    {
        flowCnt = flowHandleInt->flowCnt;
    }

    return (flowCnt);
}

void UdmaFlowPrms_init(Udma_FlowPrms *flowPrms, uint32_t chType)
{
    (void) chType;  /* MISRAC fix: could be used for future. So not removed */
    if(NULL_PTR != flowPrms)
    {
        flowPrms->rxChHandle    = (Udma_ChHandle) NULL_PTR;
        flowPrms->einfoPresent  = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_EINFO_NOT_PRESENT;
        flowPrms->psInfoPresent = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PSINFO_NOT_PRESENT;
        flowPrms->errorHandling = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_ERR_RETRY;
        flowPrms->descType      = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_DESC_HOST;
        flowPrms->psLocation    = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_PS_END_PD;
        flowPrms->sopOffset     = 0U;
        flowPrms->defaultRxCQ   = UDMA_RING_INVALID;
        flowPrms->srcTagLo      = 0U;
        flowPrms->srcTagLoSel   = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SRC_SELECT_SRC_TAG;
        flowPrms->srcTagHi      = 0U;
        flowPrms->srcTagHiSel   = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_SRC_SELECT_FLOW_ID;
        flowPrms->destTagLo     = 0U;
        flowPrms->destTagLoSel  = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_DEST_SELECT_DEST_TAG_LO;
        flowPrms->destTagHi     = 0U;
        flowPrms->destTagHiSel  = TISCI_MSG_VALUE_RM_UDMAP_RX_FLOW_DEST_SELECT_DEST_TAG_HI;
        flowPrms->sizeThreshEn  = 0U;
        flowPrms->fdq0Sz0Qnum   = UDMA_RING_INVALID;
        flowPrms->fdq1Qnum      = UDMA_RING_INVALID;
        flowPrms->fdq2Qnum      = UDMA_RING_INVALID;
        flowPrms->fdq3Qnum      = UDMA_RING_INVALID;
        flowPrms->sizeThresh0   = 0U;
        flowPrms->sizeThresh1   = 0U;
        flowPrms->sizeThresh2   = 0U;
        flowPrms->fdq0Sz1Qnum   = UDMA_RING_INVALID;
        flowPrms->fdq0Sz2Qnum   = UDMA_RING_INVALID;
        flowPrms->fdq0Sz3Qnum   = UDMA_RING_INVALID;
    }

    return;
}

#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
static int32_t Udma_mappedFlowCheckParams(Udma_DrvHandleInt drvHandle,
                                          const Udma_FlowAllocMappedPrms *flowAllocMappedPrms)
{
    int32_t     retVal = UDMA_SOK;

    DebugP_assert(flowAllocMappedPrms != NULL_PTR);

    /* Check for valid RX mapped group */
    if((UDMA_MAPPED_GROUP_INVALID == flowAllocMappedPrms->mappedFlowGrp) ||
       (flowAllocMappedPrms->mappedFlowGrp < UDMA_NUM_MAPPED_TX_GROUP) ||
       (flowAllocMappedPrms->mappedFlowGrp >= (UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP)))
    {
        retVal = UDMA_EINVALID_PARAMS;
        DebugP_logError("[UDMA] Incorrect Mapped Flow Group!!!\r\n");
    }

    if(UDMA_DMA_CH_INVALID == flowAllocMappedPrms->mappedChNum)
    {
        retVal = UDMA_EINVALID_PARAMS;
        DebugP_logError("[UDMA] Invalid Mapped Channel number!!!\r\n");
    }

    return (retVal);
}
#endif
