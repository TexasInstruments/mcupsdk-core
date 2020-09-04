/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 *  \file cbuff_edma.c
 *
 *      The file implements the CBUFF Driver EDMA Interface.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <drivers/cbuff.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                       Local Function Declarations                          */
/* ========================================================================== */
static int32_t CBUFF_setupEDMAShadowLink
(
    CBUFF_Session*                          ptrSession,
    uint32_t                                chId,
    uint32_t                                linkChId,
    EDMACCPaRAMEntry*                      config
);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  @b Description
 *  @n
 *      The function is used to setup the EDMA channel with the
 *      specified configuration
 *
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[in]  chId
 *      EDMA Channel Identifier
 *  @param[in]  linkChId
 *      EDMA Channel Identifer to be linked
 *  @param[in]  config
 *      EDMA Parameter set configuration
 *  @param[in]  transferCompletionCallbackFxn
 *      Option Transfer completion callback function to be invoked once
 *      the transfer is complete.
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - EDMA No Error (EDMA_NO_ERR)
 *  @retval
 *      Error   - EDMA Error code
 */
static int32_t CBUFF_setupEDMAShadowLink
(
    CBUFF_Session*                          ptrSession,
    uint32_t                                chId,
    uint32_t                                linkChId,
    EDMACCPaRAMEntry*                      config
)
{
    uint32_t baseAddr, regionId;

    baseAddr = EDMA_getBaseAddr(ptrSession->sessionCfg.edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(ptrSession->sessionCfg.edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    EDMA_setPaRAM(baseAddr, linkChId, config);

    EDMA_linkChannel(baseAddr, chId, linkChId);

    EDMA_linkChannel(baseAddr, linkChId, chId);

    return CBUFF_STATUS_SUCCESS;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the EDMA Transfer from the specified
 *      source address to the CBUFF FIFO.
 *
 *  @param[in]  ptrSession
 *      Pointer to the Session
 *  @param[in]  srcAddress
 *      Source Address where the data to be transfered is located
 *  @param[in]  transferSize
 *      Size of the data to be transferred to the FIFO
 *  @param[in]  isLast
 *      Flag which if set indicates this is the last transaction
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - < 0 [Error code`: EDMA/CBUFF Error]
 */
int32_t CBUFF_configEDMA
(
    CBUFF_Session*      ptrSession,
    uint32_t            srcAddress,
    uint32_t            transferSize,
    bool                isLast
)
{
    uint32_t                chId;
    uint32_t                chainChannel;
    uint32_t                linkChannel;
    uint32_t                prevLinkChannel;
    CBUFF_EDMAChannelCfg    cbuffEDMAChannelCfg;
    uint8_t                 edmaChannelCount;
    int32_t                 errCode = CBUFF_STATUS_SUCCESS;
    CBUFF_EDMAInfo          edmaInfo;
    EDMACCPaRAMEntry       edmaParam;
    uint32_t                baseAddr, regionId;

    baseAddr = EDMA_getBaseAddr(ptrSession->sessionCfg.edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(ptrSession->sessionCfg.edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Initialize the error code: */
    errCode = 0;

    /* Sanity Check: Was a valid transfer size specified? */
    if(transferSize != 0U)
    {
        /* Keep track of the EDMA Channels which have been configured. */
        edmaChannelCount = ptrSession->edmaChannelCount;

        /* Sanity Check: Do we have space to record this information */
        DebugP_assert (edmaChannelCount < CBUFF_EDMA_MAX_NUM_CHANNELS);

        /* Initialize the EDMA Information: */
        memset ((void*)&edmaInfo, 0, sizeof(CBUFF_EDMAInfo));

        /* Is this the first EDMA Channel being allocated? */
        if (edmaChannelCount == 0U)
        {
            /* YES: Setup the flag */
            edmaInfo.isFirstEDMAChannel = true;
        }
        else
        {
            /* NO: Reset the flag */
            edmaInfo.isFirstEDMAChannel = false;
        }
        edmaInfo.edmaHandle = ptrSession->sessionCfg.edmaHandle;
        edmaInfo.dmaNum     = ptrSession->dmaNum;

        /* Allocate an EDMA channel using the application registered callback function */
        errCode = ptrSession->sessionCfg.allocateEDMAChannelFxn (&edmaInfo, &cbuffEDMAChannelCfg);
        DebugP_assert(CBUFF_STATUS_SUCCESS == errCode);

        /* First Channel: This is a special case and the we always need to have this hooked up
        * via the CBUFF EDMA Physical channel identifier. Applications should know about this
        * and also because we still want the application to allocate the shadow channels */
        if (edmaChannelCount == 0)
        {
            bool isValidEDMAChannel = false;

            /* Sanity Check: Did the application give us the correct CBUFF EDMA Channel identifier. */
            if (cbuffEDMAChannelCfg.chainChannelsId == ptrSession->ptrDriverMCB->hwAttrs->cbuffChannelId[edmaInfo.dmaNum])
            {
                /* YES: Excellent we are done the application gave us the correct EDMA channel identifier */
                isValidEDMAChannel = true;
            }

            /* Was there a match? */
            if (isValidEDMAChannel == false)
            {
                /* NO: This is an invalid use case; we could return an invalid argument but modified
                * the error code to clearly show that the invalid EDMA allocation was incorrect. */
                errCode = CBUFF_EDMA_FAIL;
                DebugP_assert(CBUFF_EDMA_FAIL == errCode);
            }
        }

        /* Track the EDMA Information which is being added */
        memcpy ((void *)&ptrSession->edmaTrackingEntry[edmaChannelCount].cbuffEDMAChannelCfg, (void*)&cbuffEDMAChannelCfg,
                sizeof(CBUFF_EDMAChannelCfg));
        ptrSession->edmaTrackingEntry[edmaChannelCount].transferSize = transferSize;
        ptrSession->edmaTrackingEntry[edmaChannelCount].srcAddress   = srcAddress;

        /* Setup the channel identifier which is to be used. */
        chId = cbuffEDMAChannelCfg.chainChannelsId;

        /* Populate the EDMA Channel Parameter Set Configuration: */
        EDMA_ccPaRAMEntry_init(&edmaParam);
        edmaParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *) srcAddress);
        edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *) gCbuffAttrs->fifoBaseAddr);
        edmaParam.aCnt          = (uint16_t) ((2U * transferSize) & 0xFFFFU);
        edmaParam.bCnt          = (uint16_t) ptrSession->numChirpsPerFrame;
        edmaParam.cCnt          = (uint16_t) 1U;
        edmaParam.bCntReload    = (uint16_t) edmaParam.bCnt;
        edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
        edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
        edmaParam.linkAddr      = 0xFFFFU;
        edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
        edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);

        /*
        * DMA  = 1: FIFO Dst addressing within an array wraps around upon reaching FIFO width.
        * FWID = EDMA_TPCC_OPT_FWID_FIFOWIDTH128BIT
        * Bydefault SYNC_A type is set.
        */
        edmaParam.opt          |= ( EDMA_TPCC_OPT_DAM_MASK | (EDMA_TPCC_OPT_FWID_FIFOWIDTH128BIT << EDMA_TPCC_OPT_FWID_SHIFT) |
                                ((((uint32_t)chId) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

        /* Enable early completion when chaining. This helps reduce chaining latency and
        * while this helps reduce in general the total transfer time, more importantly
        * it reduces the probability of underflow when LVDS is used as HSI */
        if (isLast)
        {
            edmaParam.opt          |= (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK);
        }
        else
        {
            edmaParam.opt          |= (EDMA_TPCC_OPT_TCCHEN_MASK | EDMA_TPCC_OPT_ITCCHEN_MASK);
        }

        /* cbuff channel is event triggered, chain channels are not */
        if (edmaChannelCount == 0U)
        {
            EDMA_enableTransferRegion(baseAddr, regionId, chId, EDMA_TRIG_MODE_EVENT);
        }

        /* Configure the EDMA Channel: */
        EDMA_setPaRAM(baseAddr, chId, &edmaParam);

        /* Link the channels: */
        linkChannel = cbuffEDMAChannelCfg.shadowLinkChannelsId;
        errCode = CBUFF_setupEDMAShadowLink(ptrSession, chId, linkChannel, &edmaParam);
        if (errCode == CBUFF_STATUS_SUCCESS)
        {
            /* Do we need to chain the channels? We need to skip the the head of the chain. */
            if (edmaChannelCount != 0U)
            {
                /* YES: Get the previous chain & link channel from the saved EDMA configuration */
                chainChannel    = ptrSession->edmaTrackingEntry[edmaChannelCount - 1U].cbuffEDMAChannelCfg.chainChannelsId;
                prevLinkChannel = ptrSession->edmaTrackingEntry[edmaChannelCount - 1U].cbuffEDMAChannelCfg.shadowLinkChannelsId;

                /* Chain the channels */
                HW_WR_FIELD32(baseAddr + EDMA_TPCC_OPT((uint32_t)chainChannel), EDMA_TPCC_OPT_TCC, chId);

                /* Setup the chain in the linked param set as well */
                HW_WR_FIELD32(baseAddr + EDMA_TPCC_OPT((uint32_t)prevLinkChannel), EDMA_TPCC_OPT_TCC, chId);
            }

            /* Increment the number of channels which are being setup: */
            ptrSession->edmaChannelCount = edmaChannelCount + 1;
        }
    }

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The function is used to close the EDMA channels which have been configured
 *      This is invoked once the CBUFF module is closed
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF Session
 *
 *  \ingroup CBUFF_DRIVER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void CBUFF_closeEDMA (CBUFF_Session* ptrSession)
{
    uint8_t     index;
    uint32_t    baseAddr, regionId;
    int32_t     status = CBUFF_STATUS_SUCCESS;

    baseAddr = EDMA_getBaseAddr(ptrSession->sessionCfg.edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(ptrSession->sessionCfg.edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Sanity Check: Ensure that the back pointer to the driver is valid */
    DebugP_assert (ptrSession->ptrDriverMCB != NULL);

    /* Disable all the chained channels */
    for (index = 0U; index < ptrSession->edmaChannelCount; index++)
    {
        /* Free the EDMA resources managed by driver. */
        status = EDMA_freeDmaChannel(ptrSession->sessionCfg.edmaHandle, &ptrSession->edmaTrackingEntry[index].cbuffEDMAChannelCfg.chainChannelsId);
        DebugP_assert(status == CBUFF_STATUS_SUCCESS);

        status = EDMA_freeTcc(ptrSession->sessionCfg.edmaHandle, &ptrSession->edmaTrackingEntry[index].cbuffEDMAChannelCfg.chainChannelsId);
        DebugP_assert(status == CBUFF_STATUS_SUCCESS);

        status = EDMA_freeParam(ptrSession->sessionCfg.edmaHandle, &ptrSession->edmaTrackingEntry[index].cbuffEDMAChannelCfg.chainChannelsId);
        DebugP_assert(status == CBUFF_STATUS_SUCCESS);

        status = EDMA_freeParam(ptrSession->sessionCfg.edmaHandle, &ptrSession->edmaTrackingEntry[index].cbuffEDMAChannelCfg.shadowLinkChannelsId);
        DebugP_assert(status == CBUFF_STATUS_SUCCESS);

        /* Inform the application that the EDMA channel has been freed. */
        ptrSession->sessionCfg.freeEDMAChannelFxn (&ptrSession->edmaTrackingEntry[index].cbuffEDMAChannelCfg);

        /* Reset the tracking entry: */
        memset ((void *)&ptrSession->edmaTrackingEntry[index], 0, sizeof(CBUFF_EDMATrackingEntry));
    }

    /* Reset the EDMA Channel count in the driver; all the channels have been cleaned up. */
    ptrSession->edmaChannelCount = 0;

    return;
}

