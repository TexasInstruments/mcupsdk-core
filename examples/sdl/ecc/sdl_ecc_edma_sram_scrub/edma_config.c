/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_dpl_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "edma_rti_sram_scrub.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define APP_EDMA_PARAM_A_CNT    (APP_CHUNK_SIZE_BYTES)
#define APP_EDMA_PARAM_B_CNT    (APP_SRAM_SCRUB_BANK_SIZE_BYTES / APP_CHUNK_SIZE_BYTES)
#define APP_EDMA_PARAM_C_CNT    (APP_SRAM_SCRUB_NUM_BANKS)

/* RTI is configured (in sysconfig) to trigger the following */
#define APP_EDMA_TRIG_CHANNEL   (DMA_TRIG_XBAR_EDMA_MODULE_0)

/* EDMA event queue to be used  */
#define APP_EDMA_EVT_QUEUE_NO   (0U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Buffer to hold read chunk data */
volatile uint8_t gBuffer[APP_CHUNK_SIZE_BYTES] __attribute__((aligned(64U), section(".data.tcma"))) = { 0U };

/* EDMA objects */
static uint32_t gEdmaBaseAddr;
static uint32_t gEdmaParam0;
static uint32_t gEdmaParam1;
static uint32_t gEdmaRegionId;
static uint32_t gEdmaTcc;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void App_configDma(void)
{
    EDMACCPaRAMEntry    edmaParam;
    uint32_t            dmaChannel;
    int32_t             status = SystemP_SUCCESS;

    gEdmaBaseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(gEdmaBaseAddr != 0U);

    gEdmaRegionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(gEdmaRegionId < SOC_EDMA_NUM_REGIONS);

    dmaChannel = APP_EDMA_TRIG_CHANNEL;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaChannel);
    DebugP_assert(status == SystemP_SUCCESS);

    gEdmaTcc    = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &gEdmaTcc);
    DebugP_assert(status == SystemP_SUCCESS);

    gEdmaParam0 = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &gEdmaParam0);
    DebugP_assert(status == SystemP_SUCCESS);

    gEdmaParam1 = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &gEdmaParam1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Request channel */
    EDMA_configureChannelRegion(gEdmaBaseAddr, gEdmaRegionId, EDMA_CHANNEL_TYPE_DMA, dmaChannel, gEdmaTcc, 
                                gEdmaParam0, APP_EDMA_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(gEdmaBaseAddr, gEdmaRegionId, dmaChannel);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) APP_SRAM_SCRUB_START_ADDR;
    edmaParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)gBuffer);
    edmaParam.aCnt          = (uint16_t) APP_EDMA_PARAM_A_CNT;
    edmaParam.bCnt          = (uint16_t) APP_EDMA_PARAM_B_CNT;
    edmaParam.cCnt          = (uint16_t) APP_EDMA_PARAM_C_CNT;
    edmaParam.bCntReload    = (uint16_t) APP_EDMA_PARAM_B_CNT;
    edmaParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(APP_EDMA_PARAM_A_CNT);
    edmaParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);    /* Copy to gBuffer itself */
    edmaParam.srcCIdx       = (int16_t) APP_EDMA_PARAM_A_CNT;
    edmaParam.destCIdx      = (int16_t) 0;                     /* Copy to gBuffer itself */
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(APP_EDMA_PARAM_A_CNT);
    edmaParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaParam.opt          |= EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | 
                                (((((uint32_t)gEdmaTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    /* For repeated transfers,
     * set param1 as well with same transfer parameters and
     * link param1 to param0 and param1 to param1 itself */
    EDMA_setPaRAM(gEdmaBaseAddr, gEdmaParam0, &edmaParam);
    EDMA_setPaRAM(gEdmaBaseAddr, gEdmaParam1, &edmaParam);

    EDMA_linkChannel(gEdmaBaseAddr, gEdmaParam0, gEdmaParam1);
    EDMA_linkChannel(gEdmaBaseAddr, gEdmaParam1, gEdmaParam1);

    EDMA_enableTransferRegion(gEdmaBaseAddr, gEdmaRegionId, dmaChannel, EDMA_TRIG_MODE_EVENT);
}

void App_getEdmaChunkAddr(uint32_t* startAddrPtr, uint32_t* endAddrPtr)
{
    EDMACCPaRAMEntry    edmaParam;
    
    EDMA_getPaRAM(gEdmaBaseAddr, gEdmaParam0, &edmaParam);

    /* `edmaParam.srcAddr` denotes the address of the next DMA transfer source address.
     * i.e, the end address of last DMA chunk transfer. 
     * In case of param reload after completion of `aCnt*bCnt*cCnt` transfer, this resets to start address.
     * Hence handle that case separately. */
    if(edmaParam.srcAddr == APP_SRAM_SCRUB_START_ADDR) {
        *endAddrPtr = APP_SRAM_SCRUB_START_ADDR + (APP_EDMA_PARAM_A_CNT * APP_EDMA_PARAM_B_CNT * APP_EDMA_PARAM_C_CNT);
    } else {
        *endAddrPtr = edmaParam.srcAddr;
    }
    *startAddrPtr = *endAddrPtr - APP_EDMA_PARAM_A_CNT;
}

uint8_t* App_getEdmaBuffAddr(void)
{
    return (void *)gBuffer;
}

/* Nothing past this point */
