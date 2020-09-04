/*
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include "hwa_fft.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define FIXED_POINT_FRACTIONAL_BITS         (23U)   /* 1 for sign bit */

/* EDMA Event queue to be used  */
#define HWAFFT_EDMA_EVT_QUEUE_NO            (0U)

/* Number of blocks to use for interleaving DMA and fixed/float conversion */
#define HWAFFT_DMA_INTERLEAVE_BLOCKS        (2U)
#define HWAFFT_DMA_INTERLEAVE_BLOCKS_SHIFT  (1U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/*
 * ISR Callback
 */
static void HWAFFT_doneCallback(uint32_t threadIdx, void *arg);

/*
 * EDMA functions
 */
static void HWAFFT_edmaTrigger(struct HWAFFT_ResObject *resObj,
                               uint32_t edmaBaseAddr,
                               uint32_t edmaRegionId,
                               void *dest,
                               void *src,
                               uint32_t numBytes,
                               uint32_t isRd);
static void HWAFFT_edmaWait(struct HWAFFT_ResObject *resObj,
                            uint32_t edmaBaseAddr,
                            uint32_t edmaRegionId,
                            uint32_t isRd);

/*
 * Helper functions
 */
static void HWAFFT_fftSrcConvertCopy(struct HWAFFT_Object *fftObj,
                                     const float *srcBuffer,
                                     uint32_t numSamples);
static void HWAFFT_fftDestConvertCopy(struct HWAFFT_Object *fftObj,
                                      float *destBuffer,
                                      uint32_t numSamples);
static uint32_t HWAFFT_log2Approx(uint32_t x);
static uint32_t HWAFFT_maxBit(uint32_t x);

/* Converts float -> 24 bit signed number format */
static inline int32_t float_to_fixed(float input)
{
    return (int32_t)(input * ((1 << FIXED_POINT_FRACTIONAL_BITS) - 1));
}

/* Converts 24 bit signed number format -> float */
static inline float fixed_to_float(int32_t input)
{
    return ((float)input / (float)((1 << FIXED_POINT_FRACTIONAL_BITS) - 1));
}

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t HWAFFT_init(struct HWAFFT_Object *fftObj,
                    HWA_Handle hwaHandle,
                    EDMA_Handle edmaHandle,
                    int32_t *fixedPointBuf)
{
    int32_t                     status = SystemP_SUCCESS, i;
    uint32_t                    edmaBaseAddr, edmaRegionId;
    HWA_ParamConfig            *paramCfg;
    HWA_CommonConfig           *commonCfg;
    EDMACCPaRAMEntry          *edmaPrms;
    uint32_t                    numSamples = 1024U; /* Will be overwritten */
    struct HWAFFT_ResObject    *resObj;

    commonCfg = &fftObj->commonCfg;

    edmaBaseAddr = EDMA_getBaseAddr(edmaHandle);
    DebugP_assert(edmaBaseAddr != 0);

    edmaRegionId = EDMA_getRegionId(edmaHandle);
    DebugP_assert(edmaRegionId < SOC_EDMA_NUM_REGIONS);

    /*
     * Resource Init
     */
    resObj = &fftObj->resObj[HWAFFT_RES_IDX_FFT];
    resObj->paramIdx        = 0U;
    resObj->rdDmaCh         = EDMA_RESOURCE_ALLOC_ANY;
    resObj->wrDmaCh         = EDMA_RESOURCE_ALLOC_ANY;
    resObj->rdTcc           = EDMA_RESOURCE_ALLOC_ANY;
    resObj->wrTcc           = EDMA_RESOURCE_ALLOC_ANY;
    resObj->rdParamId       = EDMA_RESOURCE_ALLOC_ANY;
    resObj->wrParamId       = EDMA_RESOURCE_ALLOC_ANY;
    resObj->rdMemBankAddr   = CSL_DSS_HWA_DMA0_RAM_BANK0_BASE;
    resObj->wrMemBankAddr   = CSL_DSS_HWA_DMA0_RAM_BANK1_BASE;
    status = EDMA_allocDmaChannel(edmaHandle, &resObj->rdDmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocDmaChannel(edmaHandle, &resObj->wrDmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocTcc(edmaHandle, &resObj->rdTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocTcc(edmaHandle, &resObj->wrTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocParam(edmaHandle, &resObj->rdParamId);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocParam(edmaHandle, &resObj->wrParamId);
    DebugP_assert(status == SystemP_SUCCESS);

    resObj = &fftObj->resObj[HWAFFT_RES_IDX_IFFT];
    resObj->paramIdx        = 1U;
    resObj->rdDmaCh         = EDMA_RESOURCE_ALLOC_ANY;
    resObj->wrDmaCh         = EDMA_RESOURCE_ALLOC_ANY;
    resObj->rdTcc           = EDMA_RESOURCE_ALLOC_ANY;
    resObj->wrTcc           = EDMA_RESOURCE_ALLOC_ANY;
    resObj->rdParamId       = EDMA_RESOURCE_ALLOC_ANY;
    resObj->wrParamId       = EDMA_RESOURCE_ALLOC_ANY;
    resObj->rdMemBankAddr   = CSL_DSS_HWA_DMA0_RAM_BANK2_BASE;
    resObj->wrMemBankAddr   = CSL_DSS_HWA_DMA0_RAM_BANK3_BASE;
    status = EDMA_allocDmaChannel(edmaHandle, &resObj->rdDmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocDmaChannel(edmaHandle, &resObj->wrDmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocTcc(edmaHandle, &resObj->rdTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocTcc(edmaHandle, &resObj->wrTcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocParam(edmaHandle, &resObj->rdParamId);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_allocParam(edmaHandle, &resObj->wrParamId);
    DebugP_assert(status == SystemP_SUCCESS);

    /*
     * State variables
     */
    fftObj->hwaHandle       = hwaHandle;
    fftObj->edmaBaseAddr    = edmaBaseAddr;
    fftObj->edmaRegionId    = edmaRegionId;
    fftObj->fixedPointBuf   = fixedPointBuf;

    for(i = 0; i < HWAFFT_RES_IDX_MAX; i++)
    {
        resObj = &fftObj->resObj[i];
        /* Request EDMA channels */
        EDMA_configureChannelRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
            resObj->rdDmaCh, resObj->rdTcc, resObj->rdParamId, HWAFFT_EDMA_EVT_QUEUE_NO);

        EDMA_configureChannelRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
            resObj->wrDmaCh, resObj->wrTcc, resObj->wrParamId, HWAFFT_EDMA_EVT_QUEUE_NO);

        /* Program Read Param Set */
        edmaPrms = &resObj->rdDmaParam;
        EDMA_ccPaRAMEntry_init(edmaPrms);
        edmaPrms->linkAddr      = 0xFFFFU;
        /* Program Write Param Set */
        edmaPrms = &resObj->wrDmaParam;
        EDMA_ccPaRAMEntry_init(edmaPrms);
        edmaPrms->linkAddr      = 0xFFFFU;

        /* Init param set */
        paramCfg = &resObj->paramCfg;
        memset(paramCfg , 0, sizeof(*paramCfg));
        paramCfg->triggerMode = HWA_TRIG_MODE_DMA;
        paramCfg->triggerSrc = resObj->rdDmaCh;
        paramCfg->accelMode = HWA_ACCELMODE_FFT;
        paramCfg->source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(resObj->rdMemBankAddr);
        paramCfg->source.srcSign = HWA_SAMPLES_SIGNED;
        paramCfg->source.srcAcnt = numSamples - 1U;
        paramCfg->source.srcBcnt = 0U;
        paramCfg->source.srcBIdx = 0U;  /* dont care as bcnt is 0 */
        paramCfg->source.srcWidth = HWA_SAMPLES_WIDTH_32BIT;
        paramCfg->source.srcScale = 0U;
        paramCfg->source.srcIQSwap = 0U;
        paramCfg->dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(resObj->wrMemBankAddr);
        paramCfg->dest.dstSkipInit = 0U;
        paramCfg->dest.dstAcnt = numSamples - 1U;
        paramCfg->dest.dstAIdx = 2U * sizeof(int32_t); /* x2 for real and complex output */
        paramCfg->dest.dstBIdx = 0U;    /* dont care as bcnt is 0 */
        paramCfg->dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
        paramCfg->dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
        paramCfg->dest.dstSign = HWA_SAMPLES_SIGNED;
        paramCfg->dest.dstScale = 8U;
        paramCfg->dest.dstIQswap = 0U;
        if(i == HWAFFT_RES_IDX_FFT)
        {
            paramCfg->source.srcAIdx = sizeof(int32_t);
            paramCfg->source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
            paramCfg->source.srcConjugate = HWA_FEATURE_BIT_DISABLE;    /* no conjugate */
            paramCfg->dest.dstConjugate = HWA_FEATURE_BIT_DISABLE;      /* no conjugate */
        }
        else
        {
            paramCfg->source.srcAIdx = 2U * sizeof(int32_t);   /* x2 for complex input */
            paramCfg->source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
            paramCfg->source.srcConjugate = HWA_FEATURE_BIT_ENABLE;     /* conjugate */
            paramCfg->dest.dstConjugate = HWA_FEATURE_BIT_ENABLE;       /* conjugate */
        }
        paramCfg->accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
        paramCfg->accelModeArgs.fftMode.fftSize = HWAFFT_log2Approx(numSamples);
        paramCfg->accelModeArgs.fftMode.butterflyScaling = 0xFFFU; /* 12-bit scaling for each stage - 0 means saturate: clip MSB, 1 means scale down by 2: round-off LSB */
        paramCfg->accelModeArgs.fftMode.bpmEnable = HWA_FEATURE_BIT_DISABLE;
        paramCfg->accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_DISABLE;
        paramCfg->accelModeArgs.fftMode.windowStart = 0U;
        paramCfg->accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC;
        paramCfg->accelModeArgs.fftMode.preProcCfg.complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        paramCfg->accelModeArgs.fftMode.preProcCfg.dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET;
        paramCfg->accelModeArgs.fftMode.preProcCfg.interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET;
        paramCfg->accelModeArgs.fftMode.preProcCfg.dcSubSelect = HWA_DCSUB_SELECT_DCEST;
        paramCfg->accelModeArgs.fftMode.postProcCfg.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        paramCfg->accelModeArgs.fftMode.postProcCfg.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; /* FFT output */
        paramCfg->accelModeArgs.fftMode.postProcCfg.histogramMode = HWA_HISTOGRAM_MODE_DISABLED;
        paramCfg->accelModeArgs.fftMode.postProcCfg.histogramScaleSelect = 0U;
        paramCfg->accelModeArgs.fftMode.postProcCfg.histogramSizeSelect = 0U;
        status += HWA_configParamSet(fftObj->hwaHandle, resObj->paramIdx, paramCfg, NULL);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("Error: HWA_configParamSet failed with error: %d!!\r\n", status);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Init Common Params */
        memset(commonCfg, 0, sizeof(*commonCfg));
        commonCfg->configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG |
                                HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                                HWA_COMMONCONFIG_MASK_LFSRSEED;
        commonCfg->fftConfig.twidDitherEnable = 1U;
        commonCfg->fftConfig.lfsrSeed = 11U;
        commonCfg->paramStartIdx = 0U;  /* Will be overwritten */
        commonCfg->paramStopIdx = 0U;   /* Will be overwritten */
        commonCfg->numLoops = 1U;
        status = HWA_configCommon(fftObj->hwaHandle, commonCfg);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Error: HWA_configCommon failed with error: %d\r\n", status);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Enable done interrupt */
        status = SemaphoreP_constructBinary(&fftObj->doneSem, 0);
        DebugP_assert(status == SystemP_SUCCESS);
        status = HWA_enableDoneInterrupt(fftObj->hwaHandle, 0, HWAFFT_doneCallback, fftObj);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Error: HWA_enableDoneInterrupt failed with error: %d\r\n", status);
        }
    }

    return (status);
}

int32_t HWAFFT_deinit(struct HWAFFT_Object *fftObj)
{
    int32_t                     status = SystemP_SUCCESS, i;
    struct HWAFFT_ResObject    *resObj;

    for(i = 0; i < HWAFFT_RES_IDX_MAX; i++)
    {
        resObj = &fftObj->resObj[i];
        /* Free EDMA channels */
        EDMA_freeChannelRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
            resObj->rdDmaCh, EDMA_TRIG_MODE_MANUAL, resObj->rdTcc, HWAFFT_EDMA_EVT_QUEUE_NO);
        EDMA_freeChannelRegion(
            fftObj->edmaBaseAddr, fftObj->edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
            resObj->wrDmaCh, EDMA_TRIG_MODE_MANUAL, resObj->wrTcc, HWAFFT_EDMA_EVT_QUEUE_NO);
    }

    status += HWA_disableDoneInterrupt(fftObj->hwaHandle, 0);
    SemaphoreP_destruct(&fftObj->doneSem);

    return (status);
}

int32_t HWAFFT_fftRealToCmplxFloat(struct HWAFFT_Object *fftObj,
                                   float *destBuffer,
                                   const float *srcBuffer,
                                   uint32_t numSamples)
{
    int32_t                     status = SystemP_SUCCESS;
    DSSHWACCRegs               *ctrlBaseAddr;
    DSSHWACCPARAMRegs          *paramBaseAddr;
    struct HWAFFT_ResObject    *resObj;
    uint32_t                    numFFTStages = HWAFFT_log2Approx(numSamples);
    uint32_t                    fftStartCycle, startCycle, endCycle;

    resObj = &fftObj->resObj[HWAFFT_RES_IDX_FFT];

    fftStartCycle = CycleCounterP_getCount32();

    /* Float to Fixed and DMA to HWA memory */
    HWAFFT_fftSrcConvertCopy(fftObj, srcBuffer, numSamples);

    /* Param update */
    startCycle = CycleCounterP_getCount32();;
    ctrlBaseAddr = HWA_getCommonCtrlAddr(fftObj->hwaHandle);
    DebugP_assert(ctrlBaseAddr != NULL);
    paramBaseAddr = HWA_getParamSetAddr(fftObj->hwaHandle, resObj->paramIdx);
    DebugP_assert(paramBaseAddr != NULL);
    CSL_FINSR(paramBaseAddr->SRCA, SRCA_SRCACNT_END, SRCA_SRCACNT_START, (numSamples - 1U));
    CSL_FINSR(paramBaseAddr->DSTA, DSTA_DSTACNT_END, DSTA_DSTACNT_START, (numSamples - 1U));
    CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_FFTSIZE_END, BFLYFFT_FFTSIZE_START, numFFTStages);
    /* 12-bit scaling for each stage - 0 means saturate: clip MSB, 1 means scale down by 2: round-off LSB */
    CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_BFLY_SCALING_END, BFLYFFT_BFLY_SCALING_START, 0xFFFU);
    /* Common control update */
    CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX, PARAM_RAM_IDX_PARAM_START_IDX_END, PARAM_RAM_IDX_PARAM_START_IDX_START, resObj->paramIdx);
    CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX, PARAM_RAM_IDX_PARAM_END_IDX_END, PARAM_RAM_IDX_PARAM_END_IDX_START, resObj->paramIdx);

    /* Enable HWA */
    status += HWA_enable(fftObj->hwaHandle, 1U);
    status += HWA_reset(fftObj->hwaHandle);

    /* Trigger and wait */
    status += HWA_setDMA2ACCManualTrig(fftObj->hwaHandle, resObj->rdDmaCh);
    SemaphoreP_pend(&fftObj->doneSem, SystemP_WAIT_FOREVER);

    /* Disable HWA */
    status += HWA_enable(fftObj->hwaHandle, 0U);

    endCycle = CycleCounterP_getCount32();
    resObj->hwaCycle = endCycle - startCycle;

    /* DMA from HWA memory and then fixed to float */
    HWAFFT_fftDestConvertCopy(fftObj, destBuffer, numSamples);

    resObj->totalCycle = CycleCounterP_getCount32() - fftStartCycle;

    return (status);
}

int32_t HWAFFT_ifftCmplxToRealFloat(struct HWAFFT_Object *fftObj,
                                    float *destBuffer,
                                    const float *srcBuffer,
                                    uint32_t numSamples)
{
    int32_t                     status = SystemP_SUCCESS;
    int32_t                     i, k, conjIdx;
    int32_t                    *rdMemBankAddr, *wrMemBankAddr;
    DSSHWACCRegs               *ctrlBaseAddr;
    DSSHWACCPARAMRegs          *paramBaseAddr;
    struct HWAFFT_ResObject    *resObj;
    uint32_t                    numFFTStages = HWAFFT_log2Approx(numSamples);
    uint32_t                    scalFactor = (1U << numFFTStages);
    uint32_t                    fftStartCycle, startCycle, endCycle;

    resObj = &fftObj->resObj[HWAFFT_RES_IDX_IFFT];

    /* Convert float to fixed */
    fftStartCycle = startCycle = CycleCounterP_getCount32();
    rdMemBankAddr = (int32_t *) resObj->rdMemBankAddr;
    fftObj->fixedPointBuf[0] = 0;                 /* Im(X(0)) is always zero */
    fftObj->fixedPointBuf[1] = float_to_fixed(srcBuffer[1]);
    /* N/2 is stored in Im(X(0)) */
    fftObj->fixedPointBuf[numSamples] = 0;        /* Im(X(N/2)) is always zero */
    fftObj->fixedPointBuf[numSamples+1] = float_to_fixed(srcBuffer[0]);
    for(i = 1; i < (numSamples >> 1U); i++)
    {
        k = (i << 1U);
        conjIdx = (numSamples << 1U) - k;

        fftObj->fixedPointBuf[k] = float_to_fixed(srcBuffer[k]);
        fftObj->fixedPointBuf[k+1] = float_to_fixed(srcBuffer[k+1]);
        /* Complex conjucate as output is real:X(N-k) = X*(k) and k = 2xi for I and Q */
        fftObj->fixedPointBuf[conjIdx] = float_to_fixed(-srcBuffer[k]);
        fftObj->fixedPointBuf[conjIdx+1] = fftObj->fixedPointBuf[k+1];
    }
    endCycle = CycleCounterP_getCount32();
    resObj->float2FixedCycle = endCycle - startCycle;

    /* DMA copy to HWA memory*/
    startCycle = endCycle;
    CacheP_wb(fftObj->fixedPointBuf, numSamples * 2U * sizeof(int32_t), CacheP_TYPE_ALL);
    HWAFFT_edmaTrigger(
        resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, rdMemBankAddr,
        fftObj->fixedPointBuf, numSamples * 2U * sizeof(int32_t), 1U);
    HWAFFT_edmaWait(resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, 1U);
    endCycle = CycleCounterP_getCount32();
    resObj->memcpy2HwaCycle = endCycle - startCycle;

    /* Param update */
    startCycle = endCycle;
    ctrlBaseAddr = HWA_getCommonCtrlAddr(fftObj->hwaHandle);
    DebugP_assert(ctrlBaseAddr != NULL);
    paramBaseAddr = HWA_getParamSetAddr(fftObj->hwaHandle, resObj->paramIdx);
    DebugP_assert(paramBaseAddr != NULL);
    CSL_FINSR(paramBaseAddr->SRCA, SRCA_SRCACNT_END, SRCA_SRCACNT_START, (numSamples - 1U));
    CSL_FINSR(paramBaseAddr->DSTA, DSTA_DSTACNT_END, DSTA_DSTACNT_START, (numSamples - 1U));
    CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_FFTSIZE_END, BFLYFFT_FFTSIZE_START, numFFTStages);
    /* 12-bit scaling for each stage - 0 means saturate: clip MSB, 1 means scale down by 2: round-off LSB */
    CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_BFLY_SCALING_END, BFLYFFT_BFLY_SCALING_START, 0xFFFU);
    /* Common control update */
    CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX, PARAM_RAM_IDX_PARAM_START_IDX_END, PARAM_RAM_IDX_PARAM_START_IDX_START, resObj->paramIdx);
    CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX, PARAM_RAM_IDX_PARAM_END_IDX_END, PARAM_RAM_IDX_PARAM_END_IDX_START, resObj->paramIdx);

    /* Enable HWA */
    status += HWA_enable(fftObj->hwaHandle, 1U);
    status += HWA_reset(fftObj->hwaHandle);

    /* Trigger and wait */
    status += HWA_setDMA2ACCManualTrig(fftObj->hwaHandle, resObj->rdDmaCh);
    SemaphoreP_pend(&fftObj->doneSem, SystemP_WAIT_FOREVER);

    /* Disable HWA */
    status += HWA_enable(fftObj->hwaHandle, 0U);

    endCycle = CycleCounterP_getCount32();
    resObj->hwaCycle = endCycle - startCycle;

    /* DMA copy from HWA memory */
    startCycle = endCycle;
    wrMemBankAddr = (int32_t *) resObj->wrMemBankAddr;
    HWAFFT_edmaTrigger(
        resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, fftObj->fixedPointBuf,
        wrMemBankAddr, numSamples * 2U * sizeof(int32_t), 0U);
    HWAFFT_edmaWait(resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, 0U);
    CacheP_inv(fftObj->fixedPointBuf, numSamples * 2U * sizeof(int32_t), CacheP_TYPE_ALL);
    endCycle = CycleCounterP_getCount32();
    resObj->memcpyFromHwaCycle = endCycle - startCycle;

    /* Convert fixed to float */
    startCycle = endCycle;
    for(i = 0; i < numSamples; i++)
    {
        destBuffer[i] = fixed_to_float(fftObj->fixedPointBuf[(2U * i) + 1U]) * scalFactor;
    }
    endCycle = CycleCounterP_getCount32();
    resObj->fixed2FloatCycle = endCycle - startCycle;

    resObj->totalCycle = endCycle - fftStartCycle;

    return (status);
}

int32_t HWAFFT_fftDynamicRealToCmplxFloat(struct HWAFFT_Object *fftObj,
                                          float *destBuffer,
                                          const float *srcBuffer,
                                          uint32_t numSamples,
                                          uint32_t *scalFactor)
{
    int32_t                     status = SystemP_SUCCESS;
    DSSHWACCRegs               *ctrlBaseAddr;
    DSSHWACCPARAMRegs          *paramBaseAddr;
    struct HWAFFT_ResObject    *resObj;
    uint32_t                    numFFTStages = HWAFFT_log2Approx(numSamples);
    uint32_t                    passCnt;
    uint32_t                    fftStartCycle, startCycle, endCycle;

    resObj = &fftObj->resObj[HWAFFT_RES_IDX_FFT];

    fftStartCycle = CycleCounterP_getCount32();

    /* Float to Fixed and DMA to HWA memory */
    HWAFFT_fftSrcConvertCopy(fftObj, srcBuffer, numSamples);

    /* Param update */
    startCycle = CycleCounterP_getCount32();
    ctrlBaseAddr = HWA_getCommonCtrlAddr(fftObj->hwaHandle);
    DebugP_assert(ctrlBaseAddr != NULL);
    paramBaseAddr = HWA_getParamSetAddr(fftObj->hwaHandle, resObj->paramIdx);
    DebugP_assert(paramBaseAddr != NULL);
    CSL_FINSR(paramBaseAddr->SRCA, SRCA_SRCACNT_END, SRCA_SRCACNT_START, (numSamples - 1U));
    CSL_FINSR(paramBaseAddr->DSTA, DSTA_DSTACNT_END, DSTA_DSTACNT_START, (numSamples - 1U));
    CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_FFTSIZE_END, BFLYFFT_FFTSIZE_START, numFFTStages);
    /* Common control update */
    CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX, PARAM_RAM_IDX_PARAM_START_IDX_END, PARAM_RAM_IDX_PARAM_START_IDX_START, resObj->paramIdx);
    CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX, PARAM_RAM_IDX_PARAM_END_IDX_END, PARAM_RAM_IDX_PARAM_END_IDX_START, resObj->paramIdx);

    /* For dynamic scaling, start with aggressive scaling (no div-by-2 in any
     * stage). Then, based on clip status register, enable div-by-2 one stage
     * at a time, wherever clip happens */
    *scalFactor = 1U;
    CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_BFLY_SCALING_END, BFLYFFT_BFLY_SCALING_START, 0U);
    for(passCnt = 1U; passCnt <= (numFFTStages + 1U); passCnt++)
    {
        uint32_t fftStageClipReg, clipStageIndx, scalFactorNew;

        /* Enable HWA */
        status += HWA_enable(fftObj->hwaHandle, 1U);
        status += HWA_reset(fftObj->hwaHandle);

        /* Trigger and wait */
        status += HWA_setDMA2ACCManualTrig(fftObj->hwaHandle, resObj->rdDmaCh);
        SemaphoreP_pend(&fftObj->doneSem, SystemP_WAIT_FOREVER);

        /* Disable HWA */
        status += HWA_enable(fftObj->hwaHandle, 0U);

        /* Check and clear clip status */
        fftStageClipReg = CSL_FEXTR(ctrlBaseAddr->FFT_CLIP, FFT_CLIP_FFT_CLIP_END, FFT_CLIP_FFT_CLIP_START);
        CSL_FINSR(ctrlBaseAddr->CLR_FFTCLIP, CLR_FFTCLIP_CLR_FFTCLIP_END, CLR_FFTCLIP_CLR_FFTCLIP_START, 1U);
        if(fftStageClipReg == 0U)
        {
            /* No clip in this pass, so exit the loop */
            break;
        }

        /* Clipping has happened, find the stage at which the clip happened first,
         * and apply scaling down for that stage in the next pass */
        clipStageIndx = HWAFFT_maxBit(fftStageClipReg);  /* Find which is the highest stage that is clipping (bit position with 1 starting from MSB) */
        scalFactorNew = CSL_FEXTR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_BFLY_SCALING_END, BFLYFFT_BFLY_SCALING_START);
        scalFactorNew |= (1U << clipStageIndx);
        CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_BFLY_SCALING_END, BFLYFFT_BFLY_SCALING_START, scalFactorNew);
        *scalFactor <<= 1U;
    }
    endCycle = CycleCounterP_getCount32();
    resObj->hwaCycle = endCycle - startCycle;

    /* DMA from HWA memory and then fixed to float */
    HWAFFT_fftDestConvertCopy(fftObj, destBuffer, numSamples);

    resObj->totalCycle = CycleCounterP_getCount32() - fftStartCycle;

    return (status);
}

int32_t HWAFFT_ifftDynamicCmplxToRealFloat(struct HWAFFT_Object *fftObj,
                                           float *destBuffer,
                                           const float *srcBuffer,
                                           uint32_t numSamples,
                                           uint32_t fftScalFactor)
{
    int32_t                     status = SystemP_SUCCESS;
    int32_t                     i, k, conjIdx;
    int32_t                    *rdMemBankAddr, *wrMemBankAddr;
    uint32_t                    scalFactor;
    DSSHWACCRegs               *ctrlBaseAddr;
    DSSHWACCPARAMRegs          *paramBaseAddr;
    struct HWAFFT_ResObject    *resObj;
    uint32_t                    numFFTStages = HWAFFT_log2Approx(numSamples);
    uint32_t                    passCnt;
    uint32_t                    fftStartCycle, startCycle, endCycle;
    float                       effScaleFactor;

    resObj = &fftObj->resObj[HWAFFT_RES_IDX_IFFT];

    /* Convert float to fixed */
    fftStartCycle = startCycle = CycleCounterP_getCount32();
    rdMemBankAddr = (int32_t *) resObj->rdMemBankAddr;
    fftObj->fixedPointBuf[0] = 0;                 /* Im(X(0)) is always zero */
    fftObj->fixedPointBuf[1] = float_to_fixed(srcBuffer[1]);
    /* N/2 is stored in Im(X(0)) */
    fftObj->fixedPointBuf[numSamples] = 0;        /* Im(X(N/2)) is always zero */
    fftObj->fixedPointBuf[numSamples+1] = float_to_fixed(srcBuffer[0]);
    for(i = 1; i < (numSamples >> 1U); i++)
    {
        k = (i << 1U);
        conjIdx = (numSamples << 1U) - k;

        fftObj->fixedPointBuf[k] = float_to_fixed(srcBuffer[k]);
        fftObj->fixedPointBuf[k+1] = float_to_fixed(srcBuffer[k+1]);
        /* Complex conjucate as output is real:X(N-k) = X*(k) and k = 2xi for I and Q */
        fftObj->fixedPointBuf[conjIdx] = float_to_fixed(-srcBuffer[k]);
        fftObj->fixedPointBuf[conjIdx+1] = fftObj->fixedPointBuf[k+1];
    }
    endCycle = CycleCounterP_getCount32();
    resObj->float2FixedCycle = endCycle - startCycle;

    /* DMA copy to HWA memory */
    startCycle = endCycle;
    CacheP_wb(fftObj->fixedPointBuf, numSamples * 2U * sizeof(int32_t), CacheP_TYPE_ALL);
    HWAFFT_edmaTrigger(
        resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, rdMemBankAddr,
        fftObj->fixedPointBuf, numSamples * 2U * sizeof(int32_t), 1U);
    HWAFFT_edmaWait(resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, 1U);
    endCycle = CycleCounterP_getCount32();
    resObj->memcpy2HwaCycle = endCycle - startCycle;

    /* Param update */
    startCycle = endCycle;
    ctrlBaseAddr = HWA_getCommonCtrlAddr(fftObj->hwaHandle);
    DebugP_assert(ctrlBaseAddr != NULL);
    paramBaseAddr = HWA_getParamSetAddr(fftObj->hwaHandle, resObj->paramIdx);
    DebugP_assert(paramBaseAddr != NULL);
    CSL_FINSR(paramBaseAddr->SRCA, SRCA_SRCACNT_END, SRCA_SRCACNT_START, (numSamples - 1U));
    CSL_FINSR(paramBaseAddr->DSTA, DSTA_DSTACNT_END, DSTA_DSTACNT_START, (numSamples - 1U));
    CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_FFTSIZE_END, BFLYFFT_FFTSIZE_START, numFFTStages);
    /* Common control update */
    CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX, PARAM_RAM_IDX_PARAM_START_IDX_END, PARAM_RAM_IDX_PARAM_START_IDX_START, resObj->paramIdx);
    CSL_FINSR(ctrlBaseAddr->PARAM_RAM_IDX, PARAM_RAM_IDX_PARAM_END_IDX_END, PARAM_RAM_IDX_PARAM_END_IDX_START, resObj->paramIdx);

    /* For dynamic scaling, start with aggressive scaling (no div-by-2 in any
     * stage). Then, based on clip status register, enable div-by-2 one stage
     * at a time, wherever clip happens */
    scalFactor = 1U;
    CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_BFLY_SCALING_END, BFLYFFT_BFLY_SCALING_START, 0U);
    for(passCnt = 1U; passCnt <= (numFFTStages + 1U); passCnt++)
    {
        uint32_t fftStageClipReg, clipStageIndx, scalFactorNew;

        /* Enable HWA */
        status += HWA_enable(fftObj->hwaHandle, 1U);
        status += HWA_reset(fftObj->hwaHandle);

        /* Trigger and wait */
        status += HWA_setDMA2ACCManualTrig(fftObj->hwaHandle, resObj->rdDmaCh);
        SemaphoreP_pend(&fftObj->doneSem, SystemP_WAIT_FOREVER);

        /* Disable HWA */
        status += HWA_enable(fftObj->hwaHandle, 0U);

        /* Check and clear clip status */
        fftStageClipReg = CSL_FEXTR(ctrlBaseAddr->FFT_CLIP, FFT_CLIP_FFT_CLIP_END, FFT_CLIP_FFT_CLIP_START);
        CSL_FINSR(ctrlBaseAddr->CLR_FFTCLIP, CLR_FFTCLIP_CLR_FFTCLIP_END, CLR_FFTCLIP_CLR_FFTCLIP_START, 1U);
        if(fftStageClipReg == 0U)
        {
            /* No clip in this pass, so exit the loop */
            break;
        }

        /* Clipping has happened, find the stage at which the clip happened first,
         * and apply scaling down for that stage in the next pass */
        clipStageIndx = HWAFFT_maxBit(fftStageClipReg);  /* Find which is the highest stage that is clipping (bit position with 1 starting from MSB) */
        scalFactorNew = CSL_FEXTR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_BFLY_SCALING_END, BFLYFFT_BFLY_SCALING_START);
        scalFactorNew |= (1U << clipStageIndx);
        CSL_FINSR(paramBaseAddr->accelModeParam.FFTPATH.BFLYFFT, BFLYFFT_BFLY_SCALING_END, BFLYFFT_BFLY_SCALING_START, scalFactorNew);
        scalFactor <<= 1U;
    }
    endCycle = CycleCounterP_getCount32();
    resObj->hwaCycle = endCycle - startCycle;

    /* DMA copy from HWA memory */
    startCycle = endCycle;
    wrMemBankAddr = (int32_t *) resObj->wrMemBankAddr;
    HWAFFT_edmaTrigger(
        resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, fftObj->fixedPointBuf,
        wrMemBankAddr, numSamples * 2U * sizeof(int32_t), 0U);
    HWAFFT_edmaWait(resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, 0U);
    CacheP_inv(fftObj->fixedPointBuf, numSamples * 2U * sizeof(int32_t), CacheP_TYPE_ALL);
    endCycle = CycleCounterP_getCount32();
    resObj->memcpyFromHwaCycle = endCycle - startCycle;

    /* Convert fixed to float */
    startCycle = endCycle;
    effScaleFactor = ((float) scalFactor * (float) fftScalFactor) / (float) numSamples;
    for(i = 0; i < numSamples; i++)
    {
        destBuffer[i] = fixed_to_float(fftObj->fixedPointBuf[(2U * i) + 1U]) * effScaleFactor;
    }
    endCycle = CycleCounterP_getCount32();
    resObj->fixed2FloatCycle = endCycle - startCycle;

    resObj->totalCycle = endCycle - fftStartCycle;

    return (status);
}

float HWAFFT_getMean(const float *srcBuffer, uint32_t numSamples)
{
    uint32_t    i;
    double      mean;

    mean = 0.0;
    for(i = 0; i < numSamples; i++)
    {
        mean += (double) srcBuffer[i];
    }
    mean /= (double) numSamples;

    return ((float) mean);
}

float HWAFFT_getStdDeviation(const float *srcBuffer, uint32_t numSamples)
{
    uint32_t    i;
    double      mean, stddev, temp;

    mean = HWAFFT_getMean(srcBuffer, numSamples);
    stddev = 0.0;
    for(i = 0; i < numSamples; i++)
    {
        temp = (double) srcBuffer[i] - mean;
        stddev += (temp * temp);
    }
    stddev /= (double) numSamples;
    stddev = sqrt(stddev);

    return ((float) stddev);
}

float HWAFFT_getSnr(const float *srcBuffer1, const float *srcBuffer2, uint32_t numSamples)
{
    uint32_t    i;
    double      mean, stddev1, stddev2, temp;
    double      snr;

    /* Input Std deviation */
    stddev1 = (double) HWAFFT_getStdDeviation(srcBuffer1, numSamples);

    /* mean of error signal */
    mean = 0.0;
    for(i = 0; i < numSamples; i++)
    {
        mean += (double) ((double) srcBuffer2[i] - (double) srcBuffer1[i]);
    }
    mean /= (double) numSamples;

    /* std deviation of error signal */
    stddev2 = 0.0;
    for(i = 0; i < numSamples; i++)
    {
        temp = ((double) ((double) srcBuffer2[i] - (double) srcBuffer1[i])) - mean;
        stddev2 += (temp * temp);
    }
    stddev2 /= (double) numSamples;
    stddev2 = sqrt(stddev2);

    snr = 20.0 * log10(stddev1 / stddev2);

    return ((float) snr);
}

/*
 * ISR Callback
 */
static void HWAFFT_doneCallback(uint32_t threadIdx, void *arg)
{
    struct HWAFFT_Object   *fftObj;

    if(arg != NULL)
    {
        fftObj = (struct HWAFFT_Object *)arg;
        SemaphoreP_post(&fftObj->doneSem);
    }

    return;
}

/*
 * EDMA functions
 */
static void HWAFFT_edmaTrigger(struct HWAFFT_ResObject *resObj,
                               uint32_t edmaBaseAddr,
                               uint32_t edmaRegionId,
                               void *dest,
                               void *src,
                               uint32_t numBytes,
                               uint32_t isRd)
{
    uint32_t                    dmaCh, paramId, tcc;
    EDMACCPaRAMEntry          *edmaPrms;

    if(isRd)
    {
        dmaCh       = resObj->rdDmaCh;
        edmaPrms    = &resObj->rdDmaParam;
        tcc         = resObj->rdTcc;
        paramId     = resObj->rdParamId;
    }
    else
    {
        dmaCh       = resObj->wrDmaCh;
        edmaPrms    = &resObj->wrDmaParam;
        tcc         = resObj->wrTcc;
        paramId     = resObj->wrParamId;
    }
    edmaPrms->srcAddr   = (uint32_t) SOC_virtToPhy(src);
    edmaPrms->destAddr  = (uint32_t) SOC_virtToPhy(dest);
    edmaPrms->aCnt      = (uint16_t) numBytes;
    edmaPrms->bCnt      = (uint16_t) 1U;
    edmaPrms->cCnt      = (uint16_t) 1U;
    edmaPrms->opt       =
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(edmaBaseAddr, paramId, edmaPrms);

    EDMA_enableTransferRegion(edmaBaseAddr, edmaRegionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

    return;
}

static void HWAFFT_edmaWait(struct HWAFFT_ResObject *resObj,
                            uint32_t edmaBaseAddr,
                            uint32_t edmaRegionId,
                            uint32_t isRd)
{
    uint32_t        tcc;

    if(isRd)
    {
        tcc = resObj->rdTcc;
    }
    else
    {
        tcc = resObj->wrTcc;
    }

    while(EDMA_readIntrStatusRegion(edmaBaseAddr, edmaRegionId, tcc) != 1);
    EDMA_clrIntrRegion(edmaBaseAddr, edmaRegionId, tcc);

    return;
}

/*
 * Helper functions
 */

static void HWAFFT_fftSrcConvertCopy(struct HWAFFT_Object *fftObj,
                                     const float *srcBuffer,
                                     uint32_t numSamples)
{
    int32_t                     i, loop;
    int32_t                    *rdMemBankAddr;
    struct HWAFFT_ResObject    *resObj;
    uint32_t                    startCycle, endCycle;
    uint32_t                    numSamplesPerItr = numSamples >> HWAFFT_DMA_INTERLEAVE_BLOCKS_SHIFT;
    int32_t                    *fixedPointBuf;
    uint32_t                    dmaCh,tcc,paramId;
    EDMACCPaRAMEntry          *edmaPrms;

    startCycle = CycleCounterP_getCount32();

    resObj      = &fftObj->resObj[HWAFFT_RES_IDX_FFT];
    dmaCh       = resObj->rdDmaCh;
    tcc         = resObj->rdTcc;
    paramId     = resObj->rdParamId;
    edmaPrms    = &resObj->rdDmaParam;

    /*
     * Submit 2D DMA, covert a block, trigger DMA, convert next block, wait for DMA, trigger DMA ....
     */
    rdMemBankAddr = (int32_t *) resObj->rdMemBankAddr;
    fixedPointBuf = fftObj->fixedPointBuf;

    /* Initiate 2D DMA */
    edmaPrms->srcAddr   = (uint32_t) SOC_virtToPhy(fixedPointBuf);
    edmaPrms->destAddr  = (uint32_t) SOC_virtToPhy(rdMemBankAddr);
    edmaPrms->aCnt      = (uint16_t) numSamplesPerItr * sizeof(int32_t);
    edmaPrms->bCnt      = (uint16_t) HWAFFT_DMA_INTERLEAVE_BLOCKS;
    edmaPrms->cCnt      = (uint16_t) 1U;
    edmaPrms->srcBIdx   = (int16_t) EDMA_PARAM_BIDX(numSamplesPerItr * sizeof(int32_t));
    edmaPrms->destBIdx  = (int16_t) EDMA_PARAM_BIDX(numSamplesPerItr * sizeof(int32_t));
    edmaPrms->srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(numSamplesPerItr * sizeof(int32_t));
    edmaPrms->destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(numSamplesPerItr * sizeof(int32_t));
    edmaPrms->opt       =
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(fftObj->edmaBaseAddr, paramId, edmaPrms);

    for(loop = 0; loop < HWAFFT_DMA_INTERLEAVE_BLOCKS; loop++)
    {
        if(loop != 0)
        {
            rdMemBankAddr += numSamplesPerItr;
            fixedPointBuf += numSamplesPerItr;
            srcBuffer     += numSamplesPerItr;
        }

        /* Convert float to fixed */
        for(i = 0; i < numSamplesPerItr; i++)
        {
            fixedPointBuf[i] = float_to_fixed(srcBuffer[i]);
        }
        CacheP_wb(fixedPointBuf, numSamplesPerItr * sizeof(int32_t), CacheP_TYPE_ALL);

        if(loop != 0)
        {
            /* Wait for previous DMA to complete */
            while(EDMA_readIntrStatusRegion(fftObj->edmaBaseAddr, fftObj->edmaRegionId, tcc) != 1);
            EDMA_clrIntrRegion(fftObj->edmaBaseAddr, fftObj->edmaRegionId, tcc);
        }
        /* Tigger the next block transfer */
        EDMA_enableTransferRegion(fftObj->edmaBaseAddr, fftObj->edmaRegionId, dmaCh, EDMA_TRIG_MODE_MANUAL);
    }
    endCycle = CycleCounterP_getCount32();
    resObj->float2FixedCycle = endCycle - startCycle;

    startCycle = endCycle;
    HWAFFT_edmaWait(resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, 1U);
    endCycle = CycleCounterP_getCount32();
    resObj->memcpy2HwaCycle = endCycle - startCycle;

    return;
}

static void HWAFFT_fftDestConvertCopy(struct HWAFFT_Object *fftObj,
                                      float *destBuffer,
                                      uint32_t numSamples)
{
    int32_t                     i, loop;
    int32_t                    *wrMemBankAddr;
    struct HWAFFT_ResObject    *resObj;
    uint32_t                    startCycle, endCycle;
    uint32_t                    numSamplesPerItr = numSamples >> HWAFFT_DMA_INTERLEAVE_BLOCKS_SHIFT;
    int32_t                    *fixedPointBuf;
    volatile EDMACCPaRAMEntry *prm;
    uint16_t                    expectedAcnt;

    startCycle = CycleCounterP_getCount32();

    resObj  = &fftObj->resObj[HWAFFT_RES_IDX_FFT];
    prm     = (volatile EDMACCPaRAMEntry *) (fftObj->edmaBaseAddr + EDMA_TPCC_OPT(resObj->wrParamId));

    wrMemBankAddr = (int32_t *) resObj->wrMemBankAddr;
    fixedPointBuf = fftObj->fixedPointBuf;

    /* Trigger the entire DMA */
    CacheP_inv(fixedPointBuf, numSamples * sizeof(int32_t), CacheP_TYPE_ALL);
    HWAFFT_edmaTrigger(
        resObj, fftObj->edmaBaseAddr, fftObj->edmaRegionId, fixedPointBuf,
        wrMemBankAddr, numSamples * sizeof(int32_t), 0U);
    expectedAcnt = numSamples * sizeof(int32_t);
    for(loop = 0; loop < HWAFFT_DMA_INTERLEAVE_BLOCKS; loop++)
    {
        /* Wait for each block buffer to complete */
        expectedAcnt -= numSamplesPerItr * sizeof(int32_t);
        while(prm->aCnt > expectedAcnt);

        if(loop == 0U)
        {
            endCycle = CycleCounterP_getCount32();
            resObj->memcpyFromHwaCycle = endCycle - startCycle;
            startCycle = endCycle;

            /* Store Re(X(N/2)) in Im(X(0)) */
            fixedPointBuf[0U] = wrMemBankAddr[numSamples + 1U];
        }

        for(i = 0; i < numSamplesPerItr; i++) /* (N/2) * 2 = N */
        {
            destBuffer[i] = fixed_to_float(fixedPointBuf[i]);
        }

        fixedPointBuf += numSamplesPerItr;
        destBuffer    += numSamplesPerItr;
    }
    endCycle = CycleCounterP_getCount32();
    resObj->fixed2FloatCycle = endCycle - startCycle;

    return;
}

static uint32_t HWAFFT_log2Approx(uint32_t x)
{
    uint32_t idx, detectFlag = 0U;

    if(x < 2U)
    {
        idx = 0U;
    }
    else
    {
        idx = 32U;
        while((detectFlag == 0U) || (idx == 0U))
        {
            if(x & 0x80000000U)
            {
                detectFlag = 1;
            }
            x <<= 1U;
            idx--;
        }
    }

    return (idx);
}

static uint32_t HWAFFT_maxBit(uint32_t x)
{
    uint32_t idx;

    if(x < 2U)
    {
        idx = 0U;
    }
    else
    {
        for(idx = 31U; idx > 0U; idx--)
        {
            if(x & ((uint32_t)1U << idx))
            {
                break;
            }
        }
    }

    return (idx);
}
