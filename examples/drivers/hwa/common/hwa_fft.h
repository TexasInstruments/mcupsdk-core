/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef HWA_FFT_H_
#define HWA_FFT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/hwa.h>
#include <drivers/edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* One each for FFT and IFFT operations */
#define HWAFFT_RES_IDX_FFT              (0U)
#define HWAFFT_RES_IDX_IFFT             (1U)
#define HWAFFT_RES_IDX_MAX              (2U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* HWA FFT or IFFT Resource and Profile object */
struct HWAFFT_ResObject
{
    /* Resource variables */
    uint32_t                paramIdx;
    uint32_t                rdDmaCh;
    uint32_t                wrDmaCh;
    uint32_t                rdTcc;
    uint32_t                wrTcc;
    uint32_t                rdParamId;
    uint32_t                wrParamId;
    uint32_t                rdMemBankAddr;
    uint32_t                wrMemBankAddr;
    HWA_ParamConfig         paramCfg;
    EDMACCPaRAMEntry       rdDmaParam;
    EDMACCPaRAMEntry       wrDmaParam;

    /* Profile variables */
    uint32_t                totalCycle;
    uint32_t                float2FixedCycle;
    uint32_t                memcpy2HwaCycle;
    uint32_t                hwaCycle;
    uint32_t                memcpyFromHwaCycle;
    uint32_t                fixed2FloatCycle;
};

/* HWA FFT object */
struct HWAFFT_Object
{
    struct HWAFFT_ResObject resObj[HWAFFT_RES_IDX_MAX];
    int32_t                *fixedPointBuf;

    /* State variables */
    HWA_Handle              hwaHandle;
    uint32_t                edmaBaseAddr;
    uint32_t                edmaRegionId;
    HWA_CommonConfig        commonCfg;
    SemaphoreP_Object       doneSem;
    HWA_InterruptConfig     paramISRCfg;
};

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Init function to reserve HWA resources */
int32_t HWAFFT_init(struct HWAFFT_Object *fftObj,
                    HWA_Handle hwaHandle,
                    EDMA_Handle edmaHandle,
                    int32_t *fixedPointBuf);
/* Deinit function to free-up HWA resources */
int32_t HWAFFT_deinit(struct HWAFFT_Object *fftObj);

/*
 * Float based FFT
 */
/* Real float input to complex float output FFT. Since the input is real only
 * upper N/2 complex samples are stored.
 *
 * Size of srcBuffer  should be : N   x sizeof(float)       - all real data
 * Size of destBuffer should be : N/2 x sizeof(float) x 2   - all complex data with I and Q data
 *
 * Note: Since real input produces X(0) and X(N/2) as real, the real value of
 *       X(N/2) is stored in the imaginary part of X(0) to fit the FFT output
 *       within the output buffer of size N complex samples
 */
int32_t HWAFFT_fftRealToCmplxFloat(struct HWAFFT_Object *fftObj,
                                   float *destBuffer,
                                   const float *srcBuffer,
                                   uint32_t numSamples);

/* Complex float input to real float output IFFT. Since the output is real only
 * the upper N/2 complex input samples needs to be provided.
 *
 * Size of destBuffer should be : N   x sizeof(float)       - all real data
 * Size of srcBuffer  should be : N/2 x sizeof(float) x 2   - all complex data with I and Q data
 *
 * Note: This assumes that X(N/2)'s real portion is stored in the imaginary part
 *       of X(0). See description of #HWAFFT_fftRealToCmplxFloat()
 */
int32_t HWAFFT_ifftCmplxToRealFloat(struct HWAFFT_Object *fftObj,
                                    float *destBuffer,
                                    const float *srcBuffer,
                                    uint32_t numSamples);

/* Real float input to complex float output FFT. Since the input is real only
 * upper N/2 complex samples are stored.
 *
 * Size of srcBuffer  should be : N   x sizeof(float)       - all real data
 * Size of destBuffer should be : N/2 x sizeof(float) x 2   - all complex data with I and Q data
 *
 * Note: Since real input produces X(0) and X(N/2) as real, the real value of
 *       X(N/2) is stored in the imaginary part of X(0) to fit the FFT output
 *       within the output buffer of size N complex samples
 */
int32_t HWAFFT_fftDynamicRealToCmplxFloat(struct HWAFFT_Object *fftObj,
                                          float *destBuffer,
                                          const float *srcBuffer,
                                          uint32_t numSamples,
                                          uint32_t *fftScalFactor);

/* Complex float input to real float output IFFT. Since the output is real only
 * the upper N/2 complex input samples needs to be provided.
 *
 * Size of destBuffer should be : N   x sizeof(float)       - all real data
 * Size of srcBuffer  should be : N/2 x sizeof(float) x 2   - all complex data with I and Q data
 *
 * Note: This assumes that X(N/2)'s real portion is stored in the imaginary part
 *       of X(0). See description of #HWAFFT_fftRealToCmplxFloat()
 */
int32_t HWAFFT_ifftDynamicCmplxToRealFloat(struct HWAFFT_Object *fftObj,
                                           float *destBuffer,
                                           const float *srcBuffer,
                                           uint32_t numSamples,
                                           uint32_t fftScalFactor);

/* Calculate and return the mean of given real float data */
float HWAFFT_getMean(const float *srcBuffer, uint32_t numSamples);

/* Calculate and return the standard deviation of given real float data */
float HWAFFT_getStdDeviation(const float *srcBuffer, uint32_t numSamples);

/* Calculate and return the SNR for the provided two signals */
float HWAFFT_getSnr(const float *srcBuffer1, const float *srcBuffer2, uint32_t numSamples);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef HWA_FFT_H_ */

/** @} */
