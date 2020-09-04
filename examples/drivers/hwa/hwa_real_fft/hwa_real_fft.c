/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *  This example performs HWA based FFT and IFFT operation on real input data.
 *
 *  The input to the FFT is a sinusoidal real (float) data. This is converted to
 *  24-bit signed fixed point data which the HWA will operate on.
 *  The HWA PaRAM sets are initialized with proper configuration for FFT and
 *  IFFT operation.
 *  The HWA is triggered to perform the FFT operation and the frequency domain
 *  complex output is converted back to float just to show that user can tap
 *  this data for further processing.
 *  In the IFFT part, the frequency domain complex data is converted back to
 *  fixed point and provided to HWA to perform the IFFT operation. Once the
 *  conversion is completed, the data is converted back to float.
 *  The output is then compared to the input data and SNR is calculated and
 *  printed on the console.
 *  The example also demonstrates multi-pass dynamic scaling FFT operation which
 *  improves the SNR ratio compared to the single-pass conservative scaling FFT
 *  operation
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/hwa.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "../common/hwa_fft.h"
#include "../common/sine_wave_peak_17_bin.h"
#include "../common/audio_1031Hz_and_5156Hz.h"

#define TEST_FFT_NUM_SAMPLES            (1024U)

HWA_Handle              gHwaHandle;
struct HWAFFT_Object    gHwaFftObject;

/* This has only the first half of the FFT complex output as real input generates
 * mirror image conjugate second half samples which can be calculated from the first half while doing IFFT */
float      gTestFFTFloatOutput[TEST_FFT_NUM_SAMPLES];
/* IFFT output - has only real samples */
float      gTestIFFTFloatOutput[TEST_FFT_NUM_SAMPLES];
/* Temp buffer to store fixed point conversion */
int32_t     gTestFixedPointBuf[TEST_FFT_NUM_SAMPLES * 2U];

static void hwa_print_results(struct HWAFFT_Object *fftObj, float snr);

void hwa_real_fft(void *args)
{
    int32_t                 status = SystemP_SUCCESS;
    float                   snr;
    struct HWAFFT_Object   *fftObj = &gHwaFftObject;
    uint32_t                fftScalFactor;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("[HWA] Real FFT Test Started ...\r\n");

    /* Open HWA driver */
    gHwaHandle = HWA_open(0, NULL, &status);
    if(gHwaHandle == NULL)
    {
        DebugP_log("Error: Unable to open HWA instance. Error: %d\n", status);
        DebugP_assert(gHwaHandle == NULL);
    }
    /* Init HWA FFT library */
    status = HWAFFT_init(fftObj, gHwaHandle, gEdmaHandle[0], &gTestFixedPointBuf[0U]);
    DebugP_assert(SystemP_SUCCESS == status);

    CycleCounterP_reset(); /* Reset CPU cycle counter */

    /*
     * Sinewave Input - Single Pass
     */
    /* FFT: Time domain to Frequency domain */
    status = HWAFFT_fftRealToCmplxFloat(fftObj, &gTestFFTFloatOutput[0], &sine_wave_peak_17_bin[0], TEST_FFT_NUM_SAMPLES);
    DebugP_assert(SystemP_SUCCESS == status);
    /* IFFT: Frequency domain to Time domain */
    status = HWAFFT_ifftCmplxToRealFloat(fftObj, &gTestIFFTFloatOutput[0], &gTestFFTFloatOutput[0], TEST_FFT_NUM_SAMPLES);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Calculate SNR */
    snr = HWAFFT_getSnr(&sine_wave_peak_17_bin[0], &gTestIFFTFloatOutput[0], TEST_FFT_NUM_SAMPLES);
    DebugP_log("\r\n");
    DebugP_log("Result for Sinewave with Peak 17 Bin with Single Pass:\r\n");
    DebugP_log("------------------------------------------------------\r\n");
    hwa_print_results(fftObj, snr);

    /*
     * Sinewave Input - Multi-Pass Dynamic Scaling
     */
    /* FFT: Time domain to Frequency domain */
    status = HWAFFT_fftDynamicRealToCmplxFloat(fftObj, &gTestFFTFloatOutput[0], &sine_wave_peak_17_bin[0], TEST_FFT_NUM_SAMPLES, &fftScalFactor);
    DebugP_assert(SystemP_SUCCESS == status);
    /* IFFT: Frequency domain to Time domain */
    status = HWAFFT_ifftDynamicCmplxToRealFloat(fftObj, &gTestIFFTFloatOutput[0], &gTestFFTFloatOutput[0], TEST_FFT_NUM_SAMPLES, fftScalFactor);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Calculate SNR */
    snr = HWAFFT_getSnr(&sine_wave_peak_17_bin[0], &gTestIFFTFloatOutput[0], TEST_FFT_NUM_SAMPLES);
    DebugP_log("\r\n");
    DebugP_log("Result for Sinewave with Peak 17 Bin with Multi Pass:\r\n");
    DebugP_log("-----------------------------------------------------\r\n");
    hwa_print_results(fftObj, snr);

    /*
     * Audio Input with 1031Hz and 5156Hz - Single Pass
     */
    /* FFT: Time domain to Frequency domain */
    status = HWAFFT_fftRealToCmplxFloat(fftObj, &gTestFFTFloatOutput[0], &audio_1031Hz_and_5156Hz[0], TEST_FFT_NUM_SAMPLES);
    DebugP_assert(SystemP_SUCCESS == status);
    /* IFFT: Frequency domain to Time domain */
    status = HWAFFT_ifftCmplxToRealFloat(fftObj, &gTestIFFTFloatOutput[0], &gTestFFTFloatOutput[0], TEST_FFT_NUM_SAMPLES);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Calculate SNR */
    snr = HWAFFT_getSnr(&audio_1031Hz_and_5156Hz[0], &gTestIFFTFloatOutput[0], TEST_FFT_NUM_SAMPLES);
    DebugP_log("\r\n");
    DebugP_log("Result for Audio with 1031Hz and 5156Hz with Single Pass:\r\n");
    DebugP_log("---------------------------------------------------------\r\n");
    hwa_print_results(fftObj, snr);

    /*
     * Audio Input with 1031Hz and 5156Hz - Multi-Pass Dynamic Scaling
     */
    /* FFT: Time domain to Frequency domain */
    status = HWAFFT_fftDynamicRealToCmplxFloat(fftObj, &gTestFFTFloatOutput[0], &audio_1031Hz_and_5156Hz[0], TEST_FFT_NUM_SAMPLES, &fftScalFactor);
    DebugP_assert(SystemP_SUCCESS == status);
    /* IFFT: Frequency domain to Time domain */
    status = HWAFFT_ifftDynamicCmplxToRealFloat(fftObj, &gTestIFFTFloatOutput[0], &gTestFFTFloatOutput[0], TEST_FFT_NUM_SAMPLES, fftScalFactor);
    DebugP_assert(SystemP_SUCCESS == status);
    /* Calculate SNR */
    snr = HWAFFT_getSnr(&audio_1031Hz_and_5156Hz[0], &gTestIFFTFloatOutput[0], TEST_FFT_NUM_SAMPLES);
    DebugP_log("\r\n");
    DebugP_log("Result for Audio with 1031Hz and 5156Hz with Multi Pass:\r\n");
    DebugP_log("--------------------------------------------------------\r\n");
    hwa_print_results(fftObj, snr);

    /* HWA driver close */
    status += HWAFFT_deinit(fftObj);
    HWA_controlPeripheralSuspendMode(gHwaHandle, 0);
    status += HWA_close(gHwaHandle);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("HWA Real FFT Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

static void hwa_print_results(struct HWAFFT_Object *fftObj, float snr)
{
    float                       fftMhz, ifftMhz;
    float                       khz = 48.0;
    struct HWAFFT_ResObject    *resObj;

    DebugP_log("Summary:\r\n");
    DebugP_log("-------:\r\n");
    DebugP_log("SNR Achieved                                : %.2f dB\r\n", snr);
    DebugP_log("FFT/IFFT Samples                            : %d\r\n", TEST_FFT_NUM_SAMPLES);
    resObj = &fftObj->resObj[HWAFFT_RES_IDX_FFT];
    fftMhz  = ((float)resObj->totalCycle * khz * 2U) / (1024.0 * 1000.0);
    DebugP_log("FFT  Performance for 1CH 48 KHz Stereo Input: %.2f MHz\r\n", fftMhz);
    resObj = &fftObj->resObj[HWAFFT_RES_IDX_IFFT];
    ifftMhz = ((float)resObj->totalCycle * khz * 2U) / (1024.0 * 1000.0);
    DebugP_log("IFFT Performance for 1CH 48 KHz Stereo Input: %.2f MHz\r\n", ifftMhz);
    resObj = &fftObj->resObj[HWAFFT_RES_IDX_FFT];
    DebugP_log("Total FFT  Cycles + Float->Fixed->Float     : %d\r\n", resObj->totalCycle);
    resObj = &fftObj->resObj[HWAFFT_RES_IDX_IFFT];
    DebugP_log("Total IFFT Cycles + Float->Fixed->Float     : %d\r\n", resObj->totalCycle);
    DebugP_log("\r\n");

    DebugP_log("Cycle Details:\r\n");
    DebugP_log("-------------:\r\n");
    resObj = &fftObj->resObj[HWAFFT_RES_IDX_FFT];
    DebugP_log("FFT  Float to Fixed   Cycles                : %d\r\n", resObj->float2FixedCycle);
    DebugP_log("FFT  Memcopy to HWA   Cycles                : %d\r\n", resObj->memcpy2HwaCycle);
    DebugP_log("FFT  HWA              Cycles                : %d\r\n", resObj->hwaCycle);
    DebugP_log("FFT  Memcopy from HWA Cycles                : %d\r\n", resObj->memcpyFromHwaCycle);
    DebugP_log("FFT  Fixed to Float   Cycles                : %d\r\n", resObj->fixed2FloatCycle);
    DebugP_log("\r\n");
    resObj = &fftObj->resObj[HWAFFT_RES_IDX_IFFT];
    DebugP_log("IFFT Float to Fixed   Cycles                : %d\r\n", resObj->float2FixedCycle);
    DebugP_log("IFFT Memcopy to HWA   Cycles                : %d\r\n", resObj->memcpy2HwaCycle);
    DebugP_log("IFFT HWA              Cycles                : %d\r\n", resObj->hwaCycle);
    DebugP_log("IFFT Memcopy from HWA Cycles                : %d\r\n", resObj->memcpyFromHwaCycle);
    DebugP_log("IFFT Fixed to Float   Cycles                : %d\r\n", resObj->fixed2FloatCycle);
    DebugP_log("\r\n");

    return;
}
