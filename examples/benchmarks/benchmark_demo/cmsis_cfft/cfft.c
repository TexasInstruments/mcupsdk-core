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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#include "cfft.h"
#include "arm_math.h"
#include "arm_const_structs.h"

float32_t cfftInData[4096];
float32_t cfftMagData[2048];

/* initialize FFT complex array */
void cfftInit(void)
{
    uint16_t i;
    float sin;
    float cos;

    for(i = 0; i < 4096; i+=2)
    {
        arm_sin_cos_f32((float)i*7.5, &sin, &cos);
        cfftInData[i] = sin;
        cfftInData[i+1] = 0;
        cfftMagData[i/2] = 0;
    }
}

int32_t cfft_bench(int32_t fftSize)
{
    uint32_t start, end;

    const static arm_cfft_instance_f32 *S;

    switch (fftSize)
    {
        case 128:
            S = &arm_cfft_sR_f32_len128;
            break;
        case 256:
            S = &arm_cfft_sR_f32_len256;
            break;
        case 512:
            S = &arm_cfft_sR_f32_len512;
            break;
        default:
        case 1024:
            S = &arm_cfft_sR_f32_len1024;
            break;
        case 2048:
            S = &arm_cfft_sR_f32_len2048;
            break;
    }

    cfftInit();

    CycleCounterP_reset();
    start = CycleCounterP_getCount32();

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_f32(S, cfftInData, 0, 1);

    end = CycleCounterP_getCount32();

    App_statsUpdate(start, end);

    /* Process the data through the Complex Magnitude Module for
       calculating the magnitude at each bin */
    arm_cmplx_mag_f32(cfftInData, cfftMagData, fftSize);

    return 0;
}
