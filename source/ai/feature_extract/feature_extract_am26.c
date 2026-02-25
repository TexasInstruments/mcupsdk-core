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

#include <stdint.h>
#include <math.h>
#include "user_input_config.h"

void FE_cfft(float *input, float *output, uint16_t frame_size, uint16_t fe_stages) {
    uint16_t i, j, k, n1, n2;
    float tr, ti, c, s, temp_r, temp_i;
    const float PI = 3.14159265358979323846f;
    
    // Copy input to output and set imaginary parts to zero
    for (i = 0; i < frame_size; i++) {
        output[i*2] = input[i];     // Real part
        output[i*2+1] = 0.0f;       // Imaginary part = 0
    }
    
    // Bit-reversal reordering
    j = 0;
    for (i = 0; i < frame_size - 1; i++) {
        if (i < j) {
            // Swap real parts
            temp_r = output[i*2];
            output[i*2] = output[j*2];
            output[j*2] = temp_r;
            
            // Swap imaginary parts
            temp_i = output[i*2+1];
            output[i*2+1] = output[j*2+1];
            output[j*2+1] = temp_i;
        }
        
        // Find next bit-reversed pair
        k = frame_size >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }
    
    // Cooley-Tukey FFT
    for (i = 0; i < fe_stages; i++) {
        n1 = 1 << i;       // 2^i
        n2 = n1 << 1;      // 2^(i+1)
        
        for (j = 0; j < frame_size; j += n2) {
            for (k = 0; k < n1; k++) {
                // Compute twiddle factor
                c = cosf(-2.0f * M_PI * k / n2);
                s = sinf(-2.0f * M_PI * k / n2);
                
                // Butterfly computation
                int even_idx = j + k;
                int odd_idx = j + k + n1;
                
                // Multiply odd by twiddle factor
                tr = output[odd_idx*2] * c - output[odd_idx*2+1] * s;
                ti = output[odd_idx*2] * s + output[odd_idx*2+1] * c;
                
                // Butterfly operation
                temp_r = output[even_idx*2];
                temp_i = output[even_idx*2+1];
                
                output[even_idx*2] = temp_r + tr;
                output[even_idx*2+1] = temp_i + ti;
                
                output[odd_idx*2] = temp_r - tr;
                output[odd_idx*2+1] = temp_i - ti;
            }
        }
    }
    
    // Calculate magnitude and store back in input
    for (i = 0; i < frame_size; i++) {
        // Magnitude = sqrt(real² + imag²)
        input[i] = sqrtf(output[i*2] * output[i*2] + output[i*2+1] * output[i*2+1]);
    }
}