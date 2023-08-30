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


#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/csl_types.h>

#define RDC_T2_PARAM5_RESET_PARAM_VALUE             (6U)
#define RDC_T2_PARAM6_RESET_PARAM_VALUE             (64U)
#define RDC_T2_PARAM7_RESET_PARAM_VALUE             (10U)
#define RDC_T2_PARAM8_RESET_PARAM_VALUE             (7U)
#define RDC_T2_PARAM9_RESET_PARAM_VALUE             (FALSE)

 volatile int16_t accmsb                 __attribute__((section(".benchmark.var"))) = 0;
 volatile int32_t u0                     __attribute__((section(".benchmark.var"))) = 0;
 volatile int32_t e0                     __attribute__((section(".benchmark.var"))) = 0;
 volatile int32_t accr                   __attribute__((section(".benchmark.var"))) = 0;
 volatile int32_t fcw_piout              __attribute__((section(".benchmark.var"))) = 0;
 volatile int32_t e0filt                 __attribute__((section(".benchmark.var"))) = 0;
 volatile int32_t e0filtcmp              __attribute__((section(".benchmark.var"))) = 0;
 volatile int64_t fcw_shifted16          __attribute__((section(".benchmark.var"))) = 0;
 volatile int64_t vel0                   __attribute__((section(".benchmark.var"))) = 0;

 volatile int32_t kvelfilt               __attribute__((section(".benchmark.var"))) = 8;
 volatile int64_t speedcomp              __attribute__((section(".benchmark.var"))) = 0;

bool correct_quad_offset = TRUE;
bool boost_en __attribute__((section(".benchmark.var"))) = RDC_T2_PARAM9_RESET_PARAM_VALUE;


__attribute__((section(".benchmark.var")))
volatile int32_t offsetLookup[8] = {
    3,      //000 0
    4,      //001 1
    6,      //010 2
    7,      //011 3
    -5,     //100 4
    -4,     //101 5
    -2,     //110 6
    0,      //111 7
};

__attribute__((section(".benchmark.code")))
void track2_psuedo(int16_t theta_atan, int16_t* angle_output, int32_t* velocity_output);


static inline
void atan_offset_correction(int16_t* atan_in)
{
    if(correct_quad_offset){
        *atan_in += offsetLookup[ ((uint16_t) *atan_in) >> 13];
    }
}


__attribute__((section(".benchmark.code")))
void track2_psuedo(int16_t theta_atan, int16_t* angle_output, int32_t* velocity_output)
{
    accr += fcw_piout;

    atan_offset_correction(&theta_atan);

    /* seq 3 */
    accmsb = accr >> 16;

    e0 = ( (int32_t)( ((int32_t) theta_atan) << 16) ) - accr ;

    /* seq 5 */
    u0 = u0 + (int32_t)( ( (int64_t) ( ((int64_t) RDC_T2_PARAM6_RESET_PARAM_VALUE) * ((int64_t) e0)) ) >> 14);

    /* seq 6 */
    fcw_piout = u0 + (int32_t)( ( (int64_t) ( ((int64_t) RDC_T2_PARAM7_RESET_PARAM_VALUE) * ((int64_t) e0)) ) >> 7);

    e0filt = (e0 >> RDC_T2_PARAM5_RESET_PARAM_VALUE) + e0filt - (e0filt >> RDC_T2_PARAM5_RESET_PARAM_VALUE);

    /* seq 7 */
    e0filtcmp = e0filt >> 16;

    if (boost_en)
    {
        *angle_output = accmsb + e0filtcmp;
        speedcomp = e0filt << RDC_T2_PARAM8_RESET_PARAM_VALUE;
    }
    else
    {
        *angle_output = accmsb;
        speedcomp = 0;
    }
    fcw_shifted16 = ((int64_t) fcw_piout) << 16;

    /* seq 10 */
    int64_t temp_vel0 = vel0;

    vel0 = (temp_vel0 - (temp_vel0 >> kvelfilt)) + ((fcw_shifted16 - speedcomp) >> kvelfilt);

    /* seq 11 */
    *velocity_output = (int32_t) (((int64_t)vel0) >> 16);

    /* sequences done */
}
