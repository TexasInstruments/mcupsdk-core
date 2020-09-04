/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include <stdint.h>
#include "ti_r5fmath_trig.h"


float ti_r5fmath_sinCoef[4] = {
                                 0.999996615908002773079325846913220383f,
                                -0.16664828381895056829366054140948866f,
                                 0.00830632522715989396465411782615901079f,
                                -0.00018363653976946785297280224158683484f
                              };


float ti_r5fmath_cosCoef[5] = {
                                 0.999999953466670136306412430924463351f,
                                -0.49999905347076729097546897993796764f,
                                 0.0416635846931078386653947196040757567f,
                                -0.00138537043082318983893723662479142648f,
                                 0.0000231539316590538762175742441588523467
                               };

/* 3PI/2, 2PI, PI/2, PI */

float ti_r5fmath_PIconst[4] =  {
                                  TI_R5FMATH_3PIOVER2,
                                  TI_R5FMATH_2PI,
                                  TI_R5FMATH_PIOVER2,
                                  TI_R5FMATH_PI
                                };


/*  first 4 coef are for sin, second 4 are for cos  */

float ti_r5fmath_sincosCoef[8] = {
                                    1.0f,
                                   -0.166666507720947265625f,
                                    0.008331954479217529296875f,
                                   -0.00019490718841552734375f,
                                    1.0f,
                                   -0.499998867511749267578125f,
                                    4.165589809417724609375e-2f,
                                   -1.35934352874755859375e-3f
                                  };
/*  1/(PI/4), PI/2  */

float ti_r5fmath_sincosPIconst[2] = {
                                    TI_R5FMATH_ONEOVERPIOVER4,
                                    TI_R5FMATH_PIOVER2
                                   };
/* coef3, coef1, PI/2, coef5, PI/6, tan(PI/6), tan(PI/12)       */

float ti_r5fmath_atan_consts[7] = {
                                    -0.33298534f,
                                     0.9999986f,
                                     TI_R5FMATH_PIOVER2,
                                     0.186177492f,
                                     TI_R5FMATH_PIOVER6,
                                     TI_R5FMATH_TANPIOVER6,
                                     TI_R5FMATH_TANPIOVER12
                                   };
/* tan(PI/12), tan(PI/6), PI/6, PI/2, PI, coef1, coef3, coef5       */

float ti_r5fmath_atan2_consts[8] = {
                                    TI_R5FMATH_TANPIOVER12,
                                    TI_R5FMATH_TANPIOVER6,
                                    TI_R5FMATH_PIOVER6,
                                    TI_R5FMATH_PIOVER2,
                                    TI_R5FMATH_PI,
                                    0.9999954700469970703125f,
                                   -0.332797586917877197265625f,
                                    0.183986365795135498046875f
                                };


float ti_r5fmath_atanFast_consts[4] = {
                                        TI_R5FMATH_PI,
                                        TI_R5FMATH_PIOVER2,
                                        TI_R5FMATH_PIOVER4,
                                        0.273f
                                    };




//void Cfast_sincos9(float angleRad, float *constPIVal, float *constCoefVal, float *retValues)
void ti_r5fmath_sincos(float angleRad, float *PIconst, float *sincosCoef, float *retValues)
{
    // PIconst
    // [0] 3PI/2  [1] 2PI  [2] PI/2   [3] PI
    // sincosCoef
    // [0] SIN1  [1] SIN3  [2] SIN5   [3] SIN7
    // [0] COS0  [1] COS2  [2] COS4   [3] COS6  [4] COS8

    float a2, a4,sinVal, cosVal, modVal;
    uint32_t swapVal, signS, signC;
    int32_t r;

    swapVal = 0x0066;
    signS = 0x001e;
    signC = 0x0078;

     // mult by 1/(PI/4) and take floor

    r = (int) (angleRad * PIconst[0]); // 0 < 1/4   3/4> x > 1/4 = 1  > 3/4 = 2
    modVal = angleRad - ((r+1)>>1) * PIconst[1];  // brings into 0 -> PI/4

   a2 = modVal * modVal;
   a4 = a2 * a2;

   sinVal = modVal* sincosCoef[0];
   sinVal +=  modVal*sincosCoef[1] * a2;
   sinVal +=  modVal*sincosCoef[2] * a4;
   sinVal +=  modVal*sincosCoef[3] * a4 * a2;

   cosVal =  sincosCoef[4];
   cosVal += sincosCoef[5]*a2;
   cosVal += sincosCoef[6]*a4;
   cosVal += sincosCoef[7]*a4*a2;


   swapVal = (swapVal >> r) & 0x1;
   signS = (signS >> r) & 0x1;
   signC = (signC >> r) & 0x1;

   if (signS)
       sinVal = -sinVal;
   if (signC)
       cosVal = -cosVal;

   retValues[swapVal] = sinVal;
   retValues[swapVal^1] = cosVal;


}




