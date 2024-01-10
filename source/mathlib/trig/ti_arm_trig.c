/* Copyright (c) 2022, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <stdint.h>
#include "ti_arm_trig.h"

/* PI/2, PI, 3PI/2, 2PI, 4/PI */
TRIG_DATA_SECTION float gPiConsts[5] = { 1.5707963267f, 3.1415926535f,
                                         4.7123889803f, 6.2831853071f,
                                         1.2732395447f };

TRIG_DATA_SECTION float gSinConsts[4] = {
        0.999996615908002773079325846913220383f,
        -0.16664828381895056829366054140948866f,
        0.00830632522715989396465411782615901079f,
        -0.00018363653976946785297280224158683484f };

TRIG_DATA_SECTION float gCosConsts[5] = {
        0.999999953466670136306412430924463351f,
        -0.49999905347076729097546897993796764f,
        0.0416635846931078386653947196040757567f,
        -0.00138537043082318983893723662479142648f,
        0.0000231539316590538762175742441588523467f };

TRIG_DATA_SECTION float gSinCosConsts[8] = { -0.166666507720947265625f,
                                             0.008331954479217529296875f,
                                             -0.00019490718841552734375f,
                                             -0.499998867511749267578125f,
                                             4.165589809417724609375e-2f,
                                             -1.35934352874755859375e-3f };

TRIG_DATA_SECTION float gAsinConsts[7] = { 1.5707961728f, -0.2145852647f,
                                           0.0887556286f, -0.0488025043f,
                                           0.0268999482f, -0.0111462294f,
                                           0.0022959648f };

TRIG_DATA_SECTION float gAtan2Consts[3] = { 0.9999954700469970703125f,
                                            -0.332797586917877197265625f,
                                            0.183986365795135498046875f };

TRIG_DATA_SECTION float gAtanConsts[8] = { 4.17232513427734375e-7f,
                                           0.99997341632843017578125f,
                                           1.46687030792236328125e-4f,
                                           -0.330976545810699462890625f,
                                           -2.6895701885223388671875e-2f,
                                           0.309777557849884033203125f,
                                           -0.21780431270599365234375f,
                                           5.117702484130859375e-2f };


inline TRIG_TEXT_SECTION float ti_arm_sin(float angleRad)
{

    float trigApprox;
    float a = angleRad;

    if (a > gPiConsts[0])
    {
        angleRad = gPiConsts[1] - a;
    }
    if (a > gPiConsts[2])
    {
        angleRad = a - gPiConsts[3];
    }

    float x2 = angleRad * angleRad;
    float c1 = angleRad * gSinConsts[3];
    float c2 = angleRad * gSinConsts[1];
    float c3 = angleRad * gSinConsts[2];
    trigApprox = angleRad * gSinConsts[0];

    c1 = c1 * x2;
    float x4 = x2 * x2;

    trigApprox += c2 * x2;
    trigApprox += c3 * x4;
    trigApprox += c1 * x4;
    return trigApprox;

}

inline TRIG_TEXT_SECTION float ti_arm_cos(float angleRad)
{
    uint8_t sign = 0;
    float a;
    float trigApprox;

    a = angleRad;

    if (a > gPiConsts[0])
    {
        angleRad = a - gPiConsts[1];
        sign = 1;
    }

    if (a > gPiConsts[2])
    {
        angleRad = angleRad - gPiConsts[1];
        sign = 0;
    }

    float x2 = angleRad * angleRad;

    trigApprox = gCosConsts[0];
    trigApprox += gCosConsts[1] * x2;
    float x4 = x2 * x2;

    float c1 = gCosConsts[3] * x2;
    float c2 = gCosConsts[4] * x4;

    trigApprox += gCosConsts[2] * x4;
    trigApprox += c1 * x4;
    trigApprox += c2 * x4;

    if (sign)
        trigApprox = -trigApprox;

    return trigApprox;
}

inline TRIG_TEXT_SECTION void ti_arm_sincos(float angleRad, float *retValues)
{
    float a2;
    float a4;
    float a6;
    float sinVal;
    float cosVal;
    float modVal;
    uint8_t swapVal;
    uint8_t signS;
    uint8_t signC;
    int32_t r;

    r = (int) (angleRad * gPiConsts[4]);

    swapVal = 0x0066;
    signS = 0x001e;
    signC = 0x0078;

    modVal = angleRad - ((r + 1) >> 1) * gPiConsts[0];
    swapVal = (swapVal >> r) & 0x1;
    signS = (signS >> r) & 0x1;
    signC = (signC >> r) & 0x1;
    a2 = modVal * modVal;
    sinVal = modVal;
    a4 = a2 * a2;
    cosVal = 1.0f;
    a6 = a4 * a2;

    sinVal += modVal * gSinCosConsts[0] * a2;
    sinVal += modVal * gSinCosConsts[1] * a4;
    sinVal += modVal * gSinCosConsts[2] * a6;

    cosVal += gSinCosConsts[3] * a2;
    cosVal += gSinCosConsts[4] * a4;
    cosVal += gSinCosConsts[5] * a6;

    if (signS)
        sinVal = -sinVal;
    if (signC)
        cosVal = -cosVal;

    retValues[swapVal] = sinVal;
    retValues[swapVal ^ 1] = cosVal;
}


inline TRIG_TEXT_SECTION float ti_arm_asin(float x)
{
    float asinApprox;
    float sqrtx;
    uint8_t sign = 0;

    if (x < 0)
    {
        x = -x;
        sign = 1;
    }

    sqrtx = 1 - x;
    sqrtx = ti_arm_sqrt(sqrtx);

    float x2 = x * x;

    asinApprox = gAsinConsts[0];
    asinApprox += gAsinConsts[1] * x;

    float x3 = x2 * x;
    float x4 = x2 * x2;
    float x5 = x2 * x3;
    float x6 = x3 * x3;

    asinApprox += gAsinConsts[2] * x2;
    asinApprox += gAsinConsts[3] * x3;
    asinApprox += gAsinConsts[4] * x4;
    asinApprox += gAsinConsts[5] * x5;
    asinApprox += gAsinConsts[6] * x6;

    asinApprox *= sqrtx;

    if (sign)
        asinApprox -= gPiConsts[0];
    else
        asinApprox = gPiConsts[0] - asinApprox;

    return asinApprox;
}

inline TRIG_TEXT_SECTION float ti_arm_acos(float x)
{
    float acosApprox;
    float sqrtx;
    uint8_t sign = 0;

    if (x < 0)
    {
        x = -x;
        sign = 1;
    }

    sqrtx = 1 - x;
    sqrtx = ti_arm_sqrt(sqrtx);

    float x2 = x * x;

    acosApprox = gAsinConsts[0];
    acosApprox += gAsinConsts[1] * x;
    float x3 = x2 * x;
    float x4 = x2 * x2;
    float x5 = x2 * x3;
    float x6 = x3 * x3;

    acosApprox += gAsinConsts[2] * x2;
    acosApprox += gAsinConsts[3] * x3;
    acosApprox += gAsinConsts[4] * x4;
    acosApprox += gAsinConsts[5] * x5;
    acosApprox += gAsinConsts[6] * x6;

    acosApprox *= sqrtx;

    if (sign)
        acosApprox = acosApprox - gPiConsts[0];
    else
        acosApprox = gPiConsts[0] - acosApprox;

    acosApprox = gPiConsts[0] - acosApprox;

    return acosApprox;
}

inline TRIG_TEXT_SECTION float ti_arm_atan(float x)
{
    uint8_t sign = 0;
    uint8_t complement = 0;

    if (x < 0)
    {
        x = -x;
        sign = 1;
    }

    float atanApprox = gAtanConsts[0];

    if (ti_arm_abs(x) > 1)
    {
        x = 1.0f / (float) x;
        complement = 1;
    }

    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x2 * x2;
    float x5 = x3 * x2;
    float x6 = x3 * x3;
    float x7 = x3 * x4;

    atanApprox += gAtanConsts[1] * x;
    atanApprox += gAtanConsts[2] * x2;
    atanApprox += gAtanConsts[3] * x3;
    atanApprox += gAtanConsts[4] * x4;
    atanApprox += gAtanConsts[5] * x5;
    atanApprox += gAtanConsts[6] * x6;
    atanApprox += gAtanConsts[7] * x7;

    if (complement)
        atanApprox = gPiConsts[0] - atanApprox;
    if (sign)
        atanApprox = -atanApprox;

    return atanApprox;
}

inline TRIG_TEXT_SECTION float ti_arm_atan2(float y, float x)
{
    float ratio;
    float k1 = gPiConsts[0];
    float k2 = 0;
    uint8_t sign = 0;
    uint8_t complement = 0;

    if (ti_arm_abs(x) > ti_arm_abs(y))
    {
        ratio = y / (float) x;
    }
    else
    {
        ratio = x / (float) y;
        complement = 1;
    }

    if (ratio < 0)
    {
        ratio = -ratio;
        sign = 1;
    }

    if (x < 0)
    {
        if (y > 0)
        {
            k2 = gPiConsts[1];
        }
        else
        {
            k2 = -gPiConsts[1];
        }
    }
    if (y < 0)
    {
        k1 = -gPiConsts[0];
    }

    float x2 = ratio * ratio;
    float x3 = x2 * ratio;
    float x4 = x2 * x2;
    float x5 = x3 * x2;
    float x6 = x3 * x3;
    float x7 = x3 * x4;

    float atanApprox = gAtanConsts[0];
    atanApprox += gAtanConsts[1] * ratio;
    atanApprox += gAtanConsts[2] * x2;
    atanApprox += gAtanConsts[3] * x3;
    atanApprox += gAtanConsts[4] * x4;
    atanApprox += gAtanConsts[5] * x5;
    atanApprox += gAtanConsts[6] * x6;
    atanApprox += gAtanConsts[7] * x7;

    if (sign)
        atanApprox = -atanApprox;
    if (complement)
        atanApprox = k1 - atanApprox;
    else
        atanApprox = k2 + atanApprox;

    return atanApprox;
}







