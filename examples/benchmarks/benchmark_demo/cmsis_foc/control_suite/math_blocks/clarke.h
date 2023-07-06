/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */

/* =================================================================================
File name:       CLARKE.H
===================================================================================*/


#ifndef __CLARKE_H__
#define __CLARKE_H__

typedef struct
{
    _iq  As;        // Input: phase-a stator variable
    _iq  Bs;            // Input: phase-b stator variable
    _iq  Cs;            // Input: phase-c stator variable
    _iq  Alpha;     // Output: stationary d-axis stator variable
    _iq  Beta;      // Output: stationary q-axis stator variable
} CLARKE;

/*-----------------------------------------------------------------------------
    Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/
#define CLARKE_DEFAULTS { 0, \
        0, \
        0, \
        0, \
        0, \
    }

/*------------------------------------------------------------------------------
    CLARKE Transformation Macro Definition
------------------------------------------------------------------------------*/

//  1/sqrt(3) = 0.57735026918963
#define  ONEbySQRT3   0.57735026918963    /* 1/sqrt(3) */


// Clarke transform macro (with 2 currents)
//==========================================
#define CLARKE_MACRO(v)                                     \
    v.Alpha = v.As;                                             \
    v.Beta = _IQmpy((v.As +_IQmpy2(v.Bs)),_IQ(ONEbySQRT3));


// Clarke transform macro (with 3 currents)
//==========================================
#define CLARKE1_MACRO(v)                                    \
    v.Alpha = v.As;                                             \
    v.Beta  = _IQmpy((v.Bs - v.Cs),_IQ(ONEbySQRT3));

#endif // __CLARKE_H__

