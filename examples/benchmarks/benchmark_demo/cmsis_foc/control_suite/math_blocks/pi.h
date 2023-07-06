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
File name:       PI.H
===================================================================================*/


#ifndef __PI_H__
#define __PI_H__

typedef struct
{
    _iq  Ref;               // Input: reference set-point
    _iq  Fbk;               // Input: feedback
    _iq  Out;               // Output: controller output
    _iq  Kp;                // Parameter: proportional loop gain
    _iq  Ki;                // Parameter: integral gain
    _iq  Umax;          // Parameter: upper saturation limit
    _iq  Umin;          // Parameter: lower saturation limit
    _iq  up;                // Data: proportional term
    _iq  ui;                // Data: integral term
    _iq  v1;                // Data: pre-saturated controller output
    _iq  i1;                // Data: integrator storage: ui(k-1)
    _iq  w1;                // Data: saturation record: [u(k-1) - v(k-1)]
} PI_CONTROLLER;


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/

#define PI_CONTROLLER_DEFAULTS {        \
        0,           \
        0,           \
        0,           \
        _IQ(1.0),    \
        _IQ(0.0),    \
        _IQ(1.0),    \
        _IQ(-1.0),   \
        _IQ(0.0),    \
        _IQ(0.0),    \
        _IQ(0.0),    \
        _IQ(0.0),    \
        _IQ(1.0)     \
    }

/*------------------------------------------------------------------------------
    PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PI_MACRO(v)                                             \
    \
    /* proportional term */                                     \
    v.up = _IQmpy(v.Kp, (v.Ref - v.Fbk));                       \
    \
    /* integral term */                                         \
    v.ui = (v.Out == v.v1)?(_IQmpy(v.Ki, v.up)+ v.i1) : v.i1;   \
    v.i1 = v.ui;                                                \
    \
    /* control output */                                        \
    v.v1 = v.up + v.ui;                                         \
    v.Out= _IQsat(v.v1, v.Umax, v.Umin);                        \

    // ***********************************************************************************
    //   This macro works with angles as inputs, hence error is rolled within -pi to +pi
    // ***********************************************************************************
#define PI_POS_MACRO(v)                                         \
    /* proportional term */                                     \
    v.up = v.Ref - v.Fbk;                                       \
    if (v.up >= _IQ(0.5))                                       \
        v.up -= _IQ(1.0);             /* roll in the error */     \
    else if (v.up <= _IQ(-0.5))                                 \
        v.up += _IQ(1.0);             /* roll in the error */     \
    \
    /* integral term */                                         \
    v.up = _IQmpy(v.Kp, v.up);                                  \
    v.ui = (v.Out == v.v1)?(_IQmpy(v.Ki, v.up)+ v.i1) : v.i1;   \
    v.i1 = v.ui;                                                \
    \
    /* control output */                                        \
    v.v1 = v.up + v.ui;                                         \
    v.Out= _IQsat(v.v1, v.Umax, v.Umin);                        \


#endif // __PI_H__

