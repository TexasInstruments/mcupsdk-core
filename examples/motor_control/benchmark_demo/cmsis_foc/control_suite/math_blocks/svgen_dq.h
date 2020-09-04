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
File name:       SVGEN_DQ.H  (IQ version)

Originator: Digital Control Systems Group
            Texas Instruments

Description:
Header file containing constants, data type, and macro  definitions for the SVGEN_DQ module .
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 10-15-2009 Version 1.0
------------------------------------------------------------------------------*/
#ifndef __SVGEN_DQ_H__
#define __SVGEN_DQ_H__

typedef struct
{
    _iq  Ualpha;            // Input: reference alpha-axis phase voltage
    _iq  Ubeta;         // Input: reference beta-axis phase voltage
    _iq  Ta;                // Output: reference phase-a switching function
    _iq  Tb;                // Output: reference phase-b switching function
    _iq  Tc;                // Output: reference phase-c switching function
} SVGENDQ;

typedef SVGENDQ *SVGENDQ_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SVGENDQ object.
-----------------------------------------------------------------------------*/
#define SVGENDQ_DEFAULTS { 0,0,0,0,0 }

/*------------------------------------------------------------------------------
    Space Vector PWM Generator (SVGEN_DQ) Macro Definition
------------------------------------------------------------------------------*/


_iq Va, Vb, Vc, t1, t2, temp_sv1, temp_sv2;
uint16_t Sector = 0;  // Sector is treated as Q0 - independently with global Q

#define SVGEN_MACRO(v)                                                          \
    \
    Sector = 0;                                                                 \
    temp_sv1=_IQmpy(_IQ(0.5),v.Ubeta);                                          \
    temp_sv2=_IQmpy(_IQ(0.8660254),v.Ualpha);   /* 0.8660254 = sqrt(3)/2*/      \
    \
    /* Inverse clarke transformation */                                             \
    Va = v.Ubeta;                                                               \
    Vb = -temp_sv1 + temp_sv2;                                                  \
    Vc = -temp_sv1 - temp_sv2;                                                  \
    /* 60 degree Sector determination */                                            \
    if (Va>_IQ(0)) Sector = 1;                                                  \
    if (Vb>_IQ(0)) Sector = Sector+2;                                           \
    if (Vc>_IQ(0)) Sector = Sector+4;                                           \
    /* X,Y,Z (Va,Vb,Vc) calculations X = Va, Y = Vb, Z = Vc */                      \
    Va = v.Ubeta;                                                               \
    Vb = temp_sv1 + temp_sv2;                                                   \
    Vc = temp_sv1 - temp_sv2;                                                   \
    /* Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)*/                  \
    \
    switch(Sector)                                                                  \
    {                                                                               \
    case 0:                                                                     \
        v.Ta = _IQ(0.5);                                                            \
        v.Tb = _IQ(0.5);                                                            \
        v.Tc = _IQ(0.5);                                                            \
        break;                                                                      \
    case 1:   /*Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)*/                   \
        t1 = Vc;                                                                    \
        t2 = Vb;                                                                    \
        v.Tb = (_IQ(1)-t1-t2)/2;    /* tbon = (1-t1-t2)/2   */                      \
        /*v.Tb = (_IQ(1)-t1-t2)>>1; // tbon = (1-t1-t2)/2   */                      \
        v.Ta = v.Tb+t1;             /* taon = tbon+t1       */                      \
        v.Tc = v.Ta+t2;             /* tcon = taon+t2       */                      \
        break;                                                                      \
    case 2:   /* Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)*/                 \
        t1 = Vb;                                                                    \
        t2 = -Va;                                                                   \
        v.Ta = (_IQ(1)-t1-t2)/2;    /*  taon = (1-t1-t2)/2  */                      \
        /*  v.Ta = (_IQ(1)-t1-t2)>>1;   //  taon = (1-t1-t2)/2  */                      \
        v.Tc = v.Ta+t1;             /*  tcon = taon+t1      */                      \
        v.Tb = v.Tc+t2;             /*  tbon = tcon+t2      */                      \
        break;                                                                      \
    case 3:   /* Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)*/                 \
        t1 = -Vc;                                                                   \
        t2 = Va;                                                                    \
        v.Ta = (_IQ(1)-t1-t2)/2;    /*  taon = (1-t1-t2)/2  */                      \
        /*  v.Ta = (_IQ(1)-t1-t2)>>1;   //  taon = (1-t1-t2)/2  */                      \
        v.Tb = v.Ta+t1;             /*  tbon = taon+t1      */                      \
        v.Tc = v.Tb+t2;             /*  tcon = tbon+t2      */                      \
        break;                                                                      \
    case 4:   /* Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)*/                 \
        t1 = -Va;                                                                   \
        t2 = Vc;                                                                    \
        v.Tc = (_IQ(1)-t1-t2)/2;    /*  tcon = (1-t1-t2)/2  */                      \
        /*  v.Tc = (_IQ(1)-t1-t2)>>1;   //  tcon = (1-t1-t2)/2  */                      \
        v.Tb = v.Tc+t1;             /*  tbon = tcon+t1      */                      \
        v.Ta = v.Tb+t2;             /*  taon = tbon+t2      */                      \
        break;                                                                      \
    case 5:   /* Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)*/                 \
        t1 = Va;                                                                    \
        t2 = -Vb;                   /*  tbon = (1-t1-t2)/2  */                      \
        v.Tb = (_IQ(1)-t1-t2)/2;    /*  tcon = tbon+t1      */                      \
        /*  v.Tb = (_IQ(1)-t1-t2)>>1;   //  tcon = tbon+t1      */                      \
        v.Tc = v.Tb+t1;             /*  taon = tcon+t2      */                      \
        v.Ta = v.Tc+t2;                                                             \
        break;                                                                      \
    case 6:   /* Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)*/                \
        t1 = -Vb;                                                                   \
        t2 = -Vc;                                                                   \
        v.Tc = (_IQ(1)-t1-t2)/2;    /*  tcon = (1-t1-t2)/2  */                      \
        /*  v.Tc = (_IQ(1)-t1-t2)>>1;   //  tcon = (1-t1-t2)/2  */                      \
        v.Ta = v.Tc+t1;             /*  taon = tcon+t1      */                      \
        v.Tb = v.Ta+t2;             /*  tbon = taon+t2      */                      \
        break;                                                                      \
    }                                                                               \
    /*  Convert the unsigned GLOBAL_Q format (ranged (0,1)) ->.. */                 \
    /*  ..signed GLOBAL_Q format (ranged (-1,1))*/                                  \
    v.Ta = (v.Ta-_IQ(0.5))*2;                                                       \
    /*v.Ta = (v.Ta-_IQ(0.5))<<1;*/                                                  \
    v.Tb = (v.Tb-_IQ(0.5))*2;                                                       \
    /*v.Tb = (v.Tb-_IQ(0.5))<<1;*/                                                  \
    v.Tc = (v.Tc-_IQ(0.5))*2;                                                       \
    /*v.Tc = (v.Tc-_IQ(0.5))<<1;*/
#endif // __SVGEN_DQ_H__
