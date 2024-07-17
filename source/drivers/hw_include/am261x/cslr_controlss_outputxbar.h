/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *
 *  Name        : cslr_controlss_outputxbar.h
*/
#ifndef CSLR_CONTROLSS_OUTPUTXBAR_H_
#define CSLR_CONTROLSS_OUTPUTXBAR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/

/**************************************************************************
    XBAR INPUT Macros
**************************************************************************/

/******************* G0 *********************/
#define OUTPUTXBAR_EPWM0_TRIPOUT    (0x00000001)
#define OUTPUTXBAR_EPWM1_TRIPOUT    (0x00000002)
#define OUTPUTXBAR_EPWM2_TRIPOUT    (0x00000004)
#define OUTPUTXBAR_EPWM3_TRIPOUT    (0x00000008)
#define OUTPUTXBAR_EPWM4_TRIPOUT    (0x00000010)
#define OUTPUTXBAR_EPWM5_TRIPOUT    (0x00000020)
#define OUTPUTXBAR_EPWM6_TRIPOUT    (0x00000040)
#define OUTPUTXBAR_EPWM7_TRIPOUT    (0x00000080)
#define OUTPUTXBAR_EPWM8_TRIPOUT    (0x00000100)
#define OUTPUTXBAR_EPWM9_TRIPOUT    (0x00000200)

/******************* G1 *********************/
#define OUTPUTXBAR_EPWM0_SOCA    (0x00000001)
#define OUTPUTXBAR_EPWM1_SOCA    (0x00000002)
#define OUTPUTXBAR_EPWM2_SOCA    (0x00000004)
#define OUTPUTXBAR_EPWM3_SOCA    (0x00000008)
#define OUTPUTXBAR_EPWM4_SOCA    (0x00000010)
#define OUTPUTXBAR_EPWM5_SOCA    (0x00000020)
#define OUTPUTXBAR_EPWM6_SOCA    (0x00000040)
#define OUTPUTXBAR_EPWM7_SOCA    (0x00000080)
#define OUTPUTXBAR_EPWM8_SOCA    (0x00000100)
#define OUTPUTXBAR_EPWM9_SOCA    (0x00000200)

/******************* G2 *********************/
#define OUTPUTXBAR_EPWM0_SOCB    (0x00000001)
#define OUTPUTXBAR_EPWM1_SOCB    (0x00000002)
#define OUTPUTXBAR_EPWM2_SOCB    (0x00000004)
#define OUTPUTXBAR_EPWM3_SOCB    (0x00000008)
#define OUTPUTXBAR_EPWM4_SOCB    (0x00000010)
#define OUTPUTXBAR_EPWM5_SOCB    (0x00000020)
#define OUTPUTXBAR_EPWM6_SOCB    (0x00000040)
#define OUTPUTXBAR_EPWM7_SOCB    (0x00000080)
#define OUTPUTXBAR_EPWM8_SOCB    (0x00000100)
#define OUTPUTXBAR_EPWM9_SOCB    (0x00000200)

/******************* G3 *********************/
#define OUTPUTXBAR_DEL0_ACTIVE    (0x00000001)
#define OUTPUTXBAR_DEL1_ACTIVE    (0x00000002)
#define OUTPUTXBAR_DEL2_ACTIVE    (0x00000004)
#define OUTPUTXBAR_DEL3_ACTIVE    (0x00000008)
#define OUTPUTXBAR_DEL4_ACTIVE    (0x00000010)
#define OUTPUTXBAR_DEL5_ACTIVE    (0x00000020)
#define OUTPUTXBAR_DEL6_ACTIVE    (0x00000040)
#define OUTPUTXBAR_DEL7_ACTIVE    (0x00000080)
#define OUTPUTXBAR_DEL8_ACTIVE    (0x00000100)
#define OUTPUTXBAR_DEL9_ACTIVE    (0x00000200)

/******************* G4 *********************/
#define OUTPUTXBAR_DEL0_TRIP    (0x00000001)
#define OUTPUTXBAR_DEL1_TRIP    (0x00000002)
#define OUTPUTXBAR_DEL2_TRIP    (0x00000004)
#define OUTPUTXBAR_DEL3_TRIP    (0x00000008)
#define OUTPUTXBAR_DEL4_TRIP    (0x00000010)
#define OUTPUTXBAR_DEL5_TRIP    (0x00000020)
#define OUTPUTXBAR_DEL6_TRIP    (0x00000040)
#define OUTPUTXBAR_DEL7_TRIP    (0x00000080)
#define OUTPUTXBAR_DEL8_TRIP    (0x00000100)
#define OUTPUTXBAR_DEL9_TRIP    (0x00000200)

/******************* G5 *********************/
#define OUTPUTXBAR_SD0_FILT0_CEVT1    (0x00000001)
#define OUTPUTXBAR_SD0_FILT0_CEVT2    (0x00000002)
#define OUTPUTXBAR_SD0_FILT0_CMPHZ    (0x00000004)
#define OUTPUTXBAR_SD0_FILT1_CEVT1    (0x00000008)
#define OUTPUTXBAR_SD0_FILT1_CEVT2    (0x00000010)
#define OUTPUTXBAR_SD0_FILT1_CMPHZ    (0x00000020)
#define OUTPUTXBAR_SD0_FILT2_CEVT1    (0x00000040)
#define OUTPUTXBAR_SD0_FILT2_CEVT2    (0x00000080)
#define OUTPUTXBAR_SD0_FILT2_CMPHZ    (0x00000100)
#define OUTPUTXBAR_SD0_FILT3_CEVT1    (0x00000200)
#define OUTPUTXBAR_SD0_FILT3_CEVT2    (0x00000400)
#define OUTPUTXBAR_SD0_FILT3_CMPHZ    (0x00000800)
#define OUTPUTXBAR_SD1_FILT0_CEVT1    (0x00001000)
#define OUTPUTXBAR_SD1_FILT0_CEVT2    (0x00002000)
#define OUTPUTXBAR_SD1_FILT0_CMPHZ    (0x00004000)
#define OUTPUTXBAR_SD1_FILT1_CEVT1    (0x00008000)
#define OUTPUTXBAR_SD1_FILT1_CEVT2    (0x00010000)
#define OUTPUTXBAR_SD1_FILT1_CMPHZ    (0x00020000)
#define OUTPUTXBAR_SD1_FILT2_CEVT1    (0x00040000)
#define OUTPUTXBAR_SD1_FILT2_CEVT2    (0x00080000)
#define OUTPUTXBAR_SD1_FILT2_CMPHZ    (0x00100000)
#define OUTPUTXBAR_SD1_FILT3_CEVT1    (0x00200000)
#define OUTPUTXBAR_SD1_FILT3_CEVT2    (0x00400000)
#define OUTPUTXBAR_SD1_FILT3_CMPHZ    (0x00800000)

/******************* G6 *********************/
#define OUTPUTXBAR_CMPSSA0_CTRIPL    (0x00000001)
#define OUTPUTXBAR_CMPSSA0_CTRIPH    (0x00000002)
#define OUTPUTXBAR_CMPSSA1_CTRIPL    (0x00000004)
#define OUTPUTXBAR_CMPSSA1_CTRIPH    (0x00000008)
#define OUTPUTXBAR_CMPSSA2_CTRIPL    (0x00000010)
#define OUTPUTXBAR_CMPSSA2_CTRIPH    (0x00000020)
#define OUTPUTXBAR_CMPSSA3_CTRIPL    (0x00000040)
#define OUTPUTXBAR_CMPSSA3_CTRIPH    (0x00000080)
#define OUTPUTXBAR_CMPSSA4_CTRIPL    (0x00000100)
#define OUTPUTXBAR_CMPSSA4_CTRIPH    (0x00000200)
#define OUTPUTXBAR_CMPSSA5_CTRIPL    (0x00000400)
#define OUTPUTXBAR_CMPSSA5_CTRIPH    (0x00000800)
#define OUTPUTXBAR_CMPSSA6_CTRIPL    (0x00001000)
#define OUTPUTXBAR_CMPSSA6_CTRIPH    (0x00002000)
#define OUTPUTXBAR_CMPSSA7_CTRIPL    (0x00004000)
#define OUTPUTXBAR_CMPSSA7_CTRIPH    (0x00008000)
#define OUTPUTXBAR_CMPSSA8_CTRIPL    (0x00010000)
#define OUTPUTXBAR_CMPSSA8_CTRIPH    (0x00020000)
#define OUTPUTXBAR_CMPSSA9_CTRIPL    (0x00040000)
#define OUTPUTXBAR_CMPSSA9_CTRIPH    (0x00080000)

/******************* G7 *********************/
#define OUTPUTXBAR_ADC0_EVT1    (0x00000001)
#define OUTPUTXBAR_ADC0_EVT2    (0x00000002)
#define OUTPUTXBAR_ADC0_EVT3    (0x00000004)
#define OUTPUTXBAR_ADC0_EVT4    (0x00000008)
#define OUTPUTXBAR_ADC1_EVT1    (0x00000010)
#define OUTPUTXBAR_ADC1_EVT2    (0x00000020)
#define OUTPUTXBAR_ADC1_EVT3    (0x00000040)
#define OUTPUTXBAR_ADC1_EVT4    (0x00000080)
#define OUTPUTXBAR_ADC2_EVT1    (0x00000100)
#define OUTPUTXBAR_ADC2_EVT2    (0x00000200)
#define OUTPUTXBAR_ADC2_EVT3    (0x00000400)
#define OUTPUTXBAR_ADC2_EVT4    (0x00000800)

/******************* G8 *********************/
#define OUTPUTXBAR_EPWM_SYNCOUT_XBAR0   (0x00000001)
#define OUTPUTXBAR_EPWM_SYNCOUT_XBAR1   (0x00000002)
#define OUTPUTXBAR_EPWM_SYNCOUT_XBAR2   (0x00000004)
#define OUTPUTXBAR_EPWM_SYNCOUT_XBAR3   (0x00000008)
#define OUTPUTXBAR_EQEP0_I_OUT          (0x00000010)
#define OUTPUTXBAR_EQEP0_S_OUT          (0x00000020)
#define OUTPUTXBAR_EQEP1_I_OUT          (0x00000040)
#define OUTPUTXBAR_EQEP1_S_OUT          (0x00000080)
#define OUTPUTXBAR_ECAP0_OUT            (0x00000400)
#define OUTPUTXBAR_ECAP1_OUT            (0x00000800)
#define OUTPUTXBAR_ECAP2_OUT            (0x00001000)
#define OUTPUTXBAR_ECAP3_OUT            (0x00002000)
#define OUTPUTXBAR_ECAP4_OUT            (0x00004000)
#define OUTPUTXBAR_ECAP5_OUT            (0x00008000)
#define OUTPUTXBAR_ECAP6_OUT            (0x00010000)
#define OUTPUTXBAR_ECAP7_OUT            (0x00020000)

/******************* G9 *********************/
#define OUTPUTXBAR_FSIRX0_RX_TRIG0    (0x00000001)
#define OUTPUTXBAR_FSIRX0_RX_TRIG1    (0x00000002)
#define OUTPUTXBAR_FSIRX0_RX_TRIG2    (0x00000004)
#define OUTPUTXBAR_FSIRX0_RX_TRIG3    (0x00000008)
#define OUTPUTXBAR_INPUTXBAR_OUT7     (0x00010000)
#define OUTPUTXBAR_INPUTXBAR_OUT15    (0x00020000)
#define OUTPUTXBAR_INPUTXBAR_OUT23    (0x00040000)
#define OUTPUTXBAR_INPUTXBAR_OUT31    (0x00080000)
#define OUTPUTXBAR_INTXBAR_OUT7       (0x01000000)
#define OUTPUTXBAR_INTXBAR_OUT15      (0x02000000)
    


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint8_t  Resv_16[12];
    volatile uint32_t OUTPUTXBAR_STATUS;
    volatile uint32_t OUTPUTXBAR_FLAGINVERT;
    volatile uint32_t OUTPUTXBAR_FLAG;
    volatile uint32_t OUTPUTXBAR_FLAG_CLR;
    volatile uint32_t OUTPUTXBAR_FLAGFORCE;
    volatile uint32_t OUTPUTXBAR_OUTLATCH;
    volatile uint32_t OUTPUTXBAR_OUTSTRETCH;
    volatile uint32_t OUTPUTXBAR_OUTLENGTH;
    volatile uint32_t OUTPUTXBAR_OUTINVERT;
    volatile uint8_t  Resv_256[204];
    volatile uint32_t OUTPUTXBAR0_G0;
    volatile uint32_t OUTPUTXBAR0_G1;
    volatile uint32_t OUTPUTXBAR0_G2;
    volatile uint32_t OUTPUTXBAR0_G3;
    volatile uint32_t OUTPUTXBAR0_G4;
    volatile uint32_t OUTPUTXBAR0_G5;
    volatile uint32_t OUTPUTXBAR0_G6;
    volatile uint32_t OUTPUTXBAR0_G7;
    volatile uint32_t OUTPUTXBAR0_G8;
    volatile uint32_t OUTPUTXBAR0_G9;
    volatile uint8_t  Resv_320[24];
    volatile uint32_t OUTPUTXBAR1_G0;
    volatile uint32_t OUTPUTXBAR1_G1;
    volatile uint32_t OUTPUTXBAR1_G2;
    volatile uint32_t OUTPUTXBAR1_G3;
    volatile uint32_t OUTPUTXBAR1_G4;
    volatile uint32_t OUTPUTXBAR1_G5;
    volatile uint32_t OUTPUTXBAR1_G6;
    volatile uint32_t OUTPUTXBAR1_G7;
    volatile uint32_t OUTPUTXBAR1_G8;
    volatile uint32_t OUTPUTXBAR1_G9;
    volatile uint8_t  Resv_384[24];
    volatile uint32_t OUTPUTXBAR2_G0;
    volatile uint32_t OUTPUTXBAR2_G1;
    volatile uint32_t OUTPUTXBAR2_G2;
    volatile uint32_t OUTPUTXBAR2_G3;
    volatile uint32_t OUTPUTXBAR2_G4;
    volatile uint32_t OUTPUTXBAR2_G5;
    volatile uint32_t OUTPUTXBAR2_G6;
    volatile uint32_t OUTPUTXBAR2_G7;
    volatile uint32_t OUTPUTXBAR2_G8;
    volatile uint32_t OUTPUTXBAR2_G9;
    volatile uint8_t  Resv_448[24];
    volatile uint32_t OUTPUTXBAR3_G0;
    volatile uint32_t OUTPUTXBAR3_G1;
    volatile uint32_t OUTPUTXBAR3_G2;
    volatile uint32_t OUTPUTXBAR3_G3;
    volatile uint32_t OUTPUTXBAR3_G4;
    volatile uint32_t OUTPUTXBAR3_G5;
    volatile uint32_t OUTPUTXBAR3_G6;
    volatile uint32_t OUTPUTXBAR3_G7;
    volatile uint32_t OUTPUTXBAR3_G8;
    volatile uint32_t OUTPUTXBAR3_G9;
    volatile uint8_t  Resv_512[24];
    volatile uint32_t OUTPUTXBAR4_G0;
    volatile uint32_t OUTPUTXBAR4_G1;
    volatile uint32_t OUTPUTXBAR4_G2;
    volatile uint32_t OUTPUTXBAR4_G3;
    volatile uint32_t OUTPUTXBAR4_G4;
    volatile uint32_t OUTPUTXBAR4_G5;
    volatile uint32_t OUTPUTXBAR4_G6;
    volatile uint32_t OUTPUTXBAR4_G7;
    volatile uint32_t OUTPUTXBAR4_G8;
    volatile uint32_t OUTPUTXBAR4_G9;
    volatile uint8_t  Resv_576[24];
    volatile uint32_t OUTPUTXBAR5_G0;
    volatile uint32_t OUTPUTXBAR5_G1;
    volatile uint32_t OUTPUTXBAR5_G2;
    volatile uint32_t OUTPUTXBAR5_G3;
    volatile uint32_t OUTPUTXBAR5_G4;
    volatile uint32_t OUTPUTXBAR5_G5;
    volatile uint32_t OUTPUTXBAR5_G6;
    volatile uint32_t OUTPUTXBAR5_G7;
    volatile uint32_t OUTPUTXBAR5_G8;
    volatile uint32_t OUTPUTXBAR5_G9;
    volatile uint8_t  Resv_640[24];
    volatile uint32_t OUTPUTXBAR6_G0;
    volatile uint32_t OUTPUTXBAR6_G1;
    volatile uint32_t OUTPUTXBAR6_G2;
    volatile uint32_t OUTPUTXBAR6_G3;
    volatile uint32_t OUTPUTXBAR6_G4;
    volatile uint32_t OUTPUTXBAR6_G5;
    volatile uint32_t OUTPUTXBAR6_G6;
    volatile uint32_t OUTPUTXBAR6_G7;
    volatile uint32_t OUTPUTXBAR6_G8;
    volatile uint32_t OUTPUTXBAR6_G9;
    volatile uint8_t  Resv_704[24];
    volatile uint32_t OUTPUTXBAR7_G0;
    volatile uint32_t OUTPUTXBAR7_G1;
    volatile uint32_t OUTPUTXBAR7_G2;
    volatile uint32_t OUTPUTXBAR7_G3;
    volatile uint32_t OUTPUTXBAR7_G4;
    volatile uint32_t OUTPUTXBAR7_G5;
    volatile uint32_t OUTPUTXBAR7_G6;
    volatile uint32_t OUTPUTXBAR7_G7;
    volatile uint32_t OUTPUTXBAR7_G8;
    volatile uint32_t OUTPUTXBAR7_G9;
    volatile uint8_t  Resv_768[24];
    volatile uint32_t OUTPUTXBAR8_G0;
    volatile uint32_t OUTPUTXBAR8_G1;
    volatile uint32_t OUTPUTXBAR8_G2;
    volatile uint32_t OUTPUTXBAR8_G3;
    volatile uint32_t OUTPUTXBAR8_G4;
    volatile uint32_t OUTPUTXBAR8_G5;
    volatile uint32_t OUTPUTXBAR8_G6;
    volatile uint32_t OUTPUTXBAR8_G7;
    volatile uint32_t OUTPUTXBAR8_G8;
    volatile uint32_t OUTPUTXBAR8_G9;
    volatile uint8_t  Resv_832[24];
    volatile uint32_t OUTPUTXBAR9_G0;
    volatile uint32_t OUTPUTXBAR9_G1;
    volatile uint32_t OUTPUTXBAR9_G2;
    volatile uint32_t OUTPUTXBAR9_G3;
    volatile uint32_t OUTPUTXBAR9_G4;
    volatile uint32_t OUTPUTXBAR9_G5;
    volatile uint32_t OUTPUTXBAR9_G6;
    volatile uint32_t OUTPUTXBAR9_G7;
    volatile uint32_t OUTPUTXBAR9_G8;
    volatile uint32_t OUTPUTXBAR9_G9;
    volatile uint8_t  Resv_896[24];
    volatile uint32_t OUTPUTXBAR10_G0;
    volatile uint32_t OUTPUTXBAR10_G1;
    volatile uint32_t OUTPUTXBAR10_G2;
    volatile uint32_t OUTPUTXBAR10_G3;
    volatile uint32_t OUTPUTXBAR10_G4;
    volatile uint32_t OUTPUTXBAR10_G5;
    volatile uint32_t OUTPUTXBAR10_G6;
    volatile uint32_t OUTPUTXBAR10_G7;
    volatile uint32_t OUTPUTXBAR10_G8;
    volatile uint32_t OUTPUTXBAR10_G9;
    volatile uint8_t  Resv_960[24];
    volatile uint32_t OUTPUTXBAR11_G0;
    volatile uint32_t OUTPUTXBAR11_G1;
    volatile uint32_t OUTPUTXBAR11_G2;
    volatile uint32_t OUTPUTXBAR11_G3;
    volatile uint32_t OUTPUTXBAR11_G4;
    volatile uint32_t OUTPUTXBAR11_G5;
    volatile uint32_t OUTPUTXBAR11_G6;
    volatile uint32_t OUTPUTXBAR11_G7;
    volatile uint32_t OUTPUTXBAR11_G8;
    volatile uint32_t OUTPUTXBAR11_G9;
    volatile uint8_t  Resv_1024[24];
    volatile uint32_t OUTPUTXBAR12_G0;
    volatile uint32_t OUTPUTXBAR12_G1;
    volatile uint32_t OUTPUTXBAR12_G2;
    volatile uint32_t OUTPUTXBAR12_G3;
    volatile uint32_t OUTPUTXBAR12_G4;
    volatile uint32_t OUTPUTXBAR12_G5;
    volatile uint32_t OUTPUTXBAR12_G6;
    volatile uint32_t OUTPUTXBAR12_G7;
    volatile uint32_t OUTPUTXBAR12_G8;
    volatile uint32_t OUTPUTXBAR12_G9;
    volatile uint8_t  Resv_1088[24];
    volatile uint32_t OUTPUTXBAR13_G0;
    volatile uint32_t OUTPUTXBAR13_G1;
    volatile uint32_t OUTPUTXBAR13_G2;
    volatile uint32_t OUTPUTXBAR13_G3;
    volatile uint32_t OUTPUTXBAR13_G4;
    volatile uint32_t OUTPUTXBAR13_G5;
    volatile uint32_t OUTPUTXBAR13_G6;
    volatile uint32_t OUTPUTXBAR13_G7;
    volatile uint32_t OUTPUTXBAR13_G8;
    volatile uint32_t OUTPUTXBAR13_G9;
    volatile uint8_t  Resv_1152[24];
    volatile uint32_t OUTPUTXBAR14_G0;
    volatile uint32_t OUTPUTXBAR14_G1;
    volatile uint32_t OUTPUTXBAR14_G2;
    volatile uint32_t OUTPUTXBAR14_G3;
    volatile uint32_t OUTPUTXBAR14_G4;
    volatile uint32_t OUTPUTXBAR14_G5;
    volatile uint32_t OUTPUTXBAR14_G6;
    volatile uint32_t OUTPUTXBAR14_G7;
    volatile uint32_t OUTPUTXBAR14_G8;
    volatile uint32_t OUTPUTXBAR14_G9;
    volatile uint8_t  Resv_1216[24];
    volatile uint32_t OUTPUTXBAR15_G0;
    volatile uint32_t OUTPUTXBAR15_G1;
    volatile uint32_t OUTPUTXBAR15_G2;
    volatile uint32_t OUTPUTXBAR15_G3;
    volatile uint32_t OUTPUTXBAR15_G4;
    volatile uint32_t OUTPUTXBAR15_G5;
    volatile uint32_t OUTPUTXBAR15_G6;
    volatile uint32_t OUTPUTXBAR15_G7;
    volatile uint32_t OUTPUTXBAR15_G8;
    volatile uint32_t OUTPUTXBAR15_G9;
} CSL_controlss_outputxbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CONTROLSS_OUTPUTXBAR_PID                                           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_STATUS                             (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGINVERT                         (0x00000014U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG                               (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR                           (0x0000001CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGFORCE                          (0x00000020U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLATCH                           (0x00000024U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTSTRETCH                         (0x00000028U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLENGTH                          (0x0000002CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTINVERT                          (0x00000030U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0                                (0x00000100U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G1                                (0x00000104U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G2                                (0x00000108U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G3                                (0x0000010CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G4                                (0x00000110U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G5                                (0x00000114U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G6                                (0x00000118U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G7                                (0x0000011CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G8                                (0x00000120U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G9                                (0x00000124U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G0                                (0x00000140U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G1                                (0x00000144U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G2                                (0x00000148U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G3                                (0x0000014CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G4                                (0x00000150U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G5                                (0x00000154U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G6                                (0x00000158U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G7                                (0x0000015CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8                                (0x00000160U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9                                (0x00000164U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G0                                (0x00000180U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G1                                (0x00000184U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G2                                (0x00000188U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G3                                (0x0000018CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G4                                (0x00000190U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G5                                (0x00000194U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G6                                (0x00000198U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G7                                (0x0000019CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8                                (0x000001A0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9                                (0x000001A4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G0                                (0x000001C0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G1                                (0x000001C4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G2                                (0x000001C8U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G3                                (0x000001CCU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G4                                (0x000001D0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G5                                (0x000001D4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G6                                (0x000001D8U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G7                                (0x000001DCU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8                                (0x000001E0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9                                (0x000001E4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G0                                (0x00000200U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G1                                (0x00000204U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G2                                (0x00000208U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G3                                (0x0000020CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G4                                (0x00000210U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G5                                (0x00000214U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G6                                (0x00000218U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G7                                (0x0000021CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8                                (0x00000220U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9                                (0x00000224U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G0                                (0x00000240U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G1                                (0x00000244U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G2                                (0x00000248U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G3                                (0x0000024CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G4                                (0x00000250U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G5                                (0x00000254U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G6                                (0x00000258U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G7                                (0x0000025CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8                                (0x00000260U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9                                (0x00000264U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G0                                (0x00000280U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G1                                (0x00000284U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G2                                (0x00000288U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G3                                (0x0000028CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G4                                (0x00000290U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G5                                (0x00000294U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G6                                (0x00000298U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G7                                (0x0000029CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8                                (0x000002A0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9                                (0x000002A4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G0                                (0x000002C0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G1                                (0x000002C4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G2                                (0x000002C8U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G3                                (0x000002CCU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G4                                (0x000002D0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G5                                (0x000002D4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G6                                (0x000002D8U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G7                                (0x000002DCU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8                                (0x000002E0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9                                (0x000002E4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G0                                (0x00000300U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G1                                (0x00000304U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G2                                (0x00000308U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G3                                (0x0000030CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G4                                (0x00000310U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G5                                (0x00000314U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G6                                (0x00000318U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G7                                (0x0000031CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8                                (0x00000320U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9                                (0x00000324U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G0                                (0x00000340U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G1                                (0x00000344U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G2                                (0x00000348U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G3                                (0x0000034CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G4                                (0x00000350U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G5                                (0x00000354U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G6                                (0x00000358U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G7                                (0x0000035CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8                                (0x00000360U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9                                (0x00000364U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G0                               (0x00000380U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G1                               (0x00000384U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G2                               (0x00000388U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G3                               (0x0000038CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G4                               (0x00000390U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G5                               (0x00000394U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G6                               (0x00000398U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G7                               (0x0000039CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8                               (0x000003A0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9                               (0x000003A4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G0                               (0x000003C0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G1                               (0x000003C4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G2                               (0x000003C8U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G3                               (0x000003CCU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G4                               (0x000003D0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G5                               (0x000003D4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G6                               (0x000003D8U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G7                               (0x000003DCU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8                               (0x000003E0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9                               (0x000003E4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G0                               (0x00000400U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G1                               (0x00000404U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G2                               (0x00000408U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G3                               (0x0000040CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G4                               (0x00000410U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G5                               (0x00000414U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G6                               (0x00000418U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G7                               (0x0000041CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8                               (0x00000420U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9                               (0x00000424U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G0                               (0x00000440U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G1                               (0x00000444U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G2                               (0x00000448U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G3                               (0x0000044CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G4                               (0x00000450U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G5                               (0x00000454U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G6                               (0x00000458U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G7                               (0x0000045CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8                               (0x00000460U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9                               (0x00000464U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G0                               (0x00000480U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G1                               (0x00000484U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G2                               (0x00000488U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G3                               (0x0000048CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G4                               (0x00000490U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G5                               (0x00000494U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G6                               (0x00000498U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G7                               (0x0000049CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8                               (0x000004A0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9                               (0x000004A4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G0                               (0x000004C0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G1                               (0x000004C4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G2                               (0x000004C8U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G3                               (0x000004CCU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G4                               (0x000004D0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G5                               (0x000004D4U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G6                               (0x000004D8U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G7                               (0x000004DCU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8                               (0x000004E0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9                               (0x000004E4U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MINOR_MASK                            (0x0000003FU)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MINOR_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MINOR_RESETVAL                        (0x00000014U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MINOR_MAX                             (0x0000003FU)

#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_CUSTOM_MASK                           (0x000000C0U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_CUSTOM_SHIFT                          (0x00000006U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_CUSTOM_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_CUSTOM_MAX                            (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MAJOR_MASK                            (0x00000700U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MAJOR_SHIFT                           (0x00000008U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MAJOR_RESETVAL                        (0x00000002U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MAJOR_MAX                             (0x00000007U)

#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MISC_MASK                             (0x0000F800U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MISC_SHIFT                            (0x0000000BU)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MISC_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MISC_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MSB16_MASK                            (0xFFFF0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MSB16_SHIFT                           (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MSB16_RESETVAL                        (0x00006180U)
#define CSL_CONTROLSS_OUTPUTXBAR_PID_PID_MSB16_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_PID_RESETVAL                                  (0x61800214U)

/* OUTPUTXBAR_STATUS */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_STATUS_STS_MASK                    (0x0000FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_STATUS_STS_SHIFT                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_STATUS_STS_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_STATUS_STS_MAX                     (0x0000FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_STATUS_RESETVAL                    (0x00000000U)

/* OUTPUTXBAR_FLAGINVERT */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGINVERT_INVERT_MASK             (0x0000FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGINVERT_INVERT_SHIFT            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGINVERT_INVERT_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGINVERT_INVERT_MAX              (0x0000FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGINVERT_RESETVAL                (0x00000000U)

/* OUTPUTXBAR_FLAG */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT0_MASK                     (0x00000001U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT0_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT0_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT0_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT1_MASK                     (0x00000002U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT1_SHIFT                    (0x00000001U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT1_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT1_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT2_MASK                     (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT2_SHIFT                    (0x00000002U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT2_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT2_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT3_MASK                     (0x00000008U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT3_SHIFT                    (0x00000003U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT3_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT3_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT4_MASK                     (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT4_SHIFT                    (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT4_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT4_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT5_MASK                     (0x00000020U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT5_SHIFT                    (0x00000005U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT5_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT5_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT6_MASK                     (0x00000040U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT6_SHIFT                    (0x00000006U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT6_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT6_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT7_MASK                     (0x00000080U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT7_SHIFT                    (0x00000007U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT7_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT7_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT8_MASK                     (0x00000100U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT8_SHIFT                    (0x00000008U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT8_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT8_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT9_MASK                     (0x00000200U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT9_SHIFT                    (0x00000009U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT9_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT9_MAX                      (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT10_MASK                    (0x00000400U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT10_SHIFT                   (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT10_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT10_MAX                     (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT11_MASK                    (0x00000800U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT11_SHIFT                   (0x0000000BU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT11_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT11_MAX                     (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT12_MASK                    (0x00001000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT12_SHIFT                   (0x0000000CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT12_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT12_MAX                     (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT13_MASK                    (0x00002000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT13_SHIFT                   (0x0000000DU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT13_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT13_MAX                     (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT14_MASK                    (0x00004000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT14_SHIFT                   (0x0000000EU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT14_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT14_MAX                     (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT15_MASK                    (0x00008000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT15_SHIFT                   (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT15_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_BIT15_MAX                     (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR_FLAG_CLR */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT0_MASK                 (0x00000001U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT0_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT0_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT0_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT1_MASK                 (0x00000002U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT1_SHIFT                (0x00000001U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT1_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT1_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT2_MASK                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT2_SHIFT                (0x00000002U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT2_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT2_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT3_MASK                 (0x00000008U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT3_SHIFT                (0x00000003U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT3_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT3_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT4_MASK                 (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT4_SHIFT                (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT4_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT4_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT5_MASK                 (0x00000020U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT5_SHIFT                (0x00000005U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT5_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT5_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT6_MASK                 (0x00000040U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT6_SHIFT                (0x00000006U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT6_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT6_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT7_MASK                 (0x00000080U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT7_SHIFT                (0x00000007U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT7_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT7_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT8_MASK                 (0x00000100U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT8_SHIFT                (0x00000008U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT8_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT8_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT9_MASK                 (0x00000200U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT9_SHIFT                (0x00000009U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT9_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT9_MAX                  (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT10_MASK                (0x00000400U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT10_SHIFT               (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT10_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT10_MAX                 (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT11_MASK                (0x00000800U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT11_SHIFT               (0x0000000BU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT11_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT11_MAX                 (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT12_MASK                (0x00001000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT12_SHIFT               (0x0000000CU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT12_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT12_MAX                 (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT13_MASK                (0x00002000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT13_SHIFT               (0x0000000DU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT13_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT13_MAX                 (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT14_MASK                (0x00004000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT14_SHIFT               (0x0000000EU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT14_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT14_MAX                 (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT15_MASK                (0x00008000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT15_SHIFT               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT15_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_BIT15_MAX                 (0x00000001U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAG_CLR_RESETVAL                  (0x00000000U)

/* OUTPUTXBAR_FLAGFORCE */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGFORCE_FRC_MASK                 (0x0000FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGFORCE_FRC_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGFORCE_FRC_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGFORCE_FRC_MAX                  (0x0000FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_FLAGFORCE_RESETVAL                 (0x00000000U)

/* OUTPUTXBAR_OUTLATCH */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLATCH_LATCHSEL_MASK             (0x0000FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLATCH_LATCHSEL_SHIFT            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLATCH_LATCHSEL_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLATCH_LATCHSEL_MAX              (0x0000FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLATCH_RESETVAL                  (0x00000000U)

/* OUTPUTXBAR_OUTSTRETCH */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTSTRETCH_STRETCHSEL_MASK         (0x0000FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTSTRETCH_STRETCHSEL_SHIFT        (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTSTRETCH_STRETCHSEL_RESETVAL     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTSTRETCH_STRETCHSEL_MAX          (0x0000FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTSTRETCH_RESETVAL                (0x00000000U)

/* OUTPUTXBAR_OUTLENGTH */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLENGTH_LENGTHSEL_MASK           (0x0000FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLENGTH_LENGTHSEL_SHIFT          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLENGTH_LENGTHSEL_RESETVAL       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLENGTH_LENGTHSEL_MAX            (0x0000FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTLENGTH_RESETVAL                 (0x00000000U)

/* OUTPUTXBAR_OUTINVERT */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTINVERT_OUTINVERT_MASK           (0x0000FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTINVERT_OUTINVERT_SHIFT          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTINVERT_OUTINVERT_RESETVAL       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTINVERT_OUTINVERT_MAX            (0x0000FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR_OUTINVERT_RESETVAL                 (0x00000000U)

/* OUTPUTXBAR0_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G3_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G3_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G3_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G3_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G4_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G4_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G4_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G4_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G8_SEL_MASK                       (0x0003FCFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G8_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G8_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G8_SEL_MAX                        (0x0003FCFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR0_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G9_SEL_MASK                       (0x030F000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G9_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G9_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G9_SEL_MAX                        (0x030F000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR0_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G0_OUTPUTXBAR1_G0_SEL_MASK        (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G0_OUTPUTXBAR1_G0_SEL_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G0_OUTPUTXBAR1_G0_SEL_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G0_OUTPUTXBAR1_G0_SEL_MAX         (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G3_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G3_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G3_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G3_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G4_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G4_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G4_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G4_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_EQEP_MASK                  (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_EQEP_SHIFT                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_EQEP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_EQEP_MAX                   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR1_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_FSIRX_MASK                 (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_FSIRX_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_FSIRX_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_FSIRX_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_INPUTXBAR_MASK             (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_INPUTXBAR_SHIFT            (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_INPUTXBAR_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_INPUTXBAR_MAX              (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_INTXBAR_MASK               (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_INTXBAR_SHIFT              (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_INTXBAR_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_SEL_INTXBAR_MAX                (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR1_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G3_OUTPUTXBAR2_G3_SEL_MASK        (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G3_OUTPUTXBAR2_G3_SEL_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G3_OUTPUTXBAR2_G3_SEL_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G3_OUTPUTXBAR2_G3_SEL_MAX         (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G4_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G4_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G4_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G4_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_EQEP_MASK                  (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_EQEP_SHIFT                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_EQEP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_EQEP_MAX                   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR2_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_SEL_FSIRX_MASK                 (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_SEL_FSIRX_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_SEL_FSIRX_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_SEL_FSIRX_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_SEL_INPUTXBAR_MASK             (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_SEL_INPUTXBAR_SHIFT            (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_SEL_INPUTXBAR_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_SEL_INPUTXBAR_MAX              (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_OUTPUTXBAR2_G9_SEL_INTXBAR_MASK (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_OUTPUTXBAR2_G9_SEL_INTXBAR_SHIFT (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_OUTPUTXBAR2_G9_SEL_INTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_OUTPUTXBAR2_G9_SEL_INTXBAR_MAX (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR2_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G3_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G3_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G3_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G3_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G4_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G4_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G4_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G4_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_EQEP_MASK                  (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_EQEP_SHIFT                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_EQEP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_EQEP_MAX                   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR3_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_OUTPUTXBAR3_G9_SEL_FSIRX_MASK  (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_OUTPUTXBAR3_G9_SEL_FSIRX_SHIFT (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_OUTPUTXBAR3_G9_SEL_FSIRX_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_OUTPUTXBAR3_G9_SEL_FSIRX_MAX   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_SEL_INPUTXBAR_MASK             (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_SEL_INPUTXBAR_SHIFT            (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_SEL_INPUTXBAR_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_SEL_INPUTXBAR_MAX              (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_SEL_INTXBAR_MASK               (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_SEL_INTXBAR_SHIFT              (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_SEL_INTXBAR_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_SEL_INTXBAR_MAX                (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR3_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G1_OUTPUTXBAR4_G1_SEL_MASK        (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G1_OUTPUTXBAR4_G1_SEL_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G1_OUTPUTXBAR4_G1_SEL_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G1_OUTPUTXBAR4_G1_SEL_MAX         (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G3_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G3_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G3_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G3_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G4_OUTPUTXBAR4_G4_SEL_MASK        (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G4_OUTPUTXBAR4_G4_SEL_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G4_OUTPUTXBAR4_G4_SEL_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G4_OUTPUTXBAR4_G4_SEL_MAX         (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_EQEP_MASK                  (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_EQEP_SHIFT                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_EQEP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_EQEP_MAX                   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR4_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_SEL_FSIRX_MASK                 (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_SEL_FSIRX_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_SEL_FSIRX_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_SEL_FSIRX_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_OUTPUTXBAR4_G9_SEL_INPUTXBAR_MASK (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_OUTPUTXBAR4_G9_SEL_INPUTXBAR_SHIFT (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_OUTPUTXBAR4_G9_SEL_INPUTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_OUTPUTXBAR4_G9_SEL_INPUTXBAR_MAX (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_SEL_INTXBAR_MASK               (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_SEL_INTXBAR_SHIFT              (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_SEL_INTXBAR_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_SEL_INTXBAR_MAX                (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR4_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G3_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G3_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G3_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G3_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G4_OUTPUTXBAR5_G4_SEL_MASK        (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G4_OUTPUTXBAR5_G4_SEL_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G4_OUTPUTXBAR5_G4_SEL_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G4_OUTPUTXBAR5_G4_SEL_MAX         (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_EQEP_MASK                  (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_EQEP_SHIFT                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_EQEP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_EQEP_MAX                   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR5_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_SEL_FSIRX_MASK                 (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_SEL_FSIRX_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_SEL_FSIRX_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_SEL_FSIRX_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_OUTPUTXBAR5_G9_SEL_INPUTXBAR_MASK (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_OUTPUTXBAR5_G9_SEL_INPUTXBAR_SHIFT (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_OUTPUTXBAR5_G9_SEL_INPUTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_OUTPUTXBAR5_G9_SEL_INPUTXBAR_MAX (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_SEL_INTXBAR_MASK               (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_SEL_INTXBAR_SHIFT              (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_SEL_INTXBAR_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_SEL_INTXBAR_MAX                (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR5_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G3_OUTPUTXBAR6_G3_SEL_MASK        (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G3_OUTPUTXBAR6_G3_SEL_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G3_OUTPUTXBAR6_G3_SEL_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G3_OUTPUTXBAR6_G3_SEL_MAX         (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G4_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G4_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G4_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G4_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_OUTPUTXBAR6_G8_SEL_EQEP_MASK   (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_OUTPUTXBAR6_G8_SEL_EQEP_SHIFT  (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_OUTPUTXBAR6_G8_SEL_EQEP_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_OUTPUTXBAR6_G8_SEL_EQEP_MAX    (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR6_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_SEL_FSIRX_MASK                 (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_SEL_FSIRX_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_SEL_FSIRX_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_SEL_FSIRX_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_OUTPUTXBAR6_G9_SEL_INPUTXBAR_MASK (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_OUTPUTXBAR6_G9_SEL_INPUTXBAR_SHIFT (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_OUTPUTXBAR6_G9_SEL_INPUTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_OUTPUTXBAR6_G9_SEL_INPUTXBAR_MAX (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_OUTPUTXBAR6_G9_SEL_INTXBAR_MASK (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_OUTPUTXBAR6_G9_SEL_INTXBAR_SHIFT (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_OUTPUTXBAR6_G9_SEL_INTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_OUTPUTXBAR6_G9_SEL_INTXBAR_MAX (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR6_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G3_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G3_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G3_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G3_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G4_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G4_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G4_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G4_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_EQEP_MASK                  (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_EQEP_SHIFT                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_EQEP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_EQEP_MAX                   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR7_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_FSIRX_MASK                 (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_FSIRX_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_FSIRX_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_FSIRX_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_INPUTXBAR_MASK             (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_INPUTXBAR_SHIFT            (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_INPUTXBAR_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_INPUTXBAR_MAX              (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_INTXBAR_MASK               (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_INTXBAR_SHIFT              (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_INTXBAR_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_SEL_INTXBAR_MAX                (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR7_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G3_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G3_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G3_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G3_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G4_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G4_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G4_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G4_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_EQEP_MASK                  (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_EQEP_SHIFT                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_EQEP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_EQEP_MAX                   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR8_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_FSIRX_MASK                 (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_FSIRX_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_FSIRX_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_FSIRX_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_INPUTXBAR_MASK             (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_INPUTXBAR_SHIFT            (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_INPUTXBAR_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_INPUTXBAR_MAX              (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_INTXBAR_MASK               (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_INTXBAR_SHIFT              (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_INTXBAR_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_SEL_INTXBAR_MAX                (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR8_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G0_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G0_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G0_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G0_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G0_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G1_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G1_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G1_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G1_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G1_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G2_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G2_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G2_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G2_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G2_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G3_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G3_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G3_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G3_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G3_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G4_SEL_MASK                       (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G4_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G4_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G4_SEL_MAX                        (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G4_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G5_SEL_MASK                       (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G5_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G5_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G5_SEL_MAX                        (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G5_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G6_SEL_MASK                       (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G6_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G6_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G6_SEL_MAX                        (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G6_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G7_SEL_MASK                       (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G7_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G7_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G7_SEL_MAX                        (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G7_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_SYNCOUT_MASK               (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_SYNCOUT_SHIFT              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_SYNCOUT_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_SYNCOUT_MAX                (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_EQEP_MASK                  (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_EQEP_SHIFT                 (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_EQEP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_EQEP_MAX                   (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_ECAP_MASK                  (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_ECAP_SHIFT                 (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_ECAP_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_SEL_ECAP_MAX                   (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G8_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR9_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_FSIRX_MASK                 (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_FSIRX_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_FSIRX_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_FSIRX_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_INPUTXBAR_MASK             (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_INPUTXBAR_SHIFT            (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_INPUTXBAR_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_INPUTXBAR_MAX              (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_INTXBAR_MASK               (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_INTXBAR_SHIFT              (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_INTXBAR_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_SEL_INTXBAR_MAX                (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR9_G9_RESETVAL                       (0x00000000U)

/* OUTPUTXBAR10_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G0__SEL_MASK                     (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G0__SEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G0__SEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G0__SEL_MAX                      (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G0_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G1__SEL_MASK                     (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G1__SEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G1__SEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G1__SEL_MAX                      (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G1_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G2__SEL_MASK                     (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G2__SEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G2__SEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G2__SEL_MAX                      (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G2_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G3__SEL_MASK                     (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G3__SEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G3__SEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G3__SEL_MAX                      (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G3_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G4__SEL_MASK                     (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G4__SEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G4__SEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G4__SEL_MAX                      (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G4_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G5__SEL_MASK                     (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G5__SEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G5__SEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G5__SEL_MAX                      (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G5_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G6_SEL_MASK                      (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G6_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G6_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G6_SEL_MAX                       (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G6_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G7__SEL_MASK                     (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G7__SEL_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G7__SEL_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G7__SEL_MAX                      (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G7_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_SYNCOUT_MASK              (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_SYNCOUT_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_SYNCOUT_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_SYNCOUT_MAX               (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_EQEP_MASK                 (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_EQEP_SHIFT                (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_EQEP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_EQEP_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_ECAP_MASK                 (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_ECAP_SHIFT                (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_ECAP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_SEL_ECAP_MAX                  (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G8_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR10_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_FSIRX_MASK                (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_FSIRX_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_FSIRX_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_FSIRX_MAX                 (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_INPUTXBAR_MASK            (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_INPUTXBAR_SHIFT           (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_INPUTXBAR_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_INPUTXBAR_MAX             (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_INTXBAR_MASK              (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_INTXBAR_SHIFT             (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_INTXBAR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_SEL_INTXBAR_MAX               (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR10_G9_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G0_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G0_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G0_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G0_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G0_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G1_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G1_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G1_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G1_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G1_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G2_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G2_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G2_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G2_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G2_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G3_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G3_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G3_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G3_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G3_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G4_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G4_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G4_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G4_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G4_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G5_SEL_MASK                      (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G5_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G5_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G5_SEL_MAX                       (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G5_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G6_SEL_MASK                      (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G6_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G6_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G6_SEL_MAX                       (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G6_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G7_SEL_MASK                      (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G7_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G7_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G7_SEL_MAX                       (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G7_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_SEL_SYNCOUT_MASK              (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_SEL_SYNCOUT_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_SEL_SYNCOUT_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_SEL_SYNCOUT_MAX               (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_OUTPUTXBAR11_G8_SEL_EQEP_MASK (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_OUTPUTXBAR11_G8_SEL_EQEP_SHIFT (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_OUTPUTXBAR11_G8_SEL_EQEP_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_OUTPUTXBAR11_G8_SEL_EQEP_MAX  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_SEL_ECAP_MASK                 (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_SEL_ECAP_SHIFT                (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_SEL_ECAP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_SEL_ECAP_MAX                  (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G8_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR11_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_FSIRX_MASK                (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_FSIRX_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_FSIRX_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_FSIRX_MAX                 (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_INPUTXBAR_MASK            (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_INPUTXBAR_SHIFT           (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_INPUTXBAR_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_INPUTXBAR_MAX             (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_INTXBAR_MASK              (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_INTXBAR_SHIFT             (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_INTXBAR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_SEL_INTXBAR_MAX               (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR11_G9_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G0_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G0_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G0_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G0_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G0_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G1_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G1_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G1_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G1_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G1_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G2_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G2_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G2_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G2_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G2_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G3_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G3_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G3_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G3_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G3_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G4_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G4_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G4_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G4_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G4_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G5_SEL_MASK                      (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G5_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G5_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G5_SEL_MAX                       (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G5_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G6_SEL_MASK                      (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G6_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G6_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G6_SEL_MAX                       (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G6_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G7_SEL_MASK                      (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G7_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G7_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G7_SEL_MAX                       (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G7_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_SYNCOUT_MASK              (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_SYNCOUT_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_SYNCOUT_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_SYNCOUT_MAX               (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_EQEP_MASK                 (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_EQEP_SHIFT                (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_EQEP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_EQEP_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_ECAP_MASK                 (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_ECAP_SHIFT                (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_ECAP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_SEL_ECAP_MAX                  (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G8_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR12_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_FSIRX_MASK                (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_FSIRX_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_FSIRX_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_FSIRX_MAX                 (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_INPUTXBAR_MASK            (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_INPUTXBAR_SHIFT           (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_INPUTXBAR_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_INPUTXBAR_MAX             (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_INTXBAR_MASK              (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_INTXBAR_SHIFT             (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_INTXBAR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_SEL_INTXBAR_MAX               (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR12_G9_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G0_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G0_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G0_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G0_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G0_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G1_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G1_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G1_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G1_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G1_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G2_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G2_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G2_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G2_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G2_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G3_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G3_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G3_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G3_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G3_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G4_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G4_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G4_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G4_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G4_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G5_SEL_MASK                      (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G5_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G5_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G5_SEL_MAX                       (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G5_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G6_SEL_MASK                      (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G6_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G6_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G6_SEL_MAX                       (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G6_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G7_SEL_MASK                      (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G7_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G7_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G7_SEL_MAX                       (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G7_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_SYNCOUT_MASK              (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_SYNCOUT_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_SYNCOUT_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_SYNCOUT_MAX               (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_EQEP_MASK                 (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_EQEP_SHIFT                (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_EQEP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_EQEP_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_ECAP_MASK                 (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_ECAP_SHIFT                (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_ECAP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_SEL_ECAP_MAX                  (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G8_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR13_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_FSIRX_MASK                (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_FSIRX_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_FSIRX_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_FSIRX_MAX                 (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_INPUTXBAR_MASK            (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_INPUTXBAR_SHIFT           (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_INPUTXBAR_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_INPUTXBAR_MAX             (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_INTXBAR_MASK              (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_INTXBAR_SHIFT             (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_INTXBAR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_SEL_INTXBAR_MAX               (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR13_G9_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G0_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G0_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G0_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G0_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G0_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G1_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G1_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G1_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G1_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G1_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G2_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G2_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G2_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G2_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G2_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G3_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G3_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G3_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G3_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G3_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G4_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G4_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G4_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G4_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G4_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G5_SEL_MASK                      (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G5_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G5_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G5_SEL_MAX                       (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G5_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G6_SEL_MASK                      (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G6_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G6_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G6_SEL_MAX                       (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G6_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G7_SEL_MASK                      (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G7_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G7_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G7_SEL_MAX                       (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G7_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_SYNCOUT_MASK              (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_SYNCOUT_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_SYNCOUT_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_SYNCOUT_MAX               (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_EQEP_MASK                 (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_EQEP_SHIFT                (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_EQEP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_EQEP_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_ECAP_MASK                 (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_ECAP_SHIFT                (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_ECAP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_SEL_ECAP_MAX                  (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G8_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR14_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_FSIRX_MASK                (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_FSIRX_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_FSIRX_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_FSIRX_MAX                 (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_INPUTXBAR_MASK            (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_INPUTXBAR_SHIFT           (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_INPUTXBAR_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_INPUTXBAR_MAX             (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_INTXBAR_MASK              (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_INTXBAR_SHIFT             (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_INTXBAR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_SEL_INTXBAR_MAX               (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR14_G9_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G0 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G0_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G0_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G0_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G0_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G0_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G1 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G1_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G1_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G1_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G1_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G1_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G2 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G2_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G2_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G2_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G2_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G2_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G3 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G3_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G3_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G3_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G3_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G3_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G4 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G4_SEL_MASK                      (0x000003FFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G4_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G4_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G4_SEL_MAX                       (0x000003FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G4_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G5 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G5_SEL_MASK                      (0x00FFFFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G5_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G5_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G5_SEL_MAX                       (0x00FFFFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G5_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G6 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G6_SEL_MASK                      (0x0003FFFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G6_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G6_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G6_SEL_MAX                       (0x0003FFFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G6_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G7 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G7_SEL_MASK                      (0x00000FFFU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G7_SEL_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G7_SEL_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G7_SEL_MAX                       (0x00000FFFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G7_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G8 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_SYNCOUT_MASK              (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_SYNCOUT_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_SYNCOUT_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_SYNCOUT_MAX               (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_EQEP_MASK                 (0x000000F0U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_EQEP_SHIFT                (0x00000004U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_EQEP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_EQEP_MAX                  (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_ECAP_MASK                 (0x0003FC00U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_ECAP_SHIFT                (0x0000000AU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_ECAP_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_SEL_ECAP_MAX                  (0x000000FFU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G8_RESETVAL                      (0x00000000U)

/* OUTPUTXBAR15_G9 */

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_FSIRX_MASK                (0x0000000FU)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_FSIRX_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_FSIRX_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_FSIRX_MAX                 (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_INPUTXBAR_MASK            (0x000F0000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_INPUTXBAR_SHIFT           (0x00000010U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_INPUTXBAR_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_INPUTXBAR_MAX             (0x0000000FU)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_INTXBAR_MASK              (0x03000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_INTXBAR_SHIFT             (0x00000018U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_INTXBAR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_SEL_INTXBAR_MAX               (0x00000003U)

#define CSL_CONTROLSS_OUTPUTXBAR_OUTPUTXBAR15_G9_RESETVAL                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
