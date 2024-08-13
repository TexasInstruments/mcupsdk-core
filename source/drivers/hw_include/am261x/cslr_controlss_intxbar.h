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
 *  Name        : cslr_controlss_intxbar.h
*/
#ifndef CSLR_CONTROLSS_INTXBAR_H_
#define CSLR_CONTROLSS_INTXBAR_H_

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

/******************* GO *********************/
#define INT_XBAR_EPWM0_INT    (0x00000001)
#define INT_XBAR_EPWM1_INT    (0x00000002)
#define INT_XBAR_EPWM2_INT    (0x00000004)
#define INT_XBAR_EPWM3_INT    (0x00000008)
#define INT_XBAR_EPWM4_INT    (0x00000010)
#define INT_XBAR_EPWM5_INT    (0x00000020)
#define INT_XBAR_EPWM6_INT    (0x00000040)
#define INT_XBAR_EPWM7_INT    (0x00000080)
#define INT_XBAR_EPWM8_INT    (0x00000100)
#define INT_XBAR_EPWM9_INT    (0x00000200)

/******************* G1 *********************/
#define INT_XBAR_EPWM0_TZINT    (0x00000001)
#define INT_XBAR_EPWM1_TZINT    (0x00000002)
#define INT_XBAR_EPWM2_TZINT    (0x00000004)
#define INT_XBAR_EPWM3_TZINT    (0x00000008)
#define INT_XBAR_EPWM4_TZINT    (0x00000010)
#define INT_XBAR_EPWM5_TZINT    (0x00000020)
#define INT_XBAR_EPWM6_TZINT    (0x00000040)
#define INT_XBAR_EPWM7_TZINT    (0x00000080)
#define INT_XBAR_EPWM8_TZINT    (0x00000100)
#define INT_XBAR_EPWM9_TZINT    (0x00000200)

/******************* G2 *********************/
#define INT_XBAR_ADC0_INT1       (0x00000001)
#define INT_XBAR_ADC0_INT2       (0x00000002)
#define INT_XBAR_ADC0_INT3       (0x00000004)
#define INT_XBAR_ADC0_INT4       (0x00000008)
#define INT_XBAR_ADC0_EVTINT     (0x00000010)
#define INT_XBAR_ADC1_INT1       (0x00000020)
#define INT_XBAR_ADC1_INT2       (0x00000040)
#define INT_XBAR_ADC1_INT3       (0x00000080)
#define INT_XBAR_ADC1_INT4       (0x00000100)
#define INT_XBAR_ADC1_EVTINT     (0x00000200)
#define INT_XBAR_ADC2_INT1       (0x00000400)
#define INT_XBAR_ADC2_INT2       (0x00000800)
#define INT_XBAR_ADC2_INT3       (0x00001000)
#define INT_XBAR_ADC2_INT4       (0x00002000)
#define INT_XBAR_ADC2_EVTINT     (0x00004000)
#define INT_XBAR_EVTAGGR0        (0x80000000)

/******************* G3 *********************/
#define INT_XBAR_FSIRX0_INT1N    (0x00000001)
#define INT_XBAR_FSIRX0_INT2N    (0x00000002)
#define INT_XBAR_FSITX0_INT1N    (0x00000100)
#define INT_XBAR_FSITX0_INT2N    (0x00000200)

/******************* G4 *********************/
#define INT_XBAR_SD0_ERR         (0x00000001)
#define INT_XBAR_SD0_FILT0_DRINT (0x00000002)
#define INT_XBAR_SD0_FILT1_DRINT (0x00000004)
#define INT_XBAR_SD0_FILT2_DRINT (0x00000008)
#define INT_XBAR_SD0_FILT3_DRINT (0x00000010)
#define INT_XBAR_SD1_ERR         (0x00000020)
#define INT_XBAR_SD1_FILT0_DRINT (0x00000040)
#define INT_XBAR_SD1_FILT1_DRINT (0x00000080)
#define INT_XBAR_SD1_FILT2_DRINT (0x00000100)
#define INT_XBAR_SD1_FILT3_DRINT (0x00000200)

/******************* G5 *********************/
#define INT_XBAR_ECAP0_INT    (0x00000001)
#define INT_XBAR_ECAP1_INT    (0x00000002)
#define INT_XBAR_ECAP2_INT    (0x00000004)
#define INT_XBAR_ECAP3_INT    (0x00000008)
#define INT_XBAR_ECAP4_INT    (0x00000010)
#define INT_XBAR_ECAP5_INT    (0x00000020)
#define INT_XBAR_ECAP6_INT    (0x00000040)
#define INT_XBAR_ECAP7_INT    (0x00000080)

/******************* G6 *********************/
#define INT_XBAR_EQEP0_INT    (0x00000001)
#define INT_XBAR_EQEP1_INT    (0x00000002)
       
/******************* G7 *********************/
#define INT_XBAR_CMPSSA0_CTRIPL      (0x00000001)
#define INT_XBAR_CMPSSA0_CTRIPH      (0x00000002)
#define INT_XBAR_CMPSSA1_CTRIPL      (0x00000004)
#define INT_XBAR_CMPSSA1_CTRIPH      (0x00000008)
#define INT_XBAR_CMPSSA2_CTRIPL      (0x00000010)
#define INT_XBAR_CMPSSA2_CTRIPH      (0x00000020)
#define INT_XBAR_CMPSSA3_CTRIPL      (0x00000040)
#define INT_XBAR_CMPSSA3_CTRIPH      (0x00000080)
#define INT_XBAR_CMPSSA4_CTRIPL      (0x00000100)
#define INT_XBAR_CMPSSA4_CTRIPH      (0x00000200)
#define INT_XBAR_CMPSSA5_CTRIPL      (0x00000400)
#define INT_XBAR_CMPSSA5_CTRIPH      (0x00000800)
#define INT_XBAR_CMPSSA6_CTRIPL      (0x00001000)
#define INT_XBAR_CMPSSA6_CTRIPH      (0x00002000)
#define INT_XBAR_CMPSSA7_CTRIPL      (0x00004000)
#define INT_XBAR_CMPSSA7_CTRIPH      (0x00008000)
#define INT_XBAR_CMPSSA8_CTRIPL      (0x00010000)
#define INT_XBAR_CMPSSA8_CTRIPH      (0x00020000)
#define INT_XBAR_CMPSSA9_CTRIPL      (0x00040000)
#define INT_XBAR_CMPSSA9_CTRIPH      (0x00080000)

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint8_t  Resv_256[252];
    volatile uint32_t INTXBAR0_G0;
    volatile uint32_t INTXBAR0_G1;
    volatile uint32_t INTXBAR0_G2;
    volatile uint32_t INTXBAR0_G3;
    volatile uint32_t INTXBAR0_G4;
    volatile uint32_t INTXBAR0_G5;
    volatile uint32_t INTXBAR0_G6;
    volatile uint32_t INTXBAR0_G7;
    volatile uint8_t  Resv_320[32];
    volatile uint32_t INTXBAR1_G0;
    volatile uint32_t INTXBAR1_G1;
    volatile uint32_t INTXBAR1_G2;
    volatile uint32_t INTXBAR1_G3;
    volatile uint32_t INTXBAR1_G4;
    volatile uint32_t INTXBAR1_G5;
    volatile uint32_t INTXBAR1_G6;
    volatile uint32_t INTXBAR1_G7;
    volatile uint8_t  Resv_384[32];
    volatile uint32_t INTXBAR2_G0;
    volatile uint32_t INTXBAR2_G1;
    volatile uint32_t INTXBAR2_G2;
    volatile uint32_t INTXBAR2_G3;
    volatile uint32_t INTXBAR2_G4;
    volatile uint32_t INTXBAR2_G5;
    volatile uint32_t INTXBAR2_G6;
    volatile uint32_t INTXBAR2_G7;
    volatile uint8_t  Resv_448[32];
    volatile uint32_t INTXBAR3_G0;
    volatile uint32_t INTXBAR3_G1;
    volatile uint32_t INTXBAR3_G2;
    volatile uint32_t INTXBAR3_G3;
    volatile uint32_t INTXBAR3_G4;
    volatile uint32_t INTXBAR3_G5;
    volatile uint32_t INTXBAR3_G6;
    volatile uint32_t INTXBAR3_G7;
    volatile uint8_t  Resv_512[32];
    volatile uint32_t INTXBAR4_G0;
    volatile uint32_t INTXBAR4_G1;
    volatile uint32_t INTXBAR4_G2;
    volatile uint32_t INTXBAR4_G3;
    volatile uint32_t INTXBAR4_G4;
    volatile uint32_t INTXBAR4_G5;
    volatile uint32_t INTXBAR4_G6;
    volatile uint32_t INTXBAR4_G7;
    volatile uint8_t  Resv_576[32];
    volatile uint32_t INTXBAR5_G0;
    volatile uint32_t INTXBAR5_G1;
    volatile uint32_t INTXBAR5_G2;
    volatile uint32_t INTXBAR5_G3;
    volatile uint32_t INTXBAR5_G4;
    volatile uint32_t INTXBAR5_G5;
    volatile uint32_t INTXBAR5_G6;
    volatile uint32_t INTXBAR5_G7;
    volatile uint8_t  Resv_640[32];
    volatile uint32_t INTXBAR6_G0;
    volatile uint32_t INTXBAR6_G1;
    volatile uint32_t INTXBAR6_G2;
    volatile uint32_t INTXBAR6_G3;
    volatile uint32_t INTXBAR6_G4;
    volatile uint32_t INTXBAR6_G5;
    volatile uint32_t INTXBAR6_G6;
    volatile uint32_t INTXBAR6_G7;
    volatile uint8_t  Resv_704[32];
    volatile uint32_t INTXBAR7_G0;
    volatile uint32_t INTXBAR7_G1;
    volatile uint32_t INTXBAR7_G2;
    volatile uint32_t INTXBAR7_G3;
    volatile uint32_t INTXBAR7_G4;
    volatile uint32_t INTXBAR7_G5;
    volatile uint32_t INTXBAR7_G6;
    volatile uint32_t INTXBAR7_G7;
    volatile uint8_t  Resv_768[32];
    volatile uint32_t INTXBAR8_G0;
    volatile uint32_t INTXBAR8_G1;
    volatile uint32_t INTXBAR8_G2;
    volatile uint32_t INTXBAR8_G3;
    volatile uint32_t INTXBAR8_G4;
    volatile uint32_t INTXBAR8_G5;
    volatile uint32_t INTXBAR8_G6;
    volatile uint32_t INTXBAR8_G7;
    volatile uint8_t  Resv_832[32];
    volatile uint32_t INTXBAR9_G0;
    volatile uint32_t INTXBAR9_G1;
    volatile uint32_t INTXBAR9_G2;
    volatile uint32_t INTXBAR9_G3;
    volatile uint32_t INTXBAR9_G4;
    volatile uint32_t INTXBAR9_G5;
    volatile uint32_t INTXBAR9_G6;
    volatile uint32_t INTXBAR9_G7;
    volatile uint8_t  Resv_896[32];
    volatile uint32_t INTXBAR10_G0;
    volatile uint32_t INTXBAR10_G1;
    volatile uint32_t INTXBAR10_G2;
    volatile uint32_t INTXBAR10_G3;
    volatile uint32_t INTXBAR10_G4;
    volatile uint32_t INTXBAR10_G5;
    volatile uint32_t INTXBAR10_G6;
    volatile uint32_t INTXBAR10_G7;
    volatile uint8_t  Resv_960[32];
    volatile uint32_t INTXBAR11_G0;
    volatile uint32_t INTXBAR11_G1;
    volatile uint32_t INTXBAR11_G2;
    volatile uint32_t INTXBAR11_G3;
    volatile uint32_t INTXBAR11_G4;
    volatile uint32_t INTXBAR11_G5;
    volatile uint32_t INTXBAR11_G6;
    volatile uint32_t INTXBAR11_G7;
    volatile uint8_t  Resv_1024[32];
    volatile uint32_t INTXBAR12_G0;
    volatile uint32_t INTXBAR12_G1;
    volatile uint32_t INTXBAR12_G2;
    volatile uint32_t INTXBAR12_G3;
    volatile uint32_t INTXBAR12_G4;
    volatile uint32_t INTXBAR12_G5;
    volatile uint32_t INTXBAR12_G6;
    volatile uint32_t INTXBAR12_G7;
    volatile uint8_t  Resv_1088[32];
    volatile uint32_t INTXBAR13_G0;
    volatile uint32_t INTXBAR13_G1;
    volatile uint32_t INTXBAR13_G2;
    volatile uint32_t INTXBAR13_G3;
    volatile uint32_t INTXBAR13_G4;
    volatile uint32_t INTXBAR13_G5;
    volatile uint32_t INTXBAR13_G6;
    volatile uint32_t INTXBAR13_G7;
    volatile uint8_t  Resv_1152[32];
    volatile uint32_t INTXBAR14_G0;
    volatile uint32_t INTXBAR14_G1;
    volatile uint32_t INTXBAR14_G2;
    volatile uint32_t INTXBAR14_G3;
    volatile uint32_t INTXBAR14_G4;
    volatile uint32_t INTXBAR14_G5;
    volatile uint32_t INTXBAR14_G6;
    volatile uint32_t INTXBAR14_G7;
    volatile uint8_t  Resv_1216[32];
    volatile uint32_t INTXBAR15_G0;
    volatile uint32_t INTXBAR15_G1;
    volatile uint32_t INTXBAR15_G2;
    volatile uint32_t INTXBAR15_G3;
    volatile uint32_t INTXBAR15_G4;
    volatile uint32_t INTXBAR15_G5;
    volatile uint32_t INTXBAR15_G6;
    volatile uint32_t INTXBAR15_G7;
    volatile uint8_t  Resv_1280[32];
    volatile uint32_t INTXBAR16_G0;
    volatile uint32_t INTXBAR16_G1;
    volatile uint32_t INTXBAR16_G2;
    volatile uint32_t INTXBAR16_G3;
    volatile uint32_t INTXBAR16_G4;
    volatile uint32_t INTXBAR16_G5;
    volatile uint32_t INTXBAR16_G6;
    volatile uint32_t INTXBAR16_G7;
    volatile uint8_t  Resv_1344[32];
    volatile uint32_t INTXBAR17_G0;
    volatile uint32_t INTXBAR17_G1;
    volatile uint32_t INTXBAR17_G2;
    volatile uint32_t INTXBAR17_G3;
    volatile uint32_t INTXBAR17_G4;
    volatile uint32_t INTXBAR17_G5;
    volatile uint32_t INTXBAR17_G6;
    volatile uint32_t INTXBAR17_G7;
    volatile uint8_t  Resv_1408[32];
    volatile uint32_t INTXBAR18_G0;
    volatile uint32_t INTXBAR18_G1;
    volatile uint32_t INTXBAR18_G2;
    volatile uint32_t INTXBAR18_G3;
    volatile uint32_t INTXBAR18_G4;
    volatile uint32_t INTXBAR18_G5;
    volatile uint32_t INTXBAR18_G6;
    volatile uint32_t INTXBAR18_G7;
    volatile uint8_t  Resv_1472[32];
    volatile uint32_t INTXBAR19_G0;
    volatile uint32_t INTXBAR19_G1;
    volatile uint32_t INTXBAR19_G2;
    volatile uint32_t INTXBAR19_G3;
    volatile uint32_t INTXBAR19_G4;
    volatile uint32_t INTXBAR19_G5;
    volatile uint32_t INTXBAR19_G6;
    volatile uint32_t INTXBAR19_G7;
    volatile uint8_t  Resv_1536[32];
    volatile uint32_t INTXBAR20_G0;
    volatile uint32_t INTXBAR20_G1;
    volatile uint32_t INTXBAR20_G2;
    volatile uint32_t INTXBAR20_G3;
    volatile uint32_t INTXBAR20_G4;
    volatile uint32_t INTXBAR20_G5;
    volatile uint32_t INTXBAR20_G6;
    volatile uint32_t INTXBAR20_G7;
    volatile uint8_t  Resv_1600[32];
    volatile uint32_t INTXBAR21_G0;
    volatile uint32_t INTXBAR21_G1;
    volatile uint32_t INTXBAR21_G2;
    volatile uint32_t INTXBAR21_G3;
    volatile uint32_t INTXBAR21_G4;
    volatile uint32_t INTXBAR21_G5;
    volatile uint32_t INTXBAR21_G6;
    volatile uint32_t INTXBAR21_G7;
    volatile uint8_t  Resv_1664[32];
    volatile uint32_t INTXBAR22_G0;
    volatile uint32_t INTXBAR22_G1;
    volatile uint32_t INTXBAR22_G2;
    volatile uint32_t INTXBAR22_G3;
    volatile uint32_t INTXBAR22_G4;
    volatile uint32_t INTXBAR22_G5;
    volatile uint32_t INTXBAR22_G6;
    volatile uint32_t INTXBAR22_G7;
    volatile uint8_t  Resv_1728[32];
    volatile uint32_t INTXBAR23_G0;
    volatile uint32_t INTXBAR23_G1;
    volatile uint32_t INTXBAR23_G2;
    volatile uint32_t INTXBAR23_G3;
    volatile uint32_t INTXBAR23_G4;
    volatile uint32_t INTXBAR23_G5;
    volatile uint32_t INTXBAR23_G6;
    volatile uint32_t INTXBAR23_G7;
    volatile uint8_t  Resv_1792[32];
    volatile uint32_t INTXBAR24_G0;
    volatile uint32_t INTXBAR24_G1;
    volatile uint32_t INTXBAR24_G2;
    volatile uint32_t INTXBAR24_G3;
    volatile uint32_t INTXBAR24_G4;
    volatile uint32_t INTXBAR24_G5;
    volatile uint32_t INTXBAR24_G6;
    volatile uint32_t INTXBAR24_G7;
    volatile uint8_t  Resv_1856[32];
    volatile uint32_t INTXBAR25_G0;
    volatile uint32_t INTXBAR25_G1;
    volatile uint32_t INTXBAR25_G2;
    volatile uint32_t INTXBAR25_G3;
    volatile uint32_t INTXBAR25_G4;
    volatile uint32_t INTXBAR25_G5;
    volatile uint32_t INTXBAR25_G6;
    volatile uint32_t INTXBAR25_G7;
    volatile uint8_t  Resv_1920[32];
    volatile uint32_t INTXBAR26_G0;
    volatile uint32_t INTXBAR26_G1;
    volatile uint32_t INTXBAR26_G2;
    volatile uint32_t INTXBAR26_G3;
    volatile uint32_t INTXBAR26_G4;
    volatile uint32_t INTXBAR26_G5;
    volatile uint32_t INTXBAR26_G6;
    volatile uint32_t INTXBAR26_G7;
    volatile uint8_t  Resv_1984[32];
    volatile uint32_t INTXBAR27_G0;
    volatile uint32_t INTXBAR27_G1;
    volatile uint32_t INTXBAR27_G2;
    volatile uint32_t INTXBAR27_G3;
    volatile uint32_t INTXBAR27_G4;
    volatile uint32_t INTXBAR27_G5;
    volatile uint32_t INTXBAR27_G6;
    volatile uint32_t INTXBAR27_G7;
    volatile uint8_t  Resv_2048[32];
    volatile uint32_t INTXBAR28_G0;
    volatile uint32_t INTXBAR28_G1;
    volatile uint32_t INTXBAR28_G2;
    volatile uint32_t INTXBAR28_G3;
    volatile uint32_t INTXBAR28_G4;
    volatile uint32_t INTXBAR28_G5;
    volatile uint32_t INTXBAR28_G6;
    volatile uint32_t INTXBAR28_G7;
    volatile uint8_t  Resv_2112[32];
    volatile uint32_t INTXBAR29_G0;
    volatile uint32_t INTXBAR29_G1;
    volatile uint32_t INTXBAR29_G2;
    volatile uint32_t INTXBAR29_G3;
    volatile uint32_t INTXBAR29_G4;
    volatile uint32_t INTXBAR29_G5;
    volatile uint32_t INTXBAR29_G6;
    volatile uint32_t INTXBAR29_G7;
    volatile uint8_t  Resv_2176[32];
    volatile uint32_t INTXBAR30_G0;
    volatile uint32_t INTXBAR30_G1;
    volatile uint32_t INTXBAR30_G2;
    volatile uint32_t INTXBAR30_G3;
    volatile uint32_t INTXBAR30_G4;
    volatile uint32_t INTXBAR30_G5;
    volatile uint32_t INTXBAR30_G6;
    volatile uint32_t INTXBAR30_G7;
    volatile uint8_t  Resv_2240[32];
    volatile uint32_t INTXBAR31_G0;
    volatile uint32_t INTXBAR31_G1;
    volatile uint32_t INTXBAR31_G2;
    volatile uint32_t INTXBAR31_G3;
    volatile uint32_t INTXBAR31_G4;
    volatile uint32_t INTXBAR31_G5;
    volatile uint32_t INTXBAR31_G6;
    volatile uint32_t INTXBAR31_G7;
} CSL_controlss_intxbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CONTROLSS_INTXBAR_PID                                              (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0                                      (0x00000100U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1                                      (0x00000104U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2                                      (0x00000108U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3                                      (0x0000010CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4                                      (0x00000110U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5                                      (0x00000114U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6                                      (0x00000118U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7                                      (0x0000011CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0                                      (0x00000140U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1                                      (0x00000144U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2                                      (0x00000148U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3                                      (0x0000014CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4                                      (0x00000150U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5                                      (0x00000154U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6                                      (0x00000158U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7                                      (0x0000015CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0                                      (0x00000180U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1                                      (0x00000184U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2                                      (0x00000188U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3                                      (0x0000018CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4                                      (0x00000190U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5                                      (0x00000194U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6                                      (0x00000198U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7                                      (0x0000019CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0                                      (0x000001C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1                                      (0x000001C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2                                      (0x000001C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3                                      (0x000001CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4                                      (0x000001D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5                                      (0x000001D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6                                      (0x000001D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7                                      (0x000001DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0                                      (0x00000200U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1                                      (0x00000204U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2                                      (0x00000208U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3                                      (0x0000020CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4                                      (0x00000210U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5                                      (0x00000214U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6                                      (0x00000218U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7                                      (0x0000021CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0                                      (0x00000240U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1                                      (0x00000244U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2                                      (0x00000248U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3                                      (0x0000024CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4                                      (0x00000250U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5                                      (0x00000254U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6                                      (0x00000258U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7                                      (0x0000025CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0                                      (0x00000280U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1                                      (0x00000284U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2                                      (0x00000288U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3                                      (0x0000028CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4                                      (0x00000290U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5                                      (0x00000294U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6                                      (0x00000298U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7                                      (0x0000029CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0                                      (0x000002C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1                                      (0x000002C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2                                      (0x000002C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3                                      (0x000002CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4                                      (0x000002D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5                                      (0x000002D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6                                      (0x000002D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7                                      (0x000002DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0                                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1                                      (0x00000304U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2                                      (0x00000308U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3                                      (0x0000030CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4                                      (0x00000310U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5                                      (0x00000314U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6                                      (0x00000318U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7                                      (0x0000031CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0                                      (0x00000340U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1                                      (0x00000344U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2                                      (0x00000348U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3                                      (0x0000034CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4                                      (0x00000350U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5                                      (0x00000354U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6                                      (0x00000358U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7                                      (0x0000035CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0                                     (0x00000380U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1                                     (0x00000384U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2                                     (0x00000388U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3                                     (0x0000038CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4                                     (0x00000390U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5                                     (0x00000394U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6                                     (0x00000398U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7                                     (0x0000039CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0                                     (0x000003C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1                                     (0x000003C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2                                     (0x000003C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3                                     (0x000003CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4                                     (0x000003D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5                                     (0x000003D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6                                     (0x000003D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7                                     (0x000003DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0                                     (0x00000400U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1                                     (0x00000404U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2                                     (0x00000408U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3                                     (0x0000040CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4                                     (0x00000410U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5                                     (0x00000414U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6                                     (0x00000418U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7                                     (0x0000041CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0                                     (0x00000440U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1                                     (0x00000444U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2                                     (0x00000448U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3                                     (0x0000044CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4                                     (0x00000450U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5                                     (0x00000454U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6                                     (0x00000458U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7                                     (0x0000045CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0                                     (0x00000480U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1                                     (0x00000484U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2                                     (0x00000488U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3                                     (0x0000048CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4                                     (0x00000490U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5                                     (0x00000494U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6                                     (0x00000498U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7                                     (0x0000049CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0                                     (0x000004C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1                                     (0x000004C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2                                     (0x000004C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3                                     (0x000004CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4                                     (0x000004D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5                                     (0x000004D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6                                     (0x000004D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7                                     (0x000004DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0                                     (0x00000500U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1                                     (0x00000504U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2                                     (0x00000508U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3                                     (0x0000050CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4                                     (0x00000510U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5                                     (0x00000514U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6                                     (0x00000518U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7                                     (0x0000051CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0                                     (0x00000540U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1                                     (0x00000544U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2                                     (0x00000548U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3                                     (0x0000054CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4                                     (0x00000550U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5                                     (0x00000554U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6                                     (0x00000558U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7                                     (0x0000055CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0                                     (0x00000580U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1                                     (0x00000584U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2                                     (0x00000588U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3                                     (0x0000058CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4                                     (0x00000590U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5                                     (0x00000594U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6                                     (0x00000598U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7                                     (0x0000059CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0                                     (0x000005C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1                                     (0x000005C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2                                     (0x000005C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3                                     (0x000005CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4                                     (0x000005D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5                                     (0x000005D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6                                     (0x000005D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7                                     (0x000005DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0                                     (0x00000600U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1                                     (0x00000604U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2                                     (0x00000608U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3                                     (0x0000060CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4                                     (0x00000610U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5                                     (0x00000614U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6                                     (0x00000618U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7                                     (0x0000061CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0                                     (0x00000640U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1                                     (0x00000644U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2                                     (0x00000648U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3                                     (0x0000064CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4                                     (0x00000650U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5                                     (0x00000654U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6                                     (0x00000658U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7                                     (0x0000065CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0                                     (0x00000680U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1                                     (0x00000684U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2                                     (0x00000688U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3                                     (0x0000068CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4                                     (0x00000690U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5                                     (0x00000694U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6                                     (0x00000698U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7                                     (0x0000069CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0                                     (0x000006C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1                                     (0x000006C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2                                     (0x000006C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3                                     (0x000006CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4                                     (0x000006D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5                                     (0x000006D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6                                     (0x000006D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7                                     (0x000006DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0                                     (0x00000700U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1                                     (0x00000704U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2                                     (0x00000708U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3                                     (0x0000070CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4                                     (0x00000710U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5                                     (0x00000714U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6                                     (0x00000718U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7                                     (0x0000071CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0                                     (0x00000740U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1                                     (0x00000744U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2                                     (0x00000748U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3                                     (0x0000074CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4                                     (0x00000750U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5                                     (0x00000754U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6                                     (0x00000758U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7                                     (0x0000075CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0                                     (0x00000780U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1                                     (0x00000784U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2                                     (0x00000788U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3                                     (0x0000078CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4                                     (0x00000790U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5                                     (0x00000794U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6                                     (0x00000798U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7                                     (0x0000079CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0                                     (0x000007C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1                                     (0x000007C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2                                     (0x000007C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3                                     (0x000007CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4                                     (0x000007D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5                                     (0x000007D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6                                     (0x000007D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7                                     (0x000007DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0                                     (0x00000800U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1                                     (0x00000804U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2                                     (0x00000808U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3                                     (0x0000080CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4                                     (0x00000810U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5                                     (0x00000814U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6                                     (0x00000818U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7                                     (0x0000081CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0                                     (0x00000840U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1                                     (0x00000844U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2                                     (0x00000848U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3                                     (0x0000084CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4                                     (0x00000850U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5                                     (0x00000854U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6                                     (0x00000858U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7                                     (0x0000085CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0                                     (0x00000880U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1                                     (0x00000884U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2                                     (0x00000888U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3                                     (0x0000088CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4                                     (0x00000890U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5                                     (0x00000894U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6                                     (0x00000898U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7                                     (0x0000089CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0                                     (0x000008C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1                                     (0x000008C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2                                     (0x000008C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3                                     (0x000008CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4                                     (0x000008D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5                                     (0x000008D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6                                     (0x000008D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7                                     (0x000008DCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_CONTROLSS_INTXBAR_PID_PID_MINOR_MASK                               (0x0000003FU)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MINOR_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MINOR_RESETVAL                           (0x00000014U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MINOR_MAX                                (0x0000003FU)

#define CSL_CONTROLSS_INTXBAR_PID_PID_CUSTOM_MASK                              (0x000000C0U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_CUSTOM_SHIFT                             (0x00000006U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_CUSTOM_RESETVAL                          (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_CUSTOM_MAX                               (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_PID_PID_MAJOR_MASK                               (0x00000700U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MAJOR_SHIFT                              (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MAJOR_RESETVAL                           (0x00000002U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MAJOR_MAX                                (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_PID_PID_MISC_MASK                                (0x0000F800U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MISC_SHIFT                               (0x0000000BU)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MISC_RESETVAL                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MISC_MAX                                 (0x0000001FU)

#define CSL_CONTROLSS_INTXBAR_PID_PID_MSB16_MASK                               (0xFFFF0000U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MSB16_SHIFT                              (0x00000010U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MSB16_RESETVAL                           (0x00006180U)
#define CSL_CONTROLSS_INTXBAR_PID_PID_MSB16_MAX                                (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_PID_RESETVAL                                     (0x61800214U)

/* INTXBAR0_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_RESETVAL                             (0x00000000U)

/* INTXBAR0_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_RESETVAL                             (0x00000000U)

/* INTXBAR0_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_MASK                             (0x80007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_SHIFT                            (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_MAX                              (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_RESETVAL                             (0x00000000U)

/* INTXBAR0_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_MASK                             (0x00000303U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_SHIFT                            (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_RESETVAL                             (0x00000000U)

/* INTXBAR0_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_RESETVAL                             (0x00000000U)

/* INTXBAR0_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_RESETVAL                             (0x00000000U)

/* INTXBAR0_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_RESETVAL                             (0x00000000U)

/* INTXBAR0_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_RESETVAL                             (0x00000000U)

/* INTXBAR1_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_RESETVAL                             (0x00000000U)

/* INTXBAR1_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_RESETVAL                             (0x00000000U)

/* INTXBAR1_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_RESETVAL                             (0x00000000U)

/* INTXBAR1_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_RESETVAL                             (0x00000000U)

/* INTXBAR1_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_RESETVAL                             (0x00000000U)

/* INTXBAR1_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_RESETVAL                             (0x00000000U)

/* INTXBAR1_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_RESETVAL                             (0x00000000U)

/* INTXBAR1_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_RESETVAL                             (0x00000000U)

/* INTXBAR2_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_RESETVAL                             (0x00000000U)

/* INTXBAR2_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_RESETVAL                             (0x00000000U)

/* INTXBAR2_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_RESETVAL                             (0x00000000U)

/* INTXBAR2_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_RESETVAL                             (0x00000000U)

/* INTXBAR2_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_RESETVAL                             (0x00000000U)

/* INTXBAR2_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_RESETVAL                             (0x00000000U)

/* INTXBAR2_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_RESETVAL                             (0x00000000U)

/* INTXBAR2_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_RESETVAL                             (0x00000000U)

/* INTXBAR3_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_RESETVAL                             (0x00000000U)

/* INTXBAR3_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_RESETVAL                             (0x00000000U)

/* INTXBAR3_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_RESETVAL                             (0x00000000U)

/* INTXBAR3_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_RESETVAL                             (0x00000000U)

/* INTXBAR3_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_RESETVAL                             (0x00000000U)

/* INTXBAR3_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_RESETVAL                             (0x00000000U)

/* INTXBAR3_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_RESETVAL                             (0x00000000U)

/* INTXBAR3_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_RESETVAL                             (0x00000000U)

/* INTXBAR4_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_RESETVAL                             (0x00000000U)

/* INTXBAR4_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_RESETVAL                             (0x00000000U)

/* INTXBAR4_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_RESETVAL                             (0x00000000U)

/* INTXBAR4_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_RESETVAL                             (0x00000000U)

/* INTXBAR4_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_RESETVAL                             (0x00000000U)

/* INTXBAR4_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_RESETVAL                             (0x00000000U)

/* INTXBAR4_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_RESETVAL                             (0x00000000U)

/* INTXBAR4_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_RESETVAL                             (0x00000000U)

/* INTXBAR5_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_RESETVAL                             (0x00000000U)

/* INTXBAR5_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_RESETVAL                             (0x00000000U)

/* INTXBAR5_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_RESETVAL                             (0x00000000U)

/* INTXBAR5_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_RESETVAL                             (0x00000000U)

/* INTXBAR5_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_RESETVAL                             (0x00000000U)

/* INTXBAR5_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_RESETVAL                             (0x00000000U)

/* INTXBAR5_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_RESETVAL                             (0x00000000U)

/* INTXBAR5_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_RESETVAL                             (0x00000000U)

/* INTXBAR6_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_RESETVAL                             (0x00000000U)

/* INTXBAR6_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_RESETVAL                             (0x00000000U)

/* INTXBAR6_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_RESETVAL                             (0x00000000U)

/* INTXBAR6_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_RESETVAL                             (0x00000000U)

/* INTXBAR6_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_RESETVAL                             (0x00000000U)

/* INTXBAR6_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_RESETVAL                             (0x00000000U)

/* INTXBAR6_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_RESETVAL                             (0x00000000U)

/* INTXBAR6_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_RESETVAL                             (0x00000000U)

/* INTXBAR7_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_RESETVAL                             (0x00000000U)

/* INTXBAR7_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_RESETVAL                             (0x00000000U)

/* INTXBAR7_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_RESETVAL                             (0x00000000U)

/* INTXBAR7_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_RESETVAL                             (0x00000000U)

/* INTXBAR7_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_RESETVAL                             (0x00000000U)

/* INTXBAR7_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_RESETVAL                             (0x00000000U)

/* INTXBAR7_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_RESETVAL                             (0x00000000U)

/* INTXBAR7_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_RESETVAL                             (0x00000000U)

/* INTXBAR8_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_RESETVAL                             (0x00000000U)

/* INTXBAR8_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_RESETVAL                             (0x00000000U)

/* INTXBAR8_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_RESETVAL                             (0x00000000U)

/* INTXBAR8_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_RESETVAL                             (0x00000000U)

/* INTXBAR8_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_RESETVAL                             (0x00000000U)

/* INTXBAR8_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_RESETVAL                             (0x00000000U)

/* INTXBAR8_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_RESETVAL                             (0x00000000U)

/* INTXBAR8_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_RESETVAL                             (0x00000000U)

/* INTXBAR9_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_RESETVAL                             (0x00000000U)

/* INTXBAR9_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_RESETVAL                             (0x00000000U)

/* INTXBAR9_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_ADC_MASK                         (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_ADC_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_ADC_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_ADC_MAX                          (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_EVTAGG_MASK                      (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_EVTAGG_SHIFT                     (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_EVTAGG_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_EVTAGG_MAX                       (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_RESETVAL                             (0x00000000U)

/* INTXBAR9_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_FSIRX_MASK                       (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_FSIRX_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_FSIRX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_FSIRX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_FSITX_MASK                       (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_FSITX_SHIFT                      (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_FSITX_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_FSITX_MAX                        (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_RESETVAL                             (0x00000000U)

/* INTXBAR9_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_RESETVAL                             (0x00000000U)

/* INTXBAR9_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_SEL_MASK                             (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_SEL_MAX                              (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_RESETVAL                             (0x00000000U)

/* INTXBAR9_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_SEL_MASK                             (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_SEL_MAX                              (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_RESETVAL                             (0x00000000U)

/* INTXBAR9_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_SEL_MASK                             (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_SEL_MAX                              (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_RESETVAL                             (0x00000000U)

/* INTXBAR10_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_RESETVAL                            (0x00000000U)

/* INTXBAR10_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_RESETVAL                            (0x00000000U)

/* INTXBAR10_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_RESETVAL                            (0x00000000U)

/* INTXBAR10_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_RESETVAL                            (0x00000000U)

/* INTXBAR10_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_RESETVAL                            (0x00000000U)

/* INTXBAR10_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_RESETVAL                            (0x00000000U)

/* INTXBAR10_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_RESETVAL                            (0x00000000U)

/* INTXBAR10_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_RESETVAL                            (0x00000000U)

/* INTXBAR11_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_RESETVAL                            (0x00000000U)

/* INTXBAR11_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_RESETVAL                            (0x00000000U)

/* INTXBAR11_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_RESETVAL                            (0x00000000U)

/* INTXBAR11_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_RESETVAL                            (0x00000000U)

/* INTXBAR11_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_RESETVAL                            (0x00000000U)

/* INTXBAR11_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_RESETVAL                            (0x00000000U)

/* INTXBAR11_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_RESETVAL                            (0x00000000U)

/* INTXBAR11_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_RESETVAL                            (0x00000000U)

/* INTXBAR12_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_RESETVAL                            (0x00000000U)

/* INTXBAR12_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_RESETVAL                            (0x00000000U)

/* INTXBAR12_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_RESETVAL                            (0x00000000U)

/* INTXBAR12_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_RESETVAL                            (0x00000000U)

/* INTXBAR12_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_RESETVAL                            (0x00000000U)

/* INTXBAR12_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_RESETVAL                            (0x00000000U)

/* INTXBAR12_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_RESETVAL                            (0x00000000U)

/* INTXBAR12_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_RESETVAL                            (0x00000000U)

/* INTXBAR13_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_RESETVAL                            (0x00000000U)

/* INTXBAR13_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_RESETVAL                            (0x00000000U)

/* INTXBAR13_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_RESETVAL                            (0x00000000U)

/* INTXBAR13_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_RESETVAL                            (0x00000000U)

/* INTXBAR13_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_RESETVAL                            (0x00000000U)

/* INTXBAR13_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_RESETVAL                            (0x00000000U)

/* INTXBAR13_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_RESETVAL                            (0x00000000U)

/* INTXBAR13_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_RESETVAL                            (0x00000000U)

/* INTXBAR14_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_RESETVAL                            (0x00000000U)

/* INTXBAR14_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_RESETVAL                            (0x00000000U)

/* INTXBAR14_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_RESETVAL                            (0x00000000U)

/* INTXBAR14_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_RESETVAL                            (0x00000000U)

/* INTXBAR14_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_RESETVAL                            (0x00000000U)

/* INTXBAR14_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_RESETVAL                            (0x00000000U)

/* INTXBAR14_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_RESETVAL                            (0x00000000U)

/* INTXBAR14_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_RESETVAL                            (0x00000000U)

/* INTXBAR15_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_RESETVAL                            (0x00000000U)

/* INTXBAR15_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_RESETVAL                            (0x00000000U)

/* INTXBAR15_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_RESETVAL                            (0x00000000U)

/* INTXBAR15_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_RESETVAL                            (0x00000000U)

/* INTXBAR15_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_RESETVAL                            (0x00000000U)

/* INTXBAR15_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_RESETVAL                            (0x00000000U)

/* INTXBAR15_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_RESETVAL                            (0x00000000U)

/* INTXBAR15_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_RESETVAL                            (0x00000000U)

/* INTXBAR16_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_RESETVAL                            (0x00000000U)

/* INTXBAR16_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_RESETVAL                            (0x00000000U)

/* INTXBAR16_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_RESETVAL                            (0x00000000U)

/* INTXBAR16_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_RESETVAL                            (0x00000000U)

/* INTXBAR16_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_RESETVAL                            (0x00000000U)

/* INTXBAR16_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_RESETVAL                            (0x00000000U)

/* INTXBAR16_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_RESETVAL                            (0x00000000U)

/* INTXBAR16_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_RESETVAL                            (0x00000000U)

/* INTXBAR17_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_RESETVAL                            (0x00000000U)

/* INTXBAR17_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_RESETVAL                            (0x00000000U)

/* INTXBAR17_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_RESETVAL                            (0x00000000U)

/* INTXBAR17_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_RESETVAL                            (0x00000000U)

/* INTXBAR17_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_RESETVAL                            (0x00000000U)

/* INTXBAR17_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_RESETVAL                            (0x00000000U)

/* INTXBAR17_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_RESETVAL                            (0x00000000U)

/* INTXBAR17_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_RESETVAL                            (0x00000000U)

/* INTXBAR18_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_RESETVAL                            (0x00000000U)

/* INTXBAR18_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_RESETVAL                            (0x00000000U)

/* INTXBAR18_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_RESETVAL                            (0x00000000U)

/* INTXBAR18_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_RESETVAL                            (0x00000000U)

/* INTXBAR18_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_RESETVAL                            (0x00000000U)

/* INTXBAR18_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_RESETVAL                            (0x00000000U)

/* INTXBAR18_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_RESETVAL                            (0x00000000U)

/* INTXBAR18_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_RESETVAL                            (0x00000000U)

/* INTXBAR19_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_RESETVAL                            (0x00000000U)

/* INTXBAR19_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_RESETVAL                            (0x00000000U)

/* INTXBAR19_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_RESETVAL                            (0x00000000U)

/* INTXBAR19_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_RESETVAL                            (0x00000000U)

/* INTXBAR19_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_RESETVAL                            (0x00000000U)

/* INTXBAR19_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_RESETVAL                            (0x00000000U)

/* INTXBAR19_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_RESETVAL                            (0x00000000U)

/* INTXBAR19_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_RESETVAL                            (0x00000000U)

/* INTXBAR20_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_RESETVAL                            (0x00000000U)

/* INTXBAR20_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_RESETVAL                            (0x00000000U)

/* INTXBAR20_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_RESETVAL                            (0x00000000U)

/* INTXBAR20_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_RESETVAL                            (0x00000000U)

/* INTXBAR20_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_RESETVAL                            (0x00000000U)

/* INTXBAR20_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_RESETVAL                            (0x00000000U)

/* INTXBAR20_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_RESETVAL                            (0x00000000U)

/* INTXBAR20_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_RESETVAL                            (0x00000000U)

/* INTXBAR21_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_RESETVAL                            (0x00000000U)

/* INTXBAR21_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_RESETVAL                            (0x00000000U)

/* INTXBAR21_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_RESETVAL                            (0x00000000U)

/* INTXBAR21_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_RESETVAL                            (0x00000000U)

/* INTXBAR21_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_RESETVAL                            (0x00000000U)

/* INTXBAR21_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_RESETVAL                            (0x00000000U)

/* INTXBAR21_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_RESETVAL                            (0x00000000U)

/* INTXBAR21_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_RESETVAL                            (0x00000000U)

/* INTXBAR22_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_RESETVAL                            (0x00000000U)

/* INTXBAR22_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_RESETVAL                            (0x00000000U)

/* INTXBAR22_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_RESETVAL                            (0x00000000U)

/* INTXBAR22_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_RESETVAL                            (0x00000000U)

/* INTXBAR22_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_RESETVAL                            (0x00000000U)

/* INTXBAR22_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_RESETVAL                            (0x00000000U)

/* INTXBAR22_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_RESETVAL                            (0x00000000U)

/* INTXBAR22_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_RESETVAL                            (0x00000000U)

/* INTXBAR23_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_RESETVAL                            (0x00000000U)

/* INTXBAR23_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_RESETVAL                            (0x00000000U)

/* INTXBAR23_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_RESETVAL                            (0x00000000U)

/* INTXBAR23_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_RESETVAL                            (0x00000000U)

/* INTXBAR23_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_RESETVAL                            (0x00000000U)

/* INTXBAR23_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_RESETVAL                            (0x00000000U)

/* INTXBAR23_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_RESETVAL                            (0x00000000U)

/* INTXBAR23_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_RESETVAL                            (0x00000000U)

/* INTXBAR24_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_RESETVAL                            (0x00000000U)

/* INTXBAR24_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_RESETVAL                            (0x00000000U)

/* INTXBAR24_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_RESETVAL                            (0x00000000U)

/* INTXBAR24_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_RESETVAL                            (0x00000000U)

/* INTXBAR24_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_RESETVAL                            (0x00000000U)

/* INTXBAR24_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_RESETVAL                            (0x00000000U)

/* INTXBAR24_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_RESETVAL                            (0x00000000U)

/* INTXBAR24_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_RESETVAL                            (0x00000000U)

/* INTXBAR25_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_RESETVAL                            (0x00000000U)

/* INTXBAR25_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_RESETVAL                            (0x00000000U)

/* INTXBAR25_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_RESETVAL                            (0x00000000U)

/* INTXBAR25_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_RESETVAL                            (0x00000000U)

/* INTXBAR25_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_RESETVAL                            (0x00000000U)

/* INTXBAR25_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_RESETVAL                            (0x00000000U)

/* INTXBAR25_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_RESETVAL                            (0x00000000U)

/* INTXBAR25_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_RESETVAL                            (0x00000000U)

/* INTXBAR26_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_RESETVAL                            (0x00000000U)

/* INTXBAR26_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_RESETVAL                            (0x00000000U)

/* INTXBAR26_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_RESETVAL                            (0x00000000U)

/* INTXBAR26_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_RESETVAL                            (0x00000000U)

/* INTXBAR26_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_RESETVAL                            (0x00000000U)

/* INTXBAR26_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_RESETVAL                            (0x00000000U)

/* INTXBAR26_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_RESETVAL                            (0x00000000U)

/* INTXBAR26_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_RESETVAL                            (0x00000000U)

/* INTXBAR27_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_RESETVAL                            (0x00000000U)

/* INTXBAR27_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_RESETVAL                            (0x00000000U)

/* INTXBAR27_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_RESETVAL                            (0x00000000U)

/* INTXBAR27_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_RESETVAL                            (0x00000000U)

/* INTXBAR27_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_RESETVAL                            (0x00000000U)

/* INTXBAR27_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_RESETVAL                            (0x00000000U)

/* INTXBAR27_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_RESETVAL                            (0x00000000U)

/* INTXBAR27_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_RESETVAL                            (0x00000000U)

/* INTXBAR28_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_RESETVAL                            (0x00000000U)

/* INTXBAR28_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_RESETVAL                            (0x00000000U)

/* INTXBAR28_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_RESETVAL                            (0x00000000U)

/* INTXBAR28_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_RESETVAL                            (0x00000000U)

/* INTXBAR28_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_RESETVAL                            (0x00000000U)

/* INTXBAR28_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_RESETVAL                            (0x00000000U)

/* INTXBAR28_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_RESETVAL                            (0x00000000U)

/* INTXBAR28_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_RESETVAL                            (0x00000000U)

/* INTXBAR29_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_RESETVAL                            (0x00000000U)

/* INTXBAR29_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_RESETVAL                            (0x00000000U)

/* INTXBAR29_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_RESETVAL                            (0x00000000U)

/* INTXBAR29_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_RESETVAL                            (0x00000000U)

/* INTXBAR29_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_RESETVAL                            (0x00000000U)

/* INTXBAR29_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_RESETVAL                            (0x00000000U)

/* INTXBAR29_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_RESETVAL                            (0x00000000U)

/* INTXBAR29_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_RESETVAL                            (0x00000000U)

/* INTXBAR30_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_RESETVAL                            (0x00000000U)

/* INTXBAR30_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_RESETVAL                            (0x00000000U)

/* INTXBAR30_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_RESETVAL                            (0x00000000U)

/* INTXBAR30_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_RESETVAL                            (0x00000000U)

/* INTXBAR30_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_RESETVAL                            (0x00000000U)

/* INTXBAR30_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_RESETVAL                            (0x00000000U)

/* INTXBAR30_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_RESETVAL                            (0x00000000U)

/* INTXBAR30_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_RESETVAL                            (0x00000000U)

/* INTXBAR31_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_RESETVAL                            (0x00000000U)

/* INTXBAR31_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_RESETVAL                            (0x00000000U)

/* INTXBAR31_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_ADC_MASK                        (0x00007FFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_ADC_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_ADC_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_ADC_MAX                         (0x00007FFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_EVTAGG_MASK                     (0x80000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_EVTAGG_SHIFT                    (0x0000001FU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_EVTAGG_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_EVTAGG_MAX                      (0x00000001U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_RESETVAL                            (0x00000000U)

/* INTXBAR31_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_FSIRX_MASK                      (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_FSIRX_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_FSIRX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_FSIRX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_FSITX_MASK                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_FSITX_SHIFT                     (0x00000008U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_FSITX_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_FSITX_MAX                       (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_RESETVAL                            (0x00000000U)

/* INTXBAR31_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_RESETVAL                            (0x00000000U)

/* INTXBAR31_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_SEL_MASK                            (0x000000FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_SEL_MAX                             (0x000000FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_RESETVAL                            (0x00000000U)

/* INTXBAR31_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_SEL_MASK                            (0x00000003U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_SEL_MAX                             (0x00000003U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_RESETVAL                            (0x00000000U)

/* INTXBAR31_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_SEL_MASK                            (0x0003FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_SEL_MAX                             (0x0003FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_RESETVAL                            (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
