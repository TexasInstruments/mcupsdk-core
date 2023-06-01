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
    volatile uint32_t INTXBAR0_G8;
    volatile uint32_t INTXBAR0_G9;
    volatile uint8_t  Resv_320[24];
    volatile uint32_t INTXBAR1_G0;
    volatile uint32_t INTXBAR1_G1;
    volatile uint32_t INTXBAR1_G2;
    volatile uint32_t INTXBAR1_G3;
    volatile uint32_t INTXBAR1_G4;
    volatile uint32_t INTXBAR1_G5;
    volatile uint32_t INTXBAR1_G6;
    volatile uint32_t INTXBAR1_G7;
    volatile uint32_t INTXBAR1_G8;
    volatile uint32_t INTXBAR1_G9;
    volatile uint8_t  Resv_384[24];
    volatile uint32_t INTXBAR2_G0;
    volatile uint32_t INTXBAR2_G1;
    volatile uint32_t INTXBAR2_G2;
    volatile uint32_t INTXBAR2_G3;
    volatile uint32_t INTXBAR2_G4;
    volatile uint32_t INTXBAR2_G5;
    volatile uint32_t INTXBAR2_G6;
    volatile uint32_t INTXBAR2_G7;
    volatile uint32_t INTXBAR2_G8;
    volatile uint32_t INTXBAR2_G9;
    volatile uint8_t  Resv_448[24];
    volatile uint32_t INTXBAR3_G0;
    volatile uint32_t INTXBAR3_G1;
    volatile uint32_t INTXBAR3_G2;
    volatile uint32_t INTXBAR3_G3;
    volatile uint32_t INTXBAR3_G4;
    volatile uint32_t INTXBAR3_G5;
    volatile uint32_t INTXBAR3_G6;
    volatile uint32_t INTXBAR3_G7;
    volatile uint32_t INTXBAR3_G8;
    volatile uint32_t INTXBAR3_G9;
    volatile uint8_t  Resv_512[24];
    volatile uint32_t INTXBAR4_G0;
    volatile uint32_t INTXBAR4_G1;
    volatile uint32_t INTXBAR4_G2;
    volatile uint32_t INTXBAR4_G3;
    volatile uint32_t INTXBAR4_G4;
    volatile uint32_t INTXBAR4_G5;
    volatile uint32_t INTXBAR4_G6;
    volatile uint32_t INTXBAR4_G7;
    volatile uint32_t INTXBAR4_G8;
    volatile uint32_t INTXBAR4_G9;
    volatile uint8_t  Resv_576[24];
    volatile uint32_t INTXBAR5_G0;
    volatile uint32_t INTXBAR5_G1;
    volatile uint32_t INTXBAR5_G2;
    volatile uint32_t INTXBAR5_G3;
    volatile uint32_t INTXBAR5_G4;
    volatile uint32_t INTXBAR5_G5;
    volatile uint32_t INTXBAR5_G6;
    volatile uint32_t INTXBAR5_G7;
    volatile uint32_t INTXBAR5_G8;
    volatile uint32_t INTXBAR5_G9;
    volatile uint8_t  Resv_640[24];
    volatile uint32_t INTXBAR6_G0;
    volatile uint32_t INTXBAR6_G1;
    volatile uint32_t INTXBAR6_G2;
    volatile uint32_t INTXBAR6_G3;
    volatile uint32_t INTXBAR6_G4;
    volatile uint32_t INTXBAR6_G5;
    volatile uint32_t INTXBAR6_G6;
    volatile uint32_t INTXBAR6_G7;
    volatile uint32_t INTXBAR6_G8;
    volatile uint32_t INTXBAR6_G9;
    volatile uint8_t  Resv_704[24];
    volatile uint32_t INTXBAR7_G0;
    volatile uint32_t INTXBAR7_G1;
    volatile uint32_t INTXBAR7_G2;
    volatile uint32_t INTXBAR7_G3;
    volatile uint32_t INTXBAR7_G4;
    volatile uint32_t INTXBAR7_G5;
    volatile uint32_t INTXBAR7_G6;
    volatile uint32_t INTXBAR7_G7;
    volatile uint32_t INTXBAR7_G8;
    volatile uint32_t INTXBAR7_G9;
    volatile uint8_t  Resv_768[24];
    volatile uint32_t INTXBAR8_G0;
    volatile uint32_t INTXBAR8_G1;
    volatile uint32_t INTXBAR8_G2;
    volatile uint32_t INTXBAR8_G3;
    volatile uint32_t INTXBAR8_G4;
    volatile uint32_t INTXBAR8_G5;
    volatile uint32_t INTXBAR8_G6;
    volatile uint32_t INTXBAR8_G7;
    volatile uint32_t INTXBAR8_G8;
    volatile uint32_t INTXBAR8_G9;
    volatile uint8_t  Resv_832[24];
    volatile uint32_t INTXBAR9_G0;
    volatile uint32_t INTXBAR9_G1;
    volatile uint32_t INTXBAR9_G2;
    volatile uint32_t INTXBAR9_G3;
    volatile uint32_t INTXBAR9_G4;
    volatile uint32_t INTXBAR9_G5;
    volatile uint32_t INTXBAR9_G6;
    volatile uint32_t INTXBAR9_G7;
    volatile uint32_t INTXBAR9_G8;
    volatile uint32_t INTXBAR9_G9;
    volatile uint8_t  Resv_896[24];
    volatile uint32_t INTXBAR10_G0;
    volatile uint32_t INTXBAR10_G1;
    volatile uint32_t INTXBAR10_G2;
    volatile uint32_t INTXBAR10_G3;
    volatile uint32_t INTXBAR10_G4;
    volatile uint32_t INTXBAR10_G5;
    volatile uint32_t INTXBAR10_G6;
    volatile uint32_t INTXBAR10_G7;
    volatile uint32_t INTXBAR10_G8;
    volatile uint32_t INTXBAR10_G9;
    volatile uint8_t  Resv_960[24];
    volatile uint32_t INTXBAR11_G0;
    volatile uint32_t INTXBAR11_G1;
    volatile uint32_t INTXBAR11_G2;
    volatile uint32_t INTXBAR11_G3;
    volatile uint32_t INTXBAR11_G4;
    volatile uint32_t INTXBAR11_G5;
    volatile uint32_t INTXBAR11_G6;
    volatile uint32_t INTXBAR11_G7;
    volatile uint32_t INTXBAR11_G8;
    volatile uint32_t INTXBAR11_G9;
    volatile uint8_t  Resv_1024[24];
    volatile uint32_t INTXBAR12_G0;
    volatile uint32_t INTXBAR12_G1;
    volatile uint32_t INTXBAR12_G2;
    volatile uint32_t INTXBAR12_G3;
    volatile uint32_t INTXBAR12_G4;
    volatile uint32_t INTXBAR12_G5;
    volatile uint32_t INTXBAR12_G6;
    volatile uint32_t INTXBAR12_G7;
    volatile uint32_t INTXBAR12_G8;
    volatile uint32_t INTXBAR12_G9;
    volatile uint8_t  Resv_1088[24];
    volatile uint32_t INTXBAR13_G0;
    volatile uint32_t INTXBAR13_G1;
    volatile uint32_t INTXBAR13_G2;
    volatile uint32_t INTXBAR13_G3;
    volatile uint32_t INTXBAR13_G4;
    volatile uint32_t INTXBAR13_G5;
    volatile uint32_t INTXBAR13_G6;
    volatile uint32_t INTXBAR13_G7;
    volatile uint32_t INTXBAR13_G8;
    volatile uint32_t INTXBAR13_G9;
    volatile uint8_t  Resv_1152[24];
    volatile uint32_t INTXBAR14_G0;
    volatile uint32_t INTXBAR14_G1;
    volatile uint32_t INTXBAR14_G2;
    volatile uint32_t INTXBAR14_G3;
    volatile uint32_t INTXBAR14_G4;
    volatile uint32_t INTXBAR14_G5;
    volatile uint32_t INTXBAR14_G6;
    volatile uint32_t INTXBAR14_G7;
    volatile uint32_t INTXBAR14_G8;
    volatile uint32_t INTXBAR14_G9;
    volatile uint8_t  Resv_1216[24];
    volatile uint32_t INTXBAR15_G0;
    volatile uint32_t INTXBAR15_G1;
    volatile uint32_t INTXBAR15_G2;
    volatile uint32_t INTXBAR15_G3;
    volatile uint32_t INTXBAR15_G4;
    volatile uint32_t INTXBAR15_G5;
    volatile uint32_t INTXBAR15_G6;
    volatile uint32_t INTXBAR15_G7;
    volatile uint32_t INTXBAR15_G8;
    volatile uint32_t INTXBAR15_G9;
    volatile uint8_t  Resv_1280[24];
    volatile uint32_t INTXBAR16_G0;
    volatile uint32_t INTXBAR16_G1;
    volatile uint32_t INTXBAR16_G2;
    volatile uint32_t INTXBAR16_G3;
    volatile uint32_t INTXBAR16_G4;
    volatile uint32_t INTXBAR16_G5;
    volatile uint32_t INTXBAR16_G6;
    volatile uint32_t INTXBAR16_G7;
    volatile uint32_t INTXBAR16_G8;
    volatile uint32_t INTXBAR16_G9;
    volatile uint8_t  Resv_1344[24];
    volatile uint32_t INTXBAR17_G0;
    volatile uint32_t INTXBAR17_G1;
    volatile uint32_t INTXBAR17_G2;
    volatile uint32_t INTXBAR17_G3;
    volatile uint32_t INTXBAR17_G4;
    volatile uint32_t INTXBAR17_G5;
    volatile uint32_t INTXBAR17_G6;
    volatile uint32_t INTXBAR17_G7;
    volatile uint32_t INTXBAR17_G8;
    volatile uint32_t INTXBAR17_G9;
    volatile uint8_t  Resv_1408[24];
    volatile uint32_t INTXBAR18_G0;
    volatile uint32_t INTXBAR18_G1;
    volatile uint32_t INTXBAR18_G2;
    volatile uint32_t INTXBAR18_G3;
    volatile uint32_t INTXBAR18_G4;
    volatile uint32_t INTXBAR18_G5;
    volatile uint32_t INTXBAR18_G6;
    volatile uint32_t INTXBAR18_G7;
    volatile uint32_t INTXBAR18_G8;
    volatile uint32_t INTXBAR18_G9;
    volatile uint8_t  Resv_1472[24];
    volatile uint32_t INTXBAR19_G0;
    volatile uint32_t INTXBAR19_G1;
    volatile uint32_t INTXBAR19_G2;
    volatile uint32_t INTXBAR19_G3;
    volatile uint32_t INTXBAR19_G4;
    volatile uint32_t INTXBAR19_G5;
    volatile uint32_t INTXBAR19_G6;
    volatile uint32_t INTXBAR19_G7;
    volatile uint32_t INTXBAR19_G8;
    volatile uint32_t INTXBAR19_G9;
    volatile uint8_t  Resv_1536[24];
    volatile uint32_t INTXBAR20_G0;
    volatile uint32_t INTXBAR20_G1;
    volatile uint32_t INTXBAR20_G2;
    volatile uint32_t INTXBAR20_G3;
    volatile uint32_t INTXBAR20_G4;
    volatile uint32_t INTXBAR20_G5;
    volatile uint32_t INTXBAR20_G6;
    volatile uint32_t INTXBAR20_G7;
    volatile uint32_t INTXBAR20_G8;
    volatile uint32_t INTXBAR20_G9;
    volatile uint8_t  Resv_1600[24];
    volatile uint32_t INTXBAR21_G0;
    volatile uint32_t INTXBAR21_G1;
    volatile uint32_t INTXBAR21_G2;
    volatile uint32_t INTXBAR21_G3;
    volatile uint32_t INTXBAR21_G4;
    volatile uint32_t INTXBAR21_G5;
    volatile uint32_t INTXBAR21_G6;
    volatile uint32_t INTXBAR21_G7;
    volatile uint32_t INTXBAR21_G8;
    volatile uint32_t INTXBAR21_G9;
    volatile uint8_t  Resv_1664[24];
    volatile uint32_t INTXBAR22_G0;
    volatile uint32_t INTXBAR22_G1;
    volatile uint32_t INTXBAR22_G2;
    volatile uint32_t INTXBAR22_G3;
    volatile uint32_t INTXBAR22_G4;
    volatile uint32_t INTXBAR22_G5;
    volatile uint32_t INTXBAR22_G6;
    volatile uint32_t INTXBAR22_G7;
    volatile uint32_t INTXBAR22_G8;
    volatile uint32_t INTXBAR22_G9;
    volatile uint8_t  Resv_1728[24];
    volatile uint32_t INTXBAR23_G0;
    volatile uint32_t INTXBAR23_G1;
    volatile uint32_t INTXBAR23_G2;
    volatile uint32_t INTXBAR23_G3;
    volatile uint32_t INTXBAR23_G4;
    volatile uint32_t INTXBAR23_G5;
    volatile uint32_t INTXBAR23_G6;
    volatile uint32_t INTXBAR23_G7;
    volatile uint32_t INTXBAR23_G8;
    volatile uint32_t INTXBAR23_G9;
    volatile uint8_t  Resv_1792[24];
    volatile uint32_t INTXBAR24_G0;
    volatile uint32_t INTXBAR24_G1;
    volatile uint32_t INTXBAR24_G2;
    volatile uint32_t INTXBAR24_G3;
    volatile uint32_t INTXBAR24_G4;
    volatile uint32_t INTXBAR24_G5;
    volatile uint32_t INTXBAR24_G6;
    volatile uint32_t INTXBAR24_G7;
    volatile uint32_t INTXBAR24_G8;
    volatile uint32_t INTXBAR24_G9;
    volatile uint8_t  Resv_1856[24];
    volatile uint32_t INTXBAR25_G0;
    volatile uint32_t INTXBAR25_G1;
    volatile uint32_t INTXBAR25_G2;
    volatile uint32_t INTXBAR25_G3;
    volatile uint32_t INTXBAR25_G4;
    volatile uint32_t INTXBAR25_G5;
    volatile uint32_t INTXBAR25_G6;
    volatile uint32_t INTXBAR25_G7;
    volatile uint32_t INTXBAR25_G8;
    volatile uint32_t INTXBAR25_G9;
    volatile uint8_t  Resv_1920[24];
    volatile uint32_t INTXBAR26_G0;
    volatile uint32_t INTXBAR26_G1;
    volatile uint32_t INTXBAR26_G2;
    volatile uint32_t INTXBAR26_G3;
    volatile uint32_t INTXBAR26_G4;
    volatile uint32_t INTXBAR26_G5;
    volatile uint32_t INTXBAR26_G6;
    volatile uint32_t INTXBAR26_G7;
    volatile uint32_t INTXBAR26_G8;
    volatile uint32_t INTXBAR26_G9;
    volatile uint8_t  Resv_1984[24];
    volatile uint32_t INTXBAR27_G0;
    volatile uint32_t INTXBAR27_G1;
    volatile uint32_t INTXBAR27_G2;
    volatile uint32_t INTXBAR27_G3;
    volatile uint32_t INTXBAR27_G4;
    volatile uint32_t INTXBAR27_G5;
    volatile uint32_t INTXBAR27_G6;
    volatile uint32_t INTXBAR27_G7;
    volatile uint32_t INTXBAR27_G8;
    volatile uint32_t INTXBAR27_G9;
    volatile uint8_t  Resv_2048[24];
    volatile uint32_t INTXBAR28_G0;
    volatile uint32_t INTXBAR28_G1;
    volatile uint32_t INTXBAR28_G2;
    volatile uint32_t INTXBAR28_G3;
    volatile uint32_t INTXBAR28_G4;
    volatile uint32_t INTXBAR28_G5;
    volatile uint32_t INTXBAR28_G6;
    volatile uint32_t INTXBAR28_G7;
    volatile uint32_t INTXBAR28_G8;
    volatile uint32_t INTXBAR28_G9;
    volatile uint8_t  Resv_2112[24];
    volatile uint32_t INTXBAR29_G0;
    volatile uint32_t INTXBAR29_G1;
    volatile uint32_t INTXBAR29_G2;
    volatile uint32_t INTXBAR29_G3;
    volatile uint32_t INTXBAR29_G4;
    volatile uint32_t INTXBAR29_G5;
    volatile uint32_t INTXBAR29_G6;
    volatile uint32_t INTXBAR29_G7;
    volatile uint32_t INTXBAR29_G8;
    volatile uint32_t INTXBAR29_G9;
    volatile uint8_t  Resv_2176[24];
    volatile uint32_t INTXBAR30_G0;
    volatile uint32_t INTXBAR30_G1;
    volatile uint32_t INTXBAR30_G2;
    volatile uint32_t INTXBAR30_G3;
    volatile uint32_t INTXBAR30_G4;
    volatile uint32_t INTXBAR30_G5;
    volatile uint32_t INTXBAR30_G6;
    volatile uint32_t INTXBAR30_G7;
    volatile uint32_t INTXBAR30_G8;
    volatile uint32_t INTXBAR30_G9;
    volatile uint8_t  Resv_2240[24];
    volatile uint32_t INTXBAR31_G0;
    volatile uint32_t INTXBAR31_G1;
    volatile uint32_t INTXBAR31_G2;
    volatile uint32_t INTXBAR31_G3;
    volatile uint32_t INTXBAR31_G4;
    volatile uint32_t INTXBAR31_G5;
    volatile uint32_t INTXBAR31_G6;
    volatile uint32_t INTXBAR31_G7;
    volatile uint32_t INTXBAR31_G8;
    volatile uint32_t INTXBAR31_G9;
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
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G8                                      (0x00000120U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G9                                      (0x00000124U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0                                      (0x00000140U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1                                      (0x00000144U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2                                      (0x00000148U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3                                      (0x0000014CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4                                      (0x00000150U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5                                      (0x00000154U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6                                      (0x00000158U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7                                      (0x0000015CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G8                                      (0x00000160U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G9                                      (0x00000164U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0                                      (0x00000180U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1                                      (0x00000184U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2                                      (0x00000188U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3                                      (0x0000018CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4                                      (0x00000190U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5                                      (0x00000194U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6                                      (0x00000198U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7                                      (0x0000019CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G8                                      (0x000001A0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G9                                      (0x000001A4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0                                      (0x000001C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1                                      (0x000001C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2                                      (0x000001C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3                                      (0x000001CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4                                      (0x000001D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5                                      (0x000001D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6                                      (0x000001D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7                                      (0x000001DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G8                                      (0x000001E0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G9                                      (0x000001E4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0                                      (0x00000200U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1                                      (0x00000204U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2                                      (0x00000208U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3                                      (0x0000020CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4                                      (0x00000210U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5                                      (0x00000214U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6                                      (0x00000218U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7                                      (0x0000021CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G8                                      (0x00000220U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G9                                      (0x00000224U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0                                      (0x00000240U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1                                      (0x00000244U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2                                      (0x00000248U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3                                      (0x0000024CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4                                      (0x00000250U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5                                      (0x00000254U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6                                      (0x00000258U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7                                      (0x0000025CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G8                                      (0x00000260U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G9                                      (0x00000264U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0                                      (0x00000280U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1                                      (0x00000284U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2                                      (0x00000288U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3                                      (0x0000028CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4                                      (0x00000290U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5                                      (0x00000294U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6                                      (0x00000298U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7                                      (0x0000029CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G8                                      (0x000002A0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G9                                      (0x000002A4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0                                      (0x000002C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1                                      (0x000002C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2                                      (0x000002C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3                                      (0x000002CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4                                      (0x000002D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5                                      (0x000002D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6                                      (0x000002D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7                                      (0x000002DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G8                                      (0x000002E0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G9                                      (0x000002E4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0                                      (0x00000300U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1                                      (0x00000304U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2                                      (0x00000308U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3                                      (0x0000030CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4                                      (0x00000310U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5                                      (0x00000314U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6                                      (0x00000318U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7                                      (0x0000031CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G8                                      (0x00000320U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G9                                      (0x00000324U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0                                      (0x00000340U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1                                      (0x00000344U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2                                      (0x00000348U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3                                      (0x0000034CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4                                      (0x00000350U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5                                      (0x00000354U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6                                      (0x00000358U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7                                      (0x0000035CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G8                                      (0x00000360U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G9                                      (0x00000364U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0                                     (0x00000380U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1                                     (0x00000384U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2                                     (0x00000388U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3                                     (0x0000038CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4                                     (0x00000390U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5                                     (0x00000394U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6                                     (0x00000398U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7                                     (0x0000039CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G8                                     (0x000003A0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G9                                     (0x000003A4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0                                     (0x000003C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1                                     (0x000003C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2                                     (0x000003C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3                                     (0x000003CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4                                     (0x000003D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5                                     (0x000003D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6                                     (0x000003D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7                                     (0x000003DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G8                                     (0x000003E0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G9                                     (0x000003E4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0                                     (0x00000400U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1                                     (0x00000404U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2                                     (0x00000408U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3                                     (0x0000040CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4                                     (0x00000410U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5                                     (0x00000414U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6                                     (0x00000418U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7                                     (0x0000041CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G8                                     (0x00000420U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G9                                     (0x00000424U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0                                     (0x00000440U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1                                     (0x00000444U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2                                     (0x00000448U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3                                     (0x0000044CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4                                     (0x00000450U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5                                     (0x00000454U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6                                     (0x00000458U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7                                     (0x0000045CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G8                                     (0x00000460U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G9                                     (0x00000464U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0                                     (0x00000480U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1                                     (0x00000484U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2                                     (0x00000488U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3                                     (0x0000048CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4                                     (0x00000490U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5                                     (0x00000494U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6                                     (0x00000498U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7                                     (0x0000049CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G8                                     (0x000004A0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G9                                     (0x000004A4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0                                     (0x000004C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1                                     (0x000004C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2                                     (0x000004C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3                                     (0x000004CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4                                     (0x000004D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5                                     (0x000004D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6                                     (0x000004D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7                                     (0x000004DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G8                                     (0x000004E0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G9                                     (0x000004E4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0                                     (0x00000500U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1                                     (0x00000504U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2                                     (0x00000508U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3                                     (0x0000050CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4                                     (0x00000510U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5                                     (0x00000514U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6                                     (0x00000518U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7                                     (0x0000051CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G8                                     (0x00000520U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G9                                     (0x00000524U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0                                     (0x00000540U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1                                     (0x00000544U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2                                     (0x00000548U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3                                     (0x0000054CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4                                     (0x00000550U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5                                     (0x00000554U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6                                     (0x00000558U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7                                     (0x0000055CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G8                                     (0x00000560U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G9                                     (0x00000564U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0                                     (0x00000580U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1                                     (0x00000584U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2                                     (0x00000588U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3                                     (0x0000058CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4                                     (0x00000590U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5                                     (0x00000594U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6                                     (0x00000598U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7                                     (0x0000059CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G8                                     (0x000005A0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G9                                     (0x000005A4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0                                     (0x000005C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1                                     (0x000005C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2                                     (0x000005C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3                                     (0x000005CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4                                     (0x000005D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5                                     (0x000005D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6                                     (0x000005D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7                                     (0x000005DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G8                                     (0x000005E0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G9                                     (0x000005E4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0                                     (0x00000600U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1                                     (0x00000604U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2                                     (0x00000608U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3                                     (0x0000060CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4                                     (0x00000610U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5                                     (0x00000614U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6                                     (0x00000618U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7                                     (0x0000061CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G8                                     (0x00000620U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G9                                     (0x00000624U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0                                     (0x00000640U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1                                     (0x00000644U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2                                     (0x00000648U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3                                     (0x0000064CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4                                     (0x00000650U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5                                     (0x00000654U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6                                     (0x00000658U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7                                     (0x0000065CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G8                                     (0x00000660U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G9                                     (0x00000664U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0                                     (0x00000680U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1                                     (0x00000684U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2                                     (0x00000688U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3                                     (0x0000068CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4                                     (0x00000690U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5                                     (0x00000694U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6                                     (0x00000698U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7                                     (0x0000069CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G8                                     (0x000006A0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G9                                     (0x000006A4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0                                     (0x000006C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1                                     (0x000006C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2                                     (0x000006C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3                                     (0x000006CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4                                     (0x000006D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5                                     (0x000006D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6                                     (0x000006D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7                                     (0x000006DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G8                                     (0x000006E0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G9                                     (0x000006E4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0                                     (0x00000700U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1                                     (0x00000704U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2                                     (0x00000708U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3                                     (0x0000070CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4                                     (0x00000710U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5                                     (0x00000714U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6                                     (0x00000718U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7                                     (0x0000071CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G8                                     (0x00000720U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G9                                     (0x00000724U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0                                     (0x00000740U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1                                     (0x00000744U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2                                     (0x00000748U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3                                     (0x0000074CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4                                     (0x00000750U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5                                     (0x00000754U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6                                     (0x00000758U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7                                     (0x0000075CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G8                                     (0x00000760U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G9                                     (0x00000764U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0                                     (0x00000780U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1                                     (0x00000784U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2                                     (0x00000788U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3                                     (0x0000078CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4                                     (0x00000790U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5                                     (0x00000794U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6                                     (0x00000798U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7                                     (0x0000079CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G8                                     (0x000007A0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G9                                     (0x000007A4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0                                     (0x000007C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1                                     (0x000007C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2                                     (0x000007C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3                                     (0x000007CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4                                     (0x000007D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5                                     (0x000007D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6                                     (0x000007D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7                                     (0x000007DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G8                                     (0x000007E0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G9                                     (0x000007E4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0                                     (0x00000800U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1                                     (0x00000804U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2                                     (0x00000808U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3                                     (0x0000080CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4                                     (0x00000810U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5                                     (0x00000814U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6                                     (0x00000818U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7                                     (0x0000081CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G8                                     (0x00000820U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G9                                     (0x00000824U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0                                     (0x00000840U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1                                     (0x00000844U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2                                     (0x00000848U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3                                     (0x0000084CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4                                     (0x00000850U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5                                     (0x00000854U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6                                     (0x00000858U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7                                     (0x0000085CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G8                                     (0x00000860U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G9                                     (0x00000864U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0                                     (0x00000880U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1                                     (0x00000884U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2                                     (0x00000888U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3                                     (0x0000088CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4                                     (0x00000890U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5                                     (0x00000894U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6                                     (0x00000898U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7                                     (0x0000089CU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G8                                     (0x000008A0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G9                                     (0x000008A4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0                                     (0x000008C0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1                                     (0x000008C4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2                                     (0x000008C8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3                                     (0x000008CCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4                                     (0x000008D0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5                                     (0x000008D4U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6                                     (0x000008D8U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7                                     (0x000008DCU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G8                                     (0x000008E0U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G9                                     (0x000008E4U)

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

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G0_RESETVAL                             (0x00000000U)

/* INTXBAR0_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G1_RESETVAL                             (0x00000000U)

/* INTXBAR0_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G2_RESETVAL                             (0x00000000U)

/* INTXBAR0_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G3_RESETVAL                             (0x00000000U)

/* INTXBAR0_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G4_RESETVAL                             (0x00000000U)

/* INTXBAR0_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G5_RESETVAL                             (0x00000000U)

/* INTXBAR0_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G6_RESETVAL                             (0x00000000U)

/* INTXBAR0_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G7_RESETVAL                             (0x00000000U)

/* INTXBAR0_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G8_RESETVAL                             (0x00000000U)

/* INTXBAR0_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR0_G9_RESETVAL                             (0x00000000U)

/* INTXBAR1_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G0_RESETVAL                             (0x00000000U)

/* INTXBAR1_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G1_RESETVAL                             (0x00000000U)

/* INTXBAR1_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G2_RESETVAL                             (0x00000000U)

/* INTXBAR1_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G3_RESETVAL                             (0x00000000U)

/* INTXBAR1_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G4_RESETVAL                             (0x00000000U)

/* INTXBAR1_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G5_RESETVAL                             (0x00000000U)

/* INTXBAR1_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G6_RESETVAL                             (0x00000000U)

/* INTXBAR1_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G7_RESETVAL                             (0x00000000U)

/* INTXBAR1_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G8_RESETVAL                             (0x00000000U)

/* INTXBAR1_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR1_G9_RESETVAL                             (0x00000000U)

/* INTXBAR2_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G0_RESETVAL                             (0x00000000U)

/* INTXBAR2_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G1_RESETVAL                             (0x00000000U)

/* INTXBAR2_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G2_RESETVAL                             (0x00000000U)

/* INTXBAR2_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G3_RESETVAL                             (0x00000000U)

/* INTXBAR2_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G4_RESETVAL                             (0x00000000U)

/* INTXBAR2_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G5_RESETVAL                             (0x00000000U)

/* INTXBAR2_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G6_RESETVAL                             (0x00000000U)

/* INTXBAR2_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G7_RESETVAL                             (0x00000000U)

/* INTXBAR2_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G8_RESETVAL                             (0x00000000U)

/* INTXBAR2_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR2_G9_RESETVAL                             (0x00000000U)

/* INTXBAR3_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G0_RESETVAL                             (0x00000000U)

/* INTXBAR3_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G1_RESETVAL                             (0x00000000U)

/* INTXBAR3_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G2_RESETVAL                             (0x00000000U)

/* INTXBAR3_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G3_RESETVAL                             (0x00000000U)

/* INTXBAR3_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G4_RESETVAL                             (0x00000000U)

/* INTXBAR3_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G5_RESETVAL                             (0x00000000U)

/* INTXBAR3_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G6_RESETVAL                             (0x00000000U)

/* INTXBAR3_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G7_RESETVAL                             (0x00000000U)

/* INTXBAR3_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G8_RESETVAL                             (0x00000000U)

/* INTXBAR3_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR3_G9_RESETVAL                             (0x00000000U)

/* INTXBAR4_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G0_RESETVAL                             (0x00000000U)

/* INTXBAR4_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G1_RESETVAL                             (0x00000000U)

/* INTXBAR4_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G2_RESETVAL                             (0x00000000U)

/* INTXBAR4_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G3_RESETVAL                             (0x00000000U)

/* INTXBAR4_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G4_RESETVAL                             (0x00000000U)

/* INTXBAR4_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G5_RESETVAL                             (0x00000000U)

/* INTXBAR4_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G6_RESETVAL                             (0x00000000U)

/* INTXBAR4_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G7_RESETVAL                             (0x00000000U)

/* INTXBAR4_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G8_RESETVAL                             (0x00000000U)

/* INTXBAR4_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR4_G9_RESETVAL                             (0x00000000U)

/* INTXBAR5_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G0_RESETVAL                             (0x00000000U)

/* INTXBAR5_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G1_RESETVAL                             (0x00000000U)

/* INTXBAR5_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G2_RESETVAL                             (0x00000000U)

/* INTXBAR5_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G3_RESETVAL                             (0x00000000U)

/* INTXBAR5_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G4_RESETVAL                             (0x00000000U)

/* INTXBAR5_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G5_RESETVAL                             (0x00000000U)

/* INTXBAR5_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G6_RESETVAL                             (0x00000000U)

/* INTXBAR5_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G7_RESETVAL                             (0x00000000U)

/* INTXBAR5_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G8_RESETVAL                             (0x00000000U)

/* INTXBAR5_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR5_G9_RESETVAL                             (0x00000000U)

/* INTXBAR6_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G0_RESETVAL                             (0x00000000U)

/* INTXBAR6_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G1_RESETVAL                             (0x00000000U)

/* INTXBAR6_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G2_RESETVAL                             (0x00000000U)

/* INTXBAR6_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G3_RESETVAL                             (0x00000000U)

/* INTXBAR6_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G4_RESETVAL                             (0x00000000U)

/* INTXBAR6_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G5_RESETVAL                             (0x00000000U)

/* INTXBAR6_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G6_RESETVAL                             (0x00000000U)

/* INTXBAR6_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G7_RESETVAL                             (0x00000000U)

/* INTXBAR6_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G8_RESETVAL                             (0x00000000U)

/* INTXBAR6_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR6_G9_RESETVAL                             (0x00000000U)

/* INTXBAR7_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G0_RESETVAL                             (0x00000000U)

/* INTXBAR7_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G1_RESETVAL                             (0x00000000U)

/* INTXBAR7_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G2_RESETVAL                             (0x00000000U)

/* INTXBAR7_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G3_RESETVAL                             (0x00000000U)

/* INTXBAR7_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G4_RESETVAL                             (0x00000000U)

/* INTXBAR7_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G5_RESETVAL                             (0x00000000U)

/* INTXBAR7_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G6_RESETVAL                             (0x00000000U)

/* INTXBAR7_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G7_RESETVAL                             (0x00000000U)

/* INTXBAR7_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G8_RESETVAL                             (0x00000000U)

/* INTXBAR7_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR7_G9_RESETVAL                             (0x00000000U)

/* INTXBAR8_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G0_RESETVAL                             (0x00000000U)

/* INTXBAR8_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G1_RESETVAL                             (0x00000000U)

/* INTXBAR8_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G2_RESETVAL                             (0x00000000U)

/* INTXBAR8_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G3_RESETVAL                             (0x00000000U)

/* INTXBAR8_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G4_RESETVAL                             (0x00000000U)

/* INTXBAR8_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G5_RESETVAL                             (0x00000000U)

/* INTXBAR8_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G6_RESETVAL                             (0x00000000U)

/* INTXBAR8_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G7_RESETVAL                             (0x00000000U)

/* INTXBAR8_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G8_RESETVAL                             (0x00000000U)

/* INTXBAR8_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR8_G9_RESETVAL                             (0x00000000U)

/* INTXBAR9_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G0_RESETVAL                             (0x00000000U)

/* INTXBAR9_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G1_RESETVAL                             (0x00000000U)

/* INTXBAR9_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_MASK                             (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_SEL_MAX                              (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G2_RESETVAL                             (0x00000000U)

/* INTXBAR9_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G3_RESETVAL                             (0x00000000U)

/* INTXBAR9_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G4_RESETVAL                             (0x00000000U)

/* INTXBAR9_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_SEL_MASK                             (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_SEL_MAX                              (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G5_RESETVAL                             (0x00000000U)

/* INTXBAR9_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G6_RESETVAL                             (0x00000000U)

/* INTXBAR9_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_SEL_MASK                             (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_SEL_MAX                              (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G7_RESETVAL                             (0x00000000U)

/* INTXBAR9_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G8_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G8_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G8_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G8_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G8_RESETVAL                             (0x00000000U)

/* INTXBAR9_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G9_SEL_MASK                             (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G9_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G9_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G9_SEL_MAX                              (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR9_G9_RESETVAL                             (0x00000000U)

/* INTXBAR10_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G0_RESETVAL                            (0x00000000U)

/* INTXBAR10_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G1_RESETVAL                            (0x00000000U)

/* INTXBAR10_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G2_RESETVAL                            (0x00000000U)

/* INTXBAR10_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G3_RESETVAL                            (0x00000000U)

/* INTXBAR10_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G4_RESETVAL                            (0x00000000U)

/* INTXBAR10_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G5_RESETVAL                            (0x00000000U)

/* INTXBAR10_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G6_RESETVAL                            (0x00000000U)

/* INTXBAR10_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G7_RESETVAL                            (0x00000000U)

/* INTXBAR10_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G8_RESETVAL                            (0x00000000U)

/* INTXBAR10_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR10_G9_RESETVAL                            (0x00000000U)

/* INTXBAR11_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G0_RESETVAL                            (0x00000000U)

/* INTXBAR11_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G1_RESETVAL                            (0x00000000U)

/* INTXBAR11_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G2_RESETVAL                            (0x00000000U)

/* INTXBAR11_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G3_RESETVAL                            (0x00000000U)

/* INTXBAR11_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G4_RESETVAL                            (0x00000000U)

/* INTXBAR11_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G5_RESETVAL                            (0x00000000U)

/* INTXBAR11_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G6_RESETVAL                            (0x00000000U)

/* INTXBAR11_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G7_RESETVAL                            (0x00000000U)

/* INTXBAR11_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G8_RESETVAL                            (0x00000000U)

/* INTXBAR11_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR11_G9_RESETVAL                            (0x00000000U)

/* INTXBAR12_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G0_RESETVAL                            (0x00000000U)

/* INTXBAR12_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G1_RESETVAL                            (0x00000000U)

/* INTXBAR12_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G2_RESETVAL                            (0x00000000U)

/* INTXBAR12_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G3_RESETVAL                            (0x00000000U)

/* INTXBAR12_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G4_RESETVAL                            (0x00000000U)

/* INTXBAR12_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G5_RESETVAL                            (0x00000000U)

/* INTXBAR12_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G6_RESETVAL                            (0x00000000U)

/* INTXBAR12_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G7_RESETVAL                            (0x00000000U)

/* INTXBAR12_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G8_RESETVAL                            (0x00000000U)

/* INTXBAR12_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR12_G9_RESETVAL                            (0x00000000U)

/* INTXBAR13_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G0_RESETVAL                            (0x00000000U)

/* INTXBAR13_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G1_RESETVAL                            (0x00000000U)

/* INTXBAR13_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G2_RESETVAL                            (0x00000000U)

/* INTXBAR13_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G3_RESETVAL                            (0x00000000U)

/* INTXBAR13_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G4_RESETVAL                            (0x00000000U)

/* INTXBAR13_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G5_RESETVAL                            (0x00000000U)

/* INTXBAR13_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G6_RESETVAL                            (0x00000000U)

/* INTXBAR13_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G7_RESETVAL                            (0x00000000U)

/* INTXBAR13_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G8_RESETVAL                            (0x00000000U)

/* INTXBAR13_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR13_G9_RESETVAL                            (0x00000000U)

/* INTXBAR14_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G0_RESETVAL                            (0x00000000U)

/* INTXBAR14_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G1_RESETVAL                            (0x00000000U)

/* INTXBAR14_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G2_RESETVAL                            (0x00000000U)

/* INTXBAR14_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G3_RESETVAL                            (0x00000000U)

/* INTXBAR14_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G4_RESETVAL                            (0x00000000U)

/* INTXBAR14_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G5_RESETVAL                            (0x00000000U)

/* INTXBAR14_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G6_RESETVAL                            (0x00000000U)

/* INTXBAR14_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G7_RESETVAL                            (0x00000000U)

/* INTXBAR14_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G8_RESETVAL                            (0x00000000U)

/* INTXBAR14_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR14_G9_RESETVAL                            (0x00000000U)

/* INTXBAR15_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G0_RESETVAL                            (0x00000000U)

/* INTXBAR15_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G1_RESETVAL                            (0x00000000U)

/* INTXBAR15_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G2_RESETVAL                            (0x00000000U)

/* INTXBAR15_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G3_RESETVAL                            (0x00000000U)

/* INTXBAR15_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G4_RESETVAL                            (0x00000000U)

/* INTXBAR15_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G5_RESETVAL                            (0x00000000U)

/* INTXBAR15_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G6_RESETVAL                            (0x00000000U)

/* INTXBAR15_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G7_RESETVAL                            (0x00000000U)

/* INTXBAR15_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G8_RESETVAL                            (0x00000000U)

/* INTXBAR15_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR15_G9_RESETVAL                            (0x00000000U)

/* INTXBAR16_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G0_RESETVAL                            (0x00000000U)

/* INTXBAR16_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G1_RESETVAL                            (0x00000000U)

/* INTXBAR16_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G2_RESETVAL                            (0x00000000U)

/* INTXBAR16_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G3_RESETVAL                            (0x00000000U)

/* INTXBAR16_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G4_RESETVAL                            (0x00000000U)

/* INTXBAR16_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G5_RESETVAL                            (0x00000000U)

/* INTXBAR16_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G6_RESETVAL                            (0x00000000U)

/* INTXBAR16_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G7_RESETVAL                            (0x00000000U)

/* INTXBAR16_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G8_RESETVAL                            (0x00000000U)

/* INTXBAR16_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR16_G9_RESETVAL                            (0x00000000U)

/* INTXBAR17_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G0_RESETVAL                            (0x00000000U)

/* INTXBAR17_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G1_RESETVAL                            (0x00000000U)

/* INTXBAR17_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G2_RESETVAL                            (0x00000000U)

/* INTXBAR17_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G3_RESETVAL                            (0x00000000U)

/* INTXBAR17_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G4_RESETVAL                            (0x00000000U)

/* INTXBAR17_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G5_RESETVAL                            (0x00000000U)

/* INTXBAR17_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G6_RESETVAL                            (0x00000000U)

/* INTXBAR17_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G7_RESETVAL                            (0x00000000U)

/* INTXBAR17_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G8_RESETVAL                            (0x00000000U)

/* INTXBAR17_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR17_G9_RESETVAL                            (0x00000000U)

/* INTXBAR18_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G0_RESETVAL                            (0x00000000U)

/* INTXBAR18_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G1_RESETVAL                            (0x00000000U)

/* INTXBAR18_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G2_RESETVAL                            (0x00000000U)

/* INTXBAR18_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G3_RESETVAL                            (0x00000000U)

/* INTXBAR18_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G4_RESETVAL                            (0x00000000U)

/* INTXBAR18_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G5_RESETVAL                            (0x00000000U)

/* INTXBAR18_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G6_RESETVAL                            (0x00000000U)

/* INTXBAR18_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G7_RESETVAL                            (0x00000000U)

/* INTXBAR18_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G8_RESETVAL                            (0x00000000U)

/* INTXBAR18_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR18_G9_RESETVAL                            (0x00000000U)

/* INTXBAR19_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G0_RESETVAL                            (0x00000000U)

/* INTXBAR19_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G1_RESETVAL                            (0x00000000U)

/* INTXBAR19_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G2_RESETVAL                            (0x00000000U)

/* INTXBAR19_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G3_RESETVAL                            (0x00000000U)

/* INTXBAR19_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G4_RESETVAL                            (0x00000000U)

/* INTXBAR19_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G5_RESETVAL                            (0x00000000U)

/* INTXBAR19_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G6_RESETVAL                            (0x00000000U)

/* INTXBAR19_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G7_RESETVAL                            (0x00000000U)

/* INTXBAR19_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G8_RESETVAL                            (0x00000000U)

/* INTXBAR19_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR19_G9_RESETVAL                            (0x00000000U)

/* INTXBAR20_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G0_RESETVAL                            (0x00000000U)

/* INTXBAR20_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G1_RESETVAL                            (0x00000000U)

/* INTXBAR20_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G2_RESETVAL                            (0x00000000U)

/* INTXBAR20_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G3_RESETVAL                            (0x00000000U)

/* INTXBAR20_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G4_RESETVAL                            (0x00000000U)

/* INTXBAR20_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G5_RESETVAL                            (0x00000000U)

/* INTXBAR20_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G6_RESETVAL                            (0x00000000U)

/* INTXBAR20_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G7_RESETVAL                            (0x00000000U)

/* INTXBAR20_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G8_RESETVAL                            (0x00000000U)

/* INTXBAR20_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR20_G9_RESETVAL                            (0x00000000U)

/* INTXBAR21_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G0_RESETVAL                            (0x00000000U)

/* INTXBAR21_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G1_RESETVAL                            (0x00000000U)

/* INTXBAR21_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G2_RESETVAL                            (0x00000000U)

/* INTXBAR21_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G3_RESETVAL                            (0x00000000U)

/* INTXBAR21_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G4_RESETVAL                            (0x00000000U)

/* INTXBAR21_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G5_RESETVAL                            (0x00000000U)

/* INTXBAR21_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G6_RESETVAL                            (0x00000000U)

/* INTXBAR21_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G7_RESETVAL                            (0x00000000U)

/* INTXBAR21_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G8_RESETVAL                            (0x00000000U)

/* INTXBAR21_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR21_G9_RESETVAL                            (0x00000000U)

/* INTXBAR22_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G0_RESETVAL                            (0x00000000U)

/* INTXBAR22_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G1_RESETVAL                            (0x00000000U)

/* INTXBAR22_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G2_RESETVAL                            (0x00000000U)

/* INTXBAR22_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G3_RESETVAL                            (0x00000000U)

/* INTXBAR22_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G4_RESETVAL                            (0x00000000U)

/* INTXBAR22_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G5_RESETVAL                            (0x00000000U)

/* INTXBAR22_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G6_RESETVAL                            (0x00000000U)

/* INTXBAR22_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G7_RESETVAL                            (0x00000000U)

/* INTXBAR22_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G8_RESETVAL                            (0x00000000U)

/* INTXBAR22_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR22_G9_RESETVAL                            (0x00000000U)

/* INTXBAR23_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G0_RESETVAL                            (0x00000000U)

/* INTXBAR23_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G1_RESETVAL                            (0x00000000U)

/* INTXBAR23_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G2_RESETVAL                            (0x00000000U)

/* INTXBAR23_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G3_RESETVAL                            (0x00000000U)

/* INTXBAR23_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G4_RESETVAL                            (0x00000000U)

/* INTXBAR23_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G5_RESETVAL                            (0x00000000U)

/* INTXBAR23_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G6_RESETVAL                            (0x00000000U)

/* INTXBAR23_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G7_RESETVAL                            (0x00000000U)

/* INTXBAR23_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G8_RESETVAL                            (0x00000000U)

/* INTXBAR23_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR23_G9_RESETVAL                            (0x00000000U)

/* INTXBAR24_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G0_RESETVAL                            (0x00000000U)

/* INTXBAR24_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G1_RESETVAL                            (0x00000000U)

/* INTXBAR24_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G2_RESETVAL                            (0x00000000U)

/* INTXBAR24_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G3_RESETVAL                            (0x00000000U)

/* INTXBAR24_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G4_RESETVAL                            (0x00000000U)

/* INTXBAR24_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G5_RESETVAL                            (0x00000000U)

/* INTXBAR24_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G6_RESETVAL                            (0x00000000U)

/* INTXBAR24_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G7_RESETVAL                            (0x00000000U)

/* INTXBAR24_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G8_RESETVAL                            (0x00000000U)

/* INTXBAR24_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR24_G9_RESETVAL                            (0x00000000U)

/* INTXBAR25_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G0_RESETVAL                            (0x00000000U)

/* INTXBAR25_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G1_RESETVAL                            (0x00000000U)

/* INTXBAR25_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G2_RESETVAL                            (0x00000000U)

/* INTXBAR25_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G3_RESETVAL                            (0x00000000U)

/* INTXBAR25_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G4_RESETVAL                            (0x00000000U)

/* INTXBAR25_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G5_RESETVAL                            (0x00000000U)

/* INTXBAR25_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G6_RESETVAL                            (0x00000000U)

/* INTXBAR25_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G7_RESETVAL                            (0x00000000U)

/* INTXBAR25_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G8_RESETVAL                            (0x00000000U)

/* INTXBAR25_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR25_G9_RESETVAL                            (0x00000000U)

/* INTXBAR26_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G0_RESETVAL                            (0x00000000U)

/* INTXBAR26_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G1_RESETVAL                            (0x00000000U)

/* INTXBAR26_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G2_RESETVAL                            (0x00000000U)

/* INTXBAR26_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G3_RESETVAL                            (0x00000000U)

/* INTXBAR26_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G4_RESETVAL                            (0x00000000U)

/* INTXBAR26_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G5_RESETVAL                            (0x00000000U)

/* INTXBAR26_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G6_RESETVAL                            (0x00000000U)

/* INTXBAR26_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G7_RESETVAL                            (0x00000000U)

/* INTXBAR26_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G8_RESETVAL                            (0x00000000U)

/* INTXBAR26_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR26_G9_RESETVAL                            (0x00000000U)

/* INTXBAR27_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G0_RESETVAL                            (0x00000000U)

/* INTXBAR27_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G1_RESETVAL                            (0x00000000U)

/* INTXBAR27_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G2_RESETVAL                            (0x00000000U)

/* INTXBAR27_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G3_RESETVAL                            (0x00000000U)

/* INTXBAR27_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G4_RESETVAL                            (0x00000000U)

/* INTXBAR27_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G5_RESETVAL                            (0x00000000U)

/* INTXBAR27_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G6_RESETVAL                            (0x00000000U)

/* INTXBAR27_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G7_RESETVAL                            (0x00000000U)

/* INTXBAR27_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G8_RESETVAL                            (0x00000000U)

/* INTXBAR27_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR27_G9_RESETVAL                            (0x00000000U)

/* INTXBAR28_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G0_RESETVAL                            (0x00000000U)

/* INTXBAR28_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G1_RESETVAL                            (0x00000000U)

/* INTXBAR28_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G2_RESETVAL                            (0x00000000U)

/* INTXBAR28_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G3_RESETVAL                            (0x00000000U)

/* INTXBAR28_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G4_RESETVAL                            (0x00000000U)

/* INTXBAR28_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G5_RESETVAL                            (0x00000000U)

/* INTXBAR28_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G6_RESETVAL                            (0x00000000U)

/* INTXBAR28_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G7_RESETVAL                            (0x00000000U)

/* INTXBAR28_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G8_RESETVAL                            (0x00000000U)

/* INTXBAR28_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR28_G9_RESETVAL                            (0x00000000U)

/* INTXBAR29_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G0_RESETVAL                            (0x00000000U)

/* INTXBAR29_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G1_RESETVAL                            (0x00000000U)

/* INTXBAR29_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G2_RESETVAL                            (0x00000000U)

/* INTXBAR29_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G3_RESETVAL                            (0x00000000U)

/* INTXBAR29_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G4_RESETVAL                            (0x00000000U)

/* INTXBAR29_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G5_RESETVAL                            (0x00000000U)

/* INTXBAR29_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G6_RESETVAL                            (0x00000000U)

/* INTXBAR29_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G7_RESETVAL                            (0x00000000U)

/* INTXBAR29_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G8_RESETVAL                            (0x00000000U)

/* INTXBAR29_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR29_G9_RESETVAL                            (0x00000000U)

/* INTXBAR30_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G0_RESETVAL                            (0x00000000U)

/* INTXBAR30_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G1_RESETVAL                            (0x00000000U)

/* INTXBAR30_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G2_RESETVAL                            (0x00000000U)

/* INTXBAR30_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G3_RESETVAL                            (0x00000000U)

/* INTXBAR30_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G4_RESETVAL                            (0x00000000U)

/* INTXBAR30_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G5_RESETVAL                            (0x00000000U)

/* INTXBAR30_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G6_RESETVAL                            (0x00000000U)

/* INTXBAR30_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G7_RESETVAL                            (0x00000000U)

/* INTXBAR30_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G8_RESETVAL                            (0x00000000U)

/* INTXBAR30_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR30_G9_RESETVAL                            (0x00000000U)

/* INTXBAR31_G0 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G0_RESETVAL                            (0x00000000U)

/* INTXBAR31_G1 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G1_RESETVAL                            (0x00000000U)

/* INTXBAR31_G2 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_MASK                            (0xFFFFFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_SEL_MAX                             (0xFFFFFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G2_RESETVAL                            (0x00000000U)

/* INTXBAR31_G3 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G3_RESETVAL                            (0x00000000U)

/* INTXBAR31_G4 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G4_RESETVAL                            (0x00000000U)

/* INTXBAR31_G5 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_SEL_MASK                            (0x0000FFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_SEL_MAX                             (0x0000FFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G5_RESETVAL                            (0x00000000U)

/* INTXBAR31_G6 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G6_RESETVAL                            (0x00000000U)

/* INTXBAR31_G7 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_SEL_MASK                            (0x000003FFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_SEL_MAX                             (0x000003FFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G7_RESETVAL                            (0x00000000U)

/* INTXBAR31_G8 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G8_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G8_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G8_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G8_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G8_RESETVAL                            (0x00000000U)

/* INTXBAR31_G9 */

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G9_SEL_MASK                            (0x000FFFFFU)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G9_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G9_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G9_SEL_MAX                             (0x000FFFFFU)

#define CSL_CONTROLSS_INTXBAR_INTXBAR31_G9_RESETVAL                            (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
