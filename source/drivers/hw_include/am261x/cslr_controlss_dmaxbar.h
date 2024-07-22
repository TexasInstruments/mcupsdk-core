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
 *  Name        : cslr_controlss_dmaxbar.h
*/
#ifndef CSLR_CONTROLSS_DMAXBAR_H_
#define CSLR_CONTROLSS_DMAXBAR_H_

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
#define DMA_XBAR_EPWM0_SOCA         (0)
#define DMA_XBAR_EPWM1_SOCA         (1)
#define DMA_XBAR_EPWM2_SOCA         (2)
#define DMA_XBAR_EPWM3_SOCA         (3)
#define DMA_XBAR_EPWM4_SOCA         (4)
#define DMA_XBAR_EPWM5_SOCA         (5)
#define DMA_XBAR_EPWM6_SOCA         (6)
#define DMA_XBAR_EPWM7_SOCA         (7)
#define DMA_XBAR_EPWM8_SOCA         (8)
#define DMA_XBAR_EPWM9_SOCA         (9)

/******************* G1 *********************/
#define DMA_XBAR_EPWM0_SOCB         (0)
#define DMA_XBAR_EPWM1_SOCB         (1)
#define DMA_XBAR_EPWM2_SOCB         (2)
#define DMA_XBAR_EPWM3_SOCB         (3)
#define DMA_XBAR_EPWM4_SOCB         (4)
#define DMA_XBAR_EPWM5_SOCB         (5)
#define DMA_XBAR_EPWM6_SOCB         (6)
#define DMA_XBAR_EPWM7_SOCB         (7)
#define DMA_XBAR_EPWM8_SOCB         (8)
#define DMA_XBAR_EPWM9_SOCB         (9)

/******************* G2 *********************/
#define DMA_XBAR_ADC0_INT1          (0)
#define DMA_XBAR_ADC0_INT2          (1)
#define DMA_XBAR_ADC0_INT3          (2)
#define DMA_XBAR_ADC0_INT4          (3)
#define DMA_XBAR_ADC0_EVTINT        (4)
#define DMA_XBAR_ADC1_INT1          (5)
#define DMA_XBAR_ADC1_INT2          (6)
#define DMA_XBAR_ADC1_INT3          (7)
#define DMA_XBAR_ADC1_INT4          (8)
#define DMA_XBAR_ADC1_EVTINT        (9)
#define DMA_XBAR_ADC2_INT1          (10)
#define DMA_XBAR_ADC2_INT2          (11)
#define DMA_XBAR_ADC2_INT3          (12)
#define DMA_XBAR_ADC2_INT4          (13)
#define DMA_XBAR_ADC2_EVTINT        (14)

/******************* G3 *********************/
#define DMA_XBAR_FSI0_RX_DMA_EVT    (0)
#define DMA_XBAR_FSI0_DMA_TRIG1     (1)
#define DMA_XBAR_FSI0_DMA_TRIG2     (2)
#define DMA_XBAR_FSI0_TX_DMA_EVT    (16)

/******************* G4 *********************/
#define DMA_XBAR_SD0_FILT0_DRINT    (0)
#define DMA_XBAR_SD0_FILT1_DRINT    (1)
#define DMA_XBAR_SD0_FILT2_DRINT    (2)
#define DMA_XBAR_SD0_FILT3_DRINT    (3)
#define DMA_XBAR_SD1_FILT0_DRINT    (4)
#define DMA_XBAR_SD1_FILT1_DRINT    (5)
#define DMA_XBAR_SD1_FILT2_DRINT    (6)
#define DMA_XBAR_SD1_FILT3_DRINT    (7)

/******************* G5 *********************/
#define DMA_XBAR_ECAP0_DMA_INT    (0)
#define DMA_XBAR_ECAP1_DMA_INT    (1)
#define DMA_XBAR_ECAP2_DMA_INT    (2)
#define DMA_XBAR_ECAP3_DMA_INT    (3)
#define DMA_XBAR_ECAP4_DMA_INT    (4)
#define DMA_XBAR_ECAP5_DMA_INT    (5)
#define DMA_XBAR_ECAP6_DMA_INT    (6)
#define DMA_XBAR_ECAP7_DMA_INT    (7)

/**************************************************************************
    XBAR OUTPUT Macros
**************************************************************************/

#define DMA_XBAR_OUT0              0
#define DMA_XBAR_OUT1              1
#define DMA_XBAR_OUT2              2
#define DMA_XBAR_OUT3              3
#define DMA_XBAR_OUT4              4
#define DMA_XBAR_OUT5              5
#define DMA_XBAR_OUT6              6
#define DMA_XBAR_OUT7              7
#define DMA_XBAR_OUT8              8
#define DMA_XBAR_OUT9              9
#define DMA_XBAR_OUT10             10
#define DMA_XBAR_OUT11             11
#define DMA_XBAR_OUT12             12
#define DMA_XBAR_OUT13             13
#define DMA_XBAR_OUT14             14
#define DMA_XBAR_OUT15             15

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint8_t  Resv_256[252];
    volatile uint32_t DMAXBAR0_GSEL;
    volatile uint32_t DMAXBAR0_G0;
    volatile uint32_t DMAXBAR0_G1;
    volatile uint32_t DMAXBAR0_G2;
    volatile uint32_t DMAXBAR0_G3;
    volatile uint32_t DMAXBAR0_G4;
    volatile uint32_t DMAXBAR0_G5;
    volatile uint8_t  Resv_320[36];
    volatile uint32_t DMAXBAR1_GSEL;
    volatile uint32_t DMAXBAR1_G0;
    volatile uint32_t DMAXBAR1_G1;
    volatile uint32_t DMAXBAR1_G2;
    volatile uint32_t DMAXBAR1_G3;
    volatile uint32_t DMAXBAR1_G4;
    volatile uint32_t DMAXBAR1_G5;
    volatile uint8_t  Resv_384[36];
    volatile uint32_t DMAXBAR2_GSEL;
    volatile uint32_t DMAXBAR2_G0;
    volatile uint32_t DMAXBAR2_G1;
    volatile uint32_t DMAXBAR2_G2;
    volatile uint32_t DMAXBAR2_G3;
    volatile uint32_t DMAXBAR2_G4;
    volatile uint32_t DMAXBAR2_G5;
    volatile uint8_t  Resv_448[36];
    volatile uint32_t DMAXBAR3_GSEL;
    volatile uint32_t DMAXBAR3_G0;
    volatile uint32_t DMAXBAR3_G1;
    volatile uint32_t DMAXBAR3_G2;
    volatile uint32_t DMAXBAR3_G3;
    volatile uint32_t DMAXBAR3_G4;
    volatile uint32_t DMAXBAR3_G5;
    volatile uint8_t  Resv_512[36];
    volatile uint32_t DMAXBAR4_GSEL;
    volatile uint32_t DMAXBAR4_G0;
    volatile uint32_t DMAXBAR4_G1;
    volatile uint32_t DMAXBAR4_G2;
    volatile uint32_t DMAXBAR4_G3;
    volatile uint32_t DMAXBAR4_G4;
    volatile uint32_t DMAXBAR4_G5;
    volatile uint8_t  Resv_576[36];
    volatile uint32_t DMAXBAR5_GSEL;
    volatile uint32_t DMAXBAR5_G0;
    volatile uint32_t DMAXBAR5_G1;
    volatile uint32_t DMAXBAR5_G2;
    volatile uint32_t DMAXBAR5_G3;
    volatile uint32_t DMAXBAR5_G4;
    volatile uint32_t DMAXBAR5_G5;
    volatile uint8_t  Resv_640[36];
    volatile uint32_t DMAXBAR6_GSEL;
    volatile uint32_t DMAXBAR6_G0;
    volatile uint32_t DMAXBAR6_G1;
    volatile uint32_t DMAXBAR6_G2;
    volatile uint32_t DMAXBAR6_G3;
    volatile uint32_t DMAXBAR6_G4;
    volatile uint32_t DMAXBAR6_G5;
    volatile uint8_t  Resv_704[36];
    volatile uint32_t DMAXBAR7_GSEL;
    volatile uint32_t DMAXBAR7_G0;
    volatile uint32_t DMAXBAR7_G1;
    volatile uint32_t DMAXBAR7_G2;
    volatile uint32_t DMAXBAR7_G3;
    volatile uint32_t DMAXBAR7_G4;
    volatile uint32_t DMAXBAR7_G5;
    volatile uint8_t  Resv_768[36];
    volatile uint32_t DMAXBAR8_GSEL;
    volatile uint32_t DMAXBAR8_G0;
    volatile uint32_t DMAXBAR8_G1;
    volatile uint32_t DMAXBAR8_G2;
    volatile uint32_t DMAXBAR8_G3;
    volatile uint32_t DMAXBAR8_G4;
    volatile uint32_t DMAXBAR8_G5;
    volatile uint8_t  Resv_832[36];
    volatile uint32_t DMAXBAR9_GSEL;
    volatile uint32_t DMAXBAR9_G0;
    volatile uint32_t DMAXBAR9_G1;
    volatile uint32_t DMAXBAR9_G2;
    volatile uint32_t DMAXBAR9_G3;
    volatile uint32_t DMAXBAR9_G4;
    volatile uint32_t DMAXBAR9_G5;
    volatile uint8_t  Resv_896[36];
    volatile uint32_t DMAXBAR10_GSEL;
    volatile uint32_t DMAXBAR10_G0;
    volatile uint32_t DMAXBAR10_G1;
    volatile uint32_t DMAXBAR10_G2;
    volatile uint32_t DMAXBAR10_G3;
    volatile uint32_t DMAXBAR10_G4;
    volatile uint32_t DMAXBAR10_G5;
    volatile uint8_t  Resv_960[36];
    volatile uint32_t DMAXBAR11_GSEL;
    volatile uint32_t DMAXBAR11_G0;
    volatile uint32_t DMAXBAR11_G1;
    volatile uint32_t DMAXBAR11_G2;
    volatile uint32_t DMAXBAR11_G3;
    volatile uint32_t DMAXBAR11_G4;
    volatile uint32_t DMAXBAR11_G5;
    volatile uint8_t  Resv_1024[36];
    volatile uint32_t DMAXBAR12_GSEL;
    volatile uint32_t DMAXBAR12_G0;
    volatile uint32_t DMAXBAR12_G1;
    volatile uint32_t DMAXBAR12_G2;
    volatile uint32_t DMAXBAR12_G3;
    volatile uint32_t DMAXBAR12_G4;
    volatile uint32_t DMAXBAR12_G5;
    volatile uint8_t  Resv_1088[36];
    volatile uint32_t DMAXBAR13_GSEL;
    volatile uint32_t DMAXBAR13_G0;
    volatile uint32_t DMAXBAR13_G1;
    volatile uint32_t DMAXBAR13_G2;
    volatile uint32_t DMAXBAR13_G3;
    volatile uint32_t DMAXBAR13_G4;
    volatile uint32_t DMAXBAR13_G5;
    volatile uint8_t  Resv_1152[36];
    volatile uint32_t DMAXBAR14_GSEL;
    volatile uint32_t DMAXBAR14_G0;
    volatile uint32_t DMAXBAR14_G1;
    volatile uint32_t DMAXBAR14_G2;
    volatile uint32_t DMAXBAR14_G3;
    volatile uint32_t DMAXBAR14_G4;
    volatile uint32_t DMAXBAR14_G5;
    volatile uint8_t  Resv_1216[36];
    volatile uint32_t DMAXBAR15_GSEL;
    volatile uint32_t DMAXBAR15_G0;
    volatile uint32_t DMAXBAR15_G1;
    volatile uint32_t DMAXBAR15_G2;
    volatile uint32_t DMAXBAR15_G3;
    volatile uint32_t DMAXBAR15_G4;
    volatile uint32_t DMAXBAR15_G5;
} CSL_controlss_dmaxbarRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CONTROLSS_DMAXBAR_PID                                              (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL                                    (0x00000100U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0                                      (0x00000104U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1                                      (0x00000108U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2                                      (0x0000010CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3                                      (0x00000110U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4                                      (0x00000114U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5                                      (0x00000118U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_GSEL                                    (0x00000140U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G0                                      (0x00000144U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G1                                      (0x00000148U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G2                                      (0x0000014CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G3                                      (0x00000150U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G4                                      (0x00000154U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G5                                      (0x00000158U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_GSEL                                    (0x00000180U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G0                                      (0x00000184U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G1                                      (0x00000188U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G2                                      (0x0000018CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G3                                      (0x00000190U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G4                                      (0x00000194U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G5                                      (0x00000198U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_GSEL                                    (0x000001C0U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G0                                      (0x000001C4U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G1                                      (0x000001C8U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G2                                      (0x000001CCU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G3                                      (0x000001D0U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G4                                      (0x000001D4U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G5                                      (0x000001D8U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_GSEL                                    (0x00000200U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G0                                      (0x00000204U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G1                                      (0x00000208U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G2                                      (0x0000020CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G3                                      (0x00000210U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G4                                      (0x00000214U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G5                                      (0x00000218U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_GSEL                                    (0x00000240U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G0                                      (0x00000244U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G1                                      (0x00000248U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G2                                      (0x0000024CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G3                                      (0x00000250U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G4                                      (0x00000254U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G5                                      (0x00000258U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_GSEL                                    (0x00000280U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G0                                      (0x00000284U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G1                                      (0x00000288U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G2                                      (0x0000028CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G3                                      (0x00000290U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G4                                      (0x00000294U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G5                                      (0x00000298U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_GSEL                                    (0x000002C0U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G0                                      (0x000002C4U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G1                                      (0x000002C8U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G2                                      (0x000002CCU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G3                                      (0x000002D0U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G4                                      (0x000002D4U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G5                                      (0x000002D8U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_GSEL                                    (0x00000300U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G0                                      (0x00000304U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G1                                      (0x00000308U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G2                                      (0x0000030CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G3                                      (0x00000310U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G4                                      (0x00000314U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G5                                      (0x00000318U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_GSEL                                    (0x00000340U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G0                                      (0x00000344U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G1                                      (0x00000348U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G2                                      (0x0000034CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G3                                      (0x00000350U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G4                                      (0x00000354U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G5                                      (0x00000358U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_GSEL                                   (0x00000380U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G0                                     (0x00000384U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G1                                     (0x00000388U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G2                                     (0x0000038CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G3                                     (0x00000390U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G4                                     (0x00000394U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G5                                     (0x00000398U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_GSEL                                   (0x000003C0U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G0                                     (0x000003C4U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G1                                     (0x000003C8U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G2                                     (0x000003CCU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G3                                     (0x000003D0U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G4                                     (0x000003D4U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G5                                     (0x000003D8U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_GSEL                                   (0x00000400U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G0                                     (0x00000404U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G1                                     (0x00000408U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G2                                     (0x0000040CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G3                                     (0x00000410U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G4                                     (0x00000414U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G5                                     (0x00000418U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_GSEL                                   (0x00000440U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G0                                     (0x00000444U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G1                                     (0x00000448U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G2                                     (0x0000044CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G3                                     (0x00000450U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G4                                     (0x00000454U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G5                                     (0x00000458U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_GSEL                                   (0x00000480U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G0                                     (0x00000484U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G1                                     (0x00000488U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G2                                     (0x0000048CU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G3                                     (0x00000490U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G4                                     (0x00000494U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G5                                     (0x00000498U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_GSEL                                   (0x000004C0U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G0                                     (0x000004C4U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G1                                     (0x000004C8U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G2                                     (0x000004CCU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G3                                     (0x000004D0U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G4                                     (0x000004D4U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G5                                     (0x000004D8U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_CONTROLSS_DMAXBAR_PID_PID_MINOR_MASK                               (0x0000003FU)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MINOR_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MINOR_RESETVAL                           (0x00000014U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MINOR_MAX                                (0x0000003FU)

#define CSL_CONTROLSS_DMAXBAR_PID_PID_CUSTOM_MASK                              (0x000000C0U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_CUSTOM_SHIFT                             (0x00000006U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_CUSTOM_RESETVAL                          (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_CUSTOM_MAX                               (0x00000003U)

#define CSL_CONTROLSS_DMAXBAR_PID_PID_MAJOR_MASK                               (0x00000700U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MAJOR_SHIFT                              (0x00000008U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MAJOR_RESETVAL                           (0x00000002U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MAJOR_MAX                                (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_PID_PID_MISC_MASK                                (0x0000F800U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MISC_SHIFT                               (0x0000000BU)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MISC_RESETVAL                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MISC_MAX                                 (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_PID_PID_MSB16_MASK                               (0xFFFF0000U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MSB16_SHIFT                              (0x00000010U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MSB16_RESETVAL                           (0x00006180U)
#define CSL_CONTROLSS_DMAXBAR_PID_PID_MSB16_MAX                                (0x0000FFFFU)

#define CSL_CONTROLSS_DMAXBAR_PID_RESETVAL                                     (0x61800214U)

/* DMAXBAR0_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR0_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR0_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR0_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR0_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR0_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR0_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR0_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR1_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR1_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR1_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR1_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR1_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR1_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR1_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR1_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR2_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR2_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR2_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR2_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR2_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR2_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR2_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR2_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR3_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR3_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR3_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR3_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR3_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR3_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR3_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR3_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR4_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR4_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR4_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR4_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR4_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR4_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR4_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR4_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR5_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR5_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR5_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR5_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR5_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR5_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR5_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR5_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR6_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR6_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR6_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR6_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR6_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR6_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR6_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR6_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR7_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR7_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR7_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR7_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR7_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR7_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR7_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR7_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR8_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR8_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR8_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR8_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR8_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR8_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR8_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR8_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR9_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_GSEL_GSEL_MASK                          (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_GSEL_GSEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_GSEL_GSEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_GSEL_GSEL_MAX                           (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_GSEL_RESETVAL                           (0x00000000U)

/* DMAXBAR9_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G0_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G0_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G0_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G0_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G0_RESETVAL                             (0x00000000U)

/* DMAXBAR9_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G1_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G1_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G1_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G1_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G1_RESETVAL                             (0x00000000U)

/* DMAXBAR9_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G2_SEL_MASK                             (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G2_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G2_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G2_SEL_MAX                              (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G2_RESETVAL                             (0x00000000U)

/* DMAXBAR9_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G3_SEL_MASK                             (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G3_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G3_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G3_SEL_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G3_RESETVAL                             (0x00000000U)

/* DMAXBAR9_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G4_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G4_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G4_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G4_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G4_RESETVAL                             (0x00000000U)

/* DMAXBAR9_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G5_SEL_MASK                             (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G5_SEL_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G5_SEL_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G5_SEL_MAX                              (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR9_G5_RESETVAL                             (0x00000000U)

/* DMAXBAR10_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_GSEL_GSEL_MASK                         (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_GSEL_GSEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_GSEL_GSEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_GSEL_GSEL_MAX                          (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_GSEL_RESETVAL                          (0x00000000U)

/* DMAXBAR10_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G0_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G0_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G0_RESETVAL                            (0x00000000U)

/* DMAXBAR10_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G1_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G1_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G1_RESETVAL                            (0x00000000U)

/* DMAXBAR10_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G2_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G2_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G2_RESETVAL                            (0x00000000U)

/* DMAXBAR10_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G3_SEL_MASK                            (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G3_SEL_MAX                             (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G3_RESETVAL                            (0x00000000U)

/* DMAXBAR10_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G4_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G4_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G4_RESETVAL                            (0x00000000U)

/* DMAXBAR10_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G5_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G5_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR10_G5_RESETVAL                            (0x00000000U)

/* DMAXBAR11_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_GSEL_GSEL_MASK                         (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_GSEL_GSEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_GSEL_GSEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_GSEL_GSEL_MAX                          (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_GSEL_RESETVAL                          (0x00000000U)

/* DMAXBAR11_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G0_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G0_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G0_RESETVAL                            (0x00000000U)

/* DMAXBAR11_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G1_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G1_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G1_RESETVAL                            (0x00000000U)

/* DMAXBAR11_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G2_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G2_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G2_RESETVAL                            (0x00000000U)

/* DMAXBAR11_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G3_SEL_MASK                            (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G3_SEL_MAX                             (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G3_RESETVAL                            (0x00000000U)

/* DMAXBAR11_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G4_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G4_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G4_RESETVAL                            (0x00000000U)

/* DMAXBAR11_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G5_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G5_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR11_G5_RESETVAL                            (0x00000000U)

/* DMAXBAR12_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_GSEL_GSEL_MASK                         (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_GSEL_GSEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_GSEL_GSEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_GSEL_GSEL_MAX                          (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_GSEL_RESETVAL                          (0x00000000U)

/* DMAXBAR12_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G0_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G0_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G0_RESETVAL                            (0x00000000U)

/* DMAXBAR12_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G1_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G1_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G1_RESETVAL                            (0x00000000U)

/* DMAXBAR12_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G2_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G2_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G2_RESETVAL                            (0x00000000U)

/* DMAXBAR12_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G3_SEL_MASK                            (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G3_SEL_MAX                             (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G3_RESETVAL                            (0x00000000U)

/* DMAXBAR12_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G4_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G4_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G4_RESETVAL                            (0x00000000U)

/* DMAXBAR12_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G5_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G5_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR12_G5_RESETVAL                            (0x00000000U)

/* DMAXBAR13_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_GSEL_GSEL_MASK                         (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_GSEL_GSEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_GSEL_GSEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_GSEL_GSEL_MAX                          (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_GSEL_RESETVAL                          (0x00000000U)

/* DMAXBAR13_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G0_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G0_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G0_RESETVAL                            (0x00000000U)

/* DMAXBAR13_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G1_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G1_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G1_RESETVAL                            (0x00000000U)

/* DMAXBAR13_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G2_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G2_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G2_RESETVAL                            (0x00000000U)

/* DMAXBAR13_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G3_SEL_MASK                            (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G3_SEL_MAX                             (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G3_RESETVAL                            (0x00000000U)

/* DMAXBAR13_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G4_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G4_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G4_RESETVAL                            (0x00000000U)

/* DMAXBAR13_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G5_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G5_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR13_G5_RESETVAL                            (0x00000000U)

/* DMAXBAR14_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_GSEL_GSEL_MASK                         (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_GSEL_GSEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_GSEL_GSEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_GSEL_GSEL_MAX                          (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_GSEL_RESETVAL                          (0x00000000U)

/* DMAXBAR14_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G0_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G0_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G0_RESETVAL                            (0x00000000U)

/* DMAXBAR14_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G1_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G1_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G1_RESETVAL                            (0x00000000U)

/* DMAXBAR14_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G2_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G2_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G2_RESETVAL                            (0x00000000U)

/* DMAXBAR14_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G3_SEL_MASK                            (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G3_SEL_MAX                             (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G3_RESETVAL                            (0x00000000U)

/* DMAXBAR14_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G4_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G4_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G4_RESETVAL                            (0x00000000U)

/* DMAXBAR14_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G5_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G5_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR14_G5_RESETVAL                            (0x00000000U)

/* DMAXBAR15_GSEL */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_GSEL_GSEL_MASK                         (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_GSEL_GSEL_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_GSEL_GSEL_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_GSEL_GSEL_MAX                          (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_GSEL_RESETVAL                          (0x00000000U)

/* DMAXBAR15_G0 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G0_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G0_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G0_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G0_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G0_RESETVAL                            (0x00000000U)

/* DMAXBAR15_G1 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G1_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G1_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G1_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G1_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G1_RESETVAL                            (0x00000000U)

/* DMAXBAR15_G2 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G2_SEL_MASK                            (0x0000000FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G2_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G2_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G2_SEL_MAX                             (0x0000000FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G2_RESETVAL                            (0x00000000U)

/* DMAXBAR15_G3 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G3_SEL_MASK                            (0x0000001FU)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G3_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G3_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G3_SEL_MAX                             (0x0000001FU)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G3_RESETVAL                            (0x00000000U)

/* DMAXBAR15_G4 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G4_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G4_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G4_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G4_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G4_RESETVAL                            (0x00000000U)

/* DMAXBAR15_G5 */

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G5_SEL_MASK                            (0x00000007U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G5_SEL_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G5_SEL_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G5_SEL_MAX                             (0x00000007U)

#define CSL_CONTROLSS_DMAXBAR_DMAXBAR15_G5_RESETVAL                            (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
