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
 *  Name        : cslr_controlss_ctrl.h
*/
#ifndef CSLR_CONTROLSS_CTRL_H_
#define CSLR_CONTROLSS_CTRL_H_

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
	volatile uint32_t	PID;
	volatile uint32_t	ADCEXTCHXBAR0_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR1_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR2_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR3_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR4_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR5_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR6_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR7_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR8_G0_SEL;
	volatile uint32_t	ADCEXTCHXBAR9_G0_SEL;
	volatile uint8_t	Resv_68[24];
	volatile uint32_t	ADCSOCFRCGBSEL;
	volatile uint32_t	ADCSOCFRCGB;
	volatile uint32_t	ADC_EXTCH_DLY_SEL;
	volatile uint8_t	Resv_120[40];
	volatile uint32_t	SDFM0_CLK0_OUT_SEL;
	volatile uint32_t	SDFM0_CLK1_OUT_SEL;
	volatile uint32_t	SDFM0_CLK2_OUT_SEL;
	volatile uint32_t	SDFM0_CLK3_OUT_SEL;
	volatile uint32_t	SDFM1_CLK0_OUT_SEL;
	volatile uint32_t	SDFM1_CLK1_OUT_SEL;
	volatile uint32_t	SDFM1_CLK2_OUT_SEL;
	volatile uint32_t	SDFM1_CLK3_OUT_SEL;
	volatile uint8_t	Resv_252[100];
	volatile uint32_t	SDFM1_CLK0_SEL;
	volatile uint8_t	Resv_296[40];
    volatile uint32_t   CONTROLSS_G0_EPWM_WLINK;
    volatile uint32_t   CONTROLSS_G1_EPWM_WLINK;
	volatile uint8_t	Resv_312[8];
	volatile uint32_t	EPWM_STATICXBAR_SEL0;
	volatile uint8_t	Resv_328[12];
	volatile uint32_t	EPWM_CLKSYNC;
	volatile uint8_t	Resv_336[4];
	volatile uint32_t	EPWM_SOCA_SEL;
	volatile uint8_t	Resv_344[4];
	volatile uint32_t	EPWM_SOCB_SEL;
	volatile uint8_t	Resv_352[4];
	volatile uint32_t	EMUSTOPN_MASK;
	volatile uint32_t	CLB_AQ_EN0;
	volatile uint8_t	Resv_372[12];
	volatile uint32_t	CLB_DB_EN0;
	volatile uint8_t	Resv_428[52];
	volatile uint32_t	XBAR_LOOPBACK_CTRL;
	volatile uint8_t	Resv_512[80];
	volatile uint32_t	ETPWM0_CLK_GATE;
	volatile uint32_t	ETPWM1_CLK_GATE;
	volatile uint32_t	ETPWM2_CLK_GATE;
	volatile uint32_t	ETPWM3_CLK_GATE;
	volatile uint32_t	ETPWM4_CLK_GATE;
	volatile uint32_t	ETPWM5_CLK_GATE;
	volatile uint32_t	ETPWM6_CLK_GATE;
	volatile uint32_t	ETPWM7_CLK_GATE;
	volatile uint32_t	ETPWM8_CLK_GATE;
	volatile uint32_t	ETPWM9_CLK_GATE;
	volatile uint8_t	Resv_768[216];
	volatile uint32_t	ECAP0_CLK_GATE;
	volatile uint32_t	ECAP1_CLK_GATE;
	volatile uint32_t	ECAP2_CLK_GATE;
	volatile uint32_t	ECAP3_CLK_GATE;
	volatile uint32_t	ECAP4_CLK_GATE;
	volatile uint32_t	ECAP5_CLK_GATE;
	volatile uint32_t	ECAP6_CLK_GATE;
	volatile uint32_t	ECAP7_CLK_GATE;
	volatile uint8_t	Resv_1024[224];
	volatile uint32_t	CMPSSA0_CLK_GATE;
	volatile uint32_t	CMPSSA1_CLK_GATE;
	volatile uint32_t	CMPSSA2_CLK_GATE;
	volatile uint32_t	CMPSSA3_CLK_GATE;
	volatile uint32_t	CMPSSA4_CLK_GATE;
	volatile uint32_t	CMPSSA5_CLK_GATE;
	volatile uint32_t	CMPSSA6_CLK_GATE;
	volatile uint32_t	CMPSSA7_CLK_GATE;
	volatile uint32_t	CMPSSA8_CLK_GATE;
	volatile uint8_t	Resv_1152[92];
	volatile uint32_t	ADC_SCTILE0_CLK_GATE;
	volatile uint32_t	ADC_SCTILE1_CLK_GATE;
	volatile uint32_t	ADC_SCTILE2_CLK_GATE;
	volatile uint32_t	ADC_SCTILE3_CLK_GATE;
	volatile uint32_t	ADC_SCTILE4_CLK_GATE;
	volatile uint32_t	ADC_SCTILE5_CLK_GATE;
	volatile uint8_t	Resv_1344[168];
	volatile uint32_t	ADC0_CLK_GATE;
	volatile uint32_t	ADC1_CLK_GATE;
	volatile uint32_t	ADC2_CLK_GATE;
	volatile uint8_t	Resv_1440[84];
	volatile uint32_t	EQEP0_CLK_GATE;
	volatile uint32_t	EQEP1_CLK_GATE;
	volatile uint8_t	Resv_1472[24];
	volatile uint32_t	SDFM0_CLK_GATE;
	volatile uint32_t	SDFM1_CLK_GATE;
	volatile uint8_t	Resv_1504[24];
	volatile uint32_t	OTTO0_CLK_GATE;
	volatile uint32_t	OTTO1_CLK_GATE;
	volatile uint8_t	Resv_1536[24];
	volatile uint32_t	FSI_TX0_CLK_GATE;
	volatile uint8_t	Resv_1568[28];
	volatile uint32_t	FSI_RX0_CLK_GATE;
	volatile uint8_t	Resv_1664[92];
	volatile uint32_t	ADC_AGG0_CLK_GATE;
	volatile uint8_t	Resv_1680[12];
	volatile uint32_t	DAC_CLK_GATE;
	volatile uint8_t	Resv_1776[92];
	volatile uint32_t	FSI_TX0_PLL_CLK_GATE;
	volatile uint8_t	Resv_1808[28];
	volatile uint32_t	SDFM0_PLL_CLK_GATE;
	volatile uint32_t	SDFM1_PLL_CLK_GATE;
	volatile uint8_t	Resv_1872[56];
    volatile uint32_t 	CONTROLSS_XBAR_CLK_GATE;
	volatile uint8_t	Resv_2048[172];
	volatile uint32_t	ETPWM0_RST;
	volatile uint32_t	ETPWM1_RST;
	volatile uint32_t	ETPWM2_RST;
	volatile uint32_t	ETPWM3_RST;
	volatile uint32_t	ETPWM4_RST;
	volatile uint32_t	ETPWM5_RST;
	volatile uint32_t	ETPWM6_RST;
	volatile uint32_t	ETPWM7_RST;
	volatile uint32_t	ETPWM8_RST;
	volatile uint32_t	ETPWM9_RST;
	volatile uint8_t	Resv_2304[216];
	volatile uint32_t	ECAP0_RST;
	volatile uint32_t	ECAP1_RST;
	volatile uint32_t	ECAP2_RST;
	volatile uint32_t	ECAP3_RST;
	volatile uint32_t	ECAP4_RST;
	volatile uint32_t	ECAP5_RST;
	volatile uint32_t	ECAP6_RST;
	volatile uint32_t	ECAP7_RST;
	volatile uint8_t	Resv_2560[224];
	volatile uint32_t	CMPSSA0_RST;
	volatile uint32_t	CMPSSA1_RST;
	volatile uint32_t	CMPSSA2_RST;
	volatile uint32_t	CMPSSA3_RST;
	volatile uint32_t	CMPSSA4_RST;
	volatile uint32_t	CMPSSA5_RST;
	volatile uint32_t	CMPSSA6_RST;
	volatile uint32_t	CMPSSA7_RST;
	volatile uint32_t	CMPSSA8_RST;
	volatile uint8_t	Resv_2688[92];
	volatile uint32_t	ADC_SCTILE0_RST;
	volatile uint32_t	ADC_SCTILE1_RST;
	volatile uint32_t	ADC_SCTILE2_RST;
	volatile uint32_t	ADC_SCTILE3_RST;
	volatile uint32_t	ADC_SCTILE4_RST;
	volatile uint32_t	ADC_SCTILE5_RST;
	volatile uint8_t	Resv_2880[168];
	volatile uint32_t	ADC0_RST;
	volatile uint32_t	ADC1_RST;
	volatile uint32_t	ADC2_RST;
	volatile uint8_t	Resv_2976[84];
	volatile uint32_t	EQEP0_RST;
	volatile uint32_t	EQEP1_RST;
	volatile uint8_t	Resv_3008[24];
	volatile uint32_t	SDFM0_RST;
	volatile uint32_t	SDFM1_RST;
	volatile uint8_t	Resv_3040[24];
	volatile uint32_t	OTTO0_RST;
	volatile uint32_t	OTTO1_RST;
	volatile uint8_t	Resv_3072[24];
	volatile uint32_t	FSI_TX0_RST;
	volatile uint8_t	Resv_3104[28];
	volatile uint32_t	FSI_RX0_RST;
	volatile uint8_t	Resv_3200[92];
	volatile uint32_t	ADC_AGG0_RST;
	volatile uint8_t	Resv_3216[12];
	volatile uint32_t	DAC_RST;
	volatile uint8_t	Resv_3328[108];
	volatile uint32_t	EPWM0_HALTEN;
	volatile uint32_t	EPWM1_HALTEN;
	volatile uint32_t	EPWM2_HALTEN;
	volatile uint32_t	EPWM3_HALTEN;
	volatile uint32_t	EPWM4_HALTEN;
	volatile uint32_t	EPWM5_HALTEN;
	volatile uint32_t	EPWM6_HALTEN;
	volatile uint32_t	EPWM7_HALTEN;
	volatile uint32_t	EPWM8_HALTEN;
	volatile uint32_t	EPWM9_HALTEN;
	volatile uint8_t	Resv_3584[216];
	volatile uint32_t	CMPSSA0_HALTEN;
	volatile uint32_t	CMPSSA1_HALTEN;
	volatile uint32_t	CMPSSA2_HALTEN;
	volatile uint32_t	CMPSSA3_HALTEN;
	volatile uint32_t	CMPSSA4_HALTEN;
	volatile uint32_t	CMPSSA5_HALTEN;
	volatile uint32_t	CMPSSA6_HALTEN;
	volatile uint32_t	CMPSSA7_HALTEN;
	volatile uint32_t	CMPSSA8_HALTEN;
	volatile uint8_t	Resv_3712[92];
	volatile uint32_t	ECAP0_HALTEN;
	volatile uint32_t	ECAP1_HALTEN;
	volatile uint32_t	ECAP2_HALTEN;
	volatile uint32_t	ECAP3_HALTEN;
	volatile uint32_t	ECAP4_HALTEN;
	volatile uint32_t	ECAP5_HALTEN;
	volatile uint32_t	ECAP6_HALTEN;
	volatile uint32_t	ECAP7_HALTEN;
	volatile uint8_t	Resv_3904[160];
	volatile uint32_t	EQEP0_HALTEN;
	volatile uint32_t	EQEP1_HALTEN;
	volatile uint8_t	Resv_4104[192];
    volatile uint32_t 	LOCK0_KICK0;               /*  - KICK0 component */
    volatile uint32_t 	LOCK0_KICK1;               /*  - KICK1 component */
    volatile uint32_t 	INTR_RAW_STATUS;           /* Interrupt Raw Status/Set Register */
    volatile uint32_t 	INTR_ENABLED_STATUS_CLEAR;   /* Interrupt Enabled Status/Clear register */
    volatile uint32_t 	INTR_ENABLE;               /* Interrupt Enable register */
    volatile uint32_t 	INTR_ENABLE_CLEAR;         /* Interrupt Enable Clear register */
    volatile uint32_t 	EOI;                       /* EOI register */
    volatile uint32_t 	FAULT_ADDRESS;             /* Fault Address register */
    volatile uint32_t 	FAULT_TYPE_STATUS;         /* Fault Type Status register */
    volatile uint32_t 	FAULT_ATTR_STATUS;         /* Fault Attribute Status register */
    volatile uint32_t 	FAULT_CLEAR;               /* Fault Clear register */
} CSL_controlss_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/*--------CONTROLSS_CTRL_--------*/
#define CSL_CONTROLSS_CTRL_PID                                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL                                 (0x00000004U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL                                 (0x00000008U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL                                 (0x0000000CU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL                                 (0x00000010U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL                                 (0x00000014U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL                                 (0x00000018U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL                                 (0x0000001CU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL                                 (0x00000020U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL                                 (0x00000024U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL                                 (0x00000028U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL                                       (0x00000044U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB                                          (0x00000048U)
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL                                    (0x0000004CU)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL                                   (0x00000078U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL                                   (0x0000007CU)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL                                   (0x00000080U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL                                   (0x00000084U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL                                   (0x00000088U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL                                   (0x0000008CU)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL                                   (0x00000090U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL                                   (0x00000094U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL                                       (0x000000FCU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK                       		(0x00000128U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK                       		(0x0000012CU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0                                 (0x00000138U)
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC                                         (0x00000148U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL                                        (0x00000150U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL                                        (0x00000158U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK                                        (0x00000160U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0                                           (0x00000164U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0                                           (0x00000174U)
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL                                   (0x000001ACU)
#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE                                      (0x00000200U)
#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE                                      (0x00000204U)
#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE                                      (0x00000208U)
#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE                                      (0x0000020CU)
#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE                                      (0x00000210U)
#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE                                      (0x00000214U)
#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE                                      (0x00000218U)
#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE                                      (0x0000021CU)
#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE                                      (0x00000220U)
#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE                                      (0x00000224U)
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE                                       (0x00000300U)
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE                                       (0x00000304U)
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE                                       (0x00000308U)
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE                                       (0x0000030CU)
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE                                       (0x00000310U)
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE                                       (0x00000314U)
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE                                       (0x00000318U)
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE                                       (0x0000031CU)
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE                                   (0x00000400U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE                                   (0x00000404U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE                                   (0x00000408U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE                                   (0x0000040CU)
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE                                   (0x00000410U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE                                   (0x00000414U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE                                   (0x00000418U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE                                   (0x0000041CU)
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE                                   (0x00000420U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE                                 (0x00000480U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE                                 (0x00000484U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE                                 (0x00000488U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_CLK_GATE                                 (0x0000048CU)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE                                 (0x00000490U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE                                 (0x00000494U)
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE                                        (0x00000540U)
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE                                        (0x00000544U)
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE                                        (0x00000548U)
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE                                       (0x000005A0U)
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE                                       (0x000005A4U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE                                       (0x000005C0U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE                                       (0x000005C4U)
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE                                       (0x000005E0U)
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE                                       (0x000005E4U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE                                     (0x00000600U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE                                     (0x00000620U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE                                    (0x00000680U)
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE                                         (0x00000690U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE                                 (0x000006F0U)
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE                                   (0x00000710U)
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE                                   (0x00000714U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE                       		(0x00000750U)
#define CSL_CONTROLSS_CTRL_ETPWM0_RST                                           (0x00000800U)
#define CSL_CONTROLSS_CTRL_ETPWM1_RST                                           (0x00000804U)
#define CSL_CONTROLSS_CTRL_ETPWM2_RST                                           (0x00000808U)
#define CSL_CONTROLSS_CTRL_ETPWM3_RST                                           (0x0000080CU)
#define CSL_CONTROLSS_CTRL_ETPWM4_RST                                           (0x00000810U)
#define CSL_CONTROLSS_CTRL_ETPWM5_RST                                           (0x00000814U)
#define CSL_CONTROLSS_CTRL_ETPWM6_RST                                           (0x00000818U)
#define CSL_CONTROLSS_CTRL_ETPWM7_RST                                           (0x0000081CU)
#define CSL_CONTROLSS_CTRL_ETPWM8_RST                                           (0x00000820U)
#define CSL_CONTROLSS_CTRL_ETPWM9_RST                                           (0x00000824U)
#define CSL_CONTROLSS_CTRL_ECAP0_RST                                            (0x00000900U)
#define CSL_CONTROLSS_CTRL_ECAP1_RST                                            (0x00000904U)
#define CSL_CONTROLSS_CTRL_ECAP2_RST                                            (0x00000908U)
#define CSL_CONTROLSS_CTRL_ECAP3_RST                                            (0x0000090CU)
#define CSL_CONTROLSS_CTRL_ECAP4_RST                                            (0x00000910U)
#define CSL_CONTROLSS_CTRL_ECAP5_RST                                            (0x00000914U)
#define CSL_CONTROLSS_CTRL_ECAP6_RST                                            (0x00000918U)
#define CSL_CONTROLSS_CTRL_ECAP7_RST                                            (0x0000091CU)
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST                                        (0x00000A00U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST                                        (0x00000A04U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST                                        (0x00000A08U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST                                        (0x00000A0CU)
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST                                        (0x00000A10U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST                                        (0x00000A14U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST                                        (0x00000A18U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST                                        (0x00000A1CU)
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST                                        (0x00000A20U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST                                      (0x00000A80U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST                                      (0x00000A84U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST                                      (0x00000A88U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST                                      (0x00000A8CU)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST                                      (0x00000A90U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST                                      (0x00000A94U)
#define CSL_CONTROLSS_CTRL_ADC0_RST                                             (0x00000B40U)
#define CSL_CONTROLSS_CTRL_ADC1_RST                                             (0x00000B44U)
#define CSL_CONTROLSS_CTRL_ADC2_RST                                             (0x00000B48U)
#define CSL_CONTROLSS_CTRL_EQEP0_RST                                            (0x00000BA0U)
#define CSL_CONTROLSS_CTRL_EQEP1_RST                                            (0x00000BA4U)
#define CSL_CONTROLSS_CTRL_SDFM0_RST                                            (0x00000BC0U)
#define CSL_CONTROLSS_CTRL_SDFM1_RST                                            (0x00000BC4U)
#define CSL_CONTROLSS_CTRL_OTTO0_RST                                            (0x00000BE0U)
#define CSL_CONTROLSS_CTRL_OTTO1_RST                                            (0x00000BE4U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST                                          (0x00000C00U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST                                          (0x00000C20U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST                                         (0x00000C80U)
#define CSL_CONTROLSS_CTRL_DAC_RST                                              (0x00000C90U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN                                         (0x00000D00U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN                                         (0x00000D04U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN                                         (0x00000D08U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN                                         (0x00000D0CU)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN                                         (0x00000D10U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN                                         (0x00000D14U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN                                         (0x00000D18U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN                                         (0x00000D1CU)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN                                         (0x00000D20U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN                                         (0x00000D24U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN                                     (0x00000E00U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN                                     (0x00000E04U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN                                     (0x00000E08U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN                                     (0x00000E0CU)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN                                     (0x00000E10U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN                                     (0x00000E14U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN                                     (0x00000E18U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN                                     (0x00000E1CU)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN                                     (0x00000E20U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN                                         (0x00000E80U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN                                         (0x00000E84U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN                                         (0x00000E88U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN                                         (0x00000E8CU)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN                                         (0x00000E90U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN                                         (0x00000E94U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN                                         (0x00000E98U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN                                         (0x00000E9CU)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN                                         (0x00000F40U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN                                         (0x00000F44U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK0                                          (0x00001008U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK1                                          (0x0000100CU)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS                                      (0x00001010U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR                            (0x00001014U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE                                          (0x00001018U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR                                    (0x0000101CU)
#define CSL_CONTROLSS_CTRL_EOI                                                  (0x00001020U)
#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS                                        (0x00001024U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS                                    (0x00001028U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS                                    (0x0000102CU)
#define CSL_CONTROLSS_CTRL_FAULT_CLEAR                                          (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* PID */

#define CSL_CONTROLSS_CTRL_PID_PID_MINOR_MASK                            (0x0000003FU)
#define CSL_CONTROLSS_CTRL_PID_PID_MINOR_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_PID_PID_MINOR_RESETVAL                        (0x00000015U)
#define CSL_CONTROLSS_CTRL_PID_PID_MINOR_MAX                             (0x0000003FU)

#define CSL_CONTROLSS_CTRL_PID_PID_CUSTOM_MASK                           (0x000000C0U)
#define CSL_CONTROLSS_CTRL_PID_PID_CUSTOM_SHIFT                          (0x00000006U)
#define CSL_CONTROLSS_CTRL_PID_PID_CUSTOM_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_PID_PID_CUSTOM_MAX                            (0x00000003U)

#define CSL_CONTROLSS_CTRL_PID_PID_MAJOR_MASK                            (0x00000700U)
#define CSL_CONTROLSS_CTRL_PID_PID_MAJOR_SHIFT                           (0x00000008U)
#define CSL_CONTROLSS_CTRL_PID_PID_MAJOR_RESETVAL                        (0x00000002U)
#define CSL_CONTROLSS_CTRL_PID_PID_MAJOR_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_PID_PID_MISC_MASK                             (0x0000F800U)
#define CSL_CONTROLSS_CTRL_PID_PID_MISC_SHIFT                            (0x0000000BU)
#define CSL_CONTROLSS_CTRL_PID_PID_MISC_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_PID_PID_MISC_MAX                              (0x0000001FU)

#define CSL_CONTROLSS_CTRL_PID_PID_MSB16_MASK                            (0xFFFF0000U)
#define CSL_CONTROLSS_CTRL_PID_PID_MSB16_SHIFT                           (0x00000010U)
#define CSL_CONTROLSS_CTRL_PID_PID_MSB16_RESETVAL                        (0x00006180U)
#define CSL_CONTROLSS_CTRL_PID_PID_MSB16_MAX                             (0x0000FFFFU)



/* ADCEXTCHXBAR0_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR1_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR2_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR3_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR4_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR5_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR6_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR7_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR8_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCEXTCHXBAR9_G0_SEL */
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_SEL_MASK                        (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_SEL_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_SEL_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_SEL_MAX                         (0x0000000FU)



/* ADCSOCFRCGBSEL */
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_MAX                            (0x00000007U)



/* ADCSOCFRCGB */
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_MASK                                (0x0000FFFFU)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_SHIFT                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_RESETVAL                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_MAX                                 (0x0000FFFFU)



/* ADC_EXTCH_DLY_SEL */
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_RESETVAL                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_MAX                            (0x00000007U)



/* SDFM0_CLK0_OUT_SEL */
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_MAX                           (0x00000001U)



/* SDFM0_CLK1_OUT_SEL */
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_SEL_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_SEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_SEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_SEL_MAX                           (0x00000001U)



/* SDFM0_CLK2_OUT_SEL */
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_SEL_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_SEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_SEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_SEL_MAX                           (0x00000001U)



/* SDFM0_CLK3_OUT_SEL */
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_SEL_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_SEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_SEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_SEL_MAX                           (0x00000001U)



/* SDFM1_CLK0_OUT_SEL */
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_SEL_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_SEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_SEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_SEL_MAX                           (0x00000001U)



/* SDFM1_CLK1_OUT_SEL */
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_SEL_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_SEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_SEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_SEL_MAX                           (0x00000001U)



/* SDFM1_CLK2_OUT_SEL */
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_SEL_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_SEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_SEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_SEL_MAX                           (0x00000001U)



/* SDFM1_CLK3_OUT_SEL */
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_SEL_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_SEL_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_SEL_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_SEL_MAX                           (0x00000001U)



/* SDFM1_CLK0_SEL */
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_MASK                              (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_SHIFT                             (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_RESETVAL                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_MAX                               (0x00000001U)



/* CONTROLSS_G0_EPWM_WLINK */

#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_ENABLE_MASK           (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_ENABLE_SHIFT          (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_ENABLE_RESETVAL       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_ENABLE_MAX            (0xFFFFFFFFU)



/* CONTROLSS_G1_EPWM_WLINK */

#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_ENABLE_MASK           (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_ENABLE_SHIFT          (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_ENABLE_RESETVAL       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_ENABLE_MAX            (0xFFFFFFFFU)



/* EPWM_STATICXBAR_SEL0 */

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM0_MASK              (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM0_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM1_MASK              (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM1_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM2_MASK              (0x00000010U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM2_SHIFT             (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM2_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM2_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM3_MASK              (0x00000040U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM3_SHIFT             (0x00000006U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM3_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM3_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM4_MASK              (0x00000100U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM4_SHIFT             (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM4_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM4_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM5_MASK              (0x00000400U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM5_SHIFT             (0x0000000AU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM5_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM5_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM6_MASK              (0x00001000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM6_SHIFT             (0x0000000CU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM6_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM6_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM7_MASK              (0x00004000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM7_SHIFT             (0x0000000EU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM7_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM7_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM8_MASK              (0x00010000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM8_SHIFT             (0x00000010U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM8_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM8_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM9_MASK              (0x00040000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM9_SHIFT             (0x00000012U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM9_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM9_MAX               (0x00000001U)


/* EPWM_CLKSYNC */
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MASK                                (0x000003FFU)
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_SHIFT                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_RESETVAL                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MAX                                 (0x000003FFU)


/* EPWM_SOCA_SEL */
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_SEL_MASK                               (0x000003FFU)
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_SEL_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_SEL_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_SEL_MAX                                (0x000003FFU)


/* EPWM_SOCB_SEL */
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_SEL_MASK                               (0x000003FFU)
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_SEL_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_SEL_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_SEL_MAX                                (0x000003FFU)

/* EMUSTOPN_MASK */

#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B0_MAX                       (0x00000001U)

/* CLB_AQ_EN0 */
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_ENABLE_MASK                               (0x000FFFFFU)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_ENABLE_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_ENABLE_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_ENABLE_MAX                                (0x000FFFFFU)

/* CLB_DB_EN0 */
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_ENABLE_MASK                               (0x000FFFFFU)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_ENABLE_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_ENABLE_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_ENABLE_MAX                                (0x000FFFFFU)

/* XBAR_LOOPBACK_CTRL */
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_ENABLE_MASK                       (0x0000FFFFU)
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_ENABLE_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_ENABLE_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_ENABLE_MAX                        (0x0000FFFFU)

/* ETPWM0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM4_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM5_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM6_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM7_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM8_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

/* ETPWM9_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)



/* ECAP0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* ECAP1_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* ECAP2_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* ECAP3_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* ECAP4_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* ECAP5_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* ECAP6_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* ECAP7_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* CMPSSA0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CMPSSA1_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CMPSSA2_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CMPSSA3_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CMPSSA4_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CMPSSA5_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CMPSSA6_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CMPSSA7_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CMPSSA8_CLK_GATE */
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* ADC_SCTILE0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)



/* ADC_SCTILE1_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)



/* ADC_SCTILE2_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)



/* ADC_SCTILE3_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)



/* ADC_SCTILE4_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)



/* ADC_SCTILE5_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)



/* ADC0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_MAX                           (0x00000007U)



/* ADC1_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_CLK_GATE_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_CLK_GATE_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_CLK_GATE_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_CLK_GATE_MAX                           (0x00000007U)



/* ADC2_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_CLK_GATE_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_CLK_GATE_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_CLK_GATE_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_CLK_GATE_MAX                           (0x00000007U)



/* EQEP0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* EQEP1_CLK_GATE */
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* SDFM0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* SDFM1_CLK_GATE */
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* OTTO0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* OTTO1_CLK_GATE */
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_CLK_GATE_MASK                         (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_CLK_GATE_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_CLK_GATE_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_CLK_GATE_MAX                          (0x00000007U)



/* FSI_TX0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_MASK                       (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_MAX                        (0x00000007U)



/* FSI_RX0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_MASK                       (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_MAX                        (0x00000007U)



/* ADC_AGG0_CLK_GATE */
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_CLK_GATE_MASK                      (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_CLK_GATE_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_CLK_GATE_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_CLK_GATE_MAX                       (0x00000007U)



/* DAC_CLK_GATE */
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_MAX                            (0x00000007U)



/* FSI_TX0_PLL_CLK_GATE */
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)



/* SDFM0_PLL_CLK_GATE */
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* SDFM1_PLL_CLK_GATE */
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_CLK_GATE_MASK                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_CLK_GATE_SHIFT                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_CLK_GATE_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_CLK_GATE_MAX                      (0x00000007U)



/* CONTROLSS_XBAR_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_INPUTXBAR_MASK        (0x00000007U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_INPUTXBAR_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_INPUTXBAR_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_INPUTXBAR_MAX         (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_PWMXBAR_MASK          (0x00000070U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_PWMXBAR_SHIFT         (0x00000004U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_PWMXBAR_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_PWMXBAR_MAX           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_MDLXBAR_MASK          (0x00000700U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_MDLXBAR_SHIFT         (0x00000008U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_MDLXBAR_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_MDLXBAR_MAX           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_ICLXBAR_MASK          (0x00007000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_ICLXBAR_SHIFT         (0x0000000CU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_ICLXBAR_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_ICLXBAR_MAX           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_INTXBAR_MASK          (0x00070000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_INTXBAR_SHIFT         (0x00000010U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_INTXBAR_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_INTXBAR_MAX           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_DMAXBAR_MASK          (0x00700000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_DMAXBAR_SHIFT         (0x00000014U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_DMAXBAR_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_DMAXBAR_MAX           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_OUTPUTXBAR_MASK       (0x07000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_OUTPUTXBAR_SHIFT      (0x00000018U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_OUTPUTXBAR_RESETVAL   (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_OUTPUTXBAR_MAX        (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_PWMSYNCOUTXBAR_MASK   (0x70000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_PWMSYNCOUTXBAR_SHIFT  (0x0000001CU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_PWMSYNCOUTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_PWMSYNCOUTXBAR_MAX    (0x00000007U)



/* ETPWM0_RST */

#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_MAX                            (0x00000007U)



/* ETPWM1_RST */

#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RST_MAX                            (0x00000007U)



/* ETPWM2_RST */

#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RST_MAX                            (0x00000007U)



/* ETPWM3_RST */

#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RST_MAX                            (0x00000007U)



/* ETPWM4_RST */

#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RST_MAX                            (0x00000007U)



/* ETPWM5_RST */

#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RST_MAX                            (0x00000007U)



/* ETPWM6_RST */

#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RST_MAX                            (0x00000007U)



/* ETPWM7_RST */

#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RST_MAX                            (0x00000007U)



/* ETPWM8_RST */

#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RST_MAX                            (0x00000007U)



/* ETPWM9_RST */

#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RST_MAX                            (0x00000007U)



/* ECAP0_RST */
#define CSL_CONTROLSS_CTRL_ECAP0_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP0_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_RST_RST_MAX                                    (0x00000007U)



/* ECAP1_RST */
#define CSL_CONTROLSS_CTRL_ECAP1_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP1_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_RST_RST_MAX                                    (0x00000007U)



/* ECAP2_RST */
#define CSL_CONTROLSS_CTRL_ECAP2_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP2_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_RST_RST_MAX                                    (0x00000007U)



/* ECAP3_RST */
#define CSL_CONTROLSS_CTRL_ECAP3_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP3_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_RST_RST_MAX                                    (0x00000007U)



/* ECAP4_RST */
#define CSL_CONTROLSS_CTRL_ECAP4_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP4_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_RST_RST_MAX                                    (0x00000007U)



/* ECAP5_RST */
#define CSL_CONTROLSS_CTRL_ECAP5_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP5_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_RST_RST_MAX                                    (0x00000007U)



/* ECAP6_RST */
#define CSL_CONTROLSS_CTRL_ECAP6_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP6_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_RST_RST_MAX                                    (0x00000007U)



/* ECAP7_RST */
#define CSL_CONTROLSS_CTRL_ECAP7_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP7_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_RST_RST_MAX                                    (0x00000007U)



/* CMPSSA0_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_MAX                                (0x00000007U)



/* CMPSSA1_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RST_MAX                                (0x00000007U)



/* CMPSSA2_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RST_MAX                                (0x00000007U)



/* CMPSSA3_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RST_MAX                                (0x00000007U)



/* CMPSSA4_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RST_MAX                                (0x00000007U)



/* CMPSSA5_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RST_MAX                                (0x00000007U)



/* CMPSSA6_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RST_MAX                                (0x00000007U)



/* CMPSSA7_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RST_MAX                                (0x00000007U)



/* CMPSSA8_RST */
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RST_MASK                               (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RST_SHIFT                              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RST_RESETVAL                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RST_MAX                                (0x00000007U)



/* ADC_SCTILE0_RST */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RST_MAX                              (0x00000007U)



/* ADC_SCTILE1_RST */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RST_MAX                              (0x00000007U)



/* ADC_SCTILE2_RST */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RST_MAX                              (0x00000007U)



/* ADC_SCTILE3_RST */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RST_MAX                              (0x00000007U)



/* ADC_SCTILE4_RST */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RST_MAX                              (0x00000007U)



/* ADC_SCTILE5_RST */
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RST_MAX                              (0x00000007U)



/* ADC0_RST */
#define CSL_CONTROLSS_CTRL_ADC0_RST_RST_MASK                                    (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC0_RST_RST_SHIFT                                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC0_RST_RST_RESETVAL                                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC0_RST_RST_MAX                                     (0x00000007U)



/* ADC1_RST */
#define CSL_CONTROLSS_CTRL_ADC1_RST_RST_MASK                                    (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC1_RST_RST_SHIFT                                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC1_RST_RST_RESETVAL                                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC1_RST_RST_MAX                                     (0x00000007U)



/* ADC2_RST */
#define CSL_CONTROLSS_CTRL_ADC2_RST_RST_MASK                                    (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC2_RST_RST_SHIFT                                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC2_RST_RST_RESETVAL                                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC2_RST_RST_MAX                                     (0x00000007U)



/* EQEP0_RST */
#define CSL_CONTROLSS_CTRL_EQEP0_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP0_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_RST_RST_MAX                                    (0x00000007U)



/* EQEP1_RST */
#define CSL_CONTROLSS_CTRL_EQEP1_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP1_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_RST_RST_MAX                                    (0x00000007U)



/* SDFM0_RST */
#define CSL_CONTROLSS_CTRL_SDFM0_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM0_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_RST_RST_MAX                                    (0x00000007U)



/* SDFM1_RST */
#define CSL_CONTROLSS_CTRL_SDFM1_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM1_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_RST_RST_MAX                                    (0x00000007U)



/* OTTO0_RST */
#define CSL_CONTROLSS_CTRL_OTTO0_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO0_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO0_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO0_RST_RST_MAX                                    (0x00000007U)



/* OTTO1_RST */
#define CSL_CONTROLSS_CTRL_OTTO1_RST_RST_MASK                                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO1_RST_RST_SHIFT                                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO1_RST_RST_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO1_RST_RST_MAX                                    (0x00000007U)



/* FSI_TX0_RST */
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_MASK                                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_SHIFT                                (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_RESETVAL                             (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_MAX                                  (0x00000007U)



/* FSI_RX0_RST */
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_MASK                                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_SHIFT                                (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_RESETVAL                             (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_MAX                                  (0x00000007U)



/* ADC_AGG0_RST */
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_RST_MASK                                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_RST_SHIFT                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_RST_RESETVAL                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_RST_MAX                                 (0x00000007U)



/* DAC_RST */
#define CSL_CONTROLSS_CTRL_DAC_RST_RST_MASK                                     (0x00000007U)
#define CSL_CONTROLSS_CTRL_DAC_RST_RST_SHIFT                                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_DAC_RST_RST_RESETVAL                                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_DAC_RST_RST_MAX                                      (0x00000007U)



/* EPWM0_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM1_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM2_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM3_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM4_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM5_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM6_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM7_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM8_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EPWM9_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B0_MAX                        (0x00000001U)



/* CMPSSA0_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B0_MAX                    (0x00000001U)



/* CMPSSA1_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B0_MAX                    (0x00000001U)



/* CMPSSA2_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B0_MAX                    (0x00000001U)



/* CMPSSA3_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B0_MAX                    (0x00000001U)



/* CMPSSA4_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B0_MAX                    (0x00000001U)



/* CMPSSA5_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B0_MAX                    (0x00000001U)



/* CMPSSA6_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B0_MAX                    (0x00000001U)



/* CMPSSA7_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B0_MAX                    (0x00000001U)



/* CMPSSA8_HALTEN */
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A0_MASK                   (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A0_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A0_MAX                    (0x00000001U)


#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B0_MASK                   (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B0_SHIFT                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B0_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B0_MAX                    (0x00000001U)



/* ECAP0_HALTEN */
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_MAX                        (0x00000001U)



/* ECAP1_HALTEN */
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B0_MAX                        (0x00000001U)



/* ECAP2_HALTEN */
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B0_MAX                        (0x00000001U)



/* ECAP3_HALTEN */
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B0_MAX                        (0x00000001U)



/* ECAP4_HALTEN */
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B0_MAX                        (0x00000001U)



/* ECAP5_HALTEN */
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B0_MAX                        (0x00000001U)



/* ECAP6_HALTEN */
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B0_MAX                        (0x00000001U)



/* ECAP7_HALTEN */
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EQEP0_HALTEN */
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B0_MAX                        (0x00000001U)



/* EQEP1_HALTEN */
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A0_MAX                        (0x00000001U)


#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B0_MAX                        (0x00000001U)



/* LOCK0_KICK0 */

#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                  (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                   (0xFFFFFFFFU)



/* LOCK0_KICK1 */

#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MASK                  (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_LOCK0_KICK1_MAX                   (0xFFFFFFFFU)



/* INTR_RAW_STATUS */

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                 (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                  (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                 (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                  (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                 (0x00000004U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                  (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                (0x00000008U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT               (0x00000003U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                 (0x00000001U)



/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK (0x00000004U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK (0x00000008U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT (0x00000003U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX (0x00000001U)



/* INTR_ENABLE */

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                  (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                   (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                  (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                 (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                   (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                  (0x00000004U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                 (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                   (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                 (0x00000008U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                (0x00000003U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                  (0x00000001U)



/* INTR_ENABLE_CLEAR */

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK       (0x00000008U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT      (0x00000003U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL   (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX        (0x00000001U)



/* EOI */

#define CSL_CONTROLSS_CTRL_EOI_EOI_VECTOR_MASK                           (0x000000FFU)
#define CSL_CONTROLSS_CTRL_EOI_EOI_VECTOR_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EOI_EOI_VECTOR_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_EOI_EOI_VECTOR_MAX                            (0x000000FFU)



/* FAULT_ADDRESS */

#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                 (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                  (0xFFFFFFFFU)



/* FAULT_TYPE_STATUS */

#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK             (0x0000003FU)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX              (0x0000003FU)

#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK               (0x00000040U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT              (0x00000006U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL           (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                (0x00000001U)



/* FAULT_ATTR_STATUS */

#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK           (0x000000FFU)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT          (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL       (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX            (0x000000FFU)

#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK          (0x000FFF00U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT         (0x00000008U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX           (0x00000FFFU)

#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK              (0xFFF00000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT             (0x00000014U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX               (0x00000FFFU)



/* FAULT_CLEAR */

#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                    (0x00000001U)
#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                     (0x00000001U)



#ifdef __cplusplus
}
#endif
#endif