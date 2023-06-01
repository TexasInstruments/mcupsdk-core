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
    volatile uint32_t PID;
    volatile uint32_t EPWM_STATICXBAR_SEL0;
    volatile uint32_t EPWM_STATICXBAR_SEL1;
    volatile uint8_t  Resv_16[4];
    volatile uint32_t EPWM_CLKSYNC;
    volatile uint8_t  Resv_24[4];
    volatile uint32_t SDFM1_CLK0_SEL;
    volatile uint8_t  Resv_32[4];
    volatile uint32_t EMUSTOPN_MASK;
    volatile uint8_t  Resv_40[4];
    volatile uint32_t CLB_AQ_EN0;
    volatile uint8_t  Resv_48[4];
    volatile uint32_t CLB_AQ_EN1;
    volatile uint8_t  Resv_56[4];
    volatile uint32_t CLB_DB_EN0;
    volatile uint8_t  Resv_64[4];
    volatile uint32_t CLB_DB_EN1;
    volatile uint32_t ADCSOCFRCGBSEL;
    volatile uint32_t ADCSOCFRCGB;
    volatile uint32_t ADC_EXTCH_DLY_SEL;
    volatile uint32_t XBAR_LOOPBACK_CTRL;
    volatile uint8_t  Resv_96[12];
    volatile uint32_t EPWM_SOCA_SEL;
    volatile uint32_t EPWM_SOCB_SEL;
    volatile uint8_t  Resv_112[8];
    volatile uint32_t ADCEXTCHXBAR0_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR1_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR2_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR3_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR4_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR5_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR6_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR7_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR8_G0_SEL;
    volatile uint32_t ADCEXTCHXBAR9_G0_SEL;
    volatile uint8_t  Resv_176[24];
    volatile uint32_t SDFM0_CLK0_OUT_SEL;
    volatile uint32_t SDFM0_CLK1_OUT_SEL;
    volatile uint32_t SDFM0_CLK2_OUT_SEL;
    volatile uint32_t SDFM0_CLK3_OUT_SEL;
    volatile uint32_t SDFM1_CLK0_OUT_SEL;
    volatile uint32_t SDFM1_CLK1_OUT_SEL;
    volatile uint32_t SDFM1_CLK2_OUT_SEL;
    volatile uint32_t SDFM1_CLK3_OUT_SEL;
    volatile uint8_t  Resv_224[16];
    volatile uint32_t CONTROLSS_G0_EPWM_WLINK;
    volatile uint32_t CONTROLSS_G1_EPWM_WLINK;
    volatile uint32_t CONTROLSS_G2_EPWM_WLINK;
    volatile uint32_t CONTROLSS_G3_EPWM_WLINK;
    volatile uint8_t  Resv_256[16];
    volatile uint32_t ETPWM0_CLK_GATE;
    volatile uint32_t ETPWM1_CLK_GATE;
    volatile uint32_t ETPWM2_CLK_GATE;
    volatile uint32_t ETPWM3_CLK_GATE;
    volatile uint32_t ETPWM4_CLK_GATE;
    volatile uint32_t ETPWM5_CLK_GATE;
    volatile uint32_t ETPWM6_CLK_GATE;
    volatile uint32_t ETPWM7_CLK_GATE;
    volatile uint32_t ETPWM8_CLK_GATE;
    volatile uint32_t ETPWM9_CLK_GATE;
    volatile uint32_t ETPWM10_CLK_GATE;
    volatile uint32_t ETPWM11_CLK_GATE;
    volatile uint32_t ETPWM12_CLK_GATE;
    volatile uint32_t ETPWM13_CLK_GATE;
    volatile uint32_t ETPWM14_CLK_GATE;
    volatile uint32_t ETPWM15_CLK_GATE;
    volatile uint32_t ETPWM16_CLK_GATE;
    volatile uint32_t ETPWM17_CLK_GATE;
    volatile uint32_t ETPWM18_CLK_GATE;
    volatile uint32_t ETPWM19_CLK_GATE;
    volatile uint32_t ETPWM20_CLK_GATE;
    volatile uint32_t ETPWM21_CLK_GATE;
    volatile uint32_t ETPWM22_CLK_GATE;
    volatile uint32_t ETPWM23_CLK_GATE;
    volatile uint32_t ETPWM24_CLK_GATE;
    volatile uint32_t ETPWM25_CLK_GATE;
    volatile uint32_t ETPWM26_CLK_GATE;
    volatile uint32_t ETPWM27_CLK_GATE;
    volatile uint32_t ETPWM28_CLK_GATE;
    volatile uint32_t ETPWM29_CLK_GATE;
    volatile uint32_t ETPWM30_CLK_GATE;
    volatile uint32_t ETPWM31_CLK_GATE;
    volatile uint32_t FSI_TX0_CLK_GATE;
    volatile uint32_t FSI_TX1_CLK_GATE;
    volatile uint32_t FSI_TX2_CLK_GATE;
    volatile uint32_t FSI_TX3_CLK_GATE;
    volatile uint32_t FSI_RX0_CLK_GATE;
    volatile uint32_t FSI_RX1_CLK_GATE;
    volatile uint32_t FSI_RX2_CLK_GATE;
    volatile uint32_t FSI_RX3_CLK_GATE;
    volatile uint32_t CMPSSA0_CLK_GATE;
    volatile uint32_t CMPSSA1_CLK_GATE;
    volatile uint32_t CMPSSA2_CLK_GATE;
    volatile uint32_t CMPSSA3_CLK_GATE;
    volatile uint32_t CMPSSA4_CLK_GATE;
    volatile uint32_t CMPSSA5_CLK_GATE;
    volatile uint32_t CMPSSA6_CLK_GATE;
    volatile uint32_t CMPSSA7_CLK_GATE;
    volatile uint32_t CMPSSA8_CLK_GATE;
    volatile uint32_t CMPSSA9_CLK_GATE;
    volatile uint8_t  Resv_464[8];
    volatile uint32_t CMPSSB0_CLK_GATE;
    volatile uint32_t CMPSSB1_CLK_GATE;
    volatile uint32_t CMPSSB2_CLK_GATE;
    volatile uint32_t CMPSSB3_CLK_GATE;
    volatile uint32_t CMPSSB4_CLK_GATE;
    volatile uint32_t CMPSSB5_CLK_GATE;
    volatile uint32_t CMPSSB6_CLK_GATE;
    volatile uint32_t CMPSSB7_CLK_GATE;
    volatile uint32_t CMPSSB8_CLK_GATE;
    volatile uint32_t CMPSSB9_CLK_GATE;
    volatile uint8_t  Resv_512[8];
    volatile uint32_t ECAP0_CLK_GATE;
    volatile uint32_t ECAP1_CLK_GATE;
    volatile uint32_t ECAP2_CLK_GATE;
    volatile uint32_t ECAP3_CLK_GATE;
    volatile uint32_t ECAP4_CLK_GATE;
    volatile uint32_t ECAP5_CLK_GATE;
    volatile uint32_t ECAP6_CLK_GATE;
    volatile uint32_t ECAP7_CLK_GATE;
    volatile uint32_t ECAP8_CLK_GATE;
    volatile uint32_t ECAP9_CLK_GATE;
    volatile uint32_t ECAP10_CLK_GATE;
    volatile uint32_t ECAP11_CLK_GATE;
    volatile uint32_t ECAP12_CLK_GATE;
    volatile uint32_t ECAP13_CLK_GATE;
    volatile uint32_t ECAP14_CLK_GATE;
    volatile uint32_t ECAP15_CLK_GATE;
    volatile uint32_t EQEP0_CLK_GATE;
    volatile uint32_t EQEP1_CLK_GATE;
    volatile uint32_t EQEP2_CLK_GATE;
    volatile uint8_t  Resv_592[4];
    volatile uint32_t SDFM0_CLK_GATE;
    volatile uint32_t SDFM1_CLK_GATE;
    volatile uint32_t DAC_CLK_GATE;
    volatile uint32_t ADC0_CLK_GATE;
    volatile uint32_t ADC1_CLK_GATE;
    volatile uint32_t ADC2_CLK_GATE;
    volatile uint32_t ADC3_CLK_GATE;
    volatile uint32_t ADC4_CLK_GATE;
    volatile uint32_t OTTO0_CLK_GATE;
    volatile uint32_t OTTO1_CLK_GATE;
    volatile uint32_t OTTO2_CLK_GATE;
    volatile uint32_t OTTO3_CLK_GATE;
    volatile uint32_t SDFM0_PLL_CLK_GATE;
    volatile uint32_t SDFM1_PLL_CLK_GATE;
    volatile uint32_t FSI_TX0_PLL_CLK_GATE;
    volatile uint32_t FSI_TX1_PLL_CLK_GATE;
    volatile uint32_t FSI_TX2_PLL_CLK_GATE;
    volatile uint32_t FSI_TX3_PLL_CLK_GATE;
    volatile uint32_t HW_RESOLVER_CLK_GATE;
    volatile uint32_t ADC_SCTILE0_CLK_GATE;
    volatile uint32_t ADC_SCTILE1_CLK_GATE;
    volatile uint32_t ADC_SCTILE2_CLK_GATE;
    volatile uint32_t ADC_SCTILE3_CLK_GATE;
    volatile uint32_t ADC_SCTILE4_CLK_GATE;
    volatile uint32_t ADC_SCTILE5_CLK_GATE;
    volatile uint32_t ADC_SCTILE6_CLK_GATE;
    volatile uint32_t ADC_SCTILE7_CLK_GATE;
    volatile uint32_t ADC_SCTILE8_CLK_GATE;
    volatile uint32_t ADC_SCTILE9_CLK_GATE;
    volatile uint32_t ADC_SCTILE10_CLK_GATE;
    volatile uint32_t ADC_SCTILE11_CLK_GATE;
    volatile uint8_t  Resv_732[16];
    volatile uint32_t ADC_AGG0_CLK_GATE;
    volatile uint32_t CONTROLSS_XBAR_CLK_GATE;
    volatile uint32_t ADCR0_CLK_GATE;
    volatile uint32_t ADCR1_CLK_GATE;
    volatile uint8_t  Resv_768[20];
    volatile uint32_t ETPWM0_RST;
    volatile uint32_t ETPWM1_RST;
    volatile uint32_t ETPWM2_RST;
    volatile uint32_t ETPWM3_RST;
    volatile uint32_t ETPWM4_RST;
    volatile uint32_t ETPWM5_RST;
    volatile uint32_t ETPWM6_RST;
    volatile uint32_t ETPWM7_RST;
    volatile uint32_t ETPWM8_RST;
    volatile uint32_t ETPWM9_RST;
    volatile uint32_t ETPWM10_RST;
    volatile uint32_t ETPWM11_RST;
    volatile uint32_t ETPWM12_RST;
    volatile uint32_t ETPWM13_RST;
    volatile uint32_t ETPWM14_RST;
    volatile uint32_t ETPWM15_RST;
    volatile uint32_t ETPWM16_RST;
    volatile uint32_t ETPWM17_RST;
    volatile uint32_t ETPWM18_RST;
    volatile uint32_t ETPWM19_RST;
    volatile uint32_t ETPWM20_RST;
    volatile uint32_t ETPWM21_RST;
    volatile uint32_t ETPWM22_RST;
    volatile uint32_t ETPWM23_RST;
    volatile uint32_t ETPWM24_RST;
    volatile uint32_t ETPWM25_RST;
    volatile uint32_t ETPWM26_RST;
    volatile uint32_t ETPWM27_RST;
    volatile uint32_t ETPWM28_RST;
    volatile uint32_t ETPWM29_RST;
    volatile uint32_t ETPWM30_RST;
    volatile uint32_t ETPWM31_RST;
    volatile uint32_t FSI_TX0_RST;
    volatile uint32_t FSI_TX1_RST;
    volatile uint32_t FSI_TX2_RST;
    volatile uint32_t FSI_TX3_RST;
    volatile uint32_t FSI_RX0_RST;
    volatile uint32_t FSI_RX1_RST;
    volatile uint32_t FSI_RX2_RST;
    volatile uint32_t FSI_RX3_RST;
    volatile uint32_t CMPSSA0_RST;
    volatile uint32_t CMPSSA1_RST;
    volatile uint32_t CMPSSA2_RST;
    volatile uint32_t CMPSSA3_RST;
    volatile uint32_t CMPSSA4_RST;
    volatile uint32_t CMPSSA5_RST;
    volatile uint32_t CMPSSA6_RST;
    volatile uint32_t CMPSSA7_RST;
    volatile uint32_t CMPSSA8_RST;
    volatile uint32_t CMPSSA9_RST;
    volatile uint8_t  Resv_976[8];
    volatile uint32_t CMPSSB0_RST;
    volatile uint32_t CMPSSB1_RST;
    volatile uint32_t CMPSSB2_RST;
    volatile uint32_t CMPSSB3_RST;
    volatile uint32_t CMPSSB4_RST;
    volatile uint32_t CMPSSB5_RST;
    volatile uint32_t CMPSSB6_RST;
    volatile uint32_t CMPSSB7_RST;
    volatile uint32_t CMPSSB8_RST;
    volatile uint32_t CMPSSB9_RST;
    volatile uint8_t  Resv_1024[8];
    volatile uint32_t ECAP0_RST;
    volatile uint32_t ECAP1_RST;
    volatile uint32_t ECAP2_RST;
    volatile uint32_t ECAP3_RST;
    volatile uint32_t ECAP4_RST;
    volatile uint32_t ECAP5_RST;
    volatile uint32_t ECAP6_RST;
    volatile uint32_t ECAP7_RST;
    volatile uint32_t ECAP8_RST;
    volatile uint32_t ECAP9_RST;
    volatile uint32_t ECAP10_RST;
    volatile uint32_t ECAP11_RST;
    volatile uint32_t ECAP12_RST;
    volatile uint32_t ECAP13_RST;
    volatile uint32_t ECAP14_RST;
    volatile uint32_t ECAP15_RST;
    volatile uint32_t EQEP0_RST;
    volatile uint32_t EQEP1_RST;
    volatile uint32_t EQEP2_RST;
    volatile uint8_t  Resv_1104[4];
    volatile uint32_t SDFM0_RST;
    volatile uint32_t SDFM1_RST;
    volatile uint32_t DAC_RST;
    volatile uint32_t ADC0_RST;
    volatile uint32_t ADC1_RST;
    volatile uint32_t ADC2_RST;
    volatile uint32_t ADC3_RST;
    volatile uint32_t ADC4_RST;
    volatile uint32_t OTTO0_RST;
    volatile uint32_t OTTO1_RST;
    volatile uint32_t OTTO2_RST;
    volatile uint32_t OTTO3_RST;
    volatile uint32_t HW_RESOLVER_RST;
    volatile uint32_t ADC_SCTILE0_RST;
    volatile uint32_t ADC_SCTILE1_RST;
    volatile uint32_t ADC_SCTILE2_RST;
    volatile uint32_t ADC_SCTILE3_RST;
    volatile uint32_t ADC_SCTILE4_RST;
    volatile uint32_t ADC_SCTILE5_RST;
    volatile uint32_t ADC_SCTILE6_RST;
    volatile uint32_t ADC_SCTILE7_RST;
    volatile uint32_t ADC_SCTILE8_RST;
    volatile uint32_t ADC_SCTILE9_RST;
    volatile uint32_t ADC_SCTILE10_RST;
    volatile uint32_t ADC_SCTILE11_RST;
    volatile uint8_t  Resv_1220[16];
    volatile uint32_t ADC_AGG0_RST;
    volatile uint32_t ADCR0_RST;
    volatile uint32_t ADCR1_RST;
    volatile uint8_t  Resv_1280[48];
    volatile uint32_t EPWM0_HALTEN;
    volatile uint32_t EPWM1_HALTEN;
    volatile uint32_t EPWM2_HALTEN;
    volatile uint32_t EPWM3_HALTEN;
    volatile uint32_t EPWM4_HALTEN;
    volatile uint32_t EPWM5_HALTEN;
    volatile uint32_t EPWM6_HALTEN;
    volatile uint32_t EPWM7_HALTEN;
    volatile uint32_t EPWM8_HALTEN;
    volatile uint32_t EPWM9_HALTEN;
    volatile uint32_t EPWM10_HALTEN;
    volatile uint32_t EPWM11_HALTEN;
    volatile uint32_t EPWM12_HALTEN;
    volatile uint32_t EPWM13_HALTEN;
    volatile uint32_t EPWM14_HALTEN;
    volatile uint32_t EPWM15_HALTEN;
    volatile uint32_t EPWM16_HALTEN;
    volatile uint32_t EPWM17_HALTEN;
    volatile uint32_t EPWM18_HALTEN;
    volatile uint32_t EPWM19_HALTEN;
    volatile uint32_t EPWM20_HALTEN;
    volatile uint32_t EPWM21_HALTEN;
    volatile uint32_t EPWM22_HALTEN;
    volatile uint32_t EPWM23_HALTEN;
    volatile uint32_t EPWM24_HALTEN;
    volatile uint32_t EPWM25_HALTEN;
    volatile uint32_t EPWM26_HALTEN;
    volatile uint32_t EPWM27_HALTEN;
    volatile uint32_t EPWM28_HALTEN;
    volatile uint32_t EPWM29_HALTEN;
    volatile uint32_t EPWM30_HALTEN;
    volatile uint32_t EPWM31_HALTEN;
    volatile uint32_t CMPSSA0_HALTEN;
    volatile uint32_t CMPSSA1_HALTEN;
    volatile uint32_t CMPSSA2_HALTEN;
    volatile uint32_t CMPSSA3_HALTEN;
    volatile uint32_t CMPSSA4_HALTEN;
    volatile uint32_t CMPSSA5_HALTEN;
    volatile uint32_t CMPSSA6_HALTEN;
    volatile uint32_t CMPSSA7_HALTEN;
    volatile uint32_t CMPSSA8_HALTEN;
    volatile uint32_t CMPSSA9_HALTEN;
    volatile uint32_t CMPSSB0_HALTEN;
    volatile uint32_t CMPSSB1_HALTEN;
    volatile uint32_t CMPSSB2_HALTEN;
    volatile uint32_t CMPSSB3_HALTEN;
    volatile uint32_t CMPSSB4_HALTEN;
    volatile uint32_t CMPSSB5_HALTEN;
    volatile uint32_t CMPSSB6_HALTEN;
    volatile uint32_t CMPSSB7_HALTEN;
    volatile uint32_t CMPSSB8_HALTEN;
    volatile uint32_t CMPSSB9_HALTEN;
    volatile uint32_t ECAP0_HALTEN;
    volatile uint32_t ECAP1_HALTEN;
    volatile uint32_t ECAP2_HALTEN;
    volatile uint32_t ECAP3_HALTEN;
    volatile uint32_t ECAP4_HALTEN;
    volatile uint32_t ECAP5_HALTEN;
    volatile uint32_t ECAP6_HALTEN;
    volatile uint32_t ECAP7_HALTEN;
    volatile uint32_t ECAP8_HALTEN;
    volatile uint32_t ECAP9_HALTEN;
    volatile uint32_t EQEP0_HALTEN;
    volatile uint32_t EQEP1_HALTEN;
    volatile uint32_t EQEP2_HALTEN;
    volatile uint32_t ECAP10_HALTEN;
    volatile uint32_t ECAP11_HALTEN;
    volatile uint32_t ECAP12_HALTEN;
    volatile uint32_t ECAP13_HALTEN;
    volatile uint32_t ECAP14_HALTEN;
    volatile uint32_t ECAP15_HALTEN;
    volatile uint8_t  Resv_4104[2540];
    volatile uint32_t LOCK0_KICK0;
    volatile uint32_t LOCK0_KICK1;
    volatile uint32_t INTR_RAW_STATUS;
    volatile uint32_t INTR_ENABLED_STATUS_CLEAR;
    volatile uint32_t INTR_ENABLE;
    volatile uint32_t INTR_ENABLE_CLEAR;
    volatile uint32_t EOI;
    volatile uint32_t FAULT_ADDRESS;
    volatile uint32_t FAULT_TYPE_STATUS;
    volatile uint32_t FAULT_ATTR_STATUS;
    volatile uint32_t FAULT_CLEAR;
} CSL_controlss_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CONTROLSS_CTRL_PID                                                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0                                (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1                                (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC                                        (0x00000010U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL                                      (0x00000018U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK                                       (0x00000020U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0                                          (0x00000028U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN1                                          (0x00000030U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0                                          (0x00000038U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN1                                          (0x00000040U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL                                      (0x00000044U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB                                         (0x00000048U)
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL                                   (0x0000004CU)
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL                                  (0x00000050U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL                                       (0x00000060U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL                                       (0x00000064U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL                                (0x00000070U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL                                (0x00000074U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL                                (0x00000078U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL                                (0x0000007CU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL                                (0x00000080U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL                                (0x00000084U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL                                (0x00000088U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL                                (0x0000008CU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL                                (0x00000090U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL                                (0x00000094U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL                                  (0x000000B0U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL                                  (0x000000B4U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL                                  (0x000000B8U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL                                  (0x000000BCU)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL                                  (0x000000C0U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL                                  (0x000000C4U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL                                  (0x000000C8U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL                                  (0x000000CCU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK                             (0x000000E0U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK                             (0x000000E4U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G2_EPWM_WLINK                             (0x000000E8U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G3_EPWM_WLINK                             (0x000000ECU)
#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE                                     (0x00000100U)
#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE                                     (0x00000104U)
#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE                                     (0x00000108U)
#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE                                     (0x0000010CU)
#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE                                     (0x00000110U)
#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE                                     (0x00000114U)
#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE                                     (0x00000118U)
#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE                                     (0x0000011CU)
#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE                                     (0x00000120U)
#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE                                     (0x00000124U)
#define CSL_CONTROLSS_CTRL_ETPWM10_CLK_GATE                                    (0x00000128U)
#define CSL_CONTROLSS_CTRL_ETPWM11_CLK_GATE                                    (0x0000012CU)
#define CSL_CONTROLSS_CTRL_ETPWM12_CLK_GATE                                    (0x00000130U)
#define CSL_CONTROLSS_CTRL_ETPWM13_CLK_GATE                                    (0x00000134U)
#define CSL_CONTROLSS_CTRL_ETPWM14_CLK_GATE                                    (0x00000138U)
#define CSL_CONTROLSS_CTRL_ETPWM15_CLK_GATE                                    (0x0000013CU)
#define CSL_CONTROLSS_CTRL_ETPWM16_CLK_GATE                                    (0x00000140U)
#define CSL_CONTROLSS_CTRL_ETPWM17_CLK_GATE                                    (0x00000144U)
#define CSL_CONTROLSS_CTRL_ETPWM18_CLK_GATE                                    (0x00000148U)
#define CSL_CONTROLSS_CTRL_ETPWM19_CLK_GATE                                    (0x0000014CU)
#define CSL_CONTROLSS_CTRL_ETPWM20_CLK_GATE                                    (0x00000150U)
#define CSL_CONTROLSS_CTRL_ETPWM21_CLK_GATE                                    (0x00000154U)
#define CSL_CONTROLSS_CTRL_ETPWM22_CLK_GATE                                    (0x00000158U)
#define CSL_CONTROLSS_CTRL_ETPWM23_CLK_GATE                                    (0x0000015CU)
#define CSL_CONTROLSS_CTRL_ETPWM24_CLK_GATE                                    (0x00000160U)
#define CSL_CONTROLSS_CTRL_ETPWM25_CLK_GATE                                    (0x00000164U)
#define CSL_CONTROLSS_CTRL_ETPWM26_CLK_GATE                                    (0x00000168U)
#define CSL_CONTROLSS_CTRL_ETPWM27_CLK_GATE                                    (0x0000016CU)
#define CSL_CONTROLSS_CTRL_ETPWM28_CLK_GATE                                    (0x00000170U)
#define CSL_CONTROLSS_CTRL_ETPWM29_CLK_GATE                                    (0x00000174U)
#define CSL_CONTROLSS_CTRL_ETPWM30_CLK_GATE                                    (0x00000178U)
#define CSL_CONTROLSS_CTRL_ETPWM31_CLK_GATE                                    (0x0000017CU)
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE                                    (0x00000180U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_CLK_GATE                                    (0x00000184U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_CLK_GATE                                    (0x00000188U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_CLK_GATE                                    (0x0000018CU)
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE                                    (0x00000190U)
#define CSL_CONTROLSS_CTRL_FSI_RX1_CLK_GATE                                    (0x00000194U)
#define CSL_CONTROLSS_CTRL_FSI_RX2_CLK_GATE                                    (0x00000198U)
#define CSL_CONTROLSS_CTRL_FSI_RX3_CLK_GATE                                    (0x0000019CU)
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE                                  (0x000001A0U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE                                  (0x000001A4U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE                                  (0x000001A8U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE                                  (0x000001ACU)
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE                                  (0x000001B0U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE                                  (0x000001B4U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE                                  (0x000001B8U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE                                  (0x000001BCU)
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE                                  (0x000001C0U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_CLK_GATE                                  (0x000001C4U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE                                   (0x000001D0U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_CLK_GATE                                   (0x000001D4U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_CLK_GATE                                   (0x000001D8U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_CLK_GATE                                   (0x000001DCU)
#define CSL_CONTROLSS_CTRL_CMPSSB4_CLK_GATE                                   (0x000001E0U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_CLK_GATE                                   (0x000001E4U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_CLK_GATE                                   (0x000001E8U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_CLK_GATE                                   (0x000001ECU)
#define CSL_CONTROLSS_CTRL_CMPSSB8_CLK_GATE                                   (0x000001F0U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_CLK_GATE                                   (0x000001F4U)
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE                                      (0x00000200U)
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE                                      (0x00000204U)
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE                                      (0x00000208U)
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE                                      (0x0000020CU)
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE                                      (0x00000210U)
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE                                      (0x00000214U)
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE                                      (0x00000218U)
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE                                      (0x0000021CU)
#define CSL_CONTROLSS_CTRL_ECAP8_CLK_GATE                                      (0x00000220U)
#define CSL_CONTROLSS_CTRL_ECAP9_CLK_GATE                                      (0x00000224U)
#define CSL_CONTROLSS_CTRL_ECAP10_CLK_GATE                                     (0x00000228U)
#define CSL_CONTROLSS_CTRL_ECAP11_CLK_GATE                                     (0x0000022CU)
#define CSL_CONTROLSS_CTRL_ECAP12_CLK_GATE                                     (0x00000230U)
#define CSL_CONTROLSS_CTRL_ECAP13_CLK_GATE                                     (0x00000234U)
#define CSL_CONTROLSS_CTRL_ECAP14_CLK_GATE                                     (0x00000238U)
#define CSL_CONTROLSS_CTRL_ECAP15_CLK_GATE                                     (0x0000023CU)
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE                                      (0x00000240U)
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE                                      (0x00000244U)
#define CSL_CONTROLSS_CTRL_EQEP2_CLK_GATE                                      (0x00000248U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE                                      (0x00000250U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE                                      (0x00000254U)
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE                                        (0x00000258U)
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE                                       (0x0000025CU)
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE                                       (0x00000260U)
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE                                       (0x00000264U)
#define CSL_CONTROLSS_CTRL_ADC3_CLK_GATE                                       (0x00000268U)
#define CSL_CONTROLSS_CTRL_ADC4_CLK_GATE                                       (0x0000026CU)
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE                                      (0x00000270U)
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE                                      (0x00000274U)
#define CSL_CONTROLSS_CTRL_OTTO2_CLK_GATE                                      (0x00000278U)
#define CSL_CONTROLSS_CTRL_OTTO3_CLK_GATE                                      (0x0000027CU)
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE                                  (0x00000280U)
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE                                  (0x00000284U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE                                (0x00000288U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_PLL_CLK_GATE                                (0x0000028CU)
#define CSL_CONTROLSS_CTRL_FSI_TX2_PLL_CLK_GATE                                (0x00000290U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_PLL_CLK_GATE                                (0x00000294U)
#define CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE                                (0x00000298U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE                                (0x0000029CU)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE                                (0x000002A0U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE                                (0x000002A4U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_CLK_GATE                                (0x000002A8U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE                                (0x000002ACU)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE                                (0x000002B0U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_CLK_GATE                                (0x000002B4U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_CLK_GATE                                (0x000002B8U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_CLK_GATE                                (0x000002BCU)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_CLK_GATE                                (0x000002C0U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_CLK_GATE                               (0x000002C4U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_CLK_GATE                               (0x000002C8U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE                                   (0x000002DCU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE                             (0x000002E0U)
#define CSL_CONTROLSS_CTRL_ADCR0_CLK_GATE                                      (0x000002E4U)
#define CSL_CONTROLSS_CTRL_ADCR1_CLK_GATE                                      (0x000002E8U)
#define CSL_CONTROLSS_CTRL_ETPWM0_RST                                          (0x00000300U)
#define CSL_CONTROLSS_CTRL_ETPWM1_RST                                          (0x00000304U)
#define CSL_CONTROLSS_CTRL_ETPWM2_RST                                          (0x00000308U)
#define CSL_CONTROLSS_CTRL_ETPWM3_RST                                          (0x0000030CU)
#define CSL_CONTROLSS_CTRL_ETPWM4_RST                                          (0x00000310U)
#define CSL_CONTROLSS_CTRL_ETPWM5_RST                                          (0x00000314U)
#define CSL_CONTROLSS_CTRL_ETPWM6_RST                                          (0x00000318U)
#define CSL_CONTROLSS_CTRL_ETPWM7_RST                                          (0x0000031CU)
#define CSL_CONTROLSS_CTRL_ETPWM8_RST                                          (0x00000320U)
#define CSL_CONTROLSS_CTRL_ETPWM9_RST                                          (0x00000324U)
#define CSL_CONTROLSS_CTRL_ETPWM10_RST                                         (0x00000328U)
#define CSL_CONTROLSS_CTRL_ETPWM11_RST                                         (0x0000032CU)
#define CSL_CONTROLSS_CTRL_ETPWM12_RST                                         (0x00000330U)
#define CSL_CONTROLSS_CTRL_ETPWM13_RST                                         (0x00000334U)
#define CSL_CONTROLSS_CTRL_ETPWM14_RST                                         (0x00000338U)
#define CSL_CONTROLSS_CTRL_ETPWM15_RST                                         (0x0000033CU)
#define CSL_CONTROLSS_CTRL_ETPWM16_RST                                         (0x00000340U)
#define CSL_CONTROLSS_CTRL_ETPWM17_RST                                         (0x00000344U)
#define CSL_CONTROLSS_CTRL_ETPWM18_RST                                         (0x00000348U)
#define CSL_CONTROLSS_CTRL_ETPWM19_RST                                         (0x0000034CU)
#define CSL_CONTROLSS_CTRL_ETPWM20_RST                                         (0x00000350U)
#define CSL_CONTROLSS_CTRL_ETPWM21_RST                                         (0x00000354U)
#define CSL_CONTROLSS_CTRL_ETPWM22_RST                                         (0x00000358U)
#define CSL_CONTROLSS_CTRL_ETPWM23_RST                                         (0x0000035CU)
#define CSL_CONTROLSS_CTRL_ETPWM24_RST                                         (0x00000360U)
#define CSL_CONTROLSS_CTRL_ETPWM25_RST                                         (0x00000364U)
#define CSL_CONTROLSS_CTRL_ETPWM26_RST                                         (0x00000368U)
#define CSL_CONTROLSS_CTRL_ETPWM27_RST                                         (0x0000036CU)
#define CSL_CONTROLSS_CTRL_ETPWM28_RST                                         (0x00000370U)
#define CSL_CONTROLSS_CTRL_ETPWM29_RST                                         (0x00000374U)
#define CSL_CONTROLSS_CTRL_ETPWM30_RST                                         (0x00000378U)
#define CSL_CONTROLSS_CTRL_ETPWM31_RST                                         (0x0000037CU)
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST                                         (0x00000380U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_RST                                         (0x00000384U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_RST                                         (0x00000388U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_RST                                         (0x0000038CU)
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST                                         (0x00000390U)
#define CSL_CONTROLSS_CTRL_FSI_RX1_RST                                         (0x00000394U)
#define CSL_CONTROLSS_CTRL_FSI_RX2_RST                                         (0x00000398U)
#define CSL_CONTROLSS_CTRL_FSI_RX3_RST                                         (0x0000039CU)
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST                                       (0x000003A0U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST                                       (0x000003A4U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST                                       (0x000003A8U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST                                       (0x000003ACU)
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST                                       (0x000003B0U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST                                       (0x000003B4U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST                                       (0x000003B8U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST                                       (0x000003BCU)
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST                                       (0x000003C0U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_RST                                       (0x000003C4U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_RST                                        (0x000003D0U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_RST                                        (0x000003D4U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_RST                                        (0x000003D8U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_RST                                        (0x000003DCU)
#define CSL_CONTROLSS_CTRL_CMPSSB4_RST                                        (0x000003E0U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_RST                                        (0x000003E4U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_RST                                        (0x000003E8U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_RST                                        (0x000003ECU)
#define CSL_CONTROLSS_CTRL_CMPSSB8_RST                                        (0x000003F0U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_RST                                        (0x000003F4U)
#define CSL_CONTROLSS_CTRL_ECAP0_RST                                           (0x00000400U)
#define CSL_CONTROLSS_CTRL_ECAP1_RST                                           (0x00000404U)
#define CSL_CONTROLSS_CTRL_ECAP2_RST                                           (0x00000408U)
#define CSL_CONTROLSS_CTRL_ECAP3_RST                                           (0x0000040CU)
#define CSL_CONTROLSS_CTRL_ECAP4_RST                                           (0x00000410U)
#define CSL_CONTROLSS_CTRL_ECAP5_RST                                           (0x00000414U)
#define CSL_CONTROLSS_CTRL_ECAP6_RST                                           (0x00000418U)
#define CSL_CONTROLSS_CTRL_ECAP7_RST                                           (0x0000041CU)
#define CSL_CONTROLSS_CTRL_ECAP8_RST                                           (0x00000420U)
#define CSL_CONTROLSS_CTRL_ECAP9_RST                                           (0x00000424U)
#define CSL_CONTROLSS_CTRL_ECAP10_RST                                          (0x00000428U)
#define CSL_CONTROLSS_CTRL_ECAP11_RST                                          (0x0000042CU)
#define CSL_CONTROLSS_CTRL_ECAP12_RST                                          (0x00000430U)
#define CSL_CONTROLSS_CTRL_ECAP13_RST                                          (0x00000434U)
#define CSL_CONTROLSS_CTRL_ECAP14_RST                                          (0x00000438U)
#define CSL_CONTROLSS_CTRL_ECAP15_RST                                          (0x0000043CU)
#define CSL_CONTROLSS_CTRL_EQEP0_RST                                           (0x00000440U)
#define CSL_CONTROLSS_CTRL_EQEP1_RST                                           (0x00000444U)
#define CSL_CONTROLSS_CTRL_EQEP2_RST                                           (0x00000448U)
#define CSL_CONTROLSS_CTRL_SDFM0_RST                                           (0x00000450U)
#define CSL_CONTROLSS_CTRL_SDFM1_RST                                           (0x00000454U)
#define CSL_CONTROLSS_CTRL_DAC_RST                                             (0x00000458U)
#define CSL_CONTROLSS_CTRL_ADC0_RST                                            (0x0000045CU)
#define CSL_CONTROLSS_CTRL_ADC1_RST                                            (0x00000460U)
#define CSL_CONTROLSS_CTRL_ADC2_RST                                            (0x00000464U)
#define CSL_CONTROLSS_CTRL_ADC3_RST                                            (0x00000468U)
#define CSL_CONTROLSS_CTRL_ADC4_RST                                            (0x0000046CU)
#define CSL_CONTROLSS_CTRL_OTTO0_RST                                           (0x00000470U)
#define CSL_CONTROLSS_CTRL_OTTO1_RST                                           (0x00000474U)
#define CSL_CONTROLSS_CTRL_OTTO2_RST                                           (0x00000478U)
#define CSL_CONTROLSS_CTRL_OTTO3_RST                                           (0x0000047CU)
#define CSL_CONTROLSS_CTRL_HW_RESOLVER_RST                                     (0x00000480U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST                                     (0x00000484U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST                                     (0x00000488U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST                                     (0x0000048CU)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST                                     (0x00000490U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST                                     (0x00000494U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST                                     (0x00000498U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_RST                                     (0x0000049CU)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_RST                                     (0x000004A0U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_RST                                     (0x000004A4U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_RST                                     (0x000004A8U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_RST                                    (0x000004ACU)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_RST                                    (0x000004B0U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST                                        (0x000004C4U)
#define CSL_CONTROLSS_CTRL_ADCR0_RST                                           (0x000004C8U)
#define CSL_CONTROLSS_CTRL_ADCR1_RST                                           (0x000004CCU)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN                                        (0x00000500U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN                                        (0x00000504U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN                                        (0x00000508U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN                                        (0x0000050CU)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN                                        (0x00000510U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN                                        (0x00000514U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN                                        (0x00000518U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN                                        (0x0000051CU)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN                                        (0x00000520U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN                                        (0x00000524U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN                                       (0x00000528U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN                                       (0x0000052CU)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN                                       (0x00000530U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN                                       (0x00000534U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN                                       (0x00000538U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN                                       (0x0000053CU)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN                                       (0x00000540U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN                                       (0x00000544U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN                                       (0x00000548U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN                                       (0x0000054CU)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN                                       (0x00000550U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN                                       (0x00000554U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN                                       (0x00000558U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN                                       (0x0000055CU)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN                                       (0x00000560U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN                                       (0x00000564U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN                                       (0x00000568U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN                                       (0x0000056CU)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN                                       (0x00000570U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN                                       (0x00000574U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN                                       (0x00000578U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN                                       (0x0000057CU)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN                                    (0x00000580U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN                                    (0x00000584U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN                                    (0x00000588U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN                                    (0x0000058CU)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN                                    (0x00000590U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN                                    (0x00000594U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN                                    (0x00000598U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN                                    (0x0000059CU)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN                                    (0x000005A0U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN                                    (0x000005A4U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN                                     (0x000005A8U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN                                     (0x000005ACU)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN                                     (0x000005B0U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN                                     (0x000005B4U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN                                     (0x000005B8U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN                                     (0x000005BCU)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN                                     (0x000005C0U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN                                     (0x000005C4U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN                                     (0x000005C8U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN                                     (0x000005CCU)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN                                        (0x000005D0U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN                                        (0x000005D4U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN                                        (0x000005D8U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN                                        (0x000005DCU)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN                                        (0x000005E0U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN                                        (0x000005E4U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN                                        (0x000005E8U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN                                        (0x000005ECU)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN                                        (0x000005F0U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN                                        (0x000005F4U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN                                        (0x000005F8U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN                                        (0x000005FCU)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN                                        (0x00000600U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN                                       (0x00000604U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN                                       (0x00000608U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN                                       (0x0000060CU)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN                                       (0x00000610U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN                                       (0x00000614U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN                                       (0x00000618U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK0                                         (0x00001008U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK1                                         (0x0000100CU)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS                                     (0x00001010U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR                           (0x00001014U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE                                         (0x00001018U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR                                   (0x0000101CU)
#define CSL_CONTROLSS_CTRL_EOI                                                 (0x00001020U)
#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS                                       (0x00001024U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS                                   (0x00001028U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS                                   (0x0000102CU)
#define CSL_CONTROLSS_CTRL_FAULT_CLEAR                                         (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_CONTROLSS_CTRL_PID_PID_MINOR_MASK                                  (0x0000003FU)
#define CSL_CONTROLSS_CTRL_PID_PID_MINOR_SHIFT                                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_PID_PID_MINOR_RESETVAL                              (0x00000014U)
#define CSL_CONTROLSS_CTRL_PID_PID_MINOR_MAX                                   (0x0000003FU)

#define CSL_CONTROLSS_CTRL_PID_PID_CUSTOM_MASK                                 (0x000000C0U)
#define CSL_CONTROLSS_CTRL_PID_PID_CUSTOM_SHIFT                                (0x00000006U)
#define CSL_CONTROLSS_CTRL_PID_PID_CUSTOM_RESETVAL                             (0x00000000U)
#define CSL_CONTROLSS_CTRL_PID_PID_CUSTOM_MAX                                  (0x00000003U)

#define CSL_CONTROLSS_CTRL_PID_PID_MAJOR_MASK                                  (0x00000700U)
#define CSL_CONTROLSS_CTRL_PID_PID_MAJOR_SHIFT                                 (0x00000008U)
#define CSL_CONTROLSS_CTRL_PID_PID_MAJOR_RESETVAL                              (0x00000002U)
#define CSL_CONTROLSS_CTRL_PID_PID_MAJOR_MAX                                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_PID_PID_MISC_MASK                                   (0x0000F800U)
#define CSL_CONTROLSS_CTRL_PID_PID_MISC_SHIFT                                  (0x0000000BU)
#define CSL_CONTROLSS_CTRL_PID_PID_MISC_RESETVAL                               (0x00000000U)
#define CSL_CONTROLSS_CTRL_PID_PID_MISC_MAX                                    (0x0000001FU)

#define CSL_CONTROLSS_CTRL_PID_PID_MSB16_MASK                                  (0xFFFF0000U)
#define CSL_CONTROLSS_CTRL_PID_PID_MSB16_SHIFT                                 (0x00000010U)
#define CSL_CONTROLSS_CTRL_PID_PID_MSB16_RESETVAL                              (0x00006180U)
#define CSL_CONTROLSS_CTRL_PID_PID_MSB16_MAX                                   (0x0000FFFFU)

#define CSL_CONTROLSS_CTRL_PID_RESETVAL                                        (0x61800214U)

/* EPWM_STATICXBAR_SEL0 */

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM0_MASK              (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM0_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM0_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM1_MASK              (0x0000000CU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM1_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM1_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM2_MASK              (0x00000030U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM2_SHIFT             (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM2_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM2_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM3_MASK              (0x000000C0U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM3_SHIFT             (0x00000006U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM3_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM3_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM4_MASK              (0x00000300U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM4_SHIFT             (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM4_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM4_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM5_MASK              (0x00000C00U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM5_SHIFT             (0x0000000AU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM5_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM5_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM6_MASK              (0x00003000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM6_SHIFT             (0x0000000CU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM6_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM6_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM7_MASK              (0x0000C000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM7_SHIFT             (0x0000000EU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM7_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM7_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM8_MASK              (0x00030000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM8_SHIFT             (0x00000010U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM8_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM8_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM9_MASK              (0x000C0000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM9_SHIFT             (0x00000012U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM9_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM9_MAX               (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM10_MASK             (0x00300000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM10_SHIFT            (0x00000014U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM10_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM10_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM11_MASK             (0x00C00000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM11_SHIFT            (0x00000016U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM11_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM11_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM12_MASK             (0x03000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM12_SHIFT            (0x00000018U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM12_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM12_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM13_MASK             (0x0C000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM13_SHIFT            (0x0000001AU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM13_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM13_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM14_MASK             (0x30000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM14_SHIFT            (0x0000001CU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM14_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM14_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM15_MASK             (0xC0000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM15_SHIFT            (0x0000001EU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM15_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_ETPWM15_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL0_RESETVAL                       (0x00000000U)

/* EPWM_STATICXBAR_SEL1 */

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM16_MASK             (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM16_SHIFT            (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM16_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM16_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM17_MASK             (0x0000000CU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM17_SHIFT            (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM17_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM17_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM18_MASK             (0x00000030U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM18_SHIFT            (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM18_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM18_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM19_MASK             (0x000000C0U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM19_SHIFT            (0x00000006U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM19_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM19_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM20_MASK             (0x00000300U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM20_SHIFT            (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM20_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM20_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM21_MASK             (0x00000C00U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM21_SHIFT            (0x0000000AU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM21_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM21_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM22_MASK             (0x00003000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM22_SHIFT            (0x0000000CU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM22_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM22_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM23_MASK             (0x0000C000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM23_SHIFT            (0x0000000EU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM23_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM23_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM24_MASK             (0x00030000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM24_SHIFT            (0x00000010U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM24_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM24_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM25_MASK             (0x000C0000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM25_SHIFT            (0x00000012U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM25_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM25_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM26_MASK             (0x00300000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM26_SHIFT            (0x00000014U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM26_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM26_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM27_MASK             (0x00C00000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM27_SHIFT            (0x00000016U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM27_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM27_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM28_MASK             (0x03000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM28_SHIFT            (0x00000018U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM28_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM28_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM29_MASK             (0x0C000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM29_SHIFT            (0x0000001AU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM29_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM29_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM30_MASK             (0x30000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM30_SHIFT            (0x0000001CU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM30_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM30_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM31_MASK             (0xC0000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM31_SHIFT            (0x0000001EU)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM31_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_ETPWM31_MAX              (0x00000003U)

#define CSL_CONTROLSS_CTRL_EPWM_STATICXBAR_SEL1_RESETVAL                       (0x00000000U)

/* EPWM_CLKSYNC */

#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MASK                         (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_SHIFT                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_RESETVAL                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MAX                          (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_RESETVAL                               (0x00000000U)

/* SDFM1_CLK0_SEL */

#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_SEL_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_SEL_RESETVAL                             (0x00000000U)

/* EMUSTOPN_MASK */

#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EMUSTOPN_MASK_RESETVAL                              (0x00000000U)

/* CLB_AQ_EN0 */

#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_ENABLE_MASK                        (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_ENABLE_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_ENABLE_MAX                         (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_CLB_AQ_EN0_RESETVAL                                 (0x00000000U)

/* CLB_AQ_EN1 */

#define CSL_CONTROLSS_CTRL_CLB_AQ_EN1_ENABLE_MASK                        (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN1_ENABLE_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN1_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_AQ_EN1_ENABLE_MAX                         (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_CLB_AQ_EN1_RESETVAL                                 (0x00000000U)

/* CLB_DB_EN0 */

#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_ENABLE_MASK                        (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_ENABLE_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_ENABLE_MAX                         (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_CLB_DB_EN0_RESETVAL                                 (0x00000000U)

/* CLB_DB_EN1 */

#define CSL_CONTROLSS_CTRL_CLB_DB_EN1_ENABLE_MASK                        (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN1_ENABLE_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN1_ENABLE_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CLB_DB_EN1_ENABLE_MAX                         (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_CLB_DB_EN1_RESETVAL                                 (0x00000000U)

/* ADCSOCFRCGBSEL */

#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_MASK           (0x0000007FU)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_SHIFT          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_RESETVAL       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_ENABLE_MAX            (0x0000007FU)

#define CSL_CONTROLSS_CTRL_ADCSOCFRCGBSEL_RESETVAL                             (0x00000000U)

/* ADCSOCFRCGB */

#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_MASK                   (0x0000FFFFU)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_TRIG_MAX                    (0x0000FFFFU)

#define CSL_CONTROLSS_CTRL_ADCSOCFRCGB_RESETVAL                                (0x00000000U)

/* ADC_EXTCH_DLY_SEL */

#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_SEL_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_ADC_EXTCH_DLY_SEL_RESETVAL                          (0x00000000U)

/* XBAR_LOOPBACK_CTRL */

#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_ENABLE_MASK   (0x0000FFFFU)
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_ENABLE_SHIFT  (0x00000000U)
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_ENABLE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_ENABLE_MAX    (0x0000FFFFU)

#define CSL_CONTROLSS_CTRL_XBAR_LOOPBACK_CTRL_RESETVAL                         (0x00000000U)

/* EPWM_SOCA_SEL */

#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_SEL_MASK                (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_SEL_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_SEL_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_SEL_MAX                 (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_EPWM_SOCA_SEL_RESETVAL                              (0x00000000U)

/* EPWM_SOCB_SEL */

#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_SEL_MASK                (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_SEL_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_SEL_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_SEL_MAX                 (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_EPWM_SOCB_SEL_RESETVAL                              (0x00000000U)

/* ADCEXTCHXBAR0_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR0_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR1_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR1_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR2_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR2_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR3_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR3_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR4_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR4_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR5_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR5_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR6_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR6_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR7_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR7_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR8_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR8_G0_SEL_RESETVAL                       (0x00000000U)

/* ADCEXTCHXBAR9_G0_SEL */

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_SEL_MASK  (0x0000000FU)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_SEL_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_SEL_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_SEL_MAX   (0x0000000FU)

#define CSL_CONTROLSS_CTRL_ADCEXTCHXBAR9_G0_SEL_RESETVAL                       (0x00000000U)

/* SDFM0_CLK0_OUT_SEL */

#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_MASK      (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_SEL_MAX       (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM0_CLK0_OUT_SEL_RESETVAL                         (0x00000000U)

/* SDFM0_CLK1_OUT_SEL */

#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_SEL_MASK      (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_SEL_MAX       (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM0_CLK1_OUT_SEL_RESETVAL                         (0x00000000U)

/* SDFM0_CLK2_OUT_SEL */

#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_SEL_MASK      (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_SEL_MAX       (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM0_CLK2_OUT_SEL_RESETVAL                         (0x00000000U)

/* SDFM0_CLK3_OUT_SEL */

#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_SEL_MASK      (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_SEL_MAX       (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM0_CLK3_OUT_SEL_RESETVAL                         (0x00000000U)

/* SDFM1_CLK0_OUT_SEL */

#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_SEL_MASK      (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_SEL_MAX       (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM1_CLK0_OUT_SEL_RESETVAL                         (0x00000000U)

/* SDFM1_CLK1_OUT_SEL */

#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_SEL_MASK      (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_SEL_MAX       (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM1_CLK1_OUT_SEL_RESETVAL                         (0x00000000U)

/* SDFM1_CLK2_OUT_SEL */

#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_SEL_MASK      (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_SEL_MAX       (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM1_CLK2_OUT_SEL_RESETVAL                         (0x00000000U)

/* SDFM1_CLK3_OUT_SEL */

#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_SEL_MASK      (0x00000001U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_SEL_SHIFT     (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_SEL_RESETVAL  (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_SEL_MAX       (0x00000001U)

#define CSL_CONTROLSS_CTRL_SDFM1_CLK3_OUT_SEL_RESETVAL                         (0x00000000U)

/* CONTROLSS_G0_EPWM_WLINK */

#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_ENABLE_MASK (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_ENABLE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_ENABLE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_ENABLE_MAX (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_CONTROLSS_G0_EPWM_WLINK_RESETVAL                    (0x00000000U)

/* CONTROLSS_G1_EPWM_WLINK */

#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_ENABLE_MASK (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_ENABLE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_ENABLE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_ENABLE_MAX (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_CONTROLSS_G1_EPWM_WLINK_RESETVAL                    (0x00000000U)

/* CONTROLSS_G2_EPWM_WLINK */

#define CSL_CONTROLSS_CTRL_CONTROLSS_G2_EPWM_WLINK_ENABLE_MASK (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G2_EPWM_WLINK_ENABLE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G2_EPWM_WLINK_ENABLE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G2_EPWM_WLINK_ENABLE_MAX (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_CONTROLSS_G2_EPWM_WLINK_RESETVAL                    (0x00000000U)

/* CONTROLSS_G3_EPWM_WLINK */

#define CSL_CONTROLSS_CTRL_CONTROLSS_G3_EPWM_WLINK_ENABLE_MASK (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G3_EPWM_WLINK_ENABLE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G3_EPWM_WLINK_ENABLE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_G3_EPWM_WLINK_ENABLE_MAX (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_CONTROLSS_G3_EPWM_WLINK_RESETVAL                    (0x00000000U)

/* ETPWM0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM0_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM1_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM2_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM3_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM4_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM4_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM5_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM5_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM6_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM6_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM7_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM7_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM8_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM8_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM9_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_CLK_GATE_MASK                 (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_CLK_GATE_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_CLK_GATE_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_CLK_GATE_MAX                  (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM9_CLK_GATE_RESETVAL                      (0x00000000U)

/* ETPWM10_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM10_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM10_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM10_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM10_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM10_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM11_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM11_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM11_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM11_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM11_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM11_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM12_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM12_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM12_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM12_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM12_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM12_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM13_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM13_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM13_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM13_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM13_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM13_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM14_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM14_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM14_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM14_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM14_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM14_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM15_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM15_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM15_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM15_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM15_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM15_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM16_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM16_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM16_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM16_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM16_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM16_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM17_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM17_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM17_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM17_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM17_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM17_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM18_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM18_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM18_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM18_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM18_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM18_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM19_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM19_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM19_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM19_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM19_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM19_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM20_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM20_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM20_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM20_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM20_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM20_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM21_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM21_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM21_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM21_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM21_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM21_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM22_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM22_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM22_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM22_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM22_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM22_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM23_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM23_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM23_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM23_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM23_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM23_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM24_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM24_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM24_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM24_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM24_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM24_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM25_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM25_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM25_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM25_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM25_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM25_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM26_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM26_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM26_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM26_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM26_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM26_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM27_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM27_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM27_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM27_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM27_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM27_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM28_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM28_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM28_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM28_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM28_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM28_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM29_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM29_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM29_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM29_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM29_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM29_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM30_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM30_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM30_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM30_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM30_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM30_CLK_GATE_RESETVAL                     (0x00000000U)

/* ETPWM31_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ETPWM31_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM31_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM31_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM31_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM31_CLK_GATE_RESETVAL                     (0x00000000U)

/* FSI_TX0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX0_CLK_GATE_RESETVAL                     (0x00000000U)

/* FSI_TX1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_TX1_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX1_CLK_GATE_RESETVAL                     (0x00000000U)

/* FSI_TX2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_TX2_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX2_CLK_GATE_RESETVAL                     (0x00000000U)

/* FSI_TX3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_TX3_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX3_CLK_GATE_RESETVAL                     (0x00000000U)

/* FSI_RX0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_RX0_CLK_GATE_RESETVAL                     (0x00000000U)

/* FSI_RX1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_RX1_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX1_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX1_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX1_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_RX1_CLK_GATE_RESETVAL                     (0x00000000U)

/* FSI_RX2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_RX2_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX2_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX2_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX2_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_RX2_CLK_GATE_RESETVAL                     (0x00000000U)

/* FSI_RX3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_RX3_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX3_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX3_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX3_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_RX3_CLK_GATE_RESETVAL                           (0x00000000U)

/* CMPSSA0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA0_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA1_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA2_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA3_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA4_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA4_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA5_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA5_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA6_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA6_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA7_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA7_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA8_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA8_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSA9_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSA9_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA9_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB0_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB1_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB1_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB2_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB2_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB3_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB3_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB4_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB4_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB4_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB5_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB5_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB5_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB6_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB6_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB6_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB7_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB7_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB7_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB8_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB8_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB8_CLK_GATE_RESETVAL                     (0x00000000U)

/* CMPSSB9_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CMPSSB9_CLK_GATE_CLK_GATE_MASK                (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_CLK_GATE_CLK_GATE_SHIFT               (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_CLK_GATE_CLK_GATE_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_CLK_GATE_CLK_GATE_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB9_CLK_GATE_RESETVAL                     (0x00000000U)

/* ECAP0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP0_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP1_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP2_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP3_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP4_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP4_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP5_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP5_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP6_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP6_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP7_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP7_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP8_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP8_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP8_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP8_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP9_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP9_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP9_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP9_CLK_GATE_RESETVAL                       (0x00000000U)

/* ECAP10_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP10_CLK_GATE_CLK_GATE_MASK       (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP10_CLK_GATE_CLK_GATE_SHIFT      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_CLK_GATE_CLK_GATE_RESETVAL   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_CLK_GATE_CLK_GATE_MAX        (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP10_CLK_GATE_RESETVAL                            (0x00000000U)

/* ECAP11_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP11_CLK_GATE_CLK_GATE_MASK       (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP11_CLK_GATE_CLK_GATE_SHIFT      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_CLK_GATE_CLK_GATE_RESETVAL   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_CLK_GATE_CLK_GATE_MAX        (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP11_CLK_GATE_RESETVAL                            (0x00000000U)

/* ECAP12_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP12_CLK_GATE_CLK_GATE_MASK       (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP12_CLK_GATE_CLK_GATE_SHIFT      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_CLK_GATE_CLK_GATE_RESETVAL   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_CLK_GATE_CLK_GATE_MAX        (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP12_CLK_GATE_RESETVAL                            (0x00000000U)

/* ECAP13_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP13_CLK_GATE_CLK_GATE_MASK       (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP13_CLK_GATE_CLK_GATE_SHIFT      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_CLK_GATE_CLK_GATE_RESETVAL   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_CLK_GATE_CLK_GATE_MAX        (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP13_CLK_GATE_RESETVAL                            (0x00000000U)

/* ECAP14_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP14_CLK_GATE_CLK_GATE_MASK       (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP14_CLK_GATE_CLK_GATE_SHIFT      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_CLK_GATE_CLK_GATE_RESETVAL   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_CLK_GATE_CLK_GATE_MAX        (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP14_CLK_GATE_RESETVAL                            (0x00000000U)

/* ECAP15_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ECAP15_CLK_GATE_CLK_GATE_MASK       (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP15_CLK_GATE_CLK_GATE_SHIFT      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_CLK_GATE_CLK_GATE_RESETVAL   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_CLK_GATE_CLK_GATE_MAX        (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP15_CLK_GATE_RESETVAL                            (0x00000000U)

/* EQEP0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_EQEP0_CLK_GATE_RESETVAL                       (0x00000000U)

/* EQEP1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_EQEP1_CLK_GATE_RESETVAL                       (0x00000000U)

/* EQEP2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_EQEP2_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP2_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_EQEP2_CLK_GATE_RESETVAL                       (0x00000000U)

/* SDFM0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_SDFM0_CLK_GATE_RESETVAL                       (0x00000000U)

/* SDFM1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_SDFM1_CLK_GATE_RESETVAL                       (0x00000000U)

/* DAC_CLK_GATE */

#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_MASK                    (0x00000007U)
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_SHIFT                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_CLK_GATE_MAX                     (0x00000007U)

#define CSL_CONTROLSS_CTRL_DAC_CLK_GATE_RESETVAL                         (0x00000000U)

/* ADC0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC0_CLK_GATE_RESETVAL                        (0x00000000U)

/* ADC1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC1_CLK_GATE_RESETVAL                        (0x00000000U)

/* ADC2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC2_CLK_GATE_RESETVAL                        (0x00000000U)

/* ADC3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC3_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC3_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC3_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC3_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC3_CLK_GATE_RESETVAL                        (0x00000000U)

/* ADC4_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC4_CLK_GATE_CLK_GATE_MASK                   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC4_CLK_GATE_CLK_GATE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC4_CLK_GATE_CLK_GATE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC4_CLK_GATE_CLK_GATE_MAX                    (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC4_CLK_GATE_RESETVAL                        (0x00000000U)

/* OTTO0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_OTTO0_CLK_GATE_RESETVAL                       (0x00000000U)

/* OTTO1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_OTTO1_CLK_GATE_RESETVAL                       (0x00000000U)

/* OTTO2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_OTTO2_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO2_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO2_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO2_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_OTTO2_CLK_GATE_RESETVAL                       (0x00000000U)

/* OTTO3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_OTTO3_CLK_GATE_CLK_GATE_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO3_CLK_GATE_CLK_GATE_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO3_CLK_GATE_CLK_GATE_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO3_CLK_GATE_CLK_GATE_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_OTTO3_CLK_GATE_RESETVAL                       (0x00000000U)

/* SDFM0_PLL_CLK_GATE */

#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_MASK              (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_CLK_GATE_MAX               (0x00000007U)

#define CSL_CONTROLSS_CTRL_SDFM0_PLL_CLK_GATE_RESETVAL                   (0x00000000U)

/* SDFM1_PLL_CLK_GATE */

#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_CLK_GATE_MASK              (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_CLK_GATE_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_CLK_GATE_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_CLK_GATE_MAX               (0x00000007U)

#define CSL_CONTROLSS_CTRL_SDFM1_PLL_CLK_GATE_RESETVAL                   (0x00000000U)

/* FSI_TX0_PLL_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_CLK_GATE_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX0_PLL_CLK_GATE_RESETVAL                 (0x00000000U)

/* FSI_TX1_PLL_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_TX1_PLL_CLK_GATE_CLK_GATE_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_PLL_CLK_GATE_CLK_GATE_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_PLL_CLK_GATE_CLK_GATE_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_PLL_CLK_GATE_CLK_GATE_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX1_PLL_CLK_GATE_RESETVAL                 (0x00000000U)

/* FSI_TX2_PLL_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_TX2_PLL_CLK_GATE_CLK_GATE_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_PLL_CLK_GATE_CLK_GATE_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_PLL_CLK_GATE_CLK_GATE_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_PLL_CLK_GATE_CLK_GATE_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX2_PLL_CLK_GATE_RESETVAL                 (0x00000000U)

/* FSI_TX3_PLL_CLK_GATE */

#define CSL_CONTROLSS_CTRL_FSI_TX3_PLL_CLK_GATE_CLK_GATE_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_PLL_CLK_GATE_CLK_GATE_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_PLL_CLK_GATE_CLK_GATE_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_PLL_CLK_GATE_CLK_GATE_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX3_PLL_CLK_GATE_RESETVAL                 (0x00000000U)

/* HW_RESOLVER_CLK_GATE */

#define CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_HW_RESOLVER_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE2_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE3_CLK_GATE */

#define CSL_CONTROLSS_CTRL_SCTILE3_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_SCTILE3_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_SCTILE3_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_SCTILE3_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE4_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE5_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_ADC_SCTILE5_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_ADC_SCTILE5_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_ADC_SCTILE5_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_ADC_SCTILE5_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE6_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_CLK_GATE_ADC_SCTILE6_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_CLK_GATE_ADC_SCTILE6_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_CLK_GATE_ADC_SCTILE6_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_CLK_GATE_ADC_SCTILE6_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE7_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_CLK_GATE_ADC_SCTILE7_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_CLK_GATE_ADC_SCTILE7_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_CLK_GATE_ADC_SCTILE7_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_CLK_GATE_ADC_SCTILE7_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE8_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_CLK_GATE_ADC_SCTILE8_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_CLK_GATE_ADC_SCTILE8_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_CLK_GATE_ADC_SCTILE8_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_CLK_GATE_ADC_SCTILE8_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE9_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_CLK_GATE_ADC_SCTILE9_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_CLK_GATE_ADC_SCTILE9_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_CLK_GATE_ADC_SCTILE9_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_CLK_GATE_ADC_SCTILE9_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_CLK_GATE_RESETVAL                       (0x00000000U)

/* ADC_SCTILE10_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_CLK_GATE_ADC_SCTILE10_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_CLK_GATE_ADC_SCTILE10_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_CLK_GATE_ADC_SCTILE10_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_CLK_GATE_ADC_SCTILE10_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_CLK_GATE_RESETVAL                      (0x00000000U)

/* ADC_SCTILE11_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_CLK_GATE_ADC_SCTILE11_CLK_GATE_CLK_GATE_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_CLK_GATE_ADC_SCTILE11_CLK_GATE_CLK_GATE_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_CLK_GATE_ADC_SCTILE11_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_CLK_GATE_ADC_SCTILE11_CLK_GATE_CLK_GATE_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_CLK_GATE_RESETVAL                      (0x00000000U)

/* ADC_AGG0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_ADC_AGG0_CLK_GATE_CLK_GATE_MASK   (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_ADC_AGG0_CLK_GATE_CLK_GATE_SHIFT  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_ADC_AGG0_CLK_GATE_CLK_GATE_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_ADC_AGG0_CLK_GATE_CLK_GATE_MAX    (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_AGG0_CLK_GATE_RESETVAL                          (0x00000000U)

/* CONTROLSS_XBAR_CLK_GATE */

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_INPUTXBAR_MASK (0x00000007U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_INPUTXBAR_SHIFT (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_INPUTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_INPUTXBAR_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_PWMXBAR_MASK (0x00000070U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_PWMXBAR_SHIFT (0x00000004U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_PWMXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_PWMXBAR_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_MDLXBAR_MASK (0x00000700U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_MDLXBAR_SHIFT (0x00000008U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_MDLXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_MDLXBAR_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_ICLXBAR_MASK (0x00007000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_ICLXBAR_SHIFT (0x0000000CU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_ICLXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_ICLXBAR_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_INTXBAR_MASK (0x00070000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_INTXBAR_SHIFT (0x00000010U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_INTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_INTXBAR_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_DMAXBAR_MASK (0x00700000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_DMAXBAR_SHIFT (0x00000014U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_DMAXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_DMAXBAR_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_OUTPUTXBAR_MASK (0x07000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_OUTPUTXBAR_SHIFT (0x00000018U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_OUTPUTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_OUTPUTXBAR_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_PWMSYNCOUTXBAR_MASK (0x70000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_PWMSYNCOUTXBAR_SHIFT (0x0000001CU)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_PWMSYNCOUTXBAR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_CONTROLSS_XBAR_CLK_GATE_PWMSYNCOUTXBAR_MAX (0x00000007U)

#define CSL_CONTROLSS_CTRL_CONTROLSS_XBAR_CLK_GATE_RESETVAL                    (0x00000000U)

/* ADCR0_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADCR0_CLK_GATE_ADCR0_CLK_GATE_CLK_GATE_MASK         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADCR0_CLK_GATE_ADCR0_CLK_GATE_CLK_GATE_SHIFT        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCR0_CLK_GATE_ADCR0_CLK_GATE_CLK_GATE_RESETVAL     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCR0_CLK_GATE_ADCR0_CLK_GATE_CLK_GATE_MAX          (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADCR0_CLK_GATE_RESETVAL                             (0x00000000U)

/* ADCR1_CLK_GATE */

#define CSL_CONTROLSS_CTRL_ADCR1_CLK_GATE_ADCR1_CLK_GATE_CLK_GATE_MASK         (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADCR1_CLK_GATE_ADCR1_CLK_GATE_CLK_GATE_SHIFT        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCR1_CLK_GATE_ADCR1_CLK_GATE_CLK_GATE_RESETVAL     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCR1_CLK_GATE_ADCR1_CLK_GATE_CLK_GATE_MAX          (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADCR1_CLK_GATE_RESETVAL                             (0x00000000U)

/* ETPWM0_RST */

#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM0_RST_RESETVAL                           (0x00000000U)

/* ETPWM1_RST */

#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM1_RST_RESETVAL                           (0x00000000U)

/* ETPWM2_RST */

#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM2_RST_RESETVAL                           (0x00000000U)

/* ETPWM3_RST */

#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM3_RST_RESETVAL                           (0x00000000U)

/* ETPWM4_RST */

#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM4_RST_RESETVAL                           (0x00000000U)

/* ETPWM5_RST */

#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM5_RST_RESETVAL                           (0x00000000U)

/* ETPWM6_RST */

#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM6_RST_RESETVAL                           (0x00000000U)

/* ETPWM7_RST */

#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM7_RST_RESETVAL                           (0x00000000U)

/* ETPWM8_RST */

#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM8_RST_RESETVAL                           (0x00000000U)

/* ETPWM9_RST */

#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RST_MASK                           (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RST_SHIFT                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RST_RESETVAL                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RST_MAX                            (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM9_RST_RESETVAL                           (0x00000000U)

/* ETPWM10_RST */

#define CSL_CONTROLSS_CTRL_ETPWM10_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM10_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM10_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM10_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM10_RST_RESETVAL                          (0x00000000U)

/* ETPWM11_RST */

#define CSL_CONTROLSS_CTRL_ETPWM11_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM11_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM11_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM11_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM11_RST_RESETVAL                          (0x00000000U)

/* ETPWM12_RST */

#define CSL_CONTROLSS_CTRL_ETPWM12_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM12_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM12_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM12_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM12_RST_RESETVAL                          (0x00000000U)

/* ETPWM13_RST */

#define CSL_CONTROLSS_CTRL_ETPWM13_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM13_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM13_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM13_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM13_RST_RESETVAL                          (0x00000000U)

/* ETPWM14_RST */

#define CSL_CONTROLSS_CTRL_ETPWM14_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM14_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM14_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM14_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM14_RST_RESETVAL                          (0x00000000U)

/* ETPWM15_RST */

#define CSL_CONTROLSS_CTRL_ETPWM15_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM15_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM15_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM15_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM15_RST_RESETVAL                          (0x00000000U)

/* ETPWM16_RST */

#define CSL_CONTROLSS_CTRL_ETPWM16_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM16_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM16_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM16_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM16_RST_RESETVAL                          (0x00000000U)

/* ETPWM17_RST */

#define CSL_CONTROLSS_CTRL_ETPWM17_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM17_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM17_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM17_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM17_RST_RESETVAL                          (0x00000000U)

/* ETPWM18_RST */

#define CSL_CONTROLSS_CTRL_ETPWM18_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM18_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM18_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM18_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM18_RST_RESETVAL                          (0x00000000U)

/* ETPWM19_RST */

#define CSL_CONTROLSS_CTRL_ETPWM19_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM19_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM19_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM19_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM19_RST_RESETVAL                          (0x00000000U)

/* ETPWM20_RST */

#define CSL_CONTROLSS_CTRL_ETPWM20_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM20_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM20_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM20_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM20_RST_RESETVAL                          (0x00000000U)

/* ETPWM21_RST */

#define CSL_CONTROLSS_CTRL_ETPWM21_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM21_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM21_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM21_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM21_RST_RESETVAL                          (0x00000000U)

/* ETPWM22_RST */

#define CSL_CONTROLSS_CTRL_ETPWM22_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM22_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM22_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM22_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM22_RST_RESETVAL                          (0x00000000U)

/* ETPWM23_RST */

#define CSL_CONTROLSS_CTRL_ETPWM23_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM23_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM23_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM23_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM23_RST_RESETVAL                          (0x00000000U)

/* ETPWM24_RST */

#define CSL_CONTROLSS_CTRL_ETPWM24_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM24_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM24_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM24_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM24_RST_RESETVAL                          (0x00000000U)

/* ETPWM25_RST */

#define CSL_CONTROLSS_CTRL_ETPWM25_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM25_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM25_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM25_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM25_RST_RESETVAL                          (0x00000000U)

/* ETPWM26_RST */

#define CSL_CONTROLSS_CTRL_ETPWM26_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM26_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM26_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM26_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM26_RST_RESETVAL                          (0x00000000U)

/* ETPWM27_RST */

#define CSL_CONTROLSS_CTRL_ETPWM27_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM27_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM27_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM27_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM27_RST_RESETVAL                          (0x00000000U)

/* ETPWM28_RST */

#define CSL_CONTROLSS_CTRL_ETPWM28_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM28_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM28_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM28_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM28_RST_RESETVAL                          (0x00000000U)

/* ETPWM29_RST */

#define CSL_CONTROLSS_CTRL_ETPWM29_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM29_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM29_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM29_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM29_RST_RESETVAL                          (0x00000000U)

/* ETPWM30_RST */

#define CSL_CONTROLSS_CTRL_ETPWM30_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM30_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM30_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM30_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM30_RST_RESETVAL                          (0x00000000U)

/* ETPWM31_RST */

#define CSL_CONTROLSS_CTRL_ETPWM31_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ETPWM31_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM31_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ETPWM31_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ETPWM31_RST_RESETVAL                          (0x00000000U)

/* FSI_TX0_RST */

#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX0_RST_RESETVAL                          (0x00000000U)

/* FSI_TX1_RST */

#define CSL_CONTROLSS_CTRL_FSI_TX1_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX1_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX1_RST_RESETVAL                          (0x00000000U)

/* FSI_TX2_RST */

#define CSL_CONTROLSS_CTRL_FSI_TX2_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX2_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX2_RST_RESETVAL                          (0x00000000U)

/* FSI_TX3_RST */

#define CSL_CONTROLSS_CTRL_FSI_TX3_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_TX3_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_TX3_RST_RESETVAL                          (0x00000000U)

/* FSI_RX0_RST */

#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_RX0_RST_RESETVAL                          (0x00000000U)

/* FSI_RX1_RST */

#define CSL_CONTROLSS_CTRL_FSI_RX1_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX1_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX1_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX1_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_RX1_RST_RESETVAL                          (0x00000000U)

/* FSI_RX2_RST */

#define CSL_CONTROLSS_CTRL_FSI_RX2_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX2_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX2_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX2_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_RX2_RST_RESETVAL                          (0x00000000U)

/* FSI_RX3_RST */

#define CSL_CONTROLSS_CTRL_FSI_RX3_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_FSI_RX3_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX3_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FSI_RX3_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_FSI_RX3_RST_RESETVAL                          (0x00000000U)

/* CMPSSA0_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA0_RST_RESETVAL                          (0x00000000U)

/* CMPSSA1_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA1_RST_RESETVAL                          (0x00000000U)

/* CMPSSA2_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA2_RST_RESETVAL                          (0x00000000U)

/* CMPSSA3_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA3_RST_RESETVAL                          (0x00000000U)

/* CMPSSA4_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA4_RST_RESETVAL                          (0x00000000U)

/* CMPSSA5_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA5_RST_RESETVAL                          (0x00000000U)

/* CMPSSA6_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA6_RST_RESETVAL                          (0x00000000U)

/* CMPSSA7_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA7_RST_RESETVAL                          (0x00000000U)

/* CMPSSA8_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RST_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RST_MAX                           (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA8_RST_RESETVAL                          (0x00000000U)

/* CMPSSA9_RST */

#define CSL_CONTROLSS_CTRL_CMPSSA9_RST_RST_MASK                          (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_RST_RST_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_RST_RST_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_RST_RST_MAX                 (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSA9_RST_RESETVAL                              (0x00000000U)

/* CMPSSB0_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB0_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB0_RST_RESETVAL                               (0x00000000U)

/* CMPSSB1_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB1_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB1_RST_RESETVAL                               (0x00000000U)

/* CMPSSB2_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB2_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB2_RST_RESETVAL                               (0x00000000U)

/* CMPSSB3_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB3_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB3_RST_RESETVAL                               (0x00000000U)

/* CMPSSB4_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB4_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB4_RST_RESETVAL                               (0x00000000U)

/* CMPSSB5_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB5_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB5_RST_RESETVAL                               (0x00000000U)

/* CMPSSB6_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB6_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB6_RST_RESETVAL                               (0x00000000U)

/* CMPSSB7_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB7_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB7_RST_RESETVAL                               (0x00000000U)

/* CMPSSB8_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB8_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB8_RST_RESETVAL                               (0x00000000U)

/* CMPSSB9_RST */

#define CSL_CONTROLSS_CTRL_CMPSSB9_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_CMPSSB9_RST_RESETVAL                               (0x00000000U)

/* ECAP0_RST */

#define CSL_CONTROLSS_CTRL_ECAP0_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP0_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP0_RST_RESETVAL                            (0x00000000U)

/* ECAP1_RST */

#define CSL_CONTROLSS_CTRL_ECAP1_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP1_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP1_RST_RESETVAL                            (0x00000000U)

/* ECAP2_RST */

#define CSL_CONTROLSS_CTRL_ECAP2_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP2_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP2_RST_RESETVAL                            (0x00000000U)

/* ECAP3_RST */

#define CSL_CONTROLSS_CTRL_ECAP3_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP3_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP3_RST_RESETVAL                            (0x00000000U)

/* ECAP4_RST */

#define CSL_CONTROLSS_CTRL_ECAP4_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP4_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP4_RST_RESETVAL                            (0x00000000U)

/* ECAP5_RST */

#define CSL_CONTROLSS_CTRL_ECAP5_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP5_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP5_RST_RESETVAL                            (0x00000000U)

/* ECAP6_RST */

#define CSL_CONTROLSS_CTRL_ECAP6_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP6_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP6_RST_RESETVAL                            (0x00000000U)

/* ECAP7_RST */

#define CSL_CONTROLSS_CTRL_ECAP7_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP7_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP7_RST_RESETVAL                            (0x00000000U)

/* ECAP8_RST */

#define CSL_CONTROLSS_CTRL_ECAP8_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP8_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP8_RST_RESETVAL                            (0x00000000U)

/* ECAP9_RST */

#define CSL_CONTROLSS_CTRL_ECAP9_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP9_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP9_RST_RESETVAL                                  (0x00000000U)

/* ECAP10_RST */

#define CSL_CONTROLSS_CTRL_ECAP10_RST_RST_MASK                      (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP10_RST_RST_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_RST_RST_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_RST_RST_MAX                       (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP10_RST_RESETVAL                                 (0x00000000U)

/* ECAP11_RST */

#define CSL_CONTROLSS_CTRL_ECAP11_RST_RST_MASK                      (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP11_RST_RST_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_RST_RST_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_RST_RST_MAX                       (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP11_RST_RESETVAL                                 (0x00000000U)

/* ECAP12_RST */

#define CSL_CONTROLSS_CTRL_ECAP12_RST_RST_MASK                      (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP12_RST_RST_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_RST_RST_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_RST_RST_MAX                       (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP12_RST_RESETVAL                                 (0x00000000U)

/* ECAP13_RST */

#define CSL_CONTROLSS_CTRL_ECAP13_RST_RST_MASK                      (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP13_RST_RST_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_RST_RST_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_RST_RST_MAX                       (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP13_RST_RESETVAL                                 (0x00000000U)

/* ECAP14_RST */

#define CSL_CONTROLSS_CTRL_ECAP14_RST_RST_MASK                      (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP14_RST_RST_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_RST_RST_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_RST_RST_MAX                       (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP14_RST_RESETVAL                                 (0x00000000U)

/* ECAP15_RST */

#define CSL_CONTROLSS_CTRL_ECAP15_RST_RST_MASK                      (0x00000007U)
#define CSL_CONTROLSS_CTRL_ECAP15_RST_RST_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_RST_RST_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_RST_RST_MAX                       (0x00000007U)

#define CSL_CONTROLSS_CTRL_ECAP15_RST_RESETVAL                                 (0x00000000U)

/* EQEP0_RST */

#define CSL_CONTROLSS_CTRL_EQEP0_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP0_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_EQEP0_RST_RESETVAL                            (0x00000000U)

/* EQEP1_RST */

#define CSL_CONTROLSS_CTRL_EQEP1_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP1_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_EQEP1_RST_RESETVAL                            (0x00000000U)

/* EQEP2_RST */

#define CSL_CONTROLSS_CTRL_EQEP2_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_EQEP2_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_EQEP2_RST_RESETVAL                            (0x00000000U)

/* SDFM0_RST */

#define CSL_CONTROLSS_CTRL_SDFM0_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM0_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM0_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_SDFM0_RST_RESETVAL                            (0x00000000U)

/* SDFM1_RST */

#define CSL_CONTROLSS_CTRL_SDFM1_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_SDFM1_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_SDFM1_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_SDFM1_RST_RESETVAL                            (0x00000000U)

/* DAC_RST */

#define CSL_CONTROLSS_CTRL_DAC_RST_RST_MASK                              (0x00000007U)
#define CSL_CONTROLSS_CTRL_DAC_RST_RST_SHIFT                             (0x00000000U)
#define CSL_CONTROLSS_CTRL_DAC_RST_RST_RESETVAL                          (0x00000000U)
#define CSL_CONTROLSS_CTRL_DAC_RST_RST_MAX                               (0x00000007U)

#define CSL_CONTROLSS_CTRL_DAC_RST_RESETVAL                              (0x00000000U)

/* ADC0_RST */

#define CSL_CONTROLSS_CTRL_ADC0_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC0_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC0_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC0_RST_RST_MAX                              (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC0_RST_RESETVAL                             (0x00000000U)

/* ADC1_RST */

#define CSL_CONTROLSS_CTRL_ADC1_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC1_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC1_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC1_RST_RST_MAX                              (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC1_RST_RESETVAL                             (0x00000000U)

/* ADC2_RST */

#define CSL_CONTROLSS_CTRL_ADC2_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC2_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC2_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC2_RST_RST_MAX                              (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC2_RST_RESETVAL                             (0x00000000U)

/* ADC3_RST */

#define CSL_CONTROLSS_CTRL_ADC3_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC3_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC3_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC3_RST_RST_MAX                              (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC3_RST_RESETVAL                             (0x00000000U)

/* ADC4_RST */

#define CSL_CONTROLSS_CTRL_ADC4_RST_RST_MASK                             (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC4_RST_RST_SHIFT                            (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC4_RST_RST_RESETVAL                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC4_RST_RST_MAX                              (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC4_RST_RESETVAL                             (0x00000000U)

/* OTTO0_RST */

#define CSL_CONTROLSS_CTRL_OTTO0_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO0_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO0_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO0_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_OTTO0_RST_RESETVAL                            (0x00000000U)

/* OTTO1_RST */

#define CSL_CONTROLSS_CTRL_OTTO1_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO1_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO1_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO1_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_OTTO1_RST_RESETVAL                            (0x00000000U)

/* OTTO2_RST */

#define CSL_CONTROLSS_CTRL_OTTO2_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO2_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO2_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO2_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_OTTO2_RST_RESETVAL                            (0x00000000U)

/* OTTO3_RST */

#define CSL_CONTROLSS_CTRL_OTTO3_RST_RST_MASK                            (0x00000007U)
#define CSL_CONTROLSS_CTRL_OTTO3_RST_RST_SHIFT                           (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO3_RST_RST_RESETVAL                        (0x00000000U)
#define CSL_CONTROLSS_CTRL_OTTO3_RST_RST_MAX                             (0x00000007U)

#define CSL_CONTROLSS_CTRL_OTTO3_RST_RESETVAL                            (0x00000000U)

/* HW_RESOLVER_RST */

#define CSL_CONTROLSS_CTRL_HW_RESOLVER_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_HW_RESOLVER_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_HW_RESOLVER_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_HW_RESOLVER_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_HW_RESOLVER_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE0_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE0_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE1_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE1_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE2_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE2_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE3_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE3_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE4_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE4_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE5_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE5_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE6_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE6_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE7_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE7_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE8_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_RST_ADC_SCTILE8_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_RST_ADC_SCTILE8_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_RST_ADC_SCTILE8_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_RST_ADC_SCTILE8_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE8_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE9_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_RST_ADC_SCTILE9_RST_RST_MASK            (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_RST_ADC_SCTILE9_RST_RST_SHIFT           (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_RST_ADC_SCTILE9_RST_RST_RESETVAL        (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_RST_ADC_SCTILE9_RST_RST_MAX             (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE9_RST_RESETVAL                            (0x00000000U)

/* ADC_SCTILE10_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_RST_ADC_SCTILE10_RST_RST_MASK          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_RST_ADC_SCTILE10_RST_RST_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_RST_ADC_SCTILE10_RST_RST_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_RST_ADC_SCTILE10_RST_RST_MAX           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE10_RST_RESETVAL                           (0x00000000U)

/* ADC_SCTILE11_RST */

#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_RST_ADC_SCTILE11_RST_RST_MASK          (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_RST_ADC_SCTILE11_RST_RST_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_RST_ADC_SCTILE11_RST_RST_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_RST_ADC_SCTILE11_RST_RST_MAX           (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_SCTILE11_RST_RESETVAL                           (0x00000000U)

/* ADC_AGG0_RST */

#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_ADC_AGG0_RST_RST_MASK                  (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_ADC_AGG0_RST_RST_SHIFT                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_ADC_AGG0_RST_RST_RESETVAL              (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_ADC_AGG0_RST_RST_MAX                   (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADC_AGG0_RST_RESETVAL                               (0x00000000U)

/* ADCR0_RST */

#define CSL_CONTROLSS_CTRL_ADCR0_RST_ADCR0_RST_RST_MASK                        (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADCR0_RST_ADCR0_RST_RST_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCR0_RST_ADCR0_RST_RST_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCR0_RST_ADCR0_RST_RST_MAX                         (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADCR0_RST_RESETVAL                                  (0x00000000U)

/* ADCR1_RST */

#define CSL_CONTROLSS_CTRL_ADCR1_RST_ADCR1_RST_RST_MASK                        (0x00000007U)
#define CSL_CONTROLSS_CTRL_ADCR1_RST_ADCR1_RST_RST_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCR1_RST_ADCR1_RST_RST_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_ADCR1_RST_ADCR1_RST_RST_MAX                         (0x00000007U)

#define CSL_CONTROLSS_CTRL_ADCR1_RST_RESETVAL                                  (0x00000000U)

/* EPWM0_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM0_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM1_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM1_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM2_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM2_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM3_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM3_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM4_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM4_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM5_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM5_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM6_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM6_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM7_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM7_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM8_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM8_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM9_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM9_HALTEN_RESETVAL                         (0x00000000U)

/* EPWM10_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM10_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM11_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM11_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM12_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM12_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM13_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM13_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM14_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM14_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM15_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM15_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM16_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM16_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM17_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM17_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM18_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM18_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM19_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM19_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM20_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM20_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM21_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM21_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM22_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM22_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM23_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM23_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM24_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM24_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM25_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM25_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM26_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM26_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM27_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM27_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM28_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM28_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM29_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM29_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM30_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM30_HALTEN_RESETVAL                        (0x00000000U)

/* EPWM31_HALTEN */

#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5A0_MASK                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5A0_SHIFT                     (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5A0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5A0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5B0_MASK                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5B0_SHIFT                     (0x00000001U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5B0_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5B0_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5A1_MASK                      (0x00000004U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5A1_SHIFT                     (0x00000002U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5A1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5A1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5B1_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5B1_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5B1_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_CR5B1_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_EPWM31_HALTEN_RESETVAL                              (0x00000000U)

/* CMPSSA0_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA0_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA1_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA1_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA2_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA2_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA3_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA3_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA4_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA4_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA5_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA5_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA6_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA6_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA7_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA7_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA8_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA8_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSA9_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5A0_MASK        (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5A0_SHIFT       (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5A0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5A0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5B0_MASK        (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5B0_SHIFT       (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5B0_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5B0_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5A1_MASK        (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5A1_SHIFT       (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5A1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5A1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5B1_MASK        (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5B1_SHIFT       (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5B1_RESETVAL    (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_CR5B1_MAX         (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSA9_HALTEN_RESETVAL                           (0x00000000U)

/* CMPSSB0_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB0_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB1_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB1_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB2_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB2_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB3_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB3_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB4_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB4_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB5_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB5_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB6_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB6_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB7_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB7_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB8_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB8_HALTEN_RESETVAL                            (0x00000000U)

/* CMPSSB9_HALTEN */

#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5A0_MASK          (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5A0_SHIFT         (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5A0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5A0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5B0_MASK          (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5B0_SHIFT         (0x00000001U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5B0_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5B0_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5A1_MASK          (0x00000004U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5A1_SHIFT         (0x00000002U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5A1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5A1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5B1_MASK          (0x00000008U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5B1_SHIFT         (0x00000003U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5B1_RESETVAL      (0x00000000U)
#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_CR5B1_MAX           (0x00000001U)

#define CSL_CONTROLSS_CTRL_CMPSSB9_HALTEN_RESETVAL                            (0x00000000U)

/* ECAP0_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP0_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP1_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP1_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP2_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP2_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP3_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP3_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP4_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP4_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP5_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP5_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP6_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP6_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP7_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP7_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP8_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP8_HALTEN_RESETVAL                         (0x00000000U)

/* ECAP9_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP9_HALTEN_RESETVAL                               (0x00000000U)

/* EQEP0_HALTEN */

#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP0_HALTEN_RESETVAL                         (0x00000000U)

/* EQEP1_HALTEN */

#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP1_HALTEN_RESETVAL                         (0x00000000U)

/* EQEP2_HALTEN */

#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5A0_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5A0_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5A0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5A0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5B0_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5B0_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5B0_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5B0_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5A1_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5A1_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5A1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5A1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5B1_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5B1_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5B1_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_CR5B1_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_EQEP2_HALTEN_RESETVAL                               (0x00000000U)

/* ECAP10_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5A0_MASK              (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5A0_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5A0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5A0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5B0_MASK              (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5B0_SHIFT             (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5B0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5B0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5A1_MASK              (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5A1_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5A1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5A1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5B1_MASK              (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5B1_SHIFT             (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5B1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_CR5B1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP10_HALTEN_RESETVAL                              (0x00000000U)

/* ECAP11_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5A0_MASK              (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5A0_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5A0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5A0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5B0_MASK              (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5B0_SHIFT             (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5B0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5B0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5A1_MASK              (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5A1_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5A1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5A1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5B1_MASK              (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5B1_SHIFT             (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5B1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_CR5B1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP11_HALTEN_RESETVAL                              (0x00000000U)

/* ECAP12_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5A0_MASK              (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5A0_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5A0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5A0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5B0_MASK              (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5B0_SHIFT             (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5B0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5B0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5A1_MASK              (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5A1_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5A1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5A1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5B1_MASK              (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5B1_SHIFT             (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5B1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_CR5B1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP12_HALTEN_RESETVAL                              (0x00000000U)

/* ECAP13_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5A0_MASK              (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5A0_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5A0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5A0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5B0_MASK              (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5B0_SHIFT             (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5B0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5B0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5A1_MASK              (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5A1_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5A1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5A1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5B1_MASK              (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5B1_SHIFT             (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5B1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_CR5B1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP13_HALTEN_RESETVAL                              (0x00000000U)

/* ECAP14_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5A0_MASK              (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5A0_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5A0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5A0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5B0_MASK              (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5B0_SHIFT             (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5B0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5B0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5A1_MASK              (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5A1_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5A1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5A1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5B1_MASK              (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5B1_SHIFT             (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5B1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_CR5B1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP14_HALTEN_RESETVAL                              (0x00000000U)

/* ECAP15_HALTEN */

#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5A0_MASK              (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5A0_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5A0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5A0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5B0_MASK              (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5B0_SHIFT             (0x00000001U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5B0_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5B0_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5A1_MASK              (0x00000004U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5A1_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5A1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5A1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5B1_MASK              (0x00000008U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5B1_SHIFT             (0x00000003U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5B1_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_CR5B1_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_ECAP15_HALTEN_RESETVAL                              (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MASK                        (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_LOCK0_KICK0_MAX                         (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_LOCK0_KICK0_RESETVAL                                (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_MASK                        (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_MAX                         (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_LOCK0_KICK1_RESETVAL                                (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MASK                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROT_ERR_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROT_ERR_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROT_ERR_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MASK                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_SHIFT                      (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_ADDR_ERR_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MASK                       (0x00000004U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_KICK_ERR_SHIFT                      (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_KICK_ERR_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_KICK_ERR_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MASK                      (0x00000008U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_SHIFT                     (0x00000003U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_PROXY_ERR_MAX                       (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_RAW_STATUS_RESETVAL                            (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK     (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT    (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX      (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK     (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT    (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX      (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK     (0x00000004U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT    (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX      (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK    (0x00000008U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT   (0x00000003U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX     (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLED_STATUS_CLEAR_RESETVAL                  (0x00000000U)

/* INTR_ENABLE */

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MASK                        (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROT_ERR_EN_SHIFT                       (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROT_ERR_EN_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROT_ERR_EN_MAX                         (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MASK                        (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_SHIFT                       (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_ADDR_ERR_EN_MAX                         (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MASK                        (0x00000004U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_KICK_ERR_EN_SHIFT                       (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_KICK_ERR_EN_RESETVAL                    (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_KICK_ERR_EN_MAX                         (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MASK                       (0x00000008U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_SHIFT                      (0x00000003U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_PROXY_ERR_EN_MAX                        (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_RESETVAL                                (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK              (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT             (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK              (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT             (0x00000001U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK              (0x00000004U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT             (0x00000002U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL          (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX               (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK             (0x00000008U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT            (0x00000003U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL         (0x00000000U)
#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX              (0x00000001U)

#define CSL_CONTROLSS_CTRL_INTR_ENABLE_CLEAR_RESETVAL                          (0x00000000U)

/* EOI */

#define CSL_CONTROLSS_CTRL_EOI_EOI_VECTOR_MASK                                 (0x000000FFU)
#define CSL_CONTROLSS_CTRL_EOI_EOI_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CONTROLSS_CTRL_EOI_EOI_VECTOR_RESETVAL                             (0x00000000U)
#define CSL_CONTROLSS_CTRL_EOI_EOI_VECTOR_MAX                                  (0x000000FFU)

#define CSL_CONTROLSS_CTRL_EOI_RESETVAL                                        (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MASK                       (0xFFFFFFFFU)
#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_SHIFT                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                   (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_FAULT_ADDR_MAX                        (0xFFFFFFFFU)

#define CSL_CONTROLSS_CTRL_FAULT_ADDRESS_RESETVAL                              (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                   (0x0000003FU)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                  (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL               (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                    (0x0000003FU)

#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MASK                     (0x00000040U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                    (0x00000006U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                 (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_FAULT_NS_MAX                      (0x00000001U)

#define CSL_CONTROLSS_CTRL_FAULT_TYPE_STATUS_RESETVAL                          (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                 (0x000000FFU)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL             (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                  (0x000000FFU)

#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                (0x000FFF00U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT               (0x00000008U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL            (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                 (0x00000FFFU)

#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MASK                    (0xFFF00000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                   (0x00000014U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_FAULT_XID_MAX                     (0x00000FFFU)

#define CSL_CONTROLSS_CTRL_FAULT_ATTR_STATUS_RESETVAL                          (0x00000000U)

/* FAULT_CLEAR */

#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_FAULT_CLR_MASK                          (0x00000001U)
#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_FAULT_CLR_SHIFT                         (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_FAULT_CLR_RESETVAL                      (0x00000000U)
#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_FAULT_CLR_MAX                           (0x00000001U)

#define CSL_CONTROLSS_CTRL_FAULT_CLEAR_RESETVAL                                (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
